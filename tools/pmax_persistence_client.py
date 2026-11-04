"""Offline-first, single-use host session for the fixed ID7 PMAX512/POS2 flow.

There is no serial, probe, Flash, CLI, reset or hardware import here. The caller
injects ``exchange(request_id, command)``; command excludes the decimal request
prefix (for example ``adrc persist_begin motor=7``). A future live wrapper must
pin frozen/installed artifacts, current physical conditions and one trusted once
directory. This library never discovers those facts or writes evidence files.

Wire reports are checked for consistency; they are not independent physical bus
measurements. Cold confirmation is a caller/user statement, not measured 24V.
The retained host token provides one application Save claim, not bus exactly-once
delivery, cold EEPROM proof, or an absolute power-loss guarantee. On any failure
the caller can persist ``transcript`` and partial ``evidence`` in its finally block.
No retry, token deletion, automatic release, disable, motion or recovery occurs.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import time
from typing import Any

from pmax_persistence_evidence import compare_configuration, validate_observation_records, validate_snapshot
from pmax_persistence_once import SaveOnceBinding, claim_save_once

MAX_EXCHANGES = 2048
TOTAL_BUDGET_SECONDS = 42.0
PING_INTERVAL_SECONDS = 0.250
CONTEXT_FIELDS = ("id", "boot", "owner", "link", "epoch", "session")
OWNER_FIELDS = ("boot_epoch", "request_id", "owner_generation")
IDENTITY_FIELDS = (*OWNER_FIELDS, "phase", "sequence")
TIME_FIELDS = ("created_us", "submitted_us", "boundary_us", "send_completed_observed_us", "rx_lower_us", "received_us")
_STATUS_TYPES = {key: "u32" for key in (*CONTEXT_FIELDS, "state", "error", "fault", "snapshot_phase", "pmax_raw", "mode")}
_STATUS_TYPES.update({key: "bool" for key in ("locked", "ready", "skip_save", "save_used", "save_uncertain", "pmax_known", "mode_known", "released")})
_STATUS_TYPES.update(now_us="u64", verified_us="u64")
_HEALTH_TYPES = {key: "u32" for key in (*CONTEXT_FIELDS, "fault")}
_HEALTH_TYPES.update({key: "bool" for key in ("valid", "disabled", "idle", "health_proven", "continuity_valid")})
_HEALTH_TYPES.update({key: "float" for key in ("velocity", "torque", "motor_c", "driver_c", "reference_native", "native_position", "tolerance_native")})
_HEALTH_TYPES.update(now_us="u64", observed_us="u64")
_ITEM_TYPES = {key: "u32" for key in ("id", "index", "rid", "raw", *IDENTITY_FIELDS)}
_ITEM_TYPES.update({key: "u64" for key in TIME_FIELDS})
_ITEM_TYPES.update(valid="bool", kind="kind")
_JOURNAL_TYPES = {key: "u32" for key in ("id", *IDENTITY_FIELDS, "tx_id", "tx_dlc", "tx_raw", "ack_id", "ack_dlc", "ack_raw")}
_JOURNAL_TYPES.update({key: "u64" for key in (*TIME_FIELDS, "settle_until_us")})
_JOURNAL_TYPES.update(valid="bool")


def _unsigned(value: Any, bits: int, *, nonzero: bool = False) -> int:
    """Require an exact Python unsigned integer, never bool or coercion."""
    if type(value) is not int or not int(nonzero) <= value < 1 << bits:
        raise ValueError(f"expected {'nonzero ' if nonzero else ''}uint{bits}")
    return value


def _float32(value: float) -> float:
    """Recover a C float exactly from its nine-significant-digit round-trip export."""
    try:
        rounded = struct.unpack("<f", struct.pack("<f", value))[0]
    except (OverflowError, struct.error) as error:
        raise ValueError("value cannot represent a finite native float32") from error
    if not math.isfinite(rounded):
        raise ValueError("value cannot represent a finite native float32")
    return rounded


def _decode_word(raw: int) -> float:
    """Decode one already width-validated raw native float without extra gear conversion."""
    return struct.unpack("<f", struct.pack("<I", raw))[0]


def _continuity_tolerance(source_pmax: float, current_pmax: float) -> float:
    """Independently evaluate the frozen B1 formula with C float operation rounding.

    Rounding exported 9g values back to float32 uniquely recovers the source float;
    no broad decimal epsilon or added physical position allowance is introduced.
    All native positions are rotor radians; no external 44.8 conversion applies.
    """
    source_quantum = _float32(_float32(2.0 * source_pmax) / 65535.0)
    current_quantum = _float32(_float32(2.0 * current_pmax) / 65535.0)
    arithmetic_bound = _float32(_float32(4.0 * 2.0 ** -23) * max(source_pmax, current_pmax))
    display_bound = _float32(_float32(0.001) * _float32(0.01745329251994329577))
    return _float32(_float32(_float32(source_quantum + current_quantum) + arithmetic_bound) + display_bound)


def _poll_wait() -> None:
    """Perform only the default short ten-millisecond host poll delay."""
    time.sleep(0.010)


@dataclass(frozen=True, slots=True)
class ClientInputs:
    """Bind immutable declared frozen artifacts, controller identity and user cold confirmation."""

    firmware_hex_sha256: str
    source_manifest_sha256: str
    controller_usb_serial: str
    expected_boot: int
    physical_cold_confirmed: bool

    def __post_init__(self) -> None:
        """Reject invalid or unconfirmed declarations before any injected exchange."""
        _validate_inputs(self)


def _validate_inputs(inputs: ClientInputs) -> None:
    """Recheck exact immutable input types without making hardware or deployment claims."""
    if type(inputs) is not ClientInputs:
        raise ValueError("inputs must be an exact ClientInputs")
    for field in ("firmware_hex_sha256", "source_manifest_sha256"):
        digest = getattr(inputs, field)
        if type(digest) is not str or re.fullmatch(r"[0-9a-f]{64}", digest) is None or digest == "0" * 64:
            raise ValueError(f"{field} must be a nonzero lowercase SHA-256")
    if type(inputs.controller_usb_serial) is not str or inputs.controller_usb_serial != "386B336F3134":
        raise ValueError("expected controller USB serial 386B336F3134")
    _unsigned(inputs.expected_boot, 32, nonzero=True)
    if inputs.physical_cold_confirmed is not True:
        raise ValueError("physical_cold_confirmed must be the exact bool True")


def _parse_reply(line: str, request_id: int, subcommand: str, fields: Mapping[str, str]) -> dict[str, Any]:
    """Strictly parse one bounded ASCII CRLF response with exact request, command and schema.

    Nonconflicting extension fields are retained as strings but cannot substitute
    for mandatory typed evidence. All duplicate keys are refused, including extras.
    """
    if type(line) is not str:
        raise ValueError("missing or non-string response")
    try:
        encoded = line.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("response must be ASCII") from error
    if len(encoded) >= 520 or not line.endswith("\r\n") or "\r" in line[:-2] or "\n" in line[:-2]:
        raise ValueError("response must be one complete CRLF line shorter than 520 bytes")
    tokens = line[:-2].split(" ")
    if len(tokens) < 5 or any(not token or any(ord(character) < 33 or ord(character) > 126 for character in token) for token in tokens):
        raise ValueError("invalid response token grammar")
    if tokens[0] not in ("ok", "error") or tokens[1] != str(request_id) or tokens[2:4] != ["adrc", subcommand]:
        raise ValueError("response request or command mismatch")
    values: dict[str, Any] = {}
    for token in tokens[4:]:
        if token.count("=") != 1:
            raise ValueError("response fields must be key=value")
        key, value = token.split("=")
        if re.fullmatch(r"[a-z][a-z0-9_]*", key) is None or not value or key in values:
            raise ValueError("invalid, empty or duplicate response field")
        values[key] = value
    if values.get("schema") != "1":
        raise ValueError("unsupported or missing response schema")
    if tokens[0] == "error":
        code = values.get("code", "")
        if re.fullmatch(r"[A-Za-z0-9_]+", code) is None:
            raise ValueError("invalid error code")
        raise ValueError(f"{subcommand} rejected: {code}")
    for key, kind in {"schema": "u32", **fields}.items():
        if key not in values:
            raise ValueError(f"missing required field {key}")
        raw = values[key]
        if kind in ("u32", "u64", "bool"):
            if re.fullmatch(r"[0-9]+", raw) is None or len(raw) > 20:
                raise ValueError(f"{key} is not unsigned decimal")
            value = int(raw)
            _unsigned(value, 64 if kind == "u64" else 32)
            if kind == "bool" and value not in (0, 1):
                raise ValueError(f"{key} must be 0 or 1")
        elif kind == "float":
            if re.fullmatch(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?", raw) is None:
                raise ValueError(f"{key} is not a finite decimal float")
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError(f"{key} is not finite")
        elif kind == "kind":
            if raw not in ("uint32", "float32"):
                raise ValueError("unknown register kind")
            value = raw
        else:
            raise AssertionError("internal response field type is not defined")
        values[key] = value
    return values


class PersistenceClient:
    """Own exactly one bounded fixed maintenance session with injected, nonretrying transport.

    ``once_directory`` is captured once as an absolute path and never derived from
    a request, boot or image. The future wrapper must always supply the same trusted
    pre-existing task directory. Existing/partial tokens only forbid a new Save;
    they never prevent a successful target-already-present or read-only check.
    """

    def __init__(self, inputs: ClientInputs, once_directory: str | os.PathLike[str],
                 exchange: Callable[[int, str], str], *, clock: Callable[[], float] = time.monotonic,
                 poll_wait: Callable[[], None] = _poll_wait, first_request_id: int = 1) -> None:
        """Capture offline inputs; first_request_id starts commands, actual begin binds the root."""
        _validate_inputs(inputs)
        _unsigned(first_request_id, 32, nonzero=True)
        if not all(callable(callback) for callback in (exchange, clock, poll_wait)):
            raise ValueError("exchange, clock and poll_wait must be callable")
        self.inputs = inputs
        self._once_directory = Path(once_directory).absolute()
        self._exchange, self._clock, self._poll_wait = exchange, clock, poll_wait
        self._next_request_id = first_request_id
        self.transaction_id: int | None = None
        self.transcript: list[dict[str, Any]] = []
        self.evidence: dict[str, Any] = {}
        self.mutation_sent: set[str] = set()
        self._started = False
        self._last_clock: float | None = None
        self._total_deadline: float | None = None
        self._stage_deadline: float | None = None
        self._last_ping = 0.0
        self._context: dict[str, int] | None = None
        self._last_device_now = 0
        self._previous_records: list[dict[str, Any]] | None = None
        self._reference_native: float | None = None
        self._source_pmax: float | None = None
        self._verified_us = 0
        self._health_started_at: float | None = None
        self._reported_health_age_us = 0

    def _time(self) -> float:
        """Enforce finite monotonic host time and immutable total/stage deadlines."""
        current = self._clock()
        if type(current) not in (int, float) or not math.isfinite(current) or current < 0:
            raise ValueError("host monotonic clock is invalid")
        if self._last_clock is not None and current < self._last_clock:
            raise ValueError("host monotonic clock moved backwards")
        self._last_clock = current
        if self._total_deadline is not None and current >= self._total_deadline:
            raise TimeoutError("fixed host total budget expired")
        if self._stage_deadline is not None and current >= self._stage_deadline:
            raise TimeoutError("fixed host stage budget expired")
        return current

    def _stage(self, seconds: float) -> None:
        """Open only the next distinct phase budget without renewing the previous one."""
        current = self._time()
        assert self._total_deadline is not None
        self._stage_deadline = min(self._total_deadline, current + seconds)

    def _request(self, subcommand: str, fields: Mapping[str, str], *, arguments: str = "", mutation: bool = False) -> dict[str, Any]:
        """Send a new-ID ping when due, then emit precisely one requested command."""
        if self._time() - self._last_ping >= PING_INTERVAL_SECONDS:
            self._send("ping", "ping", {}, mutation=False)
        command = f"adrc {subcommand}" + (" " + arguments if arguments else "")
        return self._send(command, subcommand, fields, mutation=mutation)

    def _send(self, command: str, subcommand: str, fields: Mapping[str, str], *, mutation: bool) -> dict[str, Any]:
        """Mark mutations before the sole exchange, retaining original reply or exception."""
        current = self._time()
        if len(self.transcript) >= MAX_EXCHANGES:
            raise ValueError("fixed exchange count exhausted")
        request_id = _unsigned(self._next_request_id, 32, nonzero=True)
        self._next_request_id += 1
        if mutation:
            if subcommand in self.mutation_sent:
                raise ValueError("mutation already sent")
            if subcommand == "persist_begin":
                if self.transaction_id is not None:
                    raise ValueError("begin transaction root is already bound")
                self.transaction_id = request_id
            else:
                if self._health_started_at is None or self._reported_health_age_us + (current - self._health_started_at) * 1000000.0 > 100000:
                    raise ValueError("exported FD health became stale before mutation")
            self.mutation_sent.add(subcommand)
        entry: dict[str, Any] = {"request_id": request_id, "command": command, "reply": None}
        self.transcript.append(entry)
        try:
            line = self._exchange(request_id, command)
            entry["reply"] = line
            current = self._time()
            if subcommand == "ping":
                self._validate_ping(line, request_id)
                self._last_ping = current
                return {}
            return _parse_reply(line, request_id, subcommand, fields)
        except BaseException as error:
            entry["exception"] = f"{type(error).__name__}: {error}"
            raise

    @staticmethod
    def _validate_ping(line: str, request_id: int) -> None:
        """Accept exact legacy ping LF/CRLF while leaving persistence framing strict."""
        if type(line) is not str:
            raise ValueError("missing ping reply")
        try:
            encoded = line.encode("ascii")
        except UnicodeEncodeError as error:
            raise ValueError("ping reply must be ASCII") from error
        if len(encoded) >= 520 or re.fullmatch(r"ok " + str(request_id) + r" ping(?: [!-~]+)*\r?\n", line) is None:
            raise ValueError("ping reply request or framing mismatch")

    def _mutation(self, subcommand: str) -> None:
        """Require an exact accepted acknowledgment without interpreting it as completion."""
        arguments = "motor=7" if subcommand == "persist_begin" else f"id={self.transaction_id}"
        response = self._request(subcommand, {"id": "u32", "accepted": "bool"}, arguments=arguments, mutation=True)
        if response["id"] != self.transaction_id or response["accepted"] != 1 or set(response) != {"schema", "id", "accepted"}:
            raise ValueError("mutation acceptance or transaction mismatch")

    def _context_check(self, response: Mapping[str, Any]) -> None:
        """Pin boot, root request, owner, full CDC word, local epoch and protocol session."""
        context = {field: _unsigned(response[field], 32, nonzero=True) for field in CONTEXT_FIELDS}
        if context["id"] != self.transaction_id or context["boot"] != self.inputs.expected_boot:
            raise ValueError("unexpected boot or root request")
        if not context["link"] & 0x80000000 or not context["link"] & 0x7FFFFFFF:
            raise ValueError("CDC link is not configured with a nonzero generation")
        if self._context is None:
            self._context = context
        elif context != self._context:
            raise ValueError("immutable transaction context changed")
        now_us = _unsigned(response["now_us"], 64, nonzero=True)
        if now_us < self._last_device_now:
            raise ValueError("device observation clock moved backwards")
        self._last_device_now = now_us

    def _status(self, *, released: bool | None = False) -> dict[str, Any]:
        """Check every status, including COMPLETE, for sticky faults and immutable ownership."""
        status = self._request("persist_status", _STATUS_TYPES)
        self.evidence.setdefault("statuses", []).append(status)
        self._context_check(status)
        if status["error"] != 0 or status["fault"] != 0 or status["state"] not in range(1, 12):
            raise ValueError("transaction state/error/fault is unsafe")
        if status["verified_us"] > status["now_us"]:
            raise ValueError("verified_us is in the future")
        if status["snapshot_phase"] not in (0, 1, 4, 6, 10):
            raise ValueError("unknown snapshot phase")
        if self._verified_us and status["verified_us"] != self._verified_us:
            raise ValueError("original Save verification timestamp changed")
        if (status["locked"], status["released"]) not in ((1, 0), (0, 1)):
            raise ValueError("maintenance lock/release mismatch")
        if released is not None and (status["locked"] != int(not released) or status["released"] != int(released)):
            raise ValueError("maintenance lock/release mismatch")
        return status

    @staticmethod
    def _stable(status: Mapping[str, Any], phase: int, *, require_ready: bool = True) -> None:
        """Require a ready complete snapshot and known, phase-correct private mapping."""
        if (require_ready and status["ready"] != 1) or status["snapshot_phase"] != phase or status["pmax_known"] != 1 or status["mode_known"] != 1:
            raise ValueError("stable snapshot or confirmed mapping is missing")
        if status["pmax_raw"] not in (0x41480000, 0x44000000) or status["mode"] not in (1, 2):
            raise ValueError("unsupported confirmed source mapping")
        if phase != 1 and (status["pmax_raw"] != 0x44000000 or status["mode"] != 2):
            raise ValueError("stable target mapping is not PMAX512/POS2")
        if status["save_used"] != int(phase == 10) or status["save_uncertain"] != 0:
            raise ValueError("unexpected Save usage or uncertainty")
        if status["skip_save"] != int(phase == 1 and status["state"] == 11):
            raise ValueError("unexpected skip-save state")
        if status["skip_save"] and (status["pmax_raw"] != 0x44000000 or status["mode"] != 2):
            raise ValueError("skip-save mapping is not exactly PMAX512/POS2")

    def _await_ready(self, phase: int) -> dict[str, Any]:
        """Poll only allowed progressing states under the one existing phase deadline."""
        targets = {1: (2, 11), 4: (5,), 6: (7,), 10: (11,)}[phase]
        transients = {1: (1,), 4: (3, 4), 6: (6,), 10: (8, 9, 10)}[phase]
        while True:
            status = self._status()
            if status["state"] in targets and status["ready"]:
                self._stable(status, phase)
                return status
            if status["state"] not in targets + transients or status["ready"]:
                raise ValueError("unexpected phase progression or false ready")
            self._poll_wait()
            self._time()

    def _owner(self) -> dict[str, int]:
        """Map pinned status ownership into the reviewed raw observation helper contract."""
        assert self._context is not None
        return dict(boot_epoch=self._context["boot"], request_id=self._context["id"], owner_generation=self._context["owner"])

    def _health(self, status: Mapping[str, Any], words: Mapping[int, int], *, snapshot_finished_us: int,
                post_verified: bool) -> dict[str, Any]:
        """Independently check strict FD report health, raw/native position and fixed reference."""
        health_started_at = self._time()
        health = self._request("persist_health", _HEALTH_TYPES, arguments=f"id={self.transaction_id}")
        self.evidence.setdefault("health", []).append(health)
        self._context_check(health)
        if any(health[field] != 1 for field in ("valid", "disabled", "idle", "health_proven", "continuity_valid")) or health["fault"] != 0:
            raise ValueError("strict disabled FD health proof is missing")
        if not 0 < health["observed_us"] <= health["now_us"] or health["now_us"] - health["observed_us"] > 100000:
            raise ValueError("FD health is stale or has invalid time")
        if health["observed_us"] <= snapshot_finished_us:
            raise ValueError("FD observation is not strictly after the completed current snapshot")
        if abs(_float32(health["velocity"])) > _float32(0.05) or abs(_float32(health["torque"])) > _float32(0.15):
            raise ValueError("FD velocity or torque exceeds the fixed bound")
        if any(not 0 <= _float32(health[field]) < 50 for field in ("motor_c", "driver_c")):
            raise ValueError("FD temperature exceeds the fixed bound")
        if post_verified and health["observed_us"] <= status["verified_us"]:
            raise ValueError("FD observation is not strictly post-verified")
        reference = _float32(health["reference_native"])
        if self._reference_native is None:
            self._reference_native = reference
        elif reference != self._reference_native:
            raise ValueError("same-session native reference changed")
        assert self._source_pmax is not None
        tolerance = _continuity_tolerance(self._source_pmax, _decode_word(words[21]))
        if _float32(health["tolerance_native"]) != tolerance:
            raise ValueError("reported continuity tolerance differs from independent B1 formula")
        for measured in (_float32(health["native_position"]), _decode_word(words[80])):
            if abs(_float32(measured - reference)) > tolerance:
                raise ValueError("native or raw p_m continuity exceeds the fixed tolerance")
        # Account conservatively for the complete synchronous exchange and all
        # later host work, including durable token synchronization, before a
        # mutation. Firmware independently checks its current FD again as well.
        self._health_started_at = health_started_at
        self._reported_health_age_us = health["now_us"] - health["observed_us"]
        return health

    def _snapshot(self, name: str, status: dict[str, Any], *, require_target: bool) -> list[dict[str, Any]]:
        """Export exactly one ordered full45 phase, retaining partial records before validation."""
        phase = status["snapshot_phase"]
        records: list[dict[str, Any]] = []
        self.evidence[name] = records
        for index in range(45):
            response = self._request("persist_item", _ITEM_TYPES, arguments=f"id={self.transaction_id} phase={phase} index={index}")
            if response["id"] != self.transaction_id or response["index"] != index:
                raise ValueError("item transaction or index mismatch")
            records.append({field: response[field] for field in ("rid", "kind", "raw", "valid", *TIME_FIELDS)} |
                           {"identity": {field: response[field] for field in IDENTITY_FIELDS}})
        latest = self._status()
        self._stable(latest, phase)
        if latest["state"] != status["state"] or latest["verified_us"] != status["verified_us"]:
            raise ValueError("stable snapshot phase changed while exporting")
        words = validate_observation_records(records, expected_owner=self._owner(), expected_phase=phase,
                                             now_us=latest["now_us"], require_target=require_target)
        if latest["pmax_raw"] != words[21] or latest["mode"] != words[10]:
            raise ValueError("raw configuration and confirmed mapping disagree")
        if phase == 1 and bool(latest["skip_save"]) != (words[21] == 0x44000000 and words[10] == 2):
            raise ValueError("raw target presence contradicts skip-save state")
        if self._previous_records is not None:
            previous = self._previous_records[-1]
            if records[0]["identity"]["sequence"] <= previous["identity"]["sequence"] or records[0]["created_us"] < max(previous["send_completed_observed_us"], previous["received_us"]):
                raise ValueError("successive snapshots overlap or reuse sequences")
        if phase == 1:
            self._source_pmax = _decode_word(words[21])
        else:
            compare_configuration(self.evidence["baseline"], records, allow_targets=True)
        if phase == 6:
            last_finished = max(records[-1]["send_completed_observed_us"], records[-1]["received_us"])
            if not last_finished <= latest["verified_us"] <= latest["now_us"]:
                raise ValueError("Save verification time precedes completed observations")
            self._verified_us = latest["verified_us"]
        elif phase == 10 and latest["verified_us"] != self._verified_us:
            raise ValueError("original Save verification timestamp changed")
        self._health(latest, words, snapshot_finished_us=max(records[-1]["send_completed_observed_us"], records[-1]["received_us"]),
                     post_verified=phase in (6, 10))
        self._previous_records = records
        return records

    def _journal(self) -> None:
        """Require exact Save actual-TX/ACK identity, original time bounds and >=30ms settle."""
        journal = self._request("persist_save_receipt", _JOURNAL_TYPES, arguments=f"id={self.transaction_id}")
        self.evidence["save_journal"] = journal
        if journal["id"] != self.transaction_id or journal["valid"] != 1 or any(journal[field] != value for field, value in self._owner().items()):
            raise ValueError("valid original Save journal ownership is missing")
        if journal["phase"] != 8 or any(journal[key] != value for key, value in dict(tx_id=0x7FF, tx_dlc=4, tx_raw=0x01AA0007, ack_id=23, ack_dlc=4, ack_raw=0x01AA0007).items()):
            raise ValueError("Save journal is not exact AA TX/ACK")
        created, submitted, boundary, completed, lower, received = (journal[field] for field in TIME_FIELDS)
        if not 0 < created <= boundary <= submitted <= completed <= self._last_device_now or not submitted <= lower <= received <= self._last_device_now:
            raise ValueError("Save journal six-time ordering is invalid")
        if created > (1 << 64) - 1 - 100000 or completed >= created + 100000 or received >= created + 100000:
            raise ValueError("Save journal fixed frame budget expired or overflows")
        verified = self.evidence["verified"][-1]
        readback = self.evidence["readback"][0]
        if not verified["identity"]["sequence"] < journal["sequence"] < readback["identity"]["sequence"]:
            raise ValueError("Save sequence is outside verification/readback interval")
        if created < max(self._verified_us, verified["send_completed_observed_us"], verified["received_us"]):
            raise ValueError("Save was created before verification finished")
        if received > (1 << 64) - 1 - 30000 or journal["settle_until_us"] < received + 30000:
            raise ValueError("Save journal does not retain ACK plus 30ms")
        if readback["created_us"] < max(journal["settle_until_us"], completed, received):
            raise ValueError("readback began before Save observations and original settle boundary")

    def _begin(self) -> dict[str, Any]:
        """Start exactly once with declared cold input validation before the first exchange."""
        if self._started:
            raise ValueError("this client object has already consumed its one session")
        _validate_inputs(self.inputs)
        self._started = True
        current = self._time()
        self._total_deadline, self._last_ping = current + TOTAL_BUDGET_SECONDS, current
        self._stage(5.0)
        self._mutation("persist_begin")
        return self._await_ready(1)

    def _release(self, phase: int) -> dict[str, bool]:
        """Request release once and poll bounded real idle/quiet completion without retrying it."""
        self._stage(5.0)
        self._mutation("persist_release")
        while True:
            status = self._status(released=None)
            self._stable(status, phase, require_ready=bool(status["released"]))
            if status["state"] != 11:
                raise ValueError("release did not preserve successful completion")
            if status["released"]:
                self.evidence["released"] = True
                return {"released": True, "skipped_save": bool(status["skip_save"])}
            self._poll_wait()
            self._time()

    def run_prepare_save(self) -> dict[str, bool]:
        """Perform one fixed prepare/verify/claimed commit, or skip when already at target."""
        baseline_status = self._begin()
        self._snapshot("baseline", baseline_status, require_target=bool(baseline_status["skip_save"]))
        if baseline_status["skip_save"]:
            return self._release(1)
        self._stage(6.0)
        self._mutation("persist_prepare")
        self._snapshot("prepared", self._await_ready(4), require_target=True)
        self._stage(6.0)
        self._mutation("persist_verify")
        self._snapshot("verified", self._await_ready(6), require_target=True)
        canonical = json.dumps(self.evidence["baseline"], sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
        owner = self._owner()
        assert self._context is not None
        binding = SaveOnceBinding(self.inputs.firmware_hex_sha256, self.inputs.source_manifest_sha256,
                                  hashlib.sha256(canonical).hexdigest(), self.inputs.controller_usb_serial,
                                  owner["boot_epoch"], owner["request_id"], owner["owner_generation"], self._context["link"])
        self._time()
        token = claim_save_once(self._once_directory, binding)
        self.evidence["once_token"] = str(token)
        self._stage(12.0)
        self._mutation("persist_commit")
        self._snapshot("readback", self._await_ready(10), require_target=True)
        self._journal()
        return self._release(10)

    def run_readonly_verify(self, original_cold_baseline_records: Sequence[Mapping[str, Any]]) -> dict[str, bool]:
        """Only inspect the new cold target and other24 RW; never prepare, commit or claim.

        Original full baseline evidence is validated as historical phase1 evidence.
        Native positions across boots are deliberately not compared: this session
        establishes its own native reference without FE, zeroing or movement.
        """
        if self._started:
            raise ValueError("this client object has already consumed its one session")
        validate_snapshot(original_cold_baseline_records)
        first = original_cold_baseline_records[0]
        identity = first.get("identity")
        if not isinstance(identity, Mapping):
            raise ValueError("original cold baseline requires full observation records")
        original_owner = {field: identity.get(field) for field in OWNER_FIELDS}
        original_now = max(_unsigned(record.get(field), 64, nonzero=True) for record in original_cold_baseline_records
                           for field in ("send_completed_observed_us", "received_us"))
        validate_observation_records(original_cold_baseline_records, expected_owner=original_owner,
                                     expected_phase=1, now_us=original_now)
        status = self._begin()
        current = self._snapshot("baseline", status, require_target=True)
        compare_configuration(original_cold_baseline_records, current, allow_targets=True)
        if status["state"] != 11 or status["skip_save"] != 1:
            raise ValueError("read-only cold verify did not find the complete fixed target")
        return self._release(1)
