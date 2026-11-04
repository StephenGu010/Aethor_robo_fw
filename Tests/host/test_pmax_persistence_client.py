"""Synthetic wire tests for the offline host session library, never C end-to-end.

Every filesystem fixture is retained under the evidence directory. No serial,
probe, Flash, motion, reset, or automatic cleanup is used by this test module.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError
import hashlib
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import uuid

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "tools"))
import pmax_persistence_client as CLIENT
import pmax_persistence_once as ONCE

EVIDENCE_DIRECTORY = REPOSITORY / "output/repair_20261023/persistence_host_client"
REGISTERS = tuple(range(37)) + tuple(range(50, 56)) + (80, 81)
INTEGERS = {7, 8, 9, 10, 13, 14, 15, 16, 35, 36}
TIME_FIELDS = ("created_us", "submitted_us", "boundary_us", "send_completed_observed_us", "rx_lower_us", "received_us")
ACTUAL_C_PING_REPLY = "ok 763 ping state=boot enabled=00 boot=1234\n"


def raw_float(value: float) -> int:
    """Encode explicitly synthetic native IEEE754 data without host decimal formatting."""
    return struct.unpack("<I", struct.pack("<f", value))[0]


def float32(value: float) -> float:
    """Round fixture arithmetic to the actual C float representation."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def tolerance(source: float, current: float) -> float:
    """Independently reproduce the frozen B1 expression and its C float rounding."""
    first = float32(float32(2.0 * source) / 65535.0)
    second = float32(float32(2.0 * current) / 65535.0)
    rounding = float32(float32(4.0 * 2.0 ** -23) * max(source, current))
    display = float32(float32(0.001) * float32(0.01745329251994329577))
    return float32(float32(float32(first + second) + rounding) + display)


def synthetic_records(phase: int, *, source_pmax: float = 12.5, source_mode: int = 1,
                      boot: int = 11, request: int = 100, position: float = 1.0) -> list[dict]:
    """Build independent full45 synthetic observations with real-shaped bounds and identities."""
    starts = {1: (1000000, 1), 4: (1100000, 48), 6: (1200000, 93), 10: (1250000, 139)}
    first_time, first_sequence = starts[phase]
    result = []
    for index, rid in enumerate(REGISTERS):
        created = first_time + index * 100
        record = {"rid": rid, "kind": "uint32" if rid in INTEGERS else "float32",
                  "raw": 1 if rid in INTEGERS else raw_float(1.0), "valid": 1,
                  "identity": {"boot_epoch": boot, "request_id": request, "owner_generation": 13,
                               "phase": phase, "sequence": first_sequence + index},
                  "created_us": created, "submitted_us": created + 2, "boundary_us": created + 1,
                  "send_completed_observed_us": created + 5, "rx_lower_us": created + 2,
                  "received_us": created + 4}
        result.append(record)
    values = {7: 23, 8: 7, 10: source_mode if phase == 1 else 2, 14: 808597046, 36: 49,
              21: raw_float(source_pmax if phase == 1 else 512.0), 22: raw_float(200.0),
              23: raw_float(10.0), 80: raw_float(position)}
    for record in result:
        record["raw"] = values.get(record["rid"], record["raw"])
    return result


def inputs(**changes: object):
    """Construct immutable, synthetic frozen-artifact and physical-confirmation inputs."""
    values = dict(firmware_hex_sha256="1" * 64, source_manifest_sha256="2" * 64,
                  controller_usb_serial="386B336F3134", expected_boot=11, physical_cold_confirmed=True)
    values.update(changes)
    return CLIENT.ClientInputs(**values)


def retained_directory() -> Path:
    """Create one explicit evidence directory without ever deleting fixture artifacts."""
    directory = EVIDENCE_DIRECTORY / ("synthetic_" + uuid.uuid4().hex)
    directory.mkdir(parents=True, exist_ok=False)
    return directory


class FakeClock:
    """Provide fast controllable monotonic time without live waits or hardware."""

    def __init__(self) -> None:
        """Begin at a finite nonzero synthetic host monotonic timestamp."""
        self.value = 10.0

    def __call__(self) -> float:
        """Read the current synthetic monotonic timestamp."""
        return self.value

    def wait(self) -> None:
        """Advance precisely one configured synthetic ten-millisecond poll interval."""
        self.value += 0.01


class FakeExchange:
    """Model only frozen wire responses; this does not execute the C implementation."""

    def __init__(self, *, pmax: float = 12.5, mode: int = 1, boot: int = 11,
                 request: int = 100, position: float = 1.0, clock: FakeClock | None = None,
                 hook=None, line_hook=None) -> None:
        """Prepare full independent records and optional single-point fault injection."""
        self.pmax, self.mode, self.boot, self.request = pmax, mode, boot, request
        self.phase, self.released, self.calls = 1, False, []
        self.clock, self.hook, self.line_hook = clock, hook, line_hook
        self.skip = pmax == 512.0 and mode == 2
        self.records = {phase: synthetic_records(phase, source_pmax=pmax, source_mode=mode,
                                                boot=boot, request=request, position=position)
                        for phase in (1, 4, 6, 10)}
        self.position = position
        self.exchange_seconds = 0.002

    def status(self) -> dict:
        """Return one legal stable transaction state independently of host implementation."""
        state = {1: 11 if self.skip else 2, 4: 5, 6: 7, 10: 11}[self.phase]
        now = self.records[self.phase][-1]["created_us"] + 1000
        return dict(schema=1, id=self.request, boot=self.boot, owner=13, link=2147483663,
                    epoch=17, session=19, state=state, error=0, fault=0, locked=int(not self.released),
                    ready=1, snapshot_phase=self.phase, skip_save=int(self.skip),
                    save_used=int(self.phase == 10), save_uncertain=0, pmax_known=1, mode_known=1,
                    pmax_raw=raw_float(self.pmax if self.phase == 1 else 512.0),
                    mode=self.mode if self.phase == 1 else 2, now_us=now,
                    verified_us=1204405 if self.phase in (6, 10) else 0, released=int(self.released))

    def health(self) -> dict:
        """Report healthy synthetic disabled native feedback, including exact B1 tolerance."""
        status = self.status()
        return {key: status[key] for key in ("schema", "id", "boot", "owner", "link", "epoch", "session")} | dict(
            now_us=status["now_us"], observed_us=status["now_us"] - 100,
            valid=1, disabled=1, idle=1, fault=0, health_proven=1, velocity=0.0, torque=0.0,
            motor_c=25.0, driver_c=26.0, reference_native=self.position, native_position=self.position,
            tolerance_native=tolerance(self.pmax, self.pmax if self.phase == 1 else 512.0), continuity_valid=1)

    def journal(self) -> dict:
        """Preserve a synthetic exact AA actual-TX/ACK journal with the 30ms settle boundary."""
        return dict(schema=1, id=self.request, valid=1, boot_epoch=self.boot, request_id=self.request,
                    owner_generation=13, phase=8, sequence=138, created_us=1210000,
                    submitted_us=1210002, boundary_us=1210001, send_completed_observed_us=1210005,
                    rx_lower_us=1210002, received_us=1210004, tx_id=2047, tx_dlc=4,
                    tx_raw=27918343, ack_id=23, ack_dlc=4, ack_raw=27918343, settle_until_us=1240004)

    def __call__(self, request_id: int, command: str) -> str:
        """Accept each unique command once and produce its explicit synthetic wire line."""
        self.calls.append((request_id, command))
        if self.clock:
            self.clock.value += self.exchange_seconds
        if command == "ping":
            return f"ok {request_id} ping state=boot enabled=00 boot={self.boot}\r\n"
        tokens = command.split()
        if len(tokens) < 2 or tokens[0] != "adrc":
            raise AssertionError("unexpected command grammar")
        subcommand = tokens[1]
        arguments = dict(token.split("=", 1) for token in tokens[2:])
        if subcommand == "persist_begin":
            assert arguments == {"motor": "7"}
            assert request_id == self.request
            response = dict(schema=1, id=self.request, accepted=1)
        elif subcommand == "persist_status":
            assert not arguments
            response = self.status()
        elif subcommand == "persist_item":
            assert set(arguments) == {"id", "phase", "index"}
            assert int(arguments["id"]) == self.request and int(arguments["phase"]) == self.phase
            index = int(arguments["index"])
            record = deepcopy(self.records[self.phase][index])
            identity = record.pop("identity")
            response = dict(schema=1, id=self.request, index=index, **record, **identity)
        elif subcommand == "persist_health":
            assert arguments == {"id": str(self.request)}
            response = self.health()
        elif subcommand == "persist_save_receipt":
            assert arguments == {"id": str(self.request)}
            response = self.journal()
        else:
            assert arguments == {"id": str(self.request)}
            self.phase = {"persist_prepare": 4, "persist_verify": 6, "persist_commit": 10}.get(subcommand, self.phase)
            if subcommand == "persist_release":
                self.released = True
            elif subcommand not in ("persist_prepare", "persist_verify", "persist_commit"):
                raise AssertionError("unrequested mutation")
            response = dict(schema=1, id=self.request, accepted=1)
        if self.hook:
            self.hook(subcommand, self, response)
        fields = " ".join(f"{key}={format(value, '.9g') if isinstance(value, float) else value}" for key, value in response.items())
        line = f"ok {request_id} adrc {subcommand} {fields}\r\n"
        return self.line_hook(subcommand, self, line) if self.line_hook else line


class LegacyLfExchange(FakeExchange):
    """Keep synthetic persistence data while emitting the actual existing C ping LF shape."""

    def __call__(self, request_id: int, command: str) -> str:
        """Preserve fixture timing/call recording and change only the legacy ping reply."""
        response = super().__call__(request_id, command)
        if command == "ping":
            return f"ok {request_id} ping state=boot enabled=00 boot={self.boot}\n"
        return response


class StartupDelayClock(FakeClock):
    """Apply one lawful scheduling pause before the first requested command."""

    def __init__(self, delay_seconds: float) -> None:
        """Retain the requested pause and initialize the existing fast fixture clock."""
        super().__init__()
        self.delay_seconds = delay_seconds
        self.read_count = 0

    def __call__(self) -> float:
        """Advance on the first request check after the two initial budget reads."""
        self.read_count += 1
        if self.read_count == 3:
            self.value += self.delay_seconds
        return self.value


class ActualBeginExchange(FakeExchange):
    """Bind the synthetic transaction to the actual begin request, as the C contract does."""

    def __call__(self, request_id: int, command: str) -> str:
        """Use the actual begin ID for every later ACK, item identity and stable context."""
        if command == "adrc persist_begin motor=7":
            self.request = request_id
            for records in self.records.values():
                for record in records:
                    record["identity"]["request_id"] = request_id
        return super().__call__(request_id, command)


class SessionClientTests(unittest.TestCase):
    """Exercise complete synthetic flows and isolated fail-closed evidence boundaries."""

    def run_session(self, fake: FakeExchange | None = None, *, readonly=None, directory=None,
                    first_request: int = 100, configured_inputs=None):
        """Create one offline session with retained once directory and a fast injected clock."""
        clock = FakeClock()
        fake = fake or FakeExchange()
        if fake.clock is None:
            fake.clock = clock
        client = CLIENT.PersistenceClient(configured_inputs or inputs(), directory or retained_directory(),
                                          fake, clock=fake.clock, poll_wait=fake.clock.wait,
                                          first_request_id=first_request)
        result = client.run_prepare_save() if readonly is None else client.run_readonly_verify(readonly)
        return client, fake, result

    def assert_failure(self, fake: FakeExchange, *, readonly=None, directory=None, pattern=None):
        """Require one exact fixture to fail without any cleanup or automatic recovery command."""
        with self.assertRaises((ValueError, OSError, TimeoutError), msg=pattern):
            self.run_session(fake, readonly=readonly, directory=directory)
        forbidden = ("disable", "stop", "reset", "enable", "zero")
        self.assertFalse(any(any(word in command for word in forbidden) for _, command in fake.calls))
        for subcommand in ("persist_begin", "persist_prepare", "persist_verify", "persist_commit", "persist_release"):
            self.assertLessEqual(sum(command.startswith("adrc " + subcommand) for _, command in fake.calls), 1)

    def test_all_four_source_combinations_and_once_binding(self) -> None:
        """Require legal sources, exact canonical cold hash and one application commit when needed."""
        for pmax in (12.5, 512.0):
            for mode in (1, 2):
                with self.subTest(pmax=pmax, mode=mode):
                    client, fake, result = self.run_session(FakeExchange(pmax=pmax, mode=mode))
                    self.assertTrue(result["released"])
                    self.assertEqual(result["skipped_save"], fake.skip)
                    self.assertEqual(len(client.evidence["baseline"]), 45)
                    commits = [item for item in fake.calls if "persist_commit" in item[1]]
                    self.assertEqual(len(commits), int(not fake.skip))
                    if not fake.skip:
                        token = Path(client.evidence["once_token"])
                        binding = json.loads(token.read_text("ascii"))["binding"]
                        canonical = json.dumps(client.evidence["baseline"], sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
                        self.assertEqual(binding["cold_baseline_sha256"], hashlib.sha256(canonical).hexdigest())
                        self.assertEqual(binding["cdc_link_state"], 2147483663)
                        self.assertEqual(binding["request_id"], 100)
                    self.assertEqual(len({request for request, _ in fake.calls}), len(fake.calls))

    def test_readonly_second_cold_reference_and_existing_token(self) -> None:
        """Compare only raw RW config across a cold boot and never consume or consult a token."""
        directory = retained_directory()
        (directory / ONCE.TOKEN_FILENAME).write_bytes(b"previously consumed")
        original = synthetic_records(1, boot=9, request=90, position=-8.0)
        client, fake, result = self.run_session(FakeExchange(pmax=512.0, mode=2, position=7.0), readonly=original, directory=directory)
        self.assertTrue(result["released"])
        self.assertNotIn("once_token", client.evidence)
        self.assertTrue(all(command == "ping" or command.split()[1] in ("persist_begin", "persist_status", "persist_item", "persist_health", "persist_release") for _, command in fake.calls))

    def test_readonly_missing_target_and_each_other_rw_change(self) -> None:
        """Never repair a failed cold verify or permit one collateral writable raw bit change."""
        original = synthetic_records(1, boot=9, request=90)
        for pmax, mode in ((12.5, 1), (12.5, 2), (512.0, 1)):
            self.assert_failure(FakeExchange(pmax=pmax, mode=mode), readonly=original)
        for rid in tuple(range(11)) + tuple(range(21, 36)):
            if rid in (10, 21):
                continue
            fake = FakeExchange(pmax=512.0, mode=2)
            fake.records[1][REGISTERS.index(rid)]["raw"] ^= 1
            with self.subTest(rid=rid):
                self.assert_failure(fake, readonly=original)
        self.run_session(FakeExchange(pmax=512.0, mode=2), readonly=original)

    def test_input_validation_is_immutable_and_precedes_any_exchange(self) -> None:
        """Refuse unfrozen identities, false cold confirmation and invalid request values offline."""
        for changes in ({"firmware_hex_sha256": "0" * 64}, {"firmware_hex_sha256": "A" * 64},
                        {"source_manifest_sha256": "bad"}, {"controller_usb_serial": "COM4"},
                        {"expected_boot": 0}, {"expected_boot": True}, {"expected_boot": 2 ** 32},
                        {"physical_cold_confirmed": False}, {"physical_cold_confirmed": 1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                inputs(**changes)
        frozen = inputs()
        with self.assertRaises(FrozenInstanceError):
            frozen.expected_boot = 20
        for request in (0, True, -1, 2 ** 32):
            fake = FakeExchange()
            with self.assertRaises(ValueError):
                self.run_session(fake, first_request=request)
            self.assertEqual(fake.calls, [])

    def test_same_object_never_starts_a_second_session(self) -> None:
        """Reject a repeated high-level call before another begin or mutation can be emitted."""
        client, fake, _ = self.run_session()
        count = len(fake.calls)
        with self.assertRaises(ValueError):
            client.run_prepare_save()
        with self.assertRaises(ValueError):
            client.run_readonly_verify(synthetic_records(1))
        self.assertEqual(len(fake.calls), count)

    def test_context_fault_known_flags_and_false_ready(self) -> None:
        """Keep independently invalid state and health fields from authorizing a later mutation."""
        cases = [("persist_status", key, value) for key, value in
                 (("boot", 12), ("id", 101), ("owner", 0), ("link", 15), ("epoch", 0),
                  ("session", 0), ("fault", 1), ("error", 1), ("pmax_known", 0), ("mode_known", 0),
                  ("snapshot_phase", 4), ("state", 99), ("save_used", 1), ("released", 1), ("locked", 0))]
        cases += [("persist_health", key, value) for key, value in
                  (("owner", 14), ("valid", 0), ("disabled", 0), ("idle", 0), ("fault", 1),
                   ("health_proven", 0), ("continuity_valid", 0), ("velocity", .051), ("torque", -.151),
                   ("motor_c", 50.0), ("driver_c", -0.1), ("observed_us", 1),
                   ("reference_native", 2.0), ("native_position", 1.1), ("tolerance_native", 1.0))]
        for subcommand, key, value in cases:
            def corrupt(sub, fake, response, target=subcommand, field=key, replacement=value):
                """Change one initial stable response field and leave all other evidence legal."""
                if sub == target and fake.phase == 1:
                    response[field] = replacement
            with self.subTest(subcommand=subcommand, key=key):
                self.assert_failure(FakeExchange(hook=corrupt))
        self.run_session()

    def test_late_context_change_and_complete_fault_are_not_success(self) -> None:
        """Reject changed ownership after baseline and nonzero faults in pure COMPLETE."""
        for phase, key, value in ((4, "owner", 14), (6, "epoch", 18), (10, "fault", 1), (10, "save_uncertain", 1)):
            def corrupt(sub, fake, response, selected=phase, field=key, replacement=value):
                """Alter exactly one later status field after the earlier valid gates."""
                if sub == "persist_status" and fake.phase == selected:
                    response[field] = replacement
            with self.subTest(phase=phase, key=key):
                self.assert_failure(FakeExchange(hook=corrupt))

    def test_item_evidence_and_raw_tampering_at_every_snapshot(self) -> None:
        """Detect sequence, owner, phase, time, validity, target and collateral raw damage."""
        for phase in (1, 4, 6, 10):
            for field, value in (("raw", raw_float(1.1)), ("valid", 0), ("owner_generation", 14),
                                 ("phase", 8), ("sequence", 0), ("created_us", 1), ("rid", 81)):
                def corrupt(sub, fake, response, selected=phase, key=field, replacement=value):
                    """Damage only RID50 in one otherwise valid stable full45 snapshot."""
                    if sub == "persist_item" and fake.phase == selected and response["index"] == 43:
                        response[key] = replacement
                with self.subTest(phase=phase, field=field):
                    self.assert_failure(FakeExchange(hook=corrupt))
        for phase in (4, 6, 10):
            for rid in (0, 10, 21, 35):
                fake = FakeExchange()
                fake.records[phase][REGISTERS.index(rid)]["raw"] ^= 1
                with self.subTest(phase=phase, rid=rid):
                    self.assert_failure(fake)
        self.run_session()

    def test_postverified_health_and_cross_phase_order(self) -> None:
        """Require fresh postverify FD and strict item sequence/time progression across stages."""
        def old_fd(sub, fake, response):
            """Place only SAVE_HEALTH feedback exactly at verified_us, which is not newer."""
            if sub == "persist_health" and fake.phase == 6:
                response["observed_us"] = fake.status()["verified_us"]
        self.assert_failure(FakeExchange(hook=old_fd))
        for phase in (4, 6, 10):
            fake = FakeExchange()
            for record in fake.records[phase]:
                record["identity"]["sequence"] -= 100 if phase == 10 else 47
            with self.subTest(phase=phase):
                self.assert_failure(fake)
        self.run_session()

    def test_fresh_age_does_not_replace_current_snapshot_fd_order(self) -> None:
        """Refuse a health report just before this snapshot completed, despite a fresh age."""
        for phase in (1, 4, 10):
            def earlier(sub, fake, response, selected=phase):
                """Move only observed_us one microsecond before the current45 terminal observation."""
                if sub == "persist_health" and fake.phase == selected:
                    last = fake.records[selected][-1]
                    response["observed_us"] = max(last["received_us"], last["send_completed_observed_us"]) - 1
            with self.subTest(phase=phase):
                self.assert_failure(FakeExchange(hook=earlier))
        self.run_session()

    def test_snapshot_fd_strict_boundary_matrix(self) -> None:
        """Require -1/0/+1us FD boundaries to reject/reject/accept at phases1,4,10."""
        next_mutation = {1: "persist_prepare", 4: "persist_verify", 10: "persist_release"}
        for phase in (1, 4, 10):
            for delta in (-1, 0, 1):
                def shifted(subcommand, fake, response, selected=phase, offset=delta):
                    """Change only this phase's FD observation against the completed45 bound."""
                    if subcommand == "persist_health" and fake.phase == selected:
                        final = fake.records[selected][-1]
                        response["observed_us"] = max(final["received_us"], final["send_completed_observed_us"]) + offset
                fake = FakeExchange(hook=shifted)
                with self.subTest(phase=phase, delta_us=delta):
                    if delta <= 0:
                        self.assert_failure(fake)
                        self.assertFalse(any(next_mutation[phase] in command for _, command in fake.calls))
                    else:
                        _, _, result = self.run_session(fake)
                        self.assertTrue(result["released"])

    def test_exact_journal_evidence_and_settle_gate(self) -> None:
        """Reject one isolated missing or incorrect Save journal bound and restore a valid control."""
        corruptions = {"valid": 0, "boot_epoch": 12, "request_id": 101, "owner_generation": 14,
                       "phase": 6, "sequence": 137, "created_us": 1200000, "boundary_us": 1210003,
                       "submitted_us": 1210006, "send_completed_observed_us": 1310000,
                       "rx_lower_us": 1210005, "received_us": 1210001, "tx_id": 2046, "tx_dlc": 8,
                       "tx_raw": 0, "ack_id": 24, "ack_dlc": 8, "ack_raw": 0,
                       "settle_until_us": 1240003}
        for key, value in corruptions.items():
            def corrupt(sub, fake, response, field=key, replacement=value):
                """Alter exactly one receipt property after valid readback and health."""
                if sub == "persist_save_receipt":
                    response[field] = replacement
            with self.subTest(key=key):
                self.assert_failure(FakeExchange(hook=corrupt))
        def missing(sub, fake, response):
            """Return explicit invalid journal instead of inventing placeholder evidence."""
            if sub == "persist_save_receipt":
                response.clear(); response.update(schema=1, id=100, valid=0)
        self.assert_failure(FakeExchange(hook=missing))
        def future_settle(sub, fake, response):
            """Leave 30ms intact but move the settle boundary after readback creation."""
            if sub == "persist_save_receipt":
                response["settle_until_us"] = 1250001
        self.assert_failure(FakeExchange(hook=future_settle))
        self.run_session()

    def test_wire_parser_exactness_and_transport_failure(self) -> None:
        """Reject errors, wrong identities, versions, duplicate keys, nonfinite and truncated lines."""
        transforms = [lambda line: line.replace("ok 101", "ok 102", 1),
                      lambda line: line.replace("persist_status", "persist_health", 1),
                      lambda line: line.replace("schema=1", "schema=2", 1),
                      lambda line: line.replace(" ready=1", " ready=2", 1),
                      lambda line: line.replace(" boot=11", " boot=4294967296", 1),
                      lambda line: line.replace(" boot=11", " boot=-11", 1),
                      lambda line: line.replace(" boot=11", " boot=1e1", 1),
                      lambda line: line.replace(" now_us=1005400", " now_us=18446744073709551616", 1),
                      lambda line: line.replace(" ready=1", "", 1),
                      lambda line: line.replace("\r\n", " ready=1\r\n"),
                      lambda line: line[:-2], lambda line: line + line,
                      lambda line: line.replace("schema=1", "schema=1 中文=1", 1),
                      lambda line: "error 101 adrc persist_status schema=1 code=denied\r\n",
                      lambda line: line.replace("\r\n", " extra=" + "x" * 520 + "\r\n")]
        for transform in transforms:
            def corrupt(sub, fake, line, transform=transform):
                """Modify only the first baseline status line, with correct earlier begin."""
                return transform(line) if sub == "persist_status" and fake.phase == 1 else line
            with self.subTest(transform=transform):
                self.assert_failure(FakeExchange(line_hook=corrupt))
        for value in ("nan", "inf", "1e999"):
            def nan_reply(sub, fake, line, invalid=value):
                """Replace a required float with a nonfinite syntactic value."""
                return line.replace("velocity=0", "velocity=" + invalid) if sub == "persist_health" else line
            self.assert_failure(FakeExchange(line_hook=nan_reply))
        def disconnected(sub, fake, line):
            """Simulate an absent response without authorizing retry or recovery."""
            return None if sub == "persist_prepare" else line
        self.assert_failure(FakeExchange(line_hook=disconnected))
        self.run_session()

    def test_readonly_extra_fields_and_ping_do_not_replace_proof(self) -> None:
        """Permit nonconflicting diagnostic extensions and require watchdog ping with a fresh ID."""
        def extra(sub, fake, line):
            """Add only a harmless schema extension to read-only replies."""
            return line.replace("\r\n", " diagnostic=1\r\n") if sub in ("persist_status", "persist_item", "persist_health", "persist_save_receipt") else line
        _, fake, _ = self.run_session(FakeExchange(line_hook=extra))
        self.assertTrue(any(command == "ping" for _, command in fake.calls))

    def test_exact_c_legacy_lf_ping_wire_reply(self) -> None:
        """Accept the unchanged raw DEVELOPMENT02 C response, including its single LF."""
        CLIENT.PersistenceClient._validate_ping(ACTUAL_C_PING_REPLY, 763)

    def test_legacy_ping_word_and_line_ending_boundaries(self) -> None:
        """Accept ping LF/CRLF only, with exact ID and one bounded ASCII response line."""
        for response in (ACTUAL_C_PING_REPLY, ACTUAL_C_PING_REPLY.replace("\n", "\r\n")):
            with self.subTest(valid=repr(response)):
                CLIENT.PersistenceClient._validate_ping(response, 763)
        invalid_responses = (
            ACTUAL_C_PING_REPLY.replace("763", "764", 1),
            "ok 763 pong\r\n", "ok 763 unrelated\r\n", "ok 763 pingx\n", "ok 763\r\n",
            "error 763 ping state=boot\n", "ok 763 ping\r\r\n", "ok 763 ping\r",
            "ok 763 ping", "ok 763 ping\n\n", "ok 763 ping\r\n\r\n",
            "ok 763 ping\nok 763 ping\n", "ok 763 ping\r\nok 763 ping\r\n",
            "ok 763 ping boot=中文\n", "ok 763 ping boot=1\x00\n", "ok 763 ping boot=1\t\n",
            "ok 763 ping extra=" + "x" * 520 + "\n", None,
        )
        for response in invalid_responses:
            with self.subTest(invalid=repr(response)), self.assertRaises(ValueError):
                CLIENT.PersistenceClient._validate_ping(response, 763)
        persistent = "ok 763 adrc persist_begin schema=1 id=763 accepted=1\r\n"
        fields = {"id": "u32", "accepted": "bool"}
        self.assertEqual(CLIENT._parse_reply(persistent, 763, "persist_begin", fields)["accepted"], 1)
        with self.assertRaises(ValueError):
            CLIENT._parse_reply(persistent.replace("\r\n", "\n"), 763, "persist_begin", fields)

    def test_full_session_with_real_legacy_lf_ping(self) -> None:
        """Complete the synthetic Save session using actual C-shaped LF watchdog replies."""
        client, fake, result = self.run_session(LegacyLfExchange())
        self.assertTrue(result["released"])
        pings = [entry for entry in client.transcript if entry["command"] == "ping"]
        self.assertTrue(pings)
        for entry in pings:
            self.assertEqual(entry["reply"], f"ok {entry['request_id']} ping state=boot enabled=00 boot=11\n")
            self.assertNotIn("\r", entry["reply"])
        self.assertEqual(sum("persist_commit" in command for _, command in fake.calls), 1)

    def test_startup_pause_binds_actual_begin_request_and_once(self) -> None:
        """Preserve due ping at 250ms and pin the complete lifecycle/claim to actual begin."""
        for delay in (0.249999, 0.250, 0.300, 4.0):
            clock = StartupDelayClock(delay)
            fake = ActualBeginExchange(clock=clock)
            with self.subTest(startup_pause_seconds=delay):
                client, _, result = self.run_session(fake)
                expected_root = 100 if delay < 0.250 else 101
                self.assertTrue(result["released"])
                self.assertEqual(client.transaction_id, expected_root)
                begin = next(entry for entry in client.transcript if entry["command"] == "adrc persist_begin motor=7")
                self.assertEqual(begin["request_id"], expected_root)
                if delay >= 0.250:
                    self.assertEqual(fake.calls[:2], [(100, "ping"), (101, "adrc persist_begin motor=7")])
                else:
                    self.assertEqual(fake.calls[0], (100, "adrc persist_begin motor=7"))
                for name in ("baseline", "prepared", "verified", "readback"):
                    self.assertTrue(all(record["identity"]["request_id"] == expected_root for record in client.evidence[name]))
                self.assertTrue(all(status["id"] == expected_root for status in client.evidence["statuses"]))
                self.assertEqual(client.evidence["save_journal"]["request_id"], expected_root)
                binding = json.loads(Path(client.evidence["once_token"]).read_text("ascii"))["binding"]
                self.assertEqual(binding["request_id"], expected_root)
                self.assertEqual(len({request for request, _ in fake.calls}), len(fake.calls))

    def test_readonly_startup_pause_uses_actual_begin_request(self) -> None:
        """Use the same actual root rule for cold read-only verification without claiming Save."""
        clock = StartupDelayClock(0.300)
        fake = ActualBeginExchange(pmax=512.0, mode=2, clock=clock)
        original = synthetic_records(1, boot=9, request=90, position=-8.0)
        client, _, result = self.run_session(fake, readonly=original)
        self.assertTrue(result["released"])
        self.assertEqual(client.transaction_id, 101)
        self.assertEqual(fake.calls[:2], [(100, "ping"), (101, "adrc persist_begin motor=7")])
        self.assertTrue(all(record["identity"]["request_id"] == 101 for record in client.evidence["baseline"]))
        self.assertNotIn("once_token", client.evidence)
        self.assertFalse(any(command.split()[1] in ("persist_prepare", "persist_verify", "persist_commit") for _, command in fake.calls if command != "ping"))

    def test_startup_ping_loss_and_wrong_boot_fail_closed(self) -> None:
        """Neither a lost pre-begin ping nor a later wrong persistence boot may authorize Save."""
        clock = StartupDelayClock(0.300)
        fake = ActualBeginExchange(clock=clock)
        directory = retained_directory()
        def lost_ping(request_id, command):
            """Deliver the first ping but lose its reply before any begin can be emitted."""
            response = fake(request_id, command)
            if command == "ping":
                raise TimeoutError("synthetic startup ping reply lost")
            return response
        client = CLIENT.PersistenceClient(inputs(), directory, lost_ping, clock=clock,
                                          poll_wait=clock.wait, first_request_id=100)
        with self.assertRaises(TimeoutError):
            client.run_prepare_save()
        self.assertEqual(fake.calls, [(100, "ping")])
        self.assertIsNone(client.transaction_id)
        self.assertEqual(client.mutation_sent, set())
        self.assertFalse((directory / ONCE.TOKEN_FILENAME).exists())
        wrong_boot = ActualBeginExchange(boot=12, clock=StartupDelayClock(0.300))
        self.assert_failure(wrong_boot)
        self.assertTrue(any(command == "adrc persist_status" for _, command in wrong_boot.calls))
        self.assertFalse(any("persist_prepare" in command or "persist_commit" in command for _, command in wrong_boot.calls))

    def test_startup_pause_deadlines_and_late_root_drift(self) -> None:
        """Keep original stage/total expiry and refuse changed root after a delayed accepted begin."""
        for delay, label in ((5.0, "stage budget"), (42.0, "total budget")):
            clock = StartupDelayClock(delay)
            fake = ActualBeginExchange(clock=clock)
            client = CLIENT.PersistenceClient(inputs(), retained_directory(), fake, clock=clock,
                                              poll_wait=clock.wait, first_request_id=100)
            with self.subTest(startup_pause_seconds=delay), self.assertRaisesRegex(TimeoutError, label):
                client.run_prepare_save()
            self.assertEqual(fake.calls, [])
            self.assertIsNone(client.transaction_id)
        def drift(subcommand, fake, response):
            """Change only a prepared status root after legal initial root101 evidence."""
            if subcommand == "persist_status" and fake.phase == 4:
                response["id"] = 102
        fake = ActualBeginExchange(clock=StartupDelayClock(0.300), hook=drift)
        self.assert_failure(fake)
        self.assertTrue(any("persist_prepare" in command for _, command in fake.calls))
        self.assertFalse(any("persist_verify" in command or "persist_commit" in command for _, command in fake.calls))

    def test_known_transient_is_polled_without_mutation_retry(self) -> None:
        """Allow safe mapping isolation while baseline or authorized write is still in flight."""
        seen = set()
        def pending(sub, fake, response):
            """Publish one legal not-ready transitional state per relevant phase."""
            if sub == "persist_status" and fake.phase in (1, 4) and fake.phase not in seen:
                seen.add(fake.phase)
                response.update(state=1 if fake.phase == 1 else 3, ready=0, pmax_known=0, mode_known=0)
        self.run_session(FakeExchange(hook=pending))

    def test_fixed_deadline_exchange_limit_rollback_and_request_overflow(self) -> None:
        """Never renew a pending stage budget, spin without a bound, accept rollback or wrap IDs."""
        clock = FakeClock()
        def pending(sub, fake, response):
            """Leave baseline unfinished while keeping every returned state otherwise legal."""
            if sub == "persist_status":
                response.update(state=1, ready=0, pmax_known=0, mode_known=0)
        fake = FakeExchange(hook=pending, clock=clock)
        self.assert_failure(fake)
        self.assertLessEqual(clock.value, 52.1)
        clock = FakeClock()
        def backwards(sub, fake, response):
            """Move only host monotonic time backwards during an otherwise valid reply."""
            if sub == "persist_status":
                clock.value -= 1.0
        self.assert_failure(FakeExchange(clock=clock, hook=backwards))
        fake = FakeExchange(request=4294967295)
        with self.assertRaises(ValueError):
            self.run_session(fake, first_request=4294967295)
        self.assertEqual(len(fake.calls), 1)
        stagnant = FakeClock()
        stagnant.wait = lambda: None
        fake = FakeExchange(hook=pending)
        fake.clock = stagnant
        fake.exchange_seconds = 0.0
        self.assert_failure(fake)
        self.assertLessEqual(len(fake.calls), 2048)

    def test_existing_and_failed_once_claim_prevent_commit(self) -> None:
        """Retain consumed tokens after existing, short-write and fsync failures before commit."""
        directory = retained_directory()
        token = directory / ONCE.TOKEN_FILENAME
        token.write_bytes(b"consumed")
        fake = FakeExchange()
        self.assert_failure(fake, directory=directory)
        self.assertEqual(token.read_bytes(), b"consumed")
        self.assertFalse(any("persist_commit" in command for _, command in fake.calls))
        for function, result in (("write", 0), ("fsync", OSError("synthetic fsync failure"))):
            directory = retained_directory()
            fake = FakeExchange()
            with patch.object(ONCE.os, function, side_effect=result if isinstance(result, Exception) else None,
                              return_value=result if not isinstance(result, Exception) else None):
                self.assert_failure(fake, directory=directory)
            self.assertTrue((directory / ONCE.TOKEN_FILENAME).exists())
            self.assertFalse(any("persist_commit" in command for _, command in fake.calls))

    def test_once_fsync_delay_cannot_authorize_stale_commit(self) -> None:
        """Retain a real consumed claim when fsync latency makes the last FD report stale."""
        clock, fake, directory = FakeClock(), FakeExchange(), retained_directory()
        fake.clock = clock
        original_fsync = ONCE.os.fsync
        def slow_fsync(descriptor):
            """Perform real durable synchronization then advance only the synthetic host clock."""
            original_fsync(descriptor)
            clock.value += 0.101
        with patch.object(ONCE.os, "fsync", side_effect=slow_fsync):
            self.assert_failure(fake, directory=directory)
        self.assertTrue((directory / ONCE.TOKEN_FILENAME).exists())
        self.assertFalse(any("persist_commit" in command for _, command in fake.calls))

    def test_skip_final_release_still_requires_exact_target(self) -> None:
        """Refuse one changed mapping after an already-target snapshot and accepted release."""
        for key, value in (("pmax_raw", raw_float(12.5)), ("mode", 1)):
            def corrupt(sub, fake, response, field=key, replacement=value):
                """Change only one mapping field in the final unlocked status report."""
                if sub == "persist_status" and fake.released:
                    response[field] = replacement
            with self.subTest(key=key):
                self.assert_failure(FakeExchange(pmax=512.0, mode=2, hook=corrupt))
        self.run_session(FakeExchange(pmax=512.0, mode=2))

    def test_raw_target_cannot_be_hidden_by_consistent_false_skip_flags(self) -> None:
        """Derive target presence from raw words rather than allow state/skip flags to cause a Save."""
        fake = FakeExchange(pmax=512.0, mode=2)
        fake.skip = False
        self.assert_failure(fake)
        self.assertFalse(any("persist_prepare" in command or "persist_commit" in command for _, command in fake.calls))
        self.run_session(FakeExchange(pmax=512.0, mode=2))

    def test_9g_float_reconstruction_at_native_and_health_boundaries(self) -> None:
        """Accept exact C float limits while rejecting the next native float beyond tolerance."""
        def boundary_health(sub, fake, response):
            """Set the legal velocity/torque boundaries as C floats serialized with 9g."""
            if sub == "persist_health":
                response.update(velocity=float32(.05), torque=-float32(.15))
        self.run_session(FakeExchange(hook=boundary_health))
        for phase in (1, 4, 6, 10):
            fake = FakeExchange()
            bound = tolerance(fake.pmax, fake.pmax if phase == 1 else 512.0)
            reference = 1.0
            # Choose the first representable float above reference+tolerance;
            # its C float difference also remains strictly above the bound.
            bits = raw_float(reference + bound) + 1
            fake.records[phase][43]["raw"] = bits
            with self.subTest(phase=phase):
                self.assert_failure(fake)
        self.run_session()

    def test_global_budget_and_device_clock_regressions(self) -> None:
        """Check immutable 42s expiry and monotonic device observations separately."""
        clock = FakeClock()
        def expire(sub, fake, response):
            """Exceed the total budget during the sole begin exchange."""
            if sub == "persist_begin":
                clock.value += 42.0
        fake = FakeExchange(clock=clock, hook=expire)
        with self.assertRaisesRegex(TimeoutError, "total budget"):
            self.run_session(fake)
        self.assertEqual(len(fake.calls), 1)
        def regress(sub, fake, response):
            """Keep a valid health record except for device now_us preceding status now_us."""
            if sub == "persist_health":
                response["now_us"] -= 1
        self.assert_failure(FakeExchange(hook=regress))

    def test_partial_token_write_and_commit_error_are_irrevocable(self) -> None:
        """Keep partially written tokens and erroring commit claims, with zero automatic retry."""
        directory, fake = retained_directory(), FakeExchange()
        original_write = ONCE.os.write
        progress = []
        def partial_write(descriptor, data):
            """Write one real byte then return invalid progress to preserve a partial token."""
            if not progress:
                progress.append(1)
                return original_write(descriptor, data[:1])
            return 0
        with patch.object(ONCE.os, "write", side_effect=partial_write):
            self.assert_failure(fake, directory=directory)
        self.assertEqual((directory / ONCE.TOKEN_FILENAME).stat().st_size, 1)
        self.assertFalse(any("persist_commit" in command for _, command in fake.calls))
        directory = retained_directory()
        def rejected(sub, fake, line):
            """Return an exact fixed-code commit error after the durable claim exists."""
            if sub == "persist_commit":
                return line.replace("ok ", "error ", 1).split(" schema=")[0] + " schema=1 code=health\r\n"
            return line
        fake = FakeExchange(line_hook=rejected)
        self.assert_failure(fake, directory=directory)
        self.assertTrue((directory / ONCE.TOKEN_FILENAME).exists())
        self.assertEqual(sum("persist_commit" in command for _, command in fake.calls), 1)

    def test_unknown_pending_phase_and_changed_final_verified_time(self) -> None:
        """Refuse undefined snapshot enums immediately and retain the original final Save timestamp."""
        def unknown(sub, fake, response):
            """Publish only an unknown snapshot phase in an otherwise legal pending status."""
            if sub == "persist_status":
                response.update(state=1, ready=0, pmax_known=0, mode_known=0, snapshot_phase=99)
        fake = FakeExchange(hook=unknown)
        with self.assertRaisesRegex(ValueError, "snapshot phase"):
            self.run_session(fake)
        self.assertEqual(sum("persist_status" in command for _, command in fake.calls), 1)
        def changed_final_time(sub, fake, response):
            """Change only verified_us after the otherwise successful explicit release."""
            if sub == "persist_status" and fake.released:
                response["verified_us"] = 0
        self.assert_failure(FakeExchange(hook=changed_final_time))
        self.run_session()

    def test_commit_timeout_is_once_and_transcript_preserves_failure(self) -> None:
        """Consume the token before a lost commit reply and never retry or auto-release."""
        def timeout(sub, fake, response):
            """Raise only after the first commit has been delivered to the synthetic peer."""
            if sub == "persist_commit":
                raise TimeoutError("synthetic commit response lost")
        fake, directory, clock = FakeExchange(hook=timeout), retained_directory(), FakeClock()
        client = CLIENT.PersistenceClient(inputs(), directory, fake, clock=clock, poll_wait=clock.wait, first_request_id=100)
        with self.assertRaises(TimeoutError):
            client.run_prepare_save()
        self.assertEqual(sum("persist_commit" in command for _, command in fake.calls), 1)
        self.assertTrue((directory / ONCE.TOKEN_FILENAME).exists())
        self.assertIn("persist_commit", client.mutation_sent)
        self.assertIsNone(client.transcript[-1]["reply"])
        self.assertIn("synthetic commit response lost", client.transcript[-1]["exception"])
        self.assertFalse(any("persist_release" in command for _, command in fake.calls))

    def test_release_failure_does_not_erase_prior_partial_evidence(self) -> None:
        """Keep all evidence and consumed token while refusing a false final release success."""
        def fail_release(sub, fake, response):
            """Keep the final status locked despite an accepted release mutation."""
            if sub == "persist_status" and fake.released:
                response.update(locked=1, released=0)
        clock, fake, directory = FakeClock(), FakeExchange(hook=fail_release), retained_directory()
        client = CLIENT.PersistenceClient(inputs(), directory, fake, clock=clock, poll_wait=clock.wait, first_request_id=100)
        with self.assertRaises((ValueError, TimeoutError)):
            client.run_prepare_save()
        self.assertIn("readback", client.evidence)
        self.assertIn("save_journal", client.evidence)
        self.assertTrue((directory / ONCE.TOKEN_FILENAME).exists())

    def test_release_acceptance_can_wait_for_real_quiet_without_resending(self) -> None:
        """Poll an accepted release until final idle/quiet proof, with no renewed deadline."""
        pending_statuses = []
        def delayed(sub, fake, response):
            """Model three not-yet-released statuses while B2 waits for real idle/quiet."""
            if sub == "persist_status" and fake.released and len(pending_statuses) < 3:
                pending_statuses.append(response["now_us"])
                response.update(locked=1, released=0, ready=0)
        client, fake, result = self.run_session(FakeExchange(hook=delayed))
        self.assertTrue(result["released"])
        self.assertEqual(len(pending_statuses), 3)
        self.assertEqual(sum("persist_release" in command for _, command in fake.calls), 1)
        self.assertTrue(client.evidence["released"])
        def release_fault(sub, fake, response):
            """Latch one fault while release is pending instead of granting an unlocked state."""
            if sub == "persist_status" and fake.released:
                response.update(locked=1, released=0, ready=0, fault=1)
        self.assert_failure(FakeExchange(hook=release_fault))

if __name__ == "__main__":
    unittest.main(verbosity=2)
