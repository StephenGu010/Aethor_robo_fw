"""Deterministic dependency-free simulator for the aethor-text-v1 contract."""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass

JOINT_COUNT = 7
ALL_MOTORS_MASK = 0x7F
REPLAY_CAPACITY = 32
REPLAY_RETENTION_MS = 60_000


def encode_line(body: str) -> bytes:
    """Encodes one plain ASCII request with the protocol LF terminator."""
    return body.encode("ascii") + b"\n"


def compact_number(value: float) -> str:
    """Formats a finite value with three decimals and no redundant zeros."""
    if abs(value) < 0.0005:
        value = 0.0
    return f"{value:.3f}".rstrip("0").rstrip(".")


@dataclass
class SimulatedMotion:
    """Stores one bounded simulated bench or formal motion."""

    request_id: int
    namespace: str
    selected_mask: int
    start_ms: int
    duration_ms: int
    start_deg: list[float]
    target_deg: list[float]


class AethorTextSimulator:
    """Emulates readable protocol parsing, replay, streaming, and safe motion."""

    def __init__(self, boot_id: int = 1, profile: str = "bench") -> None:
        """Initializes one deterministic seven-axis simulator boot."""
        if profile not in {"bench", "arm"}:
            raise ValueError("profile must be bench or arm")
        self.boot_id = boot_id or 1
        self.profile = profile
        self.now_ms = 0
        self.arm_state = "unaligned"
        self.aligned = False
        self.motor_present_mask = ALL_MOTORS_MASK
        self.motor_enabled_mask = 0
        self.motor_fault_mask = 0
        self.joint_position_deg = [0.0] * JOINT_COUNT
        self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
        self.stream_kind = "off"
        self.stream_rate_hz = 0
        self.next_stream_ms = 0
        self.last_request_ms = 0
        self.watchdog_reported = False
        self.active_motion: SimulatedMotion | None = None
        self._pending_outputs: list[str] = []
        self._rx_buffer = bytearray()
        self._replay: OrderedDict[int, tuple[str, list[str], int]] = OrderedDict()

    @staticmethod
    def _parse_motor_list(value: str) -> tuple[list[int], int]:
        """Parses one unique ascending one-based motor list."""
        motors = [int(item) for item in value.split(",")]
        if (not motors or motors != sorted(set(motors)) or
                any(item < 1 or item > JOINT_COUNT for item in motors)):
            raise ValueError("motors")
        return motors, sum(1 << (item - 1) for item in motors)

    @staticmethod
    def _parse_vector(value: str) -> list[float]:
        """Parses exactly seven finite comma-separated values."""
        values = [float(item) for item in value.split(",")]
        if len(values) != JOINT_COUNT or not all(math.isfinite(item) for item in values):
            raise ValueError("vector")
        return values

    def _remember(self, request_id: int, canonical: str, outputs: list[str]) -> None:
        """Stores one nonzero replay result with fixed capacity and retention."""
        if request_id == 0:
            return
        self._replay[request_id] = (canonical, list(outputs), self.now_ms)
        self._replay.move_to_end(request_id)
        while len(self._replay) > REPLAY_CAPACITY:
            self._replay.popitem(last=False)

    def _expire_replay(self) -> None:
        """Expires request results older than the published retention period."""
        expired = [request_id for request_id, cached in self._replay.items()
                   if self.now_ms - cached[2] > REPLAY_RETENTION_MS]
        for request_id in expired:
            self._replay.pop(request_id, None)

    def _replace_replay_with_done(self, request_id: int, output: str) -> None:
        """Replaces an accepted response with its terminal result."""
        cached = self._replay.get(request_id)
        if cached is not None:
            self._remember(request_id, cached[0], [output])

    def _complete(self, motion: SimulatedMotion) -> None:
        """Completes one motion and queues its readable terminal output."""
        elapsed_ms = self.now_ms - motion.start_ms
        if motion.namespace == "bench":
            output = (f"done {motion.request_id} bench jog result=completed "
                      f"elapsed_ms={elapsed_ms} arrived={motion.selected_mask:02x}")
        else:
            output = (f"done {motion.request_id} arm move result=completed "
                      f"elapsed_ms={elapsed_ms} max_error_deg=0")
            self.arm_state = "ready"
        self._pending_outputs.append(output)
        self._replace_replay_with_done(motion.request_id, output)

    def _advance_motion(self) -> None:
        """Samples or completes the active linear demonstration motion."""
        motion = self.active_motion
        if motion is None:
            return
        ratio = min(1.0, (self.now_ms - motion.start_ms) / max(1, motion.duration_ms))
        for index in range(JOINT_COUNT):
            distance = motion.target_deg[index] - motion.start_deg[index]
            self.joint_position_deg[index] = motion.start_deg[index] + distance * ratio
            self.joint_velocity_deg_s[index] = (
                0.0 if ratio >= 1.0 else distance / (motion.duration_ms / 1000.0))
        if ratio >= 1.0:
            self.joint_position_deg = list(motion.target_deg)
            self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
            self.active_motion = None
            self._complete(motion)

    def _publish_stream(self) -> None:
        """Queues one due fixed-format latest-value data record."""
        if self.stream_kind == "off" or self.now_ms < self.next_stream_ms:
            return
        interval_ms = max(1, round(1000 / self.stream_rate_hz))
        if self.stream_kind == "joints":
            q_text = ",".join(compact_number(item) for item in self.joint_position_deg)
            qd_text = ",".join(compact_number(item) for item in self.joint_velocity_deg_s)
            output = (f"data 501 joints t_ms={self.now_ms} q={q_text} qd={qd_text} "
                      f"valid=7f state={self.arm_state}")
        else:
            moving_mask = self.active_motion.selected_mask if self.active_motion else 0
            output = (f"data 502 motors present={self.motor_present_mask:02x} "
                      f"enabled={self.motor_enabled_mask:02x} moving={moving_mask:02x} "
                      f"stale=00 fault={self.motor_fault_mask:02x}")
        self._pending_outputs = [item for item in self._pending_outputs
                                 if not item.startswith("data ")]
        self._pending_outputs.append(output)
        self.next_stream_ms = self.now_ms + interval_ms

    def advance(self, milliseconds: int) -> None:
        """Advances logical motion, streaming, replay expiry, and watchdog state."""
        if milliseconds < 0:
            raise ValueError("time cannot move backward")
        self.now_ms += milliseconds
        self._advance_motion()
        self._publish_stream()
        self._expire_replay()
        if (self.motor_enabled_mask and not self.watchdog_reported and
                self.now_ms - self.last_request_ms >= 1000):
            self.watchdog_reported = True
            self.motor_enabled_mask = 0
            self.active_motion = None
            self.arm_state = "fault"
            self._pending_outputs.append(
                "event 90 link_timeout elapsed_ms=1000 action=stop_disable")

    def drain_outputs(self) -> list[str]:
        """Returns and clears asynchronous done, event, and data outputs."""
        outputs = list(self._pending_outputs)
        self._pending_outputs.clear()
        return outputs

    def _show(self, request_id: int, target: str, positionals: list[str]) -> list[str]:
        """Formats one public query without changing simulated hardware state."""
        if target == "info":
            return [f"ok {request_id} show info mcu=STM32H723VGT6 transport=usb_cdc "
                    "can=classic_1m motor=s3519 driver=dm3520 build=sim"]
        if target == "state":
            return [f"ok {request_id} show state state={self.arm_state} "
                    f"aligned={int(self.aligned)} enabled={self.motor_enabled_mask:02x} "
                    f"moving={int(self.active_motion is not None)} active="
                    f"{self.active_motion.request_id if self.active_motion else 0} "
                    f"fault={'driver' if self.motor_fault_mask else 'none'}"]
        if target == "joints":
            q_text = ",".join(compact_number(item) for item in self.joint_position_deg)
            qd_text = ",".join(compact_number(item) for item in self.joint_velocity_deg_s)
            return [f"ok {request_id} show joints t_ms={self.now_ms} q={q_text} "
                    f"qd={qd_text} valid=7f"]
        if target == "motors":
            holding_mask = self.motor_enabled_mask
            moving_mask = self.active_motion.selected_mask if self.active_motion else 0
            holding_mask &= ~moving_mask
            return [f"ok {request_id} show motors present={self.motor_present_mask:02x} "
                    f"enabled={self.motor_enabled_mask:02x} moving={moving_mask:02x} "
                    f"holding={holding_mask:02x} stale=00 fault={self.motor_fault_mask:02x}"]
        if target == "config":
            return [f"ok {request_id} show config map=sim required=ff verified=7f "
                    "enable_ready=1"]
        if target == "diag":
            return [f"ok {request_id} show diag loop_max_us=4000 deadline_miss=0 "
                    "can_error=0 usb_drop=0 fault=none"]
        raise ValueError("show")

    def _bench(self, request_id: int, operation: str, positionals: list[str],
               fields: dict[str, str]) -> list[str]:
        """Executes one explicitly scoped low-energy simulated bench command."""
        if self.profile != "bench":
            return [f"error {request_id} bench {operation} code=profile "
                    "current=arm required=bench"]
        motors, mask = self._parse_motor_list(positionals[0])
        accepted = f"ok {request_id} bench {operation} accepted=1"
        if operation == "init":
            done = (f"done {request_id} bench init result=completed identity={mask:02x} "
                    f"mode={mask:02x} ranges={mask:02x} version={mask:02x}")
        elif operation == "enable":
            self.motor_enabled_mask |= mask
            done = f"done {request_id} bench enable result=completed enabled={mask:02x}"
        elif operation == "jog":
            delta = float(fields["delta"])
            speed = float(fields["speed"])
            if not math.isfinite(delta) or abs(delta) > 3.0:
                return [f"error {request_id} bench jog code=out_of_range field=delta"]
            if not math.isfinite(speed) or not 0.0 < speed <= 3.0:
                return [f"error {request_id} bench jog code=out_of_range field=speed"]
            target = list(self.joint_position_deg)
            for motor in motors:
                target[motor - 1] += delta
            duration_ms = max(4, math.ceil(abs(delta) / speed * 1000))
            self.active_motion = SimulatedMotion(request_id, "bench", mask,
                                                 self.now_ms, duration_ms,
                                                 list(self.joint_position_deg), target)
            return [accepted]
        elif operation == "stop":
            self.active_motion = None
            self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
            done = f"done {request_id} bench stop result=stopped enabled={mask:02x}"
        elif operation == "disable":
            self.motor_enabled_mask &= ~mask
            done = f"done {request_id} bench disable result=completed enabled=00"
        elif operation == "clear":
            self.motor_fault_mask &= ~mask
            done = f"done {request_id} bench clear result=completed fault=00"
        else:
            return [f"error {request_id} bench {operation} code=unknown_command"]
        self._pending_outputs.append(done)
        return [accepted]

    def _dispatch(self, request_id: int, command: list[str],
                  positionals: list[str], fields: dict[str, str]) -> list[str]:
        """Dispatches one parsed command into deterministic simulator behavior."""
        path = " ".join(command)
        if path == "hello":
            self.stream_kind = "off"
            self.stream_rate_hz = 0
            return [f"ok {request_id} hello protocol=aethor-text-v1 fw=sim-1 "
                    f"profile={self.profile} dof=7 boot={self.boot_id} watchdog_ms=1000"]
        if path == "ping":
            return [f"ok {request_id} ping state={self.arm_state} "
                    f"enabled={self.motor_enabled_mask:02x} boot={self.boot_id}"]
        if path == "help":
            return [f"ok {request_id} help topics=show,stream,arm,bench "
                    "examples=show_state,arm_move,bench_jog"]
        if command[0] == "show":
            return self._show(request_id, command[1], positionals)
        if command[0] == "stream":
            if command[1] == "off":
                self.stream_kind = "off"
                self.stream_rate_hz = 0
                return [f"ok {request_id} stream off"]
            rate_hz = int(fields["rate"])
            maximum_rate = 50 if command[1] == "joints" else 10
            if not 1 <= rate_hz <= maximum_rate:
                return [f"error {request_id} {path} code=out_of_range field=rate"]
            self.stream_kind = command[1]
            self.stream_rate_hz = rate_hz
            self.next_stream_ms = self.now_ms
            return [f"ok {request_id} {path} rate={rate_hz}"]
        if command[0] == "bench":
            return self._bench(request_id, command[1], positionals, fields)
        if command[0] == "arm" and self.profile != "arm":
            return [f"error {request_id} {path} code=profile current=bench required=arm"]
        return [f"error {request_id} {path} code=unknown_command"]

    def process_line(self, line: bytes | str) -> list[str]:
        """Parses and processes one optional-ID plain text request line."""
        try:
            text = (bytes(line).decode("ascii")
                    if isinstance(line, (bytes, bytearray)) else line).rstrip("\r\n")
        except UnicodeError:
            return ["error 0 parse code=bad_line"]
        if not text or len(text) > 160 or any(ord(item) < 0x20 or ord(item) > 0x7E
                                              for item in text):
            return ["error 0 parse code=line_too_long" if len(text) > 160
                    else "error 0 parse code=bad_line"]
        tokens = text.split()
        request_id = int(tokens.pop(0)) if tokens[0].isdigit() else 0
        if not tokens:
            return ["error 0 parse code=bad_line"]
        single_word = tokens[0].lower() in {"hello", "ping", "help"}
        command = [tokens.pop(0).lower()]
        if not single_word and tokens and "=" not in tokens[0]:
            command.append(tokens.pop(0).lower())
        positionals: list[str] = []
        fields: dict[str, str] = {}
        try:
            for token in tokens:
                if "=" in token:
                    key, value = token.split("=", 1)
                    key = key.lower()
                    if not key or not value or key in fields:
                        raise ValueError("field")
                    fields[key] = value
                elif fields:
                    raise ValueError("positional order")
                else:
                    positionals.append(token)
            canonical_parts = command + positionals + [f"{key}={value}"
                                                        for key, value in fields.items()]
            canonical = " ".join(canonical_parts)
            cached = self._replay.get(request_id) if request_id else None
            if cached is not None:
                if cached[0] != canonical:
                    return [f"error {request_id} {' '.join(command)} "
                            "code=request_conflict"]
                return list(cached[1])
            outputs = self._dispatch(request_id, command, positionals, fields)
        except (IndexError, KeyError, ValueError):
            return [f"error {request_id} {' '.join(command)} code=bad_argument"]
        self._remember(request_id, canonical, outputs)
        self.last_request_ms = self.now_ms
        self.watchdog_reported = False
        return outputs

    def feed_bytes(self, data: bytes) -> list[str]:
        """Feeds fragmented or concatenated transport bytes into the line buffer."""
        self._rx_buffer.extend(data)
        outputs: list[str] = []
        while b"\n" in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            outputs.extend(self.process_line(raw_line + b"\n"))
        return outputs
