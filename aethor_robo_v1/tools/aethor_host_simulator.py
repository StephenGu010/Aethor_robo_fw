"""Deterministic, dependency-free simulator for the aethor-arm-ascii-v1 contract."""

from __future__ import annotations

import math
import sys
import zlib
from collections import OrderedDict
from dataclasses import dataclass

JOINT_COUNT = 7
ALL_MOTORS_MASK = 0x7F
REPLAY_CAPACITY = 32
REPLAY_RETENTION_MS = 60_000


def crc16_ccitt_false(data: bytes) -> int:
    """Calculates CRC-16/CCITT-FALSE using the firmware polynomial and seed."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(body: str) -> bytes:
    """Encodes one ASCII protocol body with uppercase CRC and LF terminator."""
    encoded = body.encode("ascii")
    return encoded + f" *{crc16_ccitt_false(encoded):04X}\n".encode("ascii")


@dataclass
class MotionAction:
    """Stores one bounded synchronized simulated motion."""

    request_id: int
    start_ms: int
    duration_ms: int
    start_deg: list[float]
    target_deg: list[float]
    mode: str


class AethorHostSimulator:
    """Emulates seven motor objects, sessions, telemetry, motion, and faults."""

    def __init__(self, boot_id: int = 1) -> None:
        """Initializes one safe unaligned simulator boot."""
        self.boot_id = boot_id or 1
        self.now_ms = 0
        self.session_id = 0
        self.arm_state = "UNALIGNED"
        self.control_mode = "UNKNOWN"
        self.aligned = False
        self.motor_present = [True] * JOINT_COUNT
        self.motor_enabled = [False] * JOINT_COUNT
        self.motor_fault = ["NONE"] * JOINT_COUNT
        self.joint_position_deg = [0.0] * JOINT_COUNT
        self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
        self.stream_rate_hz = 50
        self.stream_fields = "jpos,jvel,state,motor"
        self.next_telemetry_ms = 0
        self.telemetry_sequence = 0
        self.event_sequence = 0
        self.last_request_ms = 0
        self.watchdog_reported = False
        self.active_motion: MotionAction | None = None
        self._pending_outputs: list[str] = []
        self._rx_buffer = bytearray()
        self._replay: OrderedDict[int, tuple[str, list[str], int]] = OrderedDict()

    def _expire_replay_entries(self) -> None:
        """Drops expired replay results while retaining at most the newest 32 entries."""
        expired_ids = [request_id for request_id, cached in self._replay.items()
                       if self.now_ms - cached[2] >= REPLAY_RETENTION_MS]
        for request_id in expired_ids:
            self._replay.pop(request_id, None)
        while len(self._replay) > REPLAY_CAPACITY:
            self._replay.popitem(last=False)

    def _remember_replay(self, request_id: int, body: str, outputs: list[str]) -> None:
        """Stores one bounded replay result using the firmware capacity and retention."""
        self._replay[request_id] = (body, list(outputs), self.now_ms)
        self._replay.move_to_end(request_id)
        self._expire_replay_entries()

    def _emit(self, body: str, *, replace_telemetry: bool = False) -> None:
        """Queues one output, replacing the previous unsent telemetry when requested."""
        if replace_telemetry:
            self._pending_outputs = [item for item in self._pending_outputs
                                     if not item.startswith("TEL ")]
        self._pending_outputs.append(body)

    def drain_outputs(self) -> list[str]:
        """Returns and clears asynchronous DONE, EVT, and TEL outputs."""
        outputs = list(self._pending_outputs)
        self._pending_outputs.clear()
        return outputs

    @staticmethod
    def _parse_vector(value: str, expected_count: int = JOINT_COUNT) -> list[float]:
        """Parses an exact finite comma vector or raises ValueError."""
        values = [float(item) for item in value.split(",")]
        if len(values) != expected_count or not all(math.isfinite(item) for item in values):
            raise ValueError("invalid vector")
        return values

    @staticmethod
    def _parse_motors(value: str) -> tuple[list[int], int]:
        """Parses a unique ascending one-based motor list."""
        motors = [int(item) for item in value.split(",")]
        if not motors or motors != sorted(set(motors)) or any(item < 1 or item > 7 for item in motors):
            raise ValueError("invalid motors")
        mask = sum(1 << (item - 1) for item in motors)
        return motors, mask

    def _format_state(self, request_id: int) -> str:
        """Formats the public simulated arm state."""
        return (f"RSP {request_id} ok state={self.arm_state} aligned={int(self.aligned)} "
                f"enabled={int(all(self.motor_enabled))} moving={int(self.active_motion is not None)} "
                f"mode={self.control_mode} fault={int(any(item != 'NONE' for item in self.motor_fault))}")

    def _complete(self, request_id: int, result: str = "COMPLETED", detail: int = 0) -> None:
        """Queues one terminal command result and updates its replay entry."""
        body = f"DONE {request_id} {result} detail={detail}"
        self._emit(body)
        cached = self._replay.get(request_id)
        if cached is not None:
            self._remember_replay(request_id, cached[0], [body])

    def _set_fault(self, event: str, detail: str) -> None:
        """Cancels motion, disables every motor, and emits a fault event."""
        if self.active_motion is not None:
            self._complete(self.active_motion.request_id, "FAILED", 1)
            self.active_motion = None
        self.motor_enabled = [False] * JOINT_COUNT
        self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
        self.arm_state = "FAULT"
        self.event_sequence += 1
        self._emit(f"EVT {self.event_sequence} {event} detail={detail}")

    def force_ready(self, mode: str = "POS_VEL") -> None:
        """Places the simulator in a commissioned READY state for fault tests."""
        if mode not in {"POS_VEL", "MIT"}:
            raise ValueError("invalid mode")
        self.aligned = True
        self.control_mode = mode
        self.motor_enabled = [True] * JOINT_COUNT
        self.arm_state = "READY"
        self.last_request_ms = self.now_ms
        self.watchdog_reported = False

    def inject_fault(self, joint: int, fault: str) -> None:
        """Injects one driver fault and applies all-axis fail-safe disable."""
        if joint < 1 or joint > JOINT_COUNT:
            raise ValueError("joint out of range")
        self.motor_fault[joint - 1] = fault.upper()
        self._set_fault("DRIVER_FAULT", f"joint={joint},fault={fault}")

    def clear_injected_faults(self) -> None:
        """Clears simulator-only fault sources while retaining explicit state control."""
        self.motor_fault = ["NONE"] * JOINT_COUNT

    def _advance_motion(self) -> None:
        """Samples or completes the active synchronized motion at current time."""
        action = self.active_motion
        if action is None:
            return
        elapsed_ms = max(0, self.now_ms - action.start_ms)
        ratio = min(1.0, elapsed_ms / max(1, action.duration_ms))
        if action.mode == "MIT":
            scale = 10 * ratio**3 - 15 * ratio**4 + 6 * ratio**5
        else:
            scale = ratio
        for index in range(JOINT_COUNT):
            distance = action.target_deg[index] - action.start_deg[index]
            self.joint_position_deg[index] = action.start_deg[index] + distance * scale
            self.joint_velocity_deg_s[index] = (0.0 if ratio >= 1.0 else
                                                distance / (action.duration_ms / 1000.0))
        if ratio >= 1.0:
            self.joint_position_deg = list(action.target_deg)
            self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
            self.arm_state = "READY"
            self.active_motion = None
            self._complete(action.request_id)

    def _publish_telemetry(self) -> None:
        """Publishes at most the newest due telemetry sample."""
        if self.session_id == 0 or self.stream_rate_hz == 0:
            return
        interval_ms = max(10, round(1000 / self.stream_rate_hz))
        if self.now_ms < self.next_telemetry_ms:
            return
        self.telemetry_sequence += 1
        q_text = ",".join(f"{value:.3f}" for value in self.joint_position_deg)
        v_text = ",".join(f"{value:.3f}" for value in self.joint_velocity_deg_s)
        self._emit(f"TEL {self.telemetry_sequence} JOINT_STATE timestamp_ms={self.now_ms} "
                   f"q_deg={q_text} qd_deg_s={v_text} arm_state={self.arm_state}",
                   replace_telemetry=True)
        self.next_telemetry_ms = self.now_ms + interval_ms

    def advance(self, milliseconds: int) -> None:
        """Advances deterministic simulated time, motion, telemetry, and watchdog."""
        if milliseconds < 0:
            raise ValueError("time cannot move backward")
        self.now_ms += milliseconds
        self._advance_motion()
        self._publish_telemetry()
        if (self.session_id and not self.watchdog_reported and
                self.now_ms - self.last_request_ms >= 1000):
            self.watchdog_reported = True
            self._set_fault("LINK_TIMEOUT", "elapsed_ms=1000,action=STOP_DISABLE")

    def process_line(self, line: bytes | str) -> list[str]:
        """Processes one complete CRC-protected request and returns immediate outputs."""
        raw = line.encode("ascii") if isinstance(line, str) else line
        stripped = raw.rstrip(b"\r\n")
        request_id = 0
        try:
            body_bytes, crc_text = stripped.rsplit(b" *", 1)
            body = body_bytes.decode("ascii")
            tokens = body.split(" ")
            if len(tokens) < 3 or "" in tokens or tokens[0] != "REQ":
                raise ValueError("bad frame")
            request_id = int(tokens[1])
            if int(crc_text, 16) != crc16_ccitt_false(body_bytes):
                return [f"ERR {request_id} BAD_CRC"]
        except (ValueError, UnicodeError):
            return [f"ERR {request_id} BAD_FRAME"]

        cached = self._replay.get(request_id)
        if cached is not None:
            return list(cached[1]) if cached[0] == body else [f"ERR {request_id} REQUEST_ID_CONFLICT"]

        operation = tokens[2]
        fields: dict[str, str] = {}
        try:
            for token in tokens[3:]:
                key, value = token.split("=", 1)
                if not key or not value or key in fields:
                    raise ValueError("bad field")
                fields[key] = value
            outputs = self._dispatch(request_id, operation, fields)
        except (ValueError, KeyError):
            outputs = [f"ERR {request_id} BAD_VALUE"]
        self._remember_replay(request_id, body, outputs)
        if operation != "HELLO" and self.session_id:
            self.last_request_ms = self.now_ms
            self.watchdog_reported = False
        return outputs

    def _dispatch(self, request_id: int, operation: str, fields: dict[str, str]) -> list[str]:
        """Dispatches one already framed request into deterministic simulator behavior."""
        if operation == "HELLO":
            if fields.get("protocol") != "1":
                raise ValueError("protocol")
            self.session_id += 1
            self.last_request_ms = self.now_ms
            self.watchdog_reported = False
            return [f"RSP {request_id} ok boot_id={self.boot_id} session={self.session_id} "
                    "controller_id=ctrboard-h7 arm_id=arm-1 capabilities=formal,bench,sim"]
        if self.session_id == 0:
            return [f"ERR {request_id} INVALID_SESSION"]
        if operation == "HEARTBEAT":
            return [f"RSP {request_id} ok timestamp_ms={self.now_ms}"]
        if operation == "GET_INFO":
            return [f"RSP {request_id} ok product=Aethor_robo firmware=sim-1 "
                    "protocol=aethor-arm-ascii-v1"]
        if operation == "GET_CONFIG":
            canonical = b"ids=1:17,2:18,3:19,4:20,5:21,6:22,7:23;verified=sim"
            return [f"RSP {request_id} ok joints=7 verified=sim map_hash="
                    f"{zlib.crc32(canonical) & 0xFFFFFFFF:08X}"]
        if operation == "GET_STATE":
            return [self._format_state(request_id)]
        if operation == "GET_JPOS":
            values = ",".join(f"{item:.3f}" for item in self.joint_position_deg)
            return [f"RSP {request_id} ok q_deg={values} aligned={int(self.aligned)}"]
        if operation == "GET_MOTORS":
            status = ",".join("1" if enabled else "0" for enabled in self.motor_enabled)
            return [f"RSP {request_id} ok present=1,1,1,1,1,1,1 status={status}"]
        if operation == "GET_DIAG":
            return [f"RSP {request_id} ok sim_time_ms={self.now_ms} event_ov=0"]
        if operation == "SET_STREAM":
            rate = int(fields["rate_hz"])
            if rate < 0 or rate > 100:
                raise ValueError("rate")
            self.stream_rate_hz = rate
            self.stream_fields = fields.get("fields", self.stream_fields)
            self.next_telemetry_ms = self.now_ms
            return [f"RSP {request_id} ok rate_hz={rate} fields={self.stream_fields}"]
        if operation == "ALIGN_REFERENCE":
            target = self._parse_vector(fields["q_ref_deg"])
            self.joint_position_deg = target
            self.aligned = True
            self.arm_state = "DISABLED"
            self._complete(request_id)
            return [f"ACK {request_id} accepted"]
        if operation == "SET_MODE":
            mode = fields["mode"]
            if mode not in {"POS_VEL", "MIT"} or self.arm_state != "DISABLED":
                raise ValueError("mode")
            self.control_mode = mode
            self._complete(request_id)
            return [f"ACK {request_id} accepted"]
        if operation == "ENABLE" and "motors" not in fields:
            if not self.aligned or self.control_mode == "UNKNOWN" or any(item != "NONE" for item in self.motor_fault):
                return [f"ERR {request_id} NOT_READY"]
            self.motor_enabled = [True] * JOINT_COUNT
            self.arm_state = "READY"
            self._complete(request_id)
            return [f"ACK {request_id} accepted"]
        if operation == "MOVE_JOINTS":
            if self.arm_state != "READY" or self.active_motion is not None:
                return [f"ERR {request_id} BUSY"]
            target = self._parse_vector(fields["q_deg"])
            speed = float(fields.get("speed", "0.2"))
            mode = fields.get("mode", self.control_mode)
            if not 0.01 <= speed <= 1.0 or mode != self.control_mode:
                raise ValueError("motion")
            duration_ms = max(4, math.ceil(max(abs(a - b) for a, b in
                                               zip(target, self.joint_position_deg)) /
                                           (30.0 * speed) * 1000))
            self.active_motion = MotionAction(request_id, self.now_ms, duration_ms,
                                              list(self.joint_position_deg), target, mode)
            self.arm_state = "MOVING"
            return [f"ACK {request_id} accepted motion_id={request_id} "
                    f"duration_ms={duration_ms} mode={mode}"]
        if operation == "STOP" and "motors" not in fields:
            if self.active_motion is not None:
                self._complete(self.active_motion.request_id, "CANCELLED")
                self.active_motion = None
            self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
            self.arm_state = "READY" if all(self.motor_enabled) else "DISABLED"
            self._complete(request_id, "STOPPED")
            return [f"ACK {request_id} accepted"]
        if operation == "DISABLE" and "motors" not in fields:
            self.motor_enabled = [False] * JOINT_COUNT
            self.arm_state = "DISABLED" if self.aligned else "UNALIGNED"
            self._complete(request_id)
            return [f"ACK {request_id} accepted"]
        if operation == "CLEAR_FAULT" and "motors" not in fields:
            if any(item != "NONE" for item in self.motor_fault):
                return [f"ERR {request_id} FAULT_SOURCE_ACTIVE"]
            self.arm_state = "DISABLED" if self.aligned else "UNALIGNED"
            self._complete(request_id)
            return [f"ACK {request_id} accepted"]
        if operation == "INIT_MOTORS":
            motors, mask = self._parse_motors(fields["motors"])
            self._complete(request_id)
            return [f"ACK {request_id} accepted motors={mask}"]
        if operation in {"ENABLE", "STOP", "DISABLE", "CLEAR_FAULT"}:
            motors, mask = self._parse_motors(fields["motors"])
            for motor in motors:
                if operation == "ENABLE":
                    self.motor_enabled[motor - 1] = True
                elif operation in {"STOP", "DISABLE"}:
                    self.motor_enabled[motor - 1] = operation == "STOP"
                elif operation == "CLEAR_FAULT":
                    self.motor_fault[motor - 1] = "NONE"
            self._complete(request_id, "STOPPED" if operation == "STOP" else "COMPLETED")
            return [f"ACK {request_id} accepted motors={mask}"]
        if operation == "MOVE_REL":
            motors, mask = self._parse_motors(fields["motors"])
            deltas = self._parse_vector(fields["delta_deg"], len(motors))
            speeds = self._parse_vector(fields["speed_deg_s"], len(motors))
            if any(abs(item) > 3.0 for item in deltas) or any(not 0 < item <= 3.0 for item in speeds):
                raise ValueError("relative")
            for motor, delta in zip(motors, deltas):
                self.joint_position_deg[motor - 1] += delta
            self._complete(request_id)
            return [f"ACK {request_id} accepted motors={mask}"]
        return [f"ERR {request_id} UNKNOWN_OPERATION"]

    def feed_bytes(self, data: bytes) -> list[str]:
        """Feeds fragmented or concatenated transport bytes into the line accumulator."""
        self._rx_buffer.extend(data)
        outputs: list[str] = []
        while b"\n" in self._rx_buffer:
            line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            outputs.extend(self.process_line(line + b"\n"))
        if len(self._rx_buffer) > 1024:
            self._rx_buffer.clear()
            outputs.append("ERR 0 RX_OVERFLOW")
        return outputs


def main() -> int:
    """Runs a line-oriented stdin/stdout simulator for scripts and manual use."""
    simulator = AethorHostSimulator()
    for raw_line in sys.stdin.buffer:
        if raw_line.startswith(b"@ADVANCE "):
            simulator.advance(int(raw_line.split()[1]))
            outputs = simulator.drain_outputs()
        else:
            outputs = simulator.feed_bytes(raw_line)
            outputs.extend(simulator.drain_outputs())
        for output in outputs:
            sys.stdout.buffer.write(encode_frame(output))
        sys.stdout.buffer.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
