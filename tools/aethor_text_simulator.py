"""Deterministic dependency-free simulator for the aethor-text-v1 contract."""

from __future__ import annotations

import math
import struct
from collections import OrderedDict
from dataclasses import dataclass

JOINT_COUNT = 7
ALL_MOTORS_MASK = 0x7F
REPLAY_CAPACITY = 32
REPLAY_RETENTION_MS = 60_000
AETHOR_TEXT_MAX_REQUEST_BODY_BYTES = 160
AETHOR_TEXT_MAX_FLOAT_TOKEN_CHARACTERS = 63
AETHOR_TEXT_MAX_COMMAND_WORDS = 2
AETHOR_TEXT_MAX_POSITIONAL_COUNT = 2
AETHOR_TEXT_MAX_FIELD_COUNT = 8
AETHOR_TEXT_MAX_TOKEN_COUNT = (1 + AETHOR_TEXT_MAX_COMMAND_WORDS +
                               AETHOR_TEXT_MAX_POSITIONAL_COUNT +
                               AETHOR_TEXT_MAX_FIELD_COUNT)
FLOAT32_MIN_NORMAL = 1.1754943508222875e-38
FLOAT32_MAX = 3.4028234663852886e38
SIMULATED_PMAX_DEG = 180.0
SIMULATED_VMAX_DEG_S = 360.0
SIMULATED_MAX_SPEED_DEG_S = 360.0
SIMULATED_TMAX_NM = 8.0
SIMULATED_MOVE_SPEED_LIMIT_DEG_S = min(
    SIMULATED_VMAX_DEG_S, SIMULATED_MAX_SPEED_DEG_S)


def parse_strict_float32_token(token: str) -> tuple[str, float]:
    """Parses one decimal token with the firmware's 64-byte float buffer rules."""
    if not token or len(token) > AETHOR_TEXT_MAX_FLOAT_TOKEN_CHARACTERS:
        return "bad_argument", 0.0
    index = 1 if token[0] in "+-" else 0
    digit_seen = False
    decimal_point_seen = False
    for character in token[index:]:
        if "0" <= character <= "9":
            digit_seen = True
        elif character == "." and not decimal_point_seen:
            decimal_point_seen = True
        else:
            return "bad_argument", 0.0
    if not digit_seen:
        return "bad_argument", 0.0
    try:
        numeric_value = float(token)
        if not math.isfinite(numeric_value):
            return "bad_argument", 0.0
        float32_value = struct.unpack("!f", struct.pack("!f", numeric_value))[0]
    except (OverflowError, TypeError, ValueError, struct.error):
        return "bad_argument", 0.0
    if (not math.isfinite(float32_value) or
            (numeric_value != 0.0 and float32_value == 0.0) or
            (float32_value != 0.0 and
             (abs(float32_value) < FLOAT32_MIN_NORMAL or
              abs(float32_value) > FLOAT32_MAX))):
        return "bad_argument", 0.0
    return "ok", float32_value


def parse_strict_ascii_u32_token(token: str) -> int:
    """Parses one unsigned 32-bit token using the firmware's ASCII grammar."""
    if (not token or
            any(character < "0" or character > "9" for character in token)):
        raise ValueError("u32")
    numeric_value = int(token, 10)
    if numeric_value > 0xFFFFFFFF:
        raise ValueError("u32")
    return numeric_value


def is_text_identifier(token: str) -> bool:
    """Checks the firmware's ASCII letter-led command and field-key grammar."""
    if not token or not (("A" <= token[0] <= "Z") or
                         ("a" <= token[0] <= "z")):
        return False
    return all(("A" <= character <= "Z") or
               ("a" <= character <= "z") or
               ("0" <= character <= "9") or character == "_"
               for character in token[1:])


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
        self._replay: OrderedDict[
            int, tuple[str, list[str], int, bool]] = OrderedDict()

    @staticmethod
    def _parse_motor_list(value: str,
                          require_ascending: bool) -> tuple[list[int], int]:
        """Parses one unique motor list with optional legacy ascending order."""
        motor_tokens = value.split(",")
        if any(not token or any(character < "0" or character > "9"
                                for character in token)
               for token in motor_tokens):
            raise ValueError("motors")
        motors = [int(token, 10) for token in motor_tokens]
        if (not motors or len(set(motors)) != len(motors) or
                any(item < 1 or item > JOINT_COUNT for item in motors) or
                (require_ascending and
                 any(current <= previous
                     for previous, current in zip(motors, motors[1:])))):
            raise ValueError("motors")
        return motors, sum(1 << (item - 1) for item in motors)

    @staticmethod
    def _parse_move_values(value: str, expected_count: int,
                           allow_zero: bool) -> tuple[str, list[float]]:
        """Parses one selected value list using firmware float32 admission rules."""
        tokens = value.split(",")
        if len(tokens) != expected_count:
            return "count_mismatch", []
        values: list[float] = []
        for token in tokens:
            parse_status, numeric_value = parse_strict_float32_token(token)
            if parse_status != "ok":
                return parse_status, []
            if numeric_value == 0.0:
                if allow_zero:
                    values.append(0.0)
                    continue
                return "out_of_range", []
            if not allow_zero and numeric_value < 0.0:
                return "out_of_range", []
            values.append(numeric_value)
        return "ok", values

    @staticmethod
    def _parse_vector(value: str) -> list[float]:
        """Parses exactly seven finite comma-separated values."""
        values = [float(item) for item in value.split(",")]
        if len(values) != JOINT_COUNT or not all(math.isfinite(item) for item in values):
            raise ValueError("vector")
        return values

    def _remember(self, request_id: int, canonical: str, outputs: list[str],
                  watchdog_valid: bool) -> None:
        """Stores one replay result and its original watchdog admission state."""
        if request_id == 0:
            return
        self._replay[request_id] = (
            canonical, list(outputs), self.now_ms, watchdog_valid)
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
            self._remember(request_id, cached[0], [output], cached[3])

    def _complete(self, motion: SimulatedMotion) -> None:
        """Completes one motion and queues its readable terminal output."""
        elapsed_ms = self.now_ms - motion.start_ms
        if motion.namespace == "bench":
            output = (f"done {motion.request_id} bench jog result=completed "
                      f"elapsed_ms={elapsed_ms} arrived={motion.selected_mask:02x}")
        elif motion.namespace == "bench_move":
            self.motor_enabled_mask &= ~motion.selected_mask
            output = (f"done {motion.request_id} bench move result=completed "
                      f"elapsed_ms={elapsed_ms} motors={motion.selected_mask:02x}")
        elif motion.namespace == "bench_mit":
            self.motor_enabled_mask &= ~motion.selected_mask
            output = (f"done {motion.request_id} bench mit result=completed "
                      f"elapsed_ms={elapsed_ms} motors={motion.selected_mask:02x}")
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
        self_contained_move_active = (
            self.active_motion is not None and
            self.active_motion.namespace in {"bench_move", "bench_mit"})
        if (self.motor_enabled_mask and not self_contained_move_active and
                not self.watchdog_reported and
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
        for output in outputs:
            tokens = output.split(maxsplit=3)
            if (len(tokens) >= 2 and tokens[0] == "done" and
                    tokens[1].isascii() and tokens[1].isdigit()):
                self._replace_replay_with_done(int(tokens[1], 10), output)
        return outputs

    def _show(self, request_id: int, target: str, positionals: list[str],
              fields: dict[str, str]) -> tuple[list[str], bool]:
        """Formats one public query and reports whether C accepts it as valid."""
        if target not in {"info", "state", "joints", "motors", "motor",
                          "config", "diag"}:
            return [f"error {request_id} show {target} code=unknown_command"], False
        if fields:
            return [f"error {request_id} show {target} code=bad_argument"], False
        if target == "motor":
            if len(positionals) != 1:
                return [f"error {request_id} show motor code=bad_argument"], False
            try:
                joint_number = parse_strict_ascii_u32_token(positionals[0])
            except ValueError:
                return [f"error {request_id} show motor code=bad_argument"], False
            if not 1 <= joint_number <= JOINT_COUNT:
                return [f"error {request_id} show motor "
                        "code=out_of_range field=joint"], False
            joint_index = joint_number - 1
            joint_bit = 1 << joint_index
            moving_mask = (self.active_motion.selected_mask
                           if self.active_motion is not None else 0)
            if self.motor_fault_mask & joint_bit:
                motor_state = "fault"
            elif self.motor_enabled_mask & joint_bit:
                motor_state = "moving" if moving_mask & joint_bit else "holding"
            else:
                motor_state = "disabled"
            response = (
                f"ok {request_id} show motor joint={joint_number} "
                f"esc={joint_number:02x} master={joint_number + 0x10:02x} "
                f"state={motor_state} "
                f"pos_deg={compact_number(self.joint_position_deg[joint_index])} "
                f"speed_deg_s={compact_number(self.joint_velocity_deg_s[joint_index])} "
                "torque_nm=0 mos_c=0 rotor_c=0 "
                f"fault={int(bool(self.motor_fault_mask & joint_bit))} age_ms=0 "
                f"pmax_deg={compact_number(SIMULATED_PMAX_DEG)} "
                f"vmax_deg_s={compact_number(SIMULATED_VMAX_DEG_S)} "
                f"max_speed_deg_s={compact_number(SIMULATED_MAX_SPEED_DEG_S)} "
                "move_speed_limit_deg_s="
                f"{compact_number(SIMULATED_MOVE_SPEED_LIMIT_DEG_S)}")
            return [response], True
        if target == "diag":
            if len(positionals) > 1:
                return [f"error {request_id} show diag code=bad_argument"], False
        elif target == "config":
            if len(positionals) > 1:
                return [f"error {request_id} show config code=bad_argument"], False
            if positionals:
                try:
                    joint_number = parse_strict_ascii_u32_token(positionals[0])
                except ValueError:
                    return ([f"error {request_id} show config "
                             "code=bad_argument"], False)
                if not 1 <= joint_number <= JOINT_COUNT:
                    return ([f"error {request_id} show config "
                             "code=out_of_range field=joint"], False)
        elif positionals:
            return [f"error {request_id} show {target} code=bad_argument"], False
        if target == "info":
            return ([f"ok {request_id} show info mcu=STM32H723VGT6 transport=usb_cdc "
                     "can=classic_1m motor=s3519 driver=dm3520 build=sim"], True)
        if target == "state":
            return ([f"ok {request_id} show state state={self.arm_state} "
                     f"aligned={int(self.aligned)} enabled={self.motor_enabled_mask:02x} "
                     f"moving={int(self.active_motion is not None)} active="
                     f"{self.active_motion.request_id if self.active_motion else 0} "
                     f"fault={'driver' if self.motor_fault_mask else 'none'}"], True)
        if target == "joints":
            q_text = ",".join(compact_number(item) for item in self.joint_position_deg)
            qd_text = ",".join(compact_number(item) for item in self.joint_velocity_deg_s)
            return ([f"ok {request_id} show joints t_ms={self.now_ms} q={q_text} "
                     f"qd={qd_text} valid=7f"], True)
        if target == "motors":
            holding_mask = self.motor_enabled_mask
            moving_mask = self.active_motion.selected_mask if self.active_motion else 0
            holding_mask &= ~moving_mask
            return ([f"ok {request_id} show motors present={self.motor_present_mask:02x} "
                     f"enabled={self.motor_enabled_mask:02x} moving={moving_mask:02x} "
                     f"holding={holding_mask:02x} stale=00 "
                     f"fault={self.motor_fault_mask:02x}"], True)
        if target == "config":
            return ([f"ok {request_id} show config map=sim required=ff verified=7f "
                     "enable_ready=1"], True)
        if target == "diag":
            if positionals:
                section = positionals[0]
                if section == "can":
                    return ([f"ok {request_id} show diag can rx=0 tx=0 drop=0 "
                             "error=0 busoff=0 queue_hwm=0"], True)
                if section == "motion":
                    active_request_id = (self.active_motion.request_id
                                         if self.active_motion else 0)
                    predicted_ms = (self.active_motion.duration_ms
                                    if self.active_motion else 0)
                    return ([f"ok {request_id} show diag motion "
                             f"active={active_request_id} "
                             f"predicted_ms={predicted_ms} elapsed_ms=0 "
                             "max_error_deg=0.000"], True)
                if section == "usb":
                    return ([f"ok {request_id} show diag usb rx_lines=? "
                             "bad_lines=0 rx_overflow=0 tx_drop_data=0 "
                             "busy_max_ms=?"], True)
                if section == "rtos":
                    return ([f"ok {request_id} show diag rtos control_stack=? "
                             "can_stack=? protocol_stack=? usb_stack=? "
                             "heap_min=0 telemetry_stack=? diag_stack=? ui_stack=? "
                             "stack_unit=words"], True)
                return [f"error {request_id} show diag code=bad_argument"], False
            return ([f"ok {request_id} show diag loop_max_us=4000 deadline_miss=0 "
                     "can_error=0 usb_drop=0 fault=none"], True)
        return [f"error {request_id} show {target} code=unknown_command"], False

    def _bench(self, request_id: int, operation: str, positionals: list[str],
               fields: dict[str, str]) -> tuple[list[str], bool]:
        """Executes one bench command and reports C watchdog acceptance."""
        if self.profile != "bench":
            return ([f"error {request_id} bench {operation} code=profile "
                     "current=arm required=bench"], False)
        if operation not in {"init", "mode", "mit", "enable", "jog", "move",
                             "stop", "disable", "clear"}:
            return ([f"error {request_id} bench {operation} "
                     "code=unknown_command"], False)
        if operation in {"move", "mode", "mit"} and request_id == 0:
            return ([f"error 0 bench {operation} "
                     "code=bad_argument field=request_id"], False)
        try:
            if len(positionals) != 1:
                raise ValueError("motors")
            motors, mask = self._parse_motor_list(
                positionals[0], require_ascending=(operation != "move"))
        except (IndexError, TypeError, ValueError):
            return ([f"error {request_id} bench {operation} "
                     "code=bad_argument field=motors"], False)
        if self.active_motion is not None and operation != "stop":
            return [f"error {request_id} bench {operation} code=busy"], False
        if operation not in {"jog", "move", "mode", "mit"} and fields:
            return ([f"error {request_id} bench {operation} "
                     "code=bad_argument"], False)
        accepted = f"ok {request_id} bench {operation} accepted=1"
        if operation == "init":
            done = (f"done {request_id} bench init result=completed identity={mask:02x} "
                    f"mode={mask:02x} ranges={mask:02x} version={mask:02x}")
        elif operation == "mode":
            if set(fields) != {"mode"} or fields["mode"] not in {"mit", "pos_vel"}:
                return ([f"error {request_id} bench mode "
                         "code=bad_argument field=mode"], False)
            self.motor_enabled_mask &= ~mask
            done = (f"done {request_id} bench mode result=completed "
                    f"elapsed_ms=0 motors={mask:02x}")
        elif operation == "mit":
            if len(motors) != 1:
                return ([f"error {request_id} bench mit "
                         "code=bad_argument field=motors"], False)
            action = fields.get("action")
            expected_fields = ({"action", "kp", "kd", "torque_ff", "duration_ms"}
                               if action == "hold" else
                               {"action", "position", "speed", "kp", "kd",
                                "torque_ff", "duration_ms"})
            if action not in {"hold", "move"}:
                return ([f"error {request_id} bench mit "
                         "code=bad_argument field=action"], False)
            if set(fields) != expected_fields:
                return [f"error {request_id} bench mit code=bad_argument"], False
            parsed_parameters: dict[str, float] = {}
            for field_name in ("kp", "kd", "torque_ff"):
                parse_status, parsed_value = parse_strict_float32_token(
                    fields[field_name])
                if parse_status != "ok":
                    return ([f"error {request_id} bench mit "
                             f"code=bad_argument field={field_name}"], False)
                parsed_parameters[field_name] = parsed_value
            if not 0.0 < parsed_parameters["kp"] <= 500.0:
                return ([f"error {request_id} bench mit "
                         "code=out_of_range field=kp"], False)
            if not 0.0 < parsed_parameters["kd"] <= 5.0:
                return ([f"error {request_id} bench mit "
                         "code=out_of_range field=kd"], False)
            if abs(parsed_parameters["torque_ff"]) > SIMULATED_TMAX_NM:
                return ([f"error {request_id} bench mit "
                         "code=out_of_range field=torque_ff"], False)
            try:
                duration_ms = parse_strict_ascii_u32_token(fields["duration_ms"])
            except ValueError:
                return ([f"error {request_id} bench mit "
                         "code=bad_argument field=duration_ms"], False)
            if not 100 <= duration_ms <= 10000:
                return ([f"error {request_id} bench mit "
                         "code=out_of_range field=duration_ms"], False)
            target = list(self.joint_position_deg)
            total_duration_ms = duration_ms
            if action == "move":
                position_status, position = parse_strict_float32_token(
                    fields["position"])
                speed_status, speed = parse_strict_float32_token(fields["speed"])
                if position_status != "ok":
                    return ([f"error {request_id} bench mit "
                             "code=bad_argument field=position"], False)
                if abs(position) > SIMULATED_PMAX_DEG:
                    return ([f"error {request_id} bench mit "
                             "code=position_out_of_range field=position"], False)
                if speed_status != "ok":
                    return ([f"error {request_id} bench mit "
                             "code=bad_argument field=speed"], False)
                if not 0.0 < speed <= SIMULATED_MOVE_SPEED_LIMIT_DEG_S:
                    return ([f"error {request_id} bench mit "
                             "code=out_of_range field=speed"], False)
                motor_index = motors[0] - 1
                trajectory_ms = max(
                    4,
                    math.ceil(1.875 * abs(position - target[motor_index]) /
                              speed * 1000),
                )
                target[motor_index] = position
                total_duration_ms += trajectory_ms
            self.motor_enabled_mask |= mask
            self.active_motion = SimulatedMotion(
                request_id, "bench_mit", mask, self.now_ms,
                total_duration_ms, list(self.joint_position_deg), target)
            return [accepted], True
        elif operation == "enable":
            self.motor_enabled_mask |= mask
            done = f"done {request_id} bench enable result=completed enabled={mask:02x}"
        elif operation == "jog":
            if len(fields) != 2 or "delta" not in fields:
                return ([f"error {request_id} bench jog "
                         "code=bad_argument field=delta"], False)
            if "speed" not in fields:
                return ([f"error {request_id} bench jog "
                         "code=bad_argument field=speed"], False)
            delta_status, delta = parse_strict_float32_token(fields["delta"])
            if delta_status != "ok":
                return ([f"error {request_id} bench jog "
                         "code=bad_argument field=delta"], False)
            speed_status, speed = parse_strict_float32_token(fields["speed"])
            if speed_status != "ok":
                return ([f"error {request_id} bench jog "
                         "code=bad_argument field=speed"], False)
            if speed <= 0.0:
                return ([f"error {request_id} bench jog "
                         "code=out_of_range field=speed"], False)
            target = list(self.joint_position_deg)
            for motor in motors:
                target[motor - 1] += delta
            duration_ms = max(4, math.ceil(abs(delta) / speed * 1000))
            self.active_motion = SimulatedMotion(request_id, "bench", mask,
                                                 self.now_ms, duration_ms,
                                                 list(self.joint_position_deg), target)
            return [accepted], True
        elif operation == "move":
            if "position" not in fields:
                return ([f"error {request_id} bench move "
                         "code=bad_argument field=position"], False)
            if "speed" not in fields:
                return ([f"error {request_id} bench move "
                         "code=bad_argument field=speed"], False)
            if set(fields) != {"position", "speed"}:
                return [f"error {request_id} bench move code=bad_argument"], False
            position_status, positions = self._parse_move_values(
                fields["position"], len(motors), True)
            if position_status != "ok":
                return ([f"error {request_id} bench move "
                         f"code={position_status} field=position"], False)
            speed_status, speeds = self._parse_move_values(
                fields["speed"], len(motors), False)
            if speed_status != "ok":
                return ([f"error {request_id} bench move "
                         f"code={speed_status} field=speed"], False)
            positions_by_joint = [0.0] * JOINT_COUNT
            speeds_by_joint = [0.0] * JOINT_COUNT
            for motor, position, speed in zip(motors, positions, speeds):
                positions_by_joint[motor - 1] = position
                speeds_by_joint[motor - 1] = speed
            for joint_index in range(JOINT_COUNT):
                motor = joint_index + 1
                if (mask & (1 << joint_index)) == 0:
                    continue
                if abs(positions_by_joint[joint_index]) > SIMULATED_PMAX_DEG:
                    failed = (f"done {request_id} bench move result=failed "
                              "stage=validate code=position_out_of_range "
                              f"motor={motor}")
                    self._pending_outputs.append(failed)
                    return [accepted], True
                if (speeds_by_joint[joint_index] >
                        SIMULATED_MOVE_SPEED_LIMIT_DEG_S):
                    failed = (f"done {request_id} bench move result=failed "
                              "stage=validate code=speed_out_of_range "
                              f"motor={motor}")
                    self._pending_outputs.append(failed)
                    return [accepted], True
            target = list(self.joint_position_deg)
            duration_ms = 4
            for motor, position, speed in zip(motors, positions, speeds):
                duration_ms = max(
                    duration_ms,
                    math.ceil(abs(position - target[motor - 1]) / speed * 1000),
                )
                target[motor - 1] = position
            self.motor_enabled_mask |= mask
            self.active_motion = SimulatedMotion(
                request_id, "bench_move", mask, self.now_ms, duration_ms,
                list(self.joint_position_deg), target)
            return [accepted], True
        elif operation == "stop":
            effective_mask = mask
            if self.active_motion is not None:
                cancelled_motion = self.active_motion
                effective_mask |= cancelled_motion.selected_mask
                if cancelled_motion.namespace == "bench_move":
                    cancelled = (f"done {cancelled_motion.request_id} "
                                 "bench move result=cancelled")
                elif cancelled_motion.namespace == "bench_mit":
                    cancelled = (f"done {cancelled_motion.request_id} "
                                 "bench mit result=cancelled")
                else:
                    elapsed_ms = self.now_ms - cancelled_motion.start_ms
                    cancelled = (f"done {cancelled_motion.request_id} bench jog "
                                 f"result=cancelled elapsed_ms={elapsed_ms} "
                                 "arrived=00")
                self._pending_outputs.append(cancelled)
                self._replace_replay_with_done(
                    cancelled_motion.request_id, cancelled)
            self.active_motion = None
            self.joint_velocity_deg_s = [0.0] * JOINT_COUNT
            self.motor_enabled_mask &= ~effective_mask
            done = (f"done {request_id} bench stop result=stopped "
                    f"stopped={effective_mask:02x}")
        elif operation == "disable":
            self.motor_enabled_mask &= ~mask
            done = f"done {request_id} bench disable result=completed enabled=00"
        elif operation == "clear":
            self.motor_fault_mask &= ~mask
            done = f"done {request_id} bench clear result=completed fault=00"
        else:
            return ([f"error {request_id} bench {operation} "
                     "code=unknown_command"], False)
        self._pending_outputs.append(done)
        return [accepted], True

    def _dispatch(self, request_id: int, command: list[str],
                  positionals: list[str],
                  fields: dict[str, str]) -> tuple[list[str], bool]:
        """Dispatches one request and returns explicit C watchdog acceptance."""
        path = " ".join(command)
        if len(command) == 1 and command[0] in {"show", "stream", "bench"}:
            return [f"error {request_id} {path} code=unknown_command"], False
        if path == "hello":
            if positionals or fields:
                return [f"error {request_id} hello code=bad_argument"], False
            self._replay.clear()
            self.stream_kind = "off"
            self.stream_rate_hz = 0
            return ([f"ok {request_id} hello protocol=aethor-text-v1 fw=sim-1 "
                     f"profile={self.profile} dof=7 boot={self.boot_id} "
                     "watchdog_ms=1000"], True)
        if path == "ping":
            if positionals or fields:
                return [f"error {request_id} ping code=bad_argument"], False
            return ([f"ok {request_id} ping state={self.arm_state} "
                     f"enabled={self.motor_enabled_mask:02x} boot={self.boot_id}"],
                    True)
        if path == "help":
            if fields or len(positionals) > 1:
                return [f"error {request_id} help code=bad_argument"], False
            if not positionals:
                return ([f"ok {request_id} help topics=show,stream,arm,bench "
                         "examples=show_state,arm_move,bench_mit"], True)
            help_topics = {
                "show": "commands=info,state,joints,motors,motor,config,diag",
                "stream": "commands=joints,motors,off rates=joints_1_50,motors_1_10",
                "arm": "commands=align,enable,move,stop,disable,clear",
                "bench": "commands=init,mode,mit,enable,jog,move,stop,disable,clear",
            }
            topic = positionals[0]
            if topic not in help_topics:
                return [f"error {request_id} help code=bad_argument"], False
            return [f"ok {request_id} help {topic} {help_topics[topic]}"], True
        if command[0] == "show":
            return self._show(request_id, command[1], positionals, fields)
        if command[0] == "stream":
            if command[1] not in {"off", "joints", "motors"}:
                return [f"error {request_id} {path} code=unknown_command"], False
            if command[1] == "off":
                if positionals or fields:
                    return [f"error {request_id} stream off code=bad_argument"], False
                self.stream_kind = "off"
                self.stream_rate_hz = 0
                return [f"ok {request_id} stream off"], True
            if positionals or set(fields) != {"rate"}:
                return [f"error {request_id} {path} code=bad_argument"], False
            rate_hz = parse_strict_ascii_u32_token(fields["rate"])
            maximum_rate = 50 if command[1] == "joints" else 10
            if not 1 <= rate_hz <= maximum_rate:
                return ([f"error {request_id} {path} "
                         "code=out_of_range field=rate"], False)
            self.stream_kind = command[1]
            self.stream_rate_hz = rate_hz
            self.next_stream_ms = self.now_ms
            return [f"ok {request_id} {path} rate={rate_hz}"], True
        if command[0] == "bench":
            return self._bench(request_id, command[1], positionals, fields)
        if command[0] == "arm" and self.profile != "arm":
            return ([f"error {request_id} {path} code=profile "
                     "current=bench required=arm"], False)
        return [f"error {request_id} {path} code=unknown_command"], False

    def process_line(self, line: bytes | str) -> list[str]:
        """Parses and processes one optional-ID plain text request line."""
        try:
            text = (bytes(line).decode("ascii")
                    if isinstance(line, (bytes, bytearray)) else line)
        except UnicodeError:
            return ["error 0 parse code=bad_line"]
        if text.endswith("\n"):
            text = text[:-1]
        if text.endswith("\r"):
            text = text[:-1]
        if (len(text) > AETHOR_TEXT_MAX_REQUEST_BODY_BYTES or
                any(ord(character) < 0x20 or ord(character) > 0x7E
                    for character in text)):
            return ["error 0 parse code=line_too_long"
                    if len(text) > AETHOR_TEXT_MAX_REQUEST_BODY_BYTES
                    else "error 0 parse code=bad_line"]
        tokens = text.split()
        if not tokens or len(tokens) > AETHOR_TEXT_MAX_TOKEN_COUNT:
            return ["error 0 parse code=bad_line"]
        request_id = 0
        if tokens[0][0].isascii() and tokens[0][0].isdigit():
            if not tokens[0].isascii() or not tokens[0].isdigit():
                return ["error 0 parse code=bad_line"]
            request_id = int(tokens.pop(0), 10)
            if request_id > 0xFFFFFFFF:
                return ["error 0 parse code=bad_line"]
        if not tokens:
            return ["error 0 parse code=bad_line"]
        if not is_text_identifier(tokens[0]):
            return ["error 0 parse code=bad_line"]
        first_command_word = tokens.pop(0).lower()
        single_word = first_command_word in {"hello", "ping", "help"}
        command = [first_command_word]
        if not single_word and tokens and "=" not in tokens[0]:
            if not is_text_identifier(tokens[0]):
                return ["error 0 parse code=bad_line"]
            command.append(tokens.pop(0).lower())
        positionals: list[str] = []
        fields: dict[str, str] = {}
        for token in tokens:
            if "=" in token:
                if token.count("=") != 1:
                    return ["error 0 parse code=bad_line"]
                key, value = token.split("=", 1)
                normalized_key = key.lower()
                if (not value or not is_text_identifier(key) or
                        len(fields) >= AETHOR_TEXT_MAX_FIELD_COUNT or
                        normalized_key in fields):
                    return ["error 0 parse code=bad_line"]
                fields[normalized_key] = value
            else:
                if fields or len(positionals) >= AETHOR_TEXT_MAX_POSITIONAL_COUNT:
                    return ["error 0 parse code=bad_line"]
                positionals.append(token)
        canonical_parts = command + positionals + [f"{key}={value}"
                                                    for key, value in fields.items()]
        canonical = " ".join(canonical_parts)
        cached = self._replay.get(request_id) if request_id else None
        if cached is not None:
            if cached[0] != canonical:
                return [f"error {request_id} {' '.join(command)} "
                        "code=request_conflict"]
            if cached[3]:
                self.last_request_ms = self.now_ms
                self.watchdog_reported = False
            return list(cached[1])
        try:
            outputs, accepted_for_watchdog = self._dispatch(
                request_id, command, positionals, fields)
        except (IndexError, KeyError, ValueError):
            return [f"error {request_id} {' '.join(command)} code=bad_argument"]
        self._remember(request_id, canonical, outputs, accepted_for_watchdog)
        if accepted_for_watchdog:
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
