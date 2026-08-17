"""Scriptable reference client for simulator or real USB CDC compatibility checks."""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time
from dataclasses import dataclass
from decimal import Decimal
from numbers import Real
from typing import Protocol, Sequence

from aethor_text_simulator import AethorTextSimulator, encode_line

# Matches TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH. Firmware removes CR/LF before
# applying this limit, so the client checks the ASCII request body only.
AETHOR_TEXT_MAX_REQUEST_BODY_BYTES = 160
# SimulatorTransport converts the timeout to an unsigned 32-bit millisecond
# budget; the same cap also keeps SerialTransport's monotonic deadline finite.
AETHOR_ACTION_TIMEOUT_MAX_SECONDS = 0xFFFFFFFF / 1000.0
FLOAT32_MIN_NORMAL = 1.1754943508222875e-38
FLOAT32_MAX = 3.4028234663852886e38
ONE_SHOT_MOVE_RESULTS = {"completed", "failed", "cancelled", "stopped"}
ONE_SHOT_MOVE_STAGES = {
    "validate", "discovery", "mode", "clear", "enable", "motion", "hold",
    "disable", "unknown",
}
ONE_SHOT_MOVE_ERRORS = {
    "not_ready", "position_out_of_range", "speed_out_of_range",
    "fault_present", "stale_feedback", "timeout", "feedback_timeout",
    "action_failed",
}


class LineTransport(Protocol):
    """Defines the minimal request/response transport needed by the client."""

    def transact(self, body: str) -> list[str]:
        """Sends one request body and returns available decoded response bodies."""

    def transact_until_done(self, body: str, request_id: int,
                            action_timeout: float) -> list[str]:
        """Sends one action body once and waits for its matching terminal line."""


@dataclass(frozen=True)
class OneShotMoveResult:
    """Contains the stable public fields from one bench-move terminal line."""

    request_id: int
    result: str
    elapsed_ms: int | None = None
    motor_mask: int | None = None
    stage: str | None = None
    code: str | None = None
    motor: int | None = None
    raw_line: str = ""


def _format_finite_number(value: Real) -> str:
    """Formats one finite float as round-trip-safe plain decimal text."""
    numeric_value = float(value)
    if numeric_value == 0.0:
        return "0"
    if abs(numeric_value) == FLOAT32_MIN_NORMAL:
        # The shortest Python spelling lies microscopically below FLT_MIN and
        # makes MSVCRT strtof set ERANGE. This equally rounding spelling is above.
        minimum_text = format(Decimal("1.1754943508222876e-38"), "f")
        return f"-{minimum_text}" if numeric_value < 0.0 else minimum_text
    plain_text = format(Decimal(str(numeric_value)), "f")
    if "." in plain_text:
        plain_text = plain_text.rstrip("0").rstrip(".")
    return plain_text


def _coerce_firmware_float32(value: object, field_name: str,
                             allow_zero: bool) -> float:
    """Converts one value accepted by the firmware's normal float32 parser."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field_name} must be a real number")
    try:
        numeric_value = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} cannot be converted to float") from exc
    if not math.isfinite(numeric_value):
        raise ValueError(f"{field_name} must be finite")
    if numeric_value == 0.0:
        if allow_zero:
            return 0.0
        raise ValueError(f"{field_name} must be greater than zero")
    if abs(numeric_value) < FLOAT32_MIN_NORMAL or abs(numeric_value) > FLOAT32_MAX:
        raise ValueError(f"{field_name} is outside normal float32 range")
    if not allow_zero and numeric_value < 0.0:
        raise ValueError(f"{field_name} must be greater than zero")
    return numeric_value


def _validate_action_timeout(action_timeout: object) -> float:
    """Returns one finite positive timeout before any transport can write."""
    error_message = (
        "action_timeout must be finite, positive, and no greater than "
        f"{AETHOR_ACTION_TIMEOUT_MAX_SECONDS} seconds")
    if isinstance(action_timeout, bool):
        raise ValueError(error_message)
    try:
        timeout_value = float(action_timeout)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(error_message) from exc
    if (not math.isfinite(timeout_value) or timeout_value <= 0.0 or
            timeout_value > AETHOR_ACTION_TIMEOUT_MAX_SECONDS):
        raise ValueError(error_message)
    return timeout_value


def build_one_shot_bench_move(
        request_id: int,
        motor_numbers: Sequence[int],
        positions_deg: Sequence[Real],
        speeds_deg_s: Sequence[Real]) -> str:
    """Builds one strict absolute bench-move request without broadcasting values."""
    if (isinstance(request_id, bool) or not isinstance(request_id, int) or
            request_id < 1 or request_id > 0xFFFFFFFF):
        raise ValueError("request_id must be a nonzero uint32 integer")

    motors = list(motor_numbers)
    positions_input = list(positions_deg)
    speeds_input = list(speeds_deg_s)
    if (not motors or len(motors) != len(positions_input) or
            len(motors) != len(speeds_input)):
        raise ValueError("motor, position, and speed arrays need equal non-empty lengths")
    if any(isinstance(motor, bool) or not isinstance(motor, int) or
           motor < 1 or motor > 7 for motor in motors):
        raise ValueError("motor numbers must be integers from 1 through 7")
    if len(set(motors)) != len(motors):
        raise ValueError("motor numbers must be unique")
    positions = [_coerce_firmware_float32(value, "position", True)
                 for value in positions_input]
    speeds = [_coerce_firmware_float32(value, "speed", False)
              for value in speeds_input]

    motor_text = ",".join(str(motor) for motor in motors)
    position_text = ",".join(_format_finite_number(value) for value in positions)
    speed_text = ",".join(_format_finite_number(value) for value in speeds)
    body = (f"{request_id} bench move {motor_text} "
            f"position={position_text} speed={speed_text}")
    if len(body.encode("ascii")) > AETHOR_TEXT_MAX_REQUEST_BODY_BYTES:
        raise ValueError(
            "bench move request exceeds 160 ASCII bytes excluding CR/LF")
    return body


def _parse_decimal_field(value: str, field_name: str) -> int:
    """Parses one nonnegative decimal result field with a bounded uint32 value."""
    if not value.isdecimal():
        raise ValueError(f"{field_name} must be decimal")
    numeric_value = int(value, 10)
    if numeric_value > 0xFFFFFFFF:
        raise ValueError(f"{field_name} exceeds uint32")
    return numeric_value


def _parse_nonzero_request_id(value: str, field_name: str) -> int:
    """Parses one unsigned ASCII decimal request ID excluding the zero sentinel."""
    if not value.isascii() or not value.isdigit():
        raise ValueError(f"{field_name} must be unsigned ASCII decimal")
    request_id = _parse_decimal_field(value, field_name)
    if request_id == 0:
        raise ValueError(f"{field_name} must be nonzero")
    return request_id


def parse_one_shot_move_terminal(line: str,
                                 request_id: int) -> OneShotMoveResult:
    """Parses one matching completed, failed, cancelled, or stopped move terminal."""
    tokens = line.strip().split()
    if len(tokens) < 5 or tokens[:1] != ["done"] or tokens[2:4] != ["bench", "move"]:
        raise ValueError("line is not a bench move terminal")
    if (isinstance(request_id, bool) or not isinstance(request_id, int) or
            request_id < 1 or request_id > 0xFFFFFFFF):
        raise ValueError("expected request ID must be a nonzero uint32")
    line_request_id = _parse_nonzero_request_id(tokens[1], "terminal request ID")
    if line_request_id != request_id:
        raise ValueError("terminal request ID does not match")

    fields: dict[str, str] = {}
    for token in tokens[4:]:
        if "=" not in token:
            raise ValueError("terminal field is malformed")
        key, value = token.split("=", 1)
        if not key or not value or key in fields:
            raise ValueError("terminal field is missing or duplicated")
        fields[key] = value

    result_token = fields.get("result")
    if result_token not in ONE_SHOT_MOVE_RESULTS:
        raise ValueError("terminal result token is unsupported")
    if result_token == "completed":
        if set(fields) != {"result", "elapsed_ms", "motors"}:
            raise ValueError("completed terminal fields do not match the contract")
        elapsed_ms = _parse_decimal_field(fields["elapsed_ms"], "elapsed_ms")
        motor_text = fields["motors"]
        if len(motor_text) != 2 or any(character not in "0123456789abcdef"
                                       for character in motor_text):
            raise ValueError("motors must be a two-digit lowercase hex mask")
        motor_mask = int(motor_text, 16)
        if motor_mask < 1 or motor_mask > 0x7F:
            raise ValueError("motors mask is outside the seven-motor range")
        return OneShotMoveResult(
            request_id=line_request_id,
            result=result_token,
            elapsed_ms=elapsed_ms,
            motor_mask=motor_mask,
            raw_line=line,
        )
    if result_token == "failed":
        if set(fields) != {"result", "stage", "code", "motor"}:
            raise ValueError("failed terminal fields do not match the contract")
        if fields["stage"] not in ONE_SHOT_MOVE_STAGES:
            raise ValueError("failed terminal stage token is unsupported")
        if fields["code"] not in ONE_SHOT_MOVE_ERRORS:
            raise ValueError("failed terminal error token is unsupported")
        motor_value: int | None = None
        if fields["motor"] != "?":
            motor_value = _parse_decimal_field(fields["motor"], "motor")
            if motor_value < 1 or motor_value > 7:
                raise ValueError("motor is outside the one-based range")
        return OneShotMoveResult(
            request_id=line_request_id,
            result=result_token,
            stage=fields["stage"],
            code=fields["code"],
            motor=motor_value,
            raw_line=line,
        )
    if set(fields) != {"result"}:
        raise ValueError("cancelled or stopped terminals contain unexpected fields")
    return OneShotMoveResult(
        request_id=line_request_id,
        result=result_token,
        raw_line=line,
    )


class SimulatorTransport:
    """Adapts the deterministic in-process simulator to LineTransport."""

    def __init__(self) -> None:
        """Creates one fresh simulator boot."""
        self.simulator = AethorTextSimulator(boot_id=1234, profile="bench")

    def transact(self, body: str) -> list[str]:
        """Processes one request and drains resulting immediate lifecycle output."""
        outputs = self.simulator.process_line(encode_line(body))
        outputs.extend(self.simulator.drain_outputs())
        return outputs

    def transact_until_done(self, body: str, request_id: int,
                            action_timeout: float) -> list[str]:
        """Processes one simulated action write and drains deterministic outputs."""
        timeout_value = _validate_action_timeout(action_timeout)
        outputs = self.simulator.process_line(encode_line(body))
        accepted = f"ok {request_id} bench move accepted=1"
        if accepted in outputs and self.simulator.active_motion is not None:
            remaining_ms = max(
                0,
                self.simulator.active_motion.duration_ms -
                (self.simulator.now_ms - self.simulator.active_motion.start_ms),
            )
            maximum_wait_ms = int(timeout_value * 1000.0)
            self.simulator.advance(min(remaining_ms, maximum_wait_ms))
        outputs.extend(self.simulator.drain_outputs())
        return outputs


class SerialTransport:
    """Uses optional pyserial to transact with an STM32 USB CDC COM port."""

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout: float = 0.5, action_timeout: float = 120.0) -> None:
        """Opens one explicit COM port without treating baudrate as USB timing."""
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Serial mode requires: python -m pip install pyserial") from exc
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.action_timeout = action_timeout

    def transact(self, body: str) -> list[str]:
        """Writes one plain text line and collects outputs until timeout."""
        self.serial.write(encode_line(body))
        self.serial.flush()
        outputs: list[str] = []
        deadline = time.monotonic() + float(self.serial.timeout or 0.5)
        while time.monotonic() < deadline:
            line = self.serial.readline()
            if not line:
                break
            outputs.append(line.decode("ascii", errors="strict").rstrip("\r\n"))
            if outputs[-1].startswith(("ok ", "error ", "done ")):
                break
        return outputs

    def transact_until_done(self, body: str, request_id: int,
                            action_timeout: float) -> list[str]:
        """Writes one action line once and reads until its matching DONE or timeout."""
        timeout_value = _validate_action_timeout(action_timeout)
        self.serial.write(encode_line(body))
        self.serial.flush()
        outputs: list[str] = []
        done_prefix = f"done {request_id} bench move "
        error_prefix = f"error {request_id} bench move "
        deadline = time.monotonic() + timeout_value
        while time.monotonic() < deadline:
            line = self.serial.readline()
            if not line:
                continue
            decoded_line = line.decode("ascii", errors="strict").rstrip("\r\n")
            outputs.append(decoded_line)
            if decoded_line.startswith((done_prefix, error_prefix)):
                break
        return outputs


class AethorReferenceClient:
    """Allocates request IDs and records a reproducible protocol transcript."""

    def __init__(self, transport: LineTransport) -> None:
        """Initializes one request sequence around a selected transport."""
        self.transport = transport
        self.next_request_id = 1
        self.transcript: list[str] = []

    def request(self, command: str, **fields: object) -> list[str]:
        """Builds, sends, and records one canonical readable request."""
        request_id = self.next_request_id
        self.next_request_id += 1
        field_text = " ".join(f"{key}={value}" for key, value in fields.items())
        body = f"{request_id} {command}" + (f" {field_text}" if field_text else "")
        outputs = self.transport.transact(body)
        self.transcript.append(f"> {body}")
        self.transcript.extend(f"< {output}" for output in outputs)
        return outputs

    def bench_move_once(self,
                        motor_numbers: Sequence[int],
                        positions_deg: Sequence[Real],
                        speeds_deg_s: Sequence[Real],
                        action_timeout: float | None = None) -> OneShotMoveResult:
        """Sends one absolute move once, starts no keepalive, and waits for DONE."""
        timeout_value = _validate_action_timeout(
            getattr(self.transport, "action_timeout", 120.0)
            if action_timeout is None else action_timeout)
        request_id = self.next_request_id
        self.next_request_id += 1
        body = build_one_shot_bench_move(
            request_id, motor_numbers, positions_deg, speeds_deg_s)
        outputs = self.transport.transact_until_done(
            body, request_id, timeout_value)
        self.transcript.append(f"> {body}")
        self.transcript.extend(f"< {output}" for output in outputs)

        acceptance = f"ok {request_id} bench move accepted=1"
        acceptance_indexes = [index for index, output in enumerate(outputs)
                              if output == acceptance]
        if len(acceptance_indexes) != 1:
            raise RuntimeError("one-shot move did not receive matching acceptance")
        terminal_prefix = f"done {request_id} bench move "
        terminal_lines = [output for output in outputs
                          if output.startswith(terminal_prefix)]
        if len(terminal_lines) != 1:
            raise RuntimeError("one-shot move did not receive one matching terminal")
        terminal_index = outputs.index(terminal_lines[0])
        if acceptance_indexes[0] >= terminal_index:
            raise RuntimeError("one-shot move terminal arrived before acceptance")
        result = parse_one_shot_move_terminal(terminal_lines[0], request_id)
        selected_mask = sum(1 << (motor - 1) for motor in motor_numbers)
        if result.result == "completed" and result.motor_mask != selected_mask:
            raise RuntimeError("one-shot move completed mask does not match request")
        if (result.result == "failed" and result.motor is not None and
                (selected_mask & (1 << (result.motor - 1))) == 0):
            raise RuntimeError("one-shot move failed motor is outside request")
        return result

    def run_safe_contract_probe(self) -> None:
        """Runs query-only shared checks that never enable or move hardware."""
        self.request("hello")
        self.request("show info")
        self.request("show config")
        self.request("show state")
        self.request("show joints")
        self.request("show motors")
        self.request("show diag")
        self.request("stream off")

    def save_transcript(self, output_path: pathlib.Path) -> None:
        """Writes the exact request/response transcript in UTF-8."""
        output_path.write_text("\n".join(self.transcript) + "\n", encoding="utf-8")


def main() -> int:
    """Runs a safe simulator or COM-port contract probe."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Explicit STM32 USB CDC COM port, e.g. COM7")
    parser.add_argument("--output", type=pathlib.Path,
                        default=pathlib.Path("aethor-contract-transcript.txt"))
    arguments = parser.parse_args()
    transport: LineTransport = (SerialTransport(arguments.port)
                                if arguments.port else SimulatorTransport())
    client = AethorReferenceClient(transport)
    client.run_safe_contract_probe()
    client.save_transcript(arguments.output)
    print(arguments.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
