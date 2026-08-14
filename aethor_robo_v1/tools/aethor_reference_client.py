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


class LineTransport(Protocol):
    """Defines the minimal request/response transport needed by the client."""

    def transact(self, body: str) -> list[str]:
        """Sends one request body and returns available decoded response bodies."""

    def transact_until_done(self, body: str, request_id: int) -> list[str]:
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
    plain_text = format(Decimal(str(numeric_value)), "f")
    if "." in plain_text:
        plain_text = plain_text.rstrip("0").rstrip(".")
    return plain_text


def build_one_shot_bench_move(
        request_id: int,
        motor_numbers: Sequence[int],
        positions_deg: Sequence[Real],
        speeds_deg_s: Sequence[Real]) -> str:
    """Builds one strict absolute bench-move request without broadcasting values."""
    if (isinstance(request_id, bool) or not isinstance(request_id, int) or
            request_id < 0 or request_id > 0xFFFFFFFF):
        raise ValueError("request_id must be a uint32 integer")

    motors = list(motor_numbers)
    positions = list(positions_deg)
    speeds = list(speeds_deg_s)
    if not motors or len(motors) != len(positions) or len(motors) != len(speeds):
        raise ValueError("motor, position, and speed arrays need equal non-empty lengths")
    if any(isinstance(motor, bool) or not isinstance(motor, int) or
           motor < 1 or motor > 7 for motor in motors):
        raise ValueError("motor numbers must be integers from 1 through 7")
    if len(set(motors)) != len(motors):
        raise ValueError("motor numbers must be unique")
    if any(isinstance(position, bool) or not isinstance(position, Real) or
           not math.isfinite(float(position)) for position in positions):
        raise ValueError("positions must be finite numbers")
    if any(isinstance(speed, bool) or not isinstance(speed, Real) or
           not math.isfinite(float(speed)) or float(speed) <= 0.0
           for speed in speeds):
        raise ValueError("speeds must be finite positive numbers")

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


def parse_one_shot_move_terminal(line: str,
                                 request_id: int) -> OneShotMoveResult:
    """Parses one matching completed, failed, cancelled, or stopped move terminal."""
    tokens = line.strip().split()
    if len(tokens) < 5 or tokens[:1] != ["done"] or tokens[2:4] != ["bench", "move"]:
        raise ValueError("line is not a bench move terminal")
    try:
        line_request_id = int(tokens[1], 10)
    except ValueError as exc:
        raise ValueError("terminal request ID is not decimal") from exc
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
    if result_token not in {"completed", "failed", "cancelled", "stopped"}:
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

    def transact_until_done(self, body: str, request_id: int) -> list[str]:
        """Processes one simulated action write and drains deterministic outputs."""
        del request_id
        return self.transact(body)


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

    def transact_until_done(self, body: str, request_id: int) -> list[str]:
        """Writes one action line once and reads until its matching DONE or timeout."""
        self.serial.write(encode_line(body))
        self.serial.flush()
        outputs: list[str] = []
        done_prefix = f"done {request_id} bench move "
        error_prefix = f"error {request_id} bench move "
        deadline = time.monotonic() + self.action_timeout
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
                        speeds_deg_s: Sequence[Real]) -> OneShotMoveResult:
        """Sends one absolute move once, starts no keepalive, and waits for DONE."""
        request_id = self.next_request_id
        self.next_request_id += 1
        body = build_one_shot_bench_move(
            request_id, motor_numbers, positions_deg, speeds_deg_s)
        outputs = self.transport.transact_until_done(body, request_id)
        self.transcript.append(f"> {body}")
        self.transcript.extend(f"< {output}" for output in outputs)

        acceptance = f"ok {request_id} bench move accepted=1"
        if acceptance not in outputs:
            raise RuntimeError("one-shot move did not receive matching acceptance")
        terminal_prefix = f"done {request_id} bench move "
        terminal_lines = [output for output in outputs
                          if output.startswith(terminal_prefix)]
        if len(terminal_lines) != 1:
            raise RuntimeError("one-shot move did not receive one matching terminal")
        return parse_one_shot_move_terminal(terminal_lines[0], request_id)

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
