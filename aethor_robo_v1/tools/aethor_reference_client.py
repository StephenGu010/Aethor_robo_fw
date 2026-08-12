"""Scriptable reference client for simulator or real USB CDC compatibility checks."""

from __future__ import annotations

import argparse
import pathlib
import sys
import time
from typing import Protocol

from aethor_host_simulator import AethorHostSimulator, encode_frame


class LineTransport(Protocol):
    """Defines the minimal request/response transport needed by the client."""

    def transact(self, body: str) -> list[str]:
        """Sends one request body and returns available decoded response bodies."""


class SimulatorTransport:
    """Adapts the deterministic in-process simulator to LineTransport."""

    def __init__(self) -> None:
        """Creates one fresh simulator boot."""
        self.simulator = AethorHostSimulator(boot_id=1234)

    def transact(self, body: str) -> list[str]:
        """Processes one request and drains resulting immediate lifecycle output."""
        outputs = self.simulator.process_line(encode_frame(body))
        outputs.extend(self.simulator.drain_outputs())
        return outputs


class SerialTransport:
    """Uses optional pyserial to transact with an STM32 USB CDC COM port."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5) -> None:
        """Opens one explicit COM port without treating baudrate as USB timing."""
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("Serial mode requires: python -m pip install pyserial") from exc
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def transact(self, body: str) -> list[str]:
        """Writes one CRC frame and collects lines until the timeout expires."""
        self.serial.write(encode_frame(body))
        self.serial.flush()
        outputs: list[str] = []
        deadline = time.monotonic() + float(self.serial.timeout or 0.5)
        while time.monotonic() < deadline:
            line = self.serial.readline()
            if not line:
                break
            outputs.append(line.decode("ascii", errors="strict").rstrip("\r\n"))
            if outputs[-1].startswith(("RSP ", "ERR ", "DONE ")):
                break
        return outputs


class AethorReferenceClient:
    """Allocates request IDs and records a reproducible protocol transcript."""

    def __init__(self, transport: LineTransport) -> None:
        """Initializes one request sequence around a selected transport."""
        self.transport = transport
        self.next_request_id = 1
        self.transcript: list[str] = []

    def request(self, operation: str, **fields: object) -> list[str]:
        """Builds, sends, and records one canonical request."""
        request_id = self.next_request_id
        self.next_request_id += 1
        field_text = " ".join(f"{key}={value}" for key, value in fields.items())
        body = f"REQ {request_id} {operation}" + (f" {field_text}" if field_text else "")
        outputs = self.transport.transact(body)
        self.transcript.append(f"> {body}")
        self.transcript.extend(f"< {output}" for output in outputs)
        return outputs

    def run_safe_contract_probe(self) -> None:
        """Runs query-only shared checks that never enable or move hardware."""
        self.request("HELLO", client="reference-client", protocol=1)
        self.request("GET_INFO")
        self.request("GET_CONFIG")
        self.request("GET_STATE")
        self.request("GET_JPOS")
        self.request("GET_MOTORS")
        self.request("GET_DIAG")
        self.request("SET_STREAM", rate_hz=0, fields="jpos,jvel,state,motor")

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
