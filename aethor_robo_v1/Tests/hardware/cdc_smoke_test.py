"""Windows USB CDC smoke test for Aethor PING, ECHO, and packet-length handling."""

from __future__ import annotations

import argparse
import sys
import time

import serial


def read_protocol_line(serial_port: serial.Serial, timeout_seconds: float) -> str:
    """Read one CR/LF-terminated UTF-8 protocol line before the deadline."""
    deadline = time.monotonic() + timeout_seconds
    received_bytes = bytearray()
    while time.monotonic() < deadline:
        received_byte = serial_port.read(1)
        if not received_byte:
            continue
        if received_byte in (b"\r", b"\n"):
            if received_bytes:
                decoded_line = received_bytes.decode("utf-8", errors="strict")
                if decoded_line.startswith("probe "):
                    received_bytes.clear()
                    continue
                return decoded_line
            continue
        received_bytes.extend(received_byte)
    raise TimeoutError("USB CDC response timed out")


def transact(serial_port: serial.Serial, command: str, expected_response: str) -> None:
    """Send one command and assert its exact protocol response."""
    serial_port.write(command.encode("utf-8") + b"\r\n")
    actual_response = read_protocol_line(serial_port, 2.0)
    if actual_response != expected_response:
        raise AssertionError(
            f"command={command!r}, actual={actual_response!r}, expected={expected_response!r}"
        )


def run_smoke_test(port_name: str) -> None:
    """Exercise short, fragmented, coalesced, and long CDC transfers."""
    with serial.Serial(port=port_name, baudrate=115200, timeout=0.05) as serial_port:
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        transact(serial_port, "#PING", "ok PONG")
        transact(serial_port, "#ECHO short", "ok short")

        serial_port.write(b"#PI")
        time.sleep(0.02)
        serial_port.write(b"NG\r\n#ECHO sticky\n")
        first_response = read_protocol_line(serial_port, 2.0)
        second_response = read_protocol_line(serial_port, 2.0)
        if (first_response, second_response) != ("ok PONG", "ok sticky"):
            raise AssertionError(
                f"fragment/sticky responses were {(first_response, second_response)!r}"
            )

        long_payload = "L" * 90
        transact(serial_port, f"#ECHO {long_payload}", f"ok {long_payload}")


def parse_arguments() -> argparse.Namespace:
    """Parse the required Windows COM port argument."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Windows CDC port, for example COM7")
    return parser.parse_args()


def main() -> int:
    """Run the CDC smoke test and return a process-friendly result code."""
    arguments = parse_arguments()
    run_smoke_test(arguments.port)
    print("CDC_SMOKE_TEST_PASSED")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, TimeoutError, AssertionError, UnicodeError) as error:
        print(f"CDC_SMOKE_TEST_FAILED: {error}", file=sys.stderr)
        raise SystemExit(1)
