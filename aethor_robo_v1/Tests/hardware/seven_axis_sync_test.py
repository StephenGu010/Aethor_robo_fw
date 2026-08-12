"""Seven-axis synchronized arrival test for a fully commissioned Aethor robot."""

from __future__ import annotations

import argparse
import re
import sys
import time

import serial


def read_line(serial_port: serial.Serial, timeout_seconds: float = 2.0) -> str:
    """Read one non-empty CR/LF-terminated controller line."""
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
    raise TimeoutError("controller response timed out")


def transact(serial_port: serial.Serial, command: str) -> str:
    """Send one controller command and require an explicit success response."""
    serial_port.write(command.encode("ascii") + b"\r\n")
    response = read_line(serial_port)
    if not response.startswith("ok"):
        raise AssertionError(f"command={command!r}, response={response!r}")
    return response


def parse_integer_field(response: str, field_name: str, base: int = 10) -> int:
    """Extract a decimal or 0x-prefixed integer state field."""
    value_pattern = r"0x[0-9A-Fa-f]+" if base == 16 else r"\d+"
    match = re.search(rf"(?:^|\s){re.escape(field_name)}=({value_pattern})(?:\s|$)", response)
    if match is None:
        raise AssertionError(f"missing {field_name!r} in {response!r}")
    return int(match.group(1), base)


def parse_arrival_times(response: str) -> list[int]:
    """Extract all seven qualified arrival timestamps from GETSTATE."""
    match = re.search(r"(?:^|\s)arrivals=([0-9,]+)(?:\s|$)", response)
    if match is None:
        raise AssertionError(f"missing arrivals in {response!r}")
    arrival_times = [int(value) for value in match.group(1).split(",")]
    if len(arrival_times) != 7:
        raise AssertionError(f"expected seven arrival times, got {arrival_times!r}")
    return arrival_times


def parse_positions(response: str) -> list[float]:
    """Extract seven output-shaft angles from GETJPOS."""
    if not response.startswith("ok "):
        raise AssertionError(f"invalid position response {response!r}")
    positions = [float(value) for value in response[3:].split(",")]
    if len(positions) != 7:
        raise AssertionError(f"expected seven positions, got {positions!r}")
    return positions


def wait_until_ready(serial_port: serial.Serial) -> None:
    """Require all runtime ranges and a released commissioning lock."""
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        response = transact(serial_port, "#GETSTATE")
        if parse_integer_field(response, "lock") != 0:
            raise AssertionError("seven-axis commissioning lock is still active")
        if parse_integer_field(response, "ranges", 16) == 0x7F:
            return
        time.sleep(0.2)
    raise TimeoutError("seven-axis range discovery did not complete")


def wait_for_all_arrivals(serial_port: serial.Serial, timeout_seconds: float) -> tuple[list[int], int]:
    """Wait until all seven axes satisfy the three-cycle arrival condition."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        response = transact(serial_port, "#GETSTATE")
        arrival_times = parse_arrival_times(response)
        if all(arrival_time != 0 for arrival_time in arrival_times):
            return arrival_times, parse_integer_field(response, "sync")
        time.sleep(0.05)
    raise TimeoutError("not all joints reached the qualified arrival window")


def run_sync_test(port_name: str, targets: list[float], speed_percent: float) -> None:
    """Enable seven commissioned axes, execute one common-progress move, and report errors."""
    with serial.Serial(port=port_name, baudrate=115200, timeout=0.05) as serial_port:
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        transact(serial_port, "#PING")
        wait_until_ready(serial_port)
        transact(serial_port, "!START")
        try:
            command_values = [f"{target:.4f}" for target in targets]
            command_values.append(f"{speed_percent:.1f}")
            transact(serial_port, ">" + ",".join(command_values))
            arrival_times, sync_difference_ms = wait_for_all_arrivals(serial_port, 60.0)
            positions = parse_positions(transact(serial_port, "#GETJPOS"))
            errors = [position - target for position, target in zip(positions, targets)]
            if any(abs(error) > 0.5 for error in errors):
                raise AssertionError(f"position errors exceed 0.5 degrees: {errors!r}")
            if sync_difference_ms > 50:
                raise AssertionError(f"maximum arrival difference is {sync_difference_ms} ms")
            print(
                "SEVEN_AXIS_SYNC_PASSED "
                f"errors_deg={errors!r} arrivals_ms={arrival_times!r} "
                f"maximum_sync_difference_ms={sync_difference_ms}"
            )
        finally:
            serial_port.write(b"!DISABLE\r\n")
            try:
                read_line(serial_port)
            except TimeoutError:
                pass


def parse_targets(text: str) -> list[float]:
    """Parse exactly seven comma-separated finite target angles."""
    values = [float(value) for value in text.split(",")]
    if len(values) != 7 or any(not (-36000.0 < value < 36000.0) for value in values):
        raise argparse.ArgumentTypeError("--targets requires seven finite comma-separated angles")
    return values


def parse_arguments() -> argparse.Namespace:
    """Parse Windows COM port, seven targets, and global speed percentage."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Windows CDC port, for example COM7")
    parser.add_argument("--targets", required=True, type=parse_targets)
    parser.add_argument("--speed", type=float, default=50.0)
    arguments = parser.parse_args()
    if not 1.0 <= arguments.speed <= 100.0:
        parser.error("--speed must be in the inclusive range 1..100")
    return arguments


def main() -> int:
    """Run the synchronized seven-axis hardware test."""
    arguments = parse_arguments()
    run_sync_test(arguments.port, arguments.targets, arguments.speed)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, TimeoutError, AssertionError, UnicodeError, ValueError) as error:
        print(f"SEVEN_AXIS_SYNC_FAILED: {error}", file=sys.stderr)
        raise SystemExit(1)
