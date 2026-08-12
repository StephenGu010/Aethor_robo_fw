"""Conservative Windows CDC automation for one selected S3519 commissioning axis."""

from __future__ import annotations

import argparse
import re
import sys
import time

import serial


def read_protocol_line(serial_port: serial.Serial, timeout_seconds: float = 2.0) -> str:
    """Read one non-empty CR/LF-terminated protocol line before timeout."""
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
    """Send one text command and require an explicit success response."""
    serial_port.write(command.encode("ascii") + b"\r\n")
    response = read_protocol_line(serial_port)
    if not response.startswith("ok"):
        raise AssertionError(f"command={command!r}, response={response!r}")
    return response


def parse_hex_field(response: str, field_name: str) -> int:
    """Extract one hexadecimal key/value field from a GETSTATE response."""
    match = re.search(rf"(?:^|\s){re.escape(field_name)}=0x([0-9A-Fa-f]+)(?:\s|$)", response)
    if match is None:
        raise AssertionError(f"missing {field_name!r} in {response!r}")
    return int(match.group(1), 16)


def parse_decimal_field(response: str, field_name: str) -> int:
    """Extract one unsigned decimal key/value field from a GETSTATE response."""
    match = re.search(rf"(?:^|\s){re.escape(field_name)}=(\d+)(?:\s|$)", response)
    if match is None:
        raise AssertionError(f"missing {field_name!r} in {response!r}")
    return int(match.group(1), 10)


def parse_joint_positions(response: str) -> list[float]:
    """Parse exactly seven joint angles from a GETJPOS response."""
    if not response.startswith("ok "):
        raise AssertionError(f"invalid position response {response!r}")
    positions = [float(value) for value in response[3:].split(",")]
    if len(positions) != 7:
        raise AssertionError(f"expected seven joint positions, got {positions!r}")
    return positions


def wait_for_range_discovery(serial_port: serial.Serial, joint_number: int) -> None:
    """Wait until PMAX, VMAX, and TMAX have been received for the selected motor."""
    required_bit = 1 << (joint_number - 1)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        response = transact(serial_port, "#GETSTATE")
        if parse_hex_field(response, "ranges") & required_bit:
            return
        time.sleep(0.2)
    raise TimeoutError("S3519 PMAX/VMAX/TMAX discovery did not complete")


def wait_for_enable_feedback(serial_port: serial.Serial, joint_number: int) -> None:
    """Wait until state-one motor feedback confirms the selected enable request."""
    required_bit = 1 << (joint_number - 1)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        response = transact(serial_port, "#GETSTATE")
        if parse_hex_field(response, "enabled") & required_bit:
            return
        time.sleep(0.02)
    raise TimeoutError("selected motor did not confirm enable")


def build_move_command(joint_number: int, target_degrees: float, speed_percent: float) -> str:
    """Build a seven-axis command with motion only on the selected commissioning axis."""
    targets = [0.0] * 7
    targets[joint_number - 1] = target_degrees
    values = [f"{target:.3f}" for target in targets]
    values.append(f"{speed_percent:.1f}")
    return ">" + ",".join(values)


def wait_for_arrival(serial_port: serial.Serial, timeout_seconds: float = 10.0) -> int:
    """Wait for firmware's three-cycle arrival qualification and return its timestamp."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        response = transact(serial_port, "#GETSTATE")
        arrival_time_ms = parse_decimal_field(response, "arrival")
        if arrival_time_ms != 0:
            return arrival_time_ms
        time.sleep(0.05)
    raise TimeoutError("joint did not enter the qualified arrival window")


def run_commissioning(port_name: str, joint_number: int) -> None:
    """Run enumeration-level checks, range discovery, selected-axis motion, and disable."""
    with serial.Serial(port=port_name, baudrate=115200, timeout=0.05) as serial_port:
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        transact(serial_port, "#PING")
        transact(serial_port, "#ECHO commissioning")
        transact(serial_port, f"#SELECT {joint_number}")
        wait_for_range_discovery(serial_port, joint_number)
        transact(serial_port, "!START")
        wait_for_enable_feedback(serial_port, joint_number)
        start_positions = parse_joint_positions(transact(serial_port, "#GETJPOS"))
        start_position = start_positions[joint_number - 1]

        transact(serial_port,
                 build_move_command(joint_number, start_position + 3.0, 100.0))
        positive_arrival_ms = wait_for_arrival(serial_port)
        transact(serial_port,
                 build_move_command(joint_number, start_position - 3.0, 100.0))
        negative_arrival_ms = wait_for_arrival(serial_port)

        position_response = transact(serial_port, "#GETJPOS")
        state_response = transact(serial_port, "#GETSTATE")
        transact(serial_port, "!DISABLE")
        print(
            "S3519_COMMISSIONING_PASSED "
            f"positive_arrival_ms={positive_arrival_ms} "
            f"negative_arrival_ms={negative_arrival_ms} "
            f"positions={position_response!r} state={state_response!r}"
        )


def parse_arguments() -> argparse.Namespace:
    """Parse Windows COM port and selected motor arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Windows CDC port, for example COM7")
    parser.add_argument("--joint", type=int, choices=range(1, 8), default=1)
    return parser.parse_args()


def main() -> int:
    """Run the commissioning sequence and return a process-friendly result code."""
    arguments = parse_arguments()
    run_commissioning(arguments.port, arguments.joint)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, TimeoutError, AssertionError, UnicodeError) as error:
        print(f"S3519_COMMISSIONING_FAILED: {error}", file=sys.stderr)
        raise SystemExit(1)
