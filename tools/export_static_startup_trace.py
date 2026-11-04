"""Passively export one frozen static startup trace without issuing motor commands.

Only ping/status/record reads are supported. Serial packages are imported only
for explicit --capture; importing this module never touches real hardware.
Native positions/velocities/torques retain the firmware's rad/rad/s/N m units.
Raw bus SOF ordering does not prove internal motor target acceptance or timing.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import time


USB_SERIAL = "386B336F3134"
HEADER_INDEX = 4294967295
FRAGMENT_BYTES = 128
MAX_RECORDS = 128
MAX_HEADER_BYTES = 2048
MAX_RECORD_BYTES = 96
REQUIRED_STATUS = ("schema", "generation", "active", "frozen", "count", "total", "overflow",
                   "gaps", "record_size", "header_size", "target_acceptance", "first_fault", "terminal", "valid")
EVENT_NAMES = {1: "begin", 2: "dequeued", 3: "hal_submit_attempt", 4: "actual_tx", 5: "tx_unknown",
               6: "raw_rx", 7: "accepted_rx", 8: "first_fault", 9: "terminal", 10: "phase", 11: "cancelled", 12: "queued"}
PHASE_NAMES = {0: "none", 1: "preflight", 2: "preload", 3: "enable", 4: "first_hold",
               5: "pos_disable", 6: "takeover"}


def require(condition, message):
    """Reject incomplete or ambiguous evidence even when Python assertions are disabled."""
    if not condition:
        raise RuntimeError(message)


def unsigned(value, maximum=4294967295):
    """Parse bounded decimal protocol integers without accepting signs or coercion."""
    require(isinstance(value, str) and re.fullmatch(r"[0-9]{1,10}", value), "Invalid decimal field")
    result = int(value)
    require(result <= maximum, "Decimal field exceeds its protocol bound")
    return result


def parse_fields(detail):
    """Parse an ASCII key/value reply with duplicate and malformed-field rejection."""
    result = {}
    for token in detail.split():
        require(token.count("=") == 1, "Malformed reply field")
        key, value = token.split("=", 1)
        require(key and value and key not in result, "Missing or duplicate reply field")
        result[key] = value
    return result


def query_kind(command):
    """Allow only fixed passive reads and bounded immutable fragment requests."""
    if command in ("ping", "static trace_status"):
        return command
    match = re.fullmatch(r"static trace index=([0-9]{1,10}) part=([0-9]{1,2}) generation=([0-9]{1,10})", command)
    require(match is not None, "Only passive static trace reads are permitted")
    unsigned(match[1])
    unsigned(match[2], 15)
    require(unsigned(match[3]) != 0, "Trace generation must be nonzero")
    return "static trace"


def checked_status(status):
    """Require a frozen terminal generation before reading any raw fragments."""
    require(isinstance(status, dict) and all(field in status for field in REQUIRED_STATUS),
            "Missing trace status fields")
    for field in REQUIRED_STATUS:
        if field != "target_acceptance":
            unsigned(status[field])
    require(status["valid"] == "1" and status["terminal"] == "1" and
            status["schema"] == "1" and status["active"] == "0" and status["frozen"] == "1" and
            unsigned(status["generation"]) != 0, "Trace is unavailable, active or unfrozen")
    require(0 < unsigned(status["count"]) <= MAX_RECORDS and
            unsigned(status["record_size"]) == 96 and unsigned(status["header_size"]) == 832,
            "Trace count or wire sizes exceed supported bounds")
    require(status["target_acceptance"] == "unknown", "Unsupported motor target-acceptance claim")
    total, count = unsigned(status["total"]), unsigned(status["count"])
    require(count == min(total, MAX_RECORDS) and unsigned(status["overflow"]) == total - count and
            unsigned(status["gaps"]) == total - count, "Trace ring count/overflow/gap metadata is inconsistent")
    return status


def read_blob(exchange, generation, index, length):
    """Read exact ordered fragments bound to the same generation/index and total size."""
    fragments = []
    parts = (length + FRAGMENT_BYTES - 1) // FRAGMENT_BYTES
    for part in range(parts):
        response = exchange(f"static trace index={index} part={part} generation={generation}")
        require(all(field in response for field in ("valid", "generation", "index", "part", "parts", "bytes", "hex")),
                "Missing trace fragment field")
        expected_bytes = min(FRAGMENT_BYTES, length - part * FRAGMENT_BYTES)
        require(response["valid"] == "1" and unsigned(response["generation"]) == generation and unsigned(response["index"]) == index and
                unsigned(response["part"]) == part and unsigned(response["parts"]) == parts and
                unsigned(response["bytes"]) == expected_bytes, "Trace fragment identity or length mismatch")
        encoded = response["hex"]
        require(isinstance(encoded, str) and len(encoded) == 2 * expected_bytes and
                re.fullmatch(r"[0-9a-fA-F]+", encoded), "Invalid raw trace hex")
        fragments.append(bytes.fromhex(encoded))
    return b"".join(fragments)


def trace_float(value):
    """Preserve invalid sensor floats as explicit strings instead of emitting invalid JSON numbers."""
    if math.isfinite(value):
        return value
    return "NaN" if math.isnan(value) else ("+Inf" if value > 0 else "-Inf")


def decode_record(payload, epoch, request_id, expected_serial=None):
    """Decode the fixed 96-byte ABI and require the retained original request identity."""
    require(len(payload) == 96 and payload[86:] == bytes(10), "Wrong record ABI or nonzero reserved bytes")
    values = struct.unpack("<QQ6I7fHH6B8s10s", payload)
    names = ("timestamp_us", "observed_us", "serial", "feedback_generation", "epoch", "request_id",
             "sequence", "fault", "target_rad", "position_rad", "velocity_rad_s", "torque_nm", "kp", "kd",
             "torque_ff_nm", "can_id", "sof", "event", "phase", "motor_id", "driver_state", "length", "sof_valid")
    result = dict(zip(names, values[:23]))
    require(result["epoch"] == epoch and result["request_id"] == request_id and
            (expected_serial is None or result["serial"] == expected_serial), "Borrowed or reordered trace record")
    require(result["event"] in EVENT_NAMES and result["phase"] in PHASE_NAMES and
            result["motor_id"] <= (15 if result["event"] == 6 else 7) and
            result["length"] <= 8 and result["sof_valid"] in (0, 1),
            "Invalid trace event/phase/axis/frame metadata")
    result["event_name"] = EVENT_NAMES[result["event"]]
    result["phase_name"] = PHASE_NAMES[result["phase"]]
    if result["event"] == 8:
        result["protocol_error"] = result["fault"] & 0xffff
        result["local_guard_reason"] = result["fault"] >> 16
    elif result["event"] == 9:
        result["result_code"] = result["fault"] >> 16
        result["result_detail"] = result["fault"] & 0xffff
    elif result["event"] in (6, 7):
        result["driver_fault_flags_snapshot"] = result["fault"]
    result["can_raw_hex"] = values[23][:result["length"]].hex()
    result["feedback_fields_source"] = "application snapshot at append; raw_rx precedes decode"
    for name in ("target_rad", "position_rad", "velocity_rad_s", "torque_nm", "kp", "kd", "torque_ff_nm"):
        result[name] = trace_float(result[name])
    return result


def decode_header(payload, status):
    """Validate the 832-byte header, seven frozen axes and permanent fault/terminal copies."""
    require(len(payload) == 832, "Wrong trace header ABI")
    values = struct.unpack_from("<12IQ6I", payload)
    names = ("schema", "generation", "count", "total", "overflow", "gaps", "active", "frozen",
             "first_fault", "terminal", "record_size", "header_size", "started_us", "epoch", "request_id",
             "motor_mask", "target_acceptance", "sof_clock", "cleanup_outcome")
    result = dict(zip(names, values))
    for name in ("schema", "generation", "count", "total", "overflow", "gaps", "active", "frozen",
                 "first_fault", "terminal", "record_size", "header_size"):
        require(result[name] == unsigned(status[name]), "Header/status identity or metadata mismatch")
    require(result["epoch"] and result["request_id"] and 0 < result["motor_mask"] <= 0x7f and
            result["target_acceptance"] == 0 and result["cleanup_outcome"] in (0, 1, 2) and result["sof_clock"] == 1,
            "Unsupported or malformed header identity/clock/target-acceptance state")
    result["cleanup_outcome_name"] = {0: "not_applicable_or_unverified", 1: "new_disabled_stationary_observed",
                                      2: "timeout_unknown"}[result["cleanup_outcome"]]
    axes = []
    axis_names = ("reference", "evidence", "feedback_generation", "esc_id", "master_id", "mode",
                  "target_rad", "position_ratio", "kp", "kd", "torque_ff_nm", "range_position_rad",
                  "range_velocity_rad_s", "range_torque_nm", "minimum_rad", "maximum_rad", "maximum_speed_rad_s")
    for index in range(7):
        fields = struct.unpack_from("<6I11f3I", payload, 80 + index * 80)
        require(fields[-3:] == (0, 0, 0), "Unsupported nonzero axis reserved bytes")
        axis = dict(zip(axis_names, fields[:17]))
        for name in axis_names[6:]:
            axis[name] = trace_float(axis[name])
        axis["motor_id"] = index + 1
        axis["selected"] = bool(result["motor_mask"] & (1 << index))
        axes.append(axis)
    result["axes"] = axes
    if result["first_fault"]:
        result["first_fault_record"] = decode_record(payload[640:736], result["epoch"], result["request_id"])
        require(result["first_fault_record"]["event"] == 8, "Wrong permanent first-fault record")
    else:
        require(payload[640:736] == bytes(96), "Unexpected unmarked first-fault bytes")
        result["first_fault_record"] = None
    result["terminal_record"] = decode_record(payload[736:832], result["epoch"], result["request_id"])
    require(result["terminal_record"]["event"] == 9 and result["terminal_record"]["serial"] == result["total"] - 1,
            "Missing or inconsistent permanent terminal record")
    require(result["first_fault_record"] is None or result["first_fault_record"]["serial"] < result["total"],
            "Permanent first-fault record exceeds the observed trace")
    return result


def decode_wire_payload(record, axes):
    """Decode original MIT bytes using frozen ranges, retaining the internal-acceptance uncertainty."""
    motor_id = record["motor_id"]
    if not 1 <= motor_id <= 7 or record["length"] != 8:
        return None
    axis = axes[motor_id - 1]
    ranges = [axis[name] for name in ("range_position_rad", "range_velocity_rad_s", "range_torque_nm")]
    if not all(isinstance(value, (int, float)) and math.isfinite(value) and value > 0 for value in ranges):
        return dict(decoded=False, reason="Frozen ranges unavailable")
    raw = bytes.fromhex(record["can_raw_hex"])
    position_max, velocity_max, torque_max = ranges

    def signed_range(encoded, maximum, codes):
        """Apply the same unsigned-to-symmetric mapping as the production S3519 codec."""
        return encoded * (2.0 * maximum) / codes - maximum

    if record["event"] in (6, 7):
        return dict(kind="raw_feedback", position_rad=signed_range((raw[1] << 8) | raw[2], position_max, 65535),
            velocity_rad_s=signed_range((raw[3] << 4) | (raw[4] >> 4), velocity_max, 4095),
            torque_nm=signed_range(((raw[4] & 15) << 8) | raw[5], torque_max, 4095),
            motor_id_nibble=raw[0] & 15, driver_state_nibble=raw[0] >> 4,
            mos_temperature_c=raw[6], rotor_temperature_c=raw[7],
            accepted_by_application=record["event"] == 7)
    if record["event"] in (2, 3, 4, 5, 10, 12):
        if raw[:7] == b"\xff" * 7 and raw[7] in (0xfc, 0xfd, 0xfe):
            return dict(kind="mode_command", command={0xfc: "enable", 0xfd: "disable", 0xfe: "zero"}[raw[7]],
                        actual_tx_proven=record["event"] == 4)
        return dict(kind="mit_command", position_desired_rad=signed_range((raw[0] << 8) | raw[1], position_max, 65535),
            velocity_desired_rad_s=signed_range((raw[2] << 4) | (raw[3] >> 4), velocity_max, 4095),
            kp_encoded=(((raw[3] & 15) << 8) | raw[4]) * 500.0 / 4095,
            kd_encoded=((raw[5] << 4) | (raw[6] >> 4)) * 5.0 / 4095,
            torque_ff_encoded_nm=signed_range(((raw[6] & 15) << 8) | raw[7], torque_max, 4095),
            actual_tx_proven=record["event"] == 4, target_acceptance="unknown")
    return None


def capture(exchange):
    """Collect a frozen trace once, rejecting boot, identity or terminal-header changes."""
    before_boot = exchange("ping")
    require("boot" in before_boot, "Missing boot identity")
    boot = unsigned(before_boot["boot"])
    status = checked_status(exchange("static trace_status"))
    generation = unsigned(status["generation"])
    header = read_blob(exchange, generation, HEADER_INDEX, unsigned(status["header_size"]))
    decoded_header = decode_header(header, status)
    records = []
    for index in range(unsigned(status["count"])):
        payload = read_blob(exchange, generation, index, unsigned(status["record_size"]))
        expected_serial = index if index < 64 or unsigned(status["total"]) <= 128 else unsigned(status["total"]) - 128 + index
        decoded = decode_record(payload, decoded_header["epoch"], decoded_header["request_id"], expected_serial)
        require(decoded["serial"] < unsigned(status["total"]), "Record exceeds the observed trace total")
        if decoded["event"] == 8:
            require(decoded_header["first_fault"] == 1 and payload == header[640:736],
                    "Unmarked or inconsistent ring first-fault event")
        if decoded["event"] == 9:
            require(decoded["serial"] == unsigned(status["total"]) - 1 and payload == header[736:832],
                    "Extra or inconsistent ring terminal event")
        decoded["wire"] = decode_wire_payload(decoded, decoded_header["axes"])
        records.append(dict(index=index, raw_hex=payload.hex(), sha256=hashlib.sha256(payload).hexdigest(), decoded=decoded))
    require(bytes.fromhex(records[-1]["raw_hex"]) == header[736:832], "Ring terminal differs from permanent terminal")
    if decoded_header["first_fault_record"] is not None:
        fault_serial = decoded_header["first_fault_record"]["serial"]
        for record in records:
            if record["decoded"]["serial"] == fault_serial:
                require(bytes.fromhex(record["raw_hex"]) == header[640:736], "Ring fault differs from permanent first-fault")
    require(exchange("static trace_status") == status, "Terminal trace snapshot changed while reading")
    after_boot = exchange("ping")
    require("boot" in after_boot and unsigned(after_boot["boot"]) == boot, "Boot changed while reading")
    return dict(schema="static-startup-capture-v1", captured_at_utc=datetime.now(timezone.utc).isoformat(),
        boot=boot, generation=generation, status=status, header_hex=header.hex(), decoded_header=decoded_header,
        header_sha256=hashlib.sha256(header).hexdigest(), records=records,
        continuous_coverage_claimed=False, hardware_target_acceptance="unknown", motor_commands_sent=0,
        units=dict(position="native rad", velocity="native rad/s", torque="feedback N m",
                   timestamps="host/app observation; CAN SOF is bus order, not motor sample time"))


class ReadOnlyTraceClient:
    """Exchange numbered passive queries on a caller-owned serial connection."""

    def __init__(self, port, clock=time.perf_counter):
        """Retain the existing port and a bounded monotonic deadline source."""
        self.port = port
        self.clock = clock
        self.request_id = 1000
        self.transcript = []

    def exchange(self, command):
        """Issue one query without retry, rejecting errors and malformed matching responses."""
        kind = query_kind(command)
        self.request_id += 1
        request_id = str(self.request_id)
        payload = f"{request_id} {command}\n".encode("ascii")
        self.transcript.append(dict(direction="tx", text=payload.decode("ascii").strip()))
        require(self.port.write(payload) == len(payload), "Partial query write")
        deadline = self.clock() + 2.0
        pending = bytearray()
        while self.clock() < deadline:
            chunk = self.port.readline()
            if not chunk:
                continue
            pending.extend(chunk)
            require(len(pending) <= 2048, "Overlong protocol reply")
            if not pending.endswith(b"\n"):
                continue
            try:
                line = bytes(pending).decode("ascii").strip()
            except UnicodeError as error:
                raise RuntimeError("Non-ASCII trace reply") from error
            pending.clear()
            self.transcript.append(dict(direction="rx", text=line))
            tokens = line.split(" ", 2)
            if len(tokens) < 2 or tokens[1] != request_id:
                continue
            require(len(tokens) == 3 and tokens[0] == "ok" and tokens[2].startswith(kind + " "),
                    "Query returned an error or mismatched operation")
            return parse_fields(tokens[2][len(kind) + 1:])
        raise RuntimeError("Trace query timed out; no automatic retry was sent")


def capture_hardware():
    """Identify exactly one reviewed USB board, keep DTR/RTS low and perform passive reads only."""
    import serial
    from serial.tools import list_ports
    devices = [port for port in list_ports.comports() if port.serial_number == USB_SERIAL]
    require(len(devices) == 1, "Expected exactly one matching reviewed USB serial identity")
    port = serial.Serial(port=None, baudrate=115200, timeout=0.004, write_timeout=0.5)
    port.dtr = False
    port.rts = False
    port.port = devices[0].device
    try:
        port.open()
        client = ReadOnlyTraceClient(port)
        result = capture(client.exchange)
        result["usb_serial"] = USB_SERIAL
        result["port"] = port.port
        result["transcript"] = client.transcript
        return result
    finally:
        port.close()


def main(argv=None):
    """Capture only on explicit request and create a new evidence file without overwriting history."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", action="store_true", required=True,
                        help="Read only an already frozen trace; never start a motor action")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    require(not arguments.output.exists(), "Evidence file already exists")
    result = capture_hardware()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(dict(output=str(arguments.output.resolve()), generation=result["generation"],
                         records=len(result["records"]), motor_commands_sent=0), ensure_ascii=False))


if __name__ == "__main__":
    main()
