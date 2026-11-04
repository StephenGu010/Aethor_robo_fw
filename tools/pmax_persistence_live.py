"""Fixed ID7 persistence wrapper: offline by default, with separate explicit cold Save and cold verification actions.

Live access verifies the pinned running image and exact USB/SWD pair without halt
or reset. The user confirms motor24V cold cycling; that is not measured by this
tool. One ordinary DISABLE establishes entry feedback, then only the fixed typed
maintenance client runs. A retained task-wide token forbids an automatic Save retry.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
import time

from pmax_persistence_client import ClientInputs, PersistenceClient
from pmax_persistence_evidence import validate_observation_records, validate_snapshot
from static_recovery_release import BASE, ROOT, prepare_release, shared

ONCE_DIRECTORY = BASE / "persistence_once"


class SerialExchange:
    """Send one numbered command without retry and retain every bounded raw reply, including unmatched telemetry."""

    def __init__(self, port, transcript: list, clock=time.perf_counter) -> None:
        """Capture an already verified CDC port; constructing the adapter performs no I/O."""
        self.port, self.transcript, self.clock = port, transcript, clock
        self.disable_count = 0

    def exchange(self, request_id: int, command: str) -> str:
        """Bound one write and at most two seconds of reads; preserve LF/CRLF exactly for the client parser."""
        shared.require(type(request_id) is int and 0 < request_id <= 0xFFFFFFFF and
                       (command == "hello" or command in shared.ALLOWED_COMMANDS or re.fullmatch(
                           r"adrc persist_(?:begin motor=7|status|health id=\d+|item id=\d+ phase=\d+ index=\d+|prepare id=\d+|verify id=\d+|commit id=\d+|save_receipt id=\d+|release id=\d+)", command)),
                       "Command outside fixed maintenance wrapper")
        if command == "bench disable 7":
            shared.require(self.disable_count == 0, "A second DISABLE is forbidden")
            self.disable_count += 1
        wire = f"{request_id} {command}\r\n".encode("ascii")
        started = self.clock()
        self.transcript.append(dict(direction="tx", at=started, raw=wire.decode("ascii")))
        shared.require(self.port.write(wire) == len(wire), "Short serial write; command is not replayed")
        pending = bytearray()
        while self.clock() - started < 2.0:
            payload = self.port.read_until(b"\n", size=520 - len(pending))
            if not payload:
                continue
            self.transcript.append(dict(direction="rx_chunk", at=self.clock(), raw_hex=payload.hex()))
            pending.extend(payload)
            shared.require(len(pending) < 520, "Oversized serial line")
            if not pending.endswith(b"\n"):
                continue
            line = pending.decode("ascii", errors="strict")
            self.transcript.append(dict(direction="rx", at=self.clock(), raw=line))
            pending.clear()
            if re.match(rf"^(?:ok|error) {request_id} ", line):
                return line
        raise TimeoutError("No matching serial reply; command is not replayed")


class PreflightProtocol:
    """Reuse the audited idle/disabled gates while preserving every bounded original CDC byte."""

    def __init__(self, transport: SerialExchange, lines: list, *, first_request_id: int | None = None) -> None:
        """Allocate a separate nonwrapping preflight counter; construction sends no command."""
        self.transport, self.lines = transport, lines
        self.sequence = (3000000000 + (time.time_ns() // 1000000) % 900000000
                         if first_request_id is None else first_request_id)
        shared.require(type(self.sequence) is int and 0 < self.sequence < 0xFFFFFFFF,
                       "Invalid preflight request counter")

    def exchange(self, command: str) -> tuple[dict, float, float]:
        """Send one exact allowlisted query or single DISABLE and reject malformed matching replies."""
        shared.require(command in shared.ALLOWED_COMMANDS and self.sequence < 0xFFFFFFFF,
                       "Invalid preflight command or exhausted request counter")
        self.sequence += 1
        sent = self.transport.clock()
        reply = self.transport.exchange(self.sequence, command)
        received = self.transport.clock()
        self.lines.append(dict(command=command, request_id=self.sequence, sent=sent, received=received, raw=reply))
        shared.require(reply.endswith("\n"), "Incomplete preflight reply")
        body = reply[:-2] if reply.endswith("\r\n") else reply[:-1]
        shared.require("\r" not in body and "\n" not in body, "Embedded preflight line ending")
        response_command = {"show motor 7": "show motor", "adrc gate motor=7": "adrc gate",
                            "bench disable 7": "bench disable"}.get(command, command)
        prefix = f"ok {self.sequence} {response_command} "
        shared.require(body.startswith(prefix), "Serial error or unexpected reply command")
        fields = {}
        for key, value in re.findall(r"(?:^|\s)([a-zA-Z_][a-zA-Z_0-9]*)=([^\s]+)", body[len(prefix):]):
            shared.require(key not in fields, "Duplicate response field")
            fields[key] = value
        return fields, sent, received


def verify_running_image(target, hardware, prepared, protocol, expected_boot: str) -> int:
    """Read each occupied Flash byte without halting, keeping the original session alive with bounded read-only ping."""
    verified = 0
    last_ping = time.perf_counter()
    for address, payload in prepared.new_segments:
        for offset in range(0, len(payload), 4096):
            if time.perf_counter() - last_ping >= 0.2:
                fields, _, _ = protocol.exchange("ping")
                shared.fields_equal(fields, {"boot": expected_boot, "enabled": "00"}, "Boot changed during Flash verification")
                last_ping = time.perf_counter()
            part = payload[offset:offset + 4096]
            shared.require(target.get_state() == hardware.Target.State.RUNNING and
                           shared.read_flash(target, address + offset, address + offset + len(part)) == part and
                           target.get_state() == hardware.Target.State.RUNNING, "Running Flash or core state mismatch")
            verified += len(part)
    return verified


def verify_attached_board(hardware, prepared, protocol, expected_boot: str, report: dict) -> None:
    """Read the exact running board once and retain close errors without hiding the original verification failure."""
    probes = [probe for probe in hardware.probes() if probe.unique_id == shared.PROBE_UID]
    shared.require(len(probes) == 1, "Exact unique probe UID not found")
    session = hardware.Session(probes[0], options=dict(shared.SESSION_OPTIONS))
    primary_error = None
    try:
        session.open()
        shared.require(session.probe.unique_id == shared.PROBE_UID, "Opened probe UID changed")
        target = session.target
        shared.require(target.get_state() == hardware.Target.State.RUNNING and
                       target.read32(0x5C001000) & 0xFFF == 0x483, "Wrong MCU or core is not RUNNING")
        shared.verify_usb_swd_pair(target, report)
        report["installed_verified_bytes"] = verify_running_image(target, hardware, prepared, protocol, expected_boot)
    except BaseException as error:
        primary_error = error
        raise
    finally:
        try:
            session.close()
        except BaseException as error:
            report["swd_close_error"] = f"{type(error).__name__}: {error}"
            if primary_error is None:
                raise


def load_cold_baseline(path: Path, checksum: str, manifest: dict, manifest_sha256: str) -> list:
    """Validate the pinned first cold record before any connection; original raw identity/time validation stays in the client."""
    shared.require(type(checksum) is str and re.fullmatch(r"[0-9a-f]{64}", checksum) is not None,
                   "Cold baseline requires its exact lowercase SHA-256")
    payload = shared.bounded_path(path, ROOT).read_bytes()
    shared.require(shared.digest(payload) == checksum, "Original cold baseline changed")
    baseline = shared.read_json(payload)
    shared.require(baseline.get("schema") == "id7-cold-baseline-v1" and baseline.get("user_confirmed_motor24v_cold") is True and
                   baseline.get("usb_serial") == shared.USB_SERIAL and baseline.get("hex_sha256") == manifest["hex_sha256"] and
                   baseline.get("release_manifest_sha256") == manifest_sha256 and isinstance(baseline.get("records"), list),
                   "Wrong first-cold baseline identity")
    records = baseline["records"]
    validate_snapshot(records)
    first_identity = records[0].get("identity")
    shared.require(isinstance(first_identity, dict), "Original cold baseline has no observation identity")
    owner = {field: first_identity.get(field) for field in ("boot_epoch", "request_id", "owner_generation")}
    final_us = max(record[field] for record in records for field in ("send_completed_observed_us", "received_us"))
    validate_observation_records(records, expected_owner=owner, expected_phase=1, now_us=final_us)
    return records


def execute(prepared, manifest: dict, manifest_sha256: str, original_baseline: list | None,
            *, supported_cold_confirmed: bool = False) -> dict:
    """Run one explicitly authorized no-motion session; every failure retains evidence and leaves the fixed token untouched."""
    shared.require(supported_cold_confirmed is True, "Current support, unloaded, LCD release, motor power and cold-cycle confirmations required")
    output = BASE / ("persistence_live_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    output.mkdir(parents=True, exist_ok=False)
    serial_port = None
    client = None
    raw_lines, preflight = [], []
    report = dict(hardware_qualified=False, load_qualified=False, motion_commands_sent=0,
                  cold_cycle="user_confirmation_only", release_manifest_sha256=manifest_sha256,
                  hex_sha256=manifest["hex_sha256"], action="cold_verify" if original_baseline is not None else "cold_prepare_save",
                  completed=False, error=None)
    try:
        shared.require(shared.bounded_path(ONCE_DIRECTORY, ROOT, require_file=False).is_dir(), "Fixed once directory is not a real directory")
        hardware = shared.load_hardware()
        ports = [item for item in hardware.ports() if item.vid == 0x0483 and item.pid == 0x5740 and item.serial_number == shared.USB_SERIAL]
        shared.require(len(ports) == 1, "Exact unique controller USB identity not found")
        serial_port = hardware.Serial(port=None, baudrate=115200, timeout=0.004, write_timeout=0.5)
        serial_port.dtr = False; serial_port.rts = False; serial_port.port = ports[0].device; serial_port.open()
        transport = SerialExchange(serial_port, raw_lines)
        protocol = PreflightProtocol(transport, preflight)
        gate, _, _ = protocol.exchange("adrc gate motor=7")
        mode = gate.get("mode")
        shared.require(mode in ("1", "2"), "Only a verified MIT or POS source mode is allowed")
        boot = shared.idle_snapshot(protocol, expected_mode=mode)
        verify_attached_board(hardware, prepared, protocol, boot, report)
        shared.recheck_frozen(prepared)
        shared.require(shared.idle_snapshot(protocol, expected_mode=mode) == boot, "Boot changed after read-only verification")
        hello = transport.exchange(900000001, "hello")
        shared.require(re.match(r"^ok 900000001 hello ", hello) is not None and
                       re.search(rf"(?:^| )boot={boot}(?: |\r?\n$)", hello) is not None, "Unexpected HELLO or boot identity")
        feedback = shared.disable_once(protocol, time.perf_counter, time.sleep)
        shared.wait_post_disable_idle(protocol, boot, feedback, time.perf_counter, time.sleep, expected_mode=mode)
        shared.require(shared.feedback_age_upper_ms(feedback, time.perf_counter) <= 100, "Fresh disabled entry expired")
        report["disabled_entry"] = feedback
        first_request = 1000000000 + (time.time_ns() // 1000000) % 1000000000
        inputs = ClientInputs(manifest["hex_sha256"], manifest_sha256, shared.USB_SERIAL, int(boot), True)
        client = PersistenceClient(inputs, ONCE_DIRECTORY, transport.exchange, clock=time.perf_counter, first_request_id=first_request)
        result = client.run_readonly_verify(original_baseline) if original_baseline is not None else client.run_prepare_save()
        report["result"] = result; report["completed"] = True
        if original_baseline is None:
            baseline = dict(schema="id7-cold-baseline-v1", usb_serial=shared.USB_SERIAL,
                            user_confirmed_motor24v_cold=True, hex_sha256=manifest["hex_sha256"],
                            release_manifest_sha256=manifest_sha256, records=client.evidence["baseline"])
            baseline_payload = (json.dumps(baseline, indent=2) + "\n").encode("utf-8")
            baseline_path = output / "cold_baseline.json"
            with baseline_path.open("xb") as stream:
                shared.require(stream.write(baseline_payload) == len(baseline_payload), "Short baseline evidence write")
            report["cold_baseline"] = str(baseline_path); report["cold_baseline_sha256"] = shared.digest(baseline_payload)
        return report
    except BaseException as error:
        report["completed"] = False
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        cleanup_error = None
        if serial_port is not None:
            try:
                serial_port.close()
            except BaseException as error:
                cleanup_error = error
                report["completed"] = False
                report["close_error"] = f"{type(error).__name__}: {error}"
        report["preflight"] = preflight; report["raw_serial"] = raw_lines
        report["client_transcript"] = client.transcript if client is not None else []
        report["client_evidence"] = client.evidence if client is not None else {}
        report_write_error = None
        try:
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        except BaseException as error:
            report_write_error = error
            report["completed"] = False
            report["report_write_error"] = f"{type(error).__name__}: {error}"
        try:
            if report_write_error is None:
                print("PMAX_PERSISTENCE_EVIDENCE", output, flush=True)
            else:
                print("PMAX_PERSISTENCE_REPORT_WRITE_FAILED", output,
                      "original=" + str(report["error"]), "close=" + str(report.get("close_error")),
                      "report=" + report["report_write_error"], file=sys.stderr, flush=True)
        except BaseException:
            # Console cleanup is best effort; it cannot replace a device, close or evidence-write failure.
            pass
        if report["error"] is None:
            if cleanup_error is not None:
                raise cleanup_error
            if report_write_error is not None:
                raise report_write_error


def main(argv=None) -> int:
    """Keep all hardware imports/connections behind one explicit action plus the current physical confirmations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--save", action="store_true")
    actions.add_argument("--cold-baseline", type=Path)
    parser.add_argument("--baseline-sha256")
    parser.add_argument("--cold-motor-power-cycle-confirmed", action="store_true")
    parser.add_argument("--supported-unloaded", action="store_true")
    parser.add_argument("--lcd-released", action="store_true")
    parser.add_argument("--motor-power-on", action="store_true")
    args = parser.parse_args(argv)
    live = args.save or args.cold_baseline is not None
    if live and not (args.cold_motor_power_cycle_confirmed and args.supported_unloaded and args.lcd_released and args.motor_power_on):
        parser.error("live maintenance requires all current physical and cold-cycle confirmation flags")
    if bool(args.cold_baseline) != bool(args.baseline_sha256):
        parser.error("cold verification requires both the original baseline path and its SHA-256")
    prepared, manifest = prepare_release(args.manifest, args.manifest_sha256)
    original = load_cold_baseline(args.cold_baseline, args.baseline_sha256, manifest, args.manifest_sha256) if args.cold_baseline else None
    print("PMAX_PERSISTENCE_OFFLINE_FROZEN", shared.digest(prepared.image_payload), flush=True)
    if live:
        execute(prepared, manifest, args.manifest_sha256, original, supported_cold_confirmed=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
