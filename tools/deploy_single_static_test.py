"""Verify and explicitly deploy LCD-SingleStatic-Test with one fresh DISABLE.

The default CLI is artifact-only. Serial and pyOCD imports and device discovery
exist exclusively behind --deploy plus both physical confirmation flags. No
motion, mode, tuning, command retry, resume, rollback or automatic recovery is supported.
The old HEX is a candidate until every occupied byte matches RUNNING flash.
Every preflight snapshot must retain the caller's explicitly expected current mode.
"""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import struct
import subprocess
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
TARGET_NAME = "LCD-SingleStatic-Test"
CONTROL_BASELINE = "d333d276639420637a056f15903e45dcba323d90"
RTOS_BASELINE = "a9447d70e11a7b80820f822ab5c000cc9f420051"
OLD_RELATIVE = "output/static_startup_sync_20261020/hardware_prepare/expected_c333_LCD-SingleStatic-Test.hex"
OLD_SHA256 = "c333079f935117a0ff3d0260280f4a07dc9f87e5a626a30c8c67ee11392602dc"
PROBE_UID = "07000001000000000000000000000000a5a5a5a597969908"
USB_SERIAL = "386B336F3134"
MCU_UID_ADDRESSES = (0x1FF1E800, 0x1FF1E804, 0x1FF1E808)
FLASH_START = 0x08000000
FLASH_END = 0x08100000
# Windows monotonic may use 15.625 ms GetTickCount64; perf_counter uses QPC.
# Reserve its actual resolution in addition to the firmware integer-age floor.
HOST_CLOCK_RESOLUTION_MS = time.get_clock_info("perf_counter").resolution * 1000
SESSION_OPTIONS = {"target_override": "stm32h723xx", "connect_mode": "attach",
    "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
    "frequency": 500000, "cmsis_dap.limit_packets": True,
    "cmsis_dap.deferred_transfers": False, "cache.enable_memory": False,
    "cache.read_code_from_elf": False}
REQUIRED_TARGETS = ("CtrBoard-H7_FDCAN", "LCD-ReadOnly", "LCD-POS", "LCD-MIT",
                    "ADRC-Bench", "LCD-MIT-ADRC", TARGET_NAME)
REQUIRED_RECORDS = ("lcd_suite", "load_trials", "load_trial_pages", "single_static_build_tests",
    "single_static_deploy_tests", "keil_ADRC-Bench", "keil_LCD-MIT-ADRC",
    "keil_LCD-SingleStatic-Test", "map_single_static", "single_static_platform")
REQUIRED_LCD_CHECKS = ("check_phase0_architecture", "test_architecture_publication_check",
    "check_debug_ui_architecture", "run_phase0_tests", "run_text_protocol_tests",
    "run_text_protocol_engine_tests", "run_text_protocol_arm_profile_tests",
    "run_motor_core_tests", "run_motion_tests", "run_platform_io_tests", "run_protocol_tests",
    "run_simulator_tests", "run_debug_ui_input_tests", "run_lcd_transport_tests",
    "run_debug_ui_mailbox_tests", "run_debug_ui_app_tests", "run_debug_ui_model_tests",
    "run_debug_ui_soak_tests", "run_static_load_tests", "run_single_static_load_tests",
    "run_single_static_lcd_config_tests", "run_static_lcd_workflow_tests",
    "test_debug_ui_build_contract", "check_numeric_font_subset", "run_input_model_integration",
    "run_ui_render_smoke", "run_ui_review_regressions", "keil_CtrBoard-H7_FDCAN",
    "keil_LCD-ReadOnly", "keil_LCD-POS", "keil_LCD-MIT", "check_debug_ui_map")
CURRENT_TARGETS = ("ADRC-Bench", "LCD-MIT-ADRC", TARGET_NAME)
CURRENT_LCD_CHECKS = tuple(name for name in REQUIRED_LCD_CHECKS
    if not name.startswith("keil_") and name != "check_debug_ui_map")
CURRENT_RECORDS = REQUIRED_RECORDS + ("startup_trace_export_tests", "startup_sync_tests", "map_current_targets", "trace_map_tests")
REQUIRED_SOURCES = frozenset(("MDK-ARM/LCD-SingleStatic-Test.uvprojx", "MDK-ARM/aethor_memory.sct",
    "App/Config/debug_ui_config.h", "Core/Inc/FreeRTOSConfig.h", "Core/Src/freertos.c",
    "App/DebugUi/debug_ui_model.c", "App/DebugUi/debug_ui_model.h",
    "App/DebugUi/debug_ui_load_trials.h", "App/DebugUi/debug_ui_load_trials.inc",
    "App/DebugUi/debug_ui_load_trial_pages.inc", "App/DebugUi/debug_ui_single_static_flow.inc",
    "App/DebugUi/debug_ui_single_static_config_app.inc", "App/DebugUi/debug_ui_static_load_app.inc",
    "App/aethor_app.c", "tools/build_single_static_test.ps1", "tools/deploy_single_static_test.py",
    "Tests/host/test_single_static_build.py", "Tests/hardware/test_single_static_deploy.py",
    "App/DebugUi/debug_ui_contract.h", "App/DebugUi/debug_ui_single_static_tx.h", "App/aethor_app.h",
    "App/Platform/stm32_platform.c", "App/Platform/stm32_platform.h", "App/Motor/motor_bank.c",
    "App/Motor/motor_bank.h", "App/Protocol/protocol_engine.c", "App/Protocol/protocol_engine.h",
    "Tests/host/static_lcd_recovery_cases.inc", "Tests/host/single_static_platform_test.c",
    "Tests/host/single_static_platform_fakes/fdcan.h",
    "Tests/host/single_static_platform_fakes/usbd_cdc_if.h",
    "Tests/host/run_single_static_platform_tests.ps1"))
SOURCE_ROOTS = (".gitignore", "App", "Core", "Ui", "Drivers", "Middlewares", "USB_DEVICE",
    "Tests/host", "Tests/ui", "Tests/hardware", "tools", "Models", "MDK-ARM", "CtrBoard-H7_FDCAN.ioc")
SOURCE_SUFFIXES = frozenset((".c", ".h", ".inc", ".s", ".m", ".ps1", ".py", ".json", ".txt",
                             ".uvprojx", ".sct", ".ioc"))
ALLOWED_COMMANDS = frozenset(("show state", "ping", "adrc status", "adrc gate motor=7",
                              "show diag can", "show motor 7", "bench disable 7"))


@dataclass(frozen=True)
class FrozenFile:
    """Retain the exact immutable bytes accepted at one bounded file path."""

    path: Path
    payload: bytes


@dataclass(frozen=True)
class Prepared:
    """Bind all accepted inputs, flash segments and deployment payloads immutably."""

    workspace: Path
    manifest_path: Path
    manifest_payload: bytes
    old_payload: bytes
    image_payload: bytes
    elf_payload: bytes
    old_segments: tuple
    new_segments: tuple
    files: tuple


class CanQuietPending(RuntimeError):
    """Signal only a complete, otherwise safe post-DISABLE snapshot with CAN busy."""


def digest(payload):
    """Return a lowercase SHA256 digest of immutable bytes."""
    return hashlib.sha256(payload).hexdigest()


def require(condition, message):
    """Reject an unsafe or incomplete contract without relying on optimized assertions."""
    if not condition:
        raise RuntimeError(message)


def bounded_path(path, root, require_file=True):
    """Reject traversal and every symlink/reparse component before resolving a path."""
    candidate = Path(path)
    require(".." not in candidate.parts, f"Traversal rejected: {path}")
    candidate = candidate if candidate.is_absolute() else Path(root) / candidate
    candidate = candidate.absolute()
    for component in (candidate, *candidate.parents):
        if not component.exists() and component == candidate and not require_file:
            continue
        metadata = component.lstat()
        require(not stat.S_ISLNK(metadata.st_mode) and
                not getattr(metadata, "st_file_attributes", 0) & 0x400,
                f"Symlink/reparse path rejected: {component}")
    resolved = candidate.resolve()
    require(resolved.is_relative_to(Path(root).resolve()), f"Path escapes boundary: {path}")
    require(not require_file or resolved.is_file(), f"Not a file: {path}")
    return resolved


def json_object(pairs):
    """Reject duplicate JSON keys rather than silently accepting an ambiguous manifest."""
    result = {}
    for key, value in pairs:
        require(key not in result, f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(payload):
    """Parse immutable UTF-8 JSON with explicit duplicate-key rejection."""
    try:
        value = json.loads(payload.decode("utf-8-sig"), object_pairs_hook=json_object)
        require(isinstance(value, dict), "Expected a JSON object")
        return value
    except (UnicodeError, ValueError, TypeError) as error:
        raise RuntimeError("Invalid JSON evidence") from error


def maintained_sources(workspace):
    """Independently enumerate the frozen gate's tracked and unignored maintained inputs."""
    result = subprocess.run(["git", "-c", "safe.directory=" + str(workspace), "-C", str(workspace),
        "ls-files", "-z", "--cached", "--others", "--exclude-standard", "--", *SOURCE_ROOTS],
        check=False, capture_output=True, timeout=30)
    require(result.returncode == 0, "Cannot enumerate complete maintained inputs")
    paths = set()
    for relative in result.stdout.decode("utf-8").split("\0"):
        if relative and "build" not in Path(relative).parts and (relative == ".gitignore" or
                Path(relative).suffix.lower() in SOURCE_SUFFIXES):
            paths.add(relative)
    require(REQUIRED_SOURCES <= paths, "Required maintained inputs are missing")
    return paths


def parse_hex(payload):
    """Parse only nonempty occupied flash into immutable nonoverlapping byte spans."""
    from intelhex import IntelHex, IntelHexError
    try:
        image = IntelHex(io.StringIO(payload.decode("ascii")))
        segments = tuple((start, bytes(image.tobinarray(start=start, size=end - start)))
                         for start, end in image.segments())
        require(segments and all(FLASH_START <= start < start + len(data) <= FLASH_END
                for start, data in segments), "HEX has empty or out-of-flash occupancy")
        return segments
    except (UnicodeError, ValueError, KeyError, IntelHexError) as error:
        raise RuntimeError("Invalid HEX payload") from error


def verify_image_pair(hex_payload, elf_payload):
    """Require exact HEX/PT_LOAD flash bytes and coverage, rejecting every overlap."""
    from elftools.elf.elffile import ELFFile
    from elftools.common.exceptions import ELFError
    spans = parse_hex(hex_payload)
    expected = {address + offset: value for address, payload in spans for offset, value in enumerate(payload)}
    actual = {}
    try:
        elf = ELFFile(io.BytesIO(elf_payload))
        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD" or not segment["p_filesz"]:
                continue
            address, size = segment["p_paddr"], segment["p_filesz"]
            require(FLASH_START <= address < address + size <= FLASH_END, "ELF PT_LOAD outside flash")
            payload = segment.data()
            require(len(payload) == size, "Truncated ELF PT_LOAD")
            for offset, value in enumerate(payload):
                location = address + offset
                require(location not in actual, "Overlapping ELF PT_LOAD coverage")
                actual[location] = value
        require(actual == expected, "ELF/HEX contents or coverage differ")
        return spans
    except (ELFError, ValueError, TypeError, struct.error) as error:
        raise RuntimeError("Invalid ELF payload") from error


def unique_records(records, required_names, key):
    """Require an exact set of uniquely identified acceptance records."""
    require(isinstance(records, list) and len(records) == len(required_names), "Missing/extra acceptance records")
    require(all(isinstance(record, dict) for record in records), "Invalid acceptance record")
    names = [record.get(key) for record in records]
    require(len(set(names)) == len(names) and set(names) == set(required_names), "Missing/duplicate acceptance name")
    return records


def utc_time(value):
    """Require an explicit UTC timestamp rather than a local or timezone-free value."""
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        require(parsed.utcoffset() is not None and parsed.utcoffset().total_seconds() == 0,
                "Acceptance timestamp is not UTC")
        return parsed
    except (ValueError, TypeError, AttributeError) as error:
        raise RuntimeError("Invalid acceptance timestamp") from error


def collect_hashes(entries, root, relative, frozen):
    """Validate every hash and freeze bounded bytes with case-insensitive duplicate rejection."""
    require(isinstance(entries, list) and entries, "Empty artifact/source hashes")
    bound = {}
    for entry in entries:
        require(isinstance(entry, dict) and isinstance(entry.get("path"), str), "Invalid hash entry")
        original = Path(entry["path"])
        require(original.is_absolute() is not relative, "Hash path has wrong absolute/relative form")
        path = bounded_path(original, root)
        identity = str(path).casefold()
        require(identity not in bound, "Duplicate hash path")
        checksum = entry.get("sha256")
        require(isinstance(checksum, str) and re.fullmatch(r"[0-9a-fA-F]{64}", checksum), "Invalid SHA256")
        payload = path.read_bytes()
        require(digest(payload) == checksum.lower(), f"Changed input/artifact: {path}")
        bound[identity] = FrozenFile(path, payload)
        frozen.append(bound[identity])
    return bound


def evidence_file(path, run, evidence):
    """Return only a retained absolute run file already bound to a verified digest."""
    require(Path(path).is_absolute(), "Evidence reference must be absolute")
    path = bounded_path(path, run)
    require(str(path).casefold() in evidence, f"Unhashed evidence reference: {path}")
    return evidence[str(path).casefold()]


def acceptance_contract(manifest):
    """Select an explicit immutable acceptance matrix without permitting arbitrary subsets."""
    if manifest.get("schema") == "single-static-acceptance-v1":
        return REQUIRED_TARGETS, REQUIRED_RECORDS, REQUIRED_LCD_CHECKS, True
    require(manifest.get("schema") == "single-static-startup-acceptance-v2" and
            manifest.get("acceptance_profile") == "current-static-startup" and
            isinstance(manifest.get("reviewed_startup_revision"), str) and
            re.fullmatch(r"[0-9a-f]{40}", manifest["reviewed_startup_revision"]),
            "Wrong acceptance schema/profile/review identity")
    return CURRENT_TARGETS, CURRENT_RECORDS, CURRENT_LCD_CHECKS, False


def validate_lcd(summary_file, run, evidence, required_checks=REQUIRED_LCD_CHECKS, keil_requested=True):
    """Inspect every required host check and its complete retained logs for the selected matrix."""
    summary = read_json(summary_file.payload)
    require(summary.get("first_party_sources_unchanged_during_verification") is True and
            summary.get("software_only") is True and summary.get("hardware_tested") is False and
            summary.get("keil_requested") is keil_requested, "LCD evidence flags failed")
    checks = unique_records(summary.get("checks"), required_checks, "name")
    for check in checks:
        require(type(check.get("exit_code")) is int and check["exit_code"] == 0, "LCD check failed")
        for field, suffix in (("stdout", ".log"), ("stderr", ".stderr.log")):
            original = check.get(field)
            require(isinstance(original, str) and Path(original).is_absolute() and
                    ".." not in Path(original).parts and Path(original).name == check["name"] + suffix,
                    "Invalid original LCD log reference")
            # verify_debug_ui.ps1 retains its original log location in summary;
            # read the actual copied sublog anchored by the final evidence hash.
            evidence_file(str(run / "lcd_evidence" / Path(original).name), run, evidence)


def prepare(manifest_path, manifest_sha256, workspace=ROOT):
    """Validate the independent schema and freeze complete sources, evidence and image bytes."""
    workspace = bounded_path(workspace, workspace, require_file=False)
    manifest_path = bounded_path(manifest_path, workspace)
    manifest_payload = manifest_path.read_bytes()
    require(isinstance(manifest_sha256, str) and re.fullmatch(r"[0-9a-fA-F]{64}", manifest_sha256)
            and digest(manifest_payload) == manifest_sha256.lower(), "Manifest anchor mismatch")
    manifest = read_json(manifest_payload)
    targets, records, lcd_checks, keil_requested = acceptance_contract(manifest)
    stack_scope = ("single-static-compile-gate-only" if manifest["schema"] == "single-static-acceptance-v1"
                   else "static-trace-and-integrated-lcd-task-stacks")
    require(manifest.get("target") == TARGET_NAME, "Wrong acceptance target")
    for field in ("passed", "software_only", "sources_unchanged", "control_sources_unchanged"):
        require(manifest.get(field) is True, f"Acceptance gate failed: {field}")
    require(manifest.get("hardware_tested") is False and "failure" in manifest and
            manifest["failure"] is None and type(manifest.get("lcd_checks")) is int and
            manifest["lcd_checks"] == len(lcd_checks), "Software-only/count/failure gate failed")
    require(manifest.get("control_baseline") == CONTROL_BASELINE and
            manifest.get("rtos_stack_baseline") == RTOS_BASELINE and
            manifest.get("rtos_stack_override_scope") == stack_scope, "Wrong source baseline")
    require(isinstance(manifest.get("run_id"), str) and re.fullmatch(r"\d{8}T\d{9,12}Z", manifest["run_id"]),
            "Missing acceptance run identity")
    require(utc_time(manifest.get("started_at_utc")) <= utc_time(manifest.get("completed_at_utc")),
            "Acceptance timestamps reversed")
    frozen = [FrozenFile(manifest_path, manifest_payload)]
    sources = collect_hashes(manifest.get("source_hashes"), workspace, True, frozen)
    source_names = {item.path.relative_to(workspace).as_posix() for item in sources.values()}
    require(REQUIRED_SOURCES <= source_names and source_names == maintained_sources(workspace),
            "Acceptance omits or adds maintained input paths")
    run = manifest_path.parent
    evidence = collect_hashes(manifest.get("evidence_hashes"), run, False, frozen)
    require(str(manifest_path).casefold() not in evidence, "Manifest must not hash itself")
    actual_evidence = set()
    for directory, directories, files in os.walk(run, followlinks=False):
        for name in directories:
            bounded_path(Path(directory) / name, run, require_file=False)
        for name in files:
            path = bounded_path(Path(directory) / name, run)
            if path != manifest_path:
                actual_evidence.add(str(path).casefold())
    require(set(evidence) == actual_evidence, "Unbound/missing file in acceptance run")
    script = bounded_path(manifest.get("verification_script_path", ""), workspace)
    require(Path(manifest.get("verification_script_path", "")).is_absolute(), "Verification script path must be absolute")
    script_payload = script.read_bytes()
    require(digest(script_payload) == str(manifest.get("verification_script_sha256", "")).lower(),
            "Verification script changed")
    frozen.append(FrozenFile(script, script_payload))
    for record in unique_records(manifest.get("records"), records, "name"):
        require(type(record.get("exit_code")) is int and record["exit_code"] == 0, "Acceptance check failed")
        for field in ("log", "stderr"):
            evidence_file(record.get(field, ""), run, evidence)
    for build in unique_records(manifest.get("builds"), targets, "target"):
        require(type(build.get("errors")) is int and build["errors"] == 0 and
                type(build.get("warnings")) is int and build["warnings"] == 0, "Unclean build count")
        artifacts = {key: evidence_file(build.get(key, ""), run, evidence) for key in
                     ("hex_path", "elf_path", "map_path", "log_path")}
        require(all(artifact.payload for artifact in artifacts.values()), "Empty build product/log")
        for key, extension in (("hex_path", ".hex"), ("elf_path", ".axf"), ("map_path", ".map")):
            require(artifacts[key].path == run / (build["target"] + extension), "Wrong build product identity")
        summaries = re.findall(r"(?<!\d)(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)(?!\d)",
                               artifacts["log_path"].payload.decode("utf-8-sig", errors="strict"))
        require(summaries and all(errors == "0" and warnings == "0" for errors, warnings in summaries),
                "Build log lacks an exact zero-error zero-warning summary")
    validate_lcd(evidence_file(str(run / "lcd_evidence/summary.json"), run, evidence), run, evidence,
                 lcd_checks, keil_requested)
    image_payload = evidence_file(str(run / (TARGET_NAME + ".hex")), run, evidence).payload
    elf_payload = evidence_file(str(run / (TARGET_NAME + ".axf")), run, evidence).payload
    new_segments = verify_image_pair(image_payload, elf_payload)
    old_path = bounded_path(workspace / OLD_RELATIVE, workspace)
    old_payload = old_path.read_bytes()
    require(digest(old_payload) == OLD_SHA256, "Old candidate path/hash mismatch")
    old_segments = parse_hex(old_payload)
    frozen.append(FrozenFile(old_path, old_payload))
    prepared = Prepared(workspace, manifest_path, manifest_payload, old_payload, image_payload,
                        elf_payload, old_segments, new_segments, tuple(frozen))
    recheck_frozen(prepared)
    return prepared


def recheck_frozen(prepared):
    """Reject input replacement, reparse substitution or content mutation after verification."""
    for item in prepared.files:
        require(bounded_path(item.path, prepared.workspace) == item.path and
                item.path.read_bytes() == item.payload, f"Frozen input changed: {item.path}")


def copy_payloads(prepared, output):
    """Create exclusive payload copies, reread them, then recheck all frozen source paths."""
    output = bounded_path(output, prepared.workspace, require_file=False)
    require(not output.exists(), "Deployment output already exists")
    output.mkdir()
    for name, payload in ((TARGET_NAME + ".hex", prepared.image_payload),
            (TARGET_NAME + ".axf", prepared.elf_payload), ("expected_previous_LCD-SingleStatic-Test.hex", prepared.old_payload),
            ("acceptance_manifest.json", prepared.manifest_payload)):
        destination = output / name
        with destination.open("xb") as stream:
            require(stream.write(payload) == len(payload), "Incomplete immutable payload copy")
        require(bounded_path(destination, output).read_bytes() == payload, "Mutable/incorrect payload copy")
    recheck_frozen(prepared)
    for name, payload in ((TARGET_NAME + ".hex", prepared.image_payload), (TARGET_NAME + ".axf", prepared.elf_payload),
            ("expected_previous_LCD-SingleStatic-Test.hex", prepared.old_payload), ("acceptance_manifest.json", prepared.manifest_payload)):
        require(bounded_path(output / name, output).read_bytes() == payload, "Copied payload changed before device access")
    return output


def load_hardware():
    """Load hardware packages only after the CLI's explicit deployment confirmation gate."""
    import serial
    from serial.tools import list_ports
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    from pyocd.flash.file_programmer import FileProgrammer
    return SimpleNamespace(ports=list_ports.comports, Serial=serial.Serial,
        probes=lambda: ConnectHelper.get_all_connected_probes(blocking=False),
        Session=Session, Target=Target, FileProgrammer=FileProgrammer)


class Protocol:
    """Permit only passive queries and one ID7 DISABLE with exact numbered replies."""

    def __init__(self, port, lines, clock):
        """Keep a unique request sequence and retain query send/receive timestamps."""
        self.port, self.lines, self.clock = port, lines, clock
        self.sequence = 3000000000 + time.time_ns() // 1000000 % 900000000
        self.disable_count = 0

    def exchange(self, command):
        """Send an allowlisted command once and return its strict matching response."""
        require(command in ALLOWED_COMMANDS, "Command not allowlisted")
        if command == "bench disable 7":
            require(self.disable_count == 0, "A second DISABLE is forbidden")
            self.disable_count += 1
        self.sequence += 1
        wire = f"{self.sequence} {command}\r\n".encode("ascii")
        sent = self.clock()
        self.lines.append({"direction": "tx", "at": sent, "line": wire.decode("ascii").strip()})
        require(self.port.write(wire) == len(wire), "Incomplete serial command write")
        while self.clock() - sent < 2.0:
            reply = self.port.readline().decode("ascii", errors="strict").strip()
            received = self.clock()
            if reply:
                self.lines.append({"direction": "rx", "at": received, "line": reply})
            if re.match(rf"^(ok|error) {self.sequence} ", reply):
                response_command = {"show motor 7": "show motor", "adrc gate motor=7": "adrc gate",
                                    "bench disable 7": "bench disable"}.get(command, command)
                prefix = f"ok {self.sequence} {response_command} "
                require(reply.startswith(prefix), "Serial error or unexpected reply command")
                fields = {}
                for key, value in re.findall(r"(?:^|\s)([a-zA-Z_][a-zA-Z_0-9]*)=([^\s]+)", reply[len(prefix):]):
                    require(key not in fields, "Duplicate response field")
                    fields[key] = value
                return fields, sent, received
        raise RuntimeError(f"Serial timeout: {command}")


def fields_equal(fields, expected, description):
    """Require exact parsed field values without substring identity ambiguities."""
    require(all(fields.get(key) == value for key, value in expected.items()), description)


def feedback_age_upper_ms(feedback, clock):
    """Bound original feedback age with query transit, integer floor and host resolution."""
    return (feedback["age_ms"] + (clock() - feedback["query_sent"]) * 1000 + 1 +
            HOST_CLOCK_RESOLUTION_MS)


def idle_snapshot(protocol, expected_boot=None, expected_mode="2"):
    """Validate mode-bound idle gates, deferring only CAN quiet after a boot-bound DISABLE."""
    state, _, _ = protocol.exchange("show state")
    fields_equal(state, {"enabled": "00", "moving": "0", "active": "0", "fault": "none"}, "Software is not idle")
    owner, _, _ = protocol.exchange("adrc status")
    fields_equal(owner, {"owner": "lcd", "fault": "0", "state": "0", "active_id": "0"}, "ADRC ownership is not idle")
    gate, _, _ = protocol.exchange("adrc gate motor=7")
    fields_equal(gate, {"motor": "7", "lcd_idle": "1", "mode": expected_mode, "fields": "1fff",
        "verified": "40", "authority": "0", "action": "0", "ui_busy": "0", "ui_results": "0",
        "proto_results": "0", "discovery_active": "0"}, "Control/discovery queues are not idle")
    require(gate.get("can_idle") in ("0", "1"), "Invalid or missing CAN can_idle field")
    require(expected_boot is not None or gate["can_idle"] == "1", "CAN is not idle before DISABLE")
    can, _, _ = protocol.exchange("show diag can")
    fields_equal(can, {"drop": "0", "error": "0", "busoff": "0"}, "CAN counter gate failed")
    boot, _, _ = protocol.exchange("ping")
    fields_equal(boot, {"enabled": "00"}, "Ping enabled gate failed")
    require(re.fullmatch(r"\d+", boot.get("boot", "")) is not None, "Missing current boot ID")
    if expected_boot is not None:
        require(boot["boot"] == expected_boot, "Boot changed during preflight")
        if gate["can_idle"] == "0":
            raise CanQuietPending("CAN quiet interval is still pending")
    return boot["boot"]


def wait_post_disable_idle(protocol, expected_boot, feedback, clock, sleep, expected_mode="2"):
    """Poll complete mode-bound read-only snapshots within the original 100 ms feedback budget."""
    started = clock()

    def require_fresh():
        """Include query transit and the firmware integer-age floor in every deadline check."""
        require(feedback_age_upper_ms(feedback, clock) <= 100,
                "Fresh disabled feedback expired while waiting for CAN quiet")

    for snapshot_index in range(20):
        require_fresh()
        try:
            idle_snapshot(protocol, expected_boot=expected_boot, expected_mode=expected_mode)
        except CanQuietPending:
            require_fresh()
        else:
            require_fresh()
            feedback["can_quiet_snapshot_count"] = snapshot_index + 1
            feedback["can_quiet_wait_ms"] = (clock() - started) * 1000
            return
        if snapshot_index < 19:
            sleep(0.004)
    raise RuntimeError("CAN did not become quiet within 20 read-only snapshots")


def motor_feedback(protocol):
    """Read ID7 and its raw driver state/fault without causing a motor request."""
    fields, sent, received = protocol.exchange("show motor 7")
    fields_equal(fields, {"joint": "7", "esc": "07", "master": "17"}, "Wrong motor identity")
    require(re.fullmatch(r"\d+", fields.get("age_ms", "")) is not None, "Missing feedback age")
    return fields, int(fields["age_ms"]), sent, received


def usb_serial_from_uid(uid_words):
    """Mirror usbd_desc.c uint32 addition/high-16 encoding, rejecting its zero-sum branch."""
    require(isinstance(uid_words, (tuple, list)) and len(uid_words) == 3 and
            all(type(word) is int and 0 <= word <= 0xFFFFFFFF for word in uid_words),
            "MCU UID must contain exactly three uint32 words")
    serial_sum = (uid_words[0] + uid_words[2]) & 0xFFFFFFFF
    # Get_SerialNum leaves the descriptor zero-initialized for a zero sum;
    # emitting ASCII zero digits would invent a serial the firmware never made.
    require(serial_sum != 0, "MCU UID sum leaves the USB serial descriptor uninitialized")
    return f"{serial_sum:08X}{uid_words[1] >> 16:04X}"


def verify_usb_swd_pair(target, report):
    """Read the attached MCU UID without halting and bind its derived serial to USB."""
    report["mcu_uid_addresses"] = list(MCU_UID_ADDRESSES)
    report["mcu_uid_words"] = []
    report["expected_usb_serial"] = USB_SERIAL
    for address in MCU_UID_ADDRESSES:
        word = target.read32(address)
        report["mcu_uid_words"].append(word if type(word) is int else
                                       {"invalid_type": type(word).__name__, "value": repr(word)})
        require(type(word) is int and 0 <= word <= 0xFFFFFFFF, "Invalid MCU UID uint32 read")
    derived_serial = usb_serial_from_uid(report["mcu_uid_words"])
    report["mcu_derived_usb_serial"] = derived_serial
    require(derived_serial == USB_SERIAL, "USB/SWD MCU UID serial mismatch")
    report["usb_swd_pair_verified"] = True


def disable_once(protocol, clock, sleep):
    """Send exactly one DISABLE, then at most 20 read-only polls for a new fault-free reply."""
    before, before_age, _, _ = motor_feedback(protocol)
    fields_equal(before, {"fault": "0"}, "Pre-DISABLE driver fault")
    require(before_age > 500, "Old feedback is too recent to distinguish a new DISABLE sample")
    started = clock()
    accepted, _, _ = protocol.exchange("bench disable 7")
    fields_equal(accepted, {"accepted": "1"}, "DISABLE was not accepted")
    for index in range(20):
        fields, age, sent, received = motor_feedback(protocol)
        fields_equal(fields, {"fault": "0"}, "Driver fault after DISABLE")
        elapsed = (received - started) * 1000
        if fields.get("state") == "disabled" and fields.get("fault") == "0" and age <= 100 and age <= elapsed + 4:
            return {"age_ms": age, "query_sent": sent, "received": received,
                    "disable_started": started, "disable_elapsed_ms": elapsed,
                    "before_age_ms": before_age, "poll_count": index + 1, "fields": fields}
        if index < 19:
            sleep(0.005)
    raise RuntimeError("No fresh fault-free disabled ID7 feedback after the single DISABLE")


def read_flash(target, start, end):
    """Read exactly one bounded flash span in small, explicit noncached chunks."""
    require(FLASH_START <= start < end <= FLASH_END, "Flash read outside reviewed range")
    result = bytearray()
    for address in range(start, end, 4096):
        size = min(4096, end - address)
        payload = bytes(target.read_memory_block8(address, size))
        require(len(payload) == size, "Short flash read")
        result.extend(payload)
    return bytes(result)


def compare_segments(target, segments, expected_state, hardware):
    """Compare all occupied bytes while requiring the reviewed core state throughout."""
    verified = 0
    for address, payload in segments:
        require(target.get_state() == expected_state and read_flash(target, address, address + len(payload)) == payload
                and target.get_state() == expected_state, "Current image/state mismatch")
        verified += len(payload)
    return verified


def reset_catch_verified(target, reset_vector, hardware):
    """Reset and halt once, requiring the old vector PC in Thumb Thread context before programming."""
    require(reset_vector & 1 and FLASH_START <= reset_vector & ~1 < FLASH_END, "Invalid old reset vector")
    target.reset_and_halt(reset_type=hardware.Target.ResetType.SYSRESETREQ)
    state = target.get_state()
    program_counter = target.read_core_register("pc")
    program_status = target.read_core_register("xpsr")
    require(state == hardware.Target.State.HALTED and program_counter == reset_vector & ~1 and
            program_status & 0x1FF == 0 and program_status & (1 << 24), "Reset-catch context verification failed")
    return {"pc": program_counter, "xpsr": program_status, "state": str(state)}


def save_report(output, report, lines):
    """Retain the transaction and timestamped serial evidence, always leaving qualification false."""
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (output / "serial_preflight.json").write_text(json.dumps(lines, indent=2), encoding="utf-8")


def deploy(prepared, hardware, output, supported_unloaded, motor_power_on, port=None,
           clock=time.perf_counter, sleep=time.sleep, expected_mode="2"):
    """Execute one gated serial/attach/disable/halt/backup/program/readback/boot transaction."""
    require(type(expected_mode) is str and expected_mode in ("1", "2"), "Expected mode must be the string 1 or 2")
    require(supported_unloaded is True and motor_power_on is True, "Both physical deployment confirmations are required")
    output = copy_payloads(prepared, output)
    report = {"target": TARGET_NAME, "expected_mode": expected_mode, "hardware_qualified": False, "qualified": False,
        "motor_power": "on=user_confirmation", "supported_unloaded": "user_confirmation",
        "motion_commands_sent": 0, "manifest_sha256": digest(prepared.manifest_payload),
        "new_hex_sha256": digest(prepared.image_payload), "new_elf_sha256": digest(prepared.elf_payload),
        "old_candidate_sha256": digest(prepared.old_payload), "old_image_equal": False,
        "usb_swd_pair_verified": False,
        "flash_readback_equal": False, "normal_reset_requested": False, "reset_completed": False,
        "reset_catch_requested": False, "first_halt_started": False, "error": None}
    lines, session, serial_port, target = [], None, None, None
    first_halt_started = False
    failure_halted = False

    def halt_on_failure():
        """Best-effort halt only after the authorized first halt has begun; never resume or reset."""
        nonlocal failure_halted
        if first_halt_started and target is not None and not failure_halted:
            failure_halted = True
            try:
                target.halt()
                report["failure_core_state"] = str(target.get_state())
                require(target.get_state() == hardware.Target.State.HALTED, "Failure halt did not report HALTED")
            except BaseException as error:
                report["failure_halt_error"] = f"{type(error).__name__}: {error}"

    try:
        ports = [item for item in hardware.ports() if item.vid == 0x0483 and item.pid == 0x5740
                 and item.serial_number == USB_SERIAL]
        require(len(ports) == 1 and (port is None or port == ports[0].device), "Exact unique USB CDC identity/port not found")
        serial_port = hardware.Serial(port=None, baudrate=115200, timeout=0.004, write_timeout=0.5)
        serial_port.dtr = False
        serial_port.rts = False
        serial_port.port = ports[0].device
        serial_port.open()
        protocol = Protocol(serial_port, lines, clock)
        initial_boot = idle_snapshot(protocol, expected_mode=expected_mode)
        # Inspect identity and old age passively before even attaching to SWD.
        _, initial_age, _, _ = motor_feedback(protocol)
        require(initial_age > 500, "Initial feedback is too recent for a fresh DISABLE distinction")
        probes = [probe for probe in hardware.probes() if probe.unique_id == PROBE_UID]
        require(len(probes) == 1, "Expected unique probe UID not found")
        session = hardware.Session(probes[0], options=dict(SESSION_OPTIONS))
        session.open()
        target = session.target
        require(session.probe.unique_id == PROBE_UID and target.get_state() == hardware.Target.State.RUNNING
                and target.read32(0x5C001000) & 0xFFF == 0x483, "Unexpected probe/MCU identity or core not RUNNING")
        verify_usb_swd_pair(target, report)
        report["old_verified_bytes"] = compare_segments(target, prepared.old_segments, hardware.Target.State.RUNNING, hardware)
        report["old_image_equal"] = True
        require(idle_snapshot(protocol, expected_mode=expected_mode) == initial_boot, "Boot changed while verifying current image")
        feedback = disable_once(protocol, clock, sleep)
        report["disabled_feedback"] = feedback
        wait_post_disable_idle(protocol, initial_boot, feedback, clock, sleep, expected_mode=expected_mode)
        require(target.get_state() == hardware.Target.State.RUNNING, "Core changed state before halt")
        # The feedback age is measured by firmware during the query. Conservatively
        # start host elapsed at query transmission, including response transit and
        # every recheck; include firmware integer-age floor and host resolution.
        age_upper = feedback_age_upper_ms(feedback, clock)
        feedback["age_at_first_halt_upper_ms"] = age_upper
        feedback["host_elapsed_to_first_halt_ms"] = (clock() - feedback["received"]) * 1000
        feedback["boot"] = initial_boot
        require(age_upper <= 100, "Fresh disabled feedback expired before first halt")
        first_halt_started = True
        report["first_halt_started"] = True
        target.halt()
        require(target.get_state() == hardware.Target.State.HALTED, "First halt did not report HALTED")
        feedback["age_at_halt_completion_upper_ms"] = feedback_age_upper_ms(feedback, clock)
        require(feedback["age_at_halt_completion_upper_ms"] <= 100,
                "Fresh disabled feedback expired while the first halt completed")
        previous = read_flash(target, FLASH_START, FLASH_END)
        require(len(previous) == FLASH_END - FLASH_START, "Incomplete 1 MiB backup")
        for address, payload in prepared.old_segments:
            require(previous[address - FLASH_START:address - FLASH_START + len(payload)] == payload,
                    "Halted backup differs from running old image")
        backup = output / "previous_flash_1MiB.bin"
        with backup.open("xb") as stream:
            require(stream.write(previous) == len(previous), "Short backup write")
        require(bounded_path(backup, output).read_bytes() == previous, "Backup write/read verification failed")
        report["backup"] = str(backup)
        report["backup_sha256"] = digest(previous)
        report["backup_bytes"] = len(previous)
        report["reset_catch_requested"] = True
        report["reset_catch"] = reset_catch_verified(target, struct.unpack_from("<I", previous, 4)[0], hardware)
        # FileProgrammer receives the frozen in-memory text. No mutable path is
        # reopened after the preflight or by the programmer.
        hardware.FileProgrammer(session, chip_erase="sector", smart_flash=False,
            trust_crc=False, keep_unwritten=False).program(io.StringIO(prepared.image_payload.decode("ascii")), file_format="hex")
        target.halt()
        require(target.get_state() == hardware.Target.State.HALTED, "Post-program core not HALTED")
        report["new_verified_bytes"] = compare_segments(target, prepared.new_segments, hardware.Target.State.HALTED, hardware)
        report["flash_readback_equal"] = True
        save_report(output, report, lines)
        report["normal_reset_requested"] = True
        target.reset(reset_type=hardware.Target.ResetType.SYSRESETREQ)
        require(target.get_state() == hardware.Target.State.RUNNING, "Normal reset did not report RUNNING")
        report["reset_completed"] = True
        report["state_after"] = str(target.get_state())
    except BaseException as error:
        report["error"] = f"{type(error).__name__}: {error}"
        halt_on_failure()
        raise
    finally:
        # Cleanup failures are subject to the same BaseException and post-halt
        # policy; retain the original exception if the transaction already failed.
        cleanup_error = None
        for resource in (serial_port, session):
            if resource is not None:
                try:
                    resource.close()
                except BaseException as error:
                    cleanup_error = cleanup_error or error
                    report["cleanup_error"] = f"{type(error).__name__}: {error}"
                    halt_on_failure()
        try:
            save_report(output, report, lines)
        except BaseException:
            halt_on_failure()
            raise
        if cleanup_error is not None and report["error"] is None:
            report["error"] = f"{type(cleanup_error).__name__}: {cleanup_error}"
            save_report(output, report, lines)
            raise cleanup_error
    return report


def main(argv=None):
    """Default to pure verification and enforce explicit confirmations before hardware imports."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--deploy", action="store_true")
    parser.add_argument("--supported-unloaded-confirmed", action="store_true")
    parser.add_argument("--motor-power-on-confirmed", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--expected-mode", choices=("1", "2"), default="2",
                        help="Exact current mode required by every preflight snapshot (1=MIT, 2=POS).")
    arguments = parser.parse_args(argv)
    if arguments.deploy and not (arguments.supported_unloaded_confirmed and arguments.motor_power_on_confirmed):
        parser.error("--deploy requires --supported-unloaded-confirmed and --motor-power-on-confirmed")
    prepared = prepare(arguments.manifest, arguments.manifest_sha256)
    print("SINGLE_STATIC_ARTIFACTS_VERIFIED_NO_HARDWARE", digest(prepared.image_payload), flush=True)
    if arguments.deploy:
        output = ROOT / "output/static_trials_20261019" / ("deploy_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        report = deploy(prepared, load_hardware(), output, arguments.supported_unloaded_confirmed,
                        arguments.motor_power_on_confirmed, arguments.port, expected_mode=arguments.expected_mode)
        print("SINGLE_STATIC_DEPLOY_READBACK_EQUAL_RUNNING", json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
