"""Offline regressions for the independent single-static deployment transaction.

Fixtures stay in ignored Tests/host/build. All USB, serial and SWD boundaries are
fakes; these tests never enumerate or open a real device and never remove files.
"""

import copy
import hashlib
import importlib.util
import inspect
import io
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
ADAPTER = ROOT / "tools/deploy_single_static_test.py"


def load_adapter():
    """Import only the maintained adapter, without calling its CLI entry point."""
    specification = importlib.util.spec_from_file_location("single_static_deploy", ADAPTER)
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def sha(payload):
    """Hash an immutable fixture payload."""
    return hashlib.sha256(payload).hexdigest()


def image_pair(address, payload):
    """Build a genuine Intel HEX and ELF32 pair for exact coverage tests."""
    from intelhex import IntelHex
    image = IntelHex()
    image.frombytes(payload, offset=address)
    stream = io.StringIO()
    image.write_hex_file(stream)
    identity = b"\x7fELF\x01\x01\x01" + bytes(9)
    header = identity + struct.pack("<HHIIIIIHHHHHH", 2, 40, 1, address | 1,
                                   52, 0, 0x5000000, 52, 32, 1, 0, 0, 0)
    segment = struct.pack("<IIIIIIII", 1, 84, address, address, len(payload), len(payload), 5, 4)
    return stream.getvalue().encode("ascii"), header + segment + payload


class FakeClock:
    """Advance a deterministic host clock only at fake I/O boundaries."""

    def __init__(self):
        """Start at a nonzero monotonic timestamp."""
        self.now = 10.0
        self.sleep_calls = []

    def __call__(self):
        """Return the current fake host timestamp."""
        return self.now

    def sleep(self, seconds):
        """Record elapsed polling time without blocking the host."""
        self.sleep_calls.append(seconds)
        self.now += seconds


class FakePort:
    """Emulate the current text protocol with numbered query responses."""

    def __init__(self, hardware, **options):
        """Keep serial configuration and postpone opening until DTR/RTS are low."""
        self.hardware = hardware
        self.options = options
        self.dtr = True
        self.rts = True
        self.port = options.get("port")
        self.reply = b""
        self.disabled = False
        self.disable_started = None
        self.snapshots = 0
        self.post_disable_snapshot_index = -1

    def open(self):
        """Require the reviewed serial lines before the fake port opens."""
        assert self.dtr is False and self.rts is False
        self.hardware.events.append("serial_open")

    def close(self):
        """Record deterministic port cleanup."""
        self.hardware.events.append("serial_close")

    def write(self, payload):
        """Generate a strict numbered response for an allowlisted command."""
        request, command = payload.decode("ascii").strip().split(" ", 1)
        hardware = self.hardware
        hardware.events.append(command)
        hardware.clock.now += 0.001
        detail = ""
        if command == "ping":
            self.snapshots += 1
            boot = 8 if hardware.scenario == "boot_changed" and self.disabled else 7
            detail = f"state=disabled enabled=00 boot={boot}"
        elif command == "show state":
            if self.disabled:
                self.post_disable_snapshot_index += 1
            detail = "state=disabled aligned=0 enabled=00 moving=0 active=0 fault=none"
            if hardware.scenario == "idle_changed" and self.disabled:
                detail = detail.replace("active=0", "active=9")
        elif command == "adrc status":
            detail = "state=0 fault=0 motor=7 qualified=0 disabled=1 active_id=0 owner=lcd"
        elif command == "adrc gate motor=7":
            if ((hardware.scenario == "mode_changed_before_disable" and self.snapshots >= 1) or
                    (hardware.scenario == "mode_changed_after_disable" and self.disabled)):
                hardware.current_mode = "2"
            detail = (f"motor=7 lcd_idle=1 can_idle=1 mode={hardware.current_mode} fields=1fff verified=40 "
                      "authority=0 action=0 ui_busy=0 ui_results=0 proto_results=0 discovery_active=0")
            if hardware.scenario == "queue_busy":
                detail = detail.replace("ui_results=0", "ui_results=1")
            if hardware.scenario == "deadline" and self.disabled:
                hardware.clock.now += 0.12
        elif command == "show diag can":
            detail = "rx=0 tx=0 drop=0 error=0 busoff=0"
            if hardware.can_responses:
                query_index = hardware.events.count(command) - 1
                detail = hardware.can_responses[min(query_index, len(hardware.can_responses) - 1)]
        elif command == "show motor 7":
            age = 1 if self.disabled else 900
            detail = f"joint=7 esc=07 master=17 state=disabled fault=0 age_ms={age}"
            if hardware.scenario == "stale":
                detail = detail.replace(f"age_ms={age}", "age_ms=600")
            elif hardware.scenario == "recent_before" and not self.disabled:
                detail = detail.replace("age_ms=900", "age_ms=5")
            elif hardware.scenario == "wrong_id":
                detail = detail.replace("joint=7", "joint=6")
            elif hardware.scenario == "wrong_master":
                detail = detail.replace("master=17", "master=18")
            elif hardware.scenario == "fault" and self.disabled:
                detail = detail.replace("fault=0", "fault=5")
            elif hardware.scenario == "missing_disabled" and self.disabled:
                detail = detail.replace("state=disabled", "state=enabled")
            elif hardware.scenario == "predates_disable" and self.disabled:
                old_age = int((hardware.clock.now - self.disable_started) * 1000) + 80
                detail = detail.replace("age_ms=1", f"age_ms={old_age}")
        elif command == "bench disable 7":
            self.disabled = True
            self.disable_started = hardware.clock.now
            detail = "accepted=1"
        else:
            raise AssertionError(f"Unexpected command: {command}")
        if self.disabled and self.post_disable_snapshot_index >= 0 and hardware.post_disable_snapshots:
            snapshot = hardware.post_disable_snapshots[min(self.post_disable_snapshot_index,
                                                          len(hardware.post_disable_snapshots) - 1)]
            for field, value in snapshot.get(command, {}).items():
                detail = re.sub(rf"(?:^| )\b{field}=[^ ]+", "" if value is None else f" {field}={value}", detail)
            if command == "ping":
                hardware.clock.now += snapshot.get("delay_seconds", 0)
        response_command = {"show motor 7": "show motor", "adrc gate motor=7": "adrc gate",
                            "bench disable 7": "bench disable"}.get(command, command)
        self.reply = f"ok {request} {response_command} {detail}\r\n".encode("ascii")
        if hardware.scenario == "serial_error":
            self.reply = f"error {request} {response_command} code=failed\r\n".encode("ascii")
        elif hardware.scenario == "serial_timeout":
            self.reply = b""
        return len(payload)

    def readline(self):
        """Return exactly one fake response line."""
        self.hardware.clock.now += 0.001
        reply, self.reply = self.reply, b""
        return reply


class FakeTarget:
    """Expose a memory backed H723 boundary with explicit transaction failures."""

    def __init__(self, hardware, old_segments):
        """Install occupied old bytes into a full fake 1 MiB flash."""
        self.hardware = hardware
        self.state = "running"
        self.memory = bytearray(b"\xff" * 0x100000)
        for address, payload in old_segments:
            self.memory[address - 0x08000000:address - 0x08000000 + len(payload)] = payload
        self.old_vector = struct.unpack_from("<I", self.memory, 4)[0]

    def get_state(self):
        """Return the core's fake state."""
        return self.state

    def read32(self, address):
        """Return reviewed identity registers and controlled fake STM32 UID words."""
        if address == 0x5C001000:
            self.hardware.events.append("mcu_id")
            return 0x482 if self.hardware.scenario == "wrong_mcu" else 0x10000483
        assert address in (0x1FF1E800, 0x1FF1E804, 0x1FF1E808)
        self.hardware.events.append(f"uid_read:{address:#x}")
        index = (address - 0x1FF1E800) // 4
        if index == self.hardware.uid_read_error:
            raise RuntimeError("fake UID read failure")
        return self.hardware.uid_words[index]

    def read_memory_block8(self, address, size):
        """Read fake occupied bytes or fail a selected post-halt stage."""
        hardware = self.hardware
        if self.state == "halted" and hardware.scenario == "backup_interrupt":
            raise KeyboardInterrupt("fake backup interrupt")
        if self.state == "halted" and hardware.scenario == "backup_error":
            raise RuntimeError("fake backup failure")
        result = bytes(self.memory[address - 0x08000000:address - 0x08000000 + size])
        if hardware.scenario == "old_mismatch" and self.state == "running":
            result = bytes([result[0] ^ 1]) + result[1:]
        if hardware.scenario == "readback_mismatch" and "program" in hardware.events:
            result = bytes([result[0] ^ 1]) + result[1:]
        return list(result)

    def halt(self):
        """Record a halt, including the safety halt after any BaseException."""
        self.hardware.events.append("halt")
        self.state = "halted"
        if self.hardware.scenario == "halt_latency":
            self.hardware.clock.now += 0.12
        if self.hardware.scenario == "halt_error" and self.hardware.events.count("halt") == 1:
            raise RuntimeError("first halt failed")

    def reset_and_halt(self, reset_type):
        """Record one reset catch without an ordinary boot."""
        assert reset_type == "sysresetreq"
        self.hardware.events.append("reset_catch")
        self.state = "halted"
        if self.hardware.scenario == "reset_catch_error":
            raise RuntimeError("reset catch failed")

    def read_core_register(self, register):
        """Supply old vector Thread/Thumb state or a deliberate wrong context."""
        if register == "pc":
            return self.old_vector & ~1
        assert register == "xpsr"
        return 0x1000003 if self.hardware.scenario == "exception_context" else 0x1000000

    def reset(self, reset_type):
        """Record the single ordinary SYSRESETREQ after complete readback."""
        assert reset_type == "sysresetreq"
        self.hardware.events.append("normal_reset")
        self.state = "running"
        if self.hardware.scenario == "reset_error":
            raise RuntimeError("normal reset failed")


class FakeHardware:
    """Inject all hardware packages without importing or accessing real ones."""

    def __init__(self, adapter, prepared, scenario="success"):
        """Bind fake serial, probe, target and programmer to one event stream."""
        self.adapter = adapter
        self.prepared = prepared
        self.scenario = scenario
        self.current_mode = "2"
        self.can_responses = []
        self.post_disable_snapshots = []
        self.uid_words = (0x12345678, 0x3134ABCD, 0x2636DCF7)
        self.uid_read_error = None
        self.events = []
        self.clock = FakeClock()
        self.Target = SimpleNamespace(State=SimpleNamespace(RUNNING="running", HALTED="halted"),
                                      ResetType=SimpleNamespace(SYSRESETREQ="sysresetreq"))
        self.target = FakeTarget(self, prepared.old_segments)
        self.probe = SimpleNamespace(unique_id=adapter.PROBE_UID)
        self.session = SimpleNamespace(target=self.target, probe=self.probe,
                                       open=self.open_session, close=self.close_session)

    def ports(self):
        """Return one exact USB CDC identity and a permitted COM name."""
        self.events.append("usb_enumerate")
        serial_number = "wrong" if self.scenario == "wrong_usb" else self.adapter.USB_SERIAL
        ports = [SimpleNamespace(vid=0x0483, pid=0x5740, serial_number=serial_number, device="COM_FAKE")]
        return ports * 2 if self.scenario == "duplicate_usb" else ports

    def Serial(self, **options):
        """Construct an unopened fake serial port."""
        self.port = FakePort(self, **options)
        return self.port

    def probes(self):
        """Enumerate probes only after passive serial validation."""
        self.events.append("probe_enumerate")
        return [self.probe, self.probe] if self.scenario == "duplicate_probe" else [self.probe]

    def Session(self, probe, options):
        """Capture attach options while returning an unopened fake session."""
        assert probe is self.probe
        self.options = options
        return self.session

    def open_session(self):
        """Record nonhalting attach."""
        self.events.append("attach")
        if self.scenario == "attach_error":
            raise RuntimeError("fake attach error")

    def close_session(self):
        """Disconnect without any resume or reset."""
        self.events.append("disconnect")
        if self.scenario == "disconnect_interrupt":
            raise KeyboardInterrupt("fake disconnect interrupt")

    def FileProgrammer(self, session, **options):
        """Validate destructive programming options at the fake boundary."""
        assert session is self.session
        assert options == dict(chip_erase="sector", smart_flash=False, trust_crc=False, keep_unwritten=False)
        return SimpleNamespace(program=self.program)

    def program(self, stream, file_format):
        """Consume immutable HEX text rather than reopening a mutable file path."""
        assert isinstance(stream, io.StringIO) and file_format == "hex"
        assert stream.read().encode("ascii") == self.prepared.image_payload
        self.events.append("program")
        if self.scenario == "program_interrupt":
            raise KeyboardInterrupt("fake programming interrupt")
        if self.scenario == "program_error":
            raise RuntimeError("fake programming failure")
        for address, payload in self.prepared.new_segments:
            self.target.memory[address - 0x08000000:address - 0x08000000 + len(payload)] = payload


class AdapterTests(unittest.TestCase):
    """Exercise artifact gates and the entire fake deployment state machine."""

    @classmethod
    def setUpClass(cls):
        """Load the new module only after its explicit availability assertion."""
        if not ADAPTER.is_file():
            return
        cls.adapter = load_adapter()

    def setUp(self):
        """Create retained fixture roots without any cleanup routine."""
        if not ADAPTER.is_file():
            if self._testMethodName == "test_adapter_exists":
                return
            self.skipTest("Availability test provides the missing-feature RED")
        parent = ROOT / "Tests/host/build/single_static_deploy"
        parent.mkdir(parents=True, exist_ok=True)
        self.workspace = Path(tempfile.mkdtemp(prefix="fixture_", dir=parent))
        self.run = self.workspace / "output/run_1"
        self.run.mkdir(parents=True)
        self.manifest_path = self.run / "manifest.json"
        self.old_hex, _ = image_pair(0x08000000, struct.pack("<II", 0x24020000, 0x08000009) + b"OLD!")
        self.new_hex, self.new_elf = image_pair(0x08000000, struct.pack("<II", 0x24020000, 0x08000009) + b"NEW!")
        self.old_path = self.workspace / self.adapter.OLD_RELATIVE
        self.old_path.parent.mkdir(parents=True)
        self.old_path.write_bytes(self.old_hex)
        self.sources = set(self.adapter.REQUIRED_SOURCES)
        for relative in self.sources:
            path = self.workspace / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((relative + "\n").encode("ascii"))
        script = self.workspace / "output/verify.ps1"
        script.write_bytes(b"# never execute this fixture\n")
        self.manifest = dict(schema="single-static-acceptance-v1", target="LCD-SingleStatic-Test",
            run_id="20261019T010203000Z", started_at_utc="2026-10-19T01:02:03Z",
            completed_at_utc="2026-10-19T01:02:04Z", passed=True, software_only=True,
            hardware_tested=False, sources_unchanged=True, control_sources_unchanged=True,
            failure=None, lcd_checks=32, control_baseline=self.adapter.CONTROL_BASELINE,
            rtos_stack_baseline=self.adapter.RTOS_BASELINE,
            rtos_stack_override_scope="single-static-compile-gate-only",
            verification_script_path=str(script), verification_script_sha256=sha(script.read_bytes()),
            records=[], builds=[], source_hashes=[], evidence_hashes=[])
        for name in self.adapter.REQUIRED_RECORDS:
            stdout = self.run / (name + ".log")
            stderr = self.run / (name + ".stderr.log")
            stdout.write_bytes(b"offline passed\n")
            stderr.write_bytes(b"")
            self.manifest["records"].append(dict(name=name, exit_code=0, log=str(stdout), stderr=str(stderr)))
        for target in self.adapter.REQUIRED_TARGETS:
            record = dict(target=target, errors=0, warnings=0)
            for field, extension, payload in (("hex_path", "hex", self.new_hex),
                    ("elf_path", "axf", self.new_elf), ("map_path", "map", b"map receipt\n"),
                    ("log_path", "keil.log", b"0 Error(s), 0 Warning(s)\n")):
                path = self.run / (target + "." + extension)
                path.write_bytes(payload)
                record[field] = str(path)
            self.manifest["builds"].append(record)
        lcd = self.run / "lcd_evidence"
        lcd.mkdir()
        checks = []
        for name in self.adapter.REQUIRED_LCD_CHECKS:
            for suffix in (".log", ".stderr.log"):
                (lcd / (name + suffix)).write_bytes(b"offline LCD receipt\n")
            # Original LCD summary contains source paths from its own working
            # directory; deployed validation must use retained files by basename.
            checks.append(dict(name=name, exit_code=0, stdout=str(self.workspace / "Tests/host/build/verification" / (name + ".log")),
                               stderr=str(self.workspace / "Tests/host/build/verification" / (name + ".stderr.log"))))
        self.summary = dict(software_only=True, hardware_tested=False, keil_requested=True,
            first_party_sources_unchanged_during_verification=True, checks=checks)
        self.summary_path = lcd / "summary.json"
        self.publish()

    def publish(self):
        """Rebind modified fake receipts so semantic tests do not fail on unrelated hashes."""
        self.summary_path.write_text(json.dumps(self.summary), encoding="utf-8")
        self.manifest["source_hashes"] = [dict(path=path, sha256=sha((self.workspace / path).read_bytes()))
                                          for path in sorted(self.sources)]
        self.manifest["evidence_hashes"] = [dict(path=str(path), sha256=sha(path.read_bytes()))
            for path in sorted(self.run.rglob("*")) if path.is_file() and path != self.manifest_path]
        self.manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def prepare(self):
        """Run real pure verification with only source enumeration and old digest patched."""
        with patch.object(self.adapter, "maintained_sources", return_value=self.sources), \
                patch.object(self.adapter, "OLD_SHA256", sha(self.old_hex)):
            return self.adapter.prepare(self.manifest_path, sha(self.manifest_path.read_bytes()), self.workspace)

    def execute(self, prepared, hardware, output=None, expected_mode="2"):
        """Invoke the real transaction against fake hardware and a deterministic clock."""
        self.assertIn("expected_mode", inspect.signature(self.adapter.deploy).parameters,
                      "Deployment lacks the explicit expected-mode contract")
        output = output or self.workspace / "deployment"
        return self.adapter.deploy(prepared, hardware, output, True, True,
                                   clock=hardware.clock, sleep=hardware.clock.sleep, expected_mode=expected_mode)

    def test_adapter_exists(self):
        """Provide the intentional missing-module RED without an import error."""
        self.assertTrue(ADAPTER.is_file(), "Missing independent single-static deployment adapter")

    def test_current_image_pin_is_the_deployed_single_static_test(self):
        """A recovery deployment must bind the last deployed image, never the superseded ADRC image."""
        self.assertEqual(self.adapter.OLD_RELATIVE,
            "output/static_startup_sync_20261020/hardware_prepare/expected_c333_LCD-SingleStatic-Test.hex")
        self.assertEqual(self.adapter.OLD_SHA256,
            "c333079f935117a0ff3d0260280f4a07dc9f87e5a626a30c8c67ee11392602dc")

    def use_current_target_profile(self):
        """Retain only the explicitly requested three current firmware receipts."""
        targets = {"ADRC-Bench", "LCD-MIT-ADRC", "LCD-SingleStatic-Test"}
        self.manifest.update(schema="single-static-startup-acceptance-v2",
            acceptance_profile="current-static-startup", lcd_checks=27,
            reviewed_startup_revision="1" * 40,
            rtos_stack_override_scope="static-trace-and-integrated-lcd-task-stacks")
        self.manifest["builds"] = [entry for entry in self.manifest["builds"]
                                    if entry["target"] in targets]
        self.summary["keil_requested"] = False
        self.summary["checks"] = [entry for entry in self.summary["checks"]
            if not entry["name"].startswith("keil_") and entry["name"] != "check_debug_ui_map"]
        for name in ("startup_trace_export_tests", "startup_sync_tests", "map_current_targets", "trace_map_tests"):
            stdout, stderr = self.run / (name + ".log"), self.run / (name + ".stderr.log")
            stdout.write_bytes(b"offline passed\n")
            stderr.write_bytes(b"")
            self.manifest["records"].append(dict(name=name, exit_code=0,
                log=str(stdout), stderr=str(stderr)))
        self.publish()

    def test_current_target_profile_retains_host_checks_and_only_three_native_builds(self):
        """Accept the new explicit matrix without needing legacy firmware builds."""
        self.use_current_target_profile()
        self.assertEqual(len(self.summary["checks"]), 27)
        prepared = self.prepare()
        self.assertEqual(prepared.image_payload, self.new_hex)

    def test_task_stack_scope_is_bound_to_the_acceptance_schema(self):
        """Keep legacy scope exact and require the current trace/integrated stack scope."""
        legacy = copy.deepcopy(self.manifest)
        self.assertEqual(self.prepare().image_payload, self.new_hex)
        self.manifest["rtos_stack_override_scope"] = "static-trace-and-integrated-lcd-task-stacks"
        self.publish()
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.manifest = legacy
        self.use_current_target_profile()
        self.assertEqual(self.prepare().image_payload, self.new_hex)
        for scope in ("single-static-compile-gate-only", "all-task-stacks", "", None):
            self.manifest["rtos_stack_override_scope"] = scope
            self.publish()
            with self.subTest(scope=scope), self.assertRaises(RuntimeError):
                self.prepare()

    def test_startup_stack_guard_rejects_default_or_unrelated_changes(self):
        """Exercise the producer's pure header exception against real non-noop mutations."""
        script = self.workspace / "stack_guard_check.ps1"
        producer = ROOT / "tools/verify_static_startup.ps1"
        header = ROOT / "Core/Inc/aethor_task_stacks.h"
        script.write_text(r'''# File: stack_guard_check.ps1. Pure producer guard regression; no build or hardware.
$ErrorActionPreference = 'Stop'
$parseTokens = $null; $parseErrors = $null
$syntax = [System.Management.Automation.Language.Parser]::ParseFile($args[0], [ref]$parseTokens, [ref]$parseErrors)
if ($parseErrors.Count) { throw 'Producer syntax is invalid.' }
$helper = @($syntax.FindAll({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Test-StaticStartupStackConfiguration' }, $true))
if ($helper.Count -ne 1) { throw 'Missing unique pure stack-configuration guard.' }
. ([scriptblock]::Create($helper[0].Extent.Text))
$actual = Get-Content -LiteralPath $args[1] -Raw
$baseline = @'
#ifndef AETHOR_TASK_STACKS_H
#define AETHOR_TASK_STACKS_H
#include "debug_ui_config.h"
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (2048U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (512U)
#else
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (1280U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (384U)
#endif
#endif
'@
$cases = @(
    @{Name='reviewed exact header'; Text=$actual; Baseline=$baseline; Expected=$true},
    @{Name='comment-only provenance'; Text=('/* harmless provenance */' + $actual); Baseline=$baseline; Expected=$true},
    @{Name='unreviewed trace size'; Text=$actual.Replace('(640U)', '(768U)'); Baseline=$baseline; Expected=$false},
    @{Name='changed default CAN'; Text=$actual.Replace('#define AETHOR_CAN_RX_TASK_STACK_WORDS (384U)', '#define AETHOR_CAN_RX_TASK_STACK_WORDS (640U)'); Baseline=$baseline; Expected=$false},
    @{Name='changed default parser'; Text=$actual.Replace('(1280U)', '(2048U)'); Baseline=$baseline; Expected=$false},
    @{Name='changed default diagnostics'; Text=$actual.Replace('#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (384U)', '#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (512U)'); Baseline=$baseline; Expected=$false},
    @{Name='unguarded CAN'; Text=$actual.Replace('AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)', '1'); Baseline=$baseline; Expected=$false},
    @{Name='missing POS receive guard'; Text=$actual.Replace(' || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST', ''); Baseline=$baseline; Expected=$false},
    @{Name='wrong POS receive guard'; Text=$actual.Replace('AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST', 'AETHOR_DEBUG_UI_ALLOW_MIT'); Baseline=$baseline; Expected=$false},
    @{Name='unguarded parser'; Text=$actual.Replace('AETHOR_ADRC_LCD_INTEGRATED || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD', '1'); Baseline=$baseline; Expected=$false},
    @{Name='unrelated macro'; Text=($actual + "`n#define AETHOR_UNRELATED_CHANGE 1`n"); Baseline=$baseline; Expected=$false},
    @{Name='wrong baseline'; Text=$actual; Baseline=$baseline.Replace('(1280U)', '(2560U)'); Expected=$false}
)
foreach ($case in $cases) {
    if (-not $case.Expected -and $case.Text -ceq $actual -and $case.Baseline -ceq $baseline) { throw ('No-op stack mutation: ' + $case.Name) }
    if ((Test-StaticStartupStackConfiguration -Text $case.Text -BaselineText $case.Baseline) -ne $case.Expected) { throw ('Stack guard case failed: ' + $case.Name) }
}
Write-Output ('STATIC_STARTUP_STACK_GUARD_PASS cases=' + $cases.Count)
''', encoding="utf-8")
        environment = os.environ.copy()
        windows_root = Path(os.environ["SystemRoot"])
        environment["PSModulePath"] = str(windows_root / "System32/WindowsPowerShell/v1.0/Modules")
        completed = subprocess.run([str(windows_root / "System32/WindowsPowerShell/v1.0/powershell.exe"),
            "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script),
            str(producer), str(header)], capture_output=True, text=True, errors="replace",
            env=environment, timeout=30, check=False)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("STATIC_STARTUP_STACK_GUARD_PASS cases=12", completed.stdout)

    def test_current_target_profile_rejects_weakened_or_mixed_evidence(self):
        """Fail closed on missing current builds, host checks, proof tests or review identity."""
        self.use_current_target_profile()
        original, summary = copy.deepcopy(self.manifest), copy.deepcopy(self.summary)
        mutations = [lambda: self.manifest.update(acceptance_profile="anything"),
            lambda: self.manifest.update(reviewed_startup_revision="bad"),
            lambda: self.manifest["builds"].pop(),
            lambda: self.manifest["records"].pop(),
            lambda: self.summary["checks"].pop(),
            lambda: self.summary.update(keil_requested=True),
            lambda: self.manifest.update(lcd_checks=26)]
        for mutation in mutations:
            self.manifest, self.summary = copy.deepcopy(original), copy.deepcopy(summary)
            mutation()
            self.publish()
            with self.subTest(mutation=mutation), self.assertRaises(RuntimeError):
                self.prepare()

    def test_recovery_sources_bind_receipt_and_feedback_boundaries(self):
        """Require exact receipt, per-ID feedback and enum sources in each deployment manifest."""
        expected = {"App/DebugUi/debug_ui_contract.h", "App/DebugUi/debug_ui_single_static_tx.h",
            "App/aethor_app.h", "App/Platform/stm32_platform.c", "App/Platform/stm32_platform.h",
            "App/Motor/motor_bank.c", "App/Motor/motor_bank.h", "App/Protocol/protocol_engine.c",
            "App/Protocol/protocol_engine.h", "Tests/host/static_lcd_recovery_cases.inc"}
        self.assertTrue(expected <= self.adapter.REQUIRED_SOURCES,
            "Recovery deployment omits an essential receipt/feedback/operation source")

    def test_recovery_platform_ordering_evidence_is_mandatory(self):
        """Require real platform seam tests and reject a manifest that omits their successful record."""
        expected = {"Tests/host/single_static_platform_test.c",
            "Tests/host/single_static_platform_fakes/fdcan.h",
            "Tests/host/single_static_platform_fakes/usbd_cdc_if.h",
            "Tests/host/run_single_static_platform_tests.ps1"}
        self.assertTrue(expected <= self.adapter.REQUIRED_SOURCES,
            "Recovery deployment omits a maintained platform ordering test input")
        self.assertIn("single_static_platform", self.adapter.REQUIRED_RECORDS,
            "Recovery deployment does not require successful platform ordering verification")
        self.manifest["records"] = [entry for entry in self.manifest["records"]
                                    if entry["name"] != "single_static_platform"]
        self.publish()
        with self.assertRaises(RuntimeError):
            self.prepare()

    def test_default_cli_is_pure_and_deploy_requires_both_flags(self):
        """Default verification neither loads hardware packages nor enumerates devices."""
        with patch.object(self.adapter, "prepare", return_value=SimpleNamespace(image_payload=b"pure")), \
                patch.object(self.adapter, "load_hardware", side_effect=AssertionError("hardware loaded")):
            self.adapter.main(["--manifest", str(self.manifest_path), "--manifest-sha256", "a" * 64])
            for flags in ([], ["--supported-unloaded-confirmed"], ["--motor-power-on-confirmed"]):
                with self.subTest(flags=flags), patch("sys.stderr", new=io.StringIO()), self.assertRaises(SystemExit):
                    self.adapter.main(["--manifest", str(self.manifest_path), "--manifest-sha256", "a" * 64, "--deploy", *flags])

    def test_expected_mode_cli_defaults_and_explicit_mit_are_forwarded(self):
        """Bind the caller's exact mode before hardware loading and preserve the legacy default."""
        base = ["--manifest", str(self.manifest_path), "--manifest-sha256", "a" * 64, "--deploy",
                "--supported-unloaded-confirmed", "--motor-power-on-confirmed"]
        for flags, expected in (([], "2"), (["--expected-mode", "1"], "1"), (["--expected-mode", "2"], "2")):
            with self.subTest(flags=flags), \
                    patch.object(self.adapter, "prepare", return_value=SimpleNamespace(image_payload=b"pure")), \
                    patch.object(self.adapter, "load_hardware", return_value=object()), \
                    patch.object(self.adapter, "deploy", return_value={}) as transaction:
                self.adapter.main(base + flags)
                self.assertEqual(transaction.call_args.kwargs.get("expected_mode"), expected)
        for value in ("0", "3", "01", "1.0", "MIT"):
            with self.subTest(value=value), patch("sys.stderr", new=io.StringIO()), \
                    patch.object(self.adapter, "prepare", side_effect=AssertionError("artifacts loaded")), \
                    patch.object(self.adapter, "load_hardware", side_effect=AssertionError("hardware loaded")), \
                    self.assertRaises(SystemExit):
                self.adapter.main(base + ["--expected-mode", value])

    def test_explicit_mit_mode_preserves_complete_transaction_and_quiet_polling(self):
        """Accept an explicitly selected MIT mode through every snapshot and the single DISABLE."""
        prepared = self.prepare()
        for pending in (False, True):
            with self.subTest(pending=pending):
                hardware = FakeHardware(self.adapter, prepared)
                hardware.current_mode = "1"
                if pending:
                    hardware.post_disable_snapshots = [{"adrc gate motor=7": {"can_idle": "0"}}, {}]
                report = self.execute(prepared, hardware, self.workspace / f"mit_{pending}", expected_mode="1")
                self.assertEqual(report["expected_mode"], "1")
                self.assertTrue(report["flash_readback_equal"])
                self.assertEqual(hardware.events.count("bench disable 7"), 1)
                self.assertEqual(hardware.events.count("program"), 1)
                self.assertEqual(hardware.events.count("normal_reset"), 1)
                self.assertEqual(hardware.port.post_disable_snapshot_index + 1, 2 if pending else 1)
                self.assertLessEqual(report["disabled_feedback"]["age_at_first_halt_upper_ms"], 100)

    def test_explicit_legacy_mode_rejects_mit_before_disable_or_swd(self):
        """Require mode 2 exactly when selected, without detecting or adopting mode 1."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        hardware.current_mode = "1"
        output = self.workspace / "wrong_expected_mode"
        with self.assertRaisesRegex(RuntimeError, "Control/discovery"):
            self.execute(prepared, hardware, output, expected_mode="2")
        for forbidden in ("probe_enumerate", "attach", "bench disable 7", "halt", "reset_catch", "program", "normal_reset"):
            self.assertNotIn(forbidden, hardware.events)
        self.assertEqual(hardware.events.count("adrc gate motor=7"), 1)
        self.assertEqual(json.loads((output / "report.json").read_text())["expected_mode"], "2")

    def test_selected_mode_changes_fail_before_disable_or_first_halt(self):
        """Reject mode changes at the old-image snapshot and every post-DISABLE quiet poll."""
        prepared = self.prepare()
        for scenario in ("mode_changed_before_disable", "mode_changed_after_disable", "mode_changed_while_quiet_pending"):
            with self.subTest(scenario=scenario):
                hardware = FakeHardware(self.adapter, prepared, scenario)
                hardware.current_mode = "1"
                if scenario == "mode_changed_while_quiet_pending":
                    hardware.post_disable_snapshots = [{"adrc gate motor=7": {"can_idle": "0"}},
                                                       {"adrc gate motor=7": {"mode": "2"}}]
                with self.assertRaisesRegex(RuntimeError, "Control/discovery"):
                    self.execute(prepared, hardware, self.workspace / scenario, expected_mode="1")
                self.assertEqual(hardware.events.count("bench disable 7"), int(scenario != "mode_changed_before_disable"))
                self.assertEqual(hardware.clock.sleep_calls, [0.004] if scenario == "mode_changed_while_quiet_pending" else [])
                for forbidden in ("halt", "reset_catch", "program", "normal_reset"):
                    self.assertNotIn(forbidden, hardware.events)

    def test_invalid_expected_mode_is_rejected_before_enumeration(self):
        """Require a plain string 1 or 2 before copying payloads or enumerating any device."""
        prepared = self.prepare()
        for index, value in enumerate((None, 1, 2, True, False, "", "0", "3", "01", " 1", "1 ", b"1")):
            with self.subTest(value=value):
                hardware = FakeHardware(self.adapter, prepared)
                output = self.workspace / f"invalid_expected_mode_{index}"
                with self.assertRaisesRegex(RuntimeError, "[Mm]ode"):
                    self.execute(prepared, hardware, output, expected_mode=value)
                self.assertEqual(hardware.events, [])
                self.assertFalse(output.exists())

    def test_valid_artifacts_are_frozen_and_exact_elf_hex_coverage_is_required(self):
        """Accept real parsers and reject missing, extra, overlapping or changed flash content."""
        prepared = self.prepare()
        self.assertEqual(prepared.image_payload, self.new_hex)
        self.assertEqual(sum(len(payload) for _, payload in prepared.new_segments), 12)
        for payload in (self.new_elf[:-1], self.new_elf[:-1] + b"x"):
            with self.subTest(payload=payload), self.assertRaises(RuntimeError):
                self.adapter.verify_image_pair(self.new_hex, payload)
        extra_hex, _ = image_pair(0x08000000, b"x" * 13)
        with self.assertRaises(RuntimeError):
            self.adapter.verify_image_pair(extra_hex, self.new_elf)
        overlap = bytearray(self.new_elf)
        struct.pack_into("<H", overlap, 44, 2)
        overlap[84:84] = overlap[52:84]
        struct.pack_into("<I", overlap, 56, 116)
        struct.pack_into("<I", overlap, 88, 116)
        with self.assertRaises(RuntimeError):
            self.adapter.verify_image_pair(self.new_hex, bytes(overlap))

    def test_manifest_semantic_failures_are_rejected(self):
        """Reject schema, target, gates, check identities, missing receipts and false clean logs."""
        original = copy.deepcopy(self.manifest)
        mutations = [lambda value: value.update(schema="old"), lambda value: value.update(target="LCD-MIT-ADRC"),
            lambda value: value.update(passed=False), lambda value: value.update(hardware_tested=True),
            lambda value: value.update(rtos_stack_baseline="0" * 40),
            lambda value: value["records"].pop(),
            lambda value: value["records"][0].update(exit_code=1),
            lambda value: value["records"][0].update(name=value["records"][1]["name"]),
            lambda value: value["builds"].pop(),
            lambda value: value["builds"][0].update(warnings=10)]
        for mutation in mutations:
            self.manifest = copy.deepcopy(original)
            mutation(self.manifest)
            self.publish()
            with self.subTest(mutation=mutation), self.assertRaises(RuntimeError):
                self.prepare()
        self.manifest = original
        Path(self.manifest["builds"][0]["log_path"]).write_text("10 Error(s), 10 Warning(s)\n")
        self.publish()
        with self.assertRaises(RuntimeError):
            self.prepare()

    def test_hashes_missing_sources_lcd_content_paths_and_duplicates_are_rejected(self):
        """Bind all inputs and retained sublogs rather than accepting claimed counts."""
        original = copy.deepcopy(self.manifest)
        self.manifest["evidence_hashes"][0]["sha256"] = "0" * 64
        self.manifest_path.write_text(json.dumps(self.manifest))
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.manifest = copy.deepcopy(original)
        self.manifest["source_hashes"].pop()
        self.manifest_path.write_text(json.dumps(self.manifest))
        with self.assertRaises(RuntimeError):
            self.prepare()
        for kind in ("duplicate", "traversal", "outside"):
            self.manifest = copy.deepcopy(original)
            if kind == "duplicate":
                self.manifest["evidence_hashes"].append(self.manifest["evidence_hashes"][0])
            elif kind == "traversal":
                self.manifest["source_hashes"][0]["path"] = "App/../" + self.manifest["source_hashes"][0]["path"]
            else:
                self.manifest["evidence_hashes"][0]["path"] = str(self.old_path)
            self.manifest_path.write_text(json.dumps(self.manifest))
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                self.prepare()
        self.manifest = original
        self.summary["checks"][0]["exit_code"] = 1
        self.publish()
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.summary["checks"][0]["exit_code"] = 0
        self.summary["checks"][0]["name"] = self.summary["checks"][1]["name"]
        self.publish()
        with self.assertRaises(RuntimeError):
            self.prepare()

    def test_manifest_digest_and_changed_payload_copy_fail_before_devices(self):
        """Detect anchored-manifest mismatch and post-verification TOCTOU before enumeration."""
        with patch.object(self.adapter, "maintained_sources", return_value=self.sources), self.assertRaises(RuntimeError):
            self.adapter.prepare(self.manifest_path, "0" * 64, self.workspace)
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        (self.run / "LCD-SingleStatic-Test.hex").write_bytes(b"changed after freeze")
        with self.assertRaises(RuntimeError):
            self.execute(prepared, hardware)
        self.assertEqual(hardware.events, [])

    def test_copy_mutation_and_inflight_source_change_are_rejected(self):
        """Catch TOCTOU in both initial verification and copied immutable payload receipts."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        original_recheck = self.adapter.recheck_frozen

        def mutate_copy(value):
            """Change a copied HEX after the original inputs have been rechecked."""
            original_recheck(value)
            (self.workspace / "deployment/LCD-SingleStatic-Test.hex").write_bytes(b"changed copy")

        with patch.object(self.adapter, "recheck_frozen", side_effect=mutate_copy), self.assertRaises(RuntimeError):
            self.execute(prepared, hardware)
        self.assertEqual(hardware.events, [])
        original_verify = self.adapter.verify_image_pair

        def mutate_source(hex_payload, elf_payload):
            """Change one source after its bytes were frozen but before preparation ends."""
            result = original_verify(hex_payload, elf_payload)
            (self.workspace / "Core/Src/freertos.c").write_bytes(b"changed during verification")
            return result

        with patch.object(self.adapter, "verify_image_pair", side_effect=mutate_source), self.assertRaises(RuntimeError):
            self.prepare()

    def test_reparse_paths_empty_coverage_and_duplicate_json_keys_are_rejected(self):
        """Exercise path and parser boundaries without creating a Windows junction."""
        original_lstat = Path.lstat
        victim = self.workspace / "Core/Src/freertos.c"

        def reparse_lstat(path, **options):
            """Expose a fixture file as a reparse point at the filesystem boundary."""
            metadata = original_lstat(path, **options)
            if path == victim:
                return SimpleNamespace(st_mode=metadata.st_mode, st_file_attributes=0x400)
            return metadata

        with patch.object(Path, "lstat", reparse_lstat), self.assertRaises(RuntimeError):
            self.prepare()
        for value in (b'{"passed":true,"passed":false}', b"[]"):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                self.adapter.read_json(value)
        _, elf = image_pair(0x08000000, b"abcd")
        with self.assertRaises(RuntimeError):
            self.adapter.verify_image_pair(b":00000001FF\n", elf)

    def test_first_halt_latency_over_budget_aborts_before_backup_program_or_reset(self):
        """Include actual halt completion latency in the final freshness evidence."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared, "halt_latency")
        with self.assertRaises(RuntimeError):
            self.execute(prepared, hardware)
        self.assertGreaterEqual(hardware.events.count("halt"), 2)
        self.assertNotIn("reset_catch", hardware.events)
        self.assertNotIn("program", hardware.events)
        self.assertNotIn("normal_reset", hardware.events)

    def test_post_disable_can_quiet_wait_preserves_complete_transaction(self):
        """Wait through two complete read-only snapshots before one ordinary deployment."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        hardware.post_disable_snapshots = [{"adrc gate motor=7": {"can_idle": "0"}}, {}]
        report = self.execute(prepared, hardware)
        disable_index = hardware.events.index("bench disable 7")
        first_halt_index = hardware.events.index("halt")
        snapshot_commands = ["show state", "adrc status", "adrc gate motor=7", "show diag can", "ping"]
        self.assertEqual(hardware.events[disable_index + 1:first_halt_index],
                         ["show motor 7"] + snapshot_commands * 2)
        self.assertEqual(hardware.clock.sleep_calls, [0.004])
        self.assertEqual(hardware.events.count("bench disable 7"), 1)
        self.assertEqual(hardware.events.count("reset_catch"), 1)
        self.assertEqual(hardware.events.count("program"), 1)
        self.assertEqual(hardware.events.count("normal_reset"), 1)
        self.assertEqual((self.workspace / "deployment/previous_flash_1MiB.bin").stat().st_size, 0x100000)
        self.assertTrue(report["old_image_equal"])
        self.assertTrue(report["usb_swd_pair_verified"])
        self.assertTrue(report["flash_readback_equal"])
        self.assertFalse(report["hardware_qualified"])
        self.assertLessEqual(report["disabled_feedback"]["age_at_halt_completion_upper_ms"], 100)

    def test_default_feedback_clock_has_millisecond_or_better_resolution(self):
        """Inspect the real default clock's Windows source and accuracy contract."""
        default_clock = inspect.signature(self.adapter.deploy).parameters["clock"].default
        clock_info = time.get_clock_info(default_clock.__name__)
        self.assertTrue(clock_info.monotonic)
        self.assertFalse(clock_info.adjustable)
        self.assertLessEqual(clock_info.resolution, 0.001)

    def test_feedback_deadline_includes_actual_host_clock_resolution(self):
        """Reject a quiet snapshot just inside 100 ms when host resolution crosses the bound."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        hardware.clock.now = 0.0
        host_resolution_ms = time.get_clock_info("perf_counter").resolution * 1000
        # Fake feedback transit plus one complete snapshot consume 12 ms;
        # reported age and the firmware age floor consume another 2 ms.
        hardware.post_disable_snapshots = [{"delay_seconds": (86 - host_resolution_ms / 2) / 1000}]
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.execute(prepared, hardware)
        self.assertEqual(hardware.events.count("bench disable 7"), 1)
        self.assertNotIn("halt", hardware.events)
        self.assertEqual(hardware.clock.sleep_calls, [])

    def test_post_disable_can_quiet_expires_without_halt_or_second_disable(self):
        """Keep the original feedback deadline even when CAN becomes quiet too late."""
        prepared = self.prepare()
        pending = {"adrc gate motor=7": {"can_idle": "0"}}
        for name, snapshots in (("persistent", [pending]),
                                ("late_quiet", [pending, {"delay_seconds": 0.09}]),
                                ("expired_pending", [dict(pending, delay_seconds=0.09)])):
            with self.subTest(name=name):
                hardware = FakeHardware(self.adapter, prepared)
                hardware.post_disable_snapshots = snapshots
                with self.assertRaisesRegex(RuntimeError, "expired|quiet"):
                    self.execute(prepared, hardware, self.workspace / name)
                self.assertEqual(hardware.events.count("bench disable 7"), 1)
                self.assertLessEqual(hardware.port.post_disable_snapshot_index + 1, 20)
                for forbidden in ("halt", "reset_catch", "program", "normal_reset"):
                    self.assertNotIn(forbidden, hardware.events)

    def test_post_disable_pending_rechecks_every_non_can_idle_gate(self):
        """Reject any changed safety field immediately during a pending quiet interval."""
        prepared = self.prepare()
        mutations = [("show state", field, value) for field, value in
                     (("enabled", "40"), ("moving", "1"), ("active", "7"), ("fault", "driver"))]
        mutations += [("adrc status", field, value) for field, value in
                      (("owner", "host"), ("fault", "1"), ("state", "1"), ("active_id", "7"))]
        mutations += [("adrc gate motor=7", field, value) for field, value in
                      (("motor", "6"), ("lcd_idle", "0"), ("mode", "1"), ("fields", "0"),
                       ("verified", "0"), ("authority", "1"), ("action", "1"), ("ui_busy", "1"),
                       ("ui_results", "1"), ("proto_results", "1"), ("discovery_active", "1"))]
        mutations += [("show diag can", field, "1") for field in ("drop", "error", "busoff")]
        mutations += [("ping", "boot", "8"), ("ping", "enabled", "40")]
        for index, (command, field, value) in enumerate(mutations):
            with self.subTest(command=command, field=field):
                hardware = FakeHardware(self.adapter, prepared)
                changed = {"adrc gate motor=7": {"can_idle": "0"}}
                changed.setdefault(command, {})[field] = value
                hardware.post_disable_snapshots = [
                    {"adrc gate motor=7": {"can_idle": "0"}}, changed, {}]
                with self.assertRaises(RuntimeError):
                    self.execute(prepared, hardware, self.workspace / f"pending_gate_{index}")
                self.assertEqual(hardware.port.post_disable_snapshot_index, 1)
                self.assertEqual(hardware.clock.sleep_calls, [0.004])
                self.assertEqual(hardware.events.count("bench disable 7"), 1)
                self.assertEqual(hardware.events[-3], command)
                for forbidden in ("halt", "reset_catch", "program", "normal_reset"):
                    self.assertNotIn(forbidden, hardware.events)

    def test_post_disable_can_idle_requires_exact_binary_field(self):
        """Reject missing or malformed quiet fields without another snapshot or sleep."""
        prepared = self.prepare()
        for value in (None, "00", "01", "2", "invalid"):
            with self.subTest(value=value):
                hardware = FakeHardware(self.adapter, prepared)
                hardware.post_disable_snapshots = [{"adrc gate motor=7": {"can_idle": value}}, {}]
                with self.assertRaisesRegex(RuntimeError, "CAN|can_idle"):
                    self.execute(prepared, hardware, self.workspace / f"invalid_quiet_{value}")
                self.assertEqual(hardware.port.post_disable_snapshot_index, 0)
                self.assertEqual(hardware.clock.sleep_calls, [])
                self.assertNotIn("halt", hardware.events)
                self.assertEqual(hardware.events.count("bench disable 7"), 1)

    def test_pre_disable_can_idle_zero_never_retries_or_disables(self):
        """Retain exact can_idle=1 at both initial and old-image snapshots."""
        prepared = self.prepare()
        original_write = FakePort.write
        for snapshot_index in (0, 1):
            hardware = FakeHardware(self.adapter, prepared)

            def pending_before_disable(port, payload):
                """Expose an initial quiet failure using only the fake serial response."""
                result = original_write(port, payload)
                if b"adrc gate motor=7" in payload and port.snapshots == snapshot_index:
                    port.reply = port.reply.replace(b"can_idle=1", b"can_idle=0")
                return result

            with self.subTest(snapshot_index=snapshot_index), patch.object(FakePort, "write", pending_before_disable):
                with self.assertRaises(RuntimeError):
                    self.execute(prepared, hardware, self.workspace / f"initial_quiet_{snapshot_index}")
            self.assertEqual(hardware.events.count("adrc gate motor=7"), snapshot_index + 1)
            self.assertEqual(hardware.clock.sleep_calls, [])
            for forbidden in ("bench disable 7", "halt", "reset_catch", "program", "normal_reset"):
                self.assertNotIn(forbidden, hardware.events)

    def test_all_prehalting_failures_disconnect_without_halt_reset_or_program(self):
        """Fail closed on stale feedback, identity, boot, queue and freshness problems."""
        prepared = self.prepare()
        for scenario in ("wrong_usb", "duplicate_usb", "duplicate_probe", "attach_error", "serial_error", "serial_timeout", "wrong_mcu", "old_mismatch", "stale", "recent_before",
                         "wrong_id", "wrong_master", "fault", "missing_disabled", "predates_disable", "boot_changed", "idle_changed", "queue_busy", "deadline"):
            hardware = FakeHardware(self.adapter, prepared, scenario)
            with self.subTest(scenario=scenario), self.assertRaises(RuntimeError):
                self.execute(prepared, hardware, self.workspace / scenario)
            for forbidden in ("halt", "reset_catch", "normal_reset", "program"):
                self.assertNotIn(forbidden, hardware.events, (scenario, hardware.events))
            self.assertLessEqual(hardware.events.count("bench disable 7"), 1)
            if scenario in ("wrong_mcu", "old_mismatch", "recent_before", "wrong_id", "wrong_master", "queue_busy"):
                self.assertNotIn("bench disable 7", hardware.events)

    def test_can_counters_require_exact_zero_at_every_prehalting_snapshot(self):
        """Reject nonzero, missing and malformed CAN counters before or after DISABLE."""
        prepared = self.prepare()
        healthy = "rx=0 tx=0 drop=0 error=0 busoff=0"
        for query_index in range(3):
            for field in ("drop", "error", "busoff"):
                for value in ("1", None, "00", "invalid"):
                    with self.subTest(query_index=query_index, field=field, value=value):
                        detail = healthy.replace(f" {field}=0", "" if value is None else f" {field}={value}")
                        hardware = FakeHardware(self.adapter, prepared)
                        hardware.can_responses = [healthy] * query_index + [detail]
                        output = self.workspace / f"can_{query_index}_{field}_{value}"
                        with self.assertRaisesRegex(RuntimeError, "CAN"):
                            self.execute(prepared, hardware, output)
                        self.assertEqual(hardware.events.count("show diag can"), query_index + 1)
                        for forbidden in ("halt", "reset_catch", "program", "normal_reset"):
                            self.assertNotIn(forbidden, hardware.events)
                        self.assertEqual(hardware.events.count("bench disable 7"), int(query_index == 2))
                        self.assertEqual("mcu_id" in hardware.events, query_index > 0)

    def test_uid_usb_serial_derivation_requires_uint32_and_matches_descriptor_arithmetic(self):
        """Use 32-bit wrapping addition and only UID1's high sixteen bits for USB SN."""
        self.assertTrue(hasattr(self.adapter, "usb_serial_from_uid"), "Missing STM32 UID to USB serial derivation")
        derive = self.adapter.usb_serial_from_uid
        self.assertEqual(derive((0x12345678, 0x3134ABCD, 0x2636DCF7)), "386B336F3134")
        self.assertEqual(derive((0xFFFFFFFF, 0xFEDCABCD, 2)), "00000001FEDC")
        self.assertEqual(derive((0x11111111, 0xABCD0123, 0x22222222)), "33333333ABCD")
        self.assertEqual(derive((1, 0x0000FFFF, 2)), "000000030000")
        for words in ((1, 2), (1, 2, 3, 4), (True, 2, 3), (-1, 2, 3), (1, 1 << 32, 3),
                      (1, 2, 3.0), (0, 0, 0), (0xFFFFFFFF, 0x12340000, 1)):
            with self.subTest(words=words), self.assertRaisesRegex(RuntimeError, "UID"):
                derive(words)

    def test_wrong_board_failed_uid_reads_and_invalid_words_never_disable_or_halt(self):
        """Reject cross-board USB/SWD pairing before DISABLE despite identical old flash."""
        prepared = self.prepare()
        scenarios = [("wrong_board", (0x11111111, 0x22222222, 0x33333333), None),
                     ("zero_sum", (0xFFFFFFFF, 0x3134ABCD, 1), None)]
        for index in range(3):
            scenarios.append((f"read_error_{index}", (0x12345678, 0x3134ABCD, 0x2636DCF7), index))
            for value in (-1, 1 << 32, True, False, 1.0, "0", None):
                words = [0x12345678, 0x3134ABCD, 0x2636DCF7]
                words[index] = value
                scenarios.append((f"invalid_{index}_{type(value).__name__}_{value}", tuple(words), None))
        for name, words, read_error in scenarios:
            with self.subTest(name=name):
                hardware = FakeHardware(self.adapter, prepared)
                hardware.uid_words = words
                hardware.uid_read_error = read_error
                output = self.workspace / name
                with self.assertRaisesRegex(RuntimeError, "UID"):
                    self.execute(prepared, hardware, output)
                for forbidden in ("bench disable 7", "halt", "reset_catch", "program", "normal_reset"):
                    self.assertNotIn(forbidden, hardware.events)
                self.assertIn("disconnect", hardware.events)
                report = json.loads((output / "report.json").read_text())
                self.assertFalse(report["usb_swd_pair_verified"])
                if name == "wrong_board":
                    self.assertEqual(report["mcu_uid_words"], list(words))
                    self.assertEqual(report["mcu_derived_usb_serial"], "444444442222")
                elif read_error is not None:
                    self.assertEqual(report["mcu_uid_words"], list(words[:read_error]))

    def test_success_order_backup_reset_catch_and_one_normal_boot(self):
        """Complete the transaction once using immutable text and full flash backup."""
        prepared = self.prepare()
        hardware = FakeHardware(self.adapter, prepared)
        report = self.execute(prepared, hardware)
        self.assertFalse(report["hardware_qualified"])
        self.assertTrue(report["flash_readback_equal"])
        self.assertEqual(hardware.events.count("bench disable 7"), 1)
        self.assertEqual(hardware.events.count("reset_catch"), 1)
        self.assertEqual(hardware.events.count("normal_reset"), 1)
        self.assertTrue(report["usb_swd_pair_verified"])
        self.assertEqual(report["mcu_uid_words"], list(hardware.uid_words))
        self.assertEqual(report["mcu_derived_usb_serial"], self.adapter.USB_SERIAL)
        for address in (0x1FF1E800, 0x1FF1E804, 0x1FF1E808):
            event = f"uid_read:{address:#x}"
            self.assertEqual(hardware.events.count(event), 1)
            self.assertLess(hardware.events.index(event), hardware.events.index("bench disable 7"))
        self.assertLess(hardware.events.index("serial_open"), hardware.events.index("attach"))
        self.assertLess(hardware.events.index("mcu_id"), hardware.events.index("bench disable 7"))
        self.assertLess(hardware.events.index("bench disable 7"), hardware.events.index("halt"))
        self.assertLess(hardware.events.index("reset_catch"), hardware.events.index("program"))
        self.assertLess(hardware.events.index("program"), hardware.events.index("normal_reset"))
        self.assertEqual((self.workspace / "deployment/previous_flash_1MiB.bin").stat().st_size, 0x100000)
        self.assertLessEqual(report["disabled_feedback"]["age_at_first_halt_upper_ms"], 100)
        self.assertEqual(hardware.options, self.adapter.SESSION_OPTIONS)
        self.assertEqual(hardware.options["connect_mode"], "attach")
        self.assertFalse(hardware.options["resume_on_disconnect"])

    def test_posthalt_baseexceptions_stay_halted_without_retry_or_failure_reset(self):
        """Catch KeyboardInterrupt as well as ordinary exceptions throughout post-halt stages."""
        prepared = self.prepare()
        for scenario in ("halt_error", "backup_error", "backup_interrupt", "reset_catch_error", "exception_context",
                         "program_error", "program_interrupt", "readback_mismatch", "reset_error", "disconnect_interrupt"):
            hardware = FakeHardware(self.adapter, prepared, scenario)
            with self.subTest(scenario=scenario), self.assertRaises(BaseException):
                self.execute(prepared, hardware, self.workspace / scenario)
            self.assertEqual(hardware.target.state, "halted")
            self.assertGreaterEqual(hardware.events.count("halt"), 2)
            self.assertLessEqual(hardware.events.count("program"), 1)
            self.assertLessEqual(hardware.events.count("reset_catch"), 1)
            self.assertEqual(hardware.events.count("normal_reset"), int(scenario in ("reset_error", "disconnect_interrupt")))
            if scenario in ("exception_context", "reset_catch_error"):
                self.assertNotIn("program", hardware.events)


if __name__ == "__main__":
    unittest.main(verbosity=2)
