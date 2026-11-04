"""Test the live wrapper's offline and transport boundaries using injected ports; never connect to hardware."""
from __future__ import annotations

from contextlib import ExitStack, redirect_stdout, redirect_stderr
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

from test_pmax_persistence_client import synthetic_records

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import pmax_persistence_live as LIVE


class FixtureClock:
    """Advance a bounded test clock on each read without waiting or using a device clock."""

    def __init__(self) -> None:
        """Start at zero for deterministic two-second timeouts."""
        self.value = 0.0

    def __call__(self) -> float:
        """Advance by one tenth second so an unresponsive fake port cannot block a test."""
        self.value += 0.1
        return self.value


class FixturePort:
    """Own queued synthetic fragments and record every write without a serial dependency."""

    def __init__(self, chunks=(), *, short_write=False) -> None:
        """Copy deterministic reply fragments; empty input models an idle timeout."""
        self.chunks = list(chunks)
        self.writes = []
        self.short_write = short_write

    def write(self, payload: bytes) -> int:
        """Retain one exact command and optionally simulate its partial acceptance."""
        self.writes.append(payload)
        return len(payload) - int(self.short_write)

    def read_until(self, terminator: bytes, size: int) -> bytes:
        """Return queued fragments only; real PySerial framing remains outside this fake."""
        if not self.chunks:
            return b""
        return self.chunks.pop(0)


class LiveBoundaryTests(unittest.TestCase):
    """Exercise bounded raw framing and explicit live flags without any Save, CAN or probe call."""

    def test_default_is_offline(self) -> None:
        """Default validation never invokes execute or hardware loading."""
        prepared = type("PreparedFixture", (), {"image_payload": b"frozen"})()
        with patch.object(LIVE, "prepare_release", return_value=(prepared, {})), \
                patch.object(LIVE, "execute", side_effect=AssertionError("live reached")), redirect_stdout(StringIO()):
            self.assertEqual(LIVE.main(["--manifest", "release.json", "--manifest-sha256", "1" * 64]), 0)

    def test_save_requires_every_current_condition(self) -> None:
        """Missing any fresh condition fails before manifest preparation or device discovery."""
        conditions = ("--cold-motor-power-cycle-confirmed", "--supported-unloaded", "--lcd-released", "--motor-power-on")
        for missing in conditions:
            with self.subTest(missing=missing), patch.object(LIVE, "prepare_release") as prepare, redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit):
                    LIVE.main(["--manifest", "release.json", "--manifest-sha256", "1" * 64,
                               "--save", *(flag for flag in conditions if flag != missing)])
                prepare.assert_not_called()

    def test_direct_execute_requires_confirmation(self) -> None:
        """Calling the Python API does not bypass the explicit current-condition gate."""
        with patch.object(LIVE.shared, "load_hardware", side_effect=AssertionError("hardware reached")):
            with self.assertRaises(RuntimeError):
                LIVE.execute(None, {}, "1" * 64, None)

    def test_partial_reply_preserves_exact_crlf(self) -> None:
        """A timeout-separated CDC fragment is assembled once and not mistaken for a malformed line."""
        port = FixturePort((b"ok 4 adrc per", b"sist_status schema=1 id=3\r", b"\n"))
        trace = []
        exchange = LIVE.SerialExchange(port, trace, FixtureClock())
        self.assertEqual(exchange.exchange(4, "adrc persist_status"), "ok 4 adrc persist_status schema=1 id=3\r\n")
        self.assertEqual(port.writes, [b"4 adrc persist_status\r\n"])
        self.assertEqual(len([item for item in trace if item["direction"] == "rx_chunk"]), 3)

    def test_legacy_lf_and_unmatched_diagnostic(self) -> None:
        """An unrelated diagnostic is retained while the actual legacy LF ping reaches its precise caller."""
        port = FixturePort((b"event idle\n", b"ok 7 ping boot=12\n"))
        self.assertEqual(LIVE.SerialExchange(port, [], FixtureClock()).exchange(7, "ping"), "ok 7 ping boot=12\n")
        self.assertEqual(len(port.writes), 1)

    def test_partial_write_is_not_retried(self) -> None:
        """A partial commit write fails after one call and cannot replay a potentially accepted mutation."""
        port = FixturePort(short_write=True)
        with self.assertRaises(RuntimeError):
            LIVE.SerialExchange(port, [], FixtureClock()).exchange(9, "adrc persist_commit id=3")
        self.assertEqual(len(port.writes), 1)

    def test_timeouts_and_oversize_do_not_retry(self) -> None:
        """Empty, partial and oversized replies terminate under the original single-exchange budget."""
        for chunks in ((), (b"ok 9 adrc pers",), (b"x" * 520,)):
            with self.subTest(chunks=len(chunks)):
                port = FixturePort(chunks)
                with self.assertRaises((RuntimeError, TimeoutError)):
                    LIVE.SerialExchange(port, [], FixtureClock()).exchange(9, "adrc persist_status")
                self.assertEqual(len(port.writes), 1)

    def test_invalid_bytes_are_retained_before_rejection(self) -> None:
        """Raw non-ASCII evidence survives even when strict text decoding refuses it."""
        trace = []
        with self.assertRaises(UnicodeDecodeError):
            LIVE.SerialExchange(FixturePort((b"ok 9 \xff\n",)), trace, FixtureClock()).exchange(9, "ping")
        self.assertEqual(trace[-1]["raw_hex"], b"ok 9 \xff\n".hex())

    def test_arbitrary_commands_never_reach_write(self) -> None:
        """Motion, arbitrary parameter writes and invalid request identities cannot use this transport adapter."""
        for request_id, command in ((1, "bench enable 7"), (1, "adrc persist_commit id=3 raw=1"), (True, "ping"), (0, "ping")):
            with self.subTest(command=command, request_id=request_id):
                port = FixturePort()
                with self.assertRaises(RuntimeError):
                    LIVE.SerialExchange(port, [], FixtureClock()).exchange(request_id, command)
                self.assertFalse(port.writes)

    def test_flash_comparison_is_chunked_and_keeps_boot_bound_ping(self) -> None:
        """The real wrapper reader compares all occupied bytes and pings without a halt or reset API."""
        payload = bytes(index % 251 for index in range(9000))
        prepared = SimpleNamespace(new_segments=((0x08000000, payload),))
        target = SimpleNamespace(get_state=lambda: "RUNNING")
        hardware = SimpleNamespace(Target=SimpleNamespace(State=SimpleNamespace(RUNNING="RUNNING")))
        protocol = SimpleNamespace(exchange=lambda command: ({"boot": "1234", "enabled": "00"}, 0, 0))

        def read_flash(_target, start, end):
            """Serve only the requested synthetic occupied segment and reject excess bytes."""
            self.assertIs(_target, target)
            self.assertLessEqual(end - start, 4096)
            return payload[start - 0x08000000:end - 0x08000000]

        with patch.object(LIVE.time, "perf_counter", FixtureClock()), \
                patch.object(LIVE.shared, "read_flash", side_effect=read_flash) as reads:
            self.assertEqual(LIVE.verify_running_image(target, hardware, prepared, protocol, "1234"), len(payload))
            self.assertEqual(reads.call_count, 3)
        bad_protocol = SimpleNamespace(exchange=lambda command: ({"boot": "changed", "enabled": "00"}, 0, 0))
        with patch.object(LIVE.time, "perf_counter", FixtureClock()), patch.object(LIVE.shared, "read_flash", side_effect=read_flash):
            with self.assertRaises(RuntimeError):
                LIVE.verify_running_image(target, hardware, prepared, bad_protocol, "1234")

    def test_outer_failure_and_cleanup_retain_claim_and_partial_evidence(self) -> None:
        """A fake lost commit response and a fake close error preserve the original exception, once and transcript."""
        for close_failed in (False, True):
            with self.subTest(close_failed=close_failed):
                case = ROOT / "output/repair_20261023" / ("w_" + uuid.uuid4().hex[:10])
                once = case / "once"
                once.mkdir(parents=True, exist_ok=False)
                token = once / "id7_pmax512_pos2.once"
                events = []

                class Port(FixturePort):
                    """Represent one exact fake CDC port and optional cleanup failure."""

                    def __init__(self, **_options) -> None:
                        """Queue only the real legacy HELLO shape; protocol preflight is separately injected."""
                        super().__init__((b"ok 900000001 hello boot=1234 watchdog_ms=1000\n",))

                    def open(self) -> None:
                        """Record port opening without a device."""
                        events.append("serial_open")

                    def close(self) -> None:
                        """Model successful or failed transport cleanup without retry."""
                        events.append("serial_close")
                        if close_failed: raise RuntimeError("injected close failure")

                class Session:
                    """Expose only a running fake chip; no halt/reset methods exist."""

                    def __enter__(self):
                        """Open the injected read-only SWD session."""
                        self.open()
                        return self

                    def open(self) -> None:
                        """Provide the same explicit session lifecycle as the existing audited shared deploy helper."""
                        self.target = SimpleNamespace(get_state=lambda: "RUNNING", read32=lambda _address: 0x483)
                        self.probe = SimpleNamespace(unique_id=LIVE.shared.PROBE_UID)

                    def __exit__(self, *_exception) -> None:
                        """Close only the fake session without changing core state."""
                        self.close()

                    def close(self) -> None:
                        """Record exactly one explicit cleanup without changing core state."""
                        events.append("swd_close")

                class Client:
                    """Model an already claimed commit losing its response; the real library is tested separately."""

                    def __init__(self, *_arguments, **_options) -> None:
                        """Retain a diagnostic partial command and raw baseline fixture."""
                        self.transcript = [{"command": "adrc persist_commit id=3", "reply": None}]
                        self.evidence = {"baseline": synthetic_records(1), "once_token": str(token)}

                    def run_prepare_save(self):
                        """Fail after one synthetic durable claim; never repeat the command or remove the claim."""
                        events.append("client")
                        with token.open("xb") as stream: stream.write(b"retained claim")
                        raise TimeoutError("injected commit response lost")

                hardware = SimpleNamespace(
                    ports=lambda: [SimpleNamespace(vid=0x0483, pid=0x5740, serial_number=LIVE.shared.USB_SERIAL, device="FAKE")],
                    Serial=Port, probes=lambda: [SimpleNamespace(unique_id=LIVE.shared.PROBE_UID)],
                    Session=lambda *_args, **_options: Session(),
                    Target=SimpleNamespace(State=SimpleNamespace(RUNNING="RUNNING")))
                protocol = SimpleNamespace(exchange=lambda _command: ({"mode": "1"}, 0, 0))
                with ExitStack() as stack:
                    stack.enter_context(patch.object(LIVE, "BASE", case))
                    stack.enter_context(patch.object(LIVE, "ONCE_DIRECTORY", once))
                    stack.enter_context(patch.object(LIVE, "PersistenceClient", Client))
                    stack.enter_context(patch.object(LIVE.shared, "load_hardware", return_value=hardware))
                    stack.enter_context(patch.object(LIVE.shared, "Protocol", return_value=protocol))
                    stack.enter_context(patch.object(LIVE, "PreflightProtocol", return_value=protocol, create=True))
                    stack.enter_context(patch.object(LIVE.shared, "idle_snapshot", return_value="1234"))
                    stack.enter_context(patch.object(LIVE.shared, "verify_usb_swd_pair", side_effect=lambda *_args: events.append("pair")))
                    stack.enter_context(patch.object(LIVE, "verify_running_image", side_effect=lambda *_args: events.append("image") or 2))
                    stack.enter_context(patch.object(LIVE.shared, "recheck_frozen", side_effect=lambda *_args: events.append("freeze")))
                    stack.enter_context(patch.object(LIVE.shared, "disable_once", side_effect=lambda *_args: events.append("disable") or {}))
                    stack.enter_context(patch.object(LIVE.shared, "wait_post_disable_idle", return_value=None))
                    stack.enter_context(patch.object(LIVE.shared, "feedback_age_upper_ms", return_value=1))
                    stack.enter_context(redirect_stdout(StringIO()))
                    with self.assertRaisesRegex(TimeoutError, "commit response lost"):
                        LIVE.execute(SimpleNamespace(), {"hex_sha256": "1" * 64}, "2" * 64, None, supported_cold_confirmed=True)
                self.assertEqual(token.read_bytes(), b"retained claim")
                reports = list(case.glob("persistence_live_*/report.json"))
                self.assertEqual(len(reports), 1)
                report = json.loads(reports[0].read_text(encoding="utf-8"))
                self.assertFalse(report["completed"])
                self.assertIn("commit response lost", report["error"])
                self.assertEqual(report["client_transcript"][0]["reply"], None)
                self.assertEqual("close_error" in report, close_failed)
                self.assertLess(events.index("pair"), events.index("image"))
                self.assertLess(events.index("image"), events.index("freeze"))
                self.assertLess(events.index("freeze"), events.index("disable"))
                self.assertLess(events.index("disable"), events.index("client"))

    def test_original_cold_baseline_is_bound_and_validated_before_connection(self) -> None:
        """Wrong metadata or incomplete original timing cannot progress to any live connection."""
        directory = ROOT / "output/repair_20261023" / ("b_" + uuid.uuid4().hex[:10])
        directory.mkdir(parents=True, exist_ok=False)
        path = directory / "baseline.json"
        original = dict(schema="id7-cold-baseline-v1", usb_serial=LIVE.shared.USB_SERIAL,
                        user_confirmed_motor24v_cold=True, hex_sha256="1" * 64,
                        release_manifest_sha256="2" * 64, records=synthetic_records(1))
        variants = ("valid", "usb", "hex", "release", "cold", "missing", "phase", "time")
        for variant in variants:
            with self.subTest(variant=variant):
                value = deepcopy(original)
                if variant == "usb": value["usb_serial"] = "wrong"
                if variant == "hex": value["hex_sha256"] = "3" * 64
                if variant == "release": value["release_manifest_sha256"] = "3" * 64
                if variant == "cold": value["user_confirmed_motor24v_cold"] = 1
                if variant == "missing": value["records"].pop()
                if variant == "phase": value["records"][0]["identity"]["phase"] = 4
                if variant == "time": value["records"][0]["rx_lower_us"] = 0
                payload = json.dumps(value).encode("utf-8")
                path.write_bytes(payload)
                with patch.object(LIVE.shared, "load_hardware", side_effect=AssertionError("hardware reached")):
                    if variant == "valid":
                        self.assertEqual(LIVE.load_cold_baseline(path, LIVE.shared.digest(payload), {"hex_sha256": "1" * 64}, "2" * 64), original["records"])
                    else:
                        with self.assertRaises((RuntimeError, ValueError)):
                            LIVE.load_cold_baseline(path, LIVE.shared.digest(payload), {"hex_sha256": "1" * 64}, "2" * 64)

    def test_swd_cleanup_does_not_hide_flash_failure(self) -> None:
        """Both pre-fix context-manager and fixed explicit lifecycle receive the same real body/close error pair."""
        case = ROOT / "output/repair_20261023" / ("s_" + uuid.uuid4().hex[:10])
        once = case / "once"
        once.mkdir(parents=True, exist_ok=False)
        closes = []

        class Port(FixturePort):
            """Model only setup and cleanup, with no responses needed after failed Flash verification."""

            def __init__(self, **_options) -> None:
                """Open no actual resource."""
                super().__init__()

            def open(self) -> None:
                """Keep serial opening inert for this failure-path test."""

            def close(self) -> None:
                """Keep serial cleanup successful to isolate the SWD close error."""

        class Session:
            """Expose equivalent context-manager and explicit lifecycles for an unchanged reproducer."""

            def open(self) -> None:
                """Return only the fixed fake RUNNING chip and probe identity."""
                self.target = SimpleNamespace(get_state=lambda: "RUNNING", read32=lambda _address: 0x483)
                self.probe = SimpleNamespace(unique_id=LIVE.shared.PROBE_UID)

            def close(self) -> None:
                """Fail exactly once during cleanup, after an independent Flash verification error."""
                closes.append(1)
                raise OSError("injected SWD close failure")

            def __enter__(self):
                """Let the pre-fix implementation execute the same fixture."""
                self.open()
                return self

            def __exit__(self, *_exception) -> None:
                """Inject the same close error through the pre-fix context manager."""
                self.close()

        hardware = SimpleNamespace(
            ports=lambda: [SimpleNamespace(vid=0x0483, pid=0x5740, serial_number=LIVE.shared.USB_SERIAL, device="FAKE")],
            Serial=Port, probes=lambda: [SimpleNamespace(unique_id=LIVE.shared.PROBE_UID)],
            Session=lambda *_args, **_options: Session(), Target=SimpleNamespace(State=SimpleNamespace(RUNNING="RUNNING")))
        protocol = SimpleNamespace(exchange=lambda _command: ({"mode": "1"}, 0, 0))
        with ExitStack() as stack:
            stack.enter_context(patch.object(LIVE, "BASE", case))
            stack.enter_context(patch.object(LIVE, "ONCE_DIRECTORY", once))
            stack.enter_context(patch.object(LIVE.shared, "load_hardware", return_value=hardware))
            stack.enter_context(patch.object(LIVE.shared, "Protocol", return_value=protocol))
            stack.enter_context(patch.object(LIVE, "PreflightProtocol", return_value=protocol, create=True))
            stack.enter_context(patch.object(LIVE.shared, "idle_snapshot", return_value="1234"))
            stack.enter_context(patch.object(LIVE.shared, "verify_usb_swd_pair", return_value=None))
            stack.enter_context(patch.object(LIVE, "verify_running_image", side_effect=ValueError("injected Flash mismatch")))
            stack.enter_context(redirect_stdout(StringIO()))
            with self.assertRaisesRegex(ValueError, "injected Flash mismatch"):
                LIVE.execute(SimpleNamespace(), {"hex_sha256": "1" * 64}, "2" * 64, None, supported_cold_confirmed=True)
        report = json.loads(next(case.glob("persistence_live_*/report.json")).read_text(encoding="utf-8"))
        self.assertIn("Flash mismatch", report["error"])
        self.assertIn("SWD close failure", report["swd_close_error"])
        self.assertEqual(closes, [1])

    def test_preflight_reuses_bounded_raw_port(self) -> None:
        """All ordinary preflight commands use the same fragment-aware raw evidence path as typed maintenance."""
        port = FixturePort((b"ok 101 pi", b"ng state=idle enabled=00 boot=12\r", b"\n"))
        raw, logical = [], []
        transport = LIVE.SerialExchange(port, raw, FixtureClock())
        protocol = LIVE.PreflightProtocol(transport, logical, first_request_id=100)
        fields, sent, received = protocol.exchange("ping")
        self.assertEqual(fields["boot"], "12")
        self.assertLessEqual(sent, received)
        self.assertEqual(raw[-1]["raw"], "ok 101 ping state=idle enabled=00 boot=12\r\n")
        self.assertEqual(len(port.writes), 1)
        for chunks in ((b"x" * 520,), (b"ok 101 ping boot=\xff\n",)):
            with self.subTest(chunks=chunks[0][:4]):
                port = FixturePort(chunks)
                raw = []
                protocol = LIVE.PreflightProtocol(LIVE.SerialExchange(port, raw, FixtureClock()), [], first_request_id=100)
                with self.assertRaises((RuntimeError, UnicodeDecodeError)): protocol.exchange("ping")
                self.assertTrue(any(item.get("raw_hex") == chunks[0].hex() for item in raw))

    def test_preflight_disable_is_single_attempt(self) -> None:
        """The initial ordinary DISABLE cannot be replayed after either success or a partial write."""
        for short in (False, True):
            with self.subTest(short=short):
                port = FixturePort((b"ok 101 bench disable accepted=1\n",), short_write=short)
                protocol = LIVE.PreflightProtocol(LIVE.SerialExchange(port, [], FixtureClock()), [], first_request_id=100)
                if short:
                    with self.assertRaises(RuntimeError): protocol.exchange("bench disable 7")
                else:
                    self.assertEqual(protocol.exchange("bench disable 7")[0], {"accepted": "1"})
                with self.assertRaises(RuntimeError): protocol.exchange("bench disable 7")
                self.assertEqual(len(port.writes), 1)

    def test_report_write_failure_preserves_original_exception(self) -> None:
        """A real execute failure before hardware import remains primary even if its sole report write also fails."""
        case = ROOT / "output/repair_20261023" / ("e_" + uuid.uuid4().hex[:10])
        once = case / "once"
        once.mkdir(parents=True, exist_ok=False)
        attempts = []
        original_writer = Path.write_text

        def fail_only_report(path, payload, *arguments, **options):
            """Inject an evidence-write error only at the actual report path without touching any device or token."""
            if path.name == "report.json" and path.parent.parent == case:
                attempts.append(path)
                raise OSError("injected report write failure")
            return original_writer(path, payload, *arguments, **options)

        diagnostics = StringIO()
        with patch.object(LIVE, "BASE", case), patch.object(LIVE, "ONCE_DIRECTORY", once), \
                patch.object(LIVE.shared, "load_hardware", side_effect=ValueError("injected original preflight failure")), \
                patch.object(Path, "write_text", fail_only_report), redirect_stdout(StringIO()), redirect_stderr(diagnostics):
            with self.assertRaisesRegex(ValueError, "original preflight failure"):
                LIVE.execute(None, {"hex_sha256": "1" * 64}, "2" * 64, None, supported_cold_confirmed=True)
        self.assertEqual(len(attempts), 1)
        self.assertIn("report write failure", diagnostics.getvalue())
        self.assertIn("original preflight failure", diagnostics.getvalue())
        self.assertTrue(once.is_dir())


if __name__ == "__main__":
    unittest.main()
