"""Offline tests for passive, generation-bound static startup trace export.

All responses and serial boundaries are synthetic; no device is enumerated,
opened or actuated, and fixtures never delete files.
"""

import copy
import importlib.util
from pathlib import Path
import sys
import struct
import unittest


ROOT = Path(__file__).resolve().parents[2]
EXPORTER = ROOT / "tools/export_static_startup_trace.py"


class FakeTrace:
    """Provide deterministic immutable trace fragments and retain requested commands."""

    def __init__(self):
        """Start with one frozen trace and a single raw startup record."""
        self.commands = []
        record = bytearray(96)
        struct.pack_into("<QQ6I7fHH6B8s", record, 0, 1000, 1001, 0, 5, 11, 22, 3, 0,
            1.0, 1.01, 0.2, 0.3, 4.16, 0.2, 0.0, 0x17, 15, 7, 4, 7, 1, 8, 1,
            bytes.fromhex("1780008008002219"))
        terminal = bytearray(record)
        struct.pack_into("<I", terminal, 16, 1)
        terminal[72] = 9
        self.records = [bytes(record), bytes(terminal)]
        header = bytearray(832)
        struct.pack_into("<12IQ6I", header, 0, 1, 9, 2, 2, 0, 0, 0, 1, 0, 1, 96, 832,
            900, 11, 22, 0x40, 0, 1, 0)
        struct.pack_into("<6I11f3I", header, 80 + 6 * 80, 4, 5, 3, 7, 0x17, 1,
            1.0, 19.2032, 4.16, 0.2, 0.0, 12.5, 200.0, 10.0, -12.5, 12.5, 1.0, 0, 0, 0)
        header[736:832] = terminal
        self.header = bytes(header)
        self.status = dict(schema="1", generation="9", active="0", frozen="1", count="2",
            total="2", overflow="0", gaps="0", record_size="96", header_size="832",
            target_acceptance="unknown", first_fault="0", terminal="1", valid="1")
        self.mutate = None

    def exchange(self, command):
        """Reply only to the documented read-only status and fragment commands."""
        self.commands.append(command)
        if command == "ping":
            return dict(boot="123")
        if command == "static trace_status":
            response = copy.copy(self.status)
        else:
            self.assert_command(command)
            fields = dict(item.split("=", 1) for item in command.split()[2:])
            index, part = int(fields["index"]), int(fields["part"])
            payload = self.header if index == 4294967295 else self.records[index]
            fragment = payload[part * 128:(part + 1) * 128]
            response = dict(valid="1", generation="9", index=str(index), part=str(part),
                parts=str((len(payload) + 127) // 128), bytes=str(len(fragment)), hex=fragment.hex())
        if self.mutate:
            self.mutate(command, response)
        return response

    def assert_command(self, command):
        """Reject any accidental operation outside the trace query grammar."""
        assert command.startswith("static trace index=") and " part=" in command and " generation=" in command


class ExportTests(unittest.TestCase):
    """Verify query identity, completeness, immutability and bounded data handling."""

    def setUp(self):
        """Import the pure module only after its availability test can report RED."""
        if not EXPORTER.is_file():
            if self._testMethodName == "test_exporter_exists":
                return
            self.skipTest("Availability assertion supplies the intentional RED")
        specification = importlib.util.spec_from_file_location("static_trace_export", EXPORTER)
        self.exporter = importlib.util.module_from_spec(specification)
        sys.modules[specification.name] = self.exporter
        specification.loader.exec_module(self.exporter)
        self.fake = FakeTrace()

    def test_exporter_exists(self):
        """A missing passive exporter must fail without opening hardware or import errors."""
        self.assertTrue(EXPORTER.is_file(), "Missing passive startup trace exporter")

    def test_takeover_and_pos_disable_phase_records(self):
        """Retain the new strict phase identities without changing the 96-byte record layout."""
        for phase, name in ((5, "pos_disable"), (6, "takeover")):
            record = bytearray(self.fake.records[0])
            record[73] = phase
            decoded = self.exporter.decode_record(bytes(record), 11, 22)
            self.assertEqual(decoded["phase_name"], name)

    def test_frozen_export_retains_exact_bytes_and_uses_only_queries(self):
        """Read every bounded part exactly once, with before/after boot and generation checks."""
        result = self.exporter.capture(self.fake.exchange)
        self.assertEqual(result["header_hex"], self.fake.header.hex())
        self.assertEqual(result["records"][0]["raw_hex"], self.fake.records[0].hex())
        self.assertEqual(result["generation"], 9)
        self.assertFalse(result["continuous_coverage_claimed"])
        self.assertTrue(all(command == "ping" or command.startswith("static trace")
                            for command in self.fake.commands))

    def test_active_unfrozen_unknown_size_and_oversized_traces_are_rejected(self):
        """A running or malformed trace cannot be accepted as a stable diagnostic capture."""
        for field, value in (("active", "1"), ("frozen", "0"), ("count", "1000000"),
                ("header_size", "2049"), ("record_size", "97"), ("generation", "0")):
            self.fake = FakeTrace()
            self.fake.status[field] = value
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                self.exporter.capture(self.fake.exchange)
            self.assertFalse(any("index=" in command for command in self.fake.commands))

    def test_fragment_identity_length_and_hex_are_checked(self):
        """Reject reordered, borrowed, truncated and invalid raw fragments."""
        for field, value in (("generation", "8"), ("index", "3"), ("part", "2"),
                ("parts", "6"), ("bytes", "1"), ("hex", "zz")):
            self.fake = FakeTrace()
            self.fake.mutate = lambda command, response, field=field, value=value: (
                response.update({field: value}) if "index=" in command else None)
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                self.exporter.capture(self.fake.exchange)

    def test_changed_boot_and_changed_terminal_snapshot_are_rejected(self):
        """Do not merge records from different boots or generations."""
        for changed in ("boot", "generation", "total", "gaps"):
            self.fake = FakeTrace()
            original = self.fake.exchange
            calls = {}

            def exchange(command):
                """Change only the terminal snapshot after the fragments were collected."""
                response = original(command)
                calls[command] = calls.get(command, 0) + 1
                if changed == "boot" and command == "ping" and calls[command] == 2:
                    response["boot"] = "124"
                elif changed != "boot" and command == "static trace_status" and calls[command] == 2:
                    response[changed] = str(int(response[changed]) + 1)
                return response

            with self.subTest(changed=changed), self.assertRaises(RuntimeError):
                self.exporter.capture(exchange)

    def test_parser_rejects_duplicate_fields_and_client_rejects_motion(self):
        """Prevent ambiguous replies and prohibit hidden DISABLE/ENABLE/START commands."""
        with self.assertRaises(RuntimeError):
            self.exporter.parse_fields("generation=1 generation=2")
        for command in ("bench disable 7", "bench mit 7", "static start", "static trace clear"):
            with self.subTest(command=command), self.assertRaises(RuntimeError):
                self.exporter.query_kind(command)

    def test_wire_decoder_exposes_native_measurements_and_rejects_inconsistent_header(self):
        """Validate internal ABI identities and units rather than only preserving opaque hex."""
        result = self.exporter.capture(self.fake.exchange)
        self.assertAlmostEqual(result["records"][0]["decoded"]["target_rad"], 1.0)
        self.assertAlmostEqual(result["records"][0]["decoded"]["torque_nm"], 0.3, places=5)
        self.assertEqual(result["records"][0]["decoded"]["event_name"], "accepted_rx")
        self.assertEqual(result["decoded_header"]["axes"][6]["esc_id"], 7)
        for offset, value in ((0, 2), (4, 8), (8, 3), (40, 80), (44, 136), (68, 1)):
            self.fake = FakeTrace()
            header = bytearray(self.fake.header)
            struct.pack_into("<I", header, offset, value)
            self.fake.header = bytes(header)
            with self.subTest(offset=offset), self.assertRaises(RuntimeError):
                self.exporter.capture(self.fake.exchange)

    def test_wire_records_require_request_identity_serial_order_and_original_padding(self):
        """Reject records from another request, duplicated sequence or malformed original bytes."""
        for offset, value in ((24, 12), (28, 23), (16, 9), (72, 99), (76, 9), (86, 1)):
            self.fake = FakeTrace()
            record = bytearray(self.fake.records[0])
            if offset < 40:
                struct.pack_into("<I", record, offset, value)
            else:
                record[offset] = value
            self.fake.records[0] = bytes(record)
            with self.subTest(offset=offset), self.assertRaises(RuntimeError):
                self.exporter.capture(self.fake.exchange)

    def test_ring_counts_and_overflow_boundaries_are_consistent(self):
        """Retain first64+tail64 coverage and reject invented, missing or out-of-total serials."""
        for total in (128, 129, 144):
            self.fake = FakeTrace()
            count = min(total, 128)
            records = []
            for index in range(count):
                record = bytearray(self.fake.records[0])
                serial = index if index < 64 or total <= 128 else total - 128 + index
                struct.pack_into("<I", record, 16, serial)
                records.append(bytes(record))
            terminal = bytearray(records[-1])
            terminal[72] = 9
            records[-1] = bytes(terminal)
            header = bytearray(self.fake.header)
            struct.pack_into("<4I", header, 8, count, total, total - count, total - count)
            header[736:832] = terminal
            self.fake.header, self.fake.records = bytes(header), records
            self.fake.status.update(count=str(count), total=str(total),
                overflow=str(total - count), gaps=str(total - count))
            result = self.exporter.capture(self.fake.exchange)
            self.assertEqual(result["records"][-1]["decoded"]["serial"], total - 1)
        self.fake = FakeTrace()
        self.fake.status["total"] = "3"
        header = bytearray(self.fake.header)
        struct.pack_into("<I", header, 12, 3)
        self.fake.header = bytes(header)
        with self.assertRaises(RuntimeError):
            self.exporter.capture(self.fake.exchange)

    def test_original_mit_bytes_and_raw_feedback_are_decoded_with_frozen_ranges(self):
        """Expose encoded zero quantization and both raw temperatures without unit conversion guesses."""
        result = self.exporter.capture(self.fake.exchange)
        decoded = result["records"][0]["decoded"]
        self.assertEqual(decoded["wire"]["mos_temperature_c"], 34)
        self.assertEqual(decoded["wire"]["rotor_temperature_c"], 25)
        command = dict(decoded, event=4, event_name="actual_tx", phase=2,
                       can_raw_hex="82a97ff0220a37ff", can_id=7)
        wire = self.exporter.decode_wire_payload(command, result["decoded_header"]["axes"])
        self.assertAlmostEqual(wire["velocity_desired_rad_s"], -200.0 / 4095, places=5)
        self.assertAlmostEqual(wire["torque_ff_encoded_nm"], -10.0 / 4095, places=5)
        self.assertAlmostEqual(wire["kp_encoded"], 34 * 500 / 4095, places=5)
        self.assertEqual(wire["target_acceptance"], "unknown")

    def test_ring_fault_and_terminal_events_require_matching_permanent_copies(self):
        """Reject an unmarked fault event and any extra terminal event before the frozen tail."""
        for event in (8, 9):
            self.fake = FakeTrace()
            record = bytearray(self.fake.records[0])
            record[72] = event
            self.fake.records[0] = bytes(record)
            with self.subTest(event=event), self.assertRaises(RuntimeError):
                self.exporter.capture(self.fake.exchange)

    def test_first_fault_keeps_protocol_error_and_local_guard_reason_separate(self):
        """Decode the event-specific reason contract without confusing it with driver flags."""
        record = bytearray(self.fake.records[0])
        record[72] = 8
        struct.pack_into("<I", record, 36, (6 << 16) | 9)
        header = bytearray(self.fake.header)
        struct.pack_into("<I", header, 32, 1)
        header[640:736] = record
        self.fake.records[0], self.fake.header = bytes(record), bytes(header)
        self.fake.status["first_fault"] = "1"
        result = self.exporter.capture(self.fake.exchange)
        reason = result["decoded_header"]["first_fault_record"]
        self.assertEqual(reason["protocol_error"], 9)
        self.assertEqual(reason["local_guard_reason"], 6)

    def test_scheduler_acceptance_is_separate_from_actual_bus_transmission(self):
        """Decode the queued event while refusing to label it actual TX success."""
        record = bytearray(self.fake.records[0])
        record[72] = 12
        record[73] = 2
        record[78:86] = bytes.fromhex("82a97ff0220a37ff")
        self.fake.records[0] = bytes(record)
        result = self.exporter.capture(self.fake.exchange)
        decoded = result["records"][0]["decoded"]
        self.assertEqual(decoded["event_name"], "queued")
        self.assertFalse(decoded["wire"]["actual_tx_proven"])

    def test_emergency_cleanup_outcome_preserves_unknown_instead_of_claiming_success(self):
        """Decode the unchanged header's explicit passive emergency-cleanup metadata."""
        for outcome, label in ((0, "not_applicable_or_unverified"),
                (1, "new_disabled_stationary_observed"), (2, "timeout_unknown")):
            self.fake = FakeTrace()
            header = bytearray(self.fake.header)
            struct.pack_into("<I", header, 76, outcome)
            self.fake.header = bytes(header)
            result = self.exporter.capture(self.fake.exchange)
            self.assertEqual(result["decoded_header"]["cleanup_outcome_name"], label)
        self.fake = FakeTrace()
        header = bytearray(self.fake.header)
        struct.pack_into("<I", header, 76, 3)
        self.fake.header = bytes(header)
        with self.assertRaises(RuntimeError):
            self.exporter.capture(self.fake.exchange)

    def test_rejected_raw_feedback_with_unknown_id_remains_exportable(self):
        """Retain a malformed feedback ID as raw evidence without assigning it an axis profile."""
        record = bytearray(self.fake.records[0])
        record[72] = 6
        record[74] = 9
        record[78] = 0x19
        self.fake.records[0] = bytes(record)
        result = self.exporter.capture(self.fake.exchange)
        decoded = result["records"][0]["decoded"]
        self.assertEqual(decoded["motor_id"], 9)
        self.assertEqual(decoded["can_raw_hex"][:2], "19")
        self.assertIsNone(decoded["wire"])


if __name__ == "__main__":
    unittest.main()
