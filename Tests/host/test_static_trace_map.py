"""Offline negative tests for the actual linked startup trace memory guard."""

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPECIFICATION = importlib.util.spec_from_file_location("map_guard", ROOT / "Tests/host/check_debug_ui_map.py")
MAP = importlib.util.module_from_spec(SPECIFICATION)
SPECIFICATION.loader.exec_module(MAP)


class TraceMapTests(unittest.TestCase):
    """Reject DTCM placement, oversize storage and unexpected feature-off allocation."""

    def test_guard_exists(self):
        """Retain a specific missing-memory-guard RED before implementation."""
        self.assertTrue(hasattr(MAP, "verify_static_trace_memory"), "Missing linked trace placement guard")

    def test_trace_is_bounded_and_lands_in_axi(self):
        """Use linked addresses, rather than a source section declaration, as the evidence."""
        if not hasattr(MAP, "verify_static_trace_memory"):
            self.skipTest("Availability assertion supplies RED")
        valid = "    application_static_trace 0x24011000 Data 12000 application.o\n"
        result = MAP.verify_static_trace_memory(valid, "LCD-SingleStatic-Test")
        self.assertEqual(result["bytes"], 12000)
        for contents in (valid.replace("0x24011000", "0x20001000"),
                valid.replace("12000", "15000"), valid.replace("0x24011000", "0x2404f000"), ""):
            with self.subTest(contents=contents), self.assertRaises(AssertionError):
                MAP.verify_static_trace_memory(contents, "LCD-SingleStatic-Test")

    def test_feature_off_targets_do_not_allocate_trace_storage(self):
        """Shared default-off builds must not consume static trace RAM."""
        if not hasattr(MAP, "verify_static_trace_memory"):
            self.skipTest("Availability assertion supplies RED")
        for target in ("LCD-MIT-ADRC", "ADRC-Bench"):
            self.assertTrue(MAP.verify_static_trace_memory("", target)["absent"])
            with self.assertRaises(AssertionError):
                MAP.verify_static_trace_memory(" application_static_trace 0x24011000 Data 12000 application.o\n", target)


if __name__ == "__main__":
    unittest.main()
