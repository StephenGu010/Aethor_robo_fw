"""Exercise the independent POS image builder through retained fake-process fixtures."""
from pathlib import Path
import importlib.util
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("pos_builder_reused_cases", ROOT / "Tests/host/test_single_static_build.py")
LEGACY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = LEGACY
SPEC.loader.exec_module(LEGACY)
LEGACY.BUILDER = ROOT / "tools/build_pos_load_test.ps1"
LEGACY.TARGET = "LCD-POS-Load-Test"
LEGACY.HARNESS = LEGACY.HARNESS.replace("LCD-SingleStatic-Test", LEGACY.TARGET)


class PosLoadBuildTests(LEGACY.SingleStaticBuildTests):
    """Run the production fresh-receipt and containment matrix against the new builder."""

    def test_new_target_retains_strict_static_trace_memory_guard(self):
        """The inherited startup trace must land in bounded AXI RAM for the new target too."""
        specification = importlib.util.spec_from_file_location('pos_map_guard', ROOT / 'Tests/host/check_debug_ui_map.py')
        guard = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(guard)
        self.assertIn(LEGACY.TARGET, guard.SELECTABLE_TARGETS)
        valid = ' application_static_trace 0x24011000 Data 12000 application.o\n'
        self.assertEqual(guard.verify_static_trace_memory(valid,LEGACY.TARGET)['bytes'],12000)
        for invalid in (valid.replace('0x24011000','0x20001000'),valid.replace('12000','15000'),''):
            with self.subTest(invalid=invalid), self.assertRaises(AssertionError):
                guard.verify_static_trace_memory(invalid,LEGACY.TARGET)

    def test_default_evidence_location_is_unique_and_software_only(self):
        """Verify the POS builder keeps its separate dated evidence location."""
        script = LEGACY.BUILDER.read_text(encoding="utf-8-sig")
        self.assertIn("output\\pos_load_auto_20261020\\build_", script)


if __name__ == "__main__":
    unittest.main(verbosity=2)
