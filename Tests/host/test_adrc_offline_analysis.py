"""Test offline measurement analysis without opening any device or granting qualification."""

import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

MODULE_PATH = Path(__file__).resolve().parents[2] / 'tools/analyze_adrc_quantization.py'
SPEC = importlib.util.spec_from_file_location('adrc_offline_analysis', MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analysis
SPEC.loader.exec_module(analysis)


class PositionEstimatorTests(unittest.TestCase):
    """Exercise causal timing and invalid-feedback boundaries independently of the plant."""

    def test_constant_velocity_uses_actual_timestamp_span(self):
        """A complete window estimates a known ramp in both directions."""
        for velocity in (-0.1, 0.1):
            estimator = analysis.PositionEstimator(0.032, 12.5)
            self.assertIsNone(estimator.update(0.0, 0.0))
            for timestamp in (0.007, 0.015, 0.023, 0.031):
                self.assertIsNone(estimator.update(timestamp, velocity * timestamp))
            self.assertAlmostEqual(estimator.update(0.033, velocity * 0.033), velocity)

    def test_duplicate_reverse_and_missing_timestamps_invalidate_window(self):
        """Repeated time, reversed time and a long dropout cannot yield a fresh estimate."""
        for timestamps in ((0.0, 0.0), (0.01, 0.009), (0.0, 0.02)):
            estimator = analysis.PositionEstimator(0.032, 12.5)
            estimator.update(timestamps[0], 0.0)
            self.assertIsNone(estimator.update(timestamps[1], 0.001))
            self.assertEqual(estimator.resets, 1)

    def test_position_jump_does_not_become_velocity_spike(self):
        """Encoder wrap, reset or implausible travel clears history rather than unwrapping blindly."""
        estimator = analysis.PositionEstimator(0.032, 12.5)
        estimator.update(0.0, 12.49)
        self.assertIsNone(estimator.update(0.004, -12.49))
        self.assertEqual(estimator.resets, 1)

    def test_nonfinite_inputs_are_rejected(self):
        """NaN/Inf and out-of-range positions never enter estimator history."""
        for timestamp, position in ((float('nan'), 0), (0, float('inf')), (0, 13)):
            with self.assertRaises(ValueError):
                analysis.PositionEstimator(0.032, 12.5).update(timestamp, position)

    def test_configuration_is_validated(self):
        """Nonpositive or nonfinite estimator parameters are rejected."""
        for window, limit in ((0, 12.5), (-1, 12.5), (0.032, 0), (float('nan'), 12.5)):
            with self.assertRaises(ValueError):
                analysis.PositionEstimator(window, limit)

    def test_freshness_boundary_and_recovery(self):
        """One missing 4 ms sample is allowed at 8 ms; a longer gap requires a full new window."""
        estimator = analysis.PositionEstimator(0.032, 12.5)
        for tick in range(5):
            value = estimator.update(tick * 0.008, tick * 0.0008)
        self.assertAlmostEqual(value, 0.1)
        self.assertEqual(estimator.resets, 0)
        self.assertIsNone(estimator.update(0.040001, 0.0040001))
        self.assertEqual(estimator.resets, 1)
        for tick in range(1, 9):
            value = estimator.update(0.040001 + tick * 0.004, 0.0040001 + tick * 0.0004)
        self.assertAlmostEqual(value, 0.1)


class MappingTests(unittest.TestCase):
    """Verify protocol grid assumptions and report truth boundaries."""

    def test_grid_has_no_zero(self):
        """Even-width bipolar mappings have half-LSB central values, unlike round(x/q)."""
        self.assertEqual(analysis.feedback_code(0, 200, 12, 'floor'), 2047)
        self.assertEqual(analysis.feedback_code(0, 200, 12, 'nearest'), 2048)
        self.assertEqual(analysis.feedback_code(-200, 200, 12, 'floor'), 0)
        self.assertEqual(analysis.feedback_code(200, 200, 12, 'nearest'), 4095)

    def test_unknown_rounding_and_out_of_range_are_rejected(self):
        """Unknown hardware conversion must not silently choose an arbitrary mapping."""
        for value, rounding in ((201, 'floor'), (float('nan'), 'floor'), (0, 'unknown')):
            with self.assertRaises(ValueError):
                analysis.feedback_code(value, 200, 12, rounding)


class EvidenceTests(unittest.TestCase):
    """Reject false or altered upstream evidence using memory fixtures with no file deletion."""

    def fixture(self):
        """Build a complete in-memory manifest with the same expected membership contract."""
        model_run = analysis.ROOT / 'output/adrc/models/memory_only_fixture'
        generated = model_run / 'generated/adrc_controller_ert_rtw'
        source_names = ('build_adrc_models.m', 'run_adrc_pipeline.m',
                        'adrc_simulation_parameters.m', 'adrc_generated_replay.c')
        sources = [analysis.ROOT / 'Models/Adrc' / name for name in source_names]
        artifacts = [model_run / (name + '.slx') for name in
                     ('adrc_controller', 'adrc_plant', 'adrc_validation')]
        artifacts += [generated / 'adrc_controller.c', generated / 'adrc_controller.h']
        entries = lambda paths: [{'path': str(path), 'sha256': 'a' * 64} for path in paths]
        report = {'passed': True, 'verificationHashesMatch': True,
                  'cReplay': {'passed': True}, 'closedLoopMil': {'passed': True},
                  'verificationSources': entries(sources), 'verificationArtifacts': entries(artifacts)}
        return model_run, report

    def check_fixture(self, model_run, report, changed=False):
        """Mock only file access; execute the production evidence validator unchanged."""
        def glob_entries(directory, pattern):
            """Expose exactly one generated C source and one generated header."""
            return iter([directory / ('adrc_controller.c' if pattern == '*.c' else 'adrc_controller.h')])

        def fixture_digest(path):
            """Simulate a changed generated source without altering any real source file."""
            return 'b' * 64 if changed and Path(path).name == 'adrc_controller.c' else 'a' * 64

        with patch.object(Path, 'read_text', return_value=json.dumps(report)), \
                patch.object(Path, 'glob', glob_entries), patch.object(analysis, 'digest', fixture_digest):
            return analysis.verify_model_run(model_run)

    def test_complete_manifest_is_accepted(self):
        """A complete passing manifest establishes the positive control for rejection tests."""
        model_run, report = self.fixture()
        self.check_fixture(model_run, report)

    def test_changed_generated_source_is_rejected(self):
        """A valid old report cannot certify modified generated bytes."""
        model_run, report = self.fixture()
        with self.assertRaisesRegex(ValueError, 'Changed upstream evidence'):
            self.check_fixture(model_run, report, changed=True)

    def test_missing_manifest_member_is_rejected(self):
        """An incomplete manifest cannot pass by hashing fewer files."""
        model_run, report = self.fixture()
        report['verificationArtifacts'].pop()
        with self.assertRaisesRegex(ValueError, 'manifest membership'):
            self.check_fixture(model_run, report)

    def test_failed_upstream_report_is_rejected(self):
        """Failure status remains failure even if all listed digests match."""
        model_run, report = self.fixture()
        report['passed'] = False
        with self.assertRaisesRegex(ValueError, 'did not pass'):
            self.check_fixture(model_run, report)


if __name__ == '__main__':
    unittest.main()
