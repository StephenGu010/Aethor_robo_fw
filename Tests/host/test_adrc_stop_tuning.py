"""Check the offline tuning contract without granting hardware qualification or opening devices."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import adrc_interface_replay as replay
import tune_adrc_stop as tuning


class OfflineStopTuningTests(unittest.TestCase):
    """Keep the original matrix and reject ambiguous simulation inputs before touching C state."""

    def test_original_matrix_is_preserved(self):
        """Both modes, all four signed references and all uncertainty combinations remain present."""
        scenarios = tuning.original_scenarios()
        self.assertEqual(len(scenarios), 144)
        self.assertEqual(len(set(scenarios)), 144)
        self.assertEqual({item[1] for item in scenarios}, {-0.1, -0.05, 0.05, 0.1})
        self.assertEqual(sum(item[0] == 2 for item in scenarios), 72)

    def test_bad_bandwidth_is_rejected_before_interface_access(self):
        """Nonfinite, reversed and out-of-range controller gains never enter the host bridge."""
        for gains in ((0, 20), (20, 10), (4, 251), (float('nan'), 20), (4, float('inf'))):
            with self.subTest(gains=gains), self.assertRaises(ValueError):
                replay.run_scenario(None, None, None, (2, 0.1, 1, 0, 'floor'), tuning=gains)

    def test_bad_model_is_rejected_before_interface_access(self):
        """Invalid timing or dynamics cannot quietly alter a scored experiment."""
        for model in ({'unknown': 1}, {'damping': 0}, {'damping': float('nan')},
                      {'torque_delay_ticks': -1}, {'torque_delay_ticks': 0.5},
                      {'disable_ack_delay_ticks': True}, {'stop_tick': 17},
                      {'stop_tick': 317}, {'disturbance_start': 1},
                      {'hold_velocity_on_stop': 1}):
            with self.subTest(model=model), self.assertRaises(ValueError):
                replay.run_scenario(None, None, None, (2, 0.1, 1, 0, 'floor'), model=model)

    def test_frozen_parameters_are_explicit_and_mode_specific(self):
        """Candidate gains remain distinct from the unchanged legacy experiment defaults."""
        self.assertEqual(set(replay.OFFLINE_TUNING), {1, 2})
        self.assertNotEqual(replay.OFFLINE_TUNING[1], replay.OFFLINE_TUNING[2])
        for control, observer in replay.OFFLINE_TUNING.values():
            self.assertGreater(control, 0)
            self.assertLessEqual(control, observer)
            self.assertLessEqual(observer, 250)


if __name__ == '__main__':
    unittest.main()
