"""Prove why finite-resolution position alone cannot certify continuous standstill.

These are mathematical negative witnesses, not motor simulations or hardware
measurements. They must never grant a qualification bit or relax a speed limit.
"""
import math
import unittest


class StopObservabilityTests(unittest.TestCase):
    """Keep instantaneous-speed and window-average-speed claims explicitly separate."""

    def test_known_underestimate_requires_quantization_margin(self):
        """The preserved trace begins dwelling while truth is above the unchanged limit."""
        quantum = 25 / 65535
        estimate = 0.0238418560475111
        truth = 0.025129780932261158
        self.assertLess(estimate, 0.025)
        self.assertGreater(truth, 0.025)
        self.assertGreater(estimate + quantum / 0.032, 0.025)

    def test_hidden_motion_has_constant_position_codes(self):
        """Both quantizers can conceal repeated speed excursions during a 100 ms dwell."""
        quantum = 25 / 65535
        amplitude = 0.03 * 0.032 / (2 * math.pi)
        self.assertLess(amplitude, quantum / 2)
        for rounding, center in ((math.floor, 0.5), (round, 0.0)):
            codes = []
            speeds = []
            for tick in range(10001):
                time = tick * 0.00001
                codes.append(rounding(center + amplitude / quantum * math.sin(2 * math.pi * time / 0.032)))
                speeds.append(0.03 * math.cos(2 * math.pi * time / 0.032))
            self.assertEqual(len(set(codes)), 1)
            self.assertGreater(max(abs(value) for value in speeds), 0.025)
            self.assertLess(quantum / 0.032, 0.025)

    def test_reference_slew_is_not_physical_acceleration(self):
        """Reference acceleration cannot bound a plant responding to nominal torque and disturbance."""
        actual_acceleration = 20 * 0.1 + 0.1
        self.assertGreater(actual_acceleration, 0.5)


if __name__ == '__main__':
    unittest.main()
