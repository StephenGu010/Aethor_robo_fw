"""Exercise disabled-capture evidence rejection without any hardware connection."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Tools'))
from verify_adrc_disabled_capture import parse_capture


def fixture():
    """Build explicit synthetic transcript text solely for offline parser tests."""
    lines = ['> 1 adrc hardware', '< ok 1 adrc hardware motor=7 mode=1 fields=1fff',
             '> 2 adrc limits', '< ok 2 adrc limits motor=7 pmax=12.5 vmax=200 tmax=10',
             '> 3 adrc sample', '< ok 3 adrc sample state=1 capacity=64',
             '> 4 adrc capture', '< ok 4 adrc capture id=3 motor=7 state=2 reason=0 count=64 requests=65 start_us=1000 first_request_us=4000']
    for index in range(64):
        observed = 12000 + index * 4000
        lines.extend([f'> {index + 5} adrc capture index={index}',
                      f'< ok {index + 5} adrc capture id=3 index={index} observed_us={observed} rx_lower_us={observed - 4000} pos=0 vel=0.0488 torque=-0.0024'])
    lines.extend(['> 69 adrc evidence', '< ok 69 adrc evidence verified=0',
                  'HANDOFF_WINDOW_COMPLETE',
                  'mode_restored=True final_disable_completed=True acquire_transferred=True release_verified=True'])
    return '\n'.join(lines)


class DisabledCaptureTests(unittest.TestCase):
    """Reject corrupted provenance and timing rather than promoting it to qualification."""

    def test_complete_window_is_not_motion_qualification(self):
        """A valid diagnostic retains explicit missing enabled/watchdog/identification proofs."""
        report = parse_capture(fixture())
        self.assertEqual(report['sample_count'], 64)
        self.assertEqual(report['maximum_rx_interval_us'], 4000)
        self.assertEqual(report['qualification_flags'], 0)
        for key in ('hardware_qualified', 'fit_ready', 'watchdog_action_verified', 'enabled_feedback_verified'):
            self.assertFalse(report[key])

    def test_negative_cases(self):
        """Exercise identity mismatch, stale rows, bad values, late startup and unsafe completion."""
        original = fixture()
        changes = [
            ('capture id=3 index=0', 'capture id=2 index=0'),
            ('observed_us=12000 rx_lower_us=8000', 'observed_us=22000 rx_lower_us=18000'),
            ('observed_us=16000 rx_lower_us=12000', 'observed_us=16000 rx_lower_us=8000'),
            ('observed_us=16000 rx_lower_us=12000', 'observed_us=16000 rx_lower_us=20000'),
            ('pos=0', 'pos=nan'), ('vel=0.0488', 'vel=0.05001'),
            ('requests=65', 'requests=69'), ('verified=0', 'verified=63'),
            ('mode=1', 'mode=2'), ('count=64', 'count=63'),
            ('state=2 reason=0', 'state=3 reason=1'),
            ('release_verified=True', 'release_verified=False'),
            ('> 3 adrc sample', '> 3 adrc run'),
            ('> 4 adrc capture', '> 3 adrc capture'),
            ('> 5 adrc capture index=0', '> 5 bench move 7 1'),
        ]
        for old, new in changes:
            with self.subTest(change=(old, new)), self.assertRaises(ValueError):
                parse_capture(original.replace(old, new, 1))
        with self.assertRaises(ValueError):
            parse_capture(original + '\nhost_error=injected')


if __name__ == '__main__':
    unittest.main()
