"""@file Offline deployment permission, immutable-artifact and non-retry checks; no hardware access."""
import unittest
from unittest.mock import patch
import deploy_usability_torque as release


class UsabilityTorqueDeploymentTests(unittest.TestCase):
    """Keep artifact validation separate from authority to reset the supported arm."""

    def test_missing_power_confirmation(self):
        """Reject physical deployment before preparing or accessing hardware."""
        with patch.object(release, 'prepare') as prepare:
            with self.assertRaises(SystemExit):
                release.main(['--deploy'])
            prepare.assert_not_called()

    def test_default_is_offline(self):
        """Ordinary invocation must never call the hardware transaction."""
        with patch.object(release, 'prepare', return_value=object()), patch.object(release.guarded, 'deploy') as deploy:
            release.main([])
            deploy.assert_not_called()

    def test_failure_is_not_retried(self):
        """A single synthetic deployment failure must propagate without a second attempt."""
        with patch.object(release, 'prepare', return_value=object()), patch.object(release.guarded, 'deploy', side_effect=RuntimeError('synthetic')) as deploy:
            with self.assertRaises(RuntimeError):
                release.main(['--deploy', '--motor-power-off-confirmed'])
            deploy.assert_called_once()

    def test_frozen_artifact_and_baseline(self):
        """Validate the actual frozen artifacts offline, including exact HEX/ELF agreement."""
        prepared = release.prepare()
        self.assertEqual(release.guarded.shared.digest(prepared.old_payload), release.EXPECTED_OLD)
        self.assertEqual(release.guarded.shared.digest(prepared.image_payload), release.EXPECTED_NEW)
        self.assertGreater(sum(end-start for start, end in prepared.image.segments()), 0)


if __name__ == '__main__':
    unittest.main()
