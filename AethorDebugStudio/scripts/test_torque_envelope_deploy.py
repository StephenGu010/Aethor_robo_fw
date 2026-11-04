"""@file Offline deployment gates and exact artifact verification; no serial port or probe is opened."""
import unittest
from unittest.mock import patch
import deploy_torque_envelope as release


class TorqueDeploymentTests(unittest.TestCase):
    """Keep physical confirmation ahead of every hardware path and forbid automatic retry."""

    def test_deploy_without_confirmation_never_prepares(self):
        """An incomplete physical declaration must fail before dependency or device preparation."""
        with patch.object(release, 'prepare') as prepare:
            with self.assertRaises(SystemExit):
                release.main(['--deploy'])
            prepare.assert_not_called()

    def test_default_never_calls_hardware_deploy(self):
        """Default invocation verifies only immutable files."""
        with patch.object(release, 'prepare', return_value=object()), patch.object(release.guarded, 'deploy') as deploy:
            release.main([])
            deploy.assert_not_called()

    def test_failure_is_not_retried(self):
        """A single failed deployment is propagated without a second hardware invocation."""
        with patch.object(release, 'prepare', return_value=object()), patch.object(release.guarded, 'deploy', side_effect=RuntimeError('synthetic failure')) as deploy:
            with self.assertRaises(RuntimeError):
                release.main(['--deploy', '--motor-power-off-confirmed'])
            deploy.assert_called_once()

    def test_all_frozen_files_and_image_bytes_match(self):
        """Pin the intended running baseline and every occupied byte of the candidate image."""
        prepared = release.prepare()
        self.assertEqual(release.guarded.shared.digest(prepared.image_payload), release.EXPECTED_NEW)
        self.assertEqual(release.guarded.shared.digest(prepared.old_payload), release.EXPECTED_OLD)
        self.assertEqual(sum(end-start for start,end in prepared.image.segments()), release.VERIFIED_BYTES)


if __name__ == '__main__':
    unittest.main()
