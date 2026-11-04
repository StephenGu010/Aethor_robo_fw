"""@file Offline physical-state, immutable artifact and no-retry deployment gates; no device is opened."""
import unittest
from unittest.mock import patch
import deploy_session_recovery as release


class SessionRecoveryDeploymentTests(unittest.TestCase):
    """Separate a verified binary from authorization to reset a powered loaded arm."""

    def test_missing_confirmation_never_prepares(self):
        """No current physical state means no preparation and no hardware path."""
        with patch.object(release,'prepare') as prepare:
            with self.assertRaises(SystemExit):release.main(['--deploy'])
            prepare.assert_not_called()

    def test_default_is_offline(self):
        """Ordinary invocation verifies immutable files without entering deployment."""
        with patch.object(release,'prepare',return_value=object()),patch.object(release.guarded,'deploy') as deploy:
            release.main([]);deploy.assert_not_called()

    def test_hardware_failure_is_never_retried(self):
        """A failing single deployment propagates; no automatic second attempt is permitted."""
        with patch.object(release,'prepare',return_value=object()),patch.object(release.guarded,'deploy',side_effect=RuntimeError('synthetic')) as deploy:
            with self.assertRaises(RuntimeError):release.main(['--deploy','--motor-power-off-confirmed'])
            deploy.assert_called_once()

    def test_exact_baseline_and_new_image(self):
        """Every frozen file and occupied byte must match the reviewed release."""
        prepared=release.prepare()
        self.assertEqual(release.guarded.shared.digest(prepared.old_payload),release.EXPECTED_OLD)
        self.assertEqual(release.guarded.shared.digest(prepared.image_payload),release.EXPECTED_NEW)
        self.assertEqual(sum(end-start for start,end in prepared.image.segments()),release.VERIFIED_BYTES)


if __name__=='__main__':
    unittest.main()
