"""@file Offline deployment gates for the challenge-bound UI session repair; no serial port or probe is opened."""
import unittest
from unittest.mock import patch
import deploy_session_quota_recovery as release


class SessionQuotaDeploymentTests(unittest.TestCase):
    """Separate exact release validation from permission to reset the supported loaded arm."""

    def test_missing_site_state_never_prepares(self):
        """Fail before preparation when this deployment lacks current power-off confirmation."""
        with patch.object(release,'prepare') as prepare:
            with self.assertRaises(SystemExit):release.main(['--deploy'])
            prepare.assert_not_called()

    def test_default_never_deploys(self):
        """Ordinary invocation cannot enter the hardware transaction."""
        with patch.object(release,'prepare',return_value=object()),patch.object(release.guarded,'deploy') as deploy:
            release.main([]);deploy.assert_not_called()

    def test_failed_deployment_never_retries(self):
        """Propagate a single hardware-path failure without a second attempt."""
        with patch.object(release,'prepare',return_value=object()),patch.object(release.guarded,'deploy',side_effect=RuntimeError('synthetic')) as deploy:
            with self.assertRaises(RuntimeError):release.main(['--deploy','--motor-power-off-confirmed'])
            deploy.assert_called_once()

    def test_frozen_release_and_baseline(self):
        """Require all source/product hashes and complete old/new occupied image bytes."""
        prepared=release.prepare()
        self.assertEqual(release.guarded.shared.digest(prepared.old_payload),release.EXPECTED_OLD)
        self.assertEqual(release.guarded.shared.digest(prepared.image_payload),release.EXPECTED_NEW)
        self.assertEqual(sum(end-start for start,end in prepared.image.segments()),release.VERIFIED_BYTES)


if __name__=='__main__':
    unittest.main()
