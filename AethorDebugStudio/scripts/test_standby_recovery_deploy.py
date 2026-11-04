"""@file Verify the standby release adapter without serial or SWD access."""
import unittest
from unittest.mock import patch

import deploy_standby_recovery as release


class DeploymentGateTests(unittest.TestCase):
    """Ensure physical confirmation precedes preparation or hardware calls."""

    def test_missing_power_confirmation_never_prepares(self):
        """Reject a deployment flag before loading any hardware dependency."""
        with patch.object(release, 'prepare') as prepare:
            with self.assertRaises(SystemExit):
                release.main(['--deploy'])
            prepare.assert_not_called()

    def test_offline_does_not_deploy(self):
        """Default verification must not open serial or SWD."""
        with patch.object(release, 'prepare', return_value=object()), patch.object(release.guarded, 'deploy') as deploy:
            release.main([])
            deploy.assert_not_called()

    def test_frozen_release_and_previous_image(self):
        """Rehash every frozen input and compare all occupied HEX/ELF bytes."""
        prepared = release.prepare()
        self.assertEqual(release.guarded.shared.digest(prepared.image_payload), release.EXPECTED_NEW)
        self.assertEqual(release.guarded.shared.digest(prepared.old_payload), release.EXPECTED_OLD)
        self.assertEqual(sum(end - start for start, end in prepared.image.segments()), 517764)


if __name__ == '__main__':
    unittest.main()
