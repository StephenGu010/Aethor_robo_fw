"""Exercise the reused deployment transaction with guarded fake sessions; no hardware access."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import deploy_idle_feedback_poweroff as adapter
import test_adrc_poweroff_deploy as fixture


class IdentityGuardTests(unittest.TestCase):
    """Prove both unpaired refusal and paired failure-halt using the real transaction control flow."""

    def setUp(self):
        """Reuse only fake memory and programmer objects from the established offline suite."""
        self.fixture = fixture.PoweroffDeployTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.pairing = {}
        original_connect = self.fixture.dependencies.ConnectHelper.session_with_chosen_probe
        original_read = self.fixture.dependencies.read_flash

        def connect(**kwargs):
            """Wrap the fake connection exactly as the deployment adapter wraps real pyOCD."""
            return adapter.PairedSession(original_connect(**kwargs), self.pairing)

        def read(target, start, end):
            """Stand in for a successful identity observation before any flash read."""
            self.pairing['usb_swd_pair_verified'] = True
            return original_read(target, start, end)

        self.fixture.dependencies.ConnectHelper.session_with_chosen_probe = connect
        self.fixture.dependencies.read_flash = read

    def test_wrong_probe_never_halts(self):
        """The helper's exception cleanup must not halt an unverified probe target."""
        self.fixture.scenario = 'wrong_probe'
        with self.assertRaises(RuntimeError):
            self.fixture.run_deploy()
        self.assertNotIn('halt', self.fixture.target.events)
        self.assertEqual(self.fixture.target.state, 'running')

    def test_partial_pairing_never_halts(self):
        """A populated but failed UID receipt is not authorization to change target state."""
        def failed_read(target, start, end):
            """Model the actual identity verifier storing words before rejecting their value."""
            self.pairing['mcu_uid_words'] = [1, 2, 3]
            raise RuntimeError('USB/SWD mismatch')
        self.fixture.dependencies.read_flash = failed_read
        with self.assertRaises(RuntimeError):
            self.fixture.run_deploy()
        self.assertNotIn('halt', self.fixture.target.events)

    def test_paired_program_failure_stays_halted(self):
        """Identity guarding must preserve the existing fail-halt policy after programming starts."""
        self.fixture.scenario = 'program_throw'
        with self.assertRaises(RuntimeError):
            self.fixture.run_deploy()
        self.assertIn('program', self.fixture.target.events)
        self.assertEqual(self.fixture.target.state, 'halted')
        self.assertNotIn('reset', self.fixture.target.events)

    def test_paired_success_verifies_and_resets(self):
        """A verified target still completes backup, readback and exactly one normal reset."""
        report = self.fixture.run_deploy()
        self.assertTrue(report['flash_readback_equal'])
        self.assertTrue(report['reset_completed'])
        self.assertEqual(self.fixture.target.events.count('reset'), 1)


if __name__ == '__main__':
    unittest.main()
