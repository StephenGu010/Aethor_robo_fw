"""Fake-only power-off deployment tests; no serial, pyOCD imports, or real probe.

Fixtures stay under output/adrc/poweroff_host_tests; no bulk deletion is used.
"""

import importlib.util
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

REPOSITORY = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('poweroff_deploy', REPOSITORY / 'tools/deploy_adrc_poweroff.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeImage:
    """Represent bounded image bytes using JSON instead of importing IntelHex."""

    def __init__(self, stream):
        """Read a fixture image from an immutable text stream."""
        self.payload = bytes(json.load(stream))

    def segments(self):
        """Return the fixture's contiguous flash segment."""
        return [(MODULE.FLASH_START, MODULE.FLASH_START + len(self.payload))]

    def addresses(self):
        """Enumerate only occupied bytes."""
        return range(MODULE.FLASH_START, MODULE.FLASH_START + len(self.payload))

    def __getitem__(self, address):
        """Return the requested occupied byte."""
        return self.payload[address - MODULE.FLASH_START]

    def tobinarray(self, start, size):
        """Return exact fixture bytes without filling gaps."""
        return self.payload[start-MODULE.FLASH_START:start-MODULE.FLASH_START+size]


class FakeTarget:
    """Store simulated 1 MiB flash and record every state-changing operation."""

    def __init__(self, old_image):
        """Initialize the fake MCU running the expected image."""
        self.state = 'running'
        self.flash = bytearray(b'\xff' * 0x100000)
        self.flash[:len(old_image.payload)] = old_image.payload
        self.events = []
        self.identity = 0x483

    def get_state(self):
        """Expose the current fake core state."""
        return self.state

    def read32(self, address):
        """Expose only the H723 identity register."""
        assert address == 0x5C001000
        return self.identity

    def halt(self):
        """Record a halt and stop the fake core."""
        self.events.append('halt')
        self.state = 'halted'

    def reset(self, reset_type):
        """Record the sole permitted successful normal reset."""
        assert reset_type == 'sysresetreq'
        self.events.append('reset')
        self.state = 'running'


class FakeSession:
    """Simulate an attach-only session which never resumes on exit."""

    def __init__(self, target):
        """Bind the known probe and fake target."""
        self.target = target
        self.probe = SimpleNamespace(unique_id=MODULE.PROBE_UID)

    def __enter__(self):
        """Return the already-running fake target session."""
        return self

    def __exit__(self, *arguments):
        """Record disconnect without modifying target execution state."""
        self.target.events.append('disconnect')


class PoweroffDeployTests(unittest.TestCase):
    """Exercise snapshot gates, bounded paths, and success/failure sequencing."""

    def setUp(self):
        """Create isolated persistent fixtures and fake dependencies only."""
        parent = REPOSITORY / 'output/adrc/poweroff_host_tests'
        parent.mkdir(exist_ok=True, parents=True)
        self.root = Path(tempfile.mkdtemp(prefix='fixture_', dir=parent))
        (self.root / 'output/adrc/hardware').mkdir(parents=True)
        self.old_path = self.root / 'old.hex'
        self.old_payload = json.dumps([0, 0, 0, 36, 9, 0, 0, 8]).encode()
        self.old_path.write_bytes(self.old_payload)
        self.new_path = self.root / 'LCD-MIT-ADRC.hex'
        self.new_payload = json.dumps([0, 0, 0, 36, 17, 0, 0, 8, 1, 2, 3, 4]).encode()
        self.new_path.write_bytes(self.new_payload)
        self.elf_path = self.root / 'LCD-MIT-ADRC.axf'
        self.elf_path.write_bytes(b'fake verified elf')
        self.manifest_path = self.root / 'manifest.json'
        self.manifest_path.write_text(json.dumps({'source_hashes': [], 'evidence_hashes': [
            {'path': str(path), 'sha256': MODULE.digest(path.read_bytes())}
            for path in (self.new_path, self.elf_path)]}), encoding='utf-8')
        self.root_patch = patch.multiple(MODULE, ROOT=self.root, OLD_IMAGE=self.old_path,
                                        OLD_SHA256=MODULE.digest(self.old_payload))
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.target = FakeTarget(FakeImage(io.StringIO(self.old_payload.decode())))
        self.scenario = 'success'
        self.attach_count = 0
        self.dependencies = SimpleNamespace(IntelHex=FakeImage, verify_build=self.verify_build,
            ConnectHelper=SimpleNamespace(session_with_chosen_probe=self.connect),
            Target=SimpleNamespace(State=SimpleNamespace(RUNNING='running', HALTED='halted'),
                                   ResetType=SimpleNamespace(SYSRESETREQ='sysresetreq')),
            read_flash=self.read_flash, reset_catch_verified=self.reset_catch,
            FileProgrammer=self.programmer)

    def verify_build(self, manifest):
        """Return an independently parsed fake build snapshot."""
        self.assertEqual(manifest, self.manifest_path)
        image = FakeImage(io.StringIO(self.new_payload.decode()))
        if self.scenario == 'changed_new':
            self.new_path.write_bytes(b'changed')
        return self.new_path, self.elf_path, image

    def connect(self, blocking, unique_id, options):
        """Validate exact probe and attach options before returning fake SWD."""
        self.attach_count += 1
        self.assertFalse(blocking)
        self.assertEqual(unique_id, MODULE.PROBE_UID)
        self.assertEqual(options['target_override'], 'stm32h723xx')
        self.assertEqual(options['connect_mode'], 'attach')
        self.assertFalse(options['auto_unlock'])
        self.assertFalse(options['resume_on_disconnect'])
        session = FakeSession(self.target)
        if self.scenario == 'wrong_probe':
            session.probe.unique_id = 'unknown'
        return session

    def read_flash(self, target, start, end):
        """Read fake bytes and record the execution state during the audit."""
        self.assertGreaterEqual(start, MODULE.FLASH_START)
        self.assertLessEqual(end, MODULE.FLASH_END)
        target.events.append(('read', target.state, end - start))
        return bytes(target.flash[start-MODULE.FLASH_START:end-MODULE.FLASH_START])

    def reset_catch(self, target, vector):
        """Record reset catch only after a halted full backup."""
        self.assertEqual(target.state, 'halted')
        self.assertEqual(vector, 0x08000009)
        target.events.append('reset_catch')
        return {'state': 'halted', 'pc': hex(vector & ~1)}

    def programmer(self, session, **options):
        """Return an in-memory programmer which accepts only the copied snapshot."""
        self.assertFalse(options['smart_flash'])
        self.assertFalse(options['trust_crc'])
        return SimpleNamespace(program=self.program)

    def program(self, stream, file_format):
        """Simulate programming, RAM execution failure, or readback corruption."""
        self.target.events.append('program')
        self.assertEqual(file_format, 'hex')
        self.assertEqual(stream.read().encode(), self.new_payload)
        if self.scenario == 'program_throw':
            self.target.state = 'running'
            raise RuntimeError('fake program failure')
        image = FakeImage(io.StringIO(self.new_payload.decode()))
        self.target.flash[:len(image.payload)] = image.payload
        if self.scenario == 'mismatch':
            self.target.flash[0] ^= 1

    def run_deploy(self):
        """Prepare and deploy using exclusively fake dependencies."""
        prepared = MODULE.prepare(self.manifest_path, self.old_path, self.dependencies)
        with redirect_stdout(io.StringIO()):
            return MODULE.deploy(prepared, self.dependencies, True)

    def test_missing_confirmation(self):
        """Reject missing confirmation before preparation, imports, or device access."""
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MODULE.main(['--manifest', str(self.manifest_path), '--expected-current-image',
                         str(self.old_path), '--deploy'], self.dependencies)
        self.assertEqual(self.attach_count, 0)

    def test_verify_only(self):
        """Default CLI verifies immutable artifacts without accessing SWD."""
        with redirect_stdout(io.StringIO()):
            MODULE.main(['--manifest', str(self.manifest_path), '--expected-current-image',
                         str(self.old_path)], self.dependencies)
        self.assertEqual(self.attach_count, 0)

    def test_success(self):
        """Require running audit, halt, backup, reset catch, program, readback, reset order."""
        report = self.run_deploy()
        self.assertEqual(report['motor_power'], 'off=user_confirmation')
        self.assertFalse(report['motor_feedback_verified'])
        self.assertTrue(report['reset_completed'])
        self.assertEqual(report['new_verified_bytes'], 12)
        self.assertEqual(Path(report['backup']).stat().st_size, 0x100000)
        self.assertEqual(self.target.events[0], ('read', 'running', 8))
        self.assertLess(self.target.events.index('program'), self.target.events.index('reset'))

    def test_bad_current_flash(self):
        """A mismatch in the current occupied bytes halts and prevents all programming."""
        self.target.flash[7] ^= 1
        with self.assertRaisesRegex(RuntimeError, 'Running image'):
            self.run_deploy()
        self.assertNotIn('program', self.target.events)
        self.assertNotIn('reset_catch', self.target.events)
        self.assertNotIn('reset', self.target.events)
        self.assertEqual(self.target.state, 'halted')

    def test_program_throw(self):
        """Stop a failed RAM flash algorithm without calling normal reset."""
        self.scenario = 'program_throw'
        with self.assertRaisesRegex(RuntimeError, 'fake program failure'):
            self.run_deploy()
        self.assertEqual(self.target.state, 'halted')
        self.assertNotIn('reset', self.target.events)
        report_path = next((self.root / 'output/adrc/hardware').glob('*/report.json'))
        report = json.loads(report_path.read_text())
        self.assertEqual(report['error'], 'fake program failure')
        self.assertFalse(report['normal_reset_requested'])
        self.assertFalse(report['flash_readback_equal'])
        self.assertTrue(Path(report['backup']).is_file())

    def test_readback_mismatch(self):
        """Corrupt readback stays halted and never boots the new image."""
        self.scenario = 'mismatch'
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            self.run_deploy()
        self.assertNotIn('reset', self.target.events)
        self.assertEqual(self.target.state, 'halted')

    def test_bad_old_artifact(self):
        """Reject changed expected-current-image bytes before device access."""
        self.old_path.write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError, 'current image hash'):
            self.run_deploy()
        self.assertEqual(self.attach_count, 0)

    def test_changed_new_artifact(self):
        """Reject a new image changed between build verification and snapshot."""
        self.scenario = 'changed_new'
        with self.assertRaisesRegex(RuntimeError, 'changed after build'):
            self.run_deploy()
        self.assertEqual(self.attach_count, 0)

    def test_bounded_paths(self):
        """Reject an existing path outside the explicit workspace fixture root."""
        with self.assertRaisesRegex(RuntimeError, 'outside workspace'):
            MODULE.bounded_path(Path(__file__).resolve())
        with self.assertRaisesRegex(RuntimeError, 'reviewed image path'):
            MODULE.prepare(self.manifest_path, self.new_path, self.dependencies)
        self.assertEqual(self.attach_count, 0)

    def test_wrong_identity(self):
        """Reject non-H723 targets and leave them halted without reset."""
        self.target.identity = 0x999
        with self.assertRaisesRegex(RuntimeError, 'state/identity'):
            self.run_deploy()
        self.assertEqual(self.target.state, 'halted')
        self.assertNotIn('reset', self.target.events)

    def test_wrong_probe(self):
        """Check the actual attached probe UID as well as the selection argument."""
        self.scenario = 'wrong_probe'
        with self.assertRaisesRegex(RuntimeError, 'probe UID'):
            self.run_deploy()
        self.assertNotIn('program', self.target.events)
        self.assertNotIn('reset', self.target.events)

    def test_snapshot_prevents_path_race(self):
        """Changing original HEX after preparation cannot change programmed bytes."""
        prepared = MODULE.prepare(self.manifest_path, self.old_path, self.dependencies)
        self.new_path.write_bytes(b'changed original after snapshot')
        self.old_path.write_bytes(b'changed old original after snapshot')
        with redirect_stdout(io.StringIO()):
            report = MODULE.deploy(prepared, self.dependencies, True)
        self.assertEqual(report['new_hex_sha256'], MODULE.digest(self.new_payload))
        self.assertTrue(report['flash_readback_equal'])


if __name__ == '__main__':
    unittest.main()
