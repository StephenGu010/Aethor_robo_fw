"""Offline release input tampering checks; no hardware imports or recursive cleanup."""
import importlib.util
from pathlib import Path
import sys
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))


class ReleaseInputsTest(unittest.TestCase):
    """Reject changed or escaping source snapshots before the deployment gate."""

    def test_manifest_entries_are_rechecked(self):
        """A hash match must pass; a subsequent file mutation must fail closed."""
        path = ROOT / 'tools/deploy_flange_zero_poweroff.py'
        self.assertTrue(path.is_file(), 'flange zero release adapter is missing')
        specification = importlib.util.spec_from_file_location('flange_release_under_test', path)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        folder = ROOT / 'output/release-tests' / uuid.uuid4().hex
        folder.mkdir(parents=True)
        source = folder / 'input.c'
        source.write_text('source version one', encoding='utf-8')
        entry = module.file_entry(source)
        module.verify_entries([entry])
        source.write_text('changed after freeze', encoding='utf-8')
        with self.assertRaises(RuntimeError):
            module.verify_entries([entry])
        with self.assertRaises(RuntimeError):
            module.verify_entries([{'path': '../outside.c', 'sha256': '0'*64}])


if __name__ == '__main__':
    unittest.main()
