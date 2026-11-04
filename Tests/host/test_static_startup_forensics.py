"""@file Negative evidence tests for offline static startup analysis; no hardware."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('startup_forensics', ROOT / 'tools/analyze_static_startup_root.py')
FORENSICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FORENSICS)
EVIDENCE = ROOT.parent / 'seven-axis-static-load/output'
SOURCES = {
    'success': ('static_startup_sync_20261020/lcd_pilot_trace_20261020T0826Z.json',
                'a4df4d2d223f457ef79931a5ea9757586b36bb77d38810bdbcfff22308431c37'),
    'negative': ('static_startup_sync_20261020/failure_trace_20261020T084016455Z.json',
                 '8f84c6fa2491f60155c8a76429653221ea41fbbdf97642a7ccb13d7bbae1edf4'),
    'positive': ('single_static_startup_diagnosis_20261022/latest_failure_trace.json',
                 '4d31dbe3bd7fbc67cdfefeba10778f344c869736f22f73c76b69dde87b2a2c12'),
    'near_boundary': ('pos_static_postflash_diagnosis_20261022/latest_failure_trace.json',
                      '09ad8962211441631bdd153d179e2fd8c50ede44a4f08a0a3512acc88e84ba4f'),
}


class StartupEvidenceTests(unittest.TestCase):
    """Check genuine captures and ensure damaged or insufficient evidence fails closed."""

    def setUp(self):
        """Load fixed historical artifacts without modifying the source evidence."""
        self.traces = {}
        for name, (relative, expected_hash) in SOURCES.items():
            payload = (EVIDENCE / relative).read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(), expected_hash)
            self.traces[name] = json.loads(payload.decode('utf-8-sig'))

    def test_signed_failures_have_contradiction_before_protection(self):
        """All three failures contradict the synchronous same-coordinate PD hypothesis."""
        for name in ('negative', 'positive', 'near_boundary'):
            result = FORENSICS.analyze(self.traces[name])
            self.assertEqual(result['actual_tx_ms'], [8, 12, 16])
            self.assertEqual(result['opposite_pd_feedback_sign_ms'], [20, 24])
            self.assertFalse(result['root_cause_established'])

    def test_success_is_counterexample_not_full_qualification(self):
        """Identical timing and gain values also occurred with a stable startup prefix."""
        result = FORENSICS.analyze(self.traces['success'])
        self.assertEqual(result['opposite_pd_feedback_sign_ms'], [])
        self.assertGreater(result['full_trace_gaps'], 0)
        self.assertFalse(result['root_cause_established'])

    def test_record_corruption_rejected(self):
        """Raw-byte corruption must fail the retained per-record digest check."""
        trace = copy.deepcopy(self.traces['positive'])
        trace['records'][0]['raw_hex'] = 'ff' + trace['records'][0]['raw_hex'][2:]
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            FORENSICS.analyze(trace)

    def test_missing_prefix_record_rejected(self):
        """A missing startup record cannot silently become a complete transcript."""
        trace = copy.deepcopy(self.traces['positive'])
        trace['records'].pop(3)
        with self.assertRaisesRegex(ValueError, 'prefix gap'):
            FORENSICS.analyze(trace)

    def test_dequeued_is_not_transmitted(self):
        """Even a self-consistent synthetic queue event cannot substitute for actual TX."""
        trace = copy.deepcopy(self.traces['positive'])
        record = next(row for row in trace['records'] if bytes.fromhex(row['raw_hex'])[72] == 4)
        raw = bytearray.fromhex(record['raw_hex'])
        raw[72] = 2
        record['raw_hex'] = raw.hex()
        record['sha256'] = hashlib.sha256(raw).hexdigest()
        with self.assertRaisesRegex(ValueError, 'actual TX sequence'):
            FORENSICS.analyze(trace)

    def test_convenience_decode_does_not_override_raw(self):
        """Changing a decoded display value must not alter independent raw-byte analysis."""
        trace = copy.deepcopy(self.traces['positive'])
        for record in trace['records']:
            record['decoded']['torque_nm'] = 0
        self.assertEqual(FORENSICS.analyze(trace), FORENSICS.analyze(self.traces['positive']))


if __name__ == '__main__':
    unittest.main()
