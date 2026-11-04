"""@file Compare pinned successful/failed MIT trials offline without changing control.

Raw headers and record digests are verified before comparing state. A stable
prefix or shared configuration cannot establish internal driver state or load.
"""
import hashlib
import json
from pathlib import Path
import struct

from analyze_static_startup_root import analyze, raw_record, require

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT.parent / 'seven-axis-static-load/output/static_startup_sync_20261020'
SOURCES = (
    ('success', 'lcd_pilot_trace_20261020T0826Z.json',
     'a4df4d2d223f457ef79931a5ea9757586b36bb77d38810bdbcfff22308431c37'),
    ('failure', 'failure_trace_20261020T084016455Z.json',
     '8f84c6fa2491f60155c8a76429653221ea41fbbdf97642a7ccb13d7bbae1edf4'),
)


def inspect_trial(name, filename, expected_hash):
    """Extract verified trial settings and pre-fault command invariance only."""
    path = EVIDENCE / filename
    payload = path.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == expected_hash, 'source digest changed')
    trace = json.loads(payload.decode('utf-8-sig'))
    header = bytes.fromhex(trace['header_hex'])
    require(hashlib.sha256(header).hexdigest() == trace['header_sha256'], 'header digest mismatch')
    result = analyze(trace)
    axis = struct.unpack_from('<6I11f', header, 560)
    started = struct.unpack_from('<Q', header, 48)[0]
    records = [raw_record(record) for record in trace['records']]
    fault = next((record for record in records if record['event'] == 8), None)
    cutoff = fault['timestamp'] if fault else started + 48000
    controls = [record for record in records if record['timestamp'] < cutoff and
                record['event'] in (2, 3, 4, 12) and len(record['payload']) == 8 and
                record['payload'] not in (bytes.fromhex('fffffffffffffffc'),
                                          bytes.fromhex('fffffffffffffffd'))]
    require(controls, 'missing pre-fault controls')
    feedback = [record for record in records if record['event'] == 7]
    temperatures = sorted({tuple(record['payload'][6:8]) for record in feedback})
    return dict(name=name, source=str(path), sha256=expected_hash, **result,
                started_us=started, reference=axis[0], evidence_generation=axis[1],
                esc_id=axis[3], master_id=axis[4], mode=axis[5],
                frozen_kp=axis[8], frozen_kd=axis[9], frozen_feedforward=axis[10],
                temperature_pairs=temperatures,
                first_fault_ms=None if fault is None else (fault['timestamp'] - started) / 1000,
                retained_control_events_before_cutoff=len(controls),
                retained_controls_match_preload=all(record['payload'].hex() == result['command_hex']
                                                    for record in controls),
                normal_hold_actual_tx_proven=False,
                load_mass_g=None, prior_driver_state_history='not captured')


def main():
    """Save reproducible comparison, keeping unknowns explicit and root-fix gate closed."""
    trials = [inspect_trial(*source) for source in SOURCES]
    success, failure = trials
    compared = ('boot', 'ranges', 'ratio', 'esc_id', 'master_id', 'mode',
                'frozen_kp', 'frozen_kd', 'frozen_feedforward', 'temperature_pairs', 'actual_tx_ms')
    equal_fields = {field: success[field] == failure[field] for field in compared}
    require(all(equal_fields.values()), 'historical equality changed')
    require(all(trial['retained_controls_match_preload'] for trial in trials),
            'pre-fault normal control differs from frozen target')
    require(success['first_fault_ms'] is None and failure['first_fault_ms'] == 28,
            'fault chronology changed')
    require(success['full_trace_gaps'] > 0 and failure['full_trace_gaps'] == 0,
            'coverage changed')
    report = dict(trials=trials, equal_fields=equal_fields,
                  interval_ms=(failure['started_us'] - success['started_us']) / 1000,
                  hypothesis_status={
                      'different_recorded_gains_ranges_or_temperature': 'unsupported',
                      'changed_retained_normal_hold_payload_before_fault': 'unsupported',
                      'success_proves_1kg_hold': 'unsupported: successful load unknown',
                      'all_runs_same_driver_internal_state': 'unknown',
                      'startup_root_cause': 'NOT_ESTABLISHED'},
                  hardware_access=False, production_control_changed=False)
    output = ROOT / 'output/repair_20261023/static_success_comparison.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('COMPARISON_PASS same_boot=1 equal_fields=11 retained_controls_unchanged=1 root_fix=NOT_ESTABLISHED')
    print(output)


if __name__ == '__main__':
    main()
