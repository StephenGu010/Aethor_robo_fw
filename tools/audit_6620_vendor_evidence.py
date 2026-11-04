"""@file Audit public 6620 packages and conditional startup explanations offline.

No hardware modules are imported. Upgrade packages are inspected as bytes only;
their filenames and reported versions do not establish the board's image hash.
"""
import hashlib
import json
from pathlib import Path
import struct
import subprocess

from analyze_static_startup_root import analyze, require

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'output/driver_6620_20261023'
VENDOR = OUTPUT / 'vendor-firmware'
VENDOR_COMMIT = 'fa3a63611139c865ac25ed230192255e3783b92d'
PACKAGES = {
    '3520': ('固件/分立电机/3520/APP_DM3520_6620_01.bin',
             'f9d362d9f351b7ca26935b2f94f69a29c44e34cc910933f28905b42abf70a40b'),
    '2325': ('固件/分立电机/2325/APP_DM2325_6620_01.bin',
             'efc9d502a0bad2325e903e99d87f9d4ae4512eb9c921e96dc19767236d7413f8'),
}


def pinned_bytes(path, expected_hash):
    """Require the exact artifact before deriving any conclusions from its contents."""
    payload = path.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == expected_hash, 'artifact hash mismatch: ' + str(path))
    return payload


def audit_packages():
    """Pin upstream commit, inventory and distinct model-specific package identities."""
    commit = subprocess.check_output(['git', '-C', str(VENDOR), 'rev-parse', 'HEAD'], text=True).strip()
    require(commit == VENDOR_COMMIT, 'unexpected vendor checkout')
    inventory = subprocess.check_output(['git', '-C', str(VENDOR), '-c', 'core.quotepath=false',
                                        'ls-tree', '-r', '--name-only', 'HEAD'], encoding='utf-8').splitlines()
    source_extensions = {'.c', '.cpp', '.h', '.s', '.elf', '.axf', '.map'}
    source_files = [name for name in inventory if Path(name).suffix.lower() in source_extensions]
    result = []
    for model, (relative, digest) in PACKAGES.items():
        payload = pinned_bytes(VENDOR / relative, digest)
        stack, reset = struct.unpack_from('<II', payload)
        # This checks only offset zero, not all possible container formats/offsets.
        plausible_vector = 0x20000000 <= stack < 0x40000000 and 0x08000001 <= reset < 0x10000000 and reset & 1
        result.append(dict(model=model, path=relative, sha256=digest, bytes=len(payload),
                           first_words=[hex(stack), hex(reset)],
                           conventional_cortex_m_vector_at_offset_zero=bool(plausible_vector)))
    return dict(commit=commit, files=len(inventory), control_sources_or_symbols=source_files,
                packages=result, board_image_hash_known=False,
                internal_enable_logic_recovered=False)


def conditional_models(trace):
    """Test only explicit sample-aligned hypotheses, not an unobserved current loop."""
    result = analyze(trace)
    command = result['command']
    rows = result['timeline']
    comparisons = []
    for sample in rows:
        if sample['state'] != 1 or abs(sample['torque']) <= 1:
            continue
        history = [row['native_coordinate_pd_nm'] for row in rows if row['ms'] <= sample['ms']]
        zero_target_pd = (-command['kp'] * sample['position'] +
                          command['kd'] * (command['velocity'] - sample['velocity']) + command['feedforward'])
        implied_target = sample['position'] + (sample['torque'] - command['feedforward'] -
                         command['kd'] * (command['velocity'] - sample['velocity'])) / command['kp']
        comparisons.append(dict(ms=sample['ms'], feedback_nm=sample['torque'],
                                logged_pd_history_interval_nm=[min(history), max(history)],
                                outside_logged_pd_history=not min(history) <= sample['torque'] <= max(history),
                                zero_target_pd_nm=zero_target_pd,
                                zero_target_sign_opposite=zero_target_pd * sample['torque'] < 0,
                                implied_target_under_instantaneous_pd_rad=implied_target,
                                feedback_power_proxy_w=sample['torque'] * sample['velocity']))
    return dict(boot=result['boot'], generation=result['generation'], comparisons=comparisons,
                limitation='Sample-aligned PD history only; unknown intersample motion, current-loop state, '
                           'latency and internal target are not reconstructed. Power is a protocol proxy, '
                           'not an independently measured mechanical power.')


def audit_torque_direction():
    """Cross-check a prior same-version pure-torque observation without qualifying it."""
    root = ROOT.parent / 's3519-adrc-lcd-mit/output/adrc'
    diagnostic_path = root / 'vendor_review_20261015/visible_trial_2_diagnostic.json'
    diagnostic = json.loads(diagnostic_path.read_text(encoding='utf-8'))
    original = root / 'hardware/handoff_window_20260923/motor7_handoff_20261015T043914122Z.txt'
    digest = '8862359622fb3ab58408f6ed8c18847ec3762259f14bb95b8da2d475448cbbfd'
    transcript = pinned_bytes(original, digest).decode('utf-8-sig')
    require('sw=808597046 sub=49' in transcript, 'historical version evidence absent')
    rows = diagnostic['raw_rows']
    sample = rows[diagnostic['stop_index']]
    require(sample['rdecoded'] > 0 and sample['torque'] > 0 and sample['native_p'] > rows[0]['native_p'],
            'positive direction counterexample absent')
    # Check key decimal values against the retained raw host transcript, too.
    for value in (sample['rdecoded'], sample['torque'], sample['native_p']):
        require(str(value) in transcript, 'diagnostic value missing in pinned transcript')
    return dict(raw_path=str(original), raw_sha256=digest, software_raw=808597046, subversion_raw=49,
                receipt_nm=sample['rdecoded'], feedback_nm=sample['torque'],
                start_native_rad=rows[0]['native_p'], stop_native_rad=sample['native_p'],
                native_delta_rad=sample['native_p'] - rows[0]['native_p'],
                outcome='historical ABORT, not a passed trial',
                implication='A blanket reversal of torque feedback relative to native position is unsupported; '
                            'this does not qualify MIT nonzero-gain startup or prove current board identity.')


def main():
    """Save pinned package findings and conditional counterexamples; never claim a fix."""
    traces = json.loads((ROOT / 'output/startup_root_20261023/analysis.json').read_text(encoding='utf-8'))['traces']
    findings = []
    for source in traces:
        payload = pinned_bytes(Path(source['path']), source['sha256'])
        findings.append(conditional_models(json.loads(payload.decode('utf-8-sig'))))
    report = dict(vendor=audit_packages(), startup=findings, torque_direction=audit_torque_direction(),
                  root_cause_established=False, firmware_changed=False, hardware_access=False)
    (OUTPUT / 'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    require(len(findings) == 4 and sum(len(row['comparisons']) for row in findings) == 6,
            'unexpected fixed-evidence coverage')
    require(all(sample['outside_logged_pd_history'] and sample['zero_target_sign_opposite']
                for row in findings for sample in row['comparisons']), 'counterexample changed')
    print('AUDIT_PASS packages=2 traces=4 anomalous_samples=6 root_cause_established=0 hardware_access=0')


if __name__ == '__main__':
    main()
