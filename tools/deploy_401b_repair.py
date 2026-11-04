"""Deploy the frozen 401b repair through the existing powered safety transaction.

The repair has its own 12-check receipt, not the unrecovered historical acceptance
matrix. No motor motion, enable, mode change or parameter write is introduced.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    'D:/download/TCG/output/can_id_commissioning_20260909/python_deps',
    str(ROOT.parent / 's3519-adrc-lcd-mit/output/adrc/hardware/python_deps_20261010'),
    str(ROOT / 'output/driver_6620_20261023/analysis_dependencies'),
]
import deploy_single_static_test as shared


def prepare_repair():
    """Freeze and check build, regression receipts, all audited sources and old image."""
    frozen = []

    def read_frozen(path):
        """Read an exact local input and retain its bytes for deployment recheck."""
        payload = path.read_bytes()
        frozen.append(shared.FrozenFile(path.resolve(), payload))
        return payload

    evidence_root = Path('D:/download/TCG/output/repair_401b_20261023')
    receipt_path = evidence_root / 'verification.json'
    receipt_payload = read_frozen(receipt_path)
    receipt = json.loads(receipt_payload)
    shared.require(len(receipt['checks']) == 12, 'Wrong repair regression matrix')
    for check in receipt['checks']:
        shared.require(check['passed'] is True and shared.digest(read_frozen(Path(check['log']))) == check['sha256'], 'Regression evidence mismatch')
    audit = json.loads(read_frozen(evidence_root / 'source_audit.json'))
    for entry in audit['current_source_hashes']:
        shared.require(shared.digest(read_frozen(ROOT / entry['path'])) == entry['sha256'], 'Audited source changed: ' + entry['path'])
    manifest = json.loads(read_frozen(ROOT / 'output/repair_20261023/pos_build_native/build_manifest.json'))
    shared.require(manifest['passed'] is True and manifest['target'] == 'LCD-POS-Load-Test', 'Build not passed')
    artifacts = {}
    for name, entry in manifest['artifacts'].items():
        artifacts[name] = read_frozen(Path(entry['copy']))
        shared.require(shared.digest(artifacts[name]) == entry['sha256'], 'Build artifact changed')
    shared.require(shared.digest(artifacts['hex']) == '6b4c32885d6560f3c425aba69531745826b6ad1b976a3d6dd065c152773c67bc', 'Wrong repair image')
    # Read-only SWD capture identified this installed image among 182 local images.
    # The repair remains based on 401b; this pin describes the actual rollback input.
    old = read_frozen(ROOT.parent / 'seven-axis-static-load/output/pos_load_auto_20261020/20261021T050524472Z/LCD-POS-Load-Test.hex')
    shared.require(shared.digest(old) == '74a9cf9d9d08fbc194aca1fac1e5c7c3390ec8f322d82085c97f118150decb16', 'Wrong installed image candidate')
    segments = shared.verify_image_pair(artifacts['hex'], artifacts['axf'])
    # Common workspace ancestor permits freezing the original evidence in place.
    return shared.Prepared(Path('D:/download/TCG').resolve(), receipt_path, receipt_payload,
                           old, artifacts['hex'], artifacts['axf'], shared.parse_hex(old), segments, tuple(frozen))


def main():
    """Default to offline checks; require explicit supported/powered deployment flags."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--independently-supported', action='store_true')
    parser.add_argument('--motor-power-on', action='store_true')
    args = parser.parse_args()
    prepared = prepare_repair()
    print('REPAIR_INPUTS_VERIFIED', shared.digest(prepared.image_payload), flush=True)
    if args.deploy:
        shared.require(args.independently_supported and args.motor_power_on, 'Physical conditions not supplied')
        shared.TARGET_NAME = 'LCD-POS-Load-Test'
        output = ROOT / 'output/repair_20261023' / ('deploy_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
        report = shared.deploy(prepared, shared.load_hardware(), output, True, True, expected_mode='2')
        print('DEPLOY_READBACK_VERIFIED', json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
