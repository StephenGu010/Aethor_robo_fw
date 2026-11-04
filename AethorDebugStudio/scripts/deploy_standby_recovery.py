"""@file Deploy only the frozen f82a9a72 release with confirmed 24V off.

Reuse the existing USB/SWD pairing, running-image comparison, full backup,
sector programming and exact readback transaction. No serial commands are sent.
The compiled candidate and its frozen manifests remain unchanged.
"""
import argparse
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

CANDIDATE = Path('D:/download/TCG/standby-recovery-candidate-20261027')
sys.path.insert(0, str(CANDIDATE / 'tools'))
import deploy_idle_feedback_poweroff as guarded

MANIFEST = CANDIDATE / 'output/standby-recovery/release-offline-20261027.json'
MANIFEST_SHA256 = '5fd975739aa4120ea673167837f3795939a66ade77b3088332b051fe6dea94da'
PREVIOUS_IMAGE = Path('D:/download/TCG/flange-zero-candidate-20261027/output/flange-zero/reviewed-build-20261027-linked/LCD-POS-Load-Test.hex')
EXPECTED_NEW = 'f82a9a72b0cf12013cca1f9037c55dc9742c603a63b453f15cd7a7463d2f3629'
EXPECTED_OLD = '3cf419f2d1ff31a66defbcb09b2b1038721746367e053ab038b2098ed4c84549'


def prepare():
    """Verify immutable release bytes before importing hardware dependencies."""
    from intelhex import IntelHex
    require = guarded.shared.require
    payload = MANIFEST.read_bytes()
    require(guarded.shared.digest(payload) == MANIFEST_SHA256, 'Frozen manifest changed')
    manifest = json.loads(payload)
    require(manifest['schema'] == 'standby-recovery-offline-v1', 'Wrong release schema')
    for entry in manifest['files'] + manifest['artifacts']:
        path = guarded.shared.bounded_path(CANDIDATE / entry['path'], CANDIDATE)
        require(guarded.shared.digest(path.read_bytes()) == entry['sha256'], 'Frozen input changed: ' + entry['path'])
    image_payload = (CANDIDATE / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (CANDIDATE / manifest['artifacts'][1]['path']).read_bytes()
    old_path = guarded.shared.bounded_path(PREVIOUS_IMAGE, PREVIOUS_IMAGE.parents[3])
    old_payload = old_path.read_bytes()
    require(guarded.shared.digest(image_payload) == EXPECTED_NEW, 'Wrong new image')
    require(guarded.shared.digest(old_payload) == EXPECTED_OLD, 'Wrong previous image')
    segments = guarded.shared.verify_image_pair(image_payload, elf_payload)
    require(sum(len(data) for _, data in segments) == manifest['verified_image_bytes'] == 517764,
            'Wrong occupied image size')
    print('STANDBY_RECOVERY_OFFLINE_VERIFIED', EXPECTED_NEW, len(manifest['files']), 517764, flush=True)
    return SimpleNamespace(manifest_path=MANIFEST, manifest_payload=payload,
        old_path=old_path, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main(argv=None):
    """Default to offline verification and require explicit power-off before flashing."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    args = parser.parse_args(argv)
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, independent support, LCD idle, USB and programmer')
    prepared = prepare()
    if args.deploy:
        guarded.EXPECTED_NEW = EXPECTED_NEW
        guarded.deploy(prepared)
        latest_path = CANDIDATE / 'output/idle-feedback/latest-deployment.json'
        report_path = Path(json.loads(latest_path.read_text(encoding='utf-8'))['report'])
        report = json.loads(report_path.read_text(encoding='utf-8'))
        # A supported attached payload is not an unloaded arm; retain the actual confirmation.
        report.update(supported_unloaded=False, independent_support='arm_and_payload=user_confirmation',
                      payload_attached_kg_approx=1, motion_authorized=False,
                      deployment_adapter=str(Path(__file__)),
                      deployment_adapter_sha256=guarded.shared.digest(Path(__file__).read_bytes()))
        guarded.poweroff.write_report(report_path.parent, report)


if __name__ == '__main__':
    main()
