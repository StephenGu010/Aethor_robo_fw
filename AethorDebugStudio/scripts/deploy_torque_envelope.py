"""@file Verify and deploy only the frozen ID7 5.46Nm/10s experimental image with confirmed motor power off.

Reuses the reviewed USB/SWD pairing, running-image comparison, complete backup,
program/readback and normal-reset transaction. No motor command or retry is sent.
"""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace
import deploy_standby_recovery as previous

CANDIDATE = Path('D:/download/TCG/torque-envelope-candidate-20261029')
MANIFEST = CANDIDATE / 'output/torque-envelope/release-offline-20261029.json'
MANIFEST_SHA256 = '9b066f5712f2aeea5ebe6acc21bb4974b7354758d737ade492d4fb21768f39e6'
EXPECTED_NEW = 'e1f234a6cd76515a65894e0f48b71d0b977726063fd22e7fc8bfff3d5bc689b0'
EXPECTED_OLD = 'f82a9a72b0cf12013cca1f9037c55dc9742c603a63b453f15cd7a7463d2f3629'
VERIFIED_BYTES = 520048
guarded = previous.guarded


def prepare():
    """Rehash all frozen sources/products and compare occupied HEX/ELF bytes without hardware access."""
    from intelhex import IntelHex
    require = guarded.shared.require
    old = previous.prepare()
    require(guarded.shared.digest(old.image_payload) == EXPECTED_OLD, 'Unexpected running-image baseline')
    payload = MANIFEST.read_bytes()
    require(guarded.shared.digest(payload) == MANIFEST_SHA256, 'Frozen torque manifest changed')
    manifest = json.loads(payload)
    require(manifest['schema'] == 'id7-torque-envelope-offline-v1' and len(manifest['files']) == 932,
            'Unexpected torque release schema or source count')
    policy = manifest['policy']
    require(policy['motor_id'] == 7 and policy['axis'] == 'motor_output_shaft' and
            policy['continuous_nm'] == 3.5 and policy['peak_nm'] == 5.46 and
            policy['max_continuous_excess_us'] == 10000000 and policy['boot_authorized'] is False and
            manifest['thermal_qualified'] is False, 'Unexpected experimental policy')
    for entry in manifest['files'] + manifest['artifacts']:
        path = guarded.shared.bounded_path(CANDIDATE / entry['path'], CANDIDATE)
        require(guarded.shared.digest(path.read_bytes()) == entry['sha256'], 'Frozen input changed: ' + entry['path'])
    image_payload = (CANDIDATE / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (CANDIDATE / manifest['artifacts'][1]['path']).read_bytes()
    require(guarded.shared.digest(image_payload) == EXPECTED_NEW, 'Wrong experimental image')
    segments = guarded.shared.verify_image_pair(image_payload, elf_payload)
    require(sum(len(data) for _, data in segments) == manifest['verified_image_bytes'] == VERIFIED_BYTES,
            'Wrong occupied image size')
    old_manifest = json.loads(previous.MANIFEST.read_bytes())
    old_path = previous.CANDIDATE / old_manifest['artifacts'][0]['path']
    print('TORQUE_ENVELOPE_OFFLINE_VERIFIED', EXPECTED_NEW, len(manifest['files']), VERIFIED_BYTES, flush=True)
    return SimpleNamespace(manifest_path=MANIFEST, manifest_payload=payload,
        old_path=old_path, old_payload=old.image_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old.image_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main(argv=None):
    """Default to offline checks; both deployment and this occasion's power-off confirmation are mandatory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    args = parser.parse_args(argv)
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, arm and payload independently supported, LCD idle, USB and probe connected')
    prepared = prepare()
    if args.deploy:
        guarded.ROOT = CANDIDATE
        guarded.poweroff.ROOT = CANDIDATE
        guarded.EXPECTED_NEW = EXPECTED_NEW
        (CANDIDATE / 'output/idle-feedback').mkdir(parents=True, exist_ok=True)
        guarded.deploy(prepared)
        latest_path = CANDIDATE / 'output/idle-feedback/latest-deployment.json'
        report_path = Path(json.loads(latest_path.read_text(encoding='utf-8'))['report'])
        report = json.loads(report_path.read_text(encoding='utf-8'))
        report.update(supported_unloaded=False, independent_support='arm_and_payload=user_confirmation',
            motion_authorized=False, thermal_qualified=False, experimental_torque_nm=5.46,
            experimental_excess_window_s=10, boot_experimental_credit=False,
            deployment_adapter=str(Path(__file__)),
            deployment_adapter_sha256=guarded.shared.digest(Path(__file__).read_bytes()))
        guarded.poweroff.write_report(report_path.parent, report)


if __name__ == '__main__':
    main()
