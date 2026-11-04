"""@file Deploy the frozen 209e5fc5 release once after current power-off confirmation; never send motor commands."""
import argparse
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

CANDIDATE = Path('D:/download/TCG/studio-usability-torque-candidate-20261029')
RELEASE = Path('D:/download/TCG/AethorDebugStudio/output/usability-torque-release')
MANIFEST = RELEASE / 'release-manifest.json'
MANIFEST_SHA256 = '35412c250a2277d36a96b4710fd09910ebefe7c57a9e6440c755c069919a22a2'
OLD_PATH = Path('D:/download/TCG/session-quota-recovery-candidate-20261029/output/session-quota-recovery/keil-final/LCD-POS-Load-Test.hex')
EXPECTED_OLD = '02f1ee453db560fb9022e60ac1cc23a178d99ba4854ea50f937951008d6b40e3'
EXPECTED_NEW = '209e5fc5788078e02d1dba0348a5513efd6c6ac02f0f25e3b273b080e489b118'
sys.path.insert(0, str(CANDIDATE / 'Tools'))
import deploy_idle_feedback_poweroff as guarded
from intelhex import IntelHex


def prepare():
    """Verify all frozen inputs and paired HEX/ELF bytes before any USB/SWD access."""
    require = guarded.shared.require
    payload = MANIFEST.read_bytes()
    require(guarded.shared.digest(payload) == MANIFEST_SHA256, 'Frozen release manifest changed')
    manifest = json.loads(payload)
    require(manifest['schema'] == 'studio-usability-adjustable-torque-offline-v1'
            and manifest['target'] == 'LCD-POS-Load-Test', 'Unexpected release identity')
    require(len(manifest['firmware_files']) == 934 and len(manifest['host_files']) == 144,
            'Unexpected frozen source coverage')
    for root, entries in ((CANDIDATE, manifest['firmware_files']),
                          (RELEASE / 'host-source', manifest['host_files']),
                          (RELEASE, manifest['products'] + manifest['evidence'])):
        for item in entries:
            path = guarded.shared.bounded_path(root / item['path'], root)
            require(guarded.shared.digest(path.read_bytes()) == item['sha256'], 'Frozen input changed: ' + str(path))
    image_payload = (RELEASE / 'firmware/LCD-POS-Load-Test.hex').read_bytes()
    elf_payload = (RELEASE / 'firmware/LCD-POS-Load-Test.axf').read_bytes()
    old_payload = OLD_PATH.read_bytes()
    require(guarded.shared.digest(image_payload) == EXPECTED_NEW, 'Unexpected new firmware')
    require(guarded.shared.digest(old_payload) == EXPECTED_OLD, 'Unexpected prior firmware')
    segments = guarded.shared.verify_image_pair(image_payload, elf_payload)
    print('USABILITY_TORQUE_OFFLINE_VERIFIED', EXPECTED_NEW,
          'occupied_bytes=' + str(sum(len(data) for _, data in segments)), flush=True)
    return SimpleNamespace(manifest_path=MANIFEST, manifest_payload=payload,
        old_path=OLD_PATH, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main(argv=None):
    """Default to offline validation; execute at most one guarded power-off deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    args = parser.parse_args(argv)
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, independent arm/payload support, LCD idle, USB/probe connected')
    prepared = prepare()
    if args.deploy:
        guarded.ROOT = CANDIDATE
        guarded.poweroff.ROOT = CANDIDATE
        guarded.EXPECTED_NEW = EXPECTED_NEW
        (CANDIDATE / 'output/idle-feedback').mkdir(parents=True, exist_ok=True)
        guarded.deploy(prepared)
        latest = json.loads((CANDIDATE / 'output/idle-feedback/latest-deployment.json').read_text())
        report_path = Path(latest['report'])
        report = json.loads(report_path.read_text())
        report.update(motion_authorized=False, motion_commands_sent=0, thermal_qualified=False,
            supported_unloaded=False, independent_support='arm_and_payload=user_confirmation',
            boot_experimental_credit=False, experiment_credit_restored=False,
            deployment_adapter=str(Path(__file__)),
            deployment_adapter_sha256=guarded.shared.digest(Path(__file__).read_bytes()))
        guarded.poweroff.write_report(report_path.parent, report)


if __name__ == '__main__':
    main()
