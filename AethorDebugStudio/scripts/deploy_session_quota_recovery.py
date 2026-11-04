"""@file Deploy the frozen UI-session/first-torque-fault repair once, only after current motor power-off confirmation."""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace
import deploy_session_recovery as previous
from intelhex import IntelHex

CANDIDATE=Path('D:/download/TCG/session-quota-recovery-candidate-20261029')
HOST=Path('D:/download/TCG/AethorDebugStudio')
MANIFEST=CANDIDATE/'output/session-quota-recovery/release-offline-20261029.json'
MANIFEST_SHA256='752973037445c2b59d017e2fd1351f41da8690f7207c2589c678c767bd325e13'
EXPECTED_OLD='1526a4e7c10212945f41c64773ac3f87e65f8045d30aaadb5fba15381d07b48d'
EXPECTED_NEW='02f1ee453db560fb9022e60ac1cc23a178d99ba4854ea50f937951008d6b40e3'
VERIFIED_BYTES=521764
guarded=previous.guarded


def prepare():
    """Verify the deployed baseline, every frozen firmware/host file, and matching full HEX/ELF before hardware access."""
    old=previous.prepare()
    require=guarded.shared.require
    payload=MANIFEST.read_bytes()
    require(guarded.shared.digest(payload)==MANIFEST_SHA256,'Frozen UI session manifest changed')
    manifest=json.loads(payload)
    require(manifest['schema']=='id7-session-quota-recovery-offline-v1' and len(manifest['files'])==934,'Wrong source/schema')
    require(manifest['expected_running_hex']==EXPECTED_OLD and not manifest['torque_thresholds_changed'],'Wrong baseline/protection')
    require(guarded.shared.digest(old.image_payload)==EXPECTED_OLD,'Unexpected deployed baseline bytes')
    for entry in manifest['files']+manifest['artifacts']:
        path=guarded.shared.bounded_path(CANDIDATE/entry['path'],CANDIDATE)
        require(guarded.shared.digest(path.read_bytes())==entry['sha256'],'Frozen firmware input changed: '+entry['path'])
    for entry in manifest['host_files']:
        path=guarded.shared.bounded_path(HOST/entry['path'],HOST)
        require(guarded.shared.digest(path.read_bytes())==entry['sha256'],'Frozen host input changed: '+entry['path'])
    image_payload=(CANDIDATE/manifest['artifacts'][0]['path']).read_bytes()
    elf_payload=(CANDIDATE/manifest['artifacts'][1]['path']).read_bytes()
    require(guarded.shared.digest(image_payload)==EXPECTED_NEW,'Wrong repair image')
    segments=guarded.shared.verify_image_pair(image_payload,elf_payload)
    require(sum(len(data) for _,data in segments)==manifest['verified_image_bytes']==VERIFIED_BYTES,'Wrong occupied image size')
    print('SESSION_QUOTA_RECOVERY_OFFLINE_VERIFIED',EXPECTED_NEW,len(manifest['files']),VERIFIED_BYTES,flush=True)
    return SimpleNamespace(manifest_path=MANIFEST,manifest_payload=payload,
        old_path=previous.CANDIDATE/json.loads(previous.MANIFEST.read_bytes())['artifacts'][0]['path'],
        old_payload=old.image_payload,image_payload=image_payload,elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old.image_payload.decode('ascii'))),image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main(argv=None):
    """Keep the default offline; current independent support and 24V-off evidence gates the one non-retrying deployment."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy',action='store_true')
    parser.add_argument('--motor-power-off-confirmed',action='store_true')
    args=parser.parse_args(argv)
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, independent arm/payload support, LCD idle, USB/probe connected')
    prepared=prepare()
    if args.deploy:
        guarded.ROOT=CANDIDATE;guarded.poweroff.ROOT=CANDIDATE;guarded.EXPECTED_NEW=EXPECTED_NEW
        (CANDIDATE/'output/idle-feedback').mkdir(parents=True,exist_ok=True)
        guarded.deploy(prepared)
        report_path=Path(json.loads((CANDIDATE/'output/idle-feedback/latest-deployment.json').read_text())['report'])
        report=json.loads(report_path.read_text())
        report.update(motion_authorized=False,motion_commands_sent=0,thermal_qualified=False,
            supported_unloaded=False,independent_support='arm_and_payload=user_confirmation',
            deployment_adapter=str(Path(__file__)),deployment_adapter_sha256=guarded.shared.digest(Path(__file__).read_bytes()))
        guarded.poweroff.write_report(report_path.parent,report)


if __name__=='__main__':
    main()
