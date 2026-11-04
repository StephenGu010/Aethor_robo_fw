"""@file Frozen fault-session release deployment; default is offline and hardware requires fresh power-off confirmation."""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace
import deploy_torque_envelope as previous
from intelhex import IntelHex

CANDIDATE=Path('D:/download/TCG/session-recovery-candidate-20261029')
MANIFEST=CANDIDATE/'output/session-recovery/release-offline-20261029.json'
MANIFEST_SHA256='8d392c983d407b056e1696ae1dfb41edfa478903351a446216b38f1537d04538'
EXPECTED_OLD=previous.EXPECTED_NEW
EXPECTED_NEW='1526a4e7c10212945f41c64773ac3f87e65f8045d30aaadb5fba15381d07b48d'
VERIFIED_BYTES=521204
guarded=previous.guarded


def prepare():
    """Pin the deployed baseline, every reviewed source and the complete matching HEX/ELF image before hardware imports."""
    old=previous.prepare()
    require=guarded.shared.require
    payload=MANIFEST.read_bytes()
    require(guarded.shared.digest(payload)==MANIFEST_SHA256,'Frozen session manifest changed')
    manifest=json.loads(payload)
    require(manifest['schema']=='id7-remote-fault-release-offline-v1' and len(manifest['files'])==934,'Wrong release schema')
    require(manifest['expected_running_hex']==EXPECTED_OLD and manifest['torque_thresholds_changed'] is False,
            'Wrong baseline or changed torque protection')
    for entry in manifest['files']+manifest['artifacts']:
        path=guarded.shared.bounded_path(CANDIDATE/entry['path'],CANDIDATE)
        require(guarded.shared.digest(path.read_bytes())==entry['sha256'],'Frozen input changed: '+entry['path'])
    image_payload=(CANDIDATE/manifest['artifacts'][0]['path']).read_bytes()
    elf_payload=(CANDIDATE/manifest['artifacts'][1]['path']).read_bytes()
    require(guarded.shared.digest(image_payload)==EXPECTED_NEW,'Wrong session recovery image')
    segments=guarded.shared.verify_image_pair(image_payload,elf_payload)
    require(sum(len(data) for _,data in segments)==manifest['verified_image_bytes']==VERIFIED_BYTES,'Wrong occupied image size')
    print('SESSION_RECOVERY_OFFLINE_VERIFIED',EXPECTED_NEW,len(manifest['files']),VERIFIED_BYTES,flush=True)
    return SimpleNamespace(manifest_path=MANIFEST,manifest_payload=payload,
        old_path=previous.CANDIDATE/json.loads(previous.MANIFEST.read_bytes())['artifacts'][0]['path'],
        old_payload=old.image_payload,image_payload=image_payload,elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old.image_payload.decode('ascii'))),image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main(argv=None):
    """Gate one deployment with current independent support and motor-power-off state; never retry or start motor work."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy',action='store_true')
    parser.add_argument('--motor-power-off-confirmed',action='store_true')
    args=parser.parse_args(argv)
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, arm and payload independently supported, LCD idle, USB and probe connected')
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
