"""Verify the frozen engineering/remote candidate; deploy only after current power-off confirmation.

Default execution is offline. The existing USB/SWD pairing, full backup and exact
readback transaction is reused; this adapter never sends serial motor commands.
"""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace

import deploy_idle_feedback_poweroff as guarded

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_NEW = '99a9496e672f5f4e1296da52fa83a5a4985ae7462cbfe4198964d36da5ff28d1'
EXPECTED_OLD = 'e6334b3ee710d8cd0fe2d31fe7f8f3eba345bb490fd22bef6005b2ff7c9f6fd6'


def prepare():
    """Validate all frozen inputs and both image formats without opening USB or SWD."""
    from intelhex import IntelHex
    shared = guarded.shared
    manifest_path = ROOT / 'output/idle-feedback/engineering-remote-release-20261027.json'
    manifest_payload = manifest_path.read_bytes()
    manifest = json.loads(manifest_payload)
    shared.require(manifest['schema'] == 'mode2-engineering-remote-v1' and
                   manifest['target'] == 'LCD-POS-Load-Test', 'Wrong release schema or target')
    for entry in manifest['files'] + manifest['artifacts']:
        path = shared.bounded_path(ROOT / entry['path'], ROOT)
        shared.require(shared.digest(path.read_bytes()) == entry['sha256'],
                       'Frozen input changed: ' + entry['path'])
    image_payload = (ROOT / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (ROOT / manifest['artifacts'][1]['path']).read_bytes()
    old_path = ROOT / 'output/idle-feedback/mode2-build-final-20261027/LCD-POS-Load-Test.hex'
    old_payload = old_path.read_bytes()
    shared.require(shared.digest(image_payload) == EXPECTED_NEW, 'Wrong candidate image')
    shared.require(shared.digest(old_payload) == EXPECTED_OLD, 'Wrong previous image')
    segments = shared.verify_image_pair(image_payload, elf_payload)
    shared.require(sum(len(data) for _, data in segments) == manifest['verified_image_bytes'],
                   'Image coverage mismatch')
    print('ENGINEERING_REMOTE_OFFLINE_VERIFIED', EXPECTED_NEW,
          manifest['verified_image_bytes'], flush=True)
    return SimpleNamespace(manifest_path=manifest_path, manifest_payload=manifest_payload,
        old_path=old_path, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main():
    """Require explicit deployment and present physical power-off confirmation before device access."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    arguments = parser.parse_args()
    if arguments.deploy and not arguments.motor_power_off_confirmed:
        parser.error('Confirm motor power off and independent support before deployment')
    prepared = prepare()
    if arguments.deploy:
        guarded.EXPECTED_NEW = EXPECTED_NEW
        guarded.deploy(prepared)


if __name__ == '__main__':
    main()
