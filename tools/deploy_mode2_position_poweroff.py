"""Deploy the frozen mode-2 position-display repair with motor power explicitly off.

The default action only verifies local files. The existing paired USB/SWD backup,
program and readback transaction is reused without serial motor commands.
"""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace

import deploy_idle_feedback_poweroff as guarded

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_NEW = 'e6334b3ee710d8cd0fe2d31fe7f8f3eba345bb490fd22bef6005b2ff7c9f6fd6'
EXPECTED_OLD = '7c18c51647cd07aa936db9b2114262629f557c3642cda287e5578fa398c83aeb'


def prepare():
    """Check frozen sources, previous image and complete ELF/HEX identity without hardware."""
    from intelhex import IntelHex
    shared = guarded.shared
    manifest_path = ROOT / 'output/idle-feedback/mode2-release.json'
    manifest_payload = manifest_path.read_bytes()
    manifest = json.loads(manifest_payload)
    baseline_payload = (ROOT / 'baseline.json').read_bytes()
    shared.require(shared.digest(baseline_payload) == manifest['baseline_sha256'], 'Baseline changed')
    expected = {item['path']: item['sha256'] for item in json.loads(baseline_payload)['files']}
    expected.update({item['path']: item['sha256'] for item in manifest['files'] + manifest['artifacts']})
    for relative, expected_hash in expected.items():
        payload = shared.bounded_path(ROOT / relative, ROOT).read_bytes()
        shared.require(shared.digest(payload) == expected_hash, 'Frozen input changed: ' + relative)
    shared.require(manifest['target'] == 'LCD-POS-Load-Test' and
                   manifest['new_test_groups'] == 8, 'Wrong release identity')
    image_payload = (ROOT / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (ROOT / manifest['artifacts'][1]['path']).read_bytes()
    old_path = ROOT / 'output/idle-feedback/release/LCD-POS-Load-Test.hex'
    old_payload = old_path.read_bytes()
    shared.require(shared.digest(image_payload) == EXPECTED_NEW, 'Wrong mode-2 candidate')
    shared.require(shared.digest(old_payload) == EXPECTED_OLD, 'Wrong previous image')
    segments = shared.verify_image_pair(image_payload, elf_payload)
    print('MODE2_OFFLINE_VERIFIED', EXPECTED_NEW, sum(len(data) for _, data in segments), flush=True)
    return SimpleNamespace(manifest_path=manifest_path, manifest_payload=manifest_payload,
        old_path=old_path, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main():
    """Require explicit deployment and current motor-power-off confirmation before access."""
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
