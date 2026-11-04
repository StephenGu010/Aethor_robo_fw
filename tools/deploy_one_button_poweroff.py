"""Freeze/verify the one-button candidate offline; hardware deployment requires explicit power-off flags.

The original paired backup/program/readback transaction is reused. No serial motor
commands are available. Existing manifests and deployed artifacts are never replaced.
"""
import argparse
import io
import json
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime

import deploy_idle_feedback_poweroff as guarded

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'output/idle-feedback/one-button-reviewed-build-20261027'
MANIFEST = ROOT / 'output/idle-feedback/one-button-release-20261027.json'
EXPECTED_NEW = '3d731bd1478935de2439ba52715a3dc352858a765d7961b663b581a787867cc2'
EXPECTED_OLD = '99a9496e672f5f4e1296da52fa83a5a4985ae7462cbfe4198964d36da5ff28d1'
OLD_PATH = ROOT / 'output/idle-feedback/engineering-remote-final-20261027/LCD-POS-Load-Test.hex'


def file_entry(path):
    """Hash one explicit bounded file and retain its project-relative name."""
    bounded = guarded.shared.bounded_path(path, ROOT)
    return {'path': bounded.relative_to(ROOT).as_posix(), 'sha256': guarded.shared.digest(bounded.read_bytes())}


def freeze():
    """Create a new immutable manifest only for the verified pinned build and unchanged compile inputs."""
    shared = guarded.shared
    shared.require(not MANIFEST.exists(), 'Manifest already exists; refusal to replace frozen evidence')
    build_report = json.loads((BUILD / 'build_manifest.json').read_text(encoding='utf-8-sig'))
    shared.require(build_report['passed'] is True and build_report['exit_code'] == 0 and
                   build_report['target'] == 'LCD-POS-Load-Test', 'Build did not pass')
    source_names = [entry['path'] for entry in json.loads(
        (ROOT / 'output/idle-feedback/engineering-remote-release-20261027.json').read_text())['files']]
    started = datetime.fromisoformat(build_report['started_at_utc'].replace('Z', '+00:00')).timestamp()
    for relative in source_names:
        source = shared.bounded_path(ROOT / relative, ROOT)
        if source.suffix.lower() in {'.c', '.h', '.inc', '.s', '.uvprojx'}:
            shared.require(source.stat().st_mtime <= started, 'Compile input changed after build start: ' + relative)
    extra_names = ['tools/deploy_one_button_poweroff.py', 'tools/deploy_idle_feedback_poweroff.py',
                   'Tests/host/remote_ui_adapter_test.c', 'Tests/host/run_idle_feedback_tests.py']
    artifacts = []
    for extension in ['hex', 'axf', 'map', 'htm']:
        artifact = BUILD / ('LCD-POS-Load-Test.' + extension)
        shared.require(shared.digest(artifact.read_bytes()) == build_report['artifacts'][extension]['sha256'],
                       'Build artifact changed: ' + extension)
        artifacts.append(file_entry(artifact))
    shared.require(artifacts[0]['sha256'] == EXPECTED_NEW, 'Unexpected release image')
    shared.require(shared.digest(OLD_PATH.read_bytes()) == EXPECTED_OLD, 'Previous image changed')
    segments = shared.verify_image_pair((BUILD / 'LCD-POS-Load-Test.hex').read_bytes(),
                                       (BUILD / 'LCD-POS-Load-Test.axf').read_bytes())
    payload = {'schema': 'one-button-remote-v1', 'target': 'LCD-POS-Load-Test',
               'flashed': False, 'hardware_qualified': False, 'previous_hex_sha256': EXPECTED_OLD,
               'verified_image_bytes': sum(len(data) for _, data in segments),
               'files': [file_entry(ROOT / name) for name in sorted(set(source_names + extra_names))],
               'artifacts': artifacts}
    with MANIFEST.open('x', encoding='utf-8') as destination:
        json.dump(payload, destination, ensure_ascii=False, indent=2)
    print('ONE_BUTTON_MANIFEST_CREATED', MANIFEST, flush=True)


def prepare():
    """Verify every frozen source and HEX/ELF byte range without opening serial or SWD."""
    from intelhex import IntelHex
    shared = guarded.shared
    manifest_payload = MANIFEST.read_bytes()
    manifest = json.loads(manifest_payload)
    shared.require(manifest['schema'] == 'one-button-remote-v1' and manifest['target'] == 'LCD-POS-Load-Test',
                   'Wrong release identity')
    for entry in manifest['files'] + manifest['artifacts']:
        shared.require(file_entry(ROOT / entry['path']) == entry, 'Frozen input changed: ' + entry['path'])
    image_payload = (ROOT / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (ROOT / manifest['artifacts'][1]['path']).read_bytes()
    old_payload = OLD_PATH.read_bytes()
    shared.require(shared.digest(image_payload) == EXPECTED_NEW, 'Wrong candidate image')
    shared.require(shared.digest(old_payload) == EXPECTED_OLD, 'Wrong prior image')
    segments = shared.verify_image_pair(image_payload, elf_payload)
    shared.require(sum(len(data) for _, data in segments) == manifest['verified_image_bytes'], 'Wrong coverage')
    print('ONE_BUTTON_OFFLINE_VERIFIED', EXPECTED_NEW, manifest['verified_image_bytes'], len(manifest['files']), flush=True)
    return SimpleNamespace(manifest_path=MANIFEST, manifest_payload=manifest_payload,
        old_path=OLD_PATH, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main():
    """Default to offline verification; require distinct explicit flags for physical deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freeze', action='store_true')
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    arguments = parser.parse_args()
    if arguments.freeze and arguments.deploy:
        parser.error('Freeze and deployment must be separate operations')
    if arguments.deploy and not arguments.motor_power_off_confirmed:
        parser.error('Confirm current 24V power off, independent support, LCD idle, USB and programmer before deployment')
    if arguments.freeze:
        freeze()
    prepared = prepare()
    if arguments.deploy:
        guarded.EXPECTED_NEW = EXPECTED_NEW
        guarded.deploy(prepared)


if __name__ == '__main__':
    main()
