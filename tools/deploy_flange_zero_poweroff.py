"""Freeze the reviewed flange-zero release; hardware requires fresh power-off confirmation.

The existing USB/SWD pairing, 1 MiB backup, exact old-image check and complete
readback transaction is reused unchanged. No serial motion commands are exposed.
"""
import argparse
from datetime import datetime
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import deploy_idle_feedback_poweroff as guarded

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'output/flange-zero/reviewed-build-20261027-linked'
MANIFEST = ROOT / 'output/flange-zero/release-20261027.json'
PREVIOUS_ROOT = Path('D:/download/TCG/idle-feedback-candidate-20261026')
PREVIOUS_MANIFEST = PREVIOUS_ROOT / 'output/idle-feedback/one-button-release-20261027.json'
PREVIOUS_IMAGE = PREVIOUS_ROOT / 'output/idle-feedback/one-button-reviewed-build-20261027/LCD-POS-Load-Test.hex'
PREVIOUS_SHA256 = '3d731bd1478935de2439ba52715a3dc352858a765d7961b663b581a787867cc2'


def file_entry(path):
    """Bind one explicit project file to its bounded relative path and hash."""
    bounded = guarded.shared.bounded_path(path, ROOT)
    return {'path': bounded.relative_to(ROOT).as_posix(), 'sha256': guarded.shared.digest(bounded.read_bytes())}


def verify_entries(entries):
    """Reject altered inputs and traversal before constructing immutable deployment bytes."""
    for entry in entries:
        guarded.shared.require(file_entry(ROOT / entry['path']) == entry,
                               'Frozen input changed: ' + entry['path'])


def freeze():
    """Create a new immutable source/artifact manifest only after a fresh clean build."""
    require = guarded.shared.require
    require(not MANIFEST.exists(), 'Release manifest exists; refusal to replace evidence')
    build_report = json.loads((BUILD / 'build_manifest.json').read_text(encoding='utf-8-sig'))
    require(build_report['passed'] is True and build_report['exit_code'] == 0 and
            build_report['target'] == 'LCD-POS-Load-Test', 'Reviewed build must pass')
    started = datetime.fromisoformat(build_report['started_at_utc'].replace('Z', '+00:00')).timestamp()
    names = {entry['path'] for entry in json.loads(PREVIOUS_MANIFEST.read_text())['files']}
    baseline = json.loads((ROOT / 'baseline.json').read_text(encoding='utf-8-sig'))
    for startup_name in ('MDK-ARM/startup_stm32h723xx.s', 'MDK-ARM/aethor_memory.sct'):
        startup_hash = next(entry['sha256'] for entry in baseline['files'] if entry['path'] == startup_name)
        require(file_entry(ROOT / startup_name)['sha256'] == startup_hash, 'Startup/linker differs from original baseline')
    project = ROOT / 'MDK-ARM/LCD-POS-Load-Test.uvprojx'
    project_tree = ET.parse(project)
    for entry in project_tree.findall('.//FilePath') + project_tree.findall('.//ScatterFile'):
        # Keil uses legitimate ../Core paths. Collapse lexical segments first;
        # bounded_path still checks root confinement and every reparse component.
        source = guarded.shared.bounded_path(Path(os.path.abspath(project.parent / entry.text)), ROOT)
        names.add(source.relative_to(ROOT).as_posix())
    for folder in ('App', 'Core', 'Drivers', 'MDK-ARM', 'Middlewares', 'Ui', 'USB_DEVICE', 'Tests', 'tools'):
        names.update(path.relative_to(ROOT).as_posix() for path in (ROOT / folder).rglob('*')
                     if path.is_file() and path.suffix.lower() in {'.c', '.h', '.inc', '.s', '.sct', '.uvprojx', '.py', '.ps1'}
                     and not any(part in {'build', '__pycache__', 'output'} for part in path.relative_to(ROOT).parts))
    validation_path = ROOT / 'output/flange-zero/firmware-validation.json'
    validation = json.loads(validation_path.read_text(encoding='utf-8-sig'))
    names.add(validation_path.relative_to(ROOT).as_posix())
    names.add((BUILD / 'build_manifest.json').relative_to(ROOT).as_posix())
    for result in validation['tests']:
        require(result['exit_code'] == 0, 'Offline regression failed: ' + result['label'])
        receipts = result['files'] if 'files' in result else [{'path': result['log'], 'sha256': result['sha256']}]
        for entry in receipts:
            require(file_entry(ROOT / entry['path'])['sha256'] == entry['sha256'], 'Regression receipt changed')
            names.add(entry['path'])
    inventory_path = ROOT / 'output/flange-zero/build-input-inventory.json'
    require(inventory_path.is_file(), 'Build input inventory is required')
    names.add(inventory_path.relative_to(ROOT).as_posix())
    for relative in names:
        path = guarded.shared.bounded_path(ROOT / relative, ROOT)
        if path.suffix.lower() in {'.c', '.h', '.inc', '.s', '.sct', '.uvprojx'}:
            require(path.stat().st_mtime <= started, 'Compile input changed after build: ' + relative)
    files = [file_entry(ROOT / name) for name in sorted(names)]
    artifacts = [file_entry(BUILD / ('LCD-POS-Load-Test.' + suffix)) for suffix in ('hex', 'axf', 'map', 'htm')]
    for suffix, entry in zip(('hex', 'axf', 'map', 'htm'), artifacts):
        require(entry['sha256'] == build_report['artifacts'][suffix]['sha256'], 'Build artifact changed: ' + suffix)
    segments = guarded.shared.verify_image_pair((ROOT / artifacts[0]['path']).read_bytes(),
                                               (ROOT / artifacts[1]['path']).read_bytes())
    require(guarded.shared.digest(PREVIOUS_IMAGE.read_bytes()) == PREVIOUS_SHA256, 'Expected previous image changed')
    payload = {'schema': 'flange-zero-release-v1', 'target': 'LCD-POS-Load-Test',
               'flashed': False, 'hardware_qualified': False, 'files': files, 'artifacts': artifacts,
               'previous_hex_sha256': PREVIOUS_SHA256,
               'verified_image_bytes': sum(len(data) for _, data in segments)}
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    with MANIFEST.open('x', encoding='utf-8') as destination:
        json.dump(payload, destination, ensure_ascii=False, indent=2)
    print('FLANGE_ZERO_MANIFEST_CREATED', artifacts[0]['sha256'], len(files), flush=True)


def prepare():
    """Recheck every frozen byte and HEX/ELF range without accessing serial or SWD."""
    from intelhex import IntelHex
    payload = MANIFEST.read_bytes()
    manifest = json.loads(payload)
    require = guarded.shared.require
    require(manifest['schema'] == 'flange-zero-release-v1' and manifest['target'] == 'LCD-POS-Load-Test', 'Wrong release')
    verify_entries(manifest['files'] + manifest['artifacts'])
    image_payload = (ROOT / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (ROOT / manifest['artifacts'][1]['path']).read_bytes()
    old_payload = PREVIOUS_IMAGE.read_bytes()
    require(guarded.shared.digest(old_payload) == PREVIOUS_SHA256 == manifest['previous_hex_sha256'], 'Wrong previous image')
    segments = guarded.shared.verify_image_pair(image_payload, elf_payload)
    require(sum(len(data) for _, data in segments) == manifest['verified_image_bytes'], 'Wrong image coverage')
    print('FLANGE_ZERO_OFFLINE_VERIFIED', guarded.shared.digest(image_payload), manifest['verified_image_bytes'], len(manifest['files']), flush=True)
    return SimpleNamespace(manifest_path=MANIFEST, manifest_payload=payload,
        old_path=PREVIOUS_IMAGE, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))), image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def main():
    """Keep offline verification separate from explicit, physically confirmed deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freeze', action='store_true')
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    args = parser.parse_args()
    if args.freeze and args.deploy:
        parser.error('Freeze and deployment must be separate operations')
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Confirm current 24V off, independent support, LCD idle, USB and programmer before deployment')
    if args.freeze:
        freeze()
    prepared = prepare()
    if args.deploy:
        guarded.EXPECTED_NEW = guarded.shared.digest(prepared.image_payload)
        guarded.deploy(prepared)


if __name__ == '__main__':
    main()
