"""Independent, explicitly confirmed motor-power-off H723 maintenance deployment.

Workspace dependency: output/adrc/hardware/*_20261010.py and their installed
python_deps directories must remain available. No serial preflight is substituted
or weakened: this tool never calls the ordinary powered deployment entry point.
Only --deploy together with --motor-power-off-confirmed may access SWD.
"""

import argparse
import hashlib
import importlib.util
import io
import json
import stat
import struct
import sys
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
OLD_IMAGE = ROOT / 'output/adrc/verification/20261012T054023673Z/LCD-MIT-ADRC.hex'
OLD_SHA256 = '3214ffe6df413271ad05416c5dcf1937c68d270f9e831a575b69afdd8d1f21a8'
PROBE_UID = '07000001000000000000000000000000a5a5a5a597969908'
FLASH_START = 0x08000000
FLASH_END = 0x08100000


def digest(payload):
    """Return a stable digest of an immutable artifact snapshot."""
    return hashlib.sha256(payload).hexdigest()


def bounded_path(path, root=None, require_file=True):
    """Resolve a workspace file and reject escape through traversal or symlinks."""
    root = ROOT if root is None else root
    original = Path(path).absolute()
    # Windows sandbox may deny GetFinalPathNameByHandle used by strict resolve.
    # Explicit lstat rejects every reparse component rather than swallowing a
    # resolution failure that could otherwise hide a junction outside ROOT.
    for component in (original, *original.parents):
        metadata = component.lstat()
        if stat.S_ISLNK(metadata.st_mode) or getattr(metadata, 'st_file_attributes', 0) & 0x400:
            raise RuntimeError(f'Reparse/symlink artifact path rejected: {component}')
    resolved = original.resolve()
    if not resolved.is_relative_to(root.resolve()) or (require_file and not resolved.is_file()):
        raise RuntimeError(f'Artifact outside workspace or not a file: {path}')
    return resolved


def load_dependencies():
    """Load the reviewed helper modules from exact workspace paths, without running mains."""
    helper_root = ROOT / 'output/adrc/hardware'
    loaded = {}
    for name in ('audit_running_image_20261010', 'deploy_disabled_capture_20261010',
                 'deploy_reviewed_retry_20261010', 'recover_halted_deploy_20261010'):
        helper_path = bounded_path(helper_root / (name + '.py'))
        specification = importlib.util.spec_from_file_location(name, helper_path)
        module = importlib.util.module_from_spec(specification)
        sys.modules[name] = module
        specification.loader.exec_module(module)
        loaded[name] = module
    deployment = loaded['deploy_disabled_capture_20261010']
    if deployment.ROOT.resolve() != ROOT or deployment.PROBE_UID != PROBE_UID:
        raise RuntimeError('Workspace helper root/probe contract changed')
    return SimpleNamespace(
        verify_build=deployment.verify_build, IntelHex=deployment.IntelHex,
        ConnectHelper=deployment.ConnectHelper, Target=deployment.Target,
        FileProgrammer=deployment.FileProgrammer, read_flash=deployment.read_flash,
        reset_catch_verified=loaded['recover_halted_deploy_20261010'].reset_catch_verified)


def parse_image(payload, dependencies):
    """Parse an immutable HEX snapshot and require bounded, nonempty occupied flash."""
    image = dependencies.IntelHex(io.StringIO(payload.decode('ascii')))
    if not image.segments() or not all(FLASH_START <= start < end <= FLASH_END
                                      for start, end in image.segments()):
        raise RuntimeError('Invalid flash image range')
    return image


def prepare(manifest_path, old_path, dependencies):
    """Bind verified build, current image, and copied payloads to immutable bytes."""
    manifest_path = bounded_path(manifest_path)
    old_path = bounded_path(old_path)
    if old_path != OLD_IMAGE.resolve():
        raise RuntimeError('Expected current image must be the explicitly reviewed image path')
    old_payload = old_path.read_bytes()
    if digest(old_payload) != OLD_SHA256:
        raise RuntimeError('Expected current image hash differs from reviewed image')
    old_image = parse_image(old_payload, dependencies)
    manifest_payload = manifest_path.read_bytes()
    manifest = json.loads(manifest_payload.decode('utf-8-sig'))
    for item in manifest['source_hashes'] + manifest['evidence_hashes']:
        bounded_path(item['path'])
    image_path, elf_path, verified_image = dependencies.verify_build(manifest_path)
    image_path = bounded_path(image_path)
    elf_path = bounded_path(elf_path)
    bound_hashes = {bounded_path(item['path']): item['sha256'].lower()
                    for item in manifest['evidence_hashes']}
    image_payload = image_path.read_bytes()
    elf_payload = elf_path.read_bytes()
    if digest(image_payload) != bound_hashes.get(image_path) or digest(elf_payload) != bound_hashes.get(elf_path):
        raise RuntimeError('Image changed after build verification')
    image = parse_image(image_payload, dependencies)
    if image.segments() != verified_image.segments() or any(
            image[address] != verified_image[address] for address in image.addresses()):
        raise RuntimeError('HEX snapshot differs from verified build')
    if manifest_path.read_bytes() != manifest_payload or old_path.read_bytes() != old_payload:
        raise RuntimeError('Manifest or expected current image changed during verification')
    return SimpleNamespace(manifest_path=manifest_path, manifest_payload=manifest_payload,
        old_path=old_path, old_payload=old_payload, old_image=old_image,
        image_payload=image_payload, elf_payload=elf_payload, image=image)


def write_report(output, report):
    """Persist deployment evidence without asserting measured motor feedback."""
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


def deploy(prepared, dependencies, confirmed):
    """Attach, identify running image, halt, backup, program snapshots, verify, then boot."""
    if not confirmed:
        raise RuntimeError('Deployment requires --motor-power-off-confirmed')
    output_parent = ROOT / 'output/adrc/hardware'
    if not bounded_path(output_parent, require_file=False).is_dir():
        raise RuntimeError('Deployment output escapes workspace')
    output = output_parent / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '_poweroff_deploy')
    output.mkdir()
    report = {'motor_power': 'off=user_confirmation', 'motor_feedback_verified': False,
        'motion_commands_sent': 0, 'qualified': False, 'manifest': str(prepared.manifest_path),
        'old_image': str(prepared.old_path), 'old_hex_sha256': digest(prepared.old_payload),
        'new_hex_sha256': digest(prepared.image_payload), 'new_elf_sha256': digest(prepared.elf_payload),
        'new_verified_bytes': 0, 'old_verified_bytes': 0, 'backup': None,
        'normal_reset_requested': False, 'reset_completed': False,
        'reset_catch_requested': False, 'flash_readback_equal': False, 'error': None}
    session = None
    try:
        for name, payload in (('LCD-MIT-ADRC.hex', prepared.image_payload),
                              ('LCD-MIT-ADRC.axf', prepared.elf_payload),
                              ('expected_previous.hex', prepared.old_payload),
                              ('manifest.json', prepared.manifest_payload)):
            destination = output / name
            destination.write_bytes(payload)
            if destination.read_bytes() != payload:
                raise RuntimeError('Copied deployment artifact differs from verified snapshot')
        # Program this immutable snapshot of the verified copy, never reopen a path
        # after programming starts (FileProgrammer supports a file-like HEX input).
        copied_payload = (output / 'LCD-MIT-ADRC.hex').read_bytes()
        if copied_payload != prepared.image_payload:
            raise RuntimeError('Copied HEX changed before device access')
        session = dependencies.ConnectHelper.session_with_chosen_probe(
            blocking=False, unique_id=PROBE_UID, options={
                'target_override': 'stm32h723xx', 'connect_mode': 'attach',
                'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
                'frequency': 500000, 'cmsis_dap.limit_packets': True,
                'cmsis_dap.deferred_transfers': False, 'cache.enable_memory': False,
                'cache.read_code_from_elf': False})
        if session is None:
            raise RuntimeError('Expected probe unavailable')
        with session:
            target = session.target
            try:
                if session.probe.unique_id != PROBE_UID:
                    raise RuntimeError('Attached probe UID differs from expected probe')
                if target.get_state() != dependencies.Target.State.RUNNING or target.read32(0x5C001000) & 0xFFF != 0x483:
                    raise RuntimeError('Unexpected MCU state/identity')
                # All occupied old-image bytes must match while the core is running.
                for start, end in prepared.old_image.segments():
                    actual = dependencies.read_flash(target, start, end)
                    expected = bytes(prepared.old_image.tobinarray(start=start, size=end-start))
                    if actual != expected or target.get_state() != dependencies.Target.State.RUNNING:
                        raise RuntimeError('Running image does not match expected current image')
                    report['old_verified_bytes'] += end - start
                report['old_image_equal'] = True
                print('RUNNING_OLD_IMAGE_VERIFIED', report['old_verified_bytes'], flush=True)
                target.halt()
                if target.get_state() != dependencies.Target.State.HALTED:
                    raise RuntimeError('Core did not halt')
                previous = dependencies.read_flash(target, FLASH_START, FLASH_END)
                if len(previous) != FLASH_END - FLASH_START:
                    raise RuntimeError('Incomplete 1 MiB backup')
                for start, end in prepared.old_image.segments():
                    if previous[start-FLASH_START:end-FLASH_START] != bytes(
                            prepared.old_image.tobinarray(start=start, size=end-start)):
                        raise RuntimeError('Halted backup differs from running image')
                backup_path = output / 'previous_flash_1MiB.bin'
                backup_path.write_bytes(previous)
                if backup_path.read_bytes() != previous:
                    raise RuntimeError('Backup verification failed')
                report['backup'] = str(backup_path)
                report['backup_sha256'] = digest(previous)
                print('HALTED_FLASH_BACKUP_VERIFIED', report['backup_sha256'], flush=True)
                report['reset_catch_requested'] = True
                report['reset_catch'] = dependencies.reset_catch_verified(target, struct.unpack_from('<I', previous, 4)[0])
                dependencies.FileProgrammer(session, chip_erase='sector', smart_flash=False,
                    trust_crc=False, keep_unwritten=False).program(io.StringIO(copied_payload.decode('ascii')), file_format='hex')
                target.halt()
                if target.get_state() != dependencies.Target.State.HALTED:
                    raise RuntimeError('Core not halted after programming')
                for start, end in prepared.image.segments():
                    if dependencies.read_flash(target, start, end) != bytes(
                            prepared.image.tobinarray(start=start, size=end-start)):
                        raise RuntimeError('New flash readback mismatch')
                    report['new_verified_bytes'] += end - start
                report['flash_readback_equal'] = True
                print('NEW_FLASH_READBACK_EQUAL', report['new_verified_bytes'], flush=True)
                write_report(output, report)
                report['normal_reset_requested'] = True
                target.reset(reset_type=dependencies.Target.ResetType.SYSRESETREQ)
                report['reset_completed'] = True
                report['state_after'] = str(target.get_state())
                if target.get_state() != dependencies.Target.State.RUNNING:
                    raise RuntimeError('Verified image did not report RUNNING after reset')
                print('VERIFIED_IMAGE_RESET_RUNNING_MOTOR_POWER_OFF', flush=True)
            except BaseException:
                # Even a programmer exception may leave its RAM algorithm running.
                # Halt before disconnect; there is no failure-reset or resume path.
                try:
                    target.halt()
                    report['failure_core_state'] = str(target.get_state())
                    if target.get_state() != dependencies.Target.State.HALTED:
                        report['failure_halt_error'] = 'Core did not report HALTED'
                except BaseException as halt_error:
                    report['failure_halt_error'] = str(halt_error)
                raise
    except BaseException as error:
        report['error'] = str(error)
        # Session entry/exit errors are outside the inner target transaction.
        # Make the same best-effort halt without issuing a reset or resume.
        if session is not None and 'failure_core_state' not in report:
            try:
                failure_target = session.target
                if failure_target is not None:
                    failure_target.halt()
                    report['failure_core_state'] = str(failure_target.get_state())
                    if failure_target.get_state() != dependencies.Target.State.HALTED:
                        report['failure_halt_error'] = 'Core did not report HALTED'
            except BaseException as halt_error:
                report['failure_halt_error'] = str(halt_error)
        raise
    finally:
        write_report(output, report)
        print(output, flush=True)
    return report


def main(argv=None, dependencies=None):
    """Enforce explicit user confirmation before loading any deployment connection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--expected-current-image', type=Path, required=True)
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    parser.add_argument('--deploy', action='store_true')
    arguments = parser.parse_args(argv)
    if arguments.deploy and not arguments.motor_power_off_confirmed:
        parser.error('--deploy requires --motor-power-off-confirmed')
    dependencies = dependencies or load_dependencies()
    prepared = prepare(arguments.manifest, arguments.expected_current_image, dependencies)
    print('BUILD_VERIFIED_NO_HARDWARE_ACCESS', digest(prepared.image_payload), flush=True)
    if arguments.deploy:
        deploy(prepared, dependencies, arguments.motor_power_off_confirmed)


if __name__ == '__main__':
    main()
