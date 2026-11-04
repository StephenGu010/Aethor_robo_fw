"""Deploy the frozen LCD idle-feedback candidate only with explicit motor-power-off confirmation.

Reuse the existing reviewed power-off backup/program/readback transaction. No serial
motor commands, automatic retries, mode writes or motion are available in this adapter.
"""
import argparse
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    'D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit/output/adrc/hardware/python_deps_20261010',
    'D:/download/TCG/output/can_id_commissioning_20260909/python_deps',
]
import deploy_adrc_poweroff as poweroff
import deploy_single_static_test as shared

EXPECTED_NEW = '7c18c51647cd07aa936db9b2114262629f557c3642cda287e5578fa398c83aeb'
EXPECTED_OLD = '3281d1a80777d95a4b155ff1b212623cdac72668d7b54973832e76bc2dfb0fdb'
OLD_PATH = Path('D:/download/TCG/Aethor_robo_fw/.worktrees/repair-401b-static-pos/output/repair_20261026/'
    'static_window_release_20261026T073102Z/build_LCD-POS-Load-Test_final/LCD-POS-Load-Test.hex')


class PairedTarget:
    """Delegate passive access, but refuse all state-changing calls until USB/SWD identity is proven."""

    def __init__(self, target, pairing):
        """Retain the actual target and a shared explicit identity receipt."""
        self.target = target
        self.pairing = pairing

    def __getattr__(self, name):
        """Forward read-only operations and target metadata used by the existing transaction."""
        return getattr(self.target, name)

    def halt(self):
        """Prevent the existing helper's failure cleanup from halting an unpaired target."""
        shared.require(self.pairing.get('usb_swd_pair_verified') is True, 'Unpaired target halt refused')
        return self.target.halt()

    def reset(self, **kwargs):
        """Allow the normal boot only for the identity verified by this connection."""
        shared.require(self.pairing.get('usb_swd_pair_verified') is True, 'Unpaired target reset refused')
        return self.target.reset(**kwargs)

    def reset_and_halt(self, **kwargs):
        """Keep reset catch subject to the same identity condition as ordinary halt."""
        shared.require(self.pairing.get('usb_swd_pair_verified') is True, 'Unpaired target reset-catch refused')
        return self.target.reset_and_halt(**kwargs)


class PairedSession:
    """Preserve pyOCD context management while guarding target access in every helper failure path."""

    def __init__(self, session, pairing):
        """Bind one session to one connection-local pairing receipt."""
        self.session = session
        self.pairing = pairing

    def __getattr__(self, name):
        """Forward session metadata without changing its disconnect policy."""
        return getattr(self.session, name)

    @property
    def target(self):
        """Guard even target access during a failed context-manager entry."""
        target = self.session.target
        return PairedTarget(target, self.pairing) if target is not None else None

    def __enter__(self):
        """Enter the real attach-only session and expose only the guarded facade."""
        self.session.__enter__()
        return self

    def __exit__(self, *args):
        """Use the original session cleanup with resume_on_disconnect still disabled."""
        return self.session.__exit__(*args)


def prepare():
    """Verify the full source snapshot, explicit release hashes and exact HEX/ELF coverage offline."""
    from intelhex import IntelHex
    manifest_path = ROOT / 'output/idle-feedback/release.json'
    manifest_payload = manifest_path.read_bytes()
    manifest = json.loads(manifest_payload)
    baseline_payload = (ROOT / 'baseline.json').read_bytes()
    shared.require(shared.digest(baseline_payload) == manifest['baseline_sha256'], 'Baseline changed')
    expected = {item['path']: item['sha256'] for item in json.loads(baseline_payload)['files']}
    expected.update({item['path']: item['sha256'] for item in manifest['files'] + manifest['artifacts']})
    for relative, expected_hash in expected.items():
        shared.require(shared.digest(shared.bounded_path(ROOT / relative, ROOT).read_bytes()) == expected_hash,
            'Frozen source/artifact changed: ' + relative)
    image_payload = (ROOT / manifest['artifacts'][0]['path']).read_bytes()
    elf_payload = (ROOT / manifest['artifacts'][1]['path']).read_bytes()
    old_payload = OLD_PATH.read_bytes()
    shared.require(shared.digest(image_payload) == EXPECTED_NEW, 'Wrong candidate image')
    shared.require(shared.digest(old_payload) == EXPECTED_OLD, 'Wrong expected prior image')
    segments = shared.verify_image_pair(image_payload, elf_payload)
    shared.require(manifest['target'] == 'LCD-POS-Load-Test' and manifest['new_test_groups'] == 8,
        'Wrong offline release')
    print('OFFLINE_INPUTS_VERIFIED', EXPECTED_NEW, sum(len(data) for address, data in segments), flush=True)
    return SimpleNamespace(manifest_path=manifest_path, manifest_payload=manifest_payload,
        old_path=OLD_PATH, old_payload=old_payload, image_payload=image_payload, elf_payload=elf_payload,
        old_image=IntelHex(io.StringIO(old_payload.decode('ascii'))),
        image=IntelHex(io.StringIO(image_payload.decode('ascii'))))


def deploy(prepared):
    """Bind USB/SWD identity before any halt and retain the original transaction's failure policy."""
    hardware = shared.load_hardware()
    from pyocd.core.helpers import ConnectHelper
    matches = [port for port in hardware.ports() if port.vid == 0x0483 and port.pid == 0x5740
        and port.serial_number == shared.USB_SERIAL]
    shared.require(len(matches) == 1, 'Exact USB identity unavailable')
    pairing = {}

    def checked_read_flash(target, start, end):
        """Pair the MCU UID to the known USB serial before reading or subsequently halting."""
        if pairing.get('usb_swd_pair_verified') is not True:
            shared.verify_usb_swd_pair(target, pairing)
            print('USB_SWD_PAIR_VERIFIED', pairing['mcu_derived_usb_serial'], flush=True)
        return shared.read_flash(target, start, end)

    def session_with_chosen_probe(**kwargs):
        """Use the recorded conservative SWD clock with no reset or automatic unlock on attach."""
        kwargs['options']['frequency'] = 100000
        session = ConnectHelper.session_with_chosen_probe(**kwargs)
        return PairedSession(session, pairing) if session is not None else None

    def programmer(session, **kwargs):
        """Expose the underlying programming session only after explicit identity verification."""
        shared.require(pairing.get('usb_swd_pair_verified') is True, 'Unpaired programming refused')
        return hardware.FileProgrammer(session.session, **kwargs)

    def reset_catch(target, reset_vector):
        """Reuse the verified old-vector halt contract without altering its checks."""
        return shared.reset_catch_verified(target, reset_vector, hardware)

    dependencies = SimpleNamespace(ConnectHelper=SimpleNamespace(session_with_chosen_probe=session_with_chosen_probe),
        Target=hardware.Target, FileProgrammer=programmer,
        read_flash=checked_read_flash, reset_catch_verified=reset_catch)
    (ROOT / 'output/adrc/hardware').mkdir(parents=True, exist_ok=True)
    report = poweroff.deploy(prepared, dependencies, True)
    report.update(target='LCD-POS-Load-Test', source_directory=str(ROOT), usb_swd_pair=pairing,
        swd_frequency_hz=100000, supported_unloaded='user_confirmation',
        legacy_copy_filenames='LCD-MIT-ADRC filenames belong to the reused helper; payload is pinned by SHA256',
        adapter_sha256=shared.digest(Path(__file__).read_bytes()))
    output = Path(report['backup']).parent
    poweroff.write_report(output, report)
    (ROOT / 'output/idle-feedback/latest-deployment.json').write_text(
        json.dumps({'report': str(output / 'report.json'), 'new_hex_sha256': EXPECTED_NEW}, indent=2))
    print('DEPLOYMENT_VERIFIED', output, flush=True)


def main():
    """Default to offline preparation; physical access requires both explicit deployment flags."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--motor-power-off-confirmed', action='store_true')
    args = parser.parse_args()
    if args.deploy and not args.motor_power_off_confirmed:
        parser.error('Deployment requires confirmed motor power off and independent support')
    prepared = prepare()
    if args.deploy:
        deploy(prepared)


if __name__ == '__main__':
    main()
