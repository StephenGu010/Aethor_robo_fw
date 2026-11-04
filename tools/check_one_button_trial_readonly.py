"""Read the retained POS-load report without changing CPU or motor state.

Default operation validates pinned image/linker field offsets only. --read uses
the same attach-only, no-halt/reset/resume settings as the LCD model reader.
"""
import argparse
import json
import re
import struct
import time

import check_engineering_lcd_readonly as lcd
import deploy_one_button_poweroff as release


def main():
    """Resolve exact compiled report offsets, then optionally capture one coherent retained result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read', action='store_true')
    arguments = parser.parse_args()
    release.prepare()
    offsets = (release.BUILD / 'fieldoffsets.txt').read_text()
    map_text = (release.BUILD / 'LCD-POS-Load-Test.map').read_text()

    def field(name):
        """Accept only one compiler-emitted field offset."""
        return lcd.unique_number(offsets, r'^\|' + re.escape(name) + r'\|\s+EQU\s+(0x[0-9a-fA-F]+|0)\s')

    model = lcd.unique_number(map_text, r'^\s+task_model\s+(0x[0-9a-fA-F]+)\s+Data\s')
    snapshot = model + field('DebugUiModel.snapshot')
    address = snapshot + field('DebugUiSnapshot.pos_load')
    size = lcd.unique_number(offsets, r'^; Structure, DebugUiPosLoadReport , Size (0x[0-9a-fA-F]+) bytes')
    fields = {'identity.origin': 'I', 'identity.epoch': 'I', 'identity.request_id': 'I', 'stage': 'I', 'started_at_us': 'Q',
        'elapsed_ms': 'I', 'duration_ms': 'I', 'samples': 'I', 'disturbances': 'I', 'recoveries': 'I',
        'target_position_rad': 'f', 'error_rad': 'f', 'peak_error_rad': 'f', 'peak_torque_nm': 'f',
        'peak_mos_c': 'f', 'peak_rotor_c': 'f', 'motor_id': 'B', 'active': 'B', 'complete': 'B',
        'stopped': 'B', 'disabled_confirmed': 'B', 'recovering': 'B', 'recovery_observed': 'B'}
    resolved = {name: field('DebugUiPosLoadReport.' + name) for name in fields}
    if any(resolved[name] + struct.calcsize('<' + kind) > size for name, kind in fields.items()):
        raise ValueError('Compiled field outside report')
    print('POS_REPORT_OFFSETS_VERIFIED', hex(address), size, flush=True)
    if not arguments.read:
        return
    shared = release.guarded.shared
    hardware = shared.load_hardware()
    from pyocd.core.helpers import ConnectHelper
    matching = [port for port in hardware.ports() if port.vid == 0x0483 and port.pid == 0x5740 and
                port.serial_number == shared.USB_SERIAL]
    if len(matching) != 1:
        raise RuntimeError('Exact USB identity unavailable')
    session = ConnectHelper.session_with_chosen_probe(unique_id=shared.PROBE_UID, return_first=False,
        blocking=False, options=dict(shared.SESSION_OPTIONS, frequency=100000))
    if session is None:
        raise RuntimeError('Matching probe unavailable')
    with session:
        target = session.target
        if target.get_state() != hardware.Target.State.RUNNING:
            raise RuntimeError('CPU not running; no recovery attempted')
        pairing = {}
        shared.verify_usb_swd_pair(target, pairing)
        boot = target.read32(snapshot + field('DebugUiSnapshot.boot_id'))
        first = bytes(target.read_memory_block8(address, size))
        second = bytes(target.read_memory_block8(address, size))
        if first != second or boot != target.read32(snapshot + field('DebugUiSnapshot.boot_id')):
            raise RuntimeError('Report changed during capture; no retry or control attempted')
        report = {name: struct.unpack_from('<' + kind, first, resolved[name])[0] for name, kind in fields.items()}
        running = target.get_state() == hardware.Target.State.RUNNING
        if not running:
            raise RuntimeError('CPU no longer running; no state-changing recovery attempted')
    output = release.ROOT / 'output/idle-feedback' / ('one-button-trial-readonly-' + str(time.time_ns()) + '.json')
    output.write_text(json.dumps({'access': 'attach_read_only_no_halt_reset_resume',
        'hex_sha256': release.EXPECTED_NEW, 'boot': boot, 'pairing': pairing, 'report': report,
        'coreRemainedRunning': running, 'motion_commands_sent_by_reader': 0}, indent=2), encoding='utf-8')
    print('POS_REPORT_READONLY_RECEIPT', output, flush=True)


if __name__ == '__main__':
    main()
