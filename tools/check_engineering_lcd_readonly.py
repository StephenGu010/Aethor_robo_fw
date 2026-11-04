"""Read the running LCD model using offsets from the frozen AXF; never halt, reset or write RAM.

The default verifies local symbol evidence. --read enables one bounded passive
SWD observation; it does not change the LCD page or send motor commands.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import time

import deploy_engineering_remote_poweroff as release

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'output/idle-feedback/engineering-remote-final-20261027'


def unique_number(text, pattern):
    """Reject missing or inconsistent DWARF/linker definitions rather than guessing offsets."""
    values = {int(value, 0) for value in re.findall(pattern, text, re.MULTILINE)}
    if len(values) != 1:
        raise ValueError('Ambiguous symbol: ' + pattern)
    return values.pop()


def main():
    """Verify the release, resolve exact model fields and optionally read five running snapshots."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read', action='store_true')
    arguments = parser.parse_args()
    release.prepare()
    offsets_text = (EVIDENCE / 'fieldoffsets.txt').read_text(errors='strict')
    map_text = (EVIDENCE / 'LCD-POS-Load-Test.map').read_text(errors='strict')

    def field(name):
        """Resolve a field only from the retained compiler output."""
        return unique_number(offsets_text, r'^\|' + re.escape(name) + r'\|\s+EQU\s+(0x[0-9a-fA-F]+|0)\s')

    def symbol(name):
        """Resolve a retained RAM variable from the linked target map."""
        return unique_number(map_text, r'^\s+' + re.escape(name) + r'\s+(0x[0-9a-fA-F]+)\s+Data\s')

    motor_size = unique_number(offsets_text, r'^; Structure, DebugUiMotorView , Size (0x[0-9a-fA-F]+) bytes')
    model_address, view_address = symbol('task_model'), symbol('task_view')
    motor_address = model_address + field('DebugUiModel.snapshot') + field('DebugUiSnapshot.motors') + 6 * motor_size
    required = ['register_position', 'register_age_ms', 'register_seen_mask',
        'register_estimated_native_rad_s', 'register_estimated_velocity_valid',
        'register_position_contract', 'single_static_native_per_output', 'feedback_valid']
    motor_fields = {name: field('DebugUiMotorView.' + name) for name in required}
    model_fields = {name: field('DebugUiModel.' + name) for name in ['snapshot_seen', 'display_valid']}
    title_offset = field('DebugUiViewPage.title_text')
    rows_offset = field('DebugUiViewPage.row_text')
    print('LCD_OFFSETS_VERIFIED', hex(model_address), motor_size, flush=True)
    if not arguments.read:
        return
    hardware = release.guarded.shared.load_hardware()
    from pyocd.core.helpers import ConnectHelper
    shared = release.guarded.shared
    matching_ports = [port for port in hardware.ports() if port.vid == 0x0483 and port.pid == 0x5740
        and port.serial_number == shared.USB_SERIAL]
    if len(matching_ports) != 1:
        raise RuntimeError('Exact USB identity unavailable')
    options = dict(shared.SESSION_OPTIONS, frequency=100000)
    session = ConnectHelper.session_with_chosen_probe(unique_id=shared.PROBE_UID,
        return_first=False, blocking=False, options=options)
    if session is None:
        raise RuntimeError('Matching probe unavailable')
    samples = []
    with session:
        target = session.target
        if target.get_state() != hardware.Target.State.RUNNING:
            raise RuntimeError('CPU is not running; no resume/reset attempted')
        pairing = {}
        shared.verify_usb_swd_pair(target, pairing)
        for _ in range(5):
            payload = bytes(target.read_memory_block8(motor_address, motor_size))
            sample = {'registerPosition': list(struct.unpack_from('<2f', payload, motor_fields['register_position'])),
                'registerAgeMs': list(struct.unpack_from('<2I', payload, motor_fields['register_age_ms'])),
                'estimatedNativeRadS': struct.unpack_from('<f', payload, motor_fields['register_estimated_native_rad_s'])[0],
                'nativePerOutput': struct.unpack_from('<f', payload, motor_fields['single_static_native_per_output'])[0]}
            for name in ['register_seen_mask', 'register_estimated_velocity_valid', 'register_position_contract', 'feedback_valid']:
                sample[name] = payload[motor_fields[name]]
            for name, offset in model_fields.items():
                sample[name] = target.read8(model_address + offset)
            sample['graphics_running'] = target.read8(symbol('graphics_running'))
            sample['last_view_ms'] = target.read32(symbol('last_view_ms'))
            samples.append(sample)
            time.sleep(0.2)
        title = bytes(target.read_memory_block8(view_address + title_offset, 80)).split(b'\0')[0].decode('utf-8')
        rows = bytes(target.read_memory_block8(view_address + rows_offset, 7 * 160))
        display_rows = [rows[index * 160:(index + 1) * 160].split(b'\0')[0].decode('utf-8') for index in range(7)]
        running = target.get_state() == hardware.Target.State.RUNNING
        if not running:
            raise RuntimeError('CPU no longer running; no state-changing recovery attempted')
    output = ROOT / 'output/idle-feedback' / ('engineering-lcd-readonly-' + str(time.time_ns()) + '.json')
    output.write_text(json.dumps({'access': 'attach_read_only_no_halt_reset_resume', 'hex_sha256': release.EXPECTED_NEW,
        'pairing': pairing, 'samples': samples, 'title': title, 'displayRows': display_rows,
        'coreRemainedRunning': running, 'motion_commands_sent': 0}, ensure_ascii=False, indent=2), encoding='utf-8')
    print('LCD_READONLY_RECEIPT', output, flush=True)


if __name__ == '__main__':
    main()
