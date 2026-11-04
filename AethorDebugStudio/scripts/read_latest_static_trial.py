"""@file Decode the retained ID7 trial with exact compiler offsets; optional SWD reads never halt/reset/write."""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import deploy_torque_envelope as release

ROOT = Path('D:/download/TCG/AethorDebugStudio')
OFFSETS = ROOT / 'artifacts/latest-user-trial-fieldoffsets-20261029.txt'
MAP = release.CANDIDATE / 'output/torque-envelope/keil-final/LCD-POS-Load-Test.map'


def unique(text, pattern):
    """Refuse missing or contradictory compiled addresses and field definitions."""
    values = {int(value, 0) for value in re.findall(pattern, text, re.MULTILINE)}
    if len(values) != 1:
        raise ValueError('Ambiguous compiler evidence: ' + pattern)
    return values.pop()


def main():
    """Verify the pinned image/layout and optionally capture one stable retained trial, with no retries."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read', action='store_true')
    parser.add_argument('--transaction', type=int, required=True)
    parser.add_argument('--boot', type=int, required=True)
    args = parser.parse_args()
    prepared = release.prepare()
    offsets = OFFSETS.read_text(encoding='utf-8')
    map_text = MAP.read_text(encoding='utf-8')
    compiled_fields = {}
    for match in re.finditer(r'^\|((?:AethorAppDebugUiState|AethorRemoteUi|DebugUiStaticLoadReport|DebugUiStaticTraceHeader|DebugUiStaticTraceRecord|DebugUiStaticTraceAxis|ProtocolEngine)\.[^\[\]|\r\n]+)\|\s+EQU\s+(0x[0-9a-fA-F]+|0)\s', offsets, re.MULTILINE):
        compiled_fields.setdefault(match.group(1), set()).add(int(match.group(2), 0))

    def field(name):
        """Use only one exact compiler-emitted field offset."""
        matches = compiled_fields.get(name, set())
        if len(matches) != 1:
            raise ValueError('Ambiguous or missing compiled field: ' + name)
        return next(iter(matches))

    def symbol(name):
        """Resolve a RAM symbol from the exact linked map."""
        return unique(map_text, r'^\s+' + re.escape(name) + r'\s+(0x[0-9a-fA-F]+)\s+Data\s')

    def size(name):
        """Require the compiled structure size rather than assuming host alignment."""
        return unique(offsets, r'^; Structure, ' + re.escape(name) + r' , Size (0x[0-9a-fA-F]+) bytes')

    def value(payload, type_name, member, kind):
        """Decode one in-bounds little-endian scalar from compiler evidence."""
        offset = field(type_name + '.' + member)
        if offset + struct.calcsize('<' + kind) > len(payload):
            raise ValueError('Field outside captured object: ' + member)
        return struct.unpack_from('<' + kind, payload, offset)[0]

    def decode_record(payload):
        """Retain native trace units, raw CAN bytes and original first-fault identity."""
        kinds = {'timestamp_us':'Q', 'observed_us':'Q', 'serial':'I', 'feedback_generation':'I',
            'epoch':'I','request_id':'I','sequence':'I','fault':'I','target_rad':'f','position_rad':'f',
            'velocity_rad_s':'f','torque_nm':'f','kp':'f','kd':'f','torque_ff_nm':'f','can_id':'H',
            'sof':'H','event':'B','phase':'B','motor_id':'B','driver_state':'B','length':'B','sof_valid':'B'}
        result = {name:value(payload, 'DebugUiStaticTraceRecord', name, kind) for name,kind in kinds.items()}
        raw_offset = field('DebugUiStaticTraceRecord.raw')
        result['raw'] = payload[raw_offset:raw_offset + 8].hex(' ')
        return result

    report_address = symbol('application_debug_ui') + field('AethorAppDebugUiState.static_load')
    remote_address, trace_address = symbol('application_remote_ui'), symbol('application_static_trace')
    boot_address = symbol('application_protocol_engine') + field('ProtocolEngine.boot_id')
    report_size, header_size, record_size = size('DebugUiStaticLoadReport'), size('DebugUiStaticTraceHeader'), size('DebugUiStaticTraceRecord')
    assert header_size == 832 and record_size == 96
    print('COMPILED_LAYOUT_VERIFIED', hex(report_address), hex(trace_address), report_size, flush=True)
    if not args.read:
        return
    shared = release.guarded.shared
    hardware = shared.load_hardware()
    from pyocd.core.helpers import ConnectHelper
    matches = [port for port in hardware.ports() if port.vid == 0x0483 and port.pid == 0x5740 and port.serial_number == shared.USB_SERIAL]
    if len(matches) != 1:
        raise RuntimeError('Exact USB identity unavailable; no recovery attempted')
    session = ConnectHelper.session_with_chosen_probe(unique_id=shared.PROBE_UID, return_first=False,
        blocking=False, options=dict(shared.SESSION_OPTIONS, frequency=100000))
    if session is None:
        raise RuntimeError('Expected probe unavailable')
    with session:
        target = session.target
        if target.get_state() != hardware.Target.State.RUNNING:
            raise RuntimeError('CPU not running; no halt/resume/reset attempted')
        pairing = {}
        shared.verify_usb_swd_pair(target, pairing)
        expected_vectors = bytes(prepared.image.tobinarray(start=0x08000000, size=256))
        if bytes(target.read_memory_block8(0x08000000, 256)) != expected_vectors:
            raise RuntimeError('Running vector fingerprint differs; layout not used')
        if target.read32(boot_address) != args.boot:
            raise RuntimeError('Boot changed before capture')
        remote_names = ['external_id','internal_id','terminal_id','owns','active']
        remote = {name:target.read32(remote_address + field('AethorRemoteUi.' + name)) if name.endswith('_id')
            else target.read8(remote_address + field('AethorRemoteUi.' + name)) for name in remote_names}
        if remote['terminal_id'] != args.transaction or remote['external_id'] != args.transaction or remote['active']:
            raise RuntimeError('Retained transaction changed or still active')
        report_payload = bytes(target.read_memory_block8(report_address, report_size))
        header_payload = bytes(target.read_memory_block8(trace_address, header_size))
        if value(header_payload, 'DebugUiStaticTraceHeader', 'active', 'I') or not value(header_payload, 'DebugUiStaticTraceHeader', 'frozen', 'I'):
            raise RuntimeError('Trace is not frozen; no retry attempted')
        record_count = value(header_payload, 'DebugUiStaticTraceHeader', 'count', 'I')
        if record_count > 128:
            raise RuntimeError('Invalid trace count')
        record_payload = bytes(target.read_memory_block8(trace_address + header_size, record_count * record_size))
        if report_payload != bytes(target.read_memory_block8(report_address, report_size)) or header_payload != bytes(target.read_memory_block8(trace_address, header_size)):
            raise RuntimeError('Retained evidence changed during capture')
        if target.read32(boot_address) != args.boot or target.read32(remote_address + field('AethorRemoteUi.terminal_id')) != args.transaction:
            raise RuntimeError('Boot or retained result changed during capture')
        running = target.get_state() == hardware.Target.State.RUNNING
        if not running:
            raise RuntimeError('CPU state changed; no recovery attempted')
    scalar_kinds = {'identity.epoch':'I','identity.request_id':'I','started_at_us':'Q','elapsed_ms':'I','duration_ms':'I',
        'target_torque_ff_nm':'f','requested_torque_ff_nm':'f','transmitted_torque_ff_nm':'f','takeover_elapsed_ms':'I',
        'takeover_active':'B','takeover_complete':'B','phase':'B','first_exit_phase':'B','first_exit_reason':'B',
        'first_exit_stage':'B','first_exit_code':'B','first_exit_error':'B','first_exit_latched':'B','active':'B',
        'complete':'B','stopped':'B','disabled_confirmed':'B','operation':'B','motor_mask':'B','valid_mask':'B'}
    report = {name:value(report_payload,'DebugUiStaticLoadReport',name,kind) for name,kind in scalar_kinds.items()}
    for name in ['sample_generation','samples','target_position_rad','peak_error_rad','peak_torque_nm','peak_mos_c','peak_rotor_c']:
        report[name] = struct.unpack_from('<7' + ('I' if name in ['sample_generation','samples'] else 'f'),
            report_payload, field('DebugUiStaticLoadReport.' + name))
    if report['identity.request_id'] != remote['internal_id']:
        raise RuntimeError('Report identity differs from retained transaction')
    header_names = ['generation','count','total','overflow','gaps','frozen','first_fault_valid','terminal_valid','epoch','request_id','motor_mask','cleanup_outcome']
    header = {name:value(header_payload,'DebugUiStaticTraceHeader',name,'I') for name in header_names}
    if header['request_id'] != remote['internal_id']:
        raise RuntimeError('Trace identity differs from retained transaction')
    header['started_us'] = value(header_payload,'DebugUiStaticTraceHeader','started_us','Q')
    axis_offset = field('DebugUiStaticTraceHeader.axes') + 6 * size('DebugUiStaticTraceAxis')
    axis_payload = header_payload[axis_offset:axis_offset+size('DebugUiStaticTraceAxis')]
    axis = {name:value(axis_payload,'DebugUiStaticTraceAxis',name,'f') for name in ['target_rad','position_ratio','kp','kd','torque_ff_nm','range_torque_nm']}
    first_fault_offset, terminal_offset = field('DebugUiStaticTraceHeader.first_fault'), field('DebugUiStaticTraceHeader.terminal')
    first_fault = decode_record(header_payload[first_fault_offset:first_fault_offset+record_size])
    terminal = decode_record(header_payload[terminal_offset:terminal_offset+record_size])
    records = sorted([decode_record(record_payload[index*record_size:(index+1)*record_size]) for index in range(record_count)],key=lambda record:record['serial'])
    summary = {'hold_elapsed_ms':report['elapsed_ms'],'hold_planned_ms':report['duration_ms'],
        'peak_output_error_deg':math.degrees(report['peak_error_rad'][6]),
        'peak_flange_error_deg_at_project_ratio':math.degrees(report['peak_error_rad'][6]) / (44.8 / 19.2032),
        'peak_output_torque_nm':report['peak_torque_nm'][6], 'samples':report['samples'][6],
        'mos_peak_c':report['peak_mos_c'][6],'winding_peak_c':report['peak_rotor_c'][6],
        'first_fault_output_error_deg':math.degrees(abs(first_fault['position_rad']-first_fault['target_rad'])/axis['position_ratio']),
        'kp_output':axis['kp']*axis['position_ratio'],'kd_output':axis['kd']}
    evidence = {'access':'attach_read_only_no_halt_reset_resume_write','boot':args.boot,'transaction':args.transaction,
        'pairing':pairing,'vector_fingerprint_bytes':256,'core_remained_running':running,'motor_commands_sent':0,
        'remote':remote,'report':report,'trace_header':header,'axis7':axis,'first_fault':first_fault,'terminal':terminal,
        'records':records,'summary':summary}
    output = ROOT / ('artifacts/latest-user-trial-retained-' + str(args.transaction) + '-20261029.json')
    output.write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    print('READONLY_TRIAL_RECEIPT',output,flush=True)
    print(json.dumps({'summary':summary,'report':report,'first_fault':first_fault,'trace_header':header,'axis7':axis},indent=2),flush=True)


if __name__ == '__main__':
    main()
