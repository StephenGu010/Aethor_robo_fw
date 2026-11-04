"""@file Capture a stable retained POS report from the pinned running image with attach/read only, no halt or motor commands."""
import json
import math
from pathlib import Path
import re
import struct
from functools import lru_cache
import deploy_session_recovery as release

ROOT=Path('D:/download/TCG/AethorDebugStudio')


def main():
    """Verify exact source/layout and device identity, then read the already-ended transaction once without recovery."""
    prepared=release.prepare()
    binding=json.loads((ROOT/'artifacts/pos-latest-binding-20261029.json').read_text(encoding='utf-8-sig'))
    expected_boot=int(binding['capabilities']['fields']['boot'])
    expected_id=int(binding['capabilities']['fields']['terminal_id'])
    offsets=(ROOT/'artifacts/pos-latest-fieldoffsets-1526a4e7-20261029.txt').read_text()
    map_text=(release.CANDIDATE/'output/session-recovery/keil-final/LCD-POS-Load-Test.map').read_text()

    @lru_cache(maxsize=None)
    def field(name):
        """Reject inconsistent compiler offsets rather than assume a previous release layout."""
        values={int(value,0) for value in re.findall(r'^\|'+re.escape(name)+r'\|\s+EQU\s+(0x[0-9a-fA-F]+|0)\s',offsets,re.M)}
        if len(values)!=1:raise ValueError('Missing/ambiguous compiled field: '+name)
        return values.pop()

    def symbol(name):
        """Resolve one exact linked RAM object."""
        matches={int(value,16) for value in re.findall(r'^\s+'+name+r'\s+(0x[0-9a-fA-F]+)\s+Data\s',map_text,re.M)}
        if len(matches)!=1:raise ValueError('Missing/ambiguous symbol: '+name)
        return matches.pop()

    def decode(payload,typename,kinds):
        """Decode little-endian scalar fields using current ARM compiler offsets."""
        return {name:struct.unpack_from('<'+kind,payload,field(typename+'.'+name))[0] for name,kind in kinds.items()}

    report_address=symbol('application_debug_ui')+field('AethorAppDebugUiState.pos_load')
    remote_address=symbol('application_remote_ui')
    boot_address=symbol('application_protocol_engine')+field('ProtocolEngine.boot_id')
    observation_address=symbol('application_debug_ui')+field('AethorAppDebugUiState.pos_load_observation_us')
    report_sizes={int(size,16) for size in re.findall(r'^; Structure, DebugUiPosLoadReport , Size (0x[0-9a-fA-F]+) bytes',offsets,re.M)}
    if len(report_sizes)!=1:raise ValueError('Invalid report size evidence')
    report_size=report_sizes.pop()
    remote_kinds={'external_id':'I','internal_id':'I','terminal_id':'I','owner_epoch':'I','terminal_epoch':'I',
        'terminal_session':'I','terminal_operation':'B','terminal_disabled':'B','terminal_code':'B','owns':'B','active':'B'}
    hardware=release.guarded.shared.load_hardware()
    from pyocd.core.helpers import ConnectHelper
    shared=release.guarded.shared
    ports=[port for port in hardware.ports() if port.vid==0x0483 and port.pid==0x5740 and port.serial_number==shared.USB_SERIAL]
    if len(ports)!=1:raise RuntimeError('Exact USB identity missing; no retry')
    session=ConnectHelper.session_with_chosen_probe(unique_id=shared.PROBE_UID,return_first=False,blocking=False,
        options=dict(shared.SESSION_OPTIONS,frequency=100000))
    if session is None:raise RuntimeError('Recorded probe unavailable; no retry')
    with session:
        target=session.target
        if target.get_state()!=hardware.Target.State.RUNNING:raise RuntimeError('CPU not running; no reset/halt/resume attempted')
        pairing={};shared.verify_usb_swd_pair(target,pairing)
        if bytes(target.read_memory_block8(0x08000000,256))!=bytes(prepared.image.tobinarray(start=0x08000000,size=256)):
            raise RuntimeError('Running vector fingerprint differs; layout not used')
        if target.read32(boot_address)!=expected_boot:raise RuntimeError('Boot changed before capture')
        remote_payload=bytes(target.read_memory_block8(remote_address,2368))
        remote=decode(remote_payload,'AethorRemoteUi',remote_kinds)
        if remote['external_id']!=expected_id or remote['terminal_id']!=expected_id or remote['active']:
            raise RuntimeError('Retained transaction changed or is active; no retry')
        report_payload=bytes(target.read_memory_block8(report_address,report_size))
        observation=bytes(target.read_memory_block8(observation_address,8))
        ledger_payload=bytes(target.read_memory_block8(symbol('application_torque_envelope'),40))
        if report_payload!=bytes(target.read_memory_block8(report_address,report_size)) or observation!=bytes(target.read_memory_block8(observation_address,8)):
            raise RuntimeError('POS report changed while reading; no retry')
        if target.read32(boot_address)!=expected_boot or target.read32(remote_address+field('AethorRemoteUi.terminal_id'))!=expected_id:
            raise RuntimeError('Boot or terminal changed while reading')
        if target.get_state()!=hardware.Target.State.RUNNING:raise RuntimeError('CPU state changed; no recovery attempted')
    kinds={'identity.epoch':'I','identity.request_id':'I','stage':'B','started_at_us':'Q','elapsed_ms':'I',
        'duration_ms':'I','samples':'I','disturbances':'I','recoveries':'I','target_position_rad':'f','error_rad':'f',
        'peak_error_rad':'f','peak_torque_nm':'f','peak_mos_c':'f','peak_rotor_c':'f','motor_id':'B','active':'B',
        'complete':'B','stopped':'B','disabled_confirmed':'B','recovering':'B','recovery_observed':'B'}
    report=decode(report_payload,'DebugUiPosLoadReport',kinds)
    if report['identity.request_id']!=remote['internal_id'] or report['identity.epoch']!=remote['terminal_epoch']:
        raise RuntimeError('POS report does not belong to the retained external transaction')
    ledger=decode(ledger_payload,'TorqueEnvelope',{'exposure_nm2_s':'d','last_tick_us':'Q','last_feedback_us':'Q',
        'excess_started_us':'Q','last_abs_nm':'f','last_reason':'B','authorized':'B','tracking':'B','excessive':'B'})
    observed_us=struct.unpack('<Q',observation)[0]
    summary={'peak_output_torque_nm':report['peak_torque_nm'],'hold_elapsed_ms':report['elapsed_ms'],
        'hold_planned_ms':report['duration_ms'],'samples':report['samples'],'last_observed_since_start_ms':(observed_us-report['started_at_us'])/1000,
        'target_raw_flange_deg_at_project_ratio':math.degrees(report['target_position_rad'])*19.2032/44.8,
        'peak_mos_c':report['peak_mos_c'],'peak_rotor_c':report['peak_rotor_c']}
    evidence={'scope':'attach_read_only_no_halt_reset_resume_write','boot':expected_boot,'transaction':expected_id,
        'pairing':pairing,'vector_fingerprint_bytes':256,'core_remained_running':True,'motor_commands_sent':0,
        'remote':remote,'report':report,'ledger':ledger,'observation_us':observed_us,'summary':summary,'gateway_binding':binding}
    output=ROOT/('artifacts/pos-retained-'+str(expected_id)+'-1526a4e7-20261029.json')
    output.write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('READONLY_POS_CAPTURE',output,flush=True)
    print(json.dumps({'summary':summary,'report':report,'remote':remote,'ledger':ledger},indent=2),flush=True)


if __name__=='__main__':
    main()
