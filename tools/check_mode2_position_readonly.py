"""Read ID7 identity and position samples; optional discovery only reads registers.

No enable, disable, stop, motion, mode writes or SWD operations are exposed.
Close the Studio serial connection before invoking this bounded diagnostic.
"""
import argparse
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    'D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit/output/adrc/hardware/python_deps_20261010',
    'D:/download/TCG/output/can_id_commissioning_20260909/python_deps',
]
ALLOWED = {'ping', 'show state', 'adrc status', 'adrc gate motor=7',
           'show config 7', 'show motor 7', 'show diag can', 'adrc discover motor=7', 'ui caps'}


def main():
    """Verify the exact USB device and capture a finite sequence of passive observations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh-discovery', action='store_true')
    parser.add_argument('--remote-capability', action='store_true', help='Read the new adapter capability only; never submit an action')
    arguments = parser.parse_args()
    import serial
    from serial.tools import list_ports
    ports = [port for port in list_ports.comports() if port.vid == 0x0483 and
             port.pid == 0x5740 and port.serial_number == '386B336F3134']
    if len(ports) != 1:
        raise RuntimeError('Expected exactly one matching USB device')
    output = ROOT / 'output/idle-feedback' / ('mode2-readonly-' + str(time.time_ns()) + '.json')
    records = []
    sequence = 2000000000 + time.time_ns() // 1000000 % 800000000
    port = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=1)
    port.dtr = False
    port.rts = False
    port.port = ports[0].device

    def exchange(command):
        """Send one allowlisted request and require the same request number in its reply."""
        nonlocal sequence
        if command not in ALLOWED:
            raise ValueError('Command is not read-only allowlisted')
        sequence += 1
        wire = f'{sequence} {command}\n'.encode('ascii')
        records.append({'tx':wire.decode().strip(),'at':time.time()})
        if port.write(wire) != len(wire):
            raise RuntimeError('Incomplete request')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = port.readline().decode('ascii',errors='strict').strip()
            if line:
                records.append({'rx':line,'at':time.time()})
            if re.match(rf'^(ok|error) {sequence} ',line):
                if not line.startswith(f'ok {sequence} '):
                    raise RuntimeError(line)
                return dict(re.findall(r'(\w+)=([^\s]+)',line))
        raise TimeoutError(command)

    try:
        port.open()
        state = exchange('show state')
        owner = exchange('adrc status')
        gate = exchange('adrc gate motor=7')
        if arguments.refresh_discovery:
            if any(state.get(key) != value for key,value in
                   {'enabled':'00','moving':'0','active':'0','fault':'none'}.items()) or \
               owner.get('owner') != 'lcd' or gate.get('lcd_idle') != '1':
                raise RuntimeError('Software is not idle; discovery refused')
            exchange('adrc discover motor=7')
            time.sleep(2)
        for command in ('ping','adrc gate motor=7','show config 7','show diag can'):
            print(command,exchange(command),flush=True)
        if arguments.remote_capability:
            print('remote_capability', exchange('ui caps'), flush=True)
        for _ in range(20):
            print('position',exchange('show motor 7'),flush=True)
            time.sleep(0.25)
        print('final_can',exchange('show diag can'),flush=True)
    finally:
        port.close()
        output.write_text(json.dumps({'port':ports[0].device,'motion_commands_sent':0,
            'records':records},indent=2)+'\n',encoding='utf-8')
        print('READONLY_RECEIPT',output,flush=True)


if __name__ == '__main__':
    main()
