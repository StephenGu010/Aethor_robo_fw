"""Prepare ID7 PMAX512 without motion, reusing the tested fixed maintenance flow.

Default is offline. The live path verifies installed Flash before opening the
fixed serial maintenance session; no flashing, reset, enable or EEPROM save.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys

import deploy_401b_repair as repair

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT.parent / 'seven-axis-static-load/output/pos_range_sync_20261021/range_session.py'


def load_maintenance():
    """Import existing reviewed helpers without invoking their historical CLI."""
    specification = importlib.util.spec_from_file_location('fixed_pos_range_session', HELPER)
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def main():
    """Verify frozen repair inputs, then perform at most one authorized range attempt."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--supported-powered', action='store_true')
    arguments = parser.parse_args()
    prepared = repair.prepare_repair()
    maintenance = load_maintenance()
    print('OFFLINE_REPAIR_AND_MAINTENANCE_LOADED', flush=True)
    if not arguments.apply:
        return
    repair.shared.require(arguments.supported_powered, 'Independent support and motor power required')
    parent = ROOT / 'output/repair_20261023'
    token = parent / 'pmax512_maintenance.once'
    repair.shared.require(not token.exists(), 'Maintenance already attempted; no automatic replay')
    output = parent / ('pmax512_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir()
    report = {'physical_confirmation': 'independently supported, motor power retained',
              'helper_sha256': repair.shared.digest(HELPER.read_bytes()),
              'expected_hex_sha256': repair.shared.digest(prepared.image_payload),
              'motion_sent': False, 'enable_sent': False, 'eeprom_saved': False}
    lines = []
    serial_port = None
    try:
        hardware = repair.shared.load_hardware()
        probes = [probe for probe in hardware.probes() if probe.unique_id == repair.shared.PROBE_UID]
        repair.shared.require(len(probes) == 1, 'Expected unique probe absent')
        with hardware.Session(probes[0], options=dict(repair.shared.SESSION_OPTIONS)) as session:
            target = session.target
            repair.shared.require(target.read32(0x5C001000) & 0xFFF == 0x483, 'MCU mismatch')
            repair.shared.verify_usb_swd_pair(target, report)
            report['installed_verified_bytes'] = repair.shared.compare_segments(
                target, prepared.new_segments, hardware.Target.State.RUNNING, hardware)
        print('INSTALLED_REPAIR_VERIFIED_NO_HALT', flush=True)
        serial_port = maintenance.open_serial(hardware)
        fixed_session = maintenance.FixedSession(serial_port, lines)
        maintenance.run_mapping(fixed_session, token, report=report)
        report['commands'] = fixed_session.counts
        print('PMAX512_READBACK_DISCOVERY_CONTINUITY_PASS', json.dumps(report), flush=True)
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        if serial_port is not None:
            serial_port.close()
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        (output / 'serial.json').write_text(json.dumps(lines, ensure_ascii=False, indent=2), encoding='utf-8')
        print('EVIDENCE', output, flush=True)


if __name__ == '__main__':
    main()
