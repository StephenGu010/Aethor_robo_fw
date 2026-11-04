"""Validate real DISABLE capture transcripts and bind their exact saved evidence.

Receive lower bounds remain bounds. This offline tool never opens hardware,
installs qualification, supplies missing channels, or fits a motor model.
"""
import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path


def digest(path):
    """Fingerprint exact saved bytes without claiming measurement authenticity."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_capture(text):
    """Require correlated wire replies, one complete frozen capture and safe handback."""
    requests = {}
    records = []
    completed = None
    hardware = None
    limits = None
    verified_zero = False
    last_wire_id = 0
    allowed_commands = {'show state', 'show diag can', 'adrc status', 'adrc gate motor=7',
                        'adrc discover motor=7', 'adrc hardware', 'adrc limits', 'adrc feedback',
                        'adrc evidence', 'adrc acquisition motor=7', 'bench mode 7 mode=mit',
                        'adrc acquire motor=7', 'adrc release', 'bench mode 7 mode=pos_vel',
                        'bench disable 7', 'adrc sample', 'adrc capture', 'stop'}
    for line in text.splitlines():
        sent = re.fullmatch(r'> (\d+) (.+)', line)
        if sent:
            request_id = int(sent[1])
            if request_id <= last_wire_id:
                raise ValueError('Wire request IDs are not strictly increasing')
            last_wire_id = request_id
            requests[request_id] = sent[2]
            if sent[2] not in allowed_commands and not re.fullmatch(r'adrc capture index=(?:[0-5]?\d|6[0-3])', sent[2]):
                raise ValueError('Unexpected actuation/clear command in disabled capture')
        if re.match(r'< ok \d+ adrc status ', line) and (' qualified=0 ' not in line or ' fault=0 ' not in line):
            raise ValueError('Unqualified/fault-free state was not preserved')
        reply = re.fullmatch(r'< ok (\d+) adrc (hardware|limits|capture|evidence) (.+)', line)
        if not reply:
            continue
        request_id, category = int(reply[1]), reply[2]
        fields = dict(re.findall(r'(\w+)=([^ ]+)', reply[3]))
        if request_id not in requests:
            raise ValueError('Reply has no matching transmitted request')
        expected = 'adrc ' + category
        if category == 'capture' and 'index' in fields:
            expected += ' index=' + fields['index']
        if requests[request_id] != expected:
            raise ValueError('Reply does not match its transmitted command')
        if category == 'hardware':
            hardware = fields
        elif category == 'limits':
            limits = fields
        elif category == 'evidence':
            if fields.get('verified') != '0':
                raise ValueError('Diagnostic transcript contains motion qualification')
            verified_zero = True
        elif 'index' not in fields:
            if fields.get('state') == '3':
                raise ValueError('Capture aborted')
            if fields.get('state') == '2':
                if completed is not None and completed != fields:
                    raise ValueError('Frozen capture changed during export')
                completed = fields
        else:
            if completed is None or fields.get('id') != completed['id']:
                raise ValueError('Row is not from the completed capture')
            records.append(fields)
    if completed is None or completed.get('motor') != '7' or completed.get('reason') != '0' or \
            completed.get('count') != '64' or len(records) != 64 or not 64 <= int(completed['requests']) <= 68:
        raise ValueError('Incomplete bounded motor-7 capture')
    if requests.get(int(completed['id'])) != 'adrc sample':
        raise ValueError('Capture identity has no matching start command')
    if hardware is None or limits is None or hardware.get('motor') != '7' or \
            hardware.get('mode') != '1' or hardware.get('fields') != '1fff' or limits.get('motor') != '7':
        raise ValueError('Missing selected MIT identity and range readbacks')
    if not verified_zero or 'HANDOFF_WINDOW_COMPLETE' not in text or \
            'mode_restored=True final_disable_completed=True acquire_transferred=True release_verified=True' not in text or \
            'host_error=' in text or 'hardware_poweroff_required=1' in text:
        raise ValueError('Session did not finish with confirmed handback')
    previous_rx = 0
    previous_observed = 0
    intervals = []
    ages = []
    positions = []
    normalized = []
    started = int(completed['start_us'])
    first_request = int(completed['first_request_us'])
    for index, row in enumerate(records):
        observed, received = int(row['observed_us']), int(row['rx_lower_us'])
        position, velocity, torque = (float(row[key]) for key in ('pos', 'vel', 'torque'))
        if int(row['index']) != index or not 0 <= started <= first_request < received <= observed <= 2**64 - 1 or \
                observed - received > 8000 or observed - started >= 400000 or \
                not all(math.isfinite(value) for value in (position, velocity, torque)) or abs(velocity) > 0.05:
            raise ValueError('Invalid sample ordering, timing, values or bounds')
        if index == 0 and observed - started > 16000:
            raise ValueError('First sample exceeded the startup deadline')
        if index:
            if not 0 < received - previous_rx <= 8000 or observed <= previous_observed:
                raise ValueError('Sample gap exceeded the unchanged 8 ms gate')
            intervals.append(received - previous_rx)
            if abs(position - positions[0]) > 0.005:
                raise ValueError('Disabled capture position drift exceeded its bound')
        normalized.append({'index': index, 'capture_id': int(row['id']), 'observed_us': observed,
                           'rx_lower_us': received, 'position_rad': position,
                           'raw_velocity_rad_s': velocity, 'feedback_torque_nm': torque})
        ages.append(observed - received)
        positions.append(position)
        previous_rx, previous_observed = received, observed
    return {'capture': completed, 'hardware': hardware, 'limits': limits, 'records': normalized,
            'sample_count': len(records), 'minimum_rx_interval_us': min(intervals),
            'maximum_rx_interval_us': max(intervals), 'maximum_observation_age_us': max(ages),
            'position_span_rad': max(positions) - min(positions),
            'disabled_feedback_window_passed': True, 'enabled_feedback_verified': False,
            'watchdog_action_verified': False, 'hardware_qualified': False, 'fit_ready': False,
            'qualification_flags': 0, 'sample_time_known': False, 'input_time_known': False,
            'missing': ['enabled_feedback_timing', 'watchdog_action', 'coordinate_calibration',
                        'actual_input_and_sample_timing', 'bounded_excitation', 'identified_model',
                        'candidate_specific_stop_bound']}


def main():
    """Export validated rows and hashes, retaining explicit limits on hardware qualification."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--transcript', type=Path, required=True)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    report = parse_capture(arguments.transcript.read_text(encoding='utf-8-sig'))
    deployment = json.loads(arguments.deployment.read_text(encoding='utf-8-sig'))
    if deployment.get('flash_readback_equal') is not True or deployment.get('reset_completed') is not True or \
            deployment.get('state_after') != 'State.RUNNING' or deployment.get('motion_commands_sent') != 0:
        raise ValueError('Missing successful no-motion deployment/readback evidence')
    image_path = arguments.deployment.parent / 'LCD-MIT-ADRC.hex'
    if digest(image_path) != deployment['hex_sha256']:
        raise ValueError('Saved deployed image changed')
    arguments.output.mkdir(parents=True, exist_ok=False)
    csv_path = arguments.output / 'disabled_feedback.csv'
    records = report.pop('records')
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    report.update({'source': 'serial_live_device_transcript', 'transcript_path': str(arguments.transcript.resolve()),
                   'transcript_sha256': digest(arguments.transcript), 'csv_sha256': digest(csv_path),
                   'deployment_path': str(arguments.deployment.resolve()),
                   'deployment_sha256': digest(arguments.deployment), 'deployed_image_sha256': digest(image_path),
                   'image_binding_scope': 'prior_verified_flash_readback_not_continuous_boot_attestation',
                   'torque_meaning': 'drive_feedback_not_transmitted_command',
                   'timestamp_meaning': 'receive_lower_bound_and_observation_not_physical_sample_time'})
    (arguments.output / 'capture_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('sample_count', 'maximum_rx_interval_us',
          'maximum_observation_age_us', 'disabled_feedback_window_passed', 'hardware_qualified', 'fit_ready')}))


if __name__ == '__main__':
    main()
