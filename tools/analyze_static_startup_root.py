"""@file Offline raw-byte startup forensics; never connects to a motor or probe.

The PD calculations are competing hypotheses, not an identified plant or a
driver execution trace. Passing integrity checks cannot qualify a root fix.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def require(condition, message):
    """Reject invalid evidence even when Python assertions are disabled."""
    if not condition:
        raise ValueError(message)


def linear(code, maximum, bits):
    """Decode the vendor unsigned symmetric mapping without application scaling."""
    return code * (2 * maximum) / ((1 << bits) - 1) - maximum


def decode_mit(payload, ranges):
    """Independently decode the five control fields from the actual eight bytes."""
    require(len(payload) == 8, 'MIT payload length')
    return dict(position=linear(int.from_bytes(payload[:2], 'big'), ranges[0], 16),
                velocity=linear((payload[2] << 4) | (payload[3] >> 4), ranges[1], 12),
                kp=(((payload[3] & 15) << 8) | payload[4]) * 500 / 4095,
                kd=((payload[5] << 4) | (payload[6] >> 4)) * 5 / 4095,
                feedforward=linear(((payload[6] & 15) << 8) | payload[7], ranges[2], 12))


def decode_feedback(payload, ranges):
    """Decode feedback independently of the JSON's convenience decoded fields."""
    require(len(payload) == 8, 'feedback payload length')
    return dict(state=payload[0] >> 4, motor=payload[0] & 15,
                position=linear(int.from_bytes(payload[1:3], 'big'), ranges[0], 16),
                velocity=linear((payload[3] << 4) | (payload[4] >> 4), ranges[1], 12),
                torque=linear(((payload[4] & 15) << 8) | payload[5], ranges[2], 12))


def raw_record(record):
    """Validate retained record hash and decode the version-one production ABI."""
    raw = bytes.fromhex(record['raw_hex'])
    require(len(raw) == 96, 'record ABI size')
    require(hashlib.sha256(raw).hexdigest() == record['sha256'], 'record hash mismatch')
    result = dict(timestamp=struct.unpack_from('<Q', raw)[0],
                  serial=struct.unpack_from('<I', raw, 16)[0],
                  event=raw[72], phase=raw[73], motor=raw[74],
                  payload=raw[78:78 + raw[76]])
    require(raw[76] <= 8, 'CAN length exceeds Classic CAN')
    return result


def analyze(trace):
    """Compare proven commands with raw feedback, retaining causal uncertainty."""
    header = bytes.fromhex(trace['header_hex'])
    require(len(header) == 832 and struct.unpack_from('<I', header)[0] == 1, 'header ABI')
    require(struct.unpack_from('<I', header, 64)[0] == 64, 'expected motor 7 only')
    start = struct.unpack_from('<Q', header, 48)[0]
    axis = struct.unpack_from('<6I11f', header, 80 + 6 * 80)
    target, ratio = axis[6:8]
    ranges = axis[11:14]
    require(all(value > 0 for value in (*ranges, ratio)), 'invalid range/ratio')
    records = [raw_record(record) for record in trace['records']]
    prefix = [record for record in records if record['timestamp'] <= start + 24000]
    require([record['serial'] for record in prefix] == list(range(len(prefix))), 'startup prefix gap')
    transmitted = [record for record in prefix if record['event'] == 4]
    require([record['phase'] for record in transmitted] == [2, 3, 4], 'startup actual TX sequence')
    require(transmitted[1]['payload'] == bytes.fromhex('fffffffffffffffc'), 'wrong ENABLE')
    require(transmitted[0]['payload'] == transmitted[2]['payload'], 'PRELOAD/FIRST_HOLD mismatch')
    command = decode_mit(transmitted[0]['payload'], ranges)
    require(abs(command['position'] - target) <= 2 * ranges[0] / 65535, 'frozen target mismatch')
    feedback = [record for record in prefix if record['event'] == 7]
    initial = decode_feedback(feedback[0]['payload'], ranges)
    require(abs(initial['position'] - command['position']) <= 2 * ranges[0] / 65535,
            'initial feedback and command differ by more than one position code')
    timeline = []
    for record in feedback:
        sample = decode_feedback(record['payload'], ranges)
        require(sample['motor'] == 7, 'feedback motor identity')
        # Conditional algebra only: neither formula proves the driver's internal coordinate.
        native_pd = (command['kp'] * (command['position'] - sample['position']) +
                     command['kd'] * (command['velocity'] - sample['velocity']) + command['feedforward'])
        output_feedback_pd = (command['kp'] * (command['position'] - sample['position'] / ratio) +
                              command['kd'] * (command['velocity'] - sample['velocity']) + command['feedforward'])
        timeline.append(dict(ms=(record['timestamp'] - start) / 1000, **sample,
                             native_coordinate_pd_nm=native_pd,
                             command_native_feedback_divided_by_ratio_pd_nm=output_feedback_pd))
    # A negative product is evidence against a synchronous same-coordinate PD explanation,
    # not proof of positive feedback: controller latency and internal state are unobserved.
    contradictory = [row['ms'] for row in timeline if row['state'] == 1 and
                     abs(row['torque']) > 1 and row['torque'] * row['native_coordinate_pd_nm'] < 0]
    return dict(boot=trace['boot'], generation=trace['generation'], target_native_rad=target,
                ranges=ranges, ratio=ratio, command=command,
                actual_tx_ms=[(record['timestamp'] - start) / 1000 for record in transmitted],
                command_hex=transmitted[0]['payload'].hex(), timeline=timeline,
                opposite_pd_feedback_sign_ms=contradictory,
                full_trace_gaps=struct.unpack_from('<I', header, 20)[0],
                motor_internal_target_acceptance='unknown', root_cause_established=False)


def main():
    """Write reproducible evidence; an explicitly requested repair gate fails closed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--require-root-cause', action='store_true')
    arguments = parser.parse_args()
    results = []
    for path in arguments.traces:
        payload = path.read_bytes()
        results.append(dict(path=str(path.resolve()), sha256=hashlib.sha256(payload).hexdigest(),
                            **analyze(json.loads(payload.decode('utf-8-sig')))))
    report = dict(analysis_only=True, hardware_access=False, root_fix_gate='NOT_ESTABLISHED', traces=results)
    arguments.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(traces=len(results), integrity='PASS', root_fix_gate=report['root_fix_gate'])))
    return 2 if arguments.require_root_cause else 0


if __name__ == '__main__':
    raise SystemExit(main())
