"""Reproducible offline S3519 grid, measurement and handoff analysis using production C.

No device APIs are imported. Historic ranges are evidence inputs; the plant,
feedback encoder rounding, fixture identity and handoff inputs are synthetic.
Passing this analysis never qualifies hardware or publishes a firmware image.
"""

import argparse
from collections import deque
import csv
import ctypes
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SAMPLE_TIME = 0.004


class PositionEstimator:
    """Causal endpoint difference with actual timestamps and explicit history invalidation."""

    def __init__(self, window_seconds, position_range):
        """Set an analysis-only window; use the existing 8 ms freshness deadline."""
        if not all(math.isfinite(value) and value > 0 for value in (window_seconds, position_range)):
            raise ValueError('Estimator window and range must be finite and positive')
        self.window = window_seconds
        self.position_range = position_range
        self.history = deque()
        self.resets = 0

    def update(self, timestamp, position):
        """Return no estimate until a full window exists; never unwrap reset/wrap ambiguities."""
        if not math.isfinite(timestamp) or not math.isfinite(position) or abs(position) > self.position_range:
            raise ValueError('Invalid timestamp or position')
        if self.history:
            previous_time, previous_position = self.history[-1]
            interval = timestamp - previous_time
            position_quantum = 2 * self.position_range / 65535
            if interval <= 0 or interval > 0.008000001 or abs(position - previous_position) > max(0, interval) + 2 * position_quantum:
                self.history.clear()
                self.resets += 1
        self.history.append((timestamp, position))
        while len(self.history) > 1 and timestamp - self.history[1][0] >= self.window - 1e-10:
            self.history.popleft()
        first_time, first_position = self.history[0]
        if timestamp - first_time < self.window - 1e-10:
            return None
        return (position - first_position) / (timestamp - first_time)


def feedback_code(value, magnitude, bits, rounding):
    """Model two explicitly assumed driver conversions, preserving the bipolar offset."""
    if (not math.isfinite(value) or not math.isfinite(magnitude) or magnitude <= 0 or
            abs(value) > magnitude or bits not in (12, 16) or rounding not in ('floor', 'nearest')):
        raise ValueError('Invalid feedback mapping input')
    unrounded = (value + magnitude) * ((1 << bits) - 1) / (2 * magnitude)
    return min((1 << bits) - 1, max(0, math.floor(unrounded + (0.5 if rounding == 'nearest' else 0))))


def digest(path):
    """Bind an evidence or source file to its SHA-256 bytes."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    """Persist strict JSON; NaN and Inf cannot masquerade as completed metrics."""
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def verify_model_run(model_run):
    """Require the passed upstream generated-C/MIL report and unchanged closed artifact set."""
    report_path = model_run / 'pipeline_report.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    if not all(report.get(field) is True for field in ('passed', 'verificationHashesMatch')):
        raise ValueError('Upstream model run did not pass')
    generated_directory = model_run / 'generated/adrc_controller_ert_rtw'
    expected_sources = {ROOT / 'Models/Adrc' / name for name in
                        ('build_adrc_models.m', 'run_adrc_pipeline.m', 'adrc_simulation_parameters.m', 'adrc_generated_replay.c')}
    expected_artifacts = {model_run / (name + '.slx') for name in ('adrc_controller', 'adrc_plant', 'adrc_validation')}
    expected_artifacts.update(generated_directory.glob('*.c'))
    expected_artifacts.update(generated_directory.glob('*.h'))
    for field, expected in (('verificationSources', expected_sources), ('verificationArtifacts', expected_artifacts)):
        actual = {Path(item['path']).resolve() for item in report[field]}
        if actual != {path.resolve() for path in expected}:
            raise ValueError('Unexpected upstream manifest membership: ' + field)
        for item in report[field]:
            if digest(item['path']) != item['sha256'].lower():
                raise ValueError('Changed upstream evidence: ' + item['path'])
    if not report['cReplay']['passed'] or not report['closedLoopMil']['passed']:
        raise ValueError('Missing generated-C or closed-loop validation')
    return generated_directory, expected_sources | expected_artifacts | {report_path}


class ProductionBridge:
    """Own a host-only DLL exposing production functions and the verified generated controller."""

    def __init__(self, library_path, compiler_directory, ranges):
        """Load the explicit library and declare every ctypes ABI signature."""
        self.dll_directory = os.add_dll_directory(str(compiler_directory)) if os.name == 'nt' else None
        self.library = ctypes.CDLL(str(library_path))
        self.ranges = ranges
        float_pointer = ctypes.POINTER(ctypes.c_float)
        signatures = {
            'analysis_initialize': [ctypes.c_float] * 3,
            'analysis_decode': [ctypes.c_uint32] * 3 + [float_pointer],
            'analysis_neutral': [ctypes.POINTER(ctypes.c_uint8)],
            'analysis_torque': [ctypes.c_float] * 4 + [float_pointer],
            'analysis_control': [ctypes.c_uint8] + [ctypes.c_float] * 6 + [ctypes.c_uint8] * 2 + [float_pointer],
            'analysis_estimate': [ctypes.c_uint64] * 2 + [ctypes.c_float] * 2 + [float_pointer],
            'analysis_acquire': [],
            'analysis_owner_step': [ctypes.c_uint64] * 3 + [ctypes.c_uint32],
            'analysis_queued_owner_step': [ctypes.c_uint64] * 3 + [ctypes.c_uint32],
            'analysis_release': [ctypes.c_uint64] * 3 + [ctypes.c_uint8],
        }
        for name, arguments in signatures.items():
            function = getattr(self.library, name)
            function.argtypes = arguments
            function.restype = ctypes.c_int
        self.reset()

    def reset(self):
        """Reset only synthetic host memory between independent scenarios."""
        if self.library.analysis_initialize(*self.ranges) != 1:
            raise ValueError('Invalid production fixture ranges')

    def decode(self, position_code, velocity_code, torque_code=2047):
        """Decode a simulated wire frame with production float32 arithmetic."""
        values = (ctypes.c_float * 3)()
        if self.library.analysis_decode(position_code, velocity_code, torque_code, values) != 1:
            raise ValueError('Production feedback decode rejected input')
        return tuple(values)

    def torque(self, target, previous, slew):
        """Return actual production lattice outcome instead of idealized post-limit rounding."""
        sent = ctypes.c_float()
        status = self.library.analysis_torque(target, previous, 0.1, slew * SAMPLE_TIME, ctypes.byref(sent))
        return status, sent.value

    def control(self, mode, reference, measurement, previous, bandwidth, reset=False, new_sample=True):
        """Run unmodified generated control code with explicitly synthetic b0=10."""
        outputs = (ctypes.c_float * 3)()
        if not self.library.analysis_control(mode, reference, measurement, previous, 10, bandwidth,
                                             5 * bandwidth, reset, new_sample, outputs):
            raise ArithmeticError('Generated controller produced a nonfinite result')
        return tuple(outputs)

    def estimate(self, now_us, feedback_us, position, raw_velocity):
        """Use the production fixed-memory 32 ms estimator, including its readiness gate."""
        value = ctypes.c_float()
        ready = self.library.analysis_estimate(now_us, feedback_us, position, raw_velocity, ctypes.byref(value))
        return value.value if ready else None


def compile_bridge(compiler, generated_directory, output):
    """Compile production C and a thin host adapter; retain stdout and reject build failures."""
    source_paths = [ROOT / path for path in ('Tests/host/adrc_offline_analysis_bridge.c',
                   'App/Motor/s3519_codec.c', 'App/Motor/can_frame.c',
                   'App/Motor/motor_adrc_command.c', 'App/Motor/motor_capability_collector.c', 'App/Adrc/adrc_lcd_ownership.c',
                   'App/Adrc/adrc_generated_adapter.c', 'App/Platform/can_rx_inbox.c',
                   'Tests/host/adrc_interface_analysis_bridge.c', 'App/Adrc/adrc_feedback_adapter.c',
                   'App/Adrc/adrc_simple_trial.c', 'App/Adrc/adrc_app_bridge.c', 'App/Adrc/adrc_experiment.c',
                   'App/Adrc/adrc_bench.c', 'App/Adrc/adrc_protocol.c',
                   'App/Config/arm_config.c', 'App/Config/build_info.c',
                   'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
                   'App/Motor/can_tx_scheduler.c', 'App/Motion/joint_motion.c',
                   'App/Motion/joint_motion_can.c', 'App/Arm/joint_reference.c', 'App/Arm/arm_controller.c',
                   'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
                   'App/Protocol/protocol_engine.c', 'App/Telemetry/diagnostics.c')]
    source_paths += sorted(generated_directory.glob('*.c'))
    includes = [ROOT / 'App' / name for name in ('Motor', 'Config', 'Arm', 'Motion', 'Adrc', 'Telemetry', 'Platform', 'Protocol')]
    includes += [generated_directory]
    library_path = output / 'offline_analysis.dll'
    command = [str(compiler), '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror']
    command += ['-I' + str(path) for path in includes]
    command += [str(path) for path in source_paths] + ['-lm', '-o', str(library_path)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120, check=False)
    (output / 'compile.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    if result.returncode:
        raise RuntimeError('Production bridge build failed; inspect compile.log')
    return library_path


def run_regressions(output):
    """Run the analysis tests and existing host-only ownership/channel/integration gates."""
    powershell = Path(os.environ['SystemRoot']) / 'System32/WindowsPowerShell/v1.0/powershell.exe'
    commands = [('analysis_unit_tests', [sys.executable, str(ROOT / 'Tests/host/test_adrc_offline_analysis.py')]),
                ('stop_tuning_unit_tests', [sys.executable, str(ROOT / 'Tests/host/test_adrc_stop_tuning.py')])]
    for name in ('run_adrc_lcd_ownership_tests', 'run_adrc_channel_tests', 'run_adrc_lcd_integration_tests',
                 'run_adrc_feedback_adapter_tests', 'run_adrc_app_tests', 'run_adrc_experiment_tests'):
        commands.append((name, [str(powershell), '-NoProfile', '-ExecutionPolicy', 'Bypass',
                                '-File', str(ROOT / 'Tests/host' / (name + '.ps1'))]))
    records = []
    for name, command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=180, check=False)
        (output / (name + '.log')).write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f'Offline regression {name} failed with exit {result.returncode}')
        records.append({'name': name, 'exit_code': result.returncode})
    return records


def check_grid(bridge):
    """Verify real central codes and quantify incompatibility with unchanged hardware limits."""
    position_range, velocity_range, torque_range = bridge.ranges
    neutral_bytes = (ctypes.c_uint8 * 8)()
    assert bridge.library.analysis_neutral(neutral_bytes) == 1
    assert list(neutral_bytes) == [0x7f, 0xff, 0x7f, 0xf0, 0, 0, 7, 0xff]
    negative_center = bridge.decode(32767, 2047)
    positive_center = bridge.decode(32768, 2048, 2048)
    default_status, _ = bridge.torque(0, 0, 0.5)
    candidate_status, candidate_torque = bridge.torque(0, 0, 1.25)
    assert default_status != 0 and candidate_status == 0
    assert negative_center[1] < 0 < positive_center[1]
    assert abs(candidate_torque) > 0
    # Exhaust every position/velocity code independently through the production decoder.
    previous = -math.inf
    for code in range(65536):
        position = bridge.decode(code, 2047)[0]
        assert position > previous
        previous = position
    previous = -math.inf
    for code in range(4096):
        velocity = bridge.decode(32767, code)[1]
        assert velocity > previous
        previous = velocity
    return {'position_quantum_rad': 2 * position_range / 65535,
            'velocity_quantum_rad_s': 2 * velocity_range / 4095,
            'torque_quantum_nm': 2 * torque_range / 4095,
            'negative_center': negative_center, 'positive_center': positive_center,
            'neutral_frame_hex': bytes(neutral_bytes).hex(' '),
            'default_first_step_rejected': True, 'default_status': default_status,
            'minimum_initial_slew_nm_s': torque_range / 4095 / SAMPLE_TIME,
            'minimum_adjacent_code_slew_nm_s': 2 * torque_range / 4095 / SAMPLE_TIME,
            'decoded_center_initial_slew_nm_s': abs(negative_center[2]) / SAMPLE_TIME,
            'decoded_center_adjacent_slew_nm_s': (positive_center[2] - negative_center[2]) / SAMPLE_TIME,
            'candidate_slew_nm_s': 1.25, 'candidate_requires_additional_measured_qualification': True,
            'exhaustive_decoded_codes': 65536 + 4096}


def handoff_matrix(bridge):
    """Assert repaired deadlines and immutable receive bounds without claiming physical replies."""
    library = bridge.library
    safe_flags = 503
    cases = [
        ('fresh_disabled', 104000, 103000, safe_flags, 38),
        ('no_feedback', 204000, 0, safe_flags & ~16, 8),
        ('stale_feedback', 104000, 95000, safe_flags, 21),
        ('pre_submit_feedback', 104000, 100999, safe_flags, 21),
        ('at_submit_feedback', 104000, 101000, safe_flags, 21),
        ('one_us_after_submit', 104000, 101001, safe_flags, 38),
        ('future_feedback', 104000, 104001, safe_flags, 4),
        ('no_tx_receipt', 104000, 103000, safe_flags & ~4, 21),
        ('send_failed', 104000, 103000, safe_flags | 8, 4),
        ('enabled_feedback', 104000, 103000, safe_flags & ~32, 4),
        ('fault_feedback', 104000, 103000, safe_flags & ~64, 4),
        ('moving_feedback', 104000, 103000, safe_flags & ~128, 4),
        ('age_8ms', 110000, 102000, safe_flags, 38),
        ('age_over_8ms', 110001, 102000, safe_flags, 21),
        ('exact_deadline_no_feedback', 200000, 0, safe_flags & ~16, 8),
        ('exact_deadline_fresh_feedback', 200000, 199000, safe_flags, 8),
        ('before_deadline_fresh_feedback', 199999, 199000, safe_flags, 38),
        ('after_deadline_no_feedback', 200001, 0, safe_flags & ~16, 8),
        ('after_deadline_fresh_feedback', 204000, 203000, safe_flags, 8),
    ]
    results = []
    for name, now, feedback, flags, expected in cases:
        assert library.analysis_acquire() == 0
        assert library.analysis_owner_step(100000, 0, 0, 259) == 21
        actual = library.analysis_owner_step(now, feedback, 101000, flags)
        assert actual == expected, (name, actual, expected)
        results.append({'case': name, 'owner': actual // 16, 'status': actual % 16})
    # Once timeout returns LCD, a late frame alone cannot restart acquisition.
    library.analysis_acquire()
    library.analysis_owner_step(100000, 0, 0, 259)
    assert library.analysis_owner_step(204000, 0, 101000, safe_flags & ~16) == 8
    assert library.analysis_owner_step(208000, 207000, 101000, safe_flags) == 5
    results.append({'case': 'late_frame_after_timeout_keeps_lcd', 'owner': 0, 'status': 5})
    # The real queue must preserve the old/unknown bound when task processing is late.
    for name, lower_bound, expected in (
            ('queued_pre_challenge_frame_rejected', 99000, 21),
            ('queued_unknown_arrival_rejected', 0, 21),
            ('queued_post_challenge_frame_accepted', 102000, 38)):
        library.analysis_acquire()
        library.analysis_owner_step(100000, 0, 0, 259)
        actual = library.analysis_queued_owner_step(104000, lower_bound, 101000, safe_flags)
        assert actual == expected, (name, actual, expected)
        results.append({'case': name, 'owner': actual // 16, 'status': actual % 16,
                        'arrival_lower_bound_us': lower_bound, 'processing_us': 104000})
    for name, feedback, now, transmitted, success, expected in (
            ('release_no_feedback', 0, 720001, 222000, 1, 56),
            ('release_no_receipt', 222000, 224000, 222000, 0, 53),
            ('release_pre_submit', 220999, 224000, 222000, 1, 53),
            ('release_before_receipt_sampling', 221500, 224000, 222000, 1, 7),
            ('release_fresh', 223000, 224000, 222000, 1, 7)):
        library.analysis_acquire()
        library.analysis_owner_step(100000, 0, 0, 259)
        assert library.analysis_owner_step(104000, 103000, 101000, safe_flags) == 38
        assert library.analysis_release(220000, 221000, transmitted, success) == 0
        flags = safe_flags if feedback else safe_flags & ~16
        actual = library.analysis_owner_step(now, feedback, 0, flags)
        assert actual == expected, (name, actual, expected)
        results.append({'case': name, 'owner': actual // 16, 'status': actual % 16})
    return {'contract_cases_passed': len(results), 'cases': results,
            'known_gaps': [
                '驱动失能回应及真实时间不确定性仍需实测；未连接设备。',
                '接收时间是FIFO空闲观察建立的保守下界，单次即时回复可能被拒绝。',
                'TX 完成轮询不是精确总线发送时间；本回归不替代真实ISR调度与总线测量。'],
            'hardware_safe_to_run': False}


def replay_historical_log(path):
    """Extract only captured facts; command completion is not independent disabled feedback."""
    text = path.read_text(encoding='utf-8-sig')
    gate_lines = [line for line in text.splitlines() if 'acq_submit=' in line]
    if not gate_lines:
        raise ValueError('Historical log has no acquisition diagnostic snapshot')
    fields = dict(re.findall(r'(\w+)=([^\s]+)', gate_lines[-1]))
    expected = {'acq_tx': '1', 'acq_rx': '0', 'acq_failed': '0', 'acq_neutral': '1'}
    if any(fields.get(key) != value for key, value in expected.items()):
        raise ValueError('Historical evidence no longer matches the analyzed window')
    if 'owner=lcd handoff=timeout' not in text or 'acquire_transferred=False release_verified=False' not in text:
        raise ValueError('Historical outcome was not the expected timeout')
    return {'path': str(path), 'sha256': digest(path), 'observed_fields': fields,
            'owner_after_window': 'lcd', 'handoff': 'timeout', 'release_verified': False,
            'scope': 'application_processed_frame_window_not_bus_capture',
            'driver_disable_reply_behavior_proven': False}


def simulate(bridge, output, scenario):
    """Run a synthetic plant around real generated C and production safe torque selection."""
    mode, measurement_method, rounding, b0_ratio, delay_steps, target, bandwidth = scenario
    bridge.reset()
    position_range, velocity_range, _ = bridge.ranges
    window = {'raw': 0, 'position32': 0.032, 'position64': 0.064}[measurement_method]
    estimator = PositionEstimator(window, position_range) if measurement_method == 'position64' else None
    position = velocity = previous_sent = reference = 0.0
    observation_started = False
    sensor_history = deque([(0.0, 0.0)] * (delay_steps + 1), maxlen=delay_steps + 1)
    rows = []
    name = f'm{mode}_{measurement_method}_{rounding}_b{b0_ratio}_d{delay_steps}_r{target}_w{bandwidth}'
    for tick in range(2001):
        timestamp = tick * SAMPLE_TIME
        desired = target if 1 <= timestamp < 5 else 0.0
        reference += max(-0.2 * SAMPLE_TIME, min(0.2 * SAMPLE_TIME, desired - reference))
        sensor_history.append((position, velocity))
        sensor_position, sensor_velocity = sensor_history[0]
        measured_position, raw_velocity, _ = bridge.decode(
            feedback_code(sensor_position, position_range, 16, rounding),
            feedback_code(sensor_velocity, velocity_range, 12, rounding))
        if measurement_method == 'position32':
            sample_us = 1000 + tick * 4000
            estimate = bridge.estimate(sample_us, sample_us, measured_position, raw_velocity)
        else:
            estimate = estimator.update(timestamp, measured_position) if estimator else raw_velocity
        ready = estimate is not None
        if observation_started and not ready:
            raise ArithmeticError('Observation lost after startup; abort without inventing physical stop behavior')
        observation_started = observation_started or ready
        measurement = estimate if ready else 0.0
        torque_raw, z1, z2 = bridge.control(mode, reference, measurement, previous_sent,
                                           bandwidth, reset=(tick == 0 or not ready), new_sample=ready)
        sent = 0.0
        if ready and tick:
            status, sent = bridge.torque(torque_raw, previous_sent, 1.25)
            if status:
                raise ArithmeticError(f'No permitted hypothetical torque grid point: {name}, tick={tick}, status={status}')
            assert abs(sent) <= 0.100001 and abs(sent - previous_sent) <= 1.25 * SAMPLE_TIME + 1e-7
        disturbance = 0.1 if 3 <= timestamp < 4 else 0.0
        rows.append([timestamp, reference, velocity, position, raw_velocity, measurement, sent,
                     torque_raw, z1, z2, int(ready), disturbance])
        position += SAMPLE_TIME * velocity
        velocity += SAMPLE_TIME * (10 * b0_ratio * sent - velocity + disturbance)
        previous_sent = sent
        if not all(math.isfinite(value) for value in (velocity, position, sent)):
            raise ArithmeticError('Nonfinite plant state')
    tracking = [row for row in rows if 2 <= row[0] < 5]
    stopped = [row for row in rows if row[0] >= 7]
    steady = [row for row in rows if 4.5 <= row[0] < 5]
    rms = lambda values: math.sqrt(sum(value * value for value in values) / len(values))
    metrics = {'name': name, 'mode': mode, 'measurement': measurement_method, 'rounding_assumption': rounding,
               'b0_ratio': b0_ratio, 'extra_delay_samples': delay_steps, 'target_rad_s': target,
               'wc': bandwidth, 'wo': 5 * bandwidth, 'estimator_window_ms': window * 1000,
               'estimator_group_delay_ms': window * 500,
               'tracking_rms_rad_s': rms([row[2] - row[1] for row in tracking]),
               'measurement_rms_rad_s': rms([row[5] - row[2] for row in tracking]),
               'steady_error_abs_rad_s': abs(sum(row[2] - row[1] for row in steady) / len(steady)),
               'stop_tail_peak_rad_s': max(abs(row[2]) for row in stopped),
               'peak_speed_rad_s': max(abs(row[2]) for row in rows),
               'peak_nominal_torque_nm': max(abs(row[6]) for row in rows),
               'samples': len(rows), 'hardware_qualified': False}
    metrics['illustrative_tracking_stop_criterion_met'] = (
        metrics['steady_error_abs_rad_s'] <= 0.01 and metrics['stop_tail_peak_rad_s'] <= 0.02)
    with (output / 'traces' / (name + '.csv')).open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['time_s', 'reference_rad_s', 'true_velocity_rad_s', 'true_position_rad',
                         'raw_velocity_rad_s', 'estimated_velocity_rad_s', 'nominal_sent_nm',
                         'raw_torque_nm', 'eso_z1', 'eso_z2', 'measurement_ready', 'disturbance_rad_s2'])
        writer.writerows(rows)
    return metrics


def render_summary(report):
    """Render conclusions while separating analysis completion from motor acceptance."""
    grid = report['grid']
    lines = ['# S3519 离线量化与失能反馈分析', '',
             '分析对象：历史真实量程 + 合成动力学。未连接硬件；合成资格只存在宿主进程，默认硬件资格仍未授予。', '',
             f"速度码格：{grid['velocity_quantum_rad_s']:.9f} rad/s；转矩码格：{grid['torque_quantum_nm']:.9f} Nm。",
             f"0.5 Nm/s、4 ms 第一拍被生产选码器拒绝。理想码格理论阈值：第一拍 {grid['minimum_initial_slew_nm_s']:.6f} Nm/s，相邻码格 {grid['minimum_adjacent_code_slew_nm_s']:.6f} Nm/s；不是已验证的最小浮点通过门限。",
             f"生产 float32 中心码反解对应 {grid['decoded_center_initial_slew_nm_s']:.6f}/{grid['decoded_center_adjacent_slew_nm_s']:.6f} Nm/s。", '',
             '以下432个测量链比较场景使用候选1.25 Nm/s；原始速度和64 ms分支只作敏感性比较，不是获准硬件配置。',
             '32 ms分支使用生产C估速器，64 ms分支保留Python研究模型。候选配置须额外实测资格；默认0.5 Nm/s仍拒绝该历史量程。', '',
             '| 测量 | 场景数 | 指标达标数 | 最坏跟踪 RMS rad/s | 最坏零参考尾段 rad/s |',
             '| --- | ---: | ---: | ---: | ---: |']
    for method in ('raw', 'position32', 'position64'):
        metrics = [item for item in report['scenarios'] if item['measurement'] == method]
        lines.append(f"| {method} | {len(metrics)} | {sum(item['illustrative_tracking_stop_criterion_met'] for item in metrics)} | "
                     f"{max(item['tracking_rms_rad_s'] for item in metrics):.6f} | {max(item['stop_tail_peak_rad_s'] for item in metrics):.6f} |")
    lines += ['', '指标仅为诊断比较：4.5–5 s 均值误差不超过 0.01 rad/s，7–8 s 零参考尾段峰值不超过 0.02 rad/s；不构成硬件验收。该尾段继续运行控制器，不模拟 STOP/失能过程。',
              '32/64 ms窗口在均匀采样下分别引入约16/32 ms延迟。生产32 ms分支采用0.3 rad/s加2码格的跳变门，重复同一内容不推进，重复时间内容变化则失效；Python64 ms研究分支仍为1 rad/s假设。',
              '估速等待窗口使用未驱动、零输入的合成初态；开始观察后估计丢失即中止分析，不假设可以瞬间零力矩。闭环扫描采用均匀采样，缺帧/跳变在测量器单元测试中检查，不冒充闭环硬件故障试验。',
              '扫描包含正负与零参考、b0=0.5/1/2 倍、0/8 ms 额外延迟、wc=2/4 与 wo=5wc、两种未知舍入假设；未加入额外模拟传感器噪声、摩擦或真实负载。', '',
              f"所有权/接收队列 {report['handoff']['contract_cases_passed']} 项修复后契约检查通过。剩余实测边界："]
    lines += ['- ' + text for text in report['handoff']['known_gaps']]
    lines += ['', '历史 acq_rx=0 仅表示应用层限定统计窗口未观察到帧；不是总线抓包，不证明驱动收到了命令或失能态永不回帧。',
              '软件已明确接收下界、两段挑战诊断及超时优先级；仍需取得驱动失能反馈/独立安全状态证据。',
              '真实惯量、摩擦、反馈噪声、舍入策略和位置重置行为仍需实测。']
    replay = report['production_interface']
    lines += ['', '## 生产接口完整路径', '',
              f"{replay['scenario_count']}个场景执行生产runtime解码、bridge、supervisor、生成C、真实编码选码及发送回执；幅值/步长/协议链不变量全部通过。",
              f"合成驱动模型中正常结束：{replay['normal_completion_count']}/{replay['scenario_count']}；其他场景保留故障结果，不记成正常停止。",
              f"原固定集严格真值检查通过：{replay['strict_completion_count']}/{replay['scenario_count']}；显式离线参数 wc/wo：{replay['offline_tuning_wc_wo']}。",
              '该结果仅覆盖原固定集。独立停止调参报告保留了额外留出集的真实低速驻留反例，不能据此宣称泛化验收或硬件资格通过。',
              '驱动使能/失能确认延迟假设为2个tick，确认后理想改变力矩；不是实测断能性能。完整CSV另存interface_traces。']
    return '\n'.join(lines) + '\n'


def main():
    """Validate evidence, execute deterministic offline analysis, and persist success or failure."""
    if not __debug__:
        raise RuntimeError('Run without Python -O: offline validation assertions must remain enabled')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-run', type=Path, required=True)
    parser.add_argument('--compiler', type=Path, default=Path('D:/application/WinLibs/mingw64/bin/gcc.exe'))
    parser.add_argument('--range-record', type=Path, default=ROOT / 'output/adrc/hardware/enabled_feedback_pilot/motor7_range_ram_read_20260923.txt')
    parser.add_argument('--handoff-log', type=Path, default=ROOT / 'output/adrc/hardware/handoff_window_20260923/motor7_handoff_20260923T092300329Z.txt')
    parser.add_argument('--output', type=Path)
    arguments = parser.parse_args()
    output = arguments.output or ROOT / 'output/adrc/quantization' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'traces').mkdir()
    report = {'analysis_completed': False, 'hardware_connected': False, 'hardware_qualified': False,
              'provenance': 'historical_ranges_synthetic_dynamics_unknown_driver_rounding',
              'stage': 'verify_inputs', 'output': str(output)}
    report_path = output / 'analysis_report.json'
    try:
        generated_directory, evidence_paths = verify_model_run(arguments.model_run.resolve())
        range_text = arguments.range_record.read_text(encoding='utf-8-sig')
        match = re.search(r'motor7_snapshot pmax=([\d.]+) vmax=([\d.]+) tmax=([\d.]+)', range_text)
        if not match:
            raise ValueError('Historical range record is missing or unrecognized')
        ranges = tuple(float(value) for value in match.groups())
        if ranges != (12.5, 200.0, 10.0):
            raise ValueError('This analysis contract targets the verified historical motor-7 ranges only')
        evidence_paths |= {arguments.range_record.resolve(), arguments.handoff_log.resolve(), Path(__file__).resolve(),
                           ROOT / 'Tests/host/adrc_offline_analysis_bridge.c', ROOT / 'Tests/host/test_adrc_offline_analysis.py',
                           ROOT / 'Tests/host/adrc_interface_analysis_bridge.c', ROOT / 'tools/adrc_interface_replay.py',
                           ROOT / 'tools/tune_adrc_stop.py', ROOT / 'Tests/host/test_adrc_stop_tuning.py'}
        evidence_paths |= set((ROOT / 'App').rglob('*.h')) | set((ROOT / 'App').rglob('*.c'))
        evidence_paths |= {ROOT / 'Core/Src/freertos.c'}
        for pattern in ('*.c', '*.h', '*.ps1'):
            evidence_paths |= set((ROOT / 'Tests/host').glob(pattern))
        evidence_paths |= set((ROOT / 'Tests/host/adrc_fakes').rglob('*.h'))
        before = {str(path): digest(path) for path in sorted(evidence_paths)}
        report['input_sha256'] = before
        report['ranges'] = dict(zip(('pmax', 'vmax', 'tmax'), ranges))
        report['compiler_sha256'] = digest(arguments.compiler)
        report['historical_handoff'] = replay_historical_log(arguments.handoff_log)
        report['stage'] = 'compile_bridge'
        write_json(report_path, report)
        report['regressions'] = run_regressions(output)
        library_path = compile_bridge(arguments.compiler, generated_directory, output)
        bridge = ProductionBridge(library_path, arguments.compiler.parent, ranges)
        report['grid'] = check_grid(bridge)
        report['grid']['tmax_for_full_code_at_firmware_slew'] = 0.5 * SAMPLE_TIME * 4095 / 2
        report['grid']['vmax_for_raw_velocity_qualification'] = 0.015 * 4095 / 2
        report['handoff'] = handoff_matrix(bridge)
        replay_path = ROOT / 'tools/adrc_interface_replay.py'
        replay_spec = importlib.util.spec_from_file_location('adrc_interface_replay', replay_path)
        replay_module = importlib.util.module_from_spec(replay_spec)
        replay_spec.loader.exec_module(replay_module)
        report['production_interface'] = replay_module.run_interface_replay(bridge.library, output, feedback_code)
        report['stage'] = 'synthetic_closed_loop'
        write_json(report_path, report)
        report['scenarios'] = []
        combinations = itertools.product((1, 2), ('raw', 'position32', 'position64'), ('floor', 'nearest'),
                                         (0.5, 1.0, 2.0), (0, 2), (-0.1, 0.0, 0.1), (2.0, 4.0))
        for scenario in combinations:
            report['scenarios'].append(simulate(bridge, output, scenario))
            if len(report['scenarios']) % 72 == 0:
                write_json(report_path, report)
                print(f'OFFLINE_SCENARIOS_COMPLETED={len(report["scenarios"])}', flush=True)
        after = {str(path): digest(path) for path in sorted(evidence_paths)}
        if before != after:
            raise ValueError('Inputs changed during offline analysis')
        report['inputs_unchanged'] = True
        report['scenario_count'] = len(report['scenarios'])
        report['library_sha256'] = digest(library_path)
        report['analysis_completed'] = True
        report['stage'] = 'completed'
        (output / 'summary.md').write_text(render_summary(report), encoding='utf-8')
        report['output_sha256'] = {str(path.relative_to(output)): digest(path)
                                   for path in sorted(output.rglob('*')) if path.is_file() and path != report_path}
        write_json(report_path, report)
        print(f'OFFLINE_ANALYSIS_COMPLETED scenarios={report["scenario_count"]} hardware_qualified=false output={output}', flush=True)
    except Exception as error:
        report['analysis_completed'] = False
        report['error'] = f'{type(error).__name__}: {error}'
        write_json(report_path, report)
        raise


if __name__ == '__main__':
    main()
