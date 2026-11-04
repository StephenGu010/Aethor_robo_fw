"""Replay the production motor interface, supervisor and generated controller without devices.

The simulated drive acknowledges commands after two control ticks. Its torque
then changes ideally; this declared model is not measured disable performance.
All discovery and qualification objects live only in the host test process.
"""

import csv
import ctypes
import itertools
import math

# Explicit offline candidates only; the independent holdout has a known truth-dwell failure.
# These values neither replace firmware defaults nor qualify any physical motor.
OFFLINE_TUNING = {1: (6.5, 20.0), 2: (10.75, 11.5)}


class InterfaceReplay:
    """Typed access to a host-only DLL containing the complete production control path."""

    def __init__(self, library):
        """Declare exported signatures without any serial, debugger or network access."""
        self.library = library
        signatures = {
            'interface_analysis_initialize': [ctypes.c_float] * 3,
            'interface_analysis_feedback': [ctypes.c_uint64, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint8],
            'interface_analysis_configure': [ctypes.c_uint64, ctypes.c_uint8, ctypes.c_float, ctypes.c_float],
            'interface_analysis_configure_tuned': [ctypes.c_uint64, ctypes.c_uint8] + [ctypes.c_float] * 3,
            'interface_analysis_diagnostics': [ctypes.POINTER(ctypes.c_double)],
            'interface_analysis_command': [ctypes.c_uint64, ctypes.c_uint8],
            'interface_analysis_service': [ctypes.c_uint64, ctypes.POINTER(ctypes.c_float)],
        }
        for name, arguments in signatures.items():
            function = getattr(library, name)
            function.argtypes = arguments
            function.restype = ctypes.c_int

    def command(self, now_us, command):
        """Require real protocol admission; an accepted request is not a completed action."""
        result = self.library.interface_analysis_command(now_us, command)
        if result != 0:
            raise AssertionError(('protocol_admission', command, now_us, result))

    def service(self, now_us):
        """Return the real frame, state and trace fields after a matched synthetic TX receipt."""
        values = (ctypes.c_float * 11)()
        if self.library.interface_analysis_service(now_us, values) != 1:
            raise AssertionError('Production interface service failed')
        return tuple(values)

    def diagnostics(self):
        """Read actual supervisor timing and generated-controller state without influencing them."""
        values = (ctypes.c_double * 8)()
        if self.library.interface_analysis_diagnostics(values) != 1:
            raise AssertionError('Production diagnostics failed')
        return tuple(values)


def run_scenario(replay, output, encode, scenario, *, tuning=(4.0, 20.0), model=None,
                 write_trace=True, require_final_disable=True):
    """Run one bounded experiment with delayed drive state and independently checked wire limits."""
    mode, target, plant_ratio, sensor_delay, rounding = scenario
    model_parameters = {'damping': 1.0, 'disturbance': 0.1, 'disturbance_start': 0.6,
                        'disturbance_end': 0.9, 'torque_delay_ticks': 0,
                        'disable_ack_delay_ticks': 2, 'stop_tick': None,
                        'hold_velocity_on_stop': False}
    if model:
        if set(model) - set(model_parameters):
            raise ValueError('Unknown synthetic model parameter')
        model_parameters.update(model)
    if (len(tuning) != 2 or not all(math.isfinite(value) for value in tuning) or
            not 0 < tuning[0] <= tuning[1] <= 250):
        raise ValueError('Invalid controller or observer bandwidth')
    for key in ('damping', 'disturbance', 'disturbance_start', 'disturbance_end'):
        if not math.isfinite(model_parameters[key]):
            raise ValueError('Nonfinite synthetic model parameter: ' + key)
    if (model_parameters['damping'] <= 0 or model_parameters['disturbance_start'] < 0 or
            model_parameters['disturbance_end'] <= model_parameters['disturbance_start']):
        raise ValueError('Invalid damping or disturbance interval')
    for key in ('torque_delay_ticks', 'disable_ack_delay_ticks'):
        if type(model_parameters[key]) is not int or model_parameters[key] < 0:
            raise ValueError('Delay must be a nonnegative integer: ' + key)
    if model_parameters['stop_tick'] is not None and (
            type(model_parameters['stop_tick']) is not int or not 75 <= model_parameters['stop_tick'] <= 316):
        raise ValueError('Explicit STOP must leave enough RUN samples before automatic STOP')
    if type(model_parameters['hold_velocity_on_stop']) is not bool:
        raise ValueError('Stopped-velocity injection must be Boolean')
    name = f'interface_m{mode}_r{target}_b{plant_ratio}_d{sensor_delay}_{rounding}'
    if replay.library.interface_analysis_initialize(12.5, 200.0, 10.0) != 1:
        raise AssertionError('Production interface initialization failed')
    position = velocity = physical_torque = previous_command = 0.0
    enabled = False
    state_changes = []
    torque_changes = []
    sensor_history = []
    rows = []
    first_fault = 0
    enable_count = torque_count = disable_count = 0
    configured = False
    maximum_step = 0.0
    for tick in range(701):
        now_us = 1000 + tick * 4000
        # Only the explicit drive model determines when DISABLE becomes observable.
        for due_tick, requested_enabled in list(state_changes):
            if due_tick <= tick:
                enabled = requested_enabled
                if not enabled:
                    physical_torque = 0.0
                    torque_changes.clear()
                state_changes.remove((due_tick, requested_enabled))
        for due_tick, requested_torque in list(torque_changes):
            if due_tick <= tick:
                if enabled:
                    physical_torque = requested_torque
                torque_changes.remove((due_tick, requested_torque))
        sensor_history.append((position, velocity, enabled, now_us))
        sample = sensor_history[max(0, tick - sensor_delay)]
        sample_position, sample_velocity, sample_enabled, sample_us = sample
        if tick >= sensor_delay:
            result = replay.library.interface_analysis_feedback(
                sample_us, encode(sample_position, 12.5, 16, rounding),
                encode(sample_velocity, 200.0, 12, rounding), sample_enabled)
            if result != 0:
                raise AssertionError(('feedback_rejected', name, tick, result))
        values = replay.service(now_us)
        diagnostics = replay.diagnostics()
        kind, decoded_torque, state, fault, _, disabled_confirmed = values[:6]
        if fault and not first_fault:
            first_fault = int(fault)
        if fault not in (0, 10) or (first_fault and fault != first_fault):
            raise AssertionError(('unexpected_or_replaced_fault', name, tick, first_fault, fault))
        if kind == 0:
            enable_count += 1
            previous_command = 0.0
            state_changes.append((tick + 2, True))
        elif kind == 1:
            torque_count += 1
            step = abs(decoded_torque - previous_command)
            maximum_step = max(maximum_step, step)
            if abs(decoded_torque) > 0.100001 or step > 0.0050001:
                raise AssertionError(('wire_torque_bound', name, tick, decoded_torque, step))
            code = int(values[10])
            if not 0 <= code <= 4095 or abs((code * 20.0 / 4095 - 10.0) - decoded_torque) > 0.000002:
                raise AssertionError(('wire_code_readback', name, tick, code, decoded_torque))
            previous_command = decoded_torque
            if enabled:
                if model_parameters['torque_delay_ticks']:
                    torque_changes.append((tick + model_parameters['torque_delay_ticks'], decoded_torque))
                else:
                    physical_torque = decoded_torque
        elif kind == 2:
            disable_count += 1
            if not any(not requested for _, requested in state_changes):
                state_changes.append((tick + model_parameters['disable_ack_delay_ticks'], False))
        if tick == 16:
            result = replay.library.interface_analysis_configure_tuned(now_us, mode, target, *tuning)
            if result != 0:
                raise AssertionError(('candidate_configure', name, tick, result))
            configured = True
            replay.command(now_us, 0)
        elif tick == 17:
            if state != 1:
                raise AssertionError(('prepare_not_completed', name, state, fault))
            replay.command(now_us, 1)
        elif tick > 18 and tick % 25 == 0 and state in (2, 3):
            replay.command(now_us, 2)
        if tick == model_parameters['stop_tick'] and state == 2:
            replay.command(now_us, 3)
        rows.append([now_us, sample_us, position, velocity, int(enabled), physical_torque,
                     kind, decoded_torque, state, fault, disabled_confirmed,
                     values[6], values[7], int(values[8]), int(values[9]), int(values[10]),
                     *diagnostics])
        disturbance = model_parameters['disturbance'] if (
            model_parameters['disturbance_start'] <= tick * 0.004 < model_parameters['disturbance_end']) else 0.0
        position += 0.004 * velocity
        if model_parameters['hold_velocity_on_stop'] and state == 3:
            velocity = math.copysign(0.08, target)
        else:
            velocity += 0.004 * (10.0 * plant_ratio * physical_torque - model_parameters['damping'] * velocity + disturbance)
        if not all(math.isfinite(value) for value in (position, velocity, physical_torque)):
            raise AssertionError(('nonfinite_plant', name, tick))
    if not configured or enable_count != 1 or torque_count == 0 or disable_count == 0:
        raise AssertionError(('incomplete_control_path', name, enable_count, torque_count, disable_count))
    if require_final_disable and (rows[-1][8] not in (0, 4) or not rows[-1][10]):
        raise AssertionError(('unconfirmed_or_active_final_state', name, rows[-1][8:11]))
    trace_path = None
    if write_trace:
        trace_path = output / 'interface_traces' / (name + '.csv')
        trace_path.parent.mkdir(exist_ok=True)
        with trace_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(('now_us', 'feedback_lower_bound_us', 'position_rad', 'velocity_rad_s',
                             'simulated_enabled', 'simulated_applied_torque_nm', 'frame_kind',
                             'decoded_command_nm', 'state', 'fault', 'disabled_confirmed',
                             'estimated_velocity_rad_s', 'raw_velocity_rad_s', 'trace_flags',
                             'trace_count', 'wire_torque_code', 'reference_rad_s', 'raw_torque_nm',
                             'z1', 'z2', 'low_speed_tracking', 'low_speed_started_us',
                             'disable_requested_us', 'stop_started_us'))
            writer.writerows(rows)
    stop_row = next(row for row in rows if row[23] != 0)
    stop_us = int(stop_row[23])
    disable_row = next(row for row in rows if row[6] == 2)
    dwell_rows = [row for row in rows if disable_row[0] - 100000 <= row[0] <= disable_row[0]]
    confirmation = next((row for row in rows if row[0] >= stop_us and row[10]), None)
    if confirmation is not None and confirmation[4] != 0:
        raise AssertionError(('confirmed_while_physical_drive_enabled', name))
    handoff_peak = max(abs(row[3]) for row in rows if row[0] >= disable_row[0] and
                       (confirmation is None or row[0] <= confirmation[0]))
    running_tail = [row for row in rows if stop_us - 200000 <= row[0] < stop_us and row[8] == 2]
    stop_duration = confirmation[0] - stop_us if confirmation else None
    physical_dwell_peak = max(abs(row[3]) for row in dwell_rows)
    running_rmse = math.sqrt(sum((row[3] - target) ** 2 for row in running_tail) / len(running_tail))
    normal = first_fault == 0 and rows[-1][8] == 0 and bool(rows[-1][10])
    return {'name': name, 'mode': mode, 'target_rad_s': target, 'plant_b0_ratio': plant_ratio,
            'feedback_delay_samples': sensor_delay, 'rounding_assumption': rounding,
            'sample_count': len(rows), 'enable_count': enable_count, 'torque_count': torque_count,
            'disable_count': disable_count, 'maximum_wire_step_nm': maximum_step,
            'first_fault': first_fault, 'final_state': int(rows[-1][8]),
            'first_fault_delay_us': next((row[0] - stop_us for row in rows if row[9]), None),
            'disabled_confirmed_in_model': bool(rows[-1][10]),
            'normal_completion_in_model': normal, 'tuning_wc_wo': list(tuning),
            'model_parameters': model_parameters, 'stop_duration_us': stop_duration,
            'stop_margin_us': 500000 - stop_duration if stop_duration is not None else None,
            'disable_request_delay_us': disable_row[0] - stop_us,
            'physical_velocity_at_disable_rad_s': disable_row[3],
            'physical_dwell_peak_rad_s': physical_dwell_peak,
            'physical_disable_handoff_peak_rad_s': handoff_peak,
            'physical_speed_after_disable_peak_rad_s': max(abs(row[3]) for row in rows if row[0] >= disable_row[0]),
            'physical_low_speed_dwell_passed': physical_dwell_peak <= 0.025,
            'running_tail_rmse_rad_s': running_rmse,
            'running_tail_mean_rad_s': sum(row[3] for row in running_tail) / len(running_tail),
            'maximum_physical_speed_rad_s': max(abs(row[3]) for row in rows),
            'maximum_physical_displacement_rad': max(abs(row[2]) for row in rows),
            'strict_completion_in_model': normal and physical_dwell_peak <= 0.025 and
                handoff_peak <= 0.025 and running_rmse <= 0.02,
            'trace_path': str(trace_path.relative_to(output)) if trace_path else None}


def run_interface_replay(library, output, encode):
    """Execute both small and larger references; distinguish completion from protective timeout."""
    replay = InterfaceReplay(library)
    scenarios = itertools.product((1, 2), (-0.1, -0.05, 0.05, 0.1), (0.5, 1.0, 2.0), (0, 1, 2), ('floor', 'nearest'))
    results = [run_scenario(replay, output, encode, scenario, tuning=OFFLINE_TUNING[scenario[0]])
               for scenario in scenarios]
    if not all(item['strict_completion_in_model'] for item in results):
        raise AssertionError('Frozen tuning no longer passes the original 144-case regression')
    return {'invariants_passed': True, 'scenario_count': len(results),
            'normal_completion_count': sum(item['normal_completion_in_model'] for item in results),
            'strict_completion_count': sum(item['strict_completion_in_model'] for item in results),
            'offline_tuning_wc_wo': OFFLINE_TUNING,
            'qualification_scope': 'original fixed matrix only; independent holdout has known truth-dwell failure',
            'hardware_qualified': False, 'synthetic_local_qualification': True,
            'controller_source': 'verified generated C via production adapter',
            'drive_state_ack_delay_ticks_assumed': 2, 'ideal_torque_after_ack_assumed': True,
            'feedback_timing': 'synthetic sample bounds; no measured bus timing uncertainty',
            'scenarios': results}
