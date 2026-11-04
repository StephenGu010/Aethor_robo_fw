"""Tune existing controller parameters against production C without changing protection or devices.

The original 144 scenarios remain fixed. Physical-truth checks use only synthetic
plant state and must never be described as measured hardware acceptance.
"""

import argparse
from datetime import datetime, timezone
import itertools
from pathlib import Path

import analyze_adrc_quantization as analysis
import adrc_interface_replay as replay_module


def original_scenarios():
    """Keep every original target, mode, gain, delay and quantization assumption."""
    return list(itertools.product((1, 2), (-0.1, -0.05, 0.05, 0.1),
                                  (0.5, 1.0, 2.0), (0, 1, 2), ('floor', 'nearest')))


def evaluate(replay, output, tuning, scenarios, *, model=None, write_trace=False):
    """Retain failed scenarios and distinguish normal state from physical-truth/track acceptance."""
    results = []
    failures = []
    for scenario in scenarios:
        try:
            results.append(replay_module.run_scenario(replay, output, analysis.feedback_code,
                scenario, tuning=tuning, model=model, write_trace=write_trace))
        except (AssertionError, ValueError) as error:
            failures.append({'scenario': list(scenario), 'error': str(error)})
    complete = len(results) == len(scenarios) and not failures
    return {'tuning_wc_wo': list(tuning), 'case_count': len(scenarios),
            'normal_count': sum(case['normal_completion_in_model'] for case in results),
            'strict_count': sum(case['strict_completion_in_model'] for case in results),
            'all_passed': complete and all(case['strict_completion_in_model'] for case in results),
            'minimum_margin_us': min((case['stop_margin_us'] for case in results
                                      if case['stop_margin_us'] is not None), default=-1),
            'worst_tracking_rmse_rad_s': max((case['running_tail_rmse_rad_s'] for case in results), default=None),
            'worst_physical_dwell_rad_s': max((case['physical_dwell_peak_rad_s'] for case in results), default=None),
            'failures': failures, 'scenarios': results}


def main():
    """Compile fresh C, reproduce all 43 failures, optionally tune and verify frozen candidates."""
    if not __debug__:
        raise RuntimeError('Offline validation requires assertions; do not use Python -O')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-run', type=Path, required=True)
    parser.add_argument('--compiler', type=Path, default=Path('D:/application/WinLibs/mingw64/bin/gcc.exe'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify-only', action='store_true', help='Recheck frozen parameters without repeating the search')
    arguments = parser.parse_args()
    output = arguments.output or analysis.ROOT / 'output/adrc/stop_tuning' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output.mkdir(parents=True, exist_ok=False)
    generated, model_inputs = analysis.verify_model_run(arguments.model_run.resolve())
    inputs = set(model_inputs)
    for directory in ('App', 'Tests/host'):
        inputs.update(path for path in (analysis.ROOT / directory).rglob('*')
                      if path.suffix in ('.c', '.h') and 'build' not in path.parts)
    inputs.update((Path(__file__).resolve(), Path(analysis.__file__), Path(replay_module.__file__),
                   analysis.ROOT / 'Tests/host/test_adrc_stop_tuning.py'))
    hashes = {str(path): analysis.digest(path) for path in sorted(inputs)}
    library_path = analysis.compile_bridge(arguments.compiler, generated, output)
    bridge = analysis.ProductionBridge(library_path, arguments.compiler.parent, (12.5, 200, 10))
    replay = replay_module.InterfaceReplay(bridge.library)
    scenarios = original_scenarios()
    baseline_directory = output / 'baseline'
    baseline_directory.mkdir()
    baseline = evaluate(replay, baseline_directory, (4, 20), scenarios, write_trace=True)
    if baseline['normal_count'] != 101 or len(baseline['scenarios']) != 144:
        raise AssertionError('Original 43-timeout baseline did not reproduce exactly')
    report = {'stage': 'scanning', 'hardware_connected': False, 'hardware_qualified': False,
              'source_sha256': hashes, 'compiler_sha256': analysis.digest(arguments.compiler),
              'baseline': baseline, 'candidates': [], 'selected': dict(replay_module.OFFLINE_TUNING),
              'fixed_protection': {'stop_timeout_us': 500000, 'low_speed_dwell_us': 100000,
                  'near_zero_rad_s': 0.025, 'torque_limit_nm': 0.1, 'torque_slew_nm_s': 1.25}}
    report_path = output / 'tuning_report.json'
    groups = {'original': {}, 'actuator1': {'torque_delay_ticks': 1},
              'actuator2': {'torque_delay_ticks': 2}, 'stop_phase': {'stop_tick': 289},
              'stop_disturbance': {'disturbance_start': 1.25, 'disturbance_end': 1.55, 'disturbance': -0.1},
              'damping_low': {'damping': 0.75}, 'damping_high': {'damping': 1.25},
              'phase307': {'stop_tick': 307},
              'disturbance_118': {'disturbance_start': 1.18, 'disturbance_end': 1.48, 'disturbance': 0.08}}
    if not arguments.verify_only:
        for mode in (1, 2):
            candidates = ([(wc, 20) for wc in (6, 6.5, 7)] if mode == 1 else
                [(9.75 + 0.25 * control_index, 10.75 + 0.25 * observer_index)
                 for control_index in range(9) for observer_index in range(15)
                 if 10.75 + 0.25 * observer_index >= 9.75 + 0.25 * control_index])
            accepted = []
            for tuning in candidates:
                candidate = {'mode': mode, 'tuning': list(tuning), 'groups': [], 'all_passed': True}
                for name in ('actuator2', 'disturbance_118', 'original', 'actuator1', 'stop_phase',
                             'stop_disturbance', 'damping_low', 'damping_high', 'phase307'):
                    result = evaluate(replay, output, tuning, [case for case in scenarios if case[0] == mode],
                                      model=groups[name])
                    result['group'] = name
                    candidate['groups'].append(result)
                    if not result['all_passed']:
                        candidate['all_passed'] = False
                        break  # Explicit rejection; remaining groups are not represented as passed.
                report['candidates'].append(candidate)
                if candidate['all_passed']:
                    accepted.append(candidate)
                print(f"SCAN mode={mode} tuning={tuning} accepted={candidate['all_passed']}", flush=True)
                analysis.write_json(report_path, report)
            accepted.sort(key=lambda item: (-min(group['minimum_margin_us'] for group in item['groups']),
                max(group['worst_tracking_rmse_rad_s'] for group in item['groups']), item['tuning']))
            if not accepted:
                raise AssertionError('No acceptable candidate for mode ' + str(mode))
            report['selected'][mode] = accepted[0]['tuning']
    # These conditions did not participate in the original bounded search.
    groups.update({'holdout_phase': {'stop_tick': 311},
                   'holdout_disturbance': {'disturbance_start': 1.21, 'disturbance_end': 1.51, 'disturbance': 0.06}})
    report['validation'] = []
    report['negative_tests'] = []
    for mode in (1, 2):
        tuning = report['selected'][mode]
        for name, model in groups.items():
            directory = output / f'mode{mode}_{name}'
            directory.mkdir()
            result = evaluate(replay, directory, tuning, [case for case in scenarios if case[0] == mode],
                              model=model, write_trace=True)
            result.update({'mode': mode, 'group': name})
            report['validation'].append(result)
            print(f"VERIFY mode={mode} group={name} strict={result['strict_count']}/72 "
                  f"margin_us={result['minimum_margin_us']}", flush=True)
        for name, model in (('never_stops', {'hold_velocity_on_stop': True}),
                            ('never_confirms', {'disable_ack_delay_ticks': 1000})):
            directory = output / f'mode{mode}_{name}'
            directory.mkdir()
            result = replay_module.run_scenario(replay, directory, analysis.feedback_code,
                (mode, 0.1, 1.0, 0, 'floor'), tuning=tuning, model=model,
                require_final_disable=name != 'never_confirms')
            result['negative_test_passed'] = (result['first_fault'] == 10 and result['final_state'] == 4 and
                result['first_fault_delay_us'] == 500000 and not result['normal_completion_in_model'] and
                (name != 'never_confirms' or not result['disabled_confirmed_in_model']))
            report['negative_tests'].append(result)
    original_results = {case['name']: case for group in report['validation'] if group['group'] == 'original'
                        for case in group['scenarios']}
    report['original_43_resolved'] = [case['name'] for case in baseline['scenarios'] if case['first_fault'] == 10
                                    and original_results[case['name']]['strict_completion_in_model']]
    report['original_regression_passed'] = (len(report['original_43_resolved']) == 43 and
        all(group['all_passed'] for group in report['validation'] if group['group'] == 'original'))
    report['training_passed'] = all(group['all_passed'] for group in report['validation']
                                    if not group['group'].startswith('holdout_'))
    report['holdout_passed'] = all(group['all_passed'] for group in report['validation']
                                   if group['group'].startswith('holdout_'))
    report['negative_tests_passed'] = all(case['negative_test_passed'] for case in report['negative_tests'])
    report['all_passed'] = all(report[key] for key in ('original_regression_passed', 'training_passed',
                                                      'holdout_passed', 'negative_tests_passed'))
    report['stage'] = 'completed' if report['all_passed'] else 'validation_failed'
    report['sources_unchanged'] = all(analysis.digest(path) == value for path, value in hashes.items())
    report['library_sha256'] = analysis.digest(library_path)
    report['output_sha256'] = {str(path.relative_to(output)): analysis.digest(path)
                              for path in sorted(output.rglob('*')) if path.is_file() and path != report_path}
    analysis.write_json(report_path, report)
    if not report['sources_unchanged']:
        raise AssertionError('Inputs changed during tuning')
    if not report['all_passed']:
        print(f'TUNING_VALIDATION_FAILED original_passed={report["original_regression_passed"]} '
              f'holdout_passed={report["holdout_passed"]} report={report_path}', flush=True)
        raise SystemExit(2)
    print(f'TUNING_VERIFIED selected={report["selected"]} report={report_path}', flush=True)


if __name__ == '__main__':
    main()
