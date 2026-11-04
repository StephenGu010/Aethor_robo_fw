"""Audit the quantized stopping guard against production C without opening devices.

Preserve protective failures as failures of normal completion. A finite synthetic
matrix is not a continuous physical-speed certificate or hardware qualification.
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
import analyze_adrc_quantization as analysis
import adrc_interface_replay as replay_module
import tune_adrc_stop as tuning_module


def main():
    """Compile current sources and retain strict unchanged truth criteria and failed traces."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-run', type=Path, required=True)
    parser.add_argument('--wc', type=float)
    parser.add_argument('--wo', type=float)
    parser.add_argument('--mode', type=int, choices=(1, 2))
    parser.add_argument('--full', action='store_true')
    parser.add_argument('--scan', action='store_true')
    parser.add_argument('--holdout', action='store_true', help='Include fresh combined delay/disturbance cases after frozen selection')
    arguments = parser.parse_args()
    output = analysis.ROOT / 'output/adrc/dwell' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '_' + uuid4().hex[:8])
    output.mkdir(parents=True)
    generated, inputs = analysis.verify_model_run(arguments.model_run.resolve())
    inputs = set(inputs)
    for directory in ('App', 'Tests/host', 'tools'):
        inputs.update(path for path in (analysis.ROOT / directory).rglob('*')
                      if path.suffix in ('.c', '.h', '.py') and 'build' not in path.parts)
    hashes = {str(path): analysis.digest(path) for path in sorted(inputs)}
    compiler = Path('D:/application/WinLibs/mingw64/bin/gcc.exe')
    library_path = analysis.compile_bridge(compiler, generated, output)
    bridge = analysis.ProductionBridge(library_path, compiler.parent, (12.5, 200, 10))
    replay = replay_module.InterfaceReplay(bridge.library)
    groups = {'original': {}, 'known_counterexample_group': {
        'disturbance_start': 1.21, 'disturbance_end': 1.51, 'disturbance': 0.06}}
    if arguments.full:
        groups.update({'actuator1': {'torque_delay_ticks': 1}, 'actuator2': {'torque_delay_ticks': 2},
            'phase289': {'stop_tick': 289}, 'phase307': {'stop_tick': 307}, 'phase311': {'stop_tick': 311},
            'negative_disturbance': {'disturbance_start': 1.25, 'disturbance_end': 1.55, 'disturbance': -0.1},
            'damping_low': {'damping': 0.75}, 'damping_high': {'damping': 1.25},
            'disturbance118': {'disturbance_start': 1.18, 'disturbance_end': 1.48, 'disturbance': 0.08},
            'new_phase293': {'stop_tick': 293}, 'new_phase313': {'stop_tick': 313},
            'new_disturbance': {'disturbance_start': 1.234, 'disturbance_end': 1.534, 'disturbance': -0.07}})
    if arguments.holdout:
        groups.update({
            'holdout_combined_positive': dict(stop_tick=301, torque_delay_ticks=1, damping=0.9,
                disturbance=0.075, disturbance_start=1.206, disturbance_end=1.466),
            'holdout_combined_negative': dict(stop_tick=297, torque_delay_ticks=2, damping=1.1,
                disturbance=-0.085, disturbance_start=1.182, disturbance_end=1.478)})
    report = {'hardware_connected': False, 'hardware_qualified': False, 'source_sha256': hashes,
              'scope': 'discrete_synthetic_truth_only', 'validation': [],
              'fixed_protection': {'stop_timeout_us': 500000, 'dwell_us': 100000,
                                   'true_speed_limit_rad_s': 0.025, 'torque_limit_nm': 0.1}}
    report['candidates'] = []
    report['selected'] = {}
    report['negative_tests'] = []
    for mode in ([arguments.mode] if arguments.mode else [1, 2]):
        tuning = (arguments.wc, arguments.wo) if arguments.wc is not None else replay_module.OFFLINE_TUNING[mode]
        if arguments.scan:
            candidates = ([(5 + index * 0.25, 20) for index in range(11)] if mode == 1 else
                          [(11 + index * 0.5, 11 + index * 0.5 + extra)
                           for index in range(11) for extra in (0, 0.5, 1, 2, 4)])
            accepted = []
            for candidate in candidates:
                candidate_results = []
                for name in ('original', 'known_counterexample_group'):
                    result = tuning_module.evaluate(replay, output, candidate,
                        [case for case in tuning_module.original_scenarios() if case[0] == mode], model=groups[name])
                    candidate_results.append(result)
                    if not result['all_passed']:
                        break
                passed = all(result['all_passed'] for result in candidate_results)
                report['candidates'].append({'mode': mode, 'tuning': candidate, 'passed': passed,
                                             'groups': candidate_results})
                print(f'SCAN mode={mode} tuning={candidate} passed={passed}', flush=True)
                if passed:
                    accepted.append((min(result['minimum_margin_us'] for result in candidate_results), candidate))
                analysis.write_json(output / 'progress.json', report)
            if accepted:
                tuning = sorted(accepted, reverse=True)[0][1]
        report['selected'][mode] = tuning
        for name, model in groups.items():
            directory = output / f'mode{mode}_{name}'
            directory.mkdir()
            result = tuning_module.evaluate(replay, directory, tuning,
                [case for case in tuning_module.original_scenarios() if case[0] == mode],
                model=model, write_trace=True)
            result.update({'mode': mode, 'group': name})
            report['validation'].append(result)
            print(f"mode={mode} group={name} strict={result['strict_count']}/72 normal={result['normal_count']}", flush=True)
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
            result['trace_path'] = str(directory.relative_to(output) / result['trace_path'])
            report['negative_tests'].append(result)
    report['all_passed'] = all(group['all_passed'] for group in report['validation']) and all(
        case['negative_test_passed'] for case in report['negative_tests'])
    report['sources_unchanged'] = all(analysis.digest(path) == value for path, value in hashes.items())
    report['output_sha256'] = {str(path.relative_to(output)): analysis.digest(path)
                             for path in sorted(output.rglob('*')) if path.is_file()}
    analysis.write_json(output / 'report.json', report)
    print('DWELL_REPORT=' + str(output / 'report.json'), flush=True)
    raise SystemExit(0 if report['all_passed'] and report['sources_unchanged'] else 2)


if __name__ == '__main__':
    main()
