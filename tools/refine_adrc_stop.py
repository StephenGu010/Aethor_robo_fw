"""Refine bandwidths against retained failures without changing any protection.

All prior scenarios become regression/training data. Fresh combined cases are
held out until selection finishes; neither set grants physical qualification.
"""
import argparse
import csv
import hashlib
import itertools
import json
from datetime import datetime, timezone
from pathlib import Path

import analyze_adrc_quantization as analysis
import adrc_interface_replay as interface


def load_cases(directory):
    """Check retained bridge and trace hashes and retain each declared model verbatim."""
    report = json.loads((directory / 'report.json').read_text())
    assert report['sources_unchanged']
    assert analysis.digest(directory / 'offline_analysis.dll') == report['output_sha256']['offline_analysis.dll']
    cases = []
    for group in report['validation']:
        for case in group['scenarios']:
            scenario = (case['mode'], case['target_rad_s'], case['plant_b0_ratio'],
                        case['feedback_delay_samples'], case['rounding_assumption'])
            cases.append((group['group'], scenario, case['model_parameters'], case))
    return report, cases


def evaluate(replay, output, candidate, case, trace=False):
    """Run the complete production interface with the unchanged strict acceptance."""
    group, scenario, model, _ = case
    if trace:
        output = output / group
        output.mkdir(exist_ok=True)
    result = interface.run_scenario(replay, output, analysis.feedback_code, scenario,
                                    tuning=candidate, model=model, write_trace=trace)
    result['group'] = group
    if trace:
        result['trace_path'] = str(Path(group) / result['trace_path'])
    return result


def main():
    """Search existing controller bandwidths, retaining failures and independent holdouts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', type=int, choices=(1, 2), required=True)
    arguments = parser.parse_args()
    assert __debug__, 'Assertions must remain enabled'
    name = {1: '20261010T052116628787Z_63a7bf6b', 2: '20261010T052116637787Z_bbb7becc'}[arguments.mode]
    directory = analysis.ROOT / 'output/adrc/dwell' / name
    prior, cases = load_cases(directory)
    for path, digest in prior['source_sha256'].items():
        assert analysis.digest(path) == digest, ('source_changed', path)
    output = analysis.ROOT / 'output/adrc/refinement' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + f'_m{arguments.mode}')
    output.mkdir(parents=True)
    compiler = Path('D:/application/WinLibs/mingw64/bin')
    bridge = analysis.ProductionBridge(directory / 'offline_analysis.dll', compiler, (12.5, 200, 10))
    replay = interface.InterfaceReplay(bridge.library)
    failures = [case for case in cases if not case[3]['strict_completion_in_model']]
    report = {'hardware_qualified': False, 'baselineFailures': len(failures), 'search': [],
              'sourceSha256': prior['source_sha256'], 'runnerSha256': analysis.digest(__file__),
              'protection': prior['fixed_protection'], 'selected': None}
    baseline = tuple(prior['selected'][str(arguments.mode)])
    reproduced = [evaluate(replay, output, baseline, case) for case in failures]
    assert all(not case['strict_completion_in_model'] for case in reproduced)
    report['baselineReproduction'] = reproduced
    if arguments.mode == 1:
        candidates = [(4 + i*.25, 20) for i in range(49)]
    else:
        candidates = [(4 + i*.5, 4 + i*.5 + delta) for i in range(29) for delta in (0, 2, 4, 8, 12, 16)]
    # The failure pool grows with newly discovered regressions; this is training.
    pool = list(failures)
    for candidate in candidates:
        results = [evaluate(replay, output, candidate, case) for case in pool]
        failed = sum(not item['strict_completion_in_model'] for item in results)
        entry = {'candidate': candidate, 'pool': len(pool), 'failed': failed}
        report['search'].append(entry)
        if failed == 0:
            complete = True
            for case in cases:
                result = evaluate(replay, output, candidate, case)
                if not result['strict_completion_in_model']:
                    if case not in pool:
                        pool.append(case)
                    entry['newRegression'] = result
                    complete = False
                    break
            if complete:
                report['selected'] = candidate
                break
        analysis.write_json(output / 'progress.json', report)
        print(f'mode={arguments.mode} candidate={candidate} failed={failed}/{len(results)}', flush=True)
    if report['selected'] is not None:
        candidate = report['selected']
        report['regression'] = [evaluate(replay, output, candidate, case, True) for case in cases]
        holdout = []
        for group, model in (
            ('combined_positive', dict(stop_tick=301, torque_delay_ticks=1, damping=.9, disturbance=.075, disturbance_start=1.206, disturbance_end=1.466)),
            ('combined_negative', dict(stop_tick=297, torque_delay_ticks=2, damping=1.1, disturbance=-.085, disturbance_start=1.182, disturbance_end=1.478)),
        ):
            for scenario in itertools.product((arguments.mode,), (-.1, -.05, .05, .1), (.5, 1., 2.), (0,1,2), ('floor','nearest')):
                holdout.append(evaluate(replay, output, candidate, (group, scenario, model, {}), True))
        report['holdout'] = holdout
        report['all_passed'] = all(c['strict_completion_in_model'] for c in report['regression'] + holdout)
    else:
        report['all_passed'] = False
    report['sourcesUnchanged'] = all(analysis.digest(path) == digest for path, digest in prior['source_sha256'].items())
    report['artifacts'] = {str(p.relative_to(output)): analysis.digest(p) for p in output.rglob('*') if p.is_file()}
    analysis.write_json(output / 'report.json', report)
    print('REFINEMENT_REPORT=' + str(output / 'report.json'), flush=True)
    raise SystemExit(0 if report['all_passed'] and report['sourcesUnchanged'] else 2)


if __name__ == '__main__':
    main()
