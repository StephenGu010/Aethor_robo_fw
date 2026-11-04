#!/usr/bin/env python3
"""File: persistence_joint_platform_runner.py. Real App/Core/Platform host evidence; never opens hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

from persistence_core_bridge_runner import build_extracted_source
from persistence_core_clock_runner import DEFINITIONS, DEPENDENCIES, INCLUDES


def digest(path: Path) -> str:
    """Identify the exact source bytes consumed by this offline experiment."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str], project: Path, environment: dict[str, str], log: Path) -> dict:
    """Retain one bounded native command and all output without shell interpretation."""
    result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True, timeout=60)
    content = 'COMMAND ' + subprocess.list2cmdline(command) + '\n' + result.stdout + result.stderr
    content += f'\nEXIT_CODE {result.returncode}\n'
    log.write_text(content, encoding='utf-8')
    print(content, flush=True)
    return {'command': command, 'exit_code': result.returncode, 'log': str(log)}


def main() -> int:
    """Compile real integrated App/Platform and verbatim Core against synthetic HAL/RTOS only."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=r'D:\application\WinLibs\mingw64\bin\gcc.exe')
    parser.add_argument('--label', default='GREEN')
    cases = ('readonly', 'deferred', 'debt', 'final', 'mutex', 'health_reject', 'typed', 'typed_reject', 'stop_scope',
        'pm_uoff', 'pm_baseline', 'pm_saveverify', 'health_same_tick')
    parser.add_argument('--case', default='all', choices=('all',) + cases)
    parser.add_argument('--platform-source', type=Path, help='Read-only archived Platform source for behavioral RED replay.')
    parser.add_argument('--app-inc-source', type=Path, help='Read-only archived private App include for actual RID bug replay.')
    parser.add_argument('--output-root', type=Path, help='Explicit new evidence root; the original default is unchanged.')
    arguments = parser.parse_args()
    if not arguments.label or any(character not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for character in arguments.label):
        parser.error('--label must be a simple new directory name')
    project = Path(__file__).resolve().parents[2]
    output_root = arguments.output_root if arguments.output_root else Path('output/repair_20261023/persistence_joint_platform/v3')
    output = project / output_root / arguments.label
    output.mkdir(parents=True, exist_ok=False)
    dependencies = DEPENDENCIES + ['App/Platform/can_rx_inbox.c', 'App/Platform/usb_cdc_stream.c']
    tracked = ['Core/Src/freertos.c', 'App/aethor_app.c', 'App/Motor/motor_persistence_app.inc',
        'App/Motor/motor_persistence_app_port.h', 'App/Platform/stm32_platform.c',
        'Tests/host/persistence_joint_platform_test.c', 'Tests/host/persistence_joint_platform_runner.py',
        'Tests/host/adrc_lcd_integration_test_main.c'] + dependencies
    tracked += [str(path.relative_to(project)).replace('\\', '/') for path in (project / 'App').rglob('*')
        if path.suffix in ('.h', '.inc')]
    tracked += [str(path.relative_to(project)).replace('\\', '/') for path in
        (project / 'Tests/host/single_static_platform_fakes').glob('*.h')]
    tracked += ['Tests/host/persistence_core_bridge_runner.py', 'Tests/host/persistence_core_clock_runner.py']
    tracked = sorted(set(tracked))
    before = {name: digest(project / name) for name in tracked}
    selected_inc = arguments.app_inc_source.resolve() if arguments.app_inc_source else project / 'App/Motor/motor_persistence_app.inc'
    selected_inc_before = digest(selected_inc)
    initial = output.parent / 'before'
    if not initial.exists():
        for name in ('Core/Src/freertos.c', 'App/aethor_app.c', 'App/Motor/motor_persistence_app.inc',
                'App/Motor/motor_persistence_app_port.h', 'App/Motor/motor_parameter_persistence.c',
                'App/Platform/stm32_platform.c', 'Tests/host/persistence_joint_platform_test.c',
                'Tests/host/persistence_joint_platform_runner.py'):
            target = initial / name
            target.parent.mkdir(parents=True, exist_ok=True)
            source = selected_inc if name == 'App/Motor/motor_persistence_app.inc' else project / name
            target.write_bytes(source.read_bytes())
        old_fixture = output.parent.parent / 'before/Tests/host/persistence_joint_platform_test.c'
        if old_fixture.exists():
            (initial / 'previous_joint_readonly_fixture.c').write_bytes(old_fixture.read_bytes())
        for name in ('App/Motor/motor_persistence_wire.inc',):
            if (project / name).exists():
                target = initial / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((project / name).read_bytes())
        (initial / 'source_manifest.json').write_text(json.dumps(before, indent=2), encoding='utf-8')
    archive_manifest = initial / 'archived_files_sha256.json'
    if not archive_manifest.exists():
        archived = {path.relative_to(initial).as_posix(): digest(path) for path in initial.rglob('*')
            if path.is_file() and path.suffix != '.json'}
        archive_manifest.write_text(json.dumps({'scope': 'Exact copied before bytes; source_manifest.json separately records the observed current worktree before any selected archived include overlay.',
            'archived_files': archived}, indent=2), encoding='utf-8')
    for name in ('Tests/host/persistence_joint_platform_test.c', 'Tests/host/persistence_joint_platform_runner.py',
            'App/aethor_app.c', 'App/Motor/motor_persistence_app.inc', 'App/Motor/motor_persistence_wire.inc'):
        source = selected_inc if name == 'App/Motor/motor_persistence_app.inc' else project / name
        if source.exists():
            target = output / 'source_snapshot' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
    build_extracted_source(project, output)
    environment = os.environ.copy()
    environment['PATH'] = str(Path(arguments.compiler).parent) + os.pathsep + environment.get('PATH', '')
    executable = output / 'persistence_joint_platform.exe'
    command = [arguments.compiler, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror']
    command += ['-D' + definition for definition in DEFINITIONS]
    command += ['-ITests/host/single_static_platform_fakes']
    command += ['-I' + directory for directory in INCLUDES] + ['-I' + str(output)]
    test_source = project / 'Tests/host/persistence_joint_platform_test.c'
    platform = arguments.platform_source.resolve() if arguments.platform_source else project / 'App/Platform/stm32_platform.c'
    if arguments.platform_source or arguments.app_inc_source:
        translation = test_source.read_text(encoding='utf-8')
        # Relocated overlay translation units must still include the selected real Platform source.
        original = '#include "../../App/Platform/stm32_platform.c"'
        assert translation.count(original) == 1
        translation = translation.replace(original, '#include "' + platform.as_posix() + '"')
        if arguments.app_inc_source:
            app = (project / 'App/aethor_app.c').read_text(encoding='utf-8')
            original = '#include "Motor/motor_persistence_app.inc"'
            assert app.count(original) == 1
            archived_app = output / 'joint_archived_app.c'
            archived_app.write_text(app.replace(original, '#include "' + selected_inc.as_posix() + '"'), encoding='utf-8')
            fixture = (project / 'Tests/host/adrc_lcd_integration_test_main.c').read_text(encoding='utf-8')
            original = '#include "../../App/aethor_app.c"'
            assert fixture.count(original) == 1
            archived_fixture = output / 'joint_archived_fixture.c'
            archived_fixture.write_text(fixture.replace(original, '#include "' + archived_app.as_posix() + '"'), encoding='utf-8')
            original = '#include "adrc_lcd_integration_test_main.c"'
            assert translation.count(original) == 1
            translation = translation.replace(original, '#include "' + archived_fixture.as_posix() + '"')
        test_source = output / 'joint_archived_translation.c'
        test_source.write_text(translation, encoding='utf-8')
    command += [str(test_source)] + dependencies + ['-lm', '-o', str(executable)]
    results = [run(command, project, environment, output / 'compile.log')]
    if results[0]['exit_code'] == 0:
        selected_cases = cases if arguments.case == 'all' else (arguments.case,)
        for case in selected_cases:
            results.append(run([str(executable), case], project, environment, output / (case + '.log')))
    after = {name: digest(project / name) for name in tracked}
    manifest = {'label': arguments.label, 'definitions': DEFINITIONS, 'source_before': before,
        'source_after': after, 'sources_stable_during_run': before == after and selected_inc_before == digest(selected_inc), 'results': results,
        'core_extract_sha256': digest(output / 'persistence_core_extracted.inc'),
        'platform_under_test': str(platform), 'platform_under_test_sha256': digest(platform),
        'app_inc_under_test': str(selected_inc), 'app_inc_under_test_sha256': selected_inc_before,
        'scope': 'Real App + verbatim Core helpers/control loop/RX + real Platform; HAL registers/RTOS/USB event word/empty ADRC channel synthetic. No physical board, USB stack, ARM image or EEPROM proof.'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return int(before != after or selected_inc_before != digest(selected_inc) or any(result['exit_code'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
