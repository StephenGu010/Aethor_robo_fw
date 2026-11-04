"""Build the actual LVGL precision-label probe and preserve a named RED/GREEN receipt without hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parents[1]


def digest(path):
    """Hash one exact local file for the immutable UI verification receipt."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Compile maintained LVGL sources and test rendered labels, geometry, glyphs and flush behavior."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--phase', choices=('red', 'green'), required=True)
    args = parser.parse_args()
    assert args.label and all(character.isalnum() or character in '_-' for character in args.label)
    output = DIRECTORY / args.label
    output.mkdir(exist_ok=False)
    sources = [line for line in (ROOT / 'Ui/lvgl_sources.txt').read_text().splitlines()
        if line and not line.startswith('#')]
    sources += ['Ui/debug_ui_view.c', 'Ui/debug_ui_graphics_guard.c', 'Ui/lv_port_disp.c',
        'Ui/lv_port_indev.c', 'Ui/fonts/ui_font_16.c', 'App/DebugUi/debug_ui_model.c',
        (DIRECTORY / 'static_gain_render.c').as_posix()]
    options = ['-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-O1', '-g',
        '-DLV_CONF_INCLUDE_SIMPLE', '-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1',
        '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1',
        '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1', '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1',
        '-IUi', '-ITests/ui', '-IMiddlewares/Third_Party/LVGL', '-IApp/DebugUi', *sources,
        '-lm', '-o', (output / 'render.exe').as_posix()]
    response = output / 'compile.rsp'
    response.write_text('\n'.join('"' + item + '"' for item in options), encoding='utf-8')
    environment = os.environ.copy()
    environment['PATH'] = 'D:/application/WinLibs/mingw64/bin;' + environment.get('PATH', '')
    compiled = subprocess.run(['D:/application/WinLibs/mingw64/bin/gcc.exe', '@' + response.as_posix()],
        cwd=ROOT, env=environment, capture_output=True, text=True, timeout=180)
    (output / 'compile.log').write_text(compiled.stdout + compiled.stderr, encoding='utf-8')
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(output / 'render.exe'), str(output), str(ROOT / 'Ui/fonts/charset.txt')],
        cwd=ROOT, env=environment, capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=30)
    (output / 'execution.log').write_text(executed.stdout + executed.stderr, encoding='utf-8')
    expected = (executed.returncode == 2 and 'PRECISION_LABEL_RED' in executed.stderr) if args.phase == 'red' else (
        executed.returncode == 0 and 'STATIC_GAIN_LVGL_PASS' in executed.stdout)
    receipt = dict(phase=args.phase, matched_expected_result=expected, compile_exit_code=compiled.returncode,
        execution_exit_code=executed.returncode, view_sha256=digest(ROOT / 'Ui/debug_ui_view.c'),
        test_sha256=digest(DIRECTORY / 'static_gain_render.c'), execution_log_sha256=digest(output / 'execution.log'),
        hardware_access=False)
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    print(json.dumps(receipt), flush=True)
    assert expected, executed.stdout + executed.stderr


if __name__ == '__main__':
    main()
