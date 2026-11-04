"""Compile real LVGL widgets and validate mode-2 idle labels without hardware."""
from pathlib import Path
import os
import argparse
import subprocess
import sys


def main():
    """Retain compiler, assertion and pixel evidence using maintained production UI sources."""
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='mode2-lvgl-engineering')
    arguments = parser.parse_args()
    output = root / 'output/idle-feedback' / arguments.label
    output.mkdir(parents=True, exist_ok=True)
    sources = [line for line in (root / 'Ui/lvgl_sources.txt').read_text().splitlines()
               if line and not line.startswith('#')]
    sources += ['Ui/debug_ui_view.c', 'Ui/debug_ui_graphics_guard.c', 'Ui/lv_port_disp.c',
                'Ui/lv_port_indev.c', 'Ui/fonts/ui_font_16.c', 'App/DebugUi/debug_ui_model.c',
                'Tests/ui/idle_position_render.c']
    flags = ['-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-O1',
             '-DLV_CONF_INCLUDE_SIMPLE', '-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1',
             '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1',
             '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1', '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1',
             '-IUi', '-IMiddlewares/Third_Party/LVGL', '-IApp/DebugUi']
    executable = output / 'render.exe'
    environment = dict(os.environ, PATH='D:/application/WinLibs/mingw64/bin;' + os.environ['PATH'])
    built = subprocess.run(['D:/application/WinLibs/mingw64/bin/gcc.exe', *flags, *sources,
                            '-lm', '-o', str(executable)], cwd=root, env=environment, capture_output=True)
    (output / 'build.log').write_bytes(built.stdout + built.stderr)
    if built.returncode:
        print((built.stdout + built.stderr).decode(errors='replace'))
        return built.returncode
    run = subprocess.run([str(executable), str(output), str(root / 'Ui/fonts/charset.txt')],
                         cwd=root, env=environment, capture_output=True)
    (output / 'test.log').write_bytes(run.stdout + run.stderr)
    print((run.stdout + run.stderr).decode(errors='replace'))
    return run.returncode


if __name__ == '__main__':
    sys.exit(main())
