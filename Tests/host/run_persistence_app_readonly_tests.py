"""Build and run Stage A against production App data and paths; synthetic observations only."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def regression_pair(project: Path, output: Path, compiler: str, dependencies: list[str],
                    includes: tuple[str, ...], environment: dict[str, str], suite: tuple) -> dict:
    """Compile unchanged health cases twice, selecting only the saved-before or current facade include."""
    name, source_name, definitions = suite
    text = (project / source_name).read_text(encoding="utf-8")
    fixture_name = "Tests/host/debug_ui_app_test_main.c"
    if '#include "debug_ui_app_test_main.c"' in text:
        text = text.replace('#include "debug_ui_app_test_main.c"',
            (project / fixture_name).read_text(encoding="utf-8"))
    results = []
    for variant in ("before", "after"):
        facade = output / "before/aethor_app.c" if variant == "before" else project / "App/aethor_app.c"
        generated = text.replace('#include "../../App/aethor_app.c"', f'#include "{facade.as_posix()}"')
        generated = re.sub(r'#include "\.\./\.\./App/([^\"]+)"',
            lambda match: f'#include "{(project / "App" / match.group(1)).as_posix()}"', generated)
        translation = output / f"health_{name}_{variant}.c"
        executable = output / f"health_{name}_{variant}.exe"
        translation.write_text(generated, encoding="utf-8")
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        command += [f"-D{value}" for value in definitions]
        command += [f"-I{value}" for value in includes] + ["-ITests/host"]
        command += [str(translation)] + dependencies + ["-lm", "-o", str(executable)]
        result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True)
        compiler_output = result.stdout + result.stderr
        run_output = ""
        exit_code = result.returncode
        if not exit_code:
            result = subprocess.run([str(executable)], cwd=project, env=environment, capture_output=True, text=True)
            run_output = result.stdout + result.stderr
            exit_code = result.returncode
        (output / f"health_{name}_{variant}.log").write_text(
            "COMMAND " + subprocess.list2cmdline(command) + "\n" + compiler_output + run_output +
            f"\nEXIT_CODE {exit_code}\n", encoding="utf-8")
        normalized = re.sub(r"file .*?, line (\d+)", r"file <unchanged-fixture>, line \1", run_output)
        normalized_compiler = compiler_output.replace(str(translation), "<unchanged-fixture>")
        results.append({"variant": variant, "exit_code": exit_code, "output": normalized,
            "compiler": normalized_compiler, "source_sha256": hashlib.sha256(generated.encode()).hexdigest()})
    equal = (results[0]["exit_code"] == results[1]["exit_code"] and
        results[0]["output"] == results[1]["output"] and results[0]["compiler"] == results[1]["compiler"])
    print(f"HEALTH_PAIR {name} before={results[0]['exit_code']} after={results[1]['exit_code']} exact_equal={equal}", flush=True)
    return {"suite": name, "definitions": definitions, "same_result": equal, "results": results}


def run_regressions(project: Path, output: Path, compiler: str, sources: list[str],
                    includes: tuple[str, ...], environment: dict[str, str]) -> int:
    """Attribute unchanged regression outcomes against the original dirty facade without swapping workspace files."""
    base = ("AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1")
    static = base + ("AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1")
    integrated = base + ("AETHOR_ADRC_LCD_INTEGRATED=1", "AETHOR_DEBUG_UI_ALLOW_MIT=1",
        "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1", "AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1",
        "AETHOR_S3519_SAME_MODEL_MASK=0x7F", "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F")
    suites = (
        ("default_locked", "Tests/host/debug_ui_app_test_main.c", ("AETHOR_DEBUG_UI_ENABLE=1",)),
        ("integrated_default", "Tests/host/adrc_lcd_integration_test_main.c", integrated),
        ("pos_locked", "Tests/host/pos_load_app_test.c", base + ("AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=0",)),
        ("pos_enabled", "Tests/host/pos_load_app_test.c", base + ("AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1",)),
        ("single_static", "Tests/host/single_static_load_app_test.c", static),
        ("static_peak", "Tests/host/static_peak_reason_test.c", static + ("AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1",)),
    )
    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(lambda suite: regression_pair(project, output, compiler, sources[1:],
            includes, environment, suite), suites))
    (output / "health_comparison.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0 if all(result["same_result"] and
        all(variant["exit_code"] == 0 for variant in result["results"])
        for result in results) else 1


def main() -> int:
    """Compile the complete integrated/single/POS dependency set and retain a reproducible log."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    parser.add_argument("--label", default="GREEN")
    parser.add_argument("--regressions", action="store_true")
    parser.add_argument("--output", default="output/repair_20261023/persistence_app_readonly")
    arguments = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = project / arguments.output
    output.mkdir(parents=True, exist_ok=True)
    executable = output / "persistence_app_readonly.exe"
    sources = ["Tests/host/persistence_app_readonly_test.c", "App/Motor/motor_parameter_persistence.c"]
    sources += [f"App/Adrc/{name}.c" for name in (
        "adrc_lcd_ownership", "adrc_simple_trial", "adrc_feedback_adapter", "adrc_app_bridge",
        "adrc_shaft_pilot", "adrc_shaft_pilot_adapter", "adrc_bench", "adrc_protocol", "adrc_experiment")]
    sources += [f"App/{name}.c" for name in (
        "DebugUi/debug_ui_mailbox", "Config/arm_config", "Config/build_info", "Telemetry/diagnostics",
        "Arm/arm_controller", "Arm/joint_reference", "Motion/joint_motion", "Motion/joint_motion_can",
        "Motor/can_frame", "Motor/can_tx_scheduler", "Motor/motor_bank", "Motor/motor_discovery",
        "Motor/motor_runtime", "Motor/motor_adrc_command", "Motor/motor_capability_collector",
        "Motor/s3519_codec", "Protocol/ascii_protocol", "Protocol/text_protocol", "Protocol/protocol_engine")]
    macros = ("AETHOR_ADRC_LCD_INTEGRATED=1", "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
        "AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1",
        "AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1", "AETHOR_MOTOR_PERSISTENCE_TRANSPORT=1",
        "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1", "AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1",
        "AETHOR_S3519_SAME_MODEL_MASK=0x7F", "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F")
    includes = ("App", "App/Arm", "App/Config", "App/DebugUi", "App/Motor", "App/Adrc",
        "App/Motion", "App/Protocol", "App/Telemetry", "App/Platform")
    command = [arguments.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    command += [f"-D{value}" for value in macros] + [f"-I{value}" for value in includes]
    command += sources + ["-lm", "-o", str(executable)]
    environment = os.environ.copy()
    environment["PATH"] = str(Path(arguments.compiler).parent) + os.pathsep + environment["PATH"]
    if arguments.regressions:
        return run_regressions(project, output, arguments.compiler, sources, includes, environment)
    compile_result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True)
    log = "COMMAND " + subprocess.list2cmdline(command) + "\n" + compile_result.stdout + compile_result.stderr
    exit_code = compile_result.returncode
    if not exit_code:
        result = subprocess.run([str(executable)], cwd=project, env=environment, capture_output=True, text=True)
        log += result.stdout + result.stderr
        exit_code = result.returncode
    log += f"\nEXIT_CODE {exit_code}\nSCOPE facade synthetic only; Core/Platform/HAL not linked\n"
    (output / f"{arguments.label}.log").write_text(log, encoding="utf-8")
    print(log)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
