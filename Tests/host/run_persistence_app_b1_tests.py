"""Build bounded App B1 and the real pure transaction; no Core, Platform, HAL or hardware access."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess


def main() -> int:
    """Compile a fresh native executable and retain the exact command, output and exit status."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    parser.add_argument("--label", default="GREEN")
    parser.add_argument("--output", default="output/repair_20261023/persistence_app_b1")
    arguments = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = project / arguments.output / arguments.label
    # A label identifies one immutable run; explicit reuse must not overwrite earlier evidence.
    output.mkdir(parents=True, exist_ok=False)
    executable = output / f"persistence_app_b1_{arguments.label}.exe"
    sources = ["Tests/host/persistence_app_b1_test.c", "App/Motor/motor_parameter_persistence.c"]
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
    result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True)
    log = "COMMAND " + subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr
    exit_code = result.returncode
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
