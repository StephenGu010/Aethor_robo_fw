#!/usr/bin/env python3
"""File: persistence_core_clock_runner.py. Verify verbatim Core clock ordering with the real App offline."""

from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

from persistence_core_bridge_runner import extract_block


DEFINITIONS = (
    "AETHOR_ADRC_LCD_INTEGRATED=1", "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
    "AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1",
    "AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1", "AETHOR_MOTOR_PERSISTENCE_TRANSPORT=1",
    "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1", "AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1",
    "AETHOR_S3519_SAME_MODEL_MASK=0x7F", "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F",
)
INCLUDES = (
    "App", "App/Arm", "App/Config", "App/DebugUi", "App/Motor", "App/Adrc", "App/Motion",
    "App/Protocol", "App/Telemetry", "App/Platform", "Tests/host",
)
DEPENDENCIES = ["App/Motor/motor_parameter_persistence.c"]
DEPENDENCIES += [f"App/Adrc/{name}.c" for name in (
    "adrc_lcd_ownership", "adrc_simple_trial", "adrc_feedback_adapter", "adrc_app_bridge",
    "adrc_shaft_pilot", "adrc_shaft_pilot_adapter", "adrc_bench", "adrc_protocol", "adrc_experiment")]
DEPENDENCIES += [f"App/{name}.c" for name in (
    "DebugUi/debug_ui_mailbox", "Config/arm_config", "Config/build_info", "Telemetry/diagnostics",
    "Arm/arm_controller", "Arm/joint_reference", "Motion/joint_motion", "Motion/joint_motion_can",
    "Motor/can_frame", "Motor/can_tx_scheduler", "Motor/motor_bank", "Motor/motor_discovery",
    "Motor/motor_runtime", "Motor/motor_adrc_command", "Motor/motor_capability_collector",
    "Motor/s3519_codec", "Protocol/ascii_protocol", "Protocol/text_protocol", "Protocol/protocol_engine")]


def sha256(path: Path) -> str:
    """Hash exact bytes so RED/GREEN records identify the maintained code that was tested."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract_between(source: str, start: str, end: str) -> str:
    """Return one exact maintained region and fail closed if its surrounding anchors drift."""
    if source.count(start) != 1 or source.count(end) != 1:
        raise ValueError(f"Missing or ambiguous maintained boundaries: {start!r}, {end!r}")
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


def extract_core(core_path: Path, output: Path) -> dict[str, str]:
    """Extract actual collectors/RX and bounded Control/Protocol/UI regions without rewriting any C statement."""
    source = core_path.read_text(encoding="utf-8")
    control = extract_between(source,
        "    /* Time is sampled before FIFO inspection; critical protects the shared 64-bit bound. */",
        "#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED\n    {\n      uint8_t guardPauseMs")
    protocol = extract_block(source, "if (lineStatus == USB_CDC_STREAM_STATUS_OK)")
    ui_head = extract_between(source,
        "#if AETHOR_DEBUG_UI_ENABLE\n    aethor_app_debug_ui_process(AethorMonotonicTimestampUs());\n#endif\n    do",
        "    do\n")
    ui_tail = extract_between(source,
        "#if AETHOR_DEBUG_UI_ENABLE\n    /* Keep the sole producer responsive even under continuous USB input. */",
        "    if (protocolLinesProcessed >= 4U)") + "#endif\n"
    fragments = {
        "cancel": extract_block(source, "static void CancelPersistenceTransfer(void)"),
        "parameter_receipt": extract_block(source, "static void CollectPersistenceReceipt(uint64_t observedUs)"),
        "strict_receipt": extract_block(source, "static void CollectSingleStaticReceipt(void)"),
        "receive": extract_block(source, "static void ReceiveSingleStaticFrame("),
        "capability_receipt": extract_block(source, "static void CollectCapabilityReceipt("),
        "control_boundary": control, "protocol_line": protocol, "ui_head": ui_head, "ui_tail": ui_tail,
    }
    generated = "/** @file persistence_core_clock_extracted.inc. Verbatim maintained Core timing boundaries. */\n"
    generated += "\n".join(fragments[name] for name in (
        "cancel", "parameter_receipt", "strict_receipt", "receive", "capability_receipt"))
    generated += "\n/** @brief Execute the actual bounded collector-to-service Core region. */\n"
    generated += "static void RunExtractedControlBoundary(void) { uint64_t timestampUs;\n" + control + "\n}\n"
    generated += "/** @brief Execute the actual accepted-line Core region. */\n"
    generated += "static void RunExtractedProtocolLine(void) {\n"
    generated += 'char protocolLine[] = "1 ping"; uint16_t lineLength = (uint16_t)(sizeof(protocolLine) - 1U);\n'
    generated += "ProtocolOutputBatch outputBatch; unsigned lineStatus = 0U;\n#define USB_CDC_STREAM_STATUS_OK 0U\n"
    generated += protocol + "\n}\n"
    generated += "/** @brief Execute the actual first LCD mailbox-processing call. */\n"
    generated += "static void RunExtractedUiHead(void) {\n" + ui_head + "\n}\n"
    generated += "/** @brief Execute the actual last LCD mailbox-processing call. */\n"
    generated += "static void RunExtractedUiTail(void) {\n" + ui_tail + "\n}\n"
    extracted_path = output / "persistence_core_clock_extracted.inc"
    extracted_path.write_text(generated, encoding="utf-8")
    return {name: hashlib.sha256(fragment.encode()).hexdigest() for name, fragment in fragments.items()}


def run_logged(command: list[str], project: Path, environment: dict[str, str], log_path: Path) -> dict:
    """Run one transparent native/Python operation and retain its exact command, output and exit status."""
    result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True, timeout=60)
    log = "COMMAND " + subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr
    log += f"\nEXIT_CODE {result.returncode}\n"
    log_path.write_text(log, encoding="utf-8")
    print(log, flush=True)
    return {"command": command, "exit_code": result.returncode, "log": str(log_path)}


def verify_gate0_equivalence(before_core: Path, current_core: Path, project: Path,
                             output: Path, compiler: str, environment: dict[str, str]) -> dict:
    """Compare full Core preprocessing with only includes omitted; this proves gate0 token equivalence, not a firmware build."""
    results = []
    definitions = [value for value in DEFINITIONS if not value.startswith("AETHOR_MOTOR_PERSISTENCE_TRANSPORT=")]
    definitions.append("AETHOR_MOTOR_PERSISTENCE_TRANSPORT=0")
    hal_config = (project / "Core/Inc/stm32h7xx_hal_conf.h").read_text(encoding="utf-8")
    rtos_config = (project / "Core/Inc/FreeRTOSConfig.h").read_text(encoding="utf-8")
    tick_priority = re.search(r"#if AETHOR_ADRC_BENCH \|\| AETHOR_ADRC_LCD_INTEGRATED\n#define\s+TICK_INT_PRIORITY\s+([^\n]+)", hal_config)
    syscall_priority = re.search(r"^#define\s+configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY\s+(\d+)", rtos_config, re.MULTILINE)
    if tick_priority is None or syscall_priority is None:
        raise ValueError("The actual integrated timer/syscall priority definitions changed; re-evaluate gate0 preprocessing.")
    definitions += ["TICK_INT_PRIORITY=" + tick_priority.group(1),
        "configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=" + syscall_priority.group(1)]
    preprocessed = []
    for name, core_path in (("before", before_core), ("after", current_core)):
        source_path = output / f"gate0_{name}_without_includes.c"
        source = re.sub(r"^\s*#include[^\n]*\n", "\n", core_path.read_text(encoding="utf-8"), flags=re.MULTILINE)
        source_path.write_text(source, encoding="utf-8")
        target = output / f"gate0_{name}.i"
        command = [compiler, "-E", "-P", "-x", "c"] + ["-D" + value for value in definitions]
        results.append(run_logged(command + [str(source_path), "-o", str(target)],
            project, environment, output / f"gate0_{name}_preprocess.log"))
        preprocessed.append(target.read_bytes() if results[-1]["exit_code"] == 0 else None)
    equal = preprocessed[0] is not None and preprocessed[0] == preprocessed[1]
    result = {"exit_code": 0 if equal else 1, "equal": equal,
        "definitions": definitions,
        "scope": "Full freertos.c conditional tokens with header includes omitted; maintenance=0, integrated=1, single=1, POS=1",
        "before_sha256": sha256(output / "gate0_before.i") if preprocessed[0] is not None else None,
        "after_sha256": sha256(output / "gate0_after.i") if preprocessed[1] is not None else None,
        "preprocessing": results}
    (output / "gate0_equivalence.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"GATE0_CORE_PREPROCESS_EQUAL {equal}", flush=True)
    return result


def run_regressions(project: Path, output: Path, compiler: str,
                    environment: dict[str, str], compile_prefix: list[str]) -> list[dict]:
    """Run unchanged Core seven-group, Stage A facade and gate0 POS Core tests against the current maintained source."""
    results = []
    results.append(run_logged([os.sys.executable, "Tests/host/persistence_core_bridge_runner.py",
        "--compiler", compiler, "--build-directory", str(output / "core_bridge"), "--case", "all"],
        project, environment, output / "regression_core_bridge.log"))
    app_executable = output / "regression_app_readonly.exe"
    compilation = run_logged(compile_prefix + ["Tests/host/persistence_app_readonly_test.c"] +
        DEPENDENCIES + ["-lm", "-o", str(app_executable)], project, environment, output / "regression_app_compile.log")
    results.append(compilation)
    if compilation["exit_code"] == 0:
        results.append(run_logged([str(app_executable)], project, environment, output / "regression_app_readonly.log"))
    core = (project / "Core/Src/freertos.c").read_text(encoding="utf-8")
    generated = "/** @file core_pos_route.inc. Verbatim maintained gate0 Core POS routing. */\n"
    generated += "/** @brief Run the maintained emergency loop. */\nstatic void route_emergency(void) { CanFrame pendingFrame;\n"
    generated += extract_block(core, "while (aethor_app_pop_emergency_can_frame(&pendingFrame) != 0U)") + "\n}\n"
    generated += "/** @brief Run the maintained ordinary route. */\nstatic void route_ordinary(void) {\n"
    generated += "CanFrame pendingFrame; CanTxPriority pendingPriority; uint64_t timestampUs=1000U;\n"
    generated += extract_block(core, "if (aethor_app_next_can_frame(timestampUs,") + "\n}\n"
    (output / "core_pos_route.inc").write_text(generated, encoding="utf-8")
    pos_executable = output / "regression_gate0_pos_core.exe"
    pos_command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    pos_command += ["-D" + value for value in (
        "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1", "AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1")]
    pos_command += ["-I" + path for path in INCLUDES] + ["-I" + str(output)]
    compilation = run_logged(pos_command + ["Tests/host/pos_load_core_route_test.c", "-o", str(pos_executable)],
        project, environment, output / "regression_gate0_pos_compile.log")
    results.append(compilation)
    if compilation["exit_code"] == 0:
        results.append(run_logged([str(pos_executable)], project, environment, output / "regression_gate0_pos_core.log"))
    return results


def main() -> int:
    """Preserve before evidence, compile real dependencies, and retain separate deterministic RED/GREEN cases."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    parser.add_argument("--label", default="GREEN")
    parser.add_argument("--core-source", type=Path, help="Read-only alternate maintained Core source for reproducible before-source RED.")
    parser.add_argument("--case", default="all", choices=("all", "control", "rx", "protocol", "ui", "rollback"))
    parser.add_argument("--regressions", action="store_true")
    arguments = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = project / "output/repair_20261023/persistence_core_clock_order"
    output.mkdir(parents=True, exist_ok=True)
    before = output / "before"
    before.mkdir(parents=True, exist_ok=True)
    core_path = project / "Core/Src/freertos.c"
    before_core = before / "freertos.c"
    if not before_core.exists():
        before_core.write_bytes(core_path.read_bytes())
    selected_core = arguments.core_source.resolve() if arguments.core_source else core_path
    fragments = extract_core(selected_core, output)
    (output / f"extracted_{arguments.label}.inc").write_bytes((output / "persistence_core_clock_extracted.inc").read_bytes())
    executable = output / f"persistence_core_clock_{arguments.label}.exe"
    environment = os.environ.copy()
    environment["PATH"] = str(Path(arguments.compiler).parent) + os.pathsep + environment.get("PATH", "")
    prefix = [arguments.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    prefix += ["-D" + definition for definition in DEFINITIONS]
    prefix += ["-I" + directory for directory in INCLUDES] + ["-I" + str(output)]
    compilation = run_logged(prefix + ["Tests/host/persistence_core_clock_test.c"] + DEPENDENCIES +
        ["-lm", "-o", str(executable)], project, environment, output / f"{arguments.label}_compile.log")
    results = [compilation]
    if compilation["exit_code"] == 0:
        cases = ("control", "rx", "protocol", "ui", "rollback") if arguments.case == "all" else (arguments.case,)
        for case in cases:
            results.append(run_logged([str(executable), case], project, environment, output / f"{arguments.label}_{case}.log"))
    if arguments.regressions:
        results.extend(run_regressions(project, output, arguments.compiler, environment, prefix))
    results.append(verify_gate0_equivalence(before_core, core_path, project, output, arguments.compiler, environment))
    tracked = ["Core/Src/freertos.c", "Core/Inc/stm32h7xx_hal_conf.h", "Core/Inc/FreeRTOSConfig.h",
        "App/aethor_app.c", "App/Motor/motor_persistence_app.inc",
        "App/Motor/motor_parameter_persistence.c", "App/Motor/motor_persistence_app_port.h",
        "Tests/host/persistence_core_clock_test.c", "Tests/host/persistence_core_clock_runner.py"]
    manifest = {"label": arguments.label, "definitions": DEFINITIONS,
        "scope": "Verbatim Core timing fragments + production App/private include + pure module; Platform/HAL/USB/complete firmware not linked; no hardware",
        "before_core_sha256": sha256(before_core), "core_under_test": str(selected_core),
        "core_under_test_sha256": sha256(selected_core),
        "sources": {name: sha256(project / name) for name in tracked},
        "extracted_fragment_sha256": fragments, "results": results}
    (output / f"manifest_{arguments.label}.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    patch = "".join(difflib.unified_diff(before_core.read_text(encoding="utf-8").splitlines(keepends=True),
        core_path.read_text(encoding="utf-8").splitlines(keepends=True),
        fromfile="before/Core/Src/freertos.c", tofile="after/Core/Src/freertos.c"))
    (output / "core_clock_only.patch").write_text(patch, encoding="utf-8")
    return 1 if any(result["exit_code"] != 0 for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
