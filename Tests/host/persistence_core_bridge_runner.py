#!/usr/bin/env python3
"""File: persistence_core_bridge_runner.py. Build verbatim Core port-contract tests without hardware."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess


def extract_block(source: str, anchor: str) -> str:
    """Return one balanced maintained C function or statement without rewriting its behavior."""
    if source.count(anchor) != 1:
        raise ValueError(f"Missing or ambiguous Core anchor: {anchor}")
    position = source.index(anchor)
    opening = source.index("{", position)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if not depth:
                return source[position : index + 1]
    raise ValueError(f"Unbalanced Core block: {anchor}")


def build_extracted_source(project_root: Path, build_directory: Path) -> None:
    """Extract the actual callbacks, receipts, RX function and whole control loop, preserving every preprocessor branch."""
    source = (project_root / "Core/Src/freertos.c").read_text(encoding="utf-8")
    control = extract_block(source, "void StartArmControlTask(void const * argument)\n{")
    generated = "/** @file persistence_core_extracted.inc. Verbatim maintained Core code for port-contract tests. */\n"
    begin = "/* BEGIN_PERSISTENCE_CORE_BRIDGE */"
    end = "/* END_PERSISTENCE_CORE_BRIDGE */"
    # Missing new helpers is supported only so the unchanged Core can produce behavioral RED.
    if begin in source:
        generated += source.split(begin, 1)[1].split(end, 1)[0] + "\n"
    for anchor in (
        "static void CollectSingleStaticReceipt(void)",
        "static void ReceiveSingleStaticFrame(",
        "static void CollectCapabilityReceipt(",
    ):
        generated += extract_block(source, anchor) + "\n"
    generated += "/** @brief Execute the real Core loop until the synthetic delay returns to the fixture. */\n"
    generated += "static void RunExtractedControlTask(void) { TickType_t lastWakeTime = 0U;\n"
    generated += extract_block(control, "for(;;)") + "\n}\n"
    (build_directory / "persistence_core_extracted.inc").write_text(generated, encoding="utf-8")


def main() -> int:
    """Compile the true integrated/single/POS/persistence gate1 combination and run requested behavior cases."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    parser.add_argument("--build-directory", type=Path)
    parser.add_argument("--case", default="all", choices=("all", "receipt", "route", "link", "generation", "mutex", "rx", "emergency"))
    arguments = parser.parse_args()
    project_root = Path(__file__).resolve().parents[2]
    build_directory = arguments.build_directory or project_root / "output/repair_20261023/persistence_core_bridge/build"
    build_directory = build_directory.resolve()
    build_directory.mkdir(parents=True, exist_ok=True)
    build_extracted_source(project_root, build_directory)
    executable = build_directory / "persistence_core_bridge.exe"
    environment = os.environ.copy()
    environment["PATH"] = str(Path(arguments.compiler).parent) + os.pathsep + environment.get("PATH", "")
    command = [arguments.compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wpedantic"]
    command += ["-D" + define for define in (
        "AETHOR_MOTOR_PERSISTENCE_TRANSPORT=1", "AETHOR_ADRC_LCD_INTEGRATED=1",
        "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1", "AETHOR_DEBUG_UI_ALLOW_MIT=1",
        "AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1", "AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1",
    )]
    command += ["-I" + directory for directory in (
        "App/Config", "App/Motor", "App/Platform", "App/Arm", "App/Motion", "App/Protocol", "App/Telemetry", "App/DebugUi",
    )]
    command += ["-I" + str(build_directory), "Tests/host/persistence_core_bridge_test.c", "-o", str(executable)]
    print("COMPILING_CORE_PORT_CONTRACT integrated=1 single=1 pos=1 persistence=1", flush=True)
    compilation = subprocess.run(command, cwd=project_root, env=environment, check=False)
    if compilation.returncode:
        print("COMPILE_FAILED_THIS_IS_NOT_BEHAVIOR_RED", flush=True)
        return compilation.returncode
    return subprocess.run([str(executable), arguments.case], cwd=project_root, env=environment, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
