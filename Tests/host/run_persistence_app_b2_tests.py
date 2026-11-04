"""Compile the real App maintenance wire contract offline and retain each immutable run."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess

from persistence_core_clock_runner import DEFINITIONS, DEPENDENCIES, INCLUDES


def main() -> int:
    """Build and execute one labelled synthetic wire test without loading a hardware library."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", default="output/repair_20261023/persistence_app_b2")
    arguments = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = project / arguments.output / arguments.label
    output.mkdir(parents=True, exist_ok=False)
    executable = output / "persistence_app_b2.exe"
    fixture = (project / "Tests/host/persistence_app_b1_test.c").read_text(encoding="utf-8")
    if fixture.count("int main(void)") != 1:
        raise ValueError("B1 test fixture main boundary changed")
    # Only the earlier suite entry name changes; all real App and synthetic transport statements remain verbatim.
    (output / "persistence_b1_fixture.inc").write_text(
        fixture.replace("int main(void)", "int existing_b1_suite_main(void)"), encoding="utf-8")
    command = [arguments.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    command += [f"-D{value}" for value in DEFINITIONS] + [f"-I{output}"] + [f"-I{value}" for value in INCLUDES]
    command += ["Tests/host/persistence_app_b2_test.c", *DEPENDENCIES, "-lm", "-o", str(executable)]
    environment = os.environ.copy()
    environment["PATH"] = str(Path(arguments.compiler).parent) + os.pathsep + environment["PATH"]
    result = subprocess.run(command, cwd=project, env=environment, capture_output=True, text=True)
    log = "COMMAND " + subprocess.list2cmdline(command) + "\n" + result.stdout + result.stderr
    code = result.returncode
    if not code:
        result = subprocess.run([str(executable)], cwd=project, env=environment, capture_output=True, text=True)
        log += result.stdout + result.stderr
        code = result.returncode
    log += f"\nEXIT_CODE {code}\nSCOPE real App text parser and pure module; synthetic CAN only\n"
    (output / "result.log").write_text(log, encoding="utf-8")
    print(log)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
