"""Build the production facade with the candidate target flags; never access hardware."""
from pathlib import Path
import argparse
import os
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    """Preserve separate red/green build and assertion logs for the real application fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="green")
    parser.add_argument("--source", default="Tests/host/idle_feedback_test.c")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = root / "output/idle-feedback" / args.label
    output.mkdir(parents=True, exist_ok=True)
    runner = (root / "Tests/host/run_adrc_lcd_integration_tests.ps1").read_text()
    dependencies = re.findall(r"'([^']+\.c)'", runner.split("$sourceFiles = @(")[1].split("\n)")[0])[1:]
    dependencies += ["App/Motor/motor_parameter_persistence.c"]
    definitions = ET.parse(root / "MDK-ARM/LCD-POS-Load-Test.uvprojx").find(".//Cads/VariousControls/Define").text.split(",")
    definitions = [value.strip() for value in definitions if value.strip().startswith(("AETHOR_", "DEBUG_UI_"))]
    definitions = [value for value in definitions if not value.startswith("AETHOR_IDLE_FEEDBACK_ENABLE=")]
    definitions += ["AETHOR_IDLE_FEEDBACK_ENABLE=1"]
    compiler = Path(r"D:/application/WinLibs/mingw64/bin/gcc.exe")
    includes = ["App", "App/Arm", "App/Config", "App/DebugUi", "App/Motor", "App/Adrc", "App/Motion", "App/Protocol", "App/Telemetry", "App/Platform", "Tests/host"]
    executable = output / "idle_feedback.exe"
    command = [str(compiler), "-std=c11", "-Wall", "-Wextra", "-Werror"]
    command += ["-D" + value for value in definitions] + ["-I" + value for value in includes]
    command += [args.source] + dependencies + ["-lm", "-o", str(executable)]
    environment = dict(os.environ, PATH=str(compiler.parent) + os.pathsep + os.environ["PATH"])
    built = subprocess.run(command, cwd=root, env=environment, capture_output=True)
    (output / "build.log").write_bytes(built.stdout + built.stderr)
    if built.returncode:
        print((built.stdout + built.stderr).decode(errors="replace"))
        return built.returncode
    run = subprocess.run([str(executable)], cwd=root, env=environment, capture_output=True)
    (output / "test.log").write_bytes(run.stdout + run.stderr)
    print((run.stdout + run.stderr).decode(errors="replace"))
    return run.returncode


if __name__ == "__main__":
    raise SystemExit(main())
