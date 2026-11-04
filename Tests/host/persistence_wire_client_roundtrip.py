"""Verify the real C App wire exporter against the Python library using synthetic CAN, without hardware access."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from persistence_core_clock_runner import DEFINITIONS, DEPENDENCIES, INCLUDES

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from pmax_persistence_client import ClientInputs, PersistenceClient


class NativeBackend:
    """Own one offline C test process and its synthetic monotonic clock; no serial or probe is imported."""

    def __init__(self, executable: Path, mode: int, pmax: float, early: int, environment: dict[str, str]) -> None:
        """Start a new fixture process with binary pipes to preserve actual CRLF bytes."""
        self.process = subprocess.Popen([str(executable), str(mode), str(pmax), str(early)], cwd=ROOT,
                                        env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert self.process.stdout is not None
        header = self.process.stdout.readline().decode("ascii").strip().split()
        if len(header) != 4 or header[0] != "BOOT" or header[2] != "CLOCK":
            raise ValueError(f"invalid native fixture header: {header!r}")
        self.boot, self.now_us = int(header[1]), int(header[3])

    def clock(self) -> float:
        """Expose only fixture time, never wall-clock timing as a motor measurement."""
        return self.now_us / 1000000.0

    def poll(self) -> None:
        """Keep polling deterministic; the next real C exchange advances its fixture by four milliseconds."""

    def exchange(self, request_id: int, command: str) -> str:
        """Exchange exactly one text line and receive its exact C serializer output and separate fixture clock."""
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(f"{request_id} {command}\n".encode("ascii"))
        self.process.stdin.flush()
        reply = self.process.stdout.readline().decode("ascii")
        timing = self.process.stdout.readline().decode("ascii").strip().split()
        if len(timing) != 2 or timing[0] != "CLOCK":
            raise ValueError(f"invalid native fixture clock: {timing!r}; reply={reply!r}")
        self.now_us = int(timing[1])
        return reply

    def close(self) -> None:
        """Close only this test process's stdin and require a clean bounded exit."""
        assert self.process.stdin is not None
        self.process.stdin.close()
        if self.process.wait(timeout=5) != 0:
            assert self.process.stderr is not None
            raise RuntimeError(self.process.stderr.read().decode("utf-8", errors="replace"))


def main() -> int:
    """Compile one immutable labelled run, then retain complete success and partial failure transcripts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--compiler", default=r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    arguments = parser.parse_args()
    output = ROOT / "output/repair_20261023/persistence_wire_roundtrip" / arguments.label
    output.mkdir(parents=True, exist_ok=False)
    fixture = (ROOT / "Tests/host/persistence_app_b1_test.c").read_text(encoding="utf-8")
    if fixture.count("int main(void)") != 1:
        raise ValueError("B1 fixture main boundary changed")
    (output / "persistence_b1_fixture.inc").write_text(
        fixture.replace("int main(void)", "int existing_b1_suite_main(void)"), encoding="utf-8")
    executable = output / "persistence_wire_server.exe"
    command = [arguments.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
    command += [f"-D{value}" for value in DEFINITIONS] + [f"-I{output}"] + [f"-I{value}" for value in INCLUDES]
    command += ["Tests/host/persistence_wire_server.c", *DEPENDENCIES, "-lm", "-o", str(executable)]
    environment = os.environ.copy()
    environment["PATH"] = str(Path(arguments.compiler).parent) + os.pathsep + environment["PATH"]
    compiled = subprocess.run(command, cwd=ROOT, env=environment, capture_output=True, text=True)
    (output / "compile.log").write_text(subprocess.list2cmdline(command) + "\n" + compiled.stdout + compiled.stderr +
                                        f"\nEXIT_CODE {compiled.returncode}\n", encoding="utf-8")
    if compiled.returncode:
        return compiled.returncode
    summaries: list[dict[str, object]] = []
    original_baseline = None
    for index, (mode, pmax, early) in enumerate(((1, 12.5, 0), (1, 12.5, 1), (2, 12.5, 0), (1, 512.0, 0), (2, 512.0, 0), (2, 512.0, 1))):
        case = output / f"case_{index}"
        case.mkdir()
        once = case / "once"
        once.mkdir()
        backend = NativeBackend(executable, mode, pmax, early, environment)
        client = PersistenceClient(ClientInputs("1" * 64, "2" * 64, "386B336F3134", backend.boot, True), once,
                                   backend.exchange, clock=backend.clock, poll_wait=backend.poll, first_request_id=701)
        try:
            result = client.run_readonly_verify(original_baseline) if index == 5 else client.run_prepare_save()
            if original_baseline is None:
                original_baseline = client.evidence["baseline"]
            summaries.append(dict(case=index, mode=mode, pmax=pmax, early=early, result=result,
                                  commands=len(client.transcript), synthetic_can=True, hardware_qualified=False))
        finally:
            (case / "evidence.json").write_text(json.dumps(dict(transcript=client.transcript, evidence=client.evidence),
                                                           indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
            backend.close()
    paths = ("App/aethor_app.c", "App/Motor/motor_persistence_app.inc", "App/Motor/motor_persistence_wire.inc",
             "tools/pmax_persistence_client.py", "Tests/host/persistence_wire_server.c", __file__)
    sources = {str(path): hashlib.sha256((ROOT / path).read_bytes()).hexdigest() for path in paths}
    report = dict(cases=summaries, source_hashes=sources, scope="real C App wire and Python client; synthetic CAN, no Core/Platform/HAL")
    (output / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
