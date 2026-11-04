"""Exercise the offline single-static builder at its external process boundary.

Each PowerShell fixture lives under ignored Tests/host/build and is retained.
The fake Start-Process never starts Keil, a debugger, a probe or board firmware.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
BUILDER = ROOT / "tools/build_single_static_test.ps1"
TARGET = "LCD-SingleStatic-Test"

# The fake supplies only the external process result and files; all acceptance,
# containment, copy and hash decisions execute the actual PowerShell builder.
HARNESS = r'''<# @file Fake the Keil process boundary for offline builder regressions. #>
param([string]$BuildScript, [string]$KeilPath, [string]$EvidenceDirectory,
      [string]$Scenario, [string]$FixtureRoot)
$ErrorActionPreference = 'Stop'
function Start-Process {
    <# Record the exact invocation and emit controlled build receipts without a process. #>
    param([string]$FilePath, [string[]]$ArgumentList, [string]$WindowStyle,
          [switch]$PassThru, [switch]$Wait)
    $call = [ordered]@{ executable = $FilePath; arguments = @($ArgumentList);
        window = $WindowStyle; pass_thru = [bool]$PassThru; wait = [bool]$Wait }
    [IO.File]::WriteAllText((Join-Path $FixtureRoot 'process.json'), ($call | ConvertTo-Json -Depth 5))
    if ($ArgumentList.Count -ne 7 -or $ArgumentList[0] -ne '-r' -or
        $ArgumentList[2] -ne '-t' -or $ArgumentList[3] -ne 'LCD-SingleStatic-Test' -or
        $ArgumentList[4] -ne '-j0' -or $ArgumentList[5] -ne '-o') {
        throw 'Unexpected command-line build invocation.'
    }
    $logPath = $ArgumentList[6].Trim('"')
    $outputDirectory = Join-Path $FixtureRoot 'MDK-ARM\LCD-SingleStatic-Test'
    [void](New-Item -ItemType Directory -Path $outputDirectory -Force)
    if ($Scenario -ne 'missing_log') {
        $summary = '0 Error(s), 0 Warning(s)'
        if ($Scenario -eq 'warning') { $summary = '0 Error(s), 1 Warning(s)' }
        if ($Scenario -eq 'error') { $summary = '10 Error(s), 0 Warning(s)' }
        if ($Scenario -eq 'missing_summary') { $summary = 'Build stopped.' }
        $log = "Build target 'LCD-SingleStatic-Test'`r`nProgram Size: Code=1 RO-data=2 RW-data=3 ZI-data=4`r`n$summary`r`n"
        [IO.File]::WriteAllText($logPath, $log)
        if ($Scenario -eq 'stale_log') {
            (Get-Item -LiteralPath $logPath).LastWriteTimeUtc = [DateTime]::UtcNow.AddSeconds(-1)
        }
    }
    foreach ($extension in @('axf', 'hex', 'map', 'htm')) {
        if ($Scenario -eq ('missing_' + $extension)) { continue }
        $artifact = Join-Path $outputDirectory ('LCD-SingleStatic-Test.' + $extension)
        [IO.File]::WriteAllText($artifact, ('fake software-only ' + $extension))
        if ($Scenario -eq ('empty_' + $extension)) { [IO.File]::WriteAllText($artifact, '') }
        if ($Scenario -eq ('stale_' + $extension)) {
            (Get-Item -LiteralPath $artifact).LastWriteTimeUtc = [DateTime]::UtcNow.AddSeconds(-1)
        }
        if ($Scenario -eq ('future_' + $extension)) {
            (Get-Item -LiteralPath $artifact).LastWriteTimeUtc = [DateTime]::UtcNow.AddMinutes(1)
        }
    }
    if ($Scenario -eq 'project_changed') {
        [IO.File]::AppendAllText((Join-Path $FixtureRoot 'MDK-ARM\LCD-SingleStatic-Test.uvprojx'), '<!-- changed -->')
    }
    $exitCode = 0
    if ($Scenario -eq 'nonzero') { $exitCode = 7 }
    return [pscustomobject]@{ ExitCode = $exitCode }
}
if ($Scenario -eq 'default') {
    & $BuildScript -KeilPath $KeilPath
} else {
    & $BuildScript -KeilPath $KeilPath -EvidenceDirectory $EvidenceDirectory
}
'''


class SingleStaticBuildTests(unittest.TestCase):
    """Verify receipts and failure gates using isolated, retained Windows fixtures."""

    def run_builder(self, scenario: str, evidence_kind: str = "new") -> tuple:
        """Run the real script with a fake process in a separate workspace copy."""
        self.assertTrue(BUILDER.is_file(), "Missing offline single-static builder")
        build_root = ROOT / "Tests/host/build/single_static_build"
        build_root.mkdir(parents=True, exist_ok=True)
        retained = Path(tempfile.mkdtemp(prefix=scenario + "_", dir=build_root))
        workspace = retained / "workspace with spaces"
        (workspace / "tools").mkdir(parents=True)
        (workspace / "MDK-ARM").mkdir()
        shutil.copy2(BUILDER, workspace / "tools" / BUILDER.name)
        (workspace / "MDK-ARM" / f"{TARGET}.uvprojx").write_text("<Project />\n")
        fake_keil = workspace / "fake UV4.exe"
        fake_keil.write_text("Process boundary is replaced by the PowerShell harness.\n")
        harness = retained / "harness.ps1"
        harness.write_text(HARNESS, encoding="utf-8")
        evidence = workspace / "evidence with spaces"
        if evidence_kind == "occupied":
            evidence.mkdir()
            (evidence / "keep.txt").write_text("must not be overwritten\n")
        elif evidence_kind == "outside":
            evidence = workspace.parent / (workspace.name + "-outside")
        elif evidence_kind == "traversal":
            evidence = workspace / ".." / "outside"
        # Desktop PowerShell must build its own module path instead of inheriting
        # a portable PowerShell 7 Utility module through the Python process.
        child_environment = {key: value for key, value in os.environ.items() if key.upper() != "PSMODULEPATH"}
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
             "-File", str(harness), "-BuildScript", str(workspace / "tools" / BUILDER.name),
             "-KeilPath", str(fake_keil), "-EvidenceDirectory", str(evidence),
             "-Scenario", scenario, "-FixtureRoot", str(workspace)],
            text=True, capture_output=True, check=False, timeout=45, env=child_environment)
        (retained / "stdout.log").write_text(result.stdout, encoding="utf-8")
        (retained / "stderr.log").write_text(result.stderr, encoding="utf-8")
        if scenario == "default":
            manifests = list((workspace / "output/static_trials_20261019").glob("build_*/build_manifest.json"))
            self.assertEqual(len(manifests), 1, result.stdout + result.stderr)
            evidence = manifests[0].parent
        print(f"RETAINED_BUILDER_FIXTURE {retained}")
        return result, workspace, evidence

    def test_builder_script_is_available(self) -> None:
        """Give the test-first RED a direct missing-feature assertion."""
        self.assertTrue(BUILDER.is_file(), "Missing offline single-static builder")

    def test_clean_build_retains_hash_verified_products_and_build_only_invocation(self) -> None:
        """Accept clean fresh software receipts and preserve both map-check and evidence logs."""
        result, workspace, evidence = self.run_builder("success")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        manifest = json.loads((evidence / "build_manifest.json").read_text(encoding="utf-8-sig"))
        self.assertTrue(manifest["passed"])
        self.assertTrue(manifest["software_only"])
        self.assertFalse(manifest["hardware_qualified"])
        self.assertEqual(manifest["target"], TARGET)
        self.assertEqual(manifest["exit_code"], 0)
        self.assertEqual(manifest["project_sha256"], hashlib.sha256(Path(manifest["project"]).read_bytes()).hexdigest())
        self.assertEqual(set(manifest["artifacts"]), {"axf", "hex", "map", "htm", "log"})
        for artifact in manifest["artifacts"].values():
            source = Path(artifact["source"])
            copied = Path(artifact["copy"])
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), artifact["sha256"])
            self.assertEqual(hashlib.sha256(copied.read_bytes()).hexdigest(), artifact["sha256"])
        map_log = workspace / "Tests/host/build/keil" / f"{TARGET}.log"
        self.assertEqual(map_log.read_bytes(), (evidence / f"{TARGET}.log").read_bytes())
        call = json.loads((workspace / "process.json").read_text())
        self.assertEqual(call["arguments"], ["-r", '"' + str(workspace / "MDK-ARM" / f"{TARGET}.uvprojx") + '"',
                         "-t", TARGET, "-j0", "-o", '"' + str(evidence / f"{TARGET}.log") + '"'])
        self.assertEqual(call["window"], "Hidden")
        self.assertTrue(call["wait"] and call["pass_thru"])

    def test_failed_or_incomplete_receipts_are_rejected(self) -> None:
        """Reject nonzero exit, warnings, errors and every missing/stale required receipt."""
        for scenario in ("nonzero", "warning", "error", "missing_summary", "missing_log", "stale_log",
                         "missing_axf", "stale_axf", "missing_hex", "stale_hex",
                         "missing_map", "stale_map", "missing_htm", "stale_htm",
                         "empty_axf", "future_hex", "project_changed"):
            with self.subTest(scenario=scenario):
                result, workspace, evidence = self.run_builder(scenario)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                manifest = json.loads((evidence / "build_manifest.json").read_text(encoding="utf-8-sig"))
                self.assertFalse(manifest["passed"])
                self.assertTrue(manifest["software_only"])
                self.assertFalse(manifest["hardware_qualified"])
                if scenario == "nonzero":
                    self.assertIn("exit code", manifest["failure"])
                    self.assertEqual(manifest["exit_code"], 7)
                elif scenario in ("warning", "error", "missing_summary"):
                    self.assertIn("zero-error, zero-warning", manifest["failure"])
                elif scenario == "project_changed":
                    self.assertIn("changed the maintained project", manifest["failure"])
                elif scenario.startswith("missing_"):
                    self.assertIn("Missing build receipt", manifest["failure"])
                else:
                    self.assertIn("not refreshed by this build", manifest["failure"])
                self.assertFalse((workspace / "Tests/host/build/keil" / f"{TARGET}.log").exists())

    def test_occupied_and_outside_evidence_paths_do_not_launch_a_build(self) -> None:
        """Reject evidence reuse, sibling-prefix paths and traversal before any process call."""
        for kind in ("occupied", "outside", "traversal"):
            with self.subTest(kind=kind):
                result, workspace, evidence = self.run_builder("success", kind)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertFalse((workspace / "process.json").exists())
                if kind == "occupied":
                    self.assertIn("already contains results", result.stderr)
                    self.assertEqual((evidence / "keep.txt").read_text(), "must not be overwritten\n")
                    self.assertEqual(list(evidence.iterdir()), [evidence / "keep.txt"])
                else:
                    self.assertIn("inside this workspace", result.stderr)
                    self.assertFalse(evidence.exists())

    def test_default_evidence_location_is_unique_and_software_only(self) -> None:
        """Keep the default receipts inside the workspace's ignored dated output tree."""
        result, workspace, evidence = self.run_builder("default")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(evidence.parent, workspace / "output/static_trials_20261019")
        self.assertTrue(evidence.name.startswith("build_"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
