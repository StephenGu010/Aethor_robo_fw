<#
.SYNOPSIS
Runs the non-hardware regression test for the guarded COM7 output-turn script.
.DESCRIPTION
Invokes only the script self-test path, which must not open COM7 or move motors.
#>

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$hardwareScriptPath = Join-Path $PSScriptRoot `
    '..\hardware\run_com7_output_one_turn_1_3.ps1'

if (-not (Test-Path -LiteralPath $hardwareScriptPath -PathType Leaf)) {
    throw "ONE_TURN_SCRIPT_MISSING: $hardwareScriptPath"
}

$selfTestOutput = & powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File $hardwareScriptPath -SelfTest 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "ONE_TURN_SELF_TEST_PROCESS_FAILED: $selfTestOutput"
}
if (($selfTestOutput -join "`n") -notmatch 'ONE_TURN_SELF_TESTS_PASSED') {
    throw "ONE_TURN_SELF_TEST_MARKER_MISSING: $selfTestOutput"
}

Write-Output 'COM7_OUTPUT_ONE_TURN_SCRIPT_TESTS_PASSED'
