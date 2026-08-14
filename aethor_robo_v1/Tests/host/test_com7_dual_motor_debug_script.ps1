<#
.SYNOPSIS
Runs the COM7 dual-motor script self-tests without opening physical hardware.
#>

$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$debugScriptPath = Join-Path $repositoryRoot 'Tests\hardware\debug_com7_aethor_text_v1.ps1'

if (-not (Test-Path -LiteralPath $debugScriptPath -PathType Leaf)) {
    throw "Missing debug script: $debugScriptPath"
}

$selfTestOutput = & powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File $debugScriptPath -SelfTest 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "COM7 debug self-test failed with exit code $LASTEXITCODE`: $selfTestOutput"
}

$joinedOutput = $selfTestOutput -join "`n"
if ($joinedOutput -notmatch 'SELF_TESTS_PASSED') {
    throw "COM7 debug self-test did not publish its success marker: $joinedOutput"
}
if ($joinedOutput -notmatch 'protocol=aethor-text-v1') {
    throw "Readable protocol identity was not verified: $joinedOutput"
}
if ($joinedOutput -notmatch 'motors=1,3') {
    throw "Default motor selection was not verified: $joinedOutput"
}
if ($joinedOutput -notmatch 'run_motion_default=False') {
    throw "Safe non-motion default was not verified: $joinedOutput"
}

Write-Output 'COM7_DEBUG_SCRIPT_TESTS_PASSED'
