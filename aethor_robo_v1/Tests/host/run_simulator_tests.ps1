<#
.SYNOPSIS
Runs deterministic protocol simulator, compatibility, and logical soak tests.

.DESCRIPTION
Uses the workspace Python interpreter without third-party packages. The soak
tests advance logical time and do not represent wall-clock or hardware soak.
#>

$ErrorActionPreference = 'Stop'

function Invoke-AethorSimulatorTests {
    <# Runs all simulator tests and a query-only reference-client probe. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $transcriptPath = Join-Path $PSScriptRoot 'build\reference-client-transcript.txt'

    Push-Location $projectRoot
    try {
        $env:PYTHONUTF8 = '1'
        & python -m unittest `
            Tests.host.test_host_simulator `
            Tests.host.test_soak_simulator `
            -v
        if ($LASTEXITCODE -ne 0) {
            throw "Simulator tests failed with exit code $LASTEXITCODE."
        }
        & python tools\aethor_reference_client.py --output $transcriptPath
        if ($LASTEXITCODE -ne 0) {
            throw "Reference client failed with exit code $LASTEXITCODE."
        }
        Write-Output 'SIMULATOR_TESTS_PASSED'
    }
    finally {
        Pop-Location
    }
}

Invoke-AethorSimulatorTests
