<#
.SYNOPSIS
Builds and runs hardware-independent ISR-to-task platform queue tests.
#>

$ErrorActionPreference = 'Stop'

function Invoke-PlatformIoTests {
    <# Builds and executes the static CAN inbox and USB CDC stream tests. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'platform_io_tests.exe'

    if (-not (Test-Path -LiteralPath $buildDirectory))
    {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    Push-Location $projectRoot
    try
    {
        & gcc `
            '-std=c11' `
            '-Wall' `
            '-Wextra' `
            '-Werror' `
            '-IApp\Motor' `
            '-IApp\Platform' `
            'Tests\host\platform_io_test_main.c' `
            'App\Motor\can_frame.c' `
            'App\Platform\can_rx_inbox.c' `
            'App\Platform\usb_cdc_stream.c' `
            '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Platform IO compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Platform IO tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-PlatformIoTests
