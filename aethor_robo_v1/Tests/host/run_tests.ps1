<#
.SYNOPSIS
Builds and runs the hardware-independent Aethor controller tests on Windows.

.DESCRIPTION
Compiles the pure C protocol, parser, transport, trajectory, and safety modules
with strict GCC warnings and places the temporary executable outside the project.
#>

$ErrorActionPreference = 'Stop'

function Invoke-AethorHostTests {
    <# Build and execute the allocation-free host test suite. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $testExecutable = Join-Path $env:TEMP 'aethor_host_tests.exe'
    $sourceFiles = @(
        'Tests\host\test_main.c',
        'User\fdcan_classic_codec.c',
        'User\dm_motor_protocol.c',
        'User\robot_config.c',
        'User\firmware_probe.c',
        'User\dual_motor_controller.c',
        'User\sync_trajectory.c',
        'User\usb_command.c',
        'User\usb_cdc_transport.c',
        'User\joint_controller.c'
    )

    Push-Location $projectRoot
    try {
        & gcc -std=c11 -Wall -Wextra -Werror -IUser @sourceFiles -lm -o $testExecutable
        if ($LASTEXITCODE -ne 0) {
            throw "GCC compilation failed with exit code $LASTEXITCODE."
        }
        & $testExecutable
        if ($LASTEXITCODE -ne 0) {
            throw "Host tests failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }
}

Invoke-AethorHostTests
