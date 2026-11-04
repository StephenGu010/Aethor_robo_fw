<# File: run_single_static_platform_tests.ps1. Compile actual platform and the maintained Core receive bridge against deterministic HAL stubs; no hardware or cleanup. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
function Invoke-SingleStaticPlatformTests {
    <# Retain generated Core source verbatim and execute strict gate0/gate1 production transport integration. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\single_static_platform'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $coreSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core\Src\freertos.c'))
    $receipt = [regex]::Match($coreSource, '(?s)static void CollectSingleStaticReceipt\(void\)\s*\{.*?\n\}')
    $bridge = [regex]::Match($coreSource, '(?s)/\* BEGIN_SINGLE_STATIC_RX_BRIDGE \*/(.*?)/\* END_SINGLE_STATIC_RX_BRIDGE \*/')
    if (-not $receipt.Success -or -not $bridge.Success -or -not $coreSource.Contains('ReceiveSingleStaticFrame(&receivedFrame, arrivalLowerBoundUs, receivedObservedUs);')) { throw 'Maintained Core receive bridge not found.' }
    [IO.File]::WriteAllText((Join-Path $buildDirectory 'core_receive_bridge.inc'), $receipt.Value + "`n" + $bridge.Groups[1].Value, [Text.UTF8Encoding]::new($false))
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    $sources = @('Tests/host/single_static_platform_test.c','App/DebugUi/debug_ui_mailbox.c',
        'App/Config/arm_config.c','App/Config/build_info.c','App/Telemetry/diagnostics.c',
        'App/Arm/arm_controller.c','App/Arm/joint_reference.c','App/Motion/joint_motion.c',
        'App/Motion/joint_motion_can.c','App/Motor/can_frame.c','App/Motor/can_tx_scheduler.c',
        'App/Motor/motor_bank.c','App/Motor/motor_discovery.c','App/Motor/motor_runtime.c',
        'App/Motor/s3519_codec.c','App/Protocol/ascii_protocol.c','App/Protocol/text_protocol.c',
        'App/Protocol/protocol_engine.c','App/Platform/can_rx_inbox.c','App/Platform/usb_cdc_stream.c')
    Push-Location $projectRoot
    try {
        foreach ($configuration in @(
            @{ Name = 'locked'; Single = 0; Seven = 0 },
            @{ Name = 'single'; Single = 1; Seven = 0 },
            @{ Name = 'seven'; Single = 0; Seven = 1 },
            @{ Name = 'combined'; Single = 1; Seven = 1 })) {
            $singleGate = $configuration.Single
            $executable = Join-Path $buildDirectory ('single_static_platform_' + $configuration.Name + '.exe')
            & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' ('-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=' + $singleGate) ('-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=' + $configuration.Seven) '-ITests/host/single_static_platform_fakes' '-ITests/host/build/single_static_platform' '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sources -lm -o $executable
            if ($LASTEXITCODE -ne 0) { throw ('Single-static platform compile failed: ' + $singleGate) }
            & $executable
            if ($LASTEXITCODE -ne 0) { throw ('Single-static platform execution failed: ' + $singleGate) }
            Write-Output ('SINGLE_STATIC_PLATFORM_GATE_PASS ' + $configuration.Name)
        }
    } finally { Pop-Location }
}
Invoke-SingleStaticPlatformTests
