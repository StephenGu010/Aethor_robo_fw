<# .SYNOPSIS
Compile and run the allocation-free load ledger in strict C99 without hardware.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-LoadTrialTests {
    <# Build a standalone fixture; the production include is linked by the fixture. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\load_trials'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $executablePath = Join-Path $buildDirectory 'load_trials_test.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
        ('-I' + (Join-Path $projectRoot 'App\DebugUi')) `
        (Join-Path $PSScriptRoot 'load_trials_test.c') '-lm' '-o' $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Load trial ledger compilation failed.' }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Load trial ledger behavior failed.' }
    $modelExecutablePath = Join-Path $buildDirectory 'load_trial_model_hooks_test.exe'
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
        '-DLOAD_TRIAL_MODEL_HOOKS_TEST=1' '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1' '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' `
        ('-I' + (Join-Path $projectRoot 'App\DebugUi')) `
        (Join-Path $PSScriptRoot 'load_trials_test.c') `
        (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') '-lm' '-o' $modelExecutablePath
    if ($LASTEXITCODE -ne 0) { throw 'Load trial model hooks compilation failed.' }
    & $modelExecutablePath
    if ($LASTEXITCODE -ne 0) { throw 'Load trial model hooks behavior failed.' }
    $applicationExecutablePath = Join-Path $buildDirectory 'load_trial_app_order_test.exe'
    $applicationSources = @('Tests/host/load_trials_test.c', 'App/DebugUi/debug_ui_model.c',
        'App/DebugUi/debug_ui_mailbox.c', 'App/Config/arm_config.c', 'App/Config/build_info.c',
        'App/Telemetry/diagnostics.c', 'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c',
        'App/Motion/joint_motion.c', 'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c',
        'App/Motor/can_tx_scheduler.c', 'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c',
        'App/Motor/motor_runtime.c', 'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c',
        'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c')
    Push-Location $projectRoot
    try {
        & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-DLOAD_TRIAL_APP_ORDER_TEST=1' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Motion' `
            '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @applicationSources '-lm' '-o' $applicationExecutablePath
        if ($LASTEXITCODE -ne 0) { throw 'Load trial application-order compilation failed.' }
        & $applicationExecutablePath
        if ($LASTEXITCODE -ne 0) { throw 'Load trial application-order behavior failed.' }
    }
    finally { Pop-Location }
}
Invoke-LoadTrialTests
