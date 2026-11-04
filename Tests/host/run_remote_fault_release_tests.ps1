<# @file Offline inherited production flags and explicit synthetic release assertions; no device access. #>
param([string]$Compiler='D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'run_torque_envelope_tests.ps1') -Compiler $Compiler
$sourceFiles[0]='Tests/host/remote_fault_release_test.c'
Push-Location $projectRoot
try {
    & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_ADRC_LCD_INTEGRATED=1' `
        '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1' `
        '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1' '-DAETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1' `
        '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F' '-DAETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F' `
        '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Adrc' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sourceFiles -lm `
        -o (Join-Path $evidenceDirectory 'remote_fault_release_test.exe')
    if($LASTEXITCODE -ne 0){throw 'Fault release regression compilation failed'}
    & (Join-Path $evidenceDirectory 'remote_fault_release_test.exe')
    if($LASTEXITCODE -ne 0){throw 'Fault release regression assertions failed'}
}finally{Pop-Location}
