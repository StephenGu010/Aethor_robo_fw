<# .SYNOPSIS Runs offline-only trusted shaft-pilot integration fixtures at O0 and O2; no hardware access. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$sourceFiles = @(
 'Tests/host/adrc_shaft_pilot_test_main.c','App/Adrc/adrc_shaft_pilot.c','App/Adrc/adrc_shaft_pilot_adapter.c',
 'App/Adrc/adrc_app_bridge.c','App/Adrc/adrc_simple_trial.c','App/Adrc/adrc_feedback_adapter.c','App/Adrc/adrc_bench.c',
 'App/Adrc/adrc_protocol.c','App/Adrc/adrc_experiment.c','App/Config/arm_config.c','App/Config/build_info.c',
 'App/Telemetry/diagnostics.c','App/Arm/arm_controller.c','App/Arm/joint_reference.c','App/Motion/joint_motion.c',
 'App/Motion/joint_motion_can.c','App/Motor/can_frame.c','App/Motor/can_tx_scheduler.c','App/Motor/motor_bank.c',
 'App/Motor/motor_discovery.c','App/Motor/motor_runtime.c','App/Motor/motor_capability_collector.c','App/Motor/motor_adrc_command.c',
 'App/Motor/s3519_codec.c','App/Protocol/ascii_protocol.c','App/Protocol/text_protocol.c','App/Protocol/protocol_engine.c')
Push-Location $projectRoot
try {
 foreach($optimization in @('O0','O2')) {
  $coreExecutable = Join-Path $buildDirectory ('adrc_shaft_core_' + $optimization + '.exe')
  & $Compiler -std=c99 -Wall -Wextra -Werror -Wpedantic -fno-fast-math ('-' + $optimization) `
    '-IApp/Adrc' 'App/Adrc/adrc_shaft_pilot.c' 'Tests/host/adrc_shaft_pilot_core_test_main.c' -lm -o $coreExecutable
  if($LASTEXITCODE -ne 0) { throw "Frozen core compilation failed: $optimization" }
  & $coreExecutable
  if($LASTEXITCODE -ne 0) { throw "Frozen core fixtures failed: $optimization" }
  $executable = Join-Path $buildDirectory ('adrc_shaft_pilot_' + $optimization + '.exe')
  & $Compiler -std=c99 -Wall -Wextra -Werror -Wpedantic -fno-fast-math ('-' + $optimization) `
   '-IApp' '-IApp/Adrc' '-IApp/Arm' '-IApp/Config' '-IApp/Motion' '-IApp/Motor' '-IApp/Protocol' '-IApp/Telemetry' `
   @sourceFiles -lm -o $executable
  if($LASTEXITCODE -ne 0) { throw "Shaft bridge compilation failed: $optimization" }
  & $executable
  if($LASTEXITCODE -ne 0) { throw "Shaft bridge assertions failed: $optimization" }
 }
}
finally { Pop-Location }
