<#
.SYNOPSIS
Rebuilds one existing STM32 debug-UI target without downloading firmware.
.DESCRIPTION
Uses the installed Keil compiler and writes one target-specific build log.
This command does not start a debugger, connect to a probe, or flash a board.
#>
param(
    [ValidateSet('CtrBoard-H7_FDCAN', 'LCD-ReadOnly', 'LCD-POS', 'LCD-MIT')]
    [string]$Target = 'LCD-ReadOnly',
    [string]$KeilPath = 'D:\application\keil\UV4\UV4.exe'
)
$ErrorActionPreference = 'Stop'

function Invoke-DebugUiKeilBuild {
    <# Launches the Keil command-line rebuild hidden and verifies the actual log. #>
    param([string]$TargetName, [string]$ExecutablePath)
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    if (-not (Test-Path -LiteralPath $ExecutablePath -PathType Leaf)) {
        throw 'Keil UV4.exe was not found. Pass its installed path with -KeilPath.'
    }
    $projectPath = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
    $logDirectory = Join-Path $projectRoot 'Tests\host\build\keil'
    [void](New-Item -ItemType Directory -Path $logDirectory -Force)
    $logPath = Join-Path $logDirectory ($TargetName + '.log')
    $arguments = @('-r', ('"' + $projectPath + '"'), '-t', $TargetName,
                   '-j0', '-o', ('"' + $logPath + '"'))
    $startedAt = [DateTime]::UtcNow
    $process = Start-Process -FilePath $ExecutablePath -ArgumentList $arguments `
        -WindowStyle Hidden -PassThru -Wait
    if (-not (Test-Path -LiteralPath $logPath)) { throw 'Keil did not produce a build log.' }
    $logItem = Get-Item -LiteralPath $logPath
    if ($logItem.LastWriteTimeUtc -lt $startedAt.AddSeconds(-2)) {
        throw 'Keil log was not refreshed by this build.'
    }
    $log = Get-Content -LiteralPath $logPath -Raw
    Get-Content -LiteralPath $logPath -Tail 12
    if (($process.ExitCode -ne 0) -or ($log -notmatch '0 Error\(s\), 0 Warning\(s\)')) {
        throw "Keil target $TargetName did not complete cleanly; see $logPath"
    }
    Write-Output ("KEIL_DEBUG_UI_BUILD_PASSED target=$TargetName log=$logPath")
}

Invoke-DebugUiKeilBuild -TargetName $Target -ExecutablePath $KeilPath
