# @file 仅关闭启动脚本创建且命令行匹配的本项目网关；不是电机STOP。
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$receiptPath = Join-Path $PSScriptRoot 'artifacts/gateway-process.json'
if (-not (Test-Path -LiteralPath $receiptPath)) { throw '未找到本项目启动回执，不关闭未知进程。' }
$receipt = Get-Content -LiteralPath $receiptPath -Raw | ConvertFrom-Json
$expectedExecutable = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'Backend/Studio.Gateway/bin/Debug/net8.0/Studio.Gateway.exe'))
if ($receipt.executable -ne $expectedExecutable -or [int]$receipt.processId -le 0) { throw '启动回执身份不一致。' }
$ownedProcess = Get-Process -Id ([int]$receipt.processId) -ErrorAction SilentlyContinue
if (-not $ownedProcess) { Write-Host '未找到可访问的匹配进程，未关闭任何进程。'; return }
if ($ownedProcess.Path -ne $expectedExecutable) { throw '进程身份已变化，不关闭。' }
Stop-Process -Id ([int]$receipt.processId)
Write-Host '本机只读网关已关闭。此操作不发送硬件停止或失能命令。'
