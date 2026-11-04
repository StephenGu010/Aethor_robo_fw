# @file 完整软件验收：不打开真实COM，不烧录，不发送控制指令。
[CmdletBinding()]
param([switch]$SkipBrowser)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location (Join-Path $projectRoot 'Frontend')
try {
    & npm.cmd test
    if ($LASTEXITCODE -ne 0) { throw '前端领域测试失败' }
    & npm.cmd run build
    if ($LASTEXITCODE -ne 0) { throw '前端类型/构建失败' }
    & npm.cmd audit --audit-level=high
    if ($LASTEXITCODE -ne 0) { throw '前端依赖审计未通过' }
    if (-not $SkipBrowser) {
        & npm.cmd run test:e2e
        if ($LASTEXITCODE -ne 0) { throw '浏览器验收失败' }
    }
} finally { Pop-Location }
& (Join-Path $PSScriptRoot 'Restore-Gateway.ps1')
& dotnet run --no-restore --project (Join-Path $projectRoot 'Backend/Studio.Gateway.Tests')
if ($LASTEXITCODE -ne 0) { throw '网关协议测试失败' }
Write-Host 'STUDIO_SOFTWARE_VERIFICATION_PASS hardware_connected=false motion_sent=false'
