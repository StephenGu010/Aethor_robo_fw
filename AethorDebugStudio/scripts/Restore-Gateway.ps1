# @file 从NuGet官方HTTPS源下载固定依赖，再用本地源恢复；不修改全局NuGet/代理配置。
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$feedPath = Join-Path $projectRoot '.packages-feed'
New-Item -ItemType Directory -Path $feedPath -Force | Out-Null
$packageIds = @('system.io.ports', 'runtime.native.system.io.ports',
    'runtime.linux-arm.runtime.native.system.io.ports', 'runtime.linux-arm64.runtime.native.system.io.ports',
    'runtime.linux-x64.runtime.native.system.io.ports', 'runtime.osx-arm64.runtime.native.system.io.ports',
    'runtime.osx-x64.runtime.native.system.io.ports')
$packageManifest = @()
foreach ($packageId in $packageIds) {
    $packageName = "$packageId.8.0.0.nupkg"
    $packagePath = Join-Path $feedPath $packageName
    $packageUrl = "https://api.nuget.org/v3-flatcontainer/$packageId/8.0.0/$packageName"
    if (-not (Test-Path -LiteralPath $packagePath)) {
        Invoke-WebRequest -Uri $packageUrl -OutFile $packagePath -TimeoutSec 45
    }
    $packageManifest += [pscustomobject]@{ package = $packageName; source = $packageUrl; sha256 = (Get-FileHash -LiteralPath $packagePath -Algorithm SHA256).Hash.ToLowerInvariant() }
}
$packageManifest | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $feedPath 'manifest.json') -Encoding utf8
& dotnet restore (Join-Path $projectRoot 'Backend/Studio.Gateway.Tests/Studio.Gateway.Tests.csproj') --source $feedPath -p:NuGetAudit=false
if ($LASTEXITCODE -ne 0) { throw '网关依赖恢复失败。官方HTTPS下载和本地源均未关闭证书验证。' }
