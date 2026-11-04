# @file 一键构建并启动本机工作台，默认离线演示，不打开真实COM。
[CmdletBinding()]
param([switch]$NoBrowser, [switch]$SkipBuild)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$workspaceUrl = 'http://127.0.0.1:5179'
# 只复用同一网关服务，避免在已有会话上自动重启。
$existing = $null
try { $existing = Invoke-RestMethod -Uri "$workspaceUrl/api/health" -TimeoutSec 2 } catch { }
if ($existing) {
    if ($existing.service -ne 'aethor-debug-gateway') { throw '5179端口已由其他服务使用。' }
    Write-Host "工作台已运行：$workspaceUrl"
    if (-not $NoBrowser) { Start-Process $workspaceUrl }
    return
}
if (-not $SkipBuild) {
    Push-Location (Join-Path $projectRoot 'Frontend')
    try {
        if (-not (Test-Path -LiteralPath 'node_modules')) {
            & npm.cmd ci --ignore-scripts --no-fund
            if ($LASTEXITCODE -ne 0) { throw '前端依赖安装失败。' }
        }
        & npm.cmd run build
        if ($LASTEXITCODE -ne 0) { throw '前端构建失败。' }
    } finally { Pop-Location }
    & (Join-Path $projectRoot 'scripts/Restore-Gateway.ps1')
    & dotnet build --no-restore (Join-Path $projectRoot 'Backend/Studio.Gateway') -v minimal
    if ($LASTEXITCODE -ne 0) { throw '网关构建失败。' }
}
$gatewayDirectory = Join-Path $projectRoot 'Backend/Studio.Gateway'
$gatewayAssembly = Join-Path $gatewayDirectory 'bin/Debug/net8.0/Studio.Gateway.dll'
$gatewayExecutable = Join-Path $gatewayDirectory 'bin/Debug/net8.0/Studio.Gateway.exe'
if (-not (Test-Path -LiteralPath $gatewayAssembly)) { throw '缺少网关工件，请不带SkipBuild重新运行。' }
if (-not (Test-Path -LiteralPath (Join-Path $projectRoot 'Frontend/dist/index.html'))) { throw '缺少前端构建。' }
$artifactDirectory = Join-Path $projectRoot 'artifacts'
New-Item -ItemType Directory -Path $artifactDirectory -Force | Out-Null
$logStamp = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$previousDotnetRoot = $env:DOTNET_ROOT
$previousDotnetRootX64 = $env:DOTNET_ROOT_X64
try {
    # 仅给本次子进程指定与dotnet命令一致的运行时，避免系统另一个.NET根目录覆盖它。
    $env:DOTNET_ROOT = Split-Path -Parent (Get-Command dotnet.exe).Source
    $env:DOTNET_ROOT_X64 = $env:DOTNET_ROOT
    $gatewayProcess = Start-Process -FilePath $gatewayExecutable -WorkingDirectory $gatewayDirectory -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $artifactDirectory "$logStamp-gateway.log") -RedirectStandardError (Join-Path $artifactDirectory "$logStamp-gateway-error.log")
} finally {
    $env:DOTNET_ROOT = $previousDotnetRoot
    $env:DOTNET_ROOT_X64 = $previousDotnetRootX64
}
[pscustomobject]@{ processId = $gatewayProcess.Id; executable = $gatewayExecutable; startedAt = [DateTime]::UtcNow.ToString('O') } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $artifactDirectory 'gateway-process.json') -Encoding utf8
for ($attempt = 0; $attempt -lt 20; $attempt++) {
    Start-Sleep -Milliseconds 250
    try {
        $health = Invoke-RestMethod -Uri "$workspaceUrl/api/health" -TimeoutSec 1
        if ($health.service -eq 'aethor-debug-gateway') {
            Write-Host "工作台已就绪：$workspaceUrl"
            Write-Host '默认离线演示；仅在页面选择端口并点击连接后才开始只读查询。'
            if (-not $NoBrowser) { Start-Process $workspaceUrl }
            return
        }
    } catch { }
    if ($gatewayProcess.HasExited) { throw "网关已退出，请查看artifacts/$logStamp-gateway-error.log" }
}
throw '等待网关启动超时，请查看artifacts内日志。'
