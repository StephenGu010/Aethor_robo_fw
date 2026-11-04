<# .SYNOPSIS Builds deterministic fixed-read capability and FIFO completion contracts without hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$executablePath = Join-Path $buildDirectory 'motor_capability_tests.exe'
Push-Location $projectRoot
try {
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-IApp/Motor' '-IApp/Platform' `
        'Tests/host/motor_capability_collector_test_main.c' 'App/Motor/motor_capability_collector.c' `
        'App/Motor/s3519_codec.c' 'App/Motor/can_frame.c' 'App/Platform/can_rx_inbox.c' '-lm' '-o' $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Capability host compilation failed' }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Capability host assertions failed' }
}
finally { Pop-Location }
