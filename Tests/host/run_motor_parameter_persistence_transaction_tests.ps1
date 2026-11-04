# @file run_motor_parameter_persistence_transaction_tests.ps1
# @brief Builds and runs only synthetic pure-C persistence tests; no device or transport access.
param(
    [string]$Gcc = 'D:/application/WinLibs/mingw64/bin/gcc.exe'
)
$ErrorActionPreference = 'Stop'
$repositoryRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$evidenceDirectory = Join-Path $repositoryRoot 'output/repair_20261023/pmax_persistence_task2/transaction_module'
New-Item -ItemType Directory -Force $evidenceDirectory | Out-Null
$executablePath = Join-Path $evidenceDirectory 'transaction_tests.exe'
$logPath = Join-Path $evidenceDirectory 'GREEN.log'
$compilerArguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-Wpedantic', '-fstack-usage',
    '-I', (Join-Path $repositoryRoot 'App/Motor'),
    (Join-Path $repositoryRoot 'Tests/host/motor_parameter_persistence_test_main.c'),
    (Join-Path $repositoryRoot 'App/Motor/motor_parameter_persistence.c'), '-lm', '-o', $executablePath)
'SYNTHETIC ONLY; no actual motor, EEPROM or cold-boot qualification.' | Set-Content $logPath
& $Gcc @compilerArguments 2>&1 | Tee-Object -FilePath $logPath -Append
if ($LASTEXITCODE -ne 0) { throw "Persistence compilation failed: $LASTEXITCODE" }
& $executablePath 2>&1 | Tee-Object -FilePath $logPath -Append
if ($LASTEXITCODE -ne 0) { throw "Persistence tests failed: $LASTEXITCODE" }
Get-FileHash -Algorithm SHA256 $executablePath | Format-List | Out-String | Add-Content $logPath
