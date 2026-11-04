<# @file Compile the frozen firmware in a separate output directory; no firmware edits or hardware access. #>
param([string]$FirmwareRoot='')
$ErrorActionPreference='Stop'
$hostRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if(-not $FirmwareRoot){$FirmwareRoot=Split-Path -Parent $hostRoot}
$FirmwareRoot=(Resolve-Path -LiteralPath $FirmwareRoot).Path
$outputDirectory=Join-Path $hostRoot 'output\pos-quota-regression'
[void](New-Item -ItemType Directory -Force -Path $outputDirectory)
$adapter=Get-Content -LiteralPath (Join-Path $FirmwareRoot 'Tests\host\torque_envelope_app_test.c') -Raw
$adapter=$adapter.Replace('int main(void)','int existing_quota_adapter_main(void)')
$extra=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'pos_quota_regression.inc') -Raw
$testPath=Join-Path $outputDirectory 'pos_quota_regression.c'
Set-Content -LiteralPath $testPath -Value ($adapter+"`n"+$extra) -Encoding utf8
$runner=Get-Content -LiteralPath (Join-Path $FirmwareRoot 'Tests\host\run_torque_envelope_tests.ps1') -Raw
$runner=$runner.Replace("(Resolve-Path (Join-Path `$PSScriptRoot '..\..')).Path", "'$FirmwareRoot'")
$runner=$runner.Replace("Join-Path `$projectRoot 'output\torque-envelope'", "'$outputDirectory'")
$runner=$runner.Replace("'Tests/host/torque_envelope_app_test.c'", "'$testPath'")
$runner=$runner.Replace("'-IApp'", "'-I$FirmwareRoot/Tests/host' '-IApp'")
$runnerPath=Join-Path $outputDirectory 'run-generated.ps1'
Set-Content -LiteralPath $runnerPath -Value $runner -Encoding utf8
& $runnerPath
if($LASTEXITCODE -ne 0){throw 'POS quota regression failed'}
