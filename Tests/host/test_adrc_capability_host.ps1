<# .SYNOPSIS Offline capability reply/preflight contracts; parses functions without running serial code. #>
$ErrorActionPreference = 'Stop'
$sourcePath = Join-Path $PSScriptRoot '..\hardware\adrc_motor7_handoff_window.ps1'
$parseTokens = $null; $parseErrors = $null
$sourceAst = [System.Management.Automation.Language.Parser]::ParseFile($sourcePath, [ref]$parseTokens, [ref]$parseErrors)
if ($parseErrors.Count) { throw 'Capability hardware script has parse errors' }
foreach ($functionNode in $sourceAst.FindAll({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] }, $false)) {
    Invoke-Expression $functionNode.Extent.Text
}
$caseCount = 0
function Assert-CapabilityRejected {
    <# Requires a malformed protocol response to fail without opening hardware. #>
    param([scriptblock]$Operation)
    $failed = $false
    try { & $Operation | Out-Null } catch { $failed = $true }
    if (-not $failed) { throw 'Capability malformed input was accepted' }
    ++$script:caseCount
}
$row = 'ok 1 adrc capabilities_get id=99 epoch=1 current=1 index=0 rid=1 name=KT valid=1 raw_bits=1036831949 value=0.100000001 positive=1 request_created_us=1000 queue_submitted_us=5000 pre_submit_boundary_us=5000 send_completed_observed_us=6000 rx_lower_us=5000 received_observed_us=6000 actual_send_us=0 model_known=0 qualified=0'
$result = ConvertFrom-MotorCapabilityReply $row 'capabilities_get' 99 0
if ($result.raw_bits -ne 1036831949 -or $result.rx_lower_us -ne 5000) { throw 'Valid row lost raw bits or timing' }
++ $caseCount
foreach ($badRow in @(
    ($row + ' qualified=0'), ($row + ' extra=1'), ($row.Replace(' qualified=0','')),
    ($row.Replace('value=0.100000001','value=NaN')), ($row.Replace('value=0.100000001','value=Infinity')),
    ($row.Replace('raw_bits=1036831949','raw_bits=4294967296')), ($row.Replace('epoch=1','epoch=4294967296')),
    ($row.Replace('request_created_us=1000','request_created_us=18446744073709551616')),
    ($row.Replace('rx_lower_us=5000','rx_lower_us=4999')), ($row.Replace('current=1','current=0')),
    ($row.Replace('qualified=0','qualified=1')), ($row.Replace('valid=1','valid=0')),
    ($row.Replace('name=KT','name=Gr')), ($row.Replace('id=99','id=100')),
    ($row.Replace('received_observed_us=6000','received_observed_us=22000')),
    ($row.Replace('send_completed_observed_us=6000','send_completed_observed_us=4999')),
    ($row.Replace('queue_submitted_us=5000','queue_submitted_us=1000')),
    ($row.Replace('value=0.100000001','value=-0.1')), ($row.Replace('positive=1','positive=0')),
    ($row.Replace('raw_bits=1036831949','raw_bits=2139095040')),
    ($row.Replace('actual_send_us=0','actual_send_us=6000')))) {
    Assert-CapabilityRejected { ConvertFrom-MotorCapabilityReply $badRow 'capabilities_get' 99 0 }
}
$status = 'ok 2 adrc capabilities_status id=99 epoch=1 current=1 motor=7 state=2 prepare=2 reason=0 count=6 rejected=0 qualified=0 model_known=0 actual_send_us=0'
ConvertFrom-MotorCapabilityReply $status 'capabilities_status' 99 | Out-Null
++ $caseCount
foreach ($field in @('id','epoch','current','motor','state','prepare','reason','count','rejected','qualified','model_known','actual_send_us')) {
    Assert-CapabilityRejected { ConvertFrom-MotorCapabilityReply ($status -replace (' ' + $field + '=\d+'), '') 'capabilities_status' 99 }
}
$preflightStatements = @()
foreach ($statement in $sourceAst.EndBlock.Statements) {
    if ($statement.Extent.Text -match '^\$transcriptLines\s*=') { break }
    $preflightStatements += $statement.Extent.Text
}
$preflight = [scriptblock]::Create($sourceAst.ParamBlock.Extent.Text + "`n" + ($preflightStatements -join "`n"))
Assert-CapabilityRejected { & $preflight -ReadMotorCapabilities }
Assert-CapabilityRejected { & $preflight -ReadMotorCapabilities -SampleDisabledFeedback -TestSimpleAdrc -TestIndependentStop }
Assert-CapabilityRejected { & $preflight -ReadMotorCapabilities -SampleDisabledFeedback -TestMotorWatchdog -TestIndependentStop }
& $preflight -ReadMotorCapabilities -SampleDisabledFeedback
++ $caseCount
Write-Output "ADRC_CAPABILITY_HOST_TESTS_PASSED cases=$caseCount"
