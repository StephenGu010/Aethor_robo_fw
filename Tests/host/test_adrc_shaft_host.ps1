<# .SYNOPSIS Offline strict shaft-pilot host contracts; no serial port or debugger is opened. #>
$ErrorActionPreference = 'Stop'
$helperPath = Join-Path $PSScriptRoot '..\hardware\adrc_shaft_pilot_host.ps1'
if (-not (Test-Path -LiteralPath $helperPath)) { throw 'Shaft host helper is missing' }
. $helperPath
$script:shaftCases = 0
function Assert-ShaftReject {
    <# Requires malformed evidence to fail before it can influence host acceptance. #>
    param([scriptblock]$Operation)
    $rejected = $false
    try { & $Operation | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw 'Malformed shaft evidence was accepted' }
    ++$script:shaftCases
}
$schema = [ordered]@{ id='u32'; t='u64'; enabled='u8'; direction='i32'; p='f32' }
$valid = 'ok 123 adrc shaft_trace id=99 t=18446744073709551615 enabled=1 direction=-1 p=0.100000001'
$parsed = ConvertFrom-ShaftFields $valid 'shaft_trace' 123 $schema
if ($parsed.t -ne [uint64]::MaxValue -or $parsed.id -ne 99 -or $parsed.direction -ne -1 -or
    [single]$parsed.p -ne [single]0.1) { throw 'Valid shaft values did not round-trip' }
++ $script:shaftCases
foreach ($bad in @(
    ($valid + ' extra=1'), ($valid + ' id=99'), ($valid.Replace(' id=99','')),
    ($valid.Replace('id=99','id=4294967296')), ($valid.Replace('enabled=1','enabled=256')),
    ($valid.Replace('direction=-1','direction=-2147483649')),
    ($valid.Replace('t=18446744073709551615','t=18446744073709551616')),
    ($valid.Replace('t=18446744073709551615','t=-1')),
    ($valid.Replace('t=18446744073709551615','t=1.5')),
    ($valid.Replace('p=0.100000001','p=NaN')), ($valid.Replace('p=0.100000001','p=Infinity')),
    ($valid.Replace('p=0.100000001','p=1e400')), ($valid.Replace('p=0.100000001','p=3.5e38')),
    ($valid.Replace('ok 123','ok 124')), ($valid.Replace('shaft_trace','shaft_status')),
    ($valid.Replace('id=99','ID=99')), ($valid.Replace('enabled=1','enabled=1=2')),
    ($valid + (' ' * 520)))) {
    Assert-ShaftReject { ConvertFrom-ShaftFields $bad 'shaft_trace' 123 $schema }
}
$mainPath = Join-Path $PSScriptRoot '..\hardware\adrc_motor7_handoff_window.ps1'
$goodStatus = 'ok 81000701 adrc shaft_status id=99 phase=5 outcome=1 fault=0 safe_release=1 guard_ack=1 ready=1 count=100 bridge_fault=0 prepare=2 binding=12345 boot=42 last=0.5 peak=0.5 qualified=0 control=0'
$parsedStatus = ConvertFrom-ShaftStatus $goodStatus 99 42
Assert-ShaftStatusUnchanged $parsedStatus (ConvertFrom-ShaftStatus $goodStatus 99 42)
++ $script:shaftCases
foreach ($badStatus in @(
    ($goodStatus.Replace('id=99','id=98')), ($goodStatus.Replace('boot=42','boot=43')),
    ($goodStatus.Replace('phase=5','phase=6')), ($goodStatus.Replace('outcome=1','outcome=3')),
    ($goodStatus.Replace('fault=0','fault=1')), ($goodStatus.Replace('safe_release=1','safe_release=0')),
    ($goodStatus.Replace('guard_ack=1','guard_ack=0')), ($goodStatus.Replace('binding=12345','binding=0')),
    ($goodStatus.Replace('count=100','count=513')), ($goodStatus.Replace('peak=0.5','peak=1.76')),
    ($goodStatus.Replace('qualified=0','qualified=1')), ($goodStatus.Replace('control=0','control=1')),
    ($goodStatus.Replace('ready=1','ready=2')), ($goodStatus.Replace('prepare=2','prepare=3')))) {
    Assert-ShaftReject { ConvertFrom-ShaftStatus $badStatus 99 42 }
}
$failedStatus = ConvertFrom-ShaftStatus ($goodStatus.Replace('ready=1','ready=0').Replace('outcome=1','outcome=3').Replace('fault=0','fault=1')) 99 42
if ($failedStatus.ready -ne 0 -or $failedStatus.fault -eq 0) { throw 'Stopped failure lost its primary outcome' }
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftStatusUnchanged $parsedStatus $failedStatus }
$tokens = $null; $errors = $null
$mainAst = [System.Management.Automation.Language.Parser]::ParseFile($mainPath, [ref]$tokens, [ref]$errors)
if ($errors.Count) { throw 'Main handoff script syntax error' }
$preflightStatements = @()
foreach ($statement in $mainAst.EndBlock.Statements) {
    if ($statement.Extent.Text -match '^\$transcriptLines\s*=') { break }
    $preflightStatements += $statement.Extent.Text
}
$preflight = [scriptblock]::Create($mainAst.ParamBlock.Extent.Text + "`n" + ($preflightStatements -join "`n"))
Assert-ShaftReject { & $preflight -TestShaftPilot }
Assert-ShaftReject { & $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities }
Assert-ShaftReject { & $preflight -TestShaftPilot -SampleDisabledFeedback -TestIndependentStop }
Assert-ShaftReject { & $preflight -TestShaftPilot -ReadMotorCapabilities -TestIndependentStop }
Assert-ShaftReject { & $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -TestSimpleAdrc }
Assert-ShaftReject { & $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -TestMotorWatchdog }
Assert-ShaftReject { & $preflight -ShaftDirection negative }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftDirection negative
++ $script:shaftCases
foreach ($functionNode in $mainAst.FindAll({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
    $node.Name -eq 'Invoke-WindowRequest' }, $false)) { Invoke-Expression $functionNode.Extent.Text }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc shaft_trace' }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc shaft_trace' -TraceIndex 0 }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc shaft_trace' -TraceIndex 512 -TracePart 0 }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc shaft_trace' -TraceIndex 0 -TracePart 4 }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc shaft_binding' -CaptureIndex 4 }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc simple_trace' -TraceIndex 400 }
Assert-ShaftReject { Invoke-WindowRequest $null 1 'adrc simple_trace' -TraceIndex 0 -TracePart 0 }
$script:realShaftExporter = ${function:Export-ShaftPilotTrace}
$script:bindingReplies = @(
    'ok 81000704 adrc shaft_binding id=99 index=0 boot=42 session=180388626433 binding=12345 epoch=1 capture=90 guard=91 collector=92 direction=1 cap=1.75 slew=4 step=0.0160000008 qualified=0 control=0',
    'ok 81000704 adrc shaft_binding id=99 index=1 gr=19.2031994 pmax=12.5 vmax=200 tmax=10 points=64 span=1.98649e-5 velocity_peak=0.000620778 raw_center=-0.0488433838 torque_peak=0.0122098923 qnative=0.000381475547 qshaft=1.98652087e-5 qvelocity=0.0976800993 qtorque=0.00488400506',
    'ok 81000704 adrc shaft_binding id=99 index=2 motor=7 master=23 mode=1 timeout=0 fields=8191 hw=0 sw=808597046 sub=49 accel=2 decel=2 maxspeed=1200',
    'ok 81000704 adrc shaft_binding id=99 index=3 kt=0 ot=1120403456 oc=1061997773 damp=959569463 inertia=932735168 gr=1100587047 valid_mask=63'
)
$bindingFixture = @()
foreach ($part in 0..3) { $bindingFixture += ,(ConvertFrom-ShaftShard $script:bindingReplies[$part] 'shaft_binding' 99 $part $part) }
$script:templateToWire = @{ '81000300'=[uint32]90; '81000400'=[uint32]91; '81000310'=[uint32]92 }
$script:shaftCapabilityReadback = @{ id=[uint32]92; epoch=[uint32]1; rows=@(
    @{ raw_bits=[uint32]0 }, @{ raw_bits=[uint32]1120403456 }, @{ raw_bits=[uint32]1061997773 },
    @{ raw_bits=[uint32]959569463 }, @{ raw_bits=[uint32]932735168 }, @{ raw_bits=[uint32]1100587047 }) }
Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive
++ $script:shaftCases
$lowBindingFixture = @($bindingFixture | ForEach-Object { $copy = [ordered]@{}; foreach ($key in $_.Keys) { $copy[$key] = $_[$key] }; $copy })
$lowBindingFixture[0].cap = 0.349999994
$lowStatus = ConvertFrom-ShaftStatus ($goodStatus.Replace('outcome=1','outcome=2').Replace('last=0.5 peak=0.5','last=0.349206924 peak=0.349206924')) 99 42
Assert-ShaftBinding $lowBindingFixture 42 $lowStatus positive -Profile low
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $lowBindingFixture 42 $lowStatus positive -Profile default }
Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive -Profile low }
$lowBindingFixture[0].cap = 0.36
Assert-ShaftReject { Assert-ShaftBinding $lowBindingFixture 42 $lowStatus positive -Profile low }
$lowBindingFixture[0].cap = 0.349999994
Assert-ShaftReject { Assert-ShaftBinding $lowBindingFixture 42 $parsedStatus positive -Profile low }
Assert-ShaftReject { & $preflight -ShaftProfile low }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile low
++ $script:shaftCases
foreach ($name in @('capture','guard','collector')) {
    $originalId = $bindingFixture[0][$name]
    $bindingFixture[0][$name] = $originalId + 1
    Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive }
    $bindingFixture[0][$name] = $originalId
}
$script:shaftCapabilityReadback.epoch = 2
Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive }
$script:shaftCapabilityReadback.epoch = 1
$script:shaftCapabilityReadback.rows[0].raw_bits = [uint32]1
Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive }
$script:shaftCapabilityReadback.rows[0].raw_bits = [uint32]0
foreach ($part in 0..3) {
    Assert-ShaftReject { ConvertFrom-ShaftShard ($script:bindingReplies[$part] + ' qualified=0') 'shaft_binding' 99 $part $part }
    Assert-ShaftReject { ConvertFrom-ShaftShard ($script:bindingReplies[$part].Replace('id=99','id=98')) 'shaft_binding' 99 $part $part }
}
$script:traceReplies = @(
    'ok 81000702 adrc shaft_trace id=99 index=0 part=0 now=100000 binding=12345 session=180388626433 motor=7 ga=1 gl=0 gf=1 gack=1 stop=0 valid=1 rx=99000 observed=100000',
    'ok 81000702 adrc shaft_trace id=99 index=0 part=1 native_p=0 raw_v=-0.0488433838 torque=0.012 mos=37 rotor=28 driver=1 driver_fault=0',
    'ok 81000702 adrc shaft_trace id=99 index=0 part=2 rvalid=1 rsuccess=1 rkind=3 rcmd=99 rcompleted=99000 rdecoded=0 action=3 omotor=7 obinding=12345 osession=180388626433 ocmd=100 issued=100000 prev_valid=1 requested=0 previous=0.5 step=0 cap=1.75',
    'ok 81000702 adrc shaft_trace id=99 index=0 part=3 phase=5 outcome=1 fault=0 shaft_p=0 v_rx32=0 vready=1 conditional=1 sampling_unknown=1 rxold=67000 obsold=68000 rxnew=99000 obsnew=100000 tmin=29000 tmax=35000'
)
foreach ($part in 0..3) {
    ConvertFrom-ShaftShard $script:traceReplies[$part] 'shaft_trace' 99 0 $part | Out-Null
    ++$script:shaftCases
    Assert-ShaftReject { ConvertFrom-ShaftShard ($script:traceReplies[$part].Replace('index=0','index=1')) 'shaft_trace' 99 0 $part }
}
Assert-ShaftReject { ConvertFrom-ShaftShard ($script:traceReplies[0].Replace('ga=1','ga=2')) 'shaft_trace' 99 0 0 }
Assert-ShaftReject { ConvertFrom-ShaftShard ($script:traceReplies[1].Replace('driver=1','driver=3')) 'shaft_trace' 99 0 1 }
Assert-ShaftReject { ConvertFrom-ShaftShard ($script:traceReplies[2].Replace('rkind=3','rkind=4')) 'shaft_trace' 99 0 2 }
Assert-ShaftReject { ConvertFrom-ShaftShard ($script:traceReplies[3].Replace('fault=0','fault=17')) 'shaft_trace' 99 0 3 }
function Invoke-WindowRequest {
    <# Simulates only matching firmware replies; this fixture has no device object. #>
    param($Port, [uint32]$RequestId, [string]$Command, [int]$CaptureIndex = -1,
        [int]$TraceIndex = -1, [int]$TracePart = -1)
    $script:shaftCommands.Add($Command)
    if ($Command -eq 'ping') { return "ok $RequestId ping boot=42" }
    if ($Command -like 'adrc shaft_pilot direction=*') {
        $script:templateToWire['81000700'] = [uint32]99
        if ($script:shaftScenario -eq 'ack_lost') { throw 'fixture lost start ACK' }
        return "ok $RequestId adrc shaft_pilot accepted=1 motor=7 qualified=0 control=0"
    }
    if ($Command -eq 'adrc shaft_status') { return $script:fixtureStatus }
    if ($Command -eq 'adrc shaft_binding') { return $script:bindingReplies[$CaptureIndex] }
    if ($Command -eq 'adrc shaft_trace') {
        if ($script:shaftScenario -eq 'bad_shard') { return $script:traceReplies[$TracePart].Replace('id=99','id=98') }
        if ($script:shaftScenario -eq 'later_fault') {
            $script:fixtureStatus = $script:fixtureStatus.Replace('ready=1','ready=0').Replace('outcome=1','outcome=3').Replace('fault=0','fault=1')
        }
        return $script:traceReplies[$TracePart]
    }
    if ($Command -eq 'adrc evidence') { return "ok $RequestId adrc evidence verified=0 qualified=0" }
    throw "Unexpected fixture command: $Command"
}
foreach ($scenario in @('export_full','bad_shard','later_fault')) {
    $script:shaftScenario = $scenario
    $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
    $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
    $script:fixtureStatus = $goodStatus.Replace('count=100','count=1')
    $exportStatus = ConvertFrom-ShaftStatus $script:fixtureStatus 99 42
    if ($scenario -eq 'export_full') {
        & $script:realShaftExporter $null 99 42 $exportStatus positive
        if (-not $script:transcriptLines.Contains('shaft_trace_rows=1 closed_loop=0 qualification=0')) { throw 'Complete export was not recorded' }
        ++$script:shaftCases
    } else { Assert-ShaftReject { & $script:realShaftExporter $null 99 42 $exportStatus positive } }
}
function Export-ShaftPilotTrace {
    <# Replaces bounded trace export solely for orchestration fault fixtures. #>
    param($Port, $TrialId, $BootId, $Status, $Direction)
    ++$script:shaftExportCount
    if ($script:shaftScenario -eq 'export_error') { throw 'fixture export error' }
}
foreach ($scenario in @('success','failed','ack_lost','export_error')) {
    $script:shaftScenario = $scenario
    $script:shaftExportCount = 0
    $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
    $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
    $script:templateToWire = @{}
    $script:fixtureStatus = if ($scenario -eq 'failed') {
        $goodStatus.Replace('ready=1','ready=0').Replace('outcome=1','outcome=3').Replace('fault=0','fault=1')
    } else { $goodStatus }
    if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 positive; ++$script:shaftCases }
    else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive } }
    if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot direction=*' }).Count -ne 1) {
        throw 'Start was replayed or omitted'
    }
    if ($scenario -eq 'ack_lost' -and $script:shaftExportCount -ne 0) { throw 'Unknown active state was exported before cleanup' }
    if ($scenario -eq 'failed' -and $script:shaftExportCount -ne 1) { throw 'Safely stopped failed trace was discarded' }
    $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
    if (($scenario -eq 'success') -ne ($successLines.Count -eq 1)) { throw 'Host acceptance mismatched actual result' }
    ++$script:shaftCases
}
foreach ($scenario in @('success','failed','ack_lost')) {
    $script:shaftScenario = $scenario
    $script:shaftExportCount = 0
    $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
    $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
    $script:templateToWire = @{}
    $script:fixtureStatus = $goodStatus.Replace('outcome=1','outcome=2').Replace('last=0.5 peak=0.5','last=0.349206924 peak=0.349206924')
    if ($scenario -eq 'failed') { $script:fixtureStatus = $script:fixtureStatus.Replace('ready=1','ready=0').Replace('outcome=2','outcome=4') }
    if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 positive -Profile low; ++$script:shaftCases }
    else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive -Profile low } }
    $starts = @($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot direction=*' })
    if ($starts.Count -ne 1 -or $starts[0] -cne 'adrc shaft_pilot direction=positive profile=low') {
        throw 'Low-profile start was omitted, replayed or changed'
    }
    $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
    if (($scenario -eq 'success') -ne ($successLines.Count -eq 1) -or
        ($successLines.Count -eq 1 -and $successLines[0] -notlike '* profile=low *')) {
        throw 'Low-profile host acceptance lost its profile or failure'
    }
    ++$script:shaftCases
}
# Bind the fixed visible plan, not user-provided numeric torque or angle claims.
$visibleBinding = @($bindingFixture | ForEach-Object { $copy = [ordered]@{}; foreach ($key in $_.Keys) { $copy[$key] = $_[$key] }; $copy })
$visibleBinding[0].cap = 0.5; $visibleBinding[0].slew = 2; $visibleBinding[0].step = 0.00800000038
$script:templateToWire['81000300'] = [uint32]90
$script:templateToWire['81000400'] = [uint32]91
$script:templateToWire['81000310'] = [uint32]92
Assert-ShaftBinding $visibleBinding 42 $parsedStatus positive -Profile visible
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $visibleBinding 42 $parsedStatus positive -Profile low }
$visibleBinding[0].slew = 4
Assert-ShaftReject { Assert-ShaftBinding $visibleBinding 42 $parsedStatus positive -Profile visible }
$visibleBinding[0].slew = 2; $visibleBinding[0].step = 0.016
Assert-ShaftReject { Assert-ShaftBinding $visibleBinding 42 $parsedStatus positive -Profile visible }
Assert-ShaftReject { & $preflight -ShaftProfile visible }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile visible
++ $script:shaftCases
$script:shaftScenario = 'success'; $script:fixtureStatus = $goodStatus
$script:shaftCommands = [System.Collections.Generic.List[string]]::new()
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
Invoke-ShaftPilotTrial $null 42 positive -Profile visible
if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
    -not $script:shaftCommands.Contains('adrc shaft_pilot direction=positive profile=visible')) {
    throw 'Visible trial was not a single exact fixed-profile request'
}
++ $script:shaftCases
# The fixed lower-energy profile must bind its cap and VMAX=5 together.
$fineBinding = @($bindingFixture | ForEach-Object { $copy = [ordered]@{}; foreach ($key in $_.Keys) { $copy[$key] = $_[$key] }; $copy })
$fineBinding[0].cap = [single]0.20
$fineBinding[1].vmax = 5
$fineBinding[1].qvelocity = 10.0 / 4095
$fineBinding[1].raw_center = -0.00122117996
$fineStatus = ConvertFrom-ShaftStatus ($goodStatus.Replace('outcome=1','outcome=2').Replace('last=0.5 peak=0.5','last=0.197802544 peak=0.197802544')) 99 42
Assert-ShaftBinding $fineBinding 42 $fineStatus positive -Profile fine
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $fineBinding 42 $fineStatus positive -Profile low }
$fineBinding[1].vmax = 200
Assert-ShaftReject { Assert-ShaftBinding $fineBinding 42 $fineStatus positive -Profile fine }
$script:fixtureStatus = $goodStatus
$script:shaftCommands = [System.Collections.Generic.List[string]]::new()
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
Invoke-ShaftPilotTrial $null 42 positive -Profile fine
if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
    -not $script:shaftCommands.Contains('adrc shaft_pilot direction=positive profile=fine')) {
    throw 'Fine trial was not a single exact fixed-profile request'
}
++ $script:shaftCases
# Inspect the real outer request whitelist; the orchestration stub cannot detect this boundary.
$parseTokens = $null; $parseErrors = $null
$windowAst = [System.Management.Automation.Language.Parser]::ParseFile(
    (Join-Path $PSScriptRoot '..\hardware\adrc_motor7_handoff_window.ps1'), [ref]$parseTokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw 'Actual handoff source contains syntax errors' }
$requestFunction = $windowAst.Find({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Invoke-WindowRequest'
}, $true)
$commandParameter = $requestFunction.Body.ParamBlock.Parameters | Where-Object { $_.Name.VariablePath.UserPath -eq 'Command' }
$commandValidation = $commandParameter.Attributes | Where-Object { $_.TypeName.Name -eq 'ValidateSet' }
$actualCommandWhitelist = @($commandValidation.PositionalArguments | ForEach-Object { $_.Value })
foreach ($direction in @('positive','negative')) {
    if (-not $actualCommandWhitelist.Contains('adrc shaft_pilot direction=' + $direction + ' profile=fine')) {
        throw 'Actual outer request whitelist is missing the fixed fine command'
    }
    ++ $script:shaftCases
}
# Observe shares default torque values, so the actual request boundary and fifth shard must prove its identity.
foreach ($direction in @('positive','negative')) {
    if (-not $actualCommandWhitelist.Contains('adrc shaft_pilot direction=' + $direction + ' profile=observe')) {
        throw 'Actual outer request whitelist is missing the fixed observe command'
    }
    ++ $script:shaftCases
}
Assert-ShaftReject { & $preflight -ShaftProfile observe }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile observe
$observeReply = 'ok 81000704 adrc shaft_binding id=99 index=4 profile=4 speed=20.6821518 angle=0.52359879 reverse=0.0872664601 active=1.57079637 stopped=3.14159274 ramp_us=900000 plateau_us=200000 rated_rpm=395'
$observePart = ConvertFrom-ShaftShard $observeReply 'shaft_binding' 99 4 4
$observeBinding = @($bindingFixture) + @($observePart)
Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile observe
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $parsedStatus positive -Profile observe }
Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile default }
Assert-ShaftReject { ConvertFrom-ShaftShard $observeReply 'shaft_trace' 99 4 4 }
foreach ($name in @('id','index','profile','speed','angle','reverse','active','stopped','ramp_us','plateau_us','rated_rpm')) {
    $originalValue = $observePart[$name]
    $observePart[$name] = $originalValue + 1
    Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile observe }
    $observePart[$name] = $originalValue
}
foreach ($name in @('speed','angle','reverse','active','stopped','rated_rpm')) {
    $originalValue = $observePart[$name]
    $originalBits = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$originalValue),0)
    $observePart[$name] = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]($originalBits + 1)),0)
    Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile observe }
    $observePart[$name] = $originalValue
}
foreach ($badReply in @($observeReply.Replace(' profile=4',''),($observeReply + ' extra=1'),
    $observeReply.Replace('speed=20.6821518','speed=NaN'),$observeReply.Replace('ramp_us=900000','ramp_us=900000.5'))) {
    Assert-ShaftReject { ConvertFrom-ShaftShard $badReply 'shaft_binding' 99 4 4 }
}
$observePart.extra = 1
Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile observe }
$observePart.Remove('extra')
foreach ($name in @('pmax','vmax','tmax')) {
    $originalValue = $bindingFixture[1][$name]
    $bindingFixture[1][$name] = if ($name -eq 'vmax') { 5 } else { $originalValue + 1 }
    Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $parsedStatus positive -Profile observe }
    $bindingFixture[1][$name] = $originalValue
}
Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive -Profile 'observe cap=2' }
$script:bindingReplies += $observeReply
$script:fixtureStatus = $goodStatus.Replace('count=100','count=1')
$script:shaftScenario = 'export_full'
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
& $script:realShaftExporter $null 99 42 (ConvertFrom-ShaftStatus $script:fixtureStatus 99 42) positive -Profile observe
$observeExport = @($script:transcriptLines | Where-Object { $_ -like 'shaft_binding_data=*' })
if ($observeExport.Count -ne 1 -or @((ConvertFrom-Json $observeExport[0].Substring(19))).Count -ne 5) {
    throw 'Observe exporter did not retain exactly five frozen binding shards'
}
++ $script:shaftCases
foreach ($direction in @('positive','negative')) {
    foreach ($scenario in @('success','failed','ack_lost')) {
        $script:shaftScenario = $scenario
        $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
        $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
        $script:fixtureStatus = if ($scenario -eq 'failed') {
            $goodStatus.Replace('ready=1','ready=0').Replace('outcome=1','outcome=4')
        } else { $goodStatus }
        if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 $direction -Profile observe }
        else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 $direction -Profile observe } }
        $expectedCommand = 'adrc shaft_pilot direction=' + $direction + ' profile=observe'
        $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
        if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
            -not $script:shaftCommands.Contains($expectedCommand) -or
            (($scenario -eq 'success') -ne ($successLines.Count -eq 1)) -or
            ($successLines.Count -eq 1 -and $successLines[0] -notlike "*direction=$direction profile=observe peak=*")) {
            throw 'Observe orchestration lost single-shot direction/profile identity or accepted failure'
        }
        ++ $script:shaftCases
    }
}
# Platform uses an independent fifth-shard schema and can complete successfully only on TIME.
foreach ($direction in @('positive','negative')) {
    if (-not $actualCommandWhitelist.Contains('adrc shaft_pilot direction=' + $direction + ' profile=platform')) {
        throw 'Actual outer request whitelist is missing the fixed platform command'
    }
    ++ $script:shaftCases
}
Assert-ShaftReject { & $preflight -ShaftProfile platform }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile platform
$platformReply = 'ok 81000704 adrc shaft_binding id=99 index=4 profile=5 speed=20.6821518 reverse=0.0872664601 reserve=3.14159274 lookahead_us=25000 ramp_us=900000 plateau_us=200000 rated_rpm=395'
$platformPart = ConvertFrom-ShaftShard $platformReply 'shaft_binding' 99 4 4 -Profile platform
$platformBinding = @($bindingFixture) + @($platformPart)
$platformStatus = ConvertFrom-ShaftStatus ($goodStatus.Replace('outcome=1','outcome=2')) 99 42
Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile platform
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $bindingFixture 42 $platformStatus positive -Profile platform }
Assert-ShaftReject { Assert-ShaftBinding $observeBinding 42 $platformStatus positive -Profile platform }
Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile observe }
Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $parsedStatus positive -Profile platform }
Assert-ShaftReject { ConvertFrom-ShaftShard $platformReply 'shaft_binding' 99 4 4 }
Assert-ShaftReject { ConvertFrom-ShaftShard $observeReply 'shaft_binding' 99 4 4 -Profile platform }
foreach ($name in @('id','index','profile','speed','reverse','reserve','lookahead_us','ramp_us','plateau_us','rated_rpm')) {
    $originalValue = $platformPart[$name]
    $platformPart[$name] = $originalValue + 1
    Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile platform }
    $platformPart[$name] = $originalValue
}
foreach ($name in @('speed','reverse','reserve','rated_rpm')) {
    $originalValue = $platformPart[$name]
    $originalBits = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$originalValue),0)
    $platformPart[$name] = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]($originalBits + 1)),0)
    Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile platform }
    $platformPart[$name] = $originalValue
}
foreach ($name in @('pmax','vmax','tmax')) {
    $originalValue = $bindingFixture[1][$name]
    $bindingFixture[1][$name] = $originalValue + 1
    Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile platform }
    $bindingFixture[1][$name] = $originalValue
}
foreach ($badReply in @($platformReply.Replace(' profile=5',''),($platformReply + ' angle=0.52359879'),
    $platformReply.Replace('reserve=3.14159274','reserve=NaN'),$platformReply.Replace('lookahead_us=25000','lookahead_us=25000.5'))) {
    Assert-ShaftReject { ConvertFrom-ShaftShard $badReply 'shaft_binding' 99 4 4 -Profile platform }
}
Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive -Profile 'platform cap=2' }
$script:bindingReplies[4] = $platformReply
$script:fixtureStatus = $goodStatus.Replace('count=100','count=1').Replace('outcome=1','outcome=2')
$originalTraceOutcome = $script:traceReplies[3]
$script:traceReplies[3] = $originalTraceOutcome.Replace('outcome=1','outcome=2')
$script:shaftScenario = 'export_full'
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
& $script:realShaftExporter $null 99 42 (ConvertFrom-ShaftStatus $script:fixtureStatus 99 42) positive -Profile platform
$platformExport = @($script:transcriptLines | Where-Object { $_ -like 'shaft_binding_data=*' })
if ($platformExport.Count -ne 1 -or @((ConvertFrom-Json $platformExport[0].Substring(19))).Count -ne 5) {
    throw 'Platform exporter did not retain exactly five frozen binding shards'
}
++ $script:shaftCases
$script:traceReplies[3] = $originalTraceOutcome
foreach ($direction in @('positive','negative')) {
    foreach ($scenario in @('success','failed','ack_lost','angle_result')) {
        $script:shaftScenario = $scenario
        $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
        $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
        $script:fixtureStatus = $goodStatus.Replace('outcome=1','outcome=2')
        if ($scenario -eq 'failed') { $script:fixtureStatus = $script:fixtureStatus.Replace('ready=1','ready=0').Replace('outcome=2','outcome=4') }
        if ($scenario -eq 'angle_result') { $script:fixtureStatus = $goodStatus }
        if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform }
        else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform } }
        $expectedCommand = 'adrc shaft_pilot direction=' + $direction + ' profile=platform'
        $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
        if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
            -not $script:shaftCommands.Contains($expectedCommand) -or
            (($scenario -eq 'success') -ne ($successLines.Count -eq 1)) -or
            ($successLines.Count -eq 1 -and $successLines[0] -notlike "*direction=$direction profile=platform peak=*")) {
            throw 'Platform orchestration lost single-shot identity or accepted a non-TIME completion'
        }
        ++ $script:shaftCases
    }
}
# Wide position binds its own profile6 schema, range and new position quantums.
foreach ($direction in @('positive','negative')) {
    if (-not $actualCommandWhitelist.Contains('adrc shaft_pilot direction=' + $direction + ' profile=platform_wide')) {
        throw 'Actual outer request whitelist is missing the fixed platform_wide command'
    }
    ++ $script:shaftCases
}
Assert-ShaftReject { & $preflight -ShaftProfile platform_wide }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile platform_wide
++ $script:shaftCases
$wideReply = 'ok 81000704 adrc shaft_binding id=99 index=4 profile=6 speed=20.6821518 reverse=0.0872664601 reserve=12.566371 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395'
$widePart = ConvertFrom-ShaftShard $wideReply 'shaft_binding' 99 4 4 -Profile platform_wide
$wideBinding = @($bindingFixture | ForEach-Object { $copy = [ordered]@{}; foreach ($key in $_.Keys) { $copy[$key] = $_[$key] }; $copy }) + @($widePart)
$wideBinding[1].pmax = 512
$wideBinding[1].qnative = 1024.0 / 65535
$wideBinding[1].qshaft = $wideBinding[1].qnative / $wideBinding[1].gr
Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform_wide
++ $script:shaftCases
Assert-ShaftReject { Assert-ShaftBinding $platformBinding 42 $platformStatus positive -Profile platform_wide }
Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform }
Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $parsedStatus positive -Profile platform_wide }
Assert-ShaftReject { ConvertFrom-ShaftShard $wideReply 'shaft_binding' 99 4 4 -Profile platform }
Assert-ShaftReject { ConvertFrom-ShaftShard $platformReply 'shaft_binding' 99 4 4 -Profile platform_wide }
foreach ($name in @('id','index','profile','speed','reverse','reserve','lookahead_us','entry_us','ramp_us','plateau_us','rated_rpm')) {
    $originalValue = $widePart[$name]
    $widePart[$name] = $originalValue + 1
    Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform_wide }
    $widePart[$name] = $originalValue
}
foreach ($name in @('speed','reverse','reserve','rated_rpm')) {
    $originalValue = $widePart[$name]
    $originalBits = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$originalValue),0)
    $widePart[$name] = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]($originalBits + 1)),0)
    Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform_wide }
    $widePart[$name] = $originalValue
}
foreach ($name in @('pmax','vmax','tmax','qnative','qshaft','qvelocity','qtorque')) {
    $originalValue = $wideBinding[1][$name]
    $wideBinding[1][$name] = if ($name -in @('qnative','qshaft')) { $bindingFixture[1][$name] } else { $originalValue + 1 }
    Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform_wide }
    $wideBinding[1][$name] = $originalValue
}
foreach ($name in @('valid_mask','gr')) {
    $originalValue = $wideBinding[3][$name]
    $wideBinding[3][$name] = 0
    Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform_wide }
    $wideBinding[3][$name] = $originalValue
}
foreach ($badReply in @($wideReply.Replace(' entry_us=1058000',''),($wideReply + ' angle=0.52359879'),
    $wideReply.Replace('reserve=12.566371','reserve=NaN'),$wideReply.Replace('entry_us=1058000','entry_us=1058000.5'))) {
    Assert-ShaftReject { ConvertFrom-ShaftShard $badReply 'shaft_binding' 99 4 4 -Profile platform_wide }
}
Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive -Profile 'platform_wide cap=2' }
$oldRangeReply = $script:bindingReplies[1]
$script:bindingReplies[1] = $oldRangeReply.Replace('pmax=12.5','pmax=512').Replace('qnative=0.000381475547','qnative=0.0156252384').Replace('qshaft=1.98652087e-5','qshaft=0.000813678838')
$script:bindingReplies[4] = $wideReply
$script:fixtureStatus = $goodStatus.Replace('count=100','count=1').Replace('outcome=1','outcome=2')
$script:traceReplies[3] = $originalTraceOutcome.Replace('outcome=1','outcome=2')
$script:shaftScenario = 'export_full'
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
& $script:realShaftExporter $null 99 42 (ConvertFrom-ShaftStatus $script:fixtureStatus 99 42) positive -Profile platform_wide
$wideExport = @($script:transcriptLines | Where-Object { $_ -like 'shaft_binding_data=*' })
if ($wideExport.Count -ne 1 -or @((ConvertFrom-Json $wideExport[0].Substring(19))).Count -ne 5) {
    throw 'Wide platform exporter did not retain exactly five frozen binding shards'
}
++ $script:shaftCases
$script:bindingReplies[1] = $oldRangeReply
$script:traceReplies[3] = $originalTraceOutcome
foreach ($direction in @('positive','negative')) {
    foreach ($scenario in @('success','failed','ack_lost','angle_result')) {
        $script:shaftScenario = $scenario
        $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
        $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
        $script:fixtureStatus = $goodStatus.Replace('outcome=1','outcome=2')
        if ($scenario -eq 'failed') { $script:fixtureStatus = $script:fixtureStatus.Replace('ready=1','ready=0').Replace('outcome=2','outcome=4') }
        if ($scenario -eq 'angle_result') { $script:fixtureStatus = $goodStatus }
        if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform_wide }
        else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform_wide } }
        $expectedCommand = 'adrc shaft_pilot direction=' + $direction + ' profile=platform_wide'
        $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
        if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
            -not $script:shaftCommands.Contains($expectedCommand) -or
            (($scenario -eq 'success') -ne ($successLines.Count -eq 1)) -or
            ($successLines.Count -eq 1 -and $successLines[0] -notlike "*direction=$direction profile=platform_wide peak=*")) {
            throw 'Wide platform orchestration lost single-shot identity or accepted a non-TIME completion'
        }
        ++ $script:shaftCases
    }
}
# Execute only the real outer request validation prefix, ending before request IDs or device writes.
$validationStatements = @()
foreach ($statement in $requestFunction.Body.EndBlock.Statements) {
    if ($statement.Extent.Text -match '^if \(\$script:wireRequestSequence') { break }
    $validationStatements += $statement.Extent.Text
}
$requestValidation = [scriptblock]::Create($requestFunction.Body.ParamBlock.Extent.Text + "`n" + ($validationStatements -join "`n"))
foreach ($ShaftProfile in @('observe','platform','platform_wide','platform70')) {
    & $requestValidation -RequestId 1 -Command 'adrc shaft_binding' -CaptureIndex 4
    ++ $script:shaftCases
    Assert-ShaftReject { & $requestValidation -RequestId 1 -Command 'adrc shaft_binding' -CaptureIndex 5 }
}
$ShaftProfile = 'default'
Assert-ShaftReject { & $requestValidation -RequestId 1 -Command 'adrc shaft_binding' -CaptureIndex 4 }
# Rated70 has independent profile7 identity, PMAX1024 and four native turns of minimum reserve.
foreach ($direction in @('positive','negative')) {
    if (-not $actualCommandWhitelist.Contains('adrc shaft_pilot direction=' + $direction + ' profile=platform70')) {
        throw 'Actual outer request whitelist is missing the fixed platform70 command'
    }
    ++ $script:shaftCases
}
Assert-ShaftReject { & $preflight -ShaftProfile platform70 }
& $preflight -TestShaftPilot -SampleDisabledFeedback -ReadMotorCapabilities -TestIndependentStop -ShaftProfile platform70
++ $script:shaftCases
$rated70Reply = 'ok 81000704 adrc shaft_binding id=99 index=4 profile=7 speed=28.9550114 reverse=0.0872664601 reserve=25.1327419 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395'
$rated70Part = ConvertFrom-ShaftShard $rated70Reply 'shaft_binding' 99 4 4 -Profile platform70
$rated70Binding = @($bindingFixture | ForEach-Object { $copy = [ordered]@{}; foreach ($key in $_.Keys) { $copy[$key] = $_[$key] }; $copy }) + @($rated70Part)
$rated70Binding[1].pmax = 1024
$rated70Binding[1].qnative = 2048.0 / 65535
$rated70Binding[1].qshaft = $rated70Binding[1].qnative / $rated70Binding[1].gr
foreach ($direction in @('positive','negative')) {
    $rated70Binding[0].direction = if ($direction -eq 'positive') { 1 } else { -1 }
    Assert-ShaftBinding $rated70Binding 42 $platformStatus $direction -Profile platform70
    ++ $script:shaftCases
}
$rated70Binding[0].direction = 1
foreach ($oldProfile in @('default','observe','platform','platform_wide')) {
    Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $platformStatus positive -Profile $oldProfile }
    Assert-ShaftReject { Assert-ShaftBinding $rated70Binding[0..3] 42 $platformStatus positive -Profile $oldProfile }
}
Assert-ShaftReject { Assert-ShaftBinding $rated70Binding[0..3] 42 $platformStatus positive -Profile platform70 }
Assert-ShaftReject { Assert-ShaftBinding $wideBinding 42 $platformStatus positive -Profile platform70 }
Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $parsedStatus positive -Profile platform70 }
foreach ($name in @('id','index','profile','speed','reverse','reserve','lookahead_us','entry_us','ramp_us','plateau_us','rated_rpm')) {
    $originalValue = $rated70Part[$name]
    $rated70Part[$name] = $originalValue + 1
    Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $platformStatus positive -Profile platform70 }
    $rated70Part[$name] = $originalValue
}
foreach ($name in @('speed','reverse','reserve','rated_rpm')) {
    $originalValue = $rated70Part[$name]
    $originalBits = [BitConverter]::ToUInt32([BitConverter]::GetBytes([single]$originalValue),0)
    foreach ($neighborBits in @([uint32]($originalBits - 1),[uint32]($originalBits + 1))) {
        $rated70Part[$name] = [BitConverter]::ToSingle([BitConverter]::GetBytes($neighborBits),0)
        Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $platformStatus positive -Profile platform70 }
    }
    $rated70Part[$name] = $originalValue
}
foreach ($name in @('pmax','vmax','tmax','qnative','qshaft','qvelocity','qtorque')) {
    $originalValue = $rated70Binding[1][$name]
    $rated70Binding[1][$name] = if ($name -in @('pmax','qnative','qshaft')) { $wideBinding[1][$name] } else { $originalValue + 1 }
    Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $platformStatus positive -Profile platform70 }
    $rated70Binding[1][$name] = $originalValue
}
foreach ($part in 0..3) {
    foreach ($name in @('id','index')) {
        $originalValue = $rated70Binding[$part][$name]
        $rated70Binding[$part][$name] = $originalValue + 1
        Assert-ShaftReject { Assert-ShaftBinding $rated70Binding 42 $platformStatus positive -Profile platform70 }
        $rated70Binding[$part][$name] = $originalValue
    }
}
foreach ($badReply in @($rated70Reply.Replace(' entry_us=1058000',''),($rated70Reply + ' angle=0.52359879'),
    $rated70Reply.Replace('speed=28.9550114','speed=NaN'),$rated70Reply.Replace('entry_us=1058000','entry_us=1058000.5'))) {
    Assert-ShaftReject { ConvertFrom-ShaftShard $badReply 'shaft_binding' 99 4 4 -Profile platform70 }
}
Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 positive -Profile 'platform70 cap=2' }
$script:bindingReplies[1] = $oldRangeReply.Replace('pmax=12.5','pmax=1024').Replace('qnative=0.000381475547','qnative=0.0312504768').Replace('qshaft=1.98652087e-5','qshaft=0.00162735768')
$script:bindingReplies[4] = $rated70Reply
$script:fixtureStatus = $goodStatus.Replace('count=100','count=1').Replace('outcome=1','outcome=2')
$script:traceReplies[3] = $originalTraceOutcome.Replace('outcome=1','outcome=2')
$script:shaftScenario = 'export_full'
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
& $script:realShaftExporter $null 99 42 (ConvertFrom-ShaftStatus $script:fixtureStatus 99 42) positive -Profile platform70
$rated70Export = @($script:transcriptLines | Where-Object { $_ -like 'shaft_binding_data=*' })
if ($rated70Export.Count -ne 1 -or @((ConvertFrom-Json $rated70Export[0].Substring(19))).Count -ne 5) {
    throw 'Rated70 exporter did not retain exactly five frozen binding shards'
}
++ $script:shaftCases
$script:bindingReplies[1] = $oldRangeReply
$script:traceReplies[3] = $originalTraceOutcome
foreach ($direction in @('positive','negative')) {
    foreach ($scenario in @('success','failed','ack_lost','angle_result')) {
        $script:shaftScenario = $scenario
        $script:shaftCommands = [System.Collections.Generic.List[string]]::new()
        $script:transcriptLines = [System.Collections.Generic.List[string]]::new()
        $script:fixtureStatus = $goodStatus.Replace('outcome=1','outcome=2')
        if ($scenario -eq 'failed') { $script:fixtureStatus = $script:fixtureStatus.Replace('ready=1','ready=0').Replace('outcome=2','outcome=4') }
        if ($scenario -eq 'angle_result') { $script:fixtureStatus = $goodStatus }
        if ($scenario -eq 'success') { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform70 }
        else { Assert-ShaftReject { Invoke-ShaftPilotTrial $null 42 $direction -Profile platform70 } }
        $expectedCommand = 'adrc shaft_pilot direction=' + $direction + ' profile=platform70'
        $successLines = @($script:transcriptLines | Where-Object { $_ -like 'shaft_pilot_completed=1*' })
        if (@($script:shaftCommands | Where-Object { $_ -like 'adrc shaft_pilot*' }).Count -ne 1 -or
            -not $script:shaftCommands.Contains($expectedCommand) -or
            (($scenario -eq 'success') -ne ($successLines.Count -eq 1)) -or
            ($successLines.Count -eq 1 -and $successLines[0] -notlike "*direction=$direction profile=platform70 peak=*")) {
            throw 'Rated70 orchestration lost single-shot identity or accepted a non-TIME completion'
        }
        ++ $script:shaftCases
    }
}
Write-Output "ADRC_SHAFT_HOST_TESTS_PASSED cases=$script:shaftCases"
