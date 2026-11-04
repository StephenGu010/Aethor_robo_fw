<#
.SYNOPSIS
Offline ADRC simple orchestration contract tests; never opens a serial port.
.DESCRIPTION
Loads function AST nodes only, then substitutes deterministic protocol replies.
Profile parameter checks execute only the parsed preflight, never the hardware main block.
#>
$ErrorActionPreference = 'Stop'
$sourcePath = Join-Path $PSScriptRoot '..\hardware\adrc_motor7_handoff_window.ps1'
$parseTokens = $null
$parseErrors = $null
$sourceAst = [System.Management.Automation.Language.Parser]::ParseFile(
    $sourcePath, [ref]$parseTokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw "Parse errors: $parseErrors" }
$sourceText = Get-Content -LiteralPath $sourcePath -Raw
$functionNodes = $sourceAst.FindAll({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst]
}, $false)
foreach ($functionNode in $functionNodes) { Invoke-Expression $functionNode.Extent.Text }

function Assert-Test {
    <# Fails the offline suite with a specific contract description. #>
    param([bool]$Condition, [string]$Description)
    if (-not $Condition) { throw "Contract failed: $Description" }
}

function Assert-Rejected {
    <# Requires an operation to fail with the expected reason. #>
    param([scriptblock]$Operation, [string]$Pattern)
    $failure = $null
    try { & $Operation | Out-Null } catch { $failure = $_.Exception.Message }
    Assert-Test ($null -ne $failure -and $failure -match $Pattern) "expected rejection $Pattern; got $failure"
}

# Parameter validation occurs before any serial write, and requires no port.
Assert-Rejected { Invoke-WindowRequest $null 1 'adrc simple motor=7' } 'ValidateSet|validation|valid|验证'
Assert-Rejected { Invoke-WindowRequest $null 1 'adrc simple_trace' } 'requires an index'
Assert-Rejected { Invoke-WindowRequest $null 1 'adrc simple' -TraceIndex 0 } 'Trace index'
Assert-Rejected { Invoke-WindowRequest $null 1 'adrc capture' -CaptureIndex 64 } 'ValidateRange|validation|valid|验证'
Assert-Rejected { Invoke-WindowRequest $null 1 'adrc simple_trace' -TraceIndex 400 } 'ValidateRange|validation|valid|验证'
$additionalCases = 0
$allowedCommands = (Get-Command Invoke-WindowRequest).Parameters['Command'].Attributes |
    Where-Object { $_ -is [System.Management.Automation.ValidateSetAttribute] }
foreach ($profileNumber in 1..7) {
    Assert-Test ($allowedCommands.ValidValues -contains "adrc simple profile=$profileNumber") 'reviewed profile command allowlisted'
    ++$additionalCases
}
foreach ($invalidCommand in @('adrc simple profile=0', 'adrc simple profile=9',
        'adrc simple profile=-1', 'adrc simple profile=01',
        'adrc simple profile=1 torque=1', 'adrc simple profile=2 profile=3')) {
    Assert-Rejected { Invoke-WindowRequest $null 1 $invalidCommand } 'ValidateSet|validation|valid|验证'
    ++$additionalCases
}
# Build a preflight-only script from the actual parameter block and initial guards.
$preflightStatements = @()
foreach ($statement in $sourceAst.EndBlock.Statements) {
    if ($statement.Extent.Text -match '^\$transcriptLines\s*=') { break }
    $preflightStatements += $statement.Extent.Text
}
$preflightOnly = [scriptblock]::Create($sourceAst.ParamBlock.Extent.Text + "`n" + ($preflightStatements -join "`n"))
foreach ($profileNumber in 1..7) {
    Assert-Rejected { & $preflightOnly -SimpleProfile $profileNumber } 'requires TestSimpleAdrc'
    & $preflightOnly -SimpleProfile $profileNumber -TestSimpleAdrc -TestIndependentStop -SampleDisabledFeedback
    ++$additionalCases
}
foreach ($invalidProfile in @(-1, 9)) {
    Assert-Rejected { & $preflightOnly -SimpleProfile $invalidProfile } 'ValidateRange|validation|valid|验证'
    ++$additionalCases
}
# The independent finite probe has a distinct result and must bind every envelope field.
$probeStatus = 'ok 81000601 adrc simple_status id=7 phase=9 result=5 fault=0 safe_release=1 guard_ack=1 b0=0 count=194 bridge_fault=0 profile=8 pulse_us=640000 limit=1.75 positive_us=640000 negative_us=0 torque_trip=1.85 step=0.016 slew=4 probe=1 probe_peak=1.746032 probe_reached=1'
Assert-Test ((ConvertFrom-SimpleStatus $probeStatus 7 8).Result -eq 5) 'probe completed differs from closed loop success'
++$additionalCases
$legacyStatus = 'ok 81000601 adrc simple_status id=7 phase=9 result=1 fault=0 safe_release=1 guard_ack=1 b0=1.25 count=400 bridge_fault=0'
Assert-Test ((ConvertFrom-SimpleStatus $legacyStatus 7).Profile -eq 0) 'legacy status valid only for default profile'
++ $additionalCases
foreach ($profileNumber in 0..7) {
    $profileStatus = $legacyStatus + " profile=$profileNumber pulse_us=$(@(80000,160000,200000,240000,360000,360000,360000,360000,640000)[$profileNumber]) limit=$(@(0.03,0.05,0.08,0.10,0.14,0.20,0.25,0.35,1.75)[$profileNumber])"
    if ($profileNumber -ge 4) { $profileStatus += " positive_us=312000 negative_us=360000" }
    if ($profileNumber -ge 5) { $profileStatus += " torque_trip=$(@(0.23,0.28,0.40)[$profileNumber-5]) step=$(@(0.012,0.012,0.016)[$profileNumber-5]) slew=$(@(3,3,4)[$profileNumber-5])" }
    Assert-Test ((ConvertFrom-SimpleStatus $profileStatus 7 $profileNumber).Profile -eq $profileNumber) 'profile echo bound to request'
    Assert-Rejected { ConvertFrom-SimpleStatus $profileStatus 7 (($profileNumber + 1) % 8) } 'profile mismatch'
    ++$additionalCases
}
foreach ($invalidSuffix in @(' profile=9', ' profile=-1', ' profile=01', ' profile=oops', ' profile=0 profile=0')) {
    Assert-Rejected { ConvertFrom-SimpleStatus ($legacyStatus + $invalidSuffix) 7 } 'profile format'
    ++$additionalCases
}
foreach ($invalidEnvelope in @(' profile=4', ' profile=4 pulse_us=280000 limit=0.14',
 ' profile=4 pulse_us=320000 limit=0.1', ' profile=4 pulse_us=320000 pulse_us=320000 limit=0.14',
 ' profile=4 pulse_us=320000 limit=0.14 limit=0.14', ' profile=4 pulse_us=320000 limit=NaN',
 ' profile=4 pulse_us=360000 limit=0.14',
 ' profile=4 pulse_us=360000 limit=0.14 positive_us=320000 negative_us=360000',
 ' profile=4 pulse_us=360000 limit=0.14 positive_us=312000 negative_us=360000 positive_us=312000')) {
 Assert-Rejected { ConvertFrom-SimpleStatus ($legacyStatus + $invalidEnvelope) 7 4 } 'envelope'
 ++$additionalCases
}
Assert-Rejected { ConvertFrom-SimpleStatus $legacyStatus 7 1 } 'profile mismatch'
++ $additionalCases
# New envelopes require every protection echo and reject arbitrary or duplicated suffix fields.
foreach ($profileNumber in 5..7) {
    $validNewStatus = $legacyStatus + " profile=$profileNumber pulse_us=360000 limit=$(@(0.20,0.25,0.35)[$profileNumber-5]) positive_us=312000 negative_us=360000 torque_trip=$(@(0.23,0.28,0.40)[$profileNumber-5]) step=$(@(0.012,0.012,0.016)[$profileNumber-5]) slew=$(@(3,3,4)[$profileNumber-5])"
    foreach ($fieldName in @('torque_trip', 'step', 'slew')) {
        Assert-Rejected { ConvertFrom-SimpleStatus ($validNewStatus -replace " $fieldName=[^ ]+", '') 7 $profileNumber } 'envelope'
        Assert-Rejected { ConvertFrom-SimpleStatus ($validNewStatus -replace " $fieldName=[^ ]+", " $fieldName=1") 7 $profileNumber } 'envelope'
        Assert-Rejected { ConvertFrom-SimpleStatus ($validNewStatus + " $fieldName=1") 7 $profileNumber } 'format|envelope'
        $additionalCases += 3
    }
    Assert-Rejected { ConvertFrom-SimpleStatus ($validNewStatus + ' arbitrary=1') 7 $profileNumber } 'format'
    ++$additionalCases
}
# Firmware always emits bridge_fault; absent or malformed data must never default to healthy.
foreach ($profileNumber in @(0, 6, 7)) {
    $validBridgeStatus = $legacyStatus
    if ($profileNumber -eq 6) {
        $validBridgeStatus += ' profile=6 pulse_us=360000 limit=0.25 positive_us=312000 negative_us=360000 torque_trip=0.28 step=0.012 slew=3'
    }
    if ($profileNumber -eq 7) { $validBridgeStatus += ' profile=7 pulse_us=360000 limit=0.35 positive_us=312000 negative_us=360000 torque_trip=0.40 step=0.016 slew=4' }
    foreach ($invalidBridgeValue in @('', 'oops', '-1', '1.5', '256', '4294967296', '999999999999999999999999999999')) {
        $invalidBridgeStatus = if ($invalidBridgeValue -eq '') { $validBridgeStatus -replace ' bridge_fault=0', '' }
            else { $validBridgeStatus -replace 'bridge_fault=0', "bridge_fault=$invalidBridgeValue" }
        Assert-Rejected { ConvertFrom-SimpleStatus $invalidBridgeStatus 7 $profileNumber } 'bridge_fault|format'
        ++$additionalCases
    }
    Assert-Rejected { ConvertFrom-SimpleStatus ($validBridgeStatus + ' bridge_fault=0') 7 $profileNumber } 'format'
    Assert-Test ((ConvertFrom-SimpleStatus ($validBridgeStatus -replace 'bridge_fault=0', 'bridge_fault=255') 7 $profileNumber).BridgeFault -eq 255) 'bridge_fault uint8 upper boundary is retained as unhealthy'
    $additionalCases += 2
}
$script:transcriptLines = [System.Collections.Generic.List[string]]::new()
$script:templateToWire = @{}
$script:scenario = 'success'
$script:commands = [System.Collections.Generic.List[string]]::new()
$script:statusReads = 0
$script:wireRequestSequence = 4000
$script:seenWireIds = [System.Collections.Generic.HashSet[uint32]]::new()

function Invoke-WindowRequest {
    <# Emulates firmware responses; requests are logged and assigned unique wire IDs. #>
    param($Port, [uint32]$RequestId, [string]$Command, [int]$CaptureIndex = -1, [int]$TraceIndex = -1)
    ++$script:wireRequestSequence
    Assert-Test ($script:seenWireIds.Add([uint32]$script:wireRequestSequence)) 'unique wire ID'
    $script:templateToWire[[string]$RequestId] = [uint32]$script:wireRequestSequence
    $script:commands.Add($Command)
    switch ($Command) {
        'ping' {
            if ($script:scenario -eq 'boot' -and $script:statusReads -gt 0) { return "ok $RequestId pong boot=43" }
            return "ok $RequestId pong boot=42"
        }
        { $_ -in @('adrc simple', 'adrc simple profile=1', 'adrc simple profile=2', 'adrc simple profile=3', 'adrc simple profile=4', 'adrc simple profile=5', 'adrc simple profile=6', 'adrc simple profile=7', 'adrc simple profile=8') } {
            return "ok $RequestId adrc simple=accepted"
        }
        'adrc simple_status' {
            ++$script:statusReads
            if ($script:scenario -eq 'timeout') { throw 'Request timed out: simple_status' }
            $trialIdentity = $script:templateToWire['81000600']
            if ($script:scenario -eq 'replay') { --$trialIdentity }
            $trialResult = 1
            $safeRelease = 1
            $guardAck = 1
            $trialPhase = 9
            $bridgeFault = 0
            if ($script:scenario -in @('failed', 'failed_trace')) { $trialResult = 3 }
            if ($script:scenario -eq 'no_response') { $trialResult = 2 }
            if ($script:scenario -eq 'unsafe') { $safeRelease = 0 }
            if ($script:scenario -eq 'guard') { $guardAck = 0 }
            if ($script:scenario -eq 'bridge') { $bridgeFault = 2 }
            if ($script:scenario -eq 'locked') { $trialPhase = 10; $trialResult = 4 }
            if ($script:scenario -eq 'poll_timeout') { $trialPhase = 8; $trialResult = 0 }
            if ($script:scenario -eq 'changed' -and $script:statusReads -ge 3) { $guardAck = 0 }
            $profileFields = ''
            if ($script:fixtureProfile -gt 0) {
                $reportedProfile = $script:fixtureProfile
                if ($script:scenario -eq 'profile_mismatch' -or
                    ($script:scenario -eq 'profile_changed' -and $script:statusReads -ge 3)) { $reportedProfile = 0 }
                $profileFields = " profile=$reportedProfile pulse_us=$(@(80000,160000,200000,240000,360000,360000,360000,360000,640000)[$reportedProfile]) limit=$(@(0.03,0.05,0.08,0.10,0.14,0.20,0.25,0.35,1.75)[$reportedProfile])"
            }
            if ($script:fixtureProfile -ge 4 -and $script:fixtureProfile -ne 8) { $profileFields += " positive_us=312000 negative_us=360000" }
            if ($script:fixtureProfile -ge 5 -and $script:fixtureProfile -ne 8) { $profileFields += " torque_trip=$(@(0.23,0.28,0.40)[$script:fixtureProfile-5]) step=$(@(0.012,0.012,0.016)[$script:fixtureProfile-5]) slew=$(@(3,3,4)[$script:fixtureProfile-5])" }
            $trialB0 = 1.25
            if ($script:fixtureProfile -eq 8) {
                $trialB0 = 0
                if ($trialResult -eq 1) { $trialResult = 5 }
                $profileFields += ' positive_us=640000 negative_us=0 torque_trip=1.85000002 step=0.016 slew=4 probe=1 probe_peak=1.746032 probe_reached=1'
                if ($script:scenario -eq 'probe_underpeak') { $profileFields = $profileFields -replace 'probe_peak=1.746032 probe_reached=1', 'probe_peak=0.8 probe_reached=0' }
                if ($script:scenario -eq 'probe_result') { $trialResult = 1 }
                if ($script:scenario -eq 'probe_b0') { $trialB0 = 1.25 }
                if ($script:scenario -eq 'probe_count') { $trialPhase = 9 }
                if ($script:scenario -eq 'probe_fault') { $bridgeFault = 1 }
                if ($script:scenario -eq 'probe_pending') { $trialResult = 0 }
                if ($script:scenario -eq 'probe_peak_changed' -and $script:statusReads -ge 3) {
                    $profileFields = $profileFields -replace 'probe_peak=1.746032', 'probe_peak=1.74'
                }
            }
            $statusReply = "ok $RequestId adrc simple_status id=$trialIdentity phase=$trialPhase result=$trialResult fault=0 safe_release=$safeRelease guard_ack=$guardAck b0=$trialB0 count=400 bridge_fault=$bridgeFault guard_state=5$profileFields"
            if ($script:scenario -eq 'probe_count') { $statusReply = $statusReply -replace 'count=400', 'count=0' }
            if ($script:scenario -like 'bridge_changed_*' -and $script:statusReads -ge 3) {
                switch ($script:scenario) {
                    'bridge_changed_missing' { $statusReply = $statusReply -replace ' bridge_fault=0', '' }
                    'bridge_changed_invalid' { $statusReply = $statusReply -replace 'bridge_fault=0', 'bridge_fault=oops' }
                    'bridge_changed_overflow' { $statusReply = $statusReply -replace 'bridge_fault=0', 'bridge_fault=256' }
                    'bridge_changed_duplicate' { $statusReply += ' bridge_fault=0' }
                    'bridge_changed_fault' { $statusReply = $statusReply -replace 'bridge_fault=0', 'bridge_fault=1' }
                }
            }
            return $statusReply
        }
        'adrc simple_trace' {
            if ($script:scenario -in @('trace', 'failed_trace')) { throw 'fixture trace failure' }
            $trialIdentity = $script:templateToWire['81000600']
            if ($script:scenario -eq 'trace_replay') { --$trialIdentity }
            return "ok $RequestId adrc simple_trace id=$trialIdentity index=$TraceIndex t=1 rx=2 phase=3 p=0 v=0 raw_v=0 ref=0 requested=0 sent=0 torque_fb=0 enabled=0 z1=0 z2=0"
        }
        'adrc evidence' { return "ok $RequestId adrc evidence verified=0 pending=0" }
        default { throw "Unexpected mock command: $Command" }
    }
}

function Reset-Fixture {
    <# Resets fixture state without accessing the script main block or hardware. #>
    param([string]$Name, [int]$Profile = 0)
    $script:scenario = $Name
    $script:fixtureProfile = $Profile
    $script:commands.Clear()
    $script:statusReads = 0
    $script:templateToWire.Clear()
    $script:transcriptLines.Clear()
}

Reset-Fixture 'success'
Invoke-SimpleAdrcTrial $null '42'
Assert-Test (($script:commands | Where-Object { $_ -eq 'adrc simple_trace' }).Count -eq 400) 'collect all 400 trace rows'
Assert-Test ($script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'success evidence'
foreach ($testCase in @(
        @('failed', 'trial failed'), @('no_response', 'trial failed'), @('unsafe', 'trial failed'),
        @('guard', 'trial failed'), @('bridge', 'trial failed'), @('locked', 'trial failed'),
        @('changed', 'changed while collecting'), @('poll_timeout', 'trial timed out'),
        @('failed_trace', 'trial failed'), @('trace', 'fixture trace failure'),
        @('replay', 'identity'), @('trace_replay', 'trace mismatch'),
        @('timeout', 'timed out'), @('boot', 'Boot changed'))) {
    Reset-Fixture $testCase[0]
    Assert-Rejected { Invoke-SimpleAdrcTrial $null '42' } $testCase[1]
    Assert-Test (($script:commands | Where-Object { $_ -eq 'adrc simple' }).Count -eq 1) 'never replay start'
    Assert-Test (-not $script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'never false success'
    if ($testCase[0] -in @('failed', 'no_response', 'unsafe')) {
        Assert-Test (($script:commands | Where-Object { $_ -eq 'adrc simple_trace' }).Count -eq 400) 'failure still collects raw trace'
    }
    if ($testCase[0] -in @('timeout', 'poll_timeout')) {
        Assert-Test (-not $script:commands.Contains('adrc simple_trace')) 'no trace while active or unknown'
    }
}

# Every profile completes once; failed binding never replays or escalates a start.
foreach ($profileNumber in 1..7) {
    Reset-Fixture 'success' $profileNumber
    Invoke-SimpleAdrcTrial $null '42' $profileNumber
    $startCommands = @($script:commands | Where-Object { $_ -match '^adrc simple(?: |$)' })
    Assert-Test ($startCommands.Count -eq 1 -and $startCommands[0] -eq "adrc simple profile=$profileNumber") 'one exact profile start'
    Assert-Test (($script:commands | Where-Object { $_ -eq 'adrc simple_trace' }).Count -eq 400) 'profile collects complete trace'
    ++$additionalCases
    foreach ($failureScenario in @('profile_mismatch', 'profile_changed', 'failed')) {
        Reset-Fixture $failureScenario $profileNumber
        $expectedFailure = if ($failureScenario -eq 'failed') { 'trial failed' } else { 'profile mismatch' }
        Assert-Rejected { Invoke-SimpleAdrcTrial $null '42' $profileNumber } $expectedFailure
        $startCommands = @($script:commands | Where-Object { $_ -match '^adrc simple(?: |$)' })
        Assert-Test ($startCommands.Count -eq 1 -and $startCommands[0] -eq "adrc simple profile=$profileNumber") 'profile failure never replays or escalates'
        Assert-Test (-not $script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'profile failure never false success'
        ++$additionalCases
    }
}

# A complete profile 6 response that deteriorates after trace collection cannot report success.
foreach ($failureScenario in @('bridge_changed_missing', 'bridge_changed_invalid', 'bridge_changed_overflow',
        'bridge_changed_duplicate', 'bridge_changed_fault')) {
    Reset-Fixture $failureScenario 6
    $expectedFailure = if ($failureScenario -eq 'bridge_changed_fault') { 'changed while collecting' } else { 'bridge_fault|format' }
    Assert-Rejected { Invoke-SimpleAdrcTrial $null '42' 6 } $expectedFailure
    Assert-Test (-not $script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'invalid bridge status never false success'
    Assert-Test (($script:commands | Where-Object { $_ -eq 'adrc simple profile=6' }).Count -eq 1) 'bridge status failure never replays start'
    ++$additionalCases
}

# Fixed probe commands and preflight preserve the same mandatory stop/capture boundary.
Assert-Test ($allowedCommands.ValidValues -contains 'adrc simple profile=8') 'probe exact fixed command is allowlisted'
Assert-Rejected { & $preflightOnly -SimpleProfile 8 } 'requires TestSimpleAdrc'
& $preflightOnly -SimpleProfile 8 -TestSimpleAdrc -TestIndependentStop -SampleDisabledFeedback
$additionalCases += 2
# Missing, malformed or duplicated values cannot change the fixed probe envelope.
foreach ($fieldName in @('profile','pulse_us','limit','positive_us','negative_us','torque_trip','step','slew','probe','probe_peak','probe_reached','bridge_fault')) {
    Assert-Rejected { ConvertFrom-SimpleStatus ($probeStatus -replace " $fieldName=[^ ]+", '') 7 8 } 'format|envelope|probe|profile|bridge_fault'
    Assert-Rejected { ConvertFrom-SimpleStatus ($probeStatus -replace " $fieldName=[^ ]+", " $fieldName=oops") 7 8 } 'format|envelope|probe|profile|bridge_fault'
    Assert-Rejected { ConvertFrom-SimpleStatus ($probeStatus + " $fieldName=0") 7 8 } 'format'
    $additionalCases += 3
}
foreach ($invalidSuffix in @('probe=0','probe=01','probe_reached=0','probe_reached=01','probe_peak=-1',
        'probe_peak=1.75001','probe_peak=NaN','probe_peak=1e309','probe_peak=1.7324','b0=1','result=1',
        'phase=5','phase=7','positive_us=312000','negative_us=360000','torque_trip=1.86','step=0.012','slew=3','pulse_us=360000','limit=1.76')) {
    $fieldName = $invalidSuffix.Split('=')[0]
    Assert-Rejected { ConvertFrom-SimpleStatus ($probeStatus -replace " $fieldName=[^ ]+", " $invalidSuffix") 7 8 } 'probe|envelope'
    ++$additionalCases
}
foreach ($validPeak in @('0','0.8','1.7324','1.73249996','1.75')) {
    $expectedReached = [int]([double]$validPeak -ge [double][single]([single]0.99 * [single]1.75))
    $peakStatus = $probeStatus -replace 'probe_peak=[^ ]+', "probe_peak=$validPeak" -replace 'probe_reached=1', "probe_reached=$expectedReached"
    Assert-Test ((ConvertFrom-SimpleStatus $peakStatus 7 8).ProbeReached -eq $expectedReached) 'probe peak binds float32 reached boundary'
    ++$additionalCases
}
# Legacy firmware may omit metadata; if supplied all three must be zero and result 5 is forbidden.
foreach ($profileNumber in 0..7) {
    $legacyProfileStatus = $legacyStatus + " profile=$profileNumber pulse_us=$(@(80000,160000,200000,240000,360000,360000,360000,360000)[$profileNumber]) limit=$(@(0.03,0.05,0.08,0.10,0.14,0.20,0.25,0.35)[$profileNumber])"
    if ($profileNumber -ge 4) { $legacyProfileStatus += ' positive_us=312000 negative_us=360000' }
    if ($profileNumber -ge 5) { $legacyProfileStatus += " torque_trip=$(@(0.23,0.28,0.40)[$profileNumber-5]) step=$(@(0.012,0.012,0.016)[$profileNumber-5]) slew=$(@(3,3,4)[$profileNumber-5])" }
    Assert-Test ((ConvertFrom-SimpleStatus ($legacyProfileStatus + ' probe=0 probe_peak=0 probe_reached=0') 7 $profileNumber).Probe -eq 0) 'legacy zero metadata accepted'
    Assert-Rejected { ConvertFrom-SimpleStatus ($legacyProfileStatus -replace 'result=1', 'result=5') 7 $profileNumber } 'bounds'
    foreach ($legacyMetadata in @('probe=1 probe_peak=0 probe_reached=0','probe=0 probe_peak=0.1 probe_reached=0',
            'probe=0 probe_peak=0 probe_reached=1','probe=0','probe=0 probe_peak=0')) {
        Assert-Rejected { ConvertFrom-SimpleStatus ($legacyProfileStatus + " $legacyMetadata") 7 $profileNumber } 'probe'
        ++$additionalCases
    }
    $additionalCases += 2
}
# Completion reports finite probe only, including a complete pulse whose receipt peak never reached the cap.
foreach ($probeScenario in @('success','probe_underpeak')) {
    Reset-Fixture $probeScenario 8
    Invoke-SimpleAdrcTrial $null '42' 8
    Assert-Test ($script:commands.Contains('adrc simple profile=8')) 'probe has one explicit start'
    Assert-Test (@($script:commands | Where-Object { $_ -eq 'adrc simple profile=8' }).Count -eq 1) 'probe never retries'
    Assert-Test (@($script:commands | Where-Object { $_ -eq 'adrc simple_trace' }).Count -eq 400) 'probe collects complete trace'
    Assert-Test (@($script:transcriptLines | Where-Object { $_ -like 'startup_probe_completed=1 closed_loop=0 qualification=0*' }).Count -eq 1) 'probe-only completion'
    Assert-Test (-not $script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'probe never claims closed loop'
    if ($probeScenario -eq 'probe_underpeak') {
        Assert-Test (@($script:transcriptLines | Where-Object { $_ -like '*probe_peak=0.8 probe_reached=0' }).Count -eq 1) 'unreached peak reported honestly'
    }
    ++$additionalCases
}
foreach ($failureScenario in @('failed','unsafe','guard','bridge','locked','probe_result','probe_b0','probe_count',
        'probe_fault','probe_pending','probe_peak_changed','bridge_changed_missing','bridge_changed_invalid','bridge_changed_overflow',
        'bridge_changed_duplicate','bridge_changed_fault')) {
    Reset-Fixture $failureScenario 8
    Assert-Rejected { Invoke-SimpleAdrcTrial $null '42' 8 } 'failed|binding|format|changed'
    Assert-Test (@($script:commands | Where-Object { $_ -eq 'adrc simple profile=8' }).Count -eq 1) 'probe failure never retries'
    Assert-Test (@($script:transcriptLines | Where-Object { $_ -like 'startup_probe_completed=1*' }).Count -eq 0) 'probe failure never false completion'
    Assert-Test (-not $script:transcriptLines.Contains('simple_closed_loop_success=1 qualification=0 no_motion=0')) 'probe failure never closed loop'
    ++$additionalCases
}
# Inspect the real main/finally AST contract: no hardware execution is needed.
$mainTry = $sourceAst.EndBlock.Statements | Where-Object { $_ -is [System.Management.Automation.Language.TryStatementAst] } | Select-Object -Last 1
Assert-Test ($mainTry.Finally.Extent.Text -match "81000880 'stop'" -and
    $mainTry.Finally.Extent.Text -match "81000881 'adrc release'") 'STOP and protected release cleanup retained'
Assert-Test ($sourceText -match '\$TestSimpleAdrc -and \$TestMotorWatchdog') 'fault injection excluded'
Assert-Test ($sourceText -match '-not \(\$TestMotorWatchdog -or \$TestSimpleAdrc -or \$TestShaftPilot\)') 'motion scope label includes every motion switch'
Assert-Test ($sourceText -match '\+\+\$script:wireRequestSequence' -and
    $sourceText -match '\^\(ok\|error\) \$wireRequestId ') 'real transport matches unique wire ID'
Assert-Test ($sourceText -match "' simple_profile=' \+ \`$SimpleProfile") 'requested profile logged'
++ $additionalCases
Write-Output ('ADRC_SIMPLE_HOST_TESTS_PASSED cases=' + (21 + $additionalCases) + ' original_cases=21 baseline_cases=158 hardware_access=0')
