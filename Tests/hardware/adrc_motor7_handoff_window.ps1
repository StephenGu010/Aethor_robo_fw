<#
.SYNOPSIS
Verifies motor-7 ADRC handoff and optional explicitly bounded watchdog commissioning.
.DESCRIPTION
Requires a user-confirmed 24 V supply, explicit neutral-MIT challenge approval,
and the integrated LCD-MIT/ADRC firmware. The acquisition fallback requests
Kp=0, Kd=0, velocity=0, and feed-forward torque=0; protocol quantization may
make the encoded torque slightly nonzero. The default scope sends no ENABLE.
With TestMotorWatchdog, the reviewed firmware may issue one protected ENABLE
and a quantized zero-gain request; this optional scope is not a no-motion test.
The command allowlist contains discovery, disabled mode switching, ownership
handoff, status queries, and final DISABLE. Watchdog commissioning remains a
separate explicit switch with a fixed firmware-owned deadline and limits.
TestSimpleAdrc runs one firmware-bounded trial (profile 8 is only a startup probe) after
disabled capture and guard proof; it is a motion scope and grants no qualification.
TestShaftPilot runs one fixed-direction output-shaft identification pilot after
current capability reads and guard proof; it does not execute ADRC or install a model.
ShaftProfile low selects the firmware-owned 0.35 Nm finite local platform;
ShaftProfile fine requires VMAX=5 and selects the fixed lower 0.20 Nm platform.
ShaftProfile observe requires PMAX12.5/VMAX200/TMAX10 and its explicit fifth binding shard.
ShaftProfile platform requires the separate profile5 range-reserve shard and normal TIME completion.
ShaftProfile platform_wide requires PMAX512/VMAX200/TMAX10 and profile6's fixed reserve/entry shard.
ShaftProfile platform70 requires PMAX1024/VMAX200/TMAX10 and profile7's fixed 70-percent speed/reserve/entry shard.
the default profile retains its existing 1.75 Nm ceiling.
If ownership does not return to LCD, it records the locked state and requires
physical power cutoff instead of attempting a legacy mode command.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed,
    [switch]$NeutralMitChallengeApproved,
    [switch]$SampleDisabledFeedback,
    [switch]$ReadMotorCapabilities,
    [switch]$TestIndependentStop,
    [switch]$TestMotorWatchdog,
    [switch]$TestSimpleAdrc,
    [switch]$TestShaftPilot,
    [ValidateSet('positive','negative')][string]$ShaftDirection = 'positive',
    [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$ShaftProfile = 'default',
    [ValidateRange(0, 8)][int]$SimpleProfile = 0,
    [uint32]$RequestIdBase = 0
)

$ErrorActionPreference = 'Stop'
if ($TestShaftPilot -and (-not $SampleDisabledFeedback -or -not $ReadMotorCapabilities -or
    -not $TestIndependentStop -or $TestSimpleAdrc -or $TestMotorWatchdog)) {
    throw 'Shaft pilot requires current disabled capture, capabilities and independent stop; excludes other motion trials'
}
if (-not $TestShaftPilot -and $PSBoundParameters.ContainsKey('ShaftDirection')) {
    throw 'ShaftDirection requires TestShaftPilot'
}
if (-not $TestShaftPilot -and $PSBoundParameters.ContainsKey('ShaftProfile')) {
    throw 'ShaftProfile requires TestShaftPilot'
}
if ($ReadMotorCapabilities -and (-not $SampleDisabledFeedback -or $TestSimpleAdrc -or $TestMotorWatchdog)) {
    throw 'ReadMotorCapabilities requires disabled capture and excludes simple/watchdog motion switches'
}
if ($SimpleProfile -ne 0 -and -not $TestSimpleAdrc) {
    throw 'Nonzero SimpleProfile requires TestSimpleAdrc'
}
if ($TestSimpleAdrc -and (-not $TestIndependentStop -or -not $SampleDisabledFeedback)) {
    throw 'Simple ADRC requires same-session disabled capture and independent stop selftest'
}
if ($TestSimpleAdrc -and $TestMotorWatchdog) {
    throw 'Simple ADRC and watchdog fault injection are mutually exclusive'
}
if ($TestIndependentStop -and -not $SampleDisabledFeedback) {
    throw 'Independent stop selftest requires the same-session disabled capture'
}
if ($TestMotorWatchdog -and -not $TestIndependentStop) {
    throw 'Motor watchdog trial requires same-session independent stop selftest'
}
$transcriptLines = [System.Collections.Generic.List[string]]::new()
$script:shaftCapabilityReadback = $null
if ($TestShaftPilot) { . (Join-Path $PSScriptRoot 'adrc_shaft_pilot_host.ps1') }
$completedReplies = @{}
$wireToTemplate = @{}
$templateToWire = @{}
if ($RequestIdBase -eq 0) {
    $RequestIdBase = [uint32](2000000000L + ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() % 1000000000L))
}
$reservedRequestCount = if ($TestShaftPilot) { 5000L } else { 1000L }
if ([uint64]$RequestIdBase + $reservedRequestCount -gt [uint32]::MaxValue) {
    throw 'Request ID base leaves insufficient sequence space'
}
$wireRequestSequence = [uint64]$RequestIdBase
$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 40
$serialPort.WriteTimeout = 500
$serialPort.NewLine = "`n"
$discoveryComplete = $false
$mitModeEntered = $false
$ownerReturnedToLcd = $false
$acquireTransferred = $false
$releaseVerified = $false
$modeRestored = $false
$finalDisableCompleted = $false
$unsafeAcquire = $false
$transcriptLines.Add('host_start_utc=' + [datetime]::UtcNow.ToString('o') +
    ' port=' + $PortName + ' power24v_confirmed=' + [bool]$Power24vConfirmed +
    ' neutral_mit_challenge_approved=' + [bool]$NeutralMitChallengeApproved +
    ' motor_watchdog_trial=' + [bool]$TestMotorWatchdog +
    ' simple_adrc_trial=' + [bool]$TestSimpleAdrc +
    ' shaft_pilot_trial=' + [bool]$TestShaftPilot + ' shaft_direction=' + $ShaftDirection +
    ' shaft_profile=' + $ShaftProfile.ToLowerInvariant() +
    ' simple_profile=' + $SimpleProfile +
    ' read_motor_capabilities=' + [bool]$ReadMotorCapabilities +
    ' request_id_base=' + $RequestIdBase)

function Receive-WindowLine {
    <# Records one asynchronous line and caches terminal bench replies. #>
    param([System.IO.Ports.SerialPort]$Port)
    try { $replyLine = $Port.ReadLine().Trim() }
    catch [System.TimeoutException] { return $null }
    $script:transcriptLines.Add('< ' + $replyLine)
    # Cache only known actual wire IDs. Old replies must never match a newer
    # poll merely because both polls use the same local operation label.
    if ($replyLine -match '^done (\d+) bench (mode|disable) result=' -and
        $script:wireToTemplate.ContainsKey($Matches[1])) {
        $script:completedReplies[$Matches[1] + ':' + $Matches[2]] = $replyLine
    }
    return $replyLine
}

function Invoke-WindowRequest {
    <# Sends one allowlisted command and waits for its matching ACK. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('stop', 'ping', 'show state', 'show diag can', 'adrc status',
            'adrc gate motor=7', 'adrc discover motor=7',
            'adrc hardware', 'adrc limits', 'adrc feedback', 'adrc evidence',
            'adrc sample', 'adrc capture', 'adrc guard', 'adrc guard_status',
            'adrc capabilities', 'adrc capabilities_status', 'adrc capabilities_get',
            'adrc shaft_pilot direction=positive', 'adrc shaft_pilot direction=negative',
            'adrc shaft_pilot direction=positive profile=low', 'adrc shaft_pilot direction=negative profile=low',
            'adrc shaft_pilot direction=positive profile=visible', 'adrc shaft_pilot direction=negative profile=visible',
            'adrc shaft_pilot direction=positive profile=fine', 'adrc shaft_pilot direction=negative profile=fine',
            'adrc shaft_pilot direction=positive profile=observe', 'adrc shaft_pilot direction=negative profile=observe',
            'adrc shaft_pilot direction=positive profile=platform', 'adrc shaft_pilot direction=negative profile=platform',
            'adrc shaft_pilot direction=positive profile=platform_wide', 'adrc shaft_pilot direction=negative profile=platform_wide',
            'adrc shaft_pilot direction=positive profile=platform70', 'adrc shaft_pilot direction=negative profile=platform70',
            'adrc shaft_status', 'adrc shaft_binding', 'adrc shaft_trace',
            'adrc watchdog', 'adrc watchdog_status',
            'adrc simple', 'adrc simple_status', 'adrc simple_trace',
            'adrc simple profile=1', 'adrc simple profile=2', 'adrc simple profile=3', 'adrc simple profile=4', 'adrc simple profile=5', 'adrc simple profile=6', 'adrc simple profile=7', 'adrc simple profile=8',
            'adrc acquisition motor=7',
            'bench mode 7 mode=mit', 'adrc acquire motor=7', 'adrc release',
            'bench mode 7 mode=pos_vel', 'bench disable 7')]
        [string]$Command,
        [ValidateRange(-1, 63)][int]$CaptureIndex = -1,
        [ValidateRange(-1, 511)][int]$TraceIndex = -1,
        [ValidateRange(-1, 3)][int]$TracePart = -1
    )
    $wireCommand = $Command
    if ($TraceIndex -ge 0) {
        if ($Command -notin @('adrc simple_trace','adrc shaft_trace') -or $CaptureIndex -ge 0 -or
            ($Command -eq 'adrc simple_trace' -and ($TraceIndex -gt 399 -or $TracePart -ne -1)) -or
            ($Command -eq 'adrc shaft_trace' -and $TracePart -lt 0)) {
            throw 'Trace index validation failed: index or part does not match its fixed query'
        }
        $wireCommand += ' index=' + $TraceIndex
        if ($Command -eq 'adrc shaft_trace') { $wireCommand += ' part=' + $TracePart }
    }
    elseif ($Command -in @('adrc simple_trace','adrc shaft_trace') -or $TracePart -ne -1) {
        throw 'Trace query requires an index and its bounded part'
    }
    if ($CaptureIndex -ge 0) {
        if ($Command -notin @('adrc capture','adrc capabilities_get','adrc shaft_binding')) { throw 'Sample index requires a fixed query' }
        if ($Command -eq 'adrc capabilities_get' -and $CaptureIndex -gt 5) { throw 'Capability index outside 0..5' }
        $lastBindingIndex = if ($ShaftProfile -in @('observe','platform','platform_wide','platform70')) { 4 } else { 3 }
        if ($Command -eq 'adrc shaft_binding' -and $CaptureIndex -gt $lastBindingIndex) { throw 'Shaft binding index outside selected fixed profile' }
        $wireCommand += ' index=' + $CaptureIndex
    }
    elseif ($Command -in @('adrc capabilities_get','adrc shaft_binding')) { throw 'Indexed query requires an index' }
    if ($script:wireRequestSequence -ge [uint32]::MaxValue) { throw 'Request ID space exhausted' }
    ++$script:wireRequestSequence
    $wireRequestId = [uint32]$script:wireRequestSequence
    $script:wireToTemplate[[string]$wireRequestId] = $RequestId
    $script:templateToWire[[string]$RequestId] = $wireRequestId
    $requestText = "$wireRequestId $wireCommand"
    $script:transcriptLines.Add('> ' + $requestText)
    $Port.Write($requestText + "`r`n")
    $deadlineUtc = [datetime]::UtcNow.AddMilliseconds(1500)
    while ([datetime]::UtcNow -lt $deadlineUtc) {
        $replyLine = Receive-WindowLine $Port
        if ($null -ne $replyLine -and $replyLine -match "^(ok|error) $wireRequestId ") {
            # Normalize only the already matched ACK for existing local checks.
            $replyPrefix = $Matches[1]
            return $replyPrefix + ' ' + $RequestId +
                $replyLine.Substring(($replyPrefix + ' ' + $wireRequestId).Length)
        }
    }
    throw "Request timed out: $requestText"
}

function Wait-WindowCompletion {
    <# Requires a completed mode or disable result before the next phase. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('mode', 'disable')][string]$Operation
    )
    if (-not $script:templateToWire.ContainsKey([string]$RequestId)) { throw 'No wire request for completion' }
    $wireRequestId = $script:templateToWire[[string]$RequestId]
    $completionKey = "$wireRequestId`:$Operation"
    $deadlineUtc = [datetime]::UtcNow.AddSeconds(4)
    while ([datetime]::UtcNow -lt $deadlineUtc) {
        if ($script:completedReplies.ContainsKey($completionKey)) {
            $completionReply = $script:completedReplies[$completionKey]
            if ($completionReply -match "^done $wireRequestId bench $Operation result=completed\b") {
                return $completionReply
            }
            throw "Bench operation failed: $completionReply"
        }
        [void](Receive-WindowLine $Port)
    }
    throw "Bench completion timed out: $RequestId bench $Operation"
}

function Assert-WindowDisabledLcd {
    <# Rejects motion, fault, bus-off, or non-LCD ownership at a phase boundary. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$RequestId)
    $stateReply = Invoke-WindowRequest $Port $RequestId 'show state'
    $ownerReply = Invoke-WindowRequest $Port ([uint32]($RequestId + 1)) 'adrc status'
    $canReply = Invoke-WindowRequest $Port ([uint32]($RequestId + 2)) 'show diag can'
    if ($stateReply -notmatch 'enabled=00 moving=0 .*fault=none' -or
        $ownerReply -notmatch 'owner=lcd\b' -or
        $canReply -notmatch 'busoff=0') {
        throw 'Expected disabled, LCD-owned, fault-free, non-bus-off state'
    }
    $script:ownerReturnedToLcd = $true
}

function ConvertFrom-MotorCapabilityReply {
    <# Strictly validates unique mandatory fields, unsigned bounds, raw float and observation provenance. #>
    param([string]$Reply, [ValidateSet('capabilities_status','capabilities_get')][string]$Kind,
        [uint32]$ExpectedId, [int]$ExpectedIndex = -1)
    if ($Reply -notmatch ('^ok [0-9]+ adrc ' + $Kind + ' (.+)$')) { throw 'Capability reply format mismatch' }
    $payload = $Matches[1]
    $requiredFields = if ($Kind -eq 'capabilities_status') {
        @('id','epoch','current','motor','state','prepare','reason','count','rejected','qualified','model_known','actual_send_us')
    } else {
        @('id','epoch','current','index','rid','name','valid','raw_bits','value','positive',
            'request_created_us','queue_submitted_us','pre_submit_boundary_us','send_completed_observed_us',
            'rx_lower_us','received_observed_us','actual_send_us','model_known','qualified')
    }
    $fields = @{}
    foreach ($token in $payload.Split(' ')) {
        if ($token -notmatch '^([a-z_]+)=([^= ]+)$' -or $fields.ContainsKey($Matches[1]) -or
            $requiredFields -cnotcontains $Matches[1]) { throw 'Capability unknown or duplicate field' }
        $fields[$Matches[1]] = $Matches[2]
    }
    foreach ($field in $requiredFields) {
        if (-not $fields.ContainsKey($field)) { throw "Capability missing mandatory field: $field" }
        if ($field -eq 'name' -or $field -eq 'value') { continue }
        $unsignedValue = [uint64]0
        if ($fields[$field] -notmatch '^[0-9]+$' -or -not [uint64]::TryParse($fields[$field],
            [Globalization.NumberStyles]::None, [Globalization.CultureInfo]::InvariantCulture, [ref]$unsignedValue)) {
            throw "Capability uint64 format or bounds mismatch: $field"
        }
        $maximumValue = if ($field.EndsWith('_us')) { [uint64]::MaxValue } else { [uint64][uint32]::MaxValue }
        if ($unsignedValue -gt $maximumValue) { throw "Capability uint32 bounds mismatch: $field" }
        $fields[$field] = $unsignedValue
    }
    if ($fields.id -ne $ExpectedId -or $fields.epoch -eq 0 -or $fields.current -ne 1 -or
        $fields.qualified -ne 0 -or $fields.model_known -ne 0 -or $fields.actual_send_us -ne 0) {
        throw 'Capability identity, freshness or diagnostic provenance mismatch'
    }
    if ($Kind -eq 'capabilities_status') {
        if ($fields.motor -ne 7 -or $fields.state -gt 3 -or $fields.prepare -gt 3 -or $fields.reason -gt 5 -or $fields.count -gt 6 -or
            ($fields.state -eq 2 -and $fields.prepare -ne 2)) {
            throw 'Capability status bounds mismatch'
        }
        return $fields
    }
    $registers = @(1,2,3,11,12,20); $names = @('KT','OT','OC','Damp','Inertia','Gr')
    if ($ExpectedIndex -lt 0 -or $ExpectedIndex -gt 5 -or $fields.index -ne $ExpectedIndex -or
        $fields.rid -ne $registers[$ExpectedIndex] -or $fields.name -cne $names[$ExpectedIndex] -or
        $fields.valid -ne 1 -or $fields.positive -gt 1) { throw 'Capability fixed register identity mismatch' }
    $value = [double]0
    if (-not [double]::TryParse($fields.value, [Globalization.NumberStyles]::Float,
        [Globalization.CultureInfo]::InvariantCulture, [ref]$value) -or
        [double]::IsNaN($value) -or [double]::IsInfinity($value) -or [Math]::Abs($value) -gt [single]::MaxValue) {
        throw 'Capability float is not finite'
    }
    $rawValue = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$fields.raw_bits), 0)
    if ([single]::IsNaN($rawValue) -or [single]::IsInfinity($rawValue) -or [single]$value -ne $rawValue -or
        $fields.positive -ne [uint64][int]($value -gt 0) -or
        ($ExpectedIndex -eq 0 -and $value -lt 0) -or ($ExpectedIndex -eq 1 -and ($value -lt 80 -or $value -ge 200)) -or
        ($ExpectedIndex -eq 2 -and ($value -le 0 -or $value -ge 1)) -or ($ExpectedIndex -eq 5 -and $value -le 0)) {
        throw 'Capability raw float or vendor range mismatch'
    }
    if ($fields.request_created_us -eq 0 -or $fields.pre_submit_boundary_us -le $fields.request_created_us -or
        $fields.queue_submitted_us -lt $fields.pre_submit_boundary_us -or
        $fields.send_completed_observed_us -lt $fields.queue_submitted_us -or
        $fields.rx_lower_us -lt $fields.pre_submit_boundary_us -or $fields.received_observed_us -lt $fields.rx_lower_us -or
        $fields.send_completed_observed_us - $fields.request_created_us -gt 20000 -or
        $fields.received_observed_us - $fields.request_created_us -gt 20000) {
        throw 'Capability observation provenance mismatch'
    }
    $fields.value = $value
    return $fields
}

function Invoke-MotorCapabilityRead {
    <# Runs only the fixed six reads after complete disabled capture; failure falls through existing STOP cleanup. #>
    param([System.IO.Ports.SerialPort]$Port)
    $startReply = Invoke-WindowRequest $Port 81000310 'adrc capabilities'
    $collectionId = [uint32]$script:templateToWire['81000310']
    if ($startReply -cne "ok 81000310 adrc capabilities id=$collectionId state=1 count=6 qualified=0") {
        throw "Capability start rejected: $startReply"
    }
    $deadline = [datetime]::UtcNow.AddSeconds(2)
    $complete = $false
    while ([datetime]::UtcNow -lt $deadline) {
        $status = ConvertFrom-MotorCapabilityReply (Invoke-WindowRequest $Port 81000311 'adrc capabilities_status') 'capabilities_status' $collectionId
        if ($status.state -eq 3) { throw 'Capability collection aborted; no motion permitted' }
        if ($status.state -eq 2) {
            if ($status.prepare -ne 2 -or $status.reason -ne 0 -or $status.count -ne 6) { throw 'Capability invalid completion' }
            $complete = $true; break
        }
        Start-Sleep -Milliseconds 40
    }
    if (-not $complete) { throw 'Capability collection did not finish' }
    $collectionEpoch = $status.epoch
    $rows = @()
    for ($index = 0; $index -lt 6; ++$index) {
        $row = ConvertFrom-MotorCapabilityReply (Invoke-WindowRequest $Port 81000312 'adrc capabilities_get' $index) 'capabilities_get' $collectionId $index
        if ($row.epoch -ne $collectionEpoch) { throw 'Capability epoch changed while collecting results' }
        $rows += $row
    }
    $finalStatus = ConvertFrom-MotorCapabilityReply (Invoke-WindowRequest $Port 81000313 'adrc capabilities_status') 'capabilities_status' $collectionId
    if ($finalStatus.state -ne 2 -or $finalStatus.prepare -ne 2 -or $finalStatus.count -ne 6 -or $finalStatus.reason -ne 0 -or
        $finalStatus.epoch -ne $collectionEpoch) { throw 'Capability evidence changed while collecting results' }
    $script:transcriptLines.Add('capabilities_data=' + ($rows | ConvertTo-Json -Depth 4 -Compress))
    $script:shaftCapabilityReadback = @{ id=$collectionId; epoch=$collectionEpoch; rows=$rows }
}

function ConvertFrom-SimpleStatus {
    <# Parses the complete status and rejects stale trial identity or invalid bounds. #>
    param([string]$Reply, [uint32]$TrialId,
        [ValidateRange(0, 8)][int]$RequestedProfile = 0)
    $numberPattern = '-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
    if ($Reply -notmatch "^ok 81000601 adrc simple_status id=$TrialId phase=(\d+) result=(\d+) fault=(\d+) safe_release=([01]) guard_ack=([01]) b0=($numberPattern) count=(\d+)(?: [a-z_]+=[^ ]+)*$") {
        throw "Simple ADRC status identity or format mismatch: $Reply"
    }
    $status = [pscustomobject]@{
        Phase = [int]$Matches[1]; Result = [int]$Matches[2]; Fault = [uint32]$Matches[3]
        SafeRelease = [int]$Matches[4]; GuardAck = [int]$Matches[5]
        B0 = [double]::Parse($Matches[6], [cultureinfo]::InvariantCulture); Count = [int]$Matches[7]
        BridgeFault = 0
        Profile = 0
        Probe = 0; ProbePeak = 0.0; ProbeReached = 0
    }
    # Restrict optional status fields and reject duplicates, including mandatory prefix fields.
    $seenFields = @{}
    $allowedFields = @('id','phase','result','fault','safe_release','guard_ack','b0','count',
        'bridge_fault','guard_state','profile','pulse_us','limit','positive_us','negative_us','torque_trip','step','slew','probe','probe_peak','probe_reached')
    foreach ($field in [regex]::Matches($Reply, '(?:^| )([a-z_]+)=([^ ]+)')) {
        $fieldName = $field.Groups[1].Value
        if ($fieldName -notin $allowedFields -or $seenFields.ContainsKey($fieldName)) {
            throw 'Simple ADRC profile format envelope mismatch: unknown or duplicate status field'
        }
        $seenFields[$fieldName] = $field.Groups[2].Value
    }
    $profileFields = [regex]::Matches($Reply, '(?:^| )profile=([^ ]+)')
    if ($profileFields.Count -gt 1 -or
        ($profileFields.Count -eq 1 -and $profileFields[0].Groups[1].Value -notmatch '^[0-8]$')) {
        throw 'Simple ADRC profile format mismatch'
    }
    if ($profileFields.Count -eq 1) { $status.Profile = [int]$profileFields[0].Groups[1].Value }
    if ($status.Profile -ne $RequestedProfile) { throw 'Simple ADRC profile mismatch' }
    # Bind reported duration and limit to the fixed reviewed envelope; only legacy profile 0 may omit both.
    $pulseFields = [regex]::Matches($Reply, '(?:^| )pulse_us=([^ ]+)')
    $limitFields = [regex]::Matches($Reply, '(?:^| )limit=([^ ]+)')
    if ($RequestedProfile -ne 0 -or $pulseFields.Count -ne 0 -or $limitFields.Count -ne 0) {
        if ($pulseFields.Count -ne 1 -or $limitFields.Count -ne 1 -or
            $pulseFields[0].Groups[1].Value -notmatch '^\d+$' -or
            $limitFields[0].Groups[1].Value -notmatch "^$numberPattern`$") {
            throw 'Simple ADRC envelope format mismatch'
        }
        $pulseDuration = [double]::Parse($pulseFields[0].Groups[1].Value, [cultureinfo]::InvariantCulture)
        $torqueLimit = [double]::Parse($limitFields[0].Groups[1].Value, [cultureinfo]::InvariantCulture)
        if ($pulseDuration -ne @(80000,160000,200000,240000,360000,360000,360000,360000,640000)[$RequestedProfile] -or
            [double]::IsInfinity($torqueLimit) -or [double]::IsNaN($torqueLimit) -or
            [Math]::Abs($torqueLimit - @(0.03,0.05,0.08,0.10,0.14,0.20,0.25,0.35,1.75)[$RequestedProfile]) -gt 0.00000001) {
            throw 'Simple ADRC envelope mismatch'
        }
    }
    # Asymmetric startup pulses must be explicitly echoed; max duration alone is insufficient.
    if ($RequestedProfile -ge 4) {
        foreach ($pulseField in @('positive_us', 'negative_us')) {
            $durationFields = [regex]::Matches($Reply, "(?:^| )$pulseField=([^ ]+)")
            $expectedDuration = if ($RequestedProfile -eq 8) { if ($pulseField -eq 'positive_us') { '640000' } else { '0' } }
                elseif ($pulseField -eq 'positive_us') { '312000' } else { '360000' }
            if ($durationFields.Count -ne 1 -or $durationFields[0].Groups[1].Value -cne $expectedDuration) {
                throw 'Simple ADRC envelope asymmetric duration mismatch'
            }
        }
    }
    # Running thresholds never change the separate 0.15 Nm start/disabled health gate.
    foreach ($protectionName in @('torque_trip', 'step', 'slew')) {
        if ($RequestedProfile -ge 5 -or $seenFields.ContainsKey($protectionName)) {
            $expectedProtection = switch ($protectionName) {
                'torque_trip' { @(0.15,0.15,0.15,0.15,0.15,0.23,0.28,0.40,1.85)[$RequestedProfile] }
                'step' { if ($RequestedProfile -ge 7) { 0.016 } elseif ($RequestedProfile -ge 5) { 0.012 } else { 0.008 } }
                'slew' { if ($RequestedProfile -ge 7) { 4.0 } elseif ($RequestedProfile -ge 5) { 3.0 } else { 2.0 } }
            }
            if (-not $seenFields.ContainsKey($protectionName) -or $seenFields[$protectionName] -notmatch "^$numberPattern`$") {
                throw 'Simple ADRC envelope protection format mismatch'
            }
            $actualProtection = [double]::Parse($seenFields[$protectionName], [cultureinfo]::InvariantCulture)
            if ([double]::IsInfinity($actualProtection) -or [double]::IsNaN($actualProtection) -or
                [Math]::Abs($actualProtection - $expectedProtection) -gt $(if ($RequestedProfile -eq 8) { 0.0000001 } else { 0.00000001 })) {
                throw 'Simple ADRC envelope protection mismatch'
            }
        }
    }
    # Profile 8 observes receipt peak only; peak/reached never grant identification or closed loop success.
    $hasProbeFields = $seenFields.ContainsKey('probe') -or $seenFields.ContainsKey('probe_peak') -or $seenFields.ContainsKey('probe_reached')
    if ($RequestedProfile -eq 8 -or $hasProbeFields) {
        if (-not $seenFields.ContainsKey('probe') -or -not $seenFields.ContainsKey('probe_peak') -or
            -not $seenFields.ContainsKey('probe_reached') -or $seenFields['probe'] -notmatch '^[01]$' -or
            $seenFields['probe_reached'] -notmatch '^[01]$' -or $seenFields['probe_peak'] -notmatch "^$numberPattern`$") {
            throw 'Simple ADRC probe format mismatch'
        }
        $status.Probe = [int]$seenFields['probe']
        # .NET Framework rejects overflow with TryParse=false; newer .NET may return Infinity.
        [double]$parsedProbePeak = 0.0
        if (-not [double]::TryParse($seenFields['probe_peak'], [Globalization.NumberStyles]::Float,
                [cultureinfo]::InvariantCulture, [ref]$parsedProbePeak)) {
            throw 'Simple ADRC probe peak format mismatch'
        }
        $status.ProbePeak = $parsedProbePeak
        $status.ProbeReached = [int]$seenFields['probe_reached']
        if ([double]::IsInfinity($status.ProbePeak) -or [double]::IsNaN($status.ProbePeak) -or
            $status.ProbePeak -lt 0 -or $status.ProbePeak -gt 1.75) { throw 'Simple ADRC probe peak bounds mismatch' }
        if ($RequestedProfile -eq 8) {
            # Firmware compares float32 peak against float32 (0.99f * 1.75f).
            $receiptThreshold = [double][single]([single]0.99 * [single]1.75)
            if ($status.Probe -ne 1 -or $status.B0 -ne 0 -or $status.Result -eq 1 -or
                $status.Phase -in @(4,5,6,7) -or
                $status.ProbeReached -ne [int]($status.ProbePeak -ge $receiptThreshold)) {
                throw 'Simple ADRC probe binding mismatch'
            }
        } elseif ($status.Probe -ne 0 -or $status.ProbePeak -ne 0 -or $status.ProbeReached -ne 0) {
            throw 'Simple ADRC legacy probe binding mismatch'
        }
    }
    # Every supported firmware status emits this uint8 fault; missing data is not healthy evidence.
    [byte]$parsedBridgeFault = 0
    if (-not $seenFields.ContainsKey('bridge_fault') -or
        $seenFields['bridge_fault'] -notmatch '^[0-9]+$' -or
        -not [byte]::TryParse($seenFields['bridge_fault'], [ref]$parsedBridgeFault)) {
        throw 'Simple ADRC bridge_fault format or uint8 bounds mismatch'
    }
    $status.BridgeFault = [int]$parsedBridgeFault
    if ($status.Phase -gt 10 -or $status.Result -gt $(if ($RequestedProfile -eq 8) { 5 } else { 4 }) -or $status.Count -gt 400 -or
        [double]::IsInfinity($status.B0) -or [double]::IsNaN($status.B0)) {
        throw 'Simple ADRC status outside protocol bounds'
    }
    return $status
}

function Invoke-SimpleAdrcTrial {
    <# Runs exactly once, preserves the primary failure, and collects available raw evidence. #>
    param([System.IO.Ports.SerialPort]$Port, [string]$BootId,
        [ValidateRange(0, 8)][int]$SimpleProfile = 0)
    $beforeBoot = Invoke-WindowRequest $Port 81000609 'ping'
    if ($beforeBoot -notmatch " boot=$BootId$") { throw 'Boot changed before simple ADRC trial' }
    $requiredResult = if ($SimpleProfile -eq 8) { 5 } else { 1 }
    $primaryFailure = $null
    $lastStatus = $null
    $trialId = $null
    try {
        # Select exactly one reviewed command; this function never escalates profiles.
        $simpleCommand = @('adrc simple', 'adrc simple profile=1',
            'adrc simple profile=2', 'adrc simple profile=3', 'adrc simple profile=4', 'adrc simple profile=5', 'adrc simple profile=6', 'adrc simple profile=7', 'adrc simple profile=8')[$SimpleProfile]
        $startReply = Invoke-WindowRequest $Port 81000600 $simpleCommand
        $trialId = $script:templateToWire['81000600']
        if ($startReply -notmatch '^ok 81000600 adrc simple=accepted$') {
            throw "Simple ADRC start rejected: $startReply"
        }
        $deadline = [datetime]::UtcNow.AddSeconds(4)
        do {
            $statusReply = Invoke-WindowRequest $Port 81000601 'adrc simple_status'
            $lastStatus = ConvertFrom-SimpleStatus $statusReply $trialId $SimpleProfile
            if ($lastStatus.Phase -in @(9, 10)) { break }
            Start-Sleep -Milliseconds 20
        } while ([datetime]::UtcNow -lt $deadline)
        if ($lastStatus.Phase -notin @(9, 10)) { throw 'Simple ADRC trial timed out' }
        if ($lastStatus.Phase -ne 9 -or $lastStatus.Result -ne $requiredResult -or
            $lastStatus.Fault -ne 0 -or $lastStatus.BridgeFault -ne 0 -or
            $lastStatus.SafeRelease -ne 1 -or $lastStatus.GuardAck -ne 1 -or $lastStatus.Count -eq 0) {
            throw "Simple ADRC trial failed: $statusReply"
        }
    }
    catch { $primaryFailure = $_ }
    # An ACK timeout still leaves a known wire trial ID. Never replay the start.
    if ($null -eq $trialId -and $script:templateToWire.ContainsKey('81000600')) {
        $trialId = $script:templateToWire['81000600']
    }
    try {
        if ($null -eq $lastStatus -or $lastStatus.Phase -notin @(9, 10)) {
            throw 'Simple ADRC terminal stop not observed; proceeding to STOP cleanup'
        }
        $afterBoot = Invoke-WindowRequest $Port 81000609 'ping'
        if ($afterBoot -notmatch " boot=$BootId$") { throw 'Boot changed during simple ADRC trial' }
        if ($null -eq $trialId) { throw 'Simple ADRC trial identity unavailable' }
        $traceStatus = ConvertFrom-SimpleStatus (Invoke-WindowRequest $Port 81000601 'adrc simple_status') $trialId $SimpleProfile
        if ($traceStatus.Phase -notin @(9, 10)) { throw 'Simple ADRC not terminal; trace deferred for immediate STOP cleanup' }
        if ($traceStatus.Phase -ne $lastStatus.Phase -or $traceStatus.Result -ne $lastStatus.Result -or
            $traceStatus.Count -ne $lastStatus.Count -or $traceStatus.SafeRelease -ne $lastStatus.SafeRelease -or
            $traceStatus.GuardAck -ne $lastStatus.GuardAck -or $traceStatus.Fault -ne $lastStatus.Fault -or
            $traceStatus.BridgeFault -ne $lastStatus.BridgeFault -or $traceStatus.B0 -ne $lastStatus.B0 -or
            $traceStatus.Probe -ne $lastStatus.Probe -or $traceStatus.ProbePeak -ne $lastStatus.ProbePeak -or
            $traceStatus.ProbeReached -ne $lastStatus.ProbeReached) {
            if ($null -eq $primaryFailure) { $primaryFailure = 'Simple ADRC terminal evidence changed before trace collection' }
        }
        $numberPattern = '-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
        $traceDeadline = [datetime]::UtcNow.AddSeconds(30)
        for ($traceIndex = 0; $traceIndex -lt $traceStatus.Count; ++$traceIndex) {
            if ([datetime]::UtcNow -ge $traceDeadline) { throw 'Simple ADRC trace collection timed out' }
            $traceRow = Invoke-WindowRequest $Port 81000602 'adrc simple_trace' -TraceIndex $traceIndex
            if ($traceRow -notmatch "^ok 81000602 adrc simple_trace id=$trialId index=$traceIndex t=\d+ rx=\d+ phase=\d+ p=$numberPattern v=$numberPattern raw_v=$numberPattern ref=$numberPattern requested=$numberPattern sent=$numberPattern torque_fb=$numberPattern enabled=[01] z1=$numberPattern z2=$numberPattern$") {
                throw "Simple ADRC trace mismatch: $traceRow"
            }
        }
        $finalStatus = ConvertFrom-SimpleStatus (Invoke-WindowRequest $Port 81000601 'adrc simple_status') $trialId $SimpleProfile
        if ($finalStatus.Count -ne $traceStatus.Count -or $finalStatus.Phase -ne $traceStatus.Phase -or
            $finalStatus.Result -ne $traceStatus.Result -or $finalStatus.SafeRelease -ne $traceStatus.SafeRelease -or
            $finalStatus.GuardAck -ne $traceStatus.GuardAck -or $finalStatus.Fault -ne $traceStatus.Fault -or
            $finalStatus.BridgeFault -ne $traceStatus.BridgeFault -or $finalStatus.B0 -ne $traceStatus.B0 -or
            $finalStatus.Probe -ne $traceStatus.Probe -or $finalStatus.ProbePeak -ne $traceStatus.ProbePeak -or
            $finalStatus.ProbeReached -ne $traceStatus.ProbeReached) {
            throw 'Simple ADRC trace changed while collecting'
        }
        $afterTraceBoot = Invoke-WindowRequest $Port 81000609 'ping'
        if ($afterTraceBoot -notmatch " boot=$BootId$") { throw 'Boot changed during simple ADRC trace collection' }
        $evidenceReply = Invoke-WindowRequest $Port 81000603 'adrc evidence'
        if ($evidenceReply -notmatch ' verified=0 ') { throw 'Simple ADRC unexpectedly changed qualification' }
        $script:transcriptLines.Add('simple_trace_rows=' + $traceStatus.Count + ' qualification=0')
    }
    catch {
        $script:transcriptLines.Add('simple_trace_error=' + $_.Exception.Message)
        if ($null -eq $primaryFailure) { $primaryFailure = $_ }
    }
    if ($null -ne $primaryFailure) { throw $primaryFailure }
    if ($SimpleProfile -eq 8) {
        $script:transcriptLines.Add('startup_probe_completed=1 closed_loop=0 qualification=0 no_motion=0 probe_peak=' +
            $finalStatus.ProbePeak.ToString('G9', [cultureinfo]::InvariantCulture) + ' probe_reached=' + $finalStatus.ProbeReached)
    } else { $script:transcriptLines.Add('simple_closed_loop_success=1 qualification=0 no_motion=0') }
}

try {
    if (-not $Power24vConfirmed) { throw 'User confirmation of 24 V ON is required' }
    if (-not $NeutralMitChallengeApproved) {
        throw 'Approval of the quantized zero-request MIT challenge is required'
    }
    if ($PortName -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
        throw "Serial port unavailable: $PortName"
    }
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    $bootReply = Invoke-WindowRequest $serialPort 81000000 'ping'
    if ($bootReply -notmatch ' boot=(\d+)$' -or [uint32]$Matches[1] -eq 0) {
        throw 'Current boot identity was not observed'
    }
    $sessionBootId = $Matches[1]
    Assert-WindowDisabledLcd $serialPort 81000001
    $gateReply = Invoke-WindowRequest $serialPort 81000004 'adrc gate motor=7'
    if ($gateReply -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateReply -notmatch 'discovery_active=0') {
        throw 'Discovery preflight stopped: LCD, CAN, or discovery is busy'
    }

    $discoverReply = Invoke-WindowRequest $serialPort 81000005 'adrc discover motor=7'
    if ($discoverReply -notmatch '^ok 81000005 adrc discover=accepted') {
        throw "Discovery rejected: $discoverReply"
    }
    for ($pollIndex = 0; $pollIndex -lt 25; ++$pollIndex) {
        Start-Sleep -Milliseconds 40
        $gateReply = Invoke-WindowRequest $serialPort ([uint32](81000006 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $discoveryComplete = $true
            break
        }
    }
    if (-not $discoveryComplete -or
        $gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Original mode 2 or complete motor-7 discovery was not verified'
    }
    Assert-WindowDisabledLcd $serialPort 81000040

    $modeReply = Invoke-WindowRequest $serialPort 81000050 'bench mode 7 mode=mit'
    if ($modeReply -notmatch '^ok 81000050 bench mode accepted=1') {
        throw "MIT mode switch rejected: $modeReply"
    }
    $mitModeEntered = $true
    [void](Wait-WindowCompletion $serialPort 81000050 'mode')
    $mitIdleVerified = $false
    for ($pollIndex = 0; $pollIndex -lt 25; ++$pollIndex) {
        $gateReply = Invoke-WindowRequest $serialPort ([uint32](81000051 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'mode=1 fields=1fff verified=40' -and
            $gateReply -match 'lcd_idle=1 can_idle=1') {
            $mitIdleVerified = $true
            break
        }
        Start-Sleep -Milliseconds 10
    }
    if (-not $mitIdleVerified) { throw 'MIT mode readback or idle gate failed' }
    Assert-WindowDisabledLcd $serialPort 81000080

    $acquireReply = Invoke-WindowRequest $serialPort 81000100 'adrc acquire motor=7'
    if ($acquireReply -notmatch '^ok 81000100 adrc acquire=accepted') {
        throw "ADRC acquire rejected: $acquireReply"
    }
    $ownerReturnedToLcd = $false
    for ($pollIndex = 0; $pollIndex -lt 20; ++$pollIndex) {
        $ownerReply = Invoke-WindowRequest $serialPort ([uint32](81000101 + $pollIndex)) 'adrc status'
        if ($ownerReply -match 'owner=adrc handoff=transferred') {
            $acquireTransferred = $true
            break
        }
        if ($ownerReply -match 'owner=lcd handoff=(unsafe|timeout)') {
            $unsafeAcquire = $ownerReply -match 'handoff=unsafe'
            $ownerReturnedToLcd = $true
            throw "ADRC acquire did not transfer: $ownerReply"
        }
        Start-Sleep -Milliseconds 5
    }
    if (-not $acquireTransferred) { throw 'ADRC acquire state was not confirmed' }

    # Finish selected-axis register discovery before release. Ordinary feedback
    # queries continue under ADRC ownership; release itself stops those queries
    # and enforces the measured CAN quiet gate in firmware.
    $selectedDiscoveryIdle = $false
    for ($pollIndex = 0; $pollIndex -lt 30; ++$pollIndex) {
        $gateReply = Invoke-WindowRequest $serialPort 81000121 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0' -and
            $gateReply -match 'mode=1 fields=1fff verified=40') {
            $selectedDiscoveryIdle = $true
            break
        }
        Start-Sleep -Milliseconds 10
    }
    if (-not $selectedDiscoveryIdle) { throw 'Selected-axis discovery did not drain before release' }
    $diagnosticId = 81000122
    foreach ($diagnosticCommand in @('adrc acquisition motor=7', 'adrc hardware',
            'adrc limits', 'adrc feedback', 'adrc evidence')) {
        $diagnosticReply = Invoke-WindowRequest $serialPort ([uint32]$diagnosticId) $diagnosticCommand
        if ($diagnosticReply -notmatch "^ok $diagnosticId adrc ") {
            throw "Selected-axis diagnostic failed: $diagnosticReply"
        }
        ++$diagnosticId
    }

    if ($SampleDisabledFeedback) {
        $sampleReply = Invoke-WindowRequest $serialPort 81000300 'adrc sample'
        if ($sampleReply -notmatch '^ok 81000300 adrc sample state=1 capacity=64$') {
            throw "Disabled capture start rejected: $sampleReply"
        }
        $captureRequestId = $script:templateToWire['81000300']
        $captureDeadline = [datetime]::UtcNow.AddSeconds(2)
        $captureComplete = $false
        while ([datetime]::UtcNow -lt $captureDeadline) {
            $captureReply = Invoke-WindowRequest $serialPort 81000301 'adrc capture'
            if ($captureReply -notmatch "^ok 81000301 adrc capture id=$captureRequestId motor=7 state=(\d+) reason=(\d+) count=(\d+) requests=(\d+)") {
                throw "Capture identity mismatch: $captureReply"
            }
            if ($Matches[1] -eq '3') { throw "Disabled capture aborted: $captureReply" }
            if ($Matches[1] -eq '2') {
                if ($Matches[2] -ne '0' -or $Matches[3] -ne '64' -or [int]$Matches[4] -gt 68) {
                    throw "Invalid completion: $captureReply"
                }
                $captureComplete = $true
                break
            }
            Start-Sleep -Milliseconds 40
        }
        if (-not $captureComplete) { throw 'Disabled capture did not finish' }
        for ($captureIndex = 0; $captureIndex -lt 64; ++$captureIndex) {
            $captureRow = Invoke-WindowRequest $serialPort 81000302 'adrc capture' $captureIndex
            if ($captureRow -notmatch "^ok 81000302 adrc capture id=$captureRequestId index=$captureIndex observed_us=\d+ rx_lower_us=\d+") {
                throw "Capture row mismatch: $captureRow"
            }
        }
        $captureEvidence = Invoke-WindowRequest $serialPort 81000303 'adrc evidence'
        if ($captureEvidence -notmatch ' verified=0 ') { throw 'Diagnostic unexpectedly changed qualification' }
    }

    if ($ReadMotorCapabilities) {
        if (-not $captureComplete) { throw 'Capabilities require all 64 disabled samples first' }
        Invoke-MotorCapabilityRead $serialPort
    }

    if ($TestIndependentStop) {
        $guardReply = Invoke-WindowRequest $serialPort 81000400 'adrc guard'
        if ($guardReply -notmatch '^ok 81000400 adrc guard state=1 disable_only=1$') {
            throw "Independent stop selftest rejected: $guardReply"
        }
        $guardRequestId = $script:templateToWire['81000400']
        $guardComplete = $false
        $guardDeadline = [datetime]::UtcNow.AddSeconds(2)
        while ([datetime]::UtcNow -lt $guardDeadline) {
            $guardStatus = Invoke-WindowRequest $serialPort 81000401 'adrc guard_status'
            if ($guardStatus -notmatch "^ok 81000401 adrc guard_status id=$guardRequestId state=(\d+) reason=(\d+) deadline_ms=(\d+) fired_ms=(\d+) disabled_rx_us=(\d+)$") {
                throw "Independent stop evidence mismatch: $guardStatus"
            }
            if ($Matches[1] -eq '6') { throw "Independent stop selftest failed: $guardStatus" }
            if ($Matches[1] -eq '5') {
                if ($Matches[2] -ne '0' -or $Matches[3] -ne $Matches[4] -or [uint64]$Matches[5] -eq 0) {
                    throw "Invalid independent stop proof: $guardStatus"
                }
                $guardComplete = $true
                break
            }
            Start-Sleep -Milliseconds 10
        }
        if (-not $guardComplete) { throw 'Independent stop selftest did not complete' }
        $guardEvidence = Invoke-WindowRequest $serialPort 81000402 'adrc evidence'
        if ($guardEvidence -notmatch ' verified=0 ') { throw 'Selftest unexpectedly changed qualification' }
    }

    if ($TestSimpleAdrc) {
        Invoke-SimpleAdrcTrial $serialPort $sessionBootId $SimpleProfile
    }

    if ($TestShaftPilot) {
        Invoke-ShaftPilotTrial $serialPort $sessionBootId $ShaftDirection -Profile $ShaftProfile
    }

    if ($TestMotorWatchdog) {
        $currentBootReply = Invoke-WindowRequest $serialPort 81000499 'ping'
        if ($currentBootReply -notmatch " boot=$sessionBootId$") { throw 'Boot changed before watchdog trial' }
        $watchdogReply = Invoke-WindowRequest $serialPort 81000500 'adrc watchdog'
        if ($watchdogReply -notmatch '^ok 81000500 adrc watchdog state=1 timeout_raw=120 guard_ms=20$') {
            throw "Motor watchdog trial rejected: $watchdogReply"
        }
        $watchdogRequestId = $script:templateToWire['81000500']
        $watchdogComplete = $false
        $watchdogDeadline = [datetime]::UtcNow.AddSeconds(2)
        while ([datetime]::UtcNow -lt $watchdogDeadline) {
            $watchdogStatus = Invoke-WindowRequest $serialPort 81000501 'adrc watchdog_status'
            if ($watchdogStatus -notmatch "^ok 81000501 adrc watchdog_status id=$watchdogRequestId state=(\d+) reason=(\d+) enabled=(\d+) watchdog=(\d+) restored=(\d+) ") {
                throw "Motor watchdog evidence mismatch: $watchdogStatus"
            }
            if ($Matches[1] -in @('16', '17')) { throw "Motor watchdog trial did not qualify: $watchdogStatus" }
            if ($Matches[1] -eq '15') {
                if ($Matches[2] -ne '0' -or $Matches[3] -ne '1' -or $Matches[4] -ne '1' -or $Matches[5] -ne '1') {
                    throw "Invalid motor watchdog proof: $watchdogStatus"
                }
                $watchdogComplete = $true
                break
            }
            Start-Sleep -Milliseconds 10
        }
        if (-not $watchdogComplete) { throw 'Motor watchdog trial did not finish' }
        $currentBootReply = Invoke-WindowRequest $serialPort 81000503 'ping'
        if ($currentBootReply -notmatch " boot=$sessionBootId$") { throw 'Boot changed during watchdog trial' }
        $watchdogEvidence = Invoke-WindowRequest $serialPort 81000502 'adrc evidence'
        if ($watchdogEvidence -notmatch ' verified=0 ') { throw 'Watchdog diagnostic unexpectedly changed qualification' }
    }

    $releaseReply = Invoke-WindowRequest $serialPort 81000130 'adrc release'
    if ($releaseReply -notmatch '^ok 81000130 adrc release=accepted') {
        throw "ADRC release rejected: $releaseReply"
    }
    for ($pollIndex = 0; $pollIndex -lt 45; ++$pollIndex) {
        $ownerReply = Invoke-WindowRequest $serialPort ([uint32](81000131 + $pollIndex)) 'adrc status'
        if ($ownerReply -match 'owner=lcd handoff=released') {
            $releaseVerified = $true
            $ownerReturnedToLcd = $true
            break
        }
        if ($ownerReply -match 'owner=releasing handoff=timeout') {
            throw "ADRC release timed out and remains locked: $ownerReply"
        }
        Start-Sleep -Milliseconds 10
    }
    if (-not $releaseVerified) { throw 'ADRC release state was not confirmed' }
    Assert-WindowDisabledLcd $serialPort 81000180

    $postReleaseDiscoveryIdle = $false
    for ($pollIndex = 0; $pollIndex -lt 25; ++$pollIndex) {
        $gateReply = Invoke-WindowRequest $serialPort ([uint32](81000200 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $postReleaseDiscoveryIdle = $true
            break
        }
        Start-Sleep -Milliseconds 40
    }
    if (-not $postReleaseDiscoveryIdle) {
        throw 'Selected-axis discovery remained busy after release'
    }
    $restoreReply = Invoke-WindowRequest $serialPort 81000300 'bench mode 7 mode=pos_vel'
    if ($restoreReply -notmatch '^ok 81000300 bench mode accepted=1') {
        throw "Mode 2 restore rejected: $restoreReply"
    }
    [void](Wait-WindowCompletion $serialPort 81000300 'mode')
    $modeRestored = $true
    $gateReply = Invoke-WindowRequest $serialPort 81000301 'adrc gate motor=7'
    if ($gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Mode 2 restore readback failed'
    }
    $disableReply = Invoke-WindowRequest $serialPort 81000302 'bench disable 7'
    if ($disableReply -notmatch '^ok 81000302 bench disable accepted=1') {
        throw "Final DISABLE rejected: $disableReply"
    }
    [void](Wait-WindowCompletion $serialPort 81000302 'disable')
    $finalDisableCompleted = $true
    Assert-WindowDisabledLcd $serialPort 81000303
    $transcriptLines.Add('HANDOFF_WINDOW_COMPLETE')
    Write-Output ('ADRC_HANDOFF_WINDOW_COMPLETE motor=7 no_motion=' +
        [int](-not ($TestMotorWatchdog -or $TestSimpleAdrc -or $TestShaftPilot)) + ' mode_restored=1 watchdog_trial=' +
        [int][bool]$TestMotorWatchdog + ' simple_trial=' + [int][bool]$TestSimpleAdrc +
        ' shaft_trial=' + [int][bool]$TestShaftPilot + ' qualification=0')
}
catch {
    $transcriptLines.Add('host_error=' + $_.Exception.Message)
    Write-Error -ErrorAction Continue $_
    throw
}
finally {
    # A read-only diagnostic error after acquisition must still request the same
    # guarded release. Never restore legacy mode while ADRC retains ownership.
    if ($serialPort.IsOpen -and $acquireTransferred -and -not $releaseVerified) {
        try {
            [void](Invoke-WindowRequest $serialPort 81000880 'stop')
        }
        catch { $transcriptLines.Add('stop_cleanup_error=' + $_.Exception.Message) }
        # A lost status or STOP acknowledgement must not suppress the release attempt.
        try {
            $cleanupRelease = Invoke-WindowRequest $serialPort 81000881 'adrc release'
            if ($cleanupRelease -notmatch '^ok 81000881 adrc release=accepted') {
                $transcriptLines.Add('release_cleanup_reply=' + $cleanupRelease)
            }
        }
        catch { $transcriptLines.Add('release_request_cleanup_error=' + $_.Exception.Message) }
        try {
            for ($cleanupPoll = 0; $cleanupPoll -lt 45; ++$cleanupPoll) {
                $cleanupOwner = Invoke-WindowRequest $serialPort 81000882 'adrc status'
                if ($cleanupOwner -match 'owner=lcd handoff=released') {
                    $releaseVerified = $true
                    $ownerReturnedToLcd = $true
                    break
                }
                if ($cleanupOwner -match 'owner=releasing handoff=timeout') { break }
                Start-Sleep -Milliseconds 10
            }
        }
        catch { $transcriptLines.Add('guarded_release_cleanup_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $mitModeEntered) {
        try {
            $gateReply = Invoke-WindowRequest $serialPort 81000899 'adrc gate motor=7'
            if ($gateReply -match 'fb_fresh=1' -and
                ($gateReply -notmatch 'disabled=1' -or
                 $gateReply -notmatch 'no_fault=1' -or
                 $gateReply -notmatch 'stationary=1')) {
                $unsafeAcquire = $true
            }
        }
        catch { $transcriptLines.Add('acquire_gate_read_error=' + $_.Exception.Message) }
        try {
            $ownerReply = Invoke-WindowRequest $serialPort 81000900 'adrc status'
            $ownerReturnedToLcd = $ownerReply -match 'owner=lcd\b'
        }
        catch { $transcriptLines.Add('owner_check_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $mitModeEntered -and $ownerReturnedToLcd -and
        $unsafeAcquire) {
        try {
            $disableReply = Invoke-WindowRequest $serialPort 81000903 'bench disable 7'
            if ($disableReply -match '^ok 81000903 bench disable accepted=1') {
                [void](Wait-WindowCompletion $serialPort 81000903 'disable')
                $finalDisableCompleted = $true
            }
        }
        catch { $transcriptLines.Add('unsafe_disable_error=' + $_.Exception.Message) }
        $transcriptLines.Add('hardware_poweroff_required=1 unsafe_acquire=1')
    }
    if ($serialPort.IsOpen -and $mitModeEntered -and $ownerReturnedToLcd -and
        -not $unsafeAcquire -and -not $modeRestored) {
        try {
            $restoreReply = Invoke-WindowRequest $serialPort 81000901 'bench mode 7 mode=pos_vel'
            if ($restoreReply -match '^ok 81000901 bench mode accepted=1') {
                [void](Wait-WindowCompletion $serialPort 81000901 'mode')
                $modeRestored = $true
            }
        }
        catch { $transcriptLines.Add('restore_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $mitModeEntered -and $ownerReturnedToLcd -and
        -not $unsafeAcquire -and -not $finalDisableCompleted) {
        try {
            $disableReply = Invoke-WindowRequest $serialPort 81000902 'bench disable 7'
            if ($disableReply -match '^ok 81000902 bench disable accepted=1') {
                [void](Wait-WindowCompletion $serialPort 81000902 'disable')
                $finalDisableCompleted = $true
            }
        }
        catch { $transcriptLines.Add('disable_error=' + $_.Exception.Message) }
    }
    if ($mitModeEntered -and -not $ownerReturnedToLcd) {
        $transcriptLines.Add('hardware_poweroff_required=1 owner_not_lcd=1')
    }
    if ($serialPort.IsOpen) { $serialPort.Close() }
    $serialPort.Dispose()
    $outputDirectory = Join-Path $PSScriptRoot '..\..\output\adrc\hardware\handoff_window_20260923'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $outputName = 'motor7_handoff_' +
        [datetime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '.txt'
    $transcriptLines.Add('mode_restored=' + $modeRestored +
        ' final_disable_completed=' + $finalDisableCompleted +
        ' acquire_transferred=' + $acquireTransferred +
        ' release_verified=' + $releaseVerified)
    $transcriptLines.Add('host_end_utc=' + [datetime]::UtcNow.ToString('o'))
    $transcriptLines | Set-Content -LiteralPath (Join-Path $outputDirectory $outputName) -Encoding utf8
}
