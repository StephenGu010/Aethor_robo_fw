<#
.SYNOPSIS
Strict evidence parsing and finite host orchestration for the output-shaft pilot.
.DESCRIPTION
Defining these functions does not open a device. Firmware owns all motion limits;
host replies are observations and never install a model or motion qualification.
#>

function ConvertFrom-ShaftFields {
    <# Parses an exact schema without losing uint64 clocks or accepting malformed health fields. #>
    param(
        [string]$Reply,
        [ValidateSet('shaft_status','shaft_binding','shaft_trace')][string]$Kind,
        [uint32]$ExpectedRequestId,
        [System.Collections.IDictionary]$Schema
    )
    if ($Reply.Length -ge 520 -or $Reply -notmatch
        ('^ok ' + $ExpectedRequestId + ' adrc ' + $Kind + ' (.+)$')) {
        throw 'Shaft response identity, size or format mismatch'
    }
    $payload = $Matches[1]
    $fields = [ordered]@{}
    foreach ($token in $payload.Split(' ')) {
        if ($token -cnotmatch '^([a-z][a-z0-9_]*)=([^= ]+)$') {
            throw 'Malformed shaft field'
        }
        $name = $Matches[1]; $valueText = $Matches[2]
        if (-not $Schema.Contains($name) -or $fields.Contains($name)) {
            throw 'Unknown or duplicate shaft field'
        }
        $kindName = $Schema[$name]
        if ($kindName -eq 'f32') {
            [double]$numericValue = 0
            if ($valueText -notmatch '^-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$' -or
                -not [double]::TryParse($valueText, [Globalization.NumberStyles]::Float,
                    [cultureinfo]::InvariantCulture, [ref]$numericValue) -or
                [double]::IsNaN($numericValue) -or [double]::IsInfinity($numericValue) -or
                [Math]::Abs($numericValue) -gt [single]::MaxValue) {
                throw "Invalid finite float: $name"
            }
            $fields[$name] = $numericValue
        }
        elseif ($kindName -eq 'i32') {
            [int32]$signedValue = 0
            if ($valueText -notmatch '^-?[0-9]+$' -or
                -not [int32]::TryParse($valueText, [Globalization.NumberStyles]::AllowLeadingSign,
                    [cultureinfo]::InvariantCulture, [ref]$signedValue)) {
                throw "Invalid int32: $name"
            }
            $fields[$name] = $signedValue
        }
        else {
            [uint64]$unsignedValue = 0
            $maximumValue = switch ($kindName) {
                'u8' { [uint64][byte]::MaxValue }
                'u16' { [uint64][uint16]::MaxValue }
                'u32' { [uint64][uint32]::MaxValue }
                'u64' { [uint64]::MaxValue }
                default { throw "Unknown schema type: $kindName" }
            }
            if ($valueText -notmatch '^[0-9]+$' -or
                -not [uint64]::TryParse($valueText, [Globalization.NumberStyles]::None,
                    [cultureinfo]::InvariantCulture, [ref]$unsignedValue) -or
                $unsignedValue -gt $maximumValue) {
                throw "Invalid unsigned value: $name"
            }
            $fields[$name] = $unsignedValue
        }
    }
    foreach ($name in $Schema.Keys) {
        if (-not $fields.Contains($name)) { throw "Missing shaft field: $name" }
    }
    return $fields
}

function ConvertFrom-ShaftStatus {
    <# Checks the fixed status contract; a stopped failed trial is never reclassified as successful. #>
    param([string]$Reply, [uint32]$TrialId, [uint32]$BootId)
    $schema = [ordered]@{
        id='u32'; phase='u8'; outcome='u8'; fault='u8'; safe_release='u8'; guard_ack='u8'
        ready='u8'; count='u16'; bridge_fault='u8'; prepare='u8'; binding='u64'; boot='u32'
        last='f32'; peak='f32'; qualified='u8'; control='u8'
    }
    $status = ConvertFrom-ShaftFields $Reply 'shaft_status' 81000701 $schema
    if ($status.id -ne $TrialId -or $TrialId -eq 0 -or $status.boot -ne $BootId -or $BootId -eq 0 -or
        $status.phase -gt 6 -or $status.outcome -gt 4 -or $status.fault -gt 16 -or
        $status.safe_release -gt 1 -or $status.guard_ack -gt 1 -or $status.ready -gt 1 -or
        $status.count -gt 512 -or $status.prepare -gt 3 -or $status.peak -lt 0 -or
        $status.qualified -ne 0 -or $status.control -ne 0) {
        throw 'Shaft status identity or bounds mismatch'
    }
    if ($status.ready -eq 1 -and ($status.phase -ne 5 -or $status.outcome -notin @(1,2) -or
        $status.fault -ne 0 -or $status.bridge_fault -ne 0 -or $status.prepare -ne 2 -or
        $status.safe_release -ne 1 -or $status.guard_ack -ne 1 -or $status.binding -eq 0 -or
        $status.count -eq 0 -or $status.peak -gt 1.75)) {
        throw 'Shaft readiness contradicts its evidence'
    }
    return $status
}

function Assert-ShaftStatusUnchanged {
    <# Requires terminal evidence to remain fixed throughout export; later faults revoke host success. #>
    param([System.Collections.IDictionary]$Before, [System.Collections.IDictionary]$After)
    foreach ($name in $Before.Keys) {
        if (-not $After.Contains($name) -or $Before[$name] -ne $After[$name]) {
            throw "Shaft terminal evidence changed: $name"
        }
    }
}

function Get-ShaftShardSchema {
    <# Returns the fixed firmware schema; absent or additional fields invalidate an exported shard. #>
    param([ValidateSet('shaft_binding','shaft_trace')][string]$Kind, [ValidateRange(0,4)][int]$Part,
        [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$Profile = 'default')
    if ($Kind -eq 'shaft_trace' -and $Part -eq 4) { throw 'Trace still has exactly four shards' }
    if ($Kind -eq 'shaft_binding' -and $Part -eq 4 -and $Profile -eq 'platform70') {
        return [ordered]@{ id='u32'; index='u16'; profile='u8'; speed='f32'; reverse='f32'
            reserve='f32'; lookahead_us='u32'; entry_us='u32'; ramp_us='u32'; plateau_us='u32'; rated_rpm='f32' }
    }
    if ($Kind -eq 'shaft_binding' -and $Part -eq 4 -and $Profile -eq 'platform_wide') {
        return [ordered]@{ id='u32'; index='u16'; profile='u8'; speed='f32'; reverse='f32'
            reserve='f32'; lookahead_us='u32'; entry_us='u32'; ramp_us='u32'; plateau_us='u32'; rated_rpm='f32' }
    }
    if ($Kind -eq 'shaft_binding' -and $Part -eq 4 -and $Profile -eq 'platform') {
        return [ordered]@{ id='u32'; index='u16'; profile='u8'; speed='f32'; reverse='f32'
            reserve='f32'; lookahead_us='u32'; ramp_us='u32'; plateau_us='u32'; rated_rpm='f32' }
    }
    $schema = [ordered]@{ id='u32'; index='u16' }
    if ($Kind -eq 'shaft_trace') { $schema.part = 'u8' }
    $groups = if ($Kind -eq 'shaft_binding') {
        @(
            @{'u32'='boot epoch capture guard collector'; 'u64'='session binding'; 'i32'='direction'; 'f32'='cap slew step'; 'u8'='qualified control'},
            @{'f32'='gr pmax vmax tmax span velocity_peak raw_center torque_peak qnative qshaft qvelocity qtorque'; 'u16'='points'},
            @{'u32'='motor master mode timeout fields hw sw sub'; 'f32'='accel decel maxspeed'},
            @{'u32'='kt ot oc damp inertia gr valid_mask'},
            @{'u8'='profile'; 'f32'='speed angle reverse active stopped rated_rpm'; 'u32'='ramp_us plateau_us'}
        )
    } else {
        @(
            @{'u64'='now binding session rx observed'; 'u8'='motor ga gl gf gack stop valid'},
            @{'f32'='native_p raw_v torque mos rotor'; 'u8'='driver'; 'u32'='driver_fault'},
            @{'u8'='rvalid rsuccess rkind action omotor prev_valid'; 'u64'='rcmd rcompleted obinding osession ocmd issued'; 'f32'='rdecoded requested previous step cap'},
            @{'u8'='phase outcome fault vready conditional sampling_unknown'; 'f32'='shaft_p v_rx32'; 'u64'='rxold obsold rxnew obsnew tmin tmax'}
        )
    }
    foreach ($type in $groups[$Part].Keys) {
        foreach ($name in $groups[$Part][$type].Split(' ')) { $schema[$name] = $type }
    }
    return $schema
}

function ConvertFrom-ShaftShard {
    <# Checks shard identity and fixed index/part without hiding fault-bearing raw observations. #>
    param([string]$Reply, [ValidateSet('shaft_binding','shaft_trace')][string]$Kind,
        [uint32]$TrialId, [ValidateRange(0,511)][int]$Index, [ValidateRange(0,4)][int]$Part,
        [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$Profile = 'default')
    $requestId = if ($Kind -eq 'shaft_binding') { 81000704 } else { 81000702 }
    if ($Kind -eq 'shaft_binding' -and ($Index -gt 4 -or $Index -ne $Part)) { throw 'Invalid binding shard selection' }
    $fields = ConvertFrom-ShaftFields $Reply $Kind $requestId (Get-ShaftShardSchema $Kind $Part -Profile $Profile)
    if ($fields.id -ne $TrialId -or $fields.index -ne $Index -or
        ($Kind -eq 'shaft_trace' -and $fields.part -ne $Part)) { throw 'Shaft shard identity mismatch' }
    if ($Kind -eq 'shaft_trace') {
        foreach ($name in @('ga','gl','gf','gack','stop','valid','rvalid','rsuccess','prev_valid','vready','conditional','sampling_unknown')) {
            if ($fields.Contains($name) -and $fields[$name] -gt 1) { throw "Invalid trace boolean: $name" }
        }
        foreach ($entry in @(@('driver',2),@('rkind',3),@('action',3),@('phase',6),@('outcome',4),@('fault',16))) {
            if ($fields.Contains($entry[0]) -and $fields[$entry[0]] -gt $entry[1]) { throw 'Invalid trace enum' }
        }
    }
    return $fields
}

function Assert-ShaftBinding {
    <# Verifies the exported current binding and exact register float against the fixed reviewed plan. #>
    param([object[]]$Binding, [uint32]$BootId, [System.Collections.IDictionary]$Status, [string]$Direction,
        [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$Profile = 'default')
    $expectedBindingCount = if ($Profile -in @('observe','platform','platform_wide','platform70')) { 5 } else { 4 }
    if ($Binding.Count -ne $expectedBindingCount) { throw 'Incomplete or unexpected shaft binding' }
    $identity = $Binding[0]; $range = $Binding[1]; $discovery = $Binding[2]; $bits = $Binding[3]
    if ($range.pmax -eq 1024 -and $Profile -ne 'platform70') { throw 'PMAX1024 requires the explicit platform70 profile' }
    if ($Profile -eq 'platform70') {
        foreach ($part in 0..4) {
            $partSchema = Get-ShaftShardSchema 'shaft_binding' $part -Profile $Profile
            if ($Binding[$part].Count -ne $partSchema.Count -or
                @($Binding[$part].Keys | Where-Object { -not $partSchema.Contains($_) }).Count -ne 0 -or
                $Binding[$part].id -ne $Status.id -or $Binding[$part].index -ne $part) {
                throw 'Rated70 binding schema or shard identity mismatch'
            }
        }
    }
    if ($Profile -in @('observe','platform','platform_wide','platform70')) {
        $diagnostic = $Binding[4]
        $diagnosticSchema = Get-ShaftShardSchema 'shaft_binding' 4 -Profile $Profile
        $expectedProfileId = switch ($Profile) { 'platform70' { 7 }; 'platform_wide' { 6 }; 'platform' { 5 }; default { 4 } }
        if ($diagnostic.Count -ne $diagnosticSchema.Count -or
            @($diagnostic.Keys | Where-Object { -not $diagnosticSchema.Contains($_) }).Count -ne 0 -or
            $diagnostic.id -ne $Status.id -or $diagnostic.index -ne 4 -or $diagnostic.profile -ne $expectedProfileId -or
            $diagnostic.ramp_us -ne 900000 -or $diagnostic.plateau_us -ne 200000) {
            throw 'Diagnostic binding schema or identity mismatch'
        }
        $diagnosticValues = if ($Profile -eq 'platform70') {
            @{ speed=[single]28.9550123; reverse=[single]0.0872664626; reserve=[single]25.1327412; rated_rpm=[single]395 }
        } elseif ($Profile -eq 'platform_wide') {
            @{ speed=[single]20.6821516; reverse=[single]0.0872664626; reserve=[single]12.5663706; rated_rpm=[single]395 }
        } elseif ($Profile -eq 'platform') {
            @{ speed=[single]20.6821516; reverse=[single]0.0872664626; reserve=[single]3.14159265; rated_rpm=[single]395 }
        } else {
            @{ speed=[single]20.6821516; angle=[single]0.523598776; reverse=[single]0.0872664626
                active=[single]1.57079633; stopped=[single]3.14159265; rated_rpm=[single]395 }
        }
        foreach ($name in $diagnosticValues.Keys) {
            if ([single]$diagnostic[$name] -ne $diagnosticValues[$name]) { throw "Diagnostic fixed binary32 mismatch: $name" }
        }
        if ($Profile -in @('platform','platform_wide','platform70') -and ($diagnostic.lookahead_us -ne 25000 -or
            ($Status.ready -eq 1 -and $Status.outcome -ne 2))) {
            throw 'Platform requires its fixed range lookahead and a normal TIME completion'
        }
        if ($Profile -in @('platform_wide','platform70') -and $diagnostic.entry_us -ne 1058000) {
            throw 'Wide platform requires its fixed 1058000us entry budget'
        }
        $expectedPmax = switch ($Profile) { 'platform70' { 1024 }; 'platform_wide' { 512 }; default { 12.5 } }
        if ($range.pmax -ne $expectedPmax -or $range.vmax -ne 200 -or $range.tmax -ne 10) {
            throw "Diagnostic requires synchronized PMAX$expectedPmax/VMAX200/TMAX10"
        }
    }
    $requestTemplates = @{ capture='81000300'; guard='81000400'; collector='81000310' }
    foreach ($name in $requestTemplates.Keys) {
        $template = $requestTemplates[$name]
        if ($null -eq $script:templateToWire -or -not $script:templateToWire.ContainsKey($template) -or
            $identity[$name] -ne $script:templateToWire[$template]) { throw "Shaft source is not this session: $name" }
    }
    $capabilityReadback = $script:shaftCapabilityReadback
    if ($null -eq $capabilityReadback -or $capabilityReadback.id -ne $identity.collector -or
        $capabilityReadback.epoch -ne $identity.epoch -or $capabilityReadback.rows.Count -ne 6) {
        throw 'Shaft capability collection does not match current readback'
    }
    $registerNames = @('kt','ot','oc','damp','inertia','gr')
    for ($registerIndex = 0; $registerIndex -lt 6; ++$registerIndex) {
        if ($bits[$registerNames[$registerIndex]] -ne $capabilityReadback.rows[$registerIndex].raw_bits) {
            throw 'Shaft capability raw bits differ from this session'
        }
    }
    $expectedDirection = if ($Direction -eq 'positive') { 1 } else { -1 }
    # Exact binary32 plan identity accepts the actual %.9g rendering without a wider numerical allowance.
    $expectedCap = switch ($Profile) { 'fine' { [single]0.20 }; 'low' { [single]0.35 }; 'visible' { [single]0.5 }; default { [single]1.75 } }
    if ($Profile -eq 'fine' -and $range.vmax -ne 5) { throw 'Fine profile requires the synchronized VMAX=5 mapping' }
    $expectedSlew = if ($Profile -eq 'visible') { 2 } else { 4 }
    $expectedStep = if ($Profile -eq 'visible') { [single]0.008 } else { [single]0.016 }
    if ($Status.ready -eq 1 -and $Status.peak -gt [double]$expectedCap) {
        throw 'Successful shaft peak exceeds the selected finite profile'
    }
    if ($identity.boot -ne $BootId -or $identity.binding -ne $Status.binding -or $identity.epoch -eq 0 -or
        $identity.session -eq 0 -or $identity.capture -eq 0 -or $identity.guard -eq 0 -or $identity.collector -eq 0 -or
        $identity.direction -ne $expectedDirection -or [single]$identity.cap -ne $expectedCap -or $identity.slew -ne $expectedSlew -or
        [single]$identity.step -ne $expectedStep -or $identity.qualified -ne 0 -or $identity.control -ne 0 -or
        $discovery.motor -ne 7 -or $discovery.master -ne 23 -or $discovery.mode -ne 1 -or
        $discovery.timeout -ne 0 -or $discovery.fields -ne 8191 -or $bits.valid_mask -ne 63 -or
        $range.points -ne 64 -or $range.gr -le 0 -or $range.pmax -le 0 -or $range.vmax -le 0 -or $range.tmax -le 0 -or
        $range.span -lt 0 -or $range.velocity_peak -lt 0 -or $range.torque_peak -lt 0 -or $range.torque_peak -gt 0.15 -or
        [Math]::Abs($range.raw_center) -gt 0.05) { throw 'Shaft binding differs from the finite reviewed plan' }
    $quantums = @{ qnative=2*$range.pmax/65535; qshaft=2*$range.pmax/65535/$range.gr
        qvelocity=2*$range.vmax/4095; qtorque=2*$range.tmax/4095 }
    foreach ($name in $quantums.Keys) {
        # Eight float32 epsilons bound arithmetic/decimal round-trip of this short expression.
        if ($range[$name] -le 0 -or [Math]::Abs($range[$name] - $quantums[$name]) -gt
            8.0 * 1.1920928955078125e-7 * $quantums[$name]) { throw "Shaft quantum mismatch: $name" }
    }
    $rawValues = @{}
    foreach ($name in @('kt','ot','oc','damp','inertia','gr')) {
        $rawValues[$name] = [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$bits[$name]), 0)
        if ([single]::IsNaN($rawValues[$name]) -or [single]::IsInfinity($rawValues[$name])) { throw 'Nonfinite raw capability' }
    }
    if ($rawValues.kt -lt 0 -or $rawValues.ot -lt 80 -or $rawValues.ot -ge 200 -or
        $rawValues.oc -le 0 -or $rawValues.oc -ge 1 -or $rawValues.gr -le 0 -or
        $rawValues.gr -ne [single]$range.gr) { throw 'Raw capability binding mismatch' }
}

function Export-ShaftPilotTrace {
    <# Exports complete frozen shards once; any missing row or later state change prevents host success. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$TrialId, [uint32]$BootId,
        [System.Collections.IDictionary]$Status, [string]$Direction,
        [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$Profile = 'default')
    $before = ConvertFrom-ShaftStatus (Invoke-WindowRequest $Port 81000701 'adrc shaft_status') $TrialId $BootId
    Assert-ShaftStatusUnchanged $Status $before
    $binding = @()
    $lastBindingPart = if ($Profile -in @('observe','platform','platform_wide','platform70')) { 4 } else { 3 }
    foreach ($part in 0..$lastBindingPart) {
        $reply = Invoke-WindowRequest $Port 81000704 'adrc shaft_binding' -CaptureIndex $part
        $binding += ,(ConvertFrom-ShaftShard $reply 'shaft_binding' $TrialId $part $part -Profile $Profile)
    }
    Assert-ShaftBinding $binding $BootId $Status $Direction -Profile $Profile
    $script:transcriptLines.Add('shaft_binding_data=' + (ConvertTo-Json -InputObject $binding -Depth 5 -Compress))
    $deadline = [datetime]::UtcNow.AddSeconds(90)
    for ($index = 0; $index -lt $Status.count; ++$index) {
        $shards = @()
        foreach ($part in 0..3) {
            if ([datetime]::UtcNow -ge $deadline) { throw 'Shaft trace export timed out' }
            $reply = Invoke-WindowRequest $Port 81000702 'adrc shaft_trace' -TraceIndex $index -TracePart $part
            $shards += ,(ConvertFrom-ShaftShard $reply 'shaft_trace' $TrialId $index $part)
        }
        if ($Status.ready -eq 1 -and ($shards[0].binding -ne $binding[0].binding -or
            $shards[0].session -ne $binding[0].session -or $shards[0].motor -ne 7 -or
            $shards[3].fault -ne 0 -or $shards[3].conditional -ne 1 -or $shards[3].sampling_unknown -ne 1)) {
            throw 'Successful pilot contains inconsistent trace binding or a fault'
        }
        if ($Status.ready -eq 1 -and (($shards[2].action -ne 0 -and
            [single]$shards[2].cap -ne [single]$binding[0].cap) -or
            ($shards[2].rvalid -eq 1 -and $shards[2].rsuccess -eq 1 -and $shards[2].rkind -eq 2 -and
             [Math]::Abs($shards[2].rdecoded) -gt $binding[0].cap))) {
            throw 'Successful trace exceeds or changes its selected finite profile'
        }
        $script:transcriptLines.Add('shaft_trace_data=' + (ConvertTo-Json -InputObject $shards -Depth 5 -Compress))
    }
    $after = ConvertFrom-ShaftStatus (Invoke-WindowRequest $Port 81000701 'adrc shaft_status') $TrialId $BootId
    Assert-ShaftStatusUnchanged $Status $after
    $afterBoot = Invoke-WindowRequest $Port 81000709 'ping'
    if ($afterBoot -notmatch " boot=$BootId$") { throw 'Boot changed during shaft trace export' }
    $script:transcriptLines.Add('shaft_trace_rows=' + $Status.count + ' closed_loop=0 qualification=0')
}

function Invoke-ShaftPilotTrial {
    <# Starts exactly one firmware-bounded pilot, preserves failure, and exports only stopped evidence. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$BootId,
        [ValidateSet('positive','negative')][string]$Direction,
        [ValidateSet('default','low','visible','fine','observe','platform','platform_wide','platform70')][string]$Profile = 'default')
    $beforeBoot = Invoke-WindowRequest $Port 81000709 'ping'
    if ($beforeBoot -notmatch " boot=$BootId$") { throw 'Boot changed before shaft pilot' }
    $primaryFailure = $null; $lastStatus = $null; $trialId = $null
    try {
        $startCommand = 'adrc shaft_pilot direction=' + $Direction
        if ($Profile -ne 'default') { $startCommand += ' profile=' + $Profile }
        $startReply = Invoke-WindowRequest $Port 81000700 $startCommand
        $trialId = $script:templateToWire['81000700']
        if ($startReply -cne 'ok 81000700 adrc shaft_pilot accepted=1 motor=7 qualified=0 control=0') {
            throw "Shaft pilot rejected: $startReply"
        }
        $deadline = [datetime]::UtcNow.AddSeconds(4)
        do {
            $lastStatus = ConvertFrom-ShaftStatus (Invoke-WindowRequest $Port 81000701 'adrc shaft_status') $trialId $BootId
            if ($lastStatus.phase -in @(5,6) -and $lastStatus.safe_release -eq 1 -and
                $lastStatus.guard_ack -eq 1 -and ($lastStatus.ready -eq 1 -or
                    $lastStatus.fault -ne 0 -or $lastStatus.bridge_fault -ne 0 -or $lastStatus.outcome -notin @(1,2))) { break }
            Start-Sleep -Milliseconds 20
        } while ([datetime]::UtcNow -lt $deadline)
        if ($lastStatus.phase -notin @(5,6) -or $lastStatus.safe_release -ne 1 -or $lastStatus.guard_ack -ne 1) {
            throw 'Shaft pilot stopped evidence timed out; proceed to STOP cleanup'
        }
        if ($lastStatus.ready -ne 1) { throw 'Shaft pilot failed its finite trial; later motion is prohibited' }
        if ($Profile -in @('platform','platform_wide','platform70') -and $lastStatus.outcome -ne 2) {
            throw 'Platform requires normal TIME completion; later motion is prohibited'
        }
    }
    catch { $primaryFailure = $_ }
    if ($null -eq $trialId -and $script:templateToWire.ContainsKey('81000700')) {
        $trialId = $script:templateToWire['81000700']
    }
    try {
        if ($null -eq $lastStatus -or $lastStatus.phase -notin @(5,6) -or
            $lastStatus.safe_release -ne 1 -or $lastStatus.guard_ack -ne 1) {
            throw 'Shaft state is active, unknown or unconfirmed; trace deferred for immediate STOP cleanup'
        }
        $afterBoot = Invoke-WindowRequest $Port 81000709 'ping'
        if ($afterBoot -notmatch " boot=$BootId$") { throw 'Boot changed during shaft pilot' }
        Export-ShaftPilotTrace $Port $trialId $BootId $lastStatus $Direction -Profile $Profile
        $evidenceReply = Invoke-WindowRequest $Port 81000703 'adrc evidence'
        if ($evidenceReply -notmatch ' verified=0 ') { throw 'Pilot unexpectedly changed qualification' }
    }
    catch {
        $script:transcriptLines.Add('shaft_export_error=' + $_.Exception.Message)
        if ($null -eq $primaryFailure) { $primaryFailure = $_ }
    }
    if ($null -ne $primaryFailure) { throw $primaryFailure }
    $script:transcriptLines.Add('shaft_pilot_completed=1 closed_loop=0 qualification=0 no_motion=0 direction=' +
        $Direction + ' profile=' + $Profile.ToLowerInvariant() + ' peak=' +
        $lastStatus.peak.ToString('G9', [cultureinfo]::InvariantCulture))
}
