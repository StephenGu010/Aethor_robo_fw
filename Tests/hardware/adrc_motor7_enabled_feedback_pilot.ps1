<#
.SYNOPSIS
Records feedback during one bounded, low-gain motor-7 MIT HOLD pilot.

.DESCRIPTION
Run only after explicit approval of a powered, potentially moving test. The
existing firmware owns discovery, preflight DISABLE, mode readback, ENABLE,
100 ms HOLD, and feedback-confirmed DISABLE. This host script preflights LCD
ownership, records feedback, requests STOP on a host-side limit or timeout,
restores POS_VEL when possible, and finally requests DISABLE. Host-side polling
cannot replace the physical 24 V cutoff or prove a 4 ms feedback period.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed,
    [switch]$EnabledPilotApproved,
    [switch]$SupplyCurrentLimitConfirmed
)

$ErrorActionPreference = 'Stop'
$transcriptLines = [System.Collections.Generic.List[string]]::new()
$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 30
$serialPort.WriteTimeout = 500
$serialPort.NewLine = "`n"
$holdRequestSent = $false
$holdRequestAccepted = $false
$holdTerminalReply = $null
$modeRestored = $false
$finalDisableCompleted = $false
$finalStateSafe = $false
$initialPositionDeg = $null
$feedbackPollCount = 0
$freshActivePollCount = 0
$pilotFailed = $false
$completedReplies = @{}
$transcriptLines.Add('host_utc=' + [datetime]::UtcNow.ToString('o') +
    ' port=' + $PortName + ' power24v_confirmed=' + [bool]$Power24vConfirmed +
    ' enabled_pilot_approved=' + [bool]$EnabledPilotApproved +
    ' supply_current_limit_confirmed=' + [bool]$SupplyCurrentLimitConfirmed)

function Receive-PilotLine {
    <# Records one received line, including asynchronous MIT and cleanup results. #>
    param([System.IO.Ports.SerialPort]$Port)
    try { $replyLine = $Port.ReadLine().Trim() }
    catch [System.TimeoutException] { return $null }
    $script:transcriptLines.Add('< ' + $replyLine)
    if ($replyLine -match '^done 66000040 bench mit result=') {
        $script:holdTerminalReply = $replyLine
    }
    if ($replyLine -match '^done (\d+) bench (stop|mode|disable) result=') {
        $script:completedReplies[$Matches[1] + ':' + $Matches[2]] = $replyLine
    }
    return $replyLine
}

function Invoke-PilotRequest {
    <# Sends one allowlisted command and waits for its matching ACK. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('show state', 'show motor 7', 'show diag can',
            'adrc status', 'adrc gate motor=7', 'adrc discover motor=7',
            'bench mit 7 action=hold kp=1 kd=1 torque_ff=0 duration_ms=100',
            'bench stop 7', 'bench disable 7', 'bench mode 7 mode=pos_vel')]
        [string]$Command
    )
    $requestText = "$RequestId $Command"
    $script:transcriptLines.Add('> ' + $requestText)
    $Port.Write($requestText + "`r`n")
    $deadline = [datetime]::UtcNow.AddMilliseconds(1200)
    while ([datetime]::UtcNow -lt $deadline) {
        $replyLine = Receive-PilotLine $Port
        if ($null -ne $replyLine -and $replyLine -match "^(ok|error) $RequestId ") {
            return $replyLine
        }
    }
    throw "Request timed out: $requestText"
}

function Wait-PilotCompletion {
    <# Requires the terminal reply of a specific cleanup operation. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$RequestId,
          [ValidateSet('stop', 'mode', 'disable')][string]$Operation)
    $completionKey = "$RequestId`:$Operation"
    $deadline = [datetime]::UtcNow.AddSeconds(4)
    while ([datetime]::UtcNow -lt $deadline) {
        if ($script:completedReplies.ContainsKey($completionKey)) {
            $completionReply = $script:completedReplies[$completionKey]
            if ($completionReply -match "^done $RequestId bench $Operation result=completed\b") {
                return $completionReply
            }
            throw "Cleanup failed: $completionReply"
        }
        $replyLine = Receive-PilotLine $Port
    }
    throw "Cleanup completion timed out: $RequestId bench $Operation"
}

function Assert-PilotPreflight {
    <# Requires a disabled, idle LCD owner without CAN bus-off. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$RequestId)
    $stateReply = Invoke-PilotRequest $Port $RequestId 'show state'
    $ownerReply = Invoke-PilotRequest $Port ([uint32]($RequestId + 1)) 'adrc status'
    $canReply = Invoke-PilotRequest $Port ([uint32]($RequestId + 2)) 'show diag can'
    if ($stateReply -notmatch 'enabled=00 moving=0 .*fault=none' -or
        $ownerReply -notmatch 'owner=lcd\b' -or
        $canReply -notmatch 'busoff=0') {
        throw 'Pilot preflight failed: motor not disabled, LCD not owner, or CAN bus-off'
    }
}

function Record-PilotFeedback {
    <# Logs one feedback poll and aborts on gross speed, torque, or travel. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$RequestId)
    $motorReply = Invoke-PilotRequest $Port $RequestId 'show motor 7'
    if ($motorReply -notmatch 'state=(\w+) pos_deg=([-+\d.]+) speed_deg_s=([-+\d.]+) torque_nm=([-+\d.]+).* age_ms=(\d+)') {
        throw "Unparseable motor feedback: $motorReply"
    }
    $feedbackState = $Matches[1]
    $positionDeg = [double]::Parse($Matches[2], [globalization.CultureInfo]::InvariantCulture)
    $speedDegS = [double]::Parse($Matches[3], [globalization.CultureInfo]::InvariantCulture)
    $torqueNm = [double]::Parse($Matches[4], [globalization.CultureInfo]::InvariantCulture)
    $ageMs = [uint64]::Parse($Matches[5], [globalization.CultureInfo]::InvariantCulture)
    ++$script:feedbackPollCount
    if ($null -eq $script:initialPositionDeg -and $feedbackState -eq 'disabled' -and $ageMs -lt 100) {
        $script:initialPositionDeg = $positionDeg
    }
    # The public text protocol labels enabled feedback as moving or holding.
    if ($feedbackState -in @('moving', 'holding') -and $ageMs -lt 100) {
        ++$script:freshActivePollCount
    }
    if ([math]::Abs($speedDegS) -gt 12.0 -or
        [math]::Abs($torqueNm) -gt 1.0 -or
        ($null -ne $script:initialPositionDeg -and
         [math]::Abs($positionDeg - $script:initialPositionDeg) -gt 2.0)) {
        throw "Host-side pilot limit exceeded: $motorReply"
    }
}

try {
    if (-not $Power24vConfirmed -or -not $EnabledPilotApproved -or
        -not $SupplyCurrentLimitConfirmed) {
        throw 'Requires confirmed 24 V ON, enabled-pilot approval, and a verified supply current limit'
    }
    if ($PortName -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
        throw "Serial port unavailable: $PortName"
    }
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    Assert-PilotPreflight $serialPort 66000000
    $gateReply = Invoke-PilotRequest $serialPort 66000003 'adrc gate motor=7'
    if ($gateReply -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateReply -notmatch 'discovery_active=0') {
        throw 'Pilot preflight failed: LCD, CAN, or discovery busy'
    }
    $discoverReply = Invoke-PilotRequest $serialPort 66000004 'adrc discover motor=7'
    if ($discoverReply -notmatch '^ok 66000004 adrc discover=accepted') {
        throw "Discovery rejected: $discoverReply"
    }
    $discoveryComplete = $false
    for ($pollIndex = 0; $pollIndex -lt 20; ++$pollIndex) {
        Start-Sleep -Milliseconds 50
        $gateReply = Invoke-PilotRequest $serialPort ([uint32](66000005 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $discoveryComplete = $true
            break
        }
    }
    if (-not $discoveryComplete -or
        $gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Original mode 2 or complete discovery was not verified'
    }
    Assert-PilotPreflight $serialPort 66000030
    $preflightDisableReply = Invoke-PilotRequest $serialPort 66000034 'bench disable 7'
    if ($preflightDisableReply -notmatch '^ok 66000034 bench disable accepted=1') {
        throw "Preflight DISABLE rejected: $preflightDisableReply"
    }
    [void](Wait-PilotCompletion $serialPort 66000034 'disable')
    Record-PilotFeedback $serialPort 66000035
    if ($null -eq $initialPositionDeg) {
        throw 'Preflight DISABLE produced no fresh disabled position anchor'
    }

    $holdRequestSent = $true
    $holdReply = Invoke-PilotRequest $serialPort 66000040 `
        'bench mit 7 action=hold kp=1 kd=1 torque_ff=0 duration_ms=100'
    if ($holdReply -notmatch '^ok 66000040 bench mit accepted=1') {
        throw "MIT HOLD rejected: $holdReply"
    }
    $holdRequestAccepted = $true
    $pilotDeadline = [datetime]::UtcNow.AddSeconds(3)
    for ($pollIndex = 0; $null -eq $holdTerminalReply -and
         [datetime]::UtcNow -lt $pilotDeadline; ++$pollIndex) {
        Record-PilotFeedback $serialPort ([uint32](66000100 + $pollIndex))
        Start-Sleep -Milliseconds 10
    }
    if ($null -eq $holdTerminalReply) {
        throw 'MIT HOLD had no terminal result within three seconds'
    }
    if ($holdTerminalReply -notmatch '^done 66000040 bench mit result=completed\b') {
        throw "MIT HOLD did not complete safely: $holdTerminalReply"
    }
    $transcriptLines.Add('pilot_result=completed')
}
catch {
    $pilotFailed = $true
    $transcriptLines.Add('host_error=' + $_.Exception.Message)
    Write-Error -ErrorAction Continue $_
}
finally {
    if ($serialPort.IsOpen -and $holdRequestSent -and $null -eq $holdTerminalReply) {
        try {
            $stopReply = Invoke-PilotRequest $serialPort 66000900 'bench stop 7'
            $transcriptLines.Add('stop_reply=' + $stopReply)
            if ($stopReply -match '^ok 66000900 bench stop accepted=1') {
                [void](Wait-PilotCompletion $serialPort 66000900 'stop')
            }
        }
        catch { $transcriptLines.Add('stop_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $holdRequestSent) {
        try {
            $disableReply = Invoke-PilotRequest $serialPort 66000901 'bench disable 7'
            if ($disableReply -match '^ok 66000901 bench disable accepted=1') {
                [void](Wait-PilotCompletion $serialPort 66000901 'disable')
                $finalDisableCompleted = $true
            }
            else { $transcriptLines.Add('disable_rejected=' + $disableReply) }
        }
        catch { $transcriptLines.Add('disable_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $finalDisableCompleted) {
        try {
            $restoreReply = Invoke-PilotRequest $serialPort 66000902 'bench mode 7 mode=pos_vel'
            if ($restoreReply -match '^ok 66000902 bench mode accepted=1') {
                [void](Wait-PilotCompletion $serialPort 66000902 'mode')
                $modeRestored = $true
            }
            else { $transcriptLines.Add('restore_rejected=' + $restoreReply) }
        }
        catch { $transcriptLines.Add('restore_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $holdRequestSent) {
        try {
            $disableReply = Invoke-PilotRequest $serialPort 66000906 'bench disable 7'
            if ($disableReply -match '^ok 66000906 bench disable accepted=1') {
                [void](Wait-PilotCompletion $serialPort 66000906 'disable')
                $finalDisableCompleted = $true
            }
            else {
                $finalDisableCompleted = $false
                $transcriptLines.Add('final_disable_rejected=' + $disableReply)
            }
        }
        catch {
            $finalDisableCompleted = $false
            $transcriptLines.Add('final_disable_error=' + $_.Exception.Message)
        }
    }
    if ($serialPort.IsOpen) {
        try {
            $stateReply = Invoke-PilotRequest $serialPort 66000903 'show state'
            $ownerReply = Invoke-PilotRequest $serialPort 66000904 'adrc status'
            $canReply = Invoke-PilotRequest $serialPort 66000905 'show diag can'
            $transcriptLines.Add('final_state=' + $stateReply)
            $transcriptLines.Add('final_owner=' + $ownerReply)
            $transcriptLines.Add('final_can=' + $canReply)
            $finalStateSafe = ($stateReply -match 'enabled=00 moving=0 .*fault=none' -and
                $ownerReply -match 'owner=lcd\b' -and $canReply -match 'busoff=0')
        }
        catch { $transcriptLines.Add('final_check_error=' + $_.Exception.Message) }
        try {
            $timingReply = Invoke-PilotRequest $serialPort 66000907 'adrc gate motor=7'
            $transcriptLines.Add('final_timing_gate=' + $timingReply)
        }
        catch { $transcriptLines.Add('final_timing_error=' + $_.Exception.Message) }
        $serialPort.Close()
    }
    $serialPort.Dispose()
    $outputDirectory = Join-Path $PSScriptRoot '..\..\output\adrc\hardware\enabled_feedback_pilot'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $outputName = 'motor7_enabled_feedback_' + [datetime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '.txt'
    $transcriptLines.Add('hold_terminal=' + $holdTerminalReply)
    $transcriptLines.Add('feedback_polls=' + $feedbackPollCount +
        ' fresh_active_polls=' + $freshActivePollCount +
        ' final_disable_completed=' + $finalDisableCompleted +
        ' mode_restored=' + $modeRestored +
        ' final_state_safe=' + $finalStateSafe)
    $transcriptLines.Add('host_end_utc=' + [datetime]::UtcNow.ToString('o'))
    $transcriptLines | Set-Content -LiteralPath (Join-Path $outputDirectory $outputName) -Encoding utf8
    Write-Output "ADRC_ENABLED_FEEDBACK_PILOT_LOG_WRITTEN log=$outputName final_disable_completed=$finalDisableCompleted mode_restored=$modeRestored"
}
if ($pilotFailed -or ($holdRequestSent -and
    (-not $finalDisableCompleted -or -not $modeRestored -or -not $finalStateSafe))) { exit 1 }
