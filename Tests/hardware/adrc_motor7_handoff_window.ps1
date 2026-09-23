<#
.SYNOPSIS
Verifies one non-motion motor-7 ADRC acquire and release in a single power window.
.DESCRIPTION
Requires a user-confirmed 24 V supply, explicit neutral-MIT challenge approval,
and the integrated LCD-MIT/ADRC firmware. The acquisition fallback requests
Kp=0, Kd=0, velocity=0, and feed-forward torque=0; protocol quantization may
make the encoded torque slightly nonzero. It sends no explicit ENABLE.
The command allowlist contains discovery, disabled mode switching, ownership
handoff, status queries, and final DISABLE. It requests no nonzero torque or speed.
If ownership does not return to LCD, it records the locked state and requires
physical power cutoff instead of attempting a legacy mode command.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed,
    [switch]$NeutralMitChallengeApproved
)

$ErrorActionPreference = 'Stop'
$transcriptLines = [System.Collections.Generic.List[string]]::new()
$completedReplies = @{}
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
    ' neutral_mit_challenge_approved=' + [bool]$NeutralMitChallengeApproved)

function Receive-WindowLine {
    <# Records one asynchronous line and caches terminal bench replies. #>
    param([System.IO.Ports.SerialPort]$Port)
    try { $replyLine = $Port.ReadLine().Trim() }
    catch [System.TimeoutException] { return $null }
    $script:transcriptLines.Add('< ' + $replyLine)
    if ($replyLine -match '^done (\d+) bench (mode|disable) result=') {
        $script:completedReplies[$Matches[1] + ':' + $Matches[2]] = $replyLine
    }
    return $replyLine
}

function Invoke-WindowRequest {
    <# Sends one allowlisted command and waits for its matching ACK. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('show state', 'show diag can', 'adrc status',
            'adrc gate motor=7', 'adrc discover motor=7',
            'bench mode 7 mode=mit', 'adrc acquire motor=7', 'adrc release',
            'bench mode 7 mode=pos_vel', 'bench disable 7')]
        [string]$Command
    )
    $requestText = "$RequestId $Command"
    $script:transcriptLines.Add('> ' + $requestText)
    $Port.Write($requestText + "`r`n")
    $deadlineUtc = [datetime]::UtcNow.AddMilliseconds(1500)
    while ([datetime]::UtcNow -lt $deadlineUtc) {
        $replyLine = Receive-WindowLine $Port
        if ($null -ne $replyLine -and $replyLine -match "^(ok|error) $RequestId ") {
            return $replyLine
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
    $completionKey = "$RequestId`:$Operation"
    $deadlineUtc = [datetime]::UtcNow.AddSeconds(4)
    while ([datetime]::UtcNow -lt $deadlineUtc) {
        if ($script:completedReplies.ContainsKey($completionKey)) {
            $completionReply = $script:completedReplies[$completionKey]
            if ($completionReply -match "^done $RequestId bench $Operation result=completed\b") {
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
    Write-Output 'ADRC_HANDOFF_WINDOW_COMPLETE motor=7 no_motion=1 mode_restored=1'
}
catch {
    $transcriptLines.Add('host_error=' + $_.Exception.Message)
    Write-Error -ErrorAction Continue $_
    throw
}
finally {
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
