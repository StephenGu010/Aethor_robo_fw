<#
.SYNOPSIS
Verifies one non-motion motor-7 ADRC acquire and release in a single power window.
.DESCRIPTION
Requires a user-confirmed 24 V supply and the integrated LCD-MIT/ADRC firmware.
The command allowlist contains discovery, disabled mode switching, ownership
handoff, status queries, and final DISABLE. It sends no ENABLE, torque, or motion.
If ownership does not return to LCD, it records the locked state and requires
physical power cutoff instead of attempting a legacy mode command.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed
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
$transcriptLines.Add('host_start_utc=' + [datetime]::UtcNow.ToString('o') +
    ' port=' + $PortName + ' power24v_confirmed=' + [bool]$Power24vConfirmed)

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
    if ($PortName -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
        throw "Serial port unavailable: $PortName"
    }
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    Assert-WindowDisabledLcd $serialPort 67000001
    $gateReply = Invoke-WindowRequest $serialPort 67000004 'adrc gate motor=7'
    if ($gateReply -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateReply -notmatch 'discovery_active=0') {
        throw 'Discovery preflight stopped: LCD, CAN, or discovery is busy'
    }

    $discoverReply = Invoke-WindowRequest $serialPort 67000005 'adrc discover motor=7'
    if ($discoverReply -notmatch '^ok 67000005 adrc discover=accepted') {
        throw "Discovery rejected: $discoverReply"
    }
    for ($pollIndex = 0; $pollIndex -lt 25; ++$pollIndex) {
        Start-Sleep -Milliseconds 40
        $gateReply = Invoke-WindowRequest $serialPort ([uint32](67000006 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $discoveryComplete = $true
            break
        }
    }
    if (-not $discoveryComplete -or
        $gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Original mode 2 or complete motor-7 discovery was not verified'
    }
    Assert-WindowDisabledLcd $serialPort 67000040

    $modeReply = Invoke-WindowRequest $serialPort 67000050 'bench mode 7 mode=mit'
    if ($modeReply -notmatch '^ok 67000050 bench mode accepted=1') {
        throw "MIT mode switch rejected: $modeReply"
    }
    $mitModeEntered = $true
    [void](Wait-WindowCompletion $serialPort 67000050 'mode')
    $gateReply = Invoke-WindowRequest $serialPort 67000051 'adrc gate motor=7'
    if ($gateReply -notmatch 'mode=1 fields=1fff verified=40' -or
        $gateReply -notmatch 'lcd_idle=1 can_idle=1') {
        throw 'MIT mode readback or ownership gate failed'
    }
    Assert-WindowDisabledLcd $serialPort 67000052

    $acquireReply = Invoke-WindowRequest $serialPort 67000060 'adrc acquire motor=7'
    if ($acquireReply -notmatch '^ok 67000060 adrc acquire=accepted') {
        throw "ADRC acquire rejected: $acquireReply"
    }
    $ownerReturnedToLcd = $false
    for ($pollIndex = 0; $pollIndex -lt 20; ++$pollIndex) {
        $ownerReply = Invoke-WindowRequest $serialPort ([uint32](67000061 + $pollIndex)) 'adrc status'
        if ($ownerReply -match 'owner=adrc handoff=transferred') {
            $acquireTransferred = $true
            break
        }
        if ($ownerReply -match 'owner=lcd handoff=(unsafe|timeout)') {
            $ownerReturnedToLcd = $true
            throw "ADRC acquire did not transfer: $ownerReply"
        }
        Start-Sleep -Milliseconds 5
    }
    if (-not $acquireTransferred) { throw 'ADRC acquire state was not confirmed' }

    $releaseReply = Invoke-WindowRequest $serialPort 67000090 'adrc release'
    if ($releaseReply -notmatch '^ok 67000090 adrc release=accepted') {
        throw "ADRC release rejected: $releaseReply"
    }
    for ($pollIndex = 0; $pollIndex -lt 45; ++$pollIndex) {
        $ownerReply = Invoke-WindowRequest $serialPort ([uint32](67000091 + $pollIndex)) 'adrc status'
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
    Assert-WindowDisabledLcd $serialPort 67000140

    $postReleaseDiscoveryIdle = $false
    for ($pollIndex = 0; $pollIndex -lt 25; ++$pollIndex) {
        $gateReply = Invoke-WindowRequest $serialPort ([uint32](67000200 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $postReleaseDiscoveryIdle = $true
            break
        }
        Start-Sleep -Milliseconds 40
    }
    if (-not $postReleaseDiscoveryIdle) {
        throw 'Selected-axis discovery remained busy after release'
    }
    $restoreReply = Invoke-WindowRequest $serialPort 67000300 'bench mode 7 mode=pos_vel'
    if ($restoreReply -notmatch '^ok 67000300 bench mode accepted=1') {
        throw "Mode 2 restore rejected: $restoreReply"
    }
    [void](Wait-WindowCompletion $serialPort 67000300 'mode')
    $modeRestored = $true
    $gateReply = Invoke-WindowRequest $serialPort 67000301 'adrc gate motor=7'
    if ($gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Mode 2 restore readback failed'
    }
    $disableReply = Invoke-WindowRequest $serialPort 67000302 'bench disable 7'
    if ($disableReply -notmatch '^ok 67000302 bench disable accepted=1') {
        throw "Final DISABLE rejected: $disableReply"
    }
    [void](Wait-WindowCompletion $serialPort 67000302 'disable')
    $finalDisableCompleted = $true
    Assert-WindowDisabledLcd $serialPort 67000303
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
            $ownerReply = Invoke-WindowRequest $serialPort 67000900 'adrc status'
            $ownerReturnedToLcd = $ownerReply -match 'owner=lcd\b'
        }
        catch { $transcriptLines.Add('owner_check_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $mitModeEntered -and $ownerReturnedToLcd -and
        -not $modeRestored) {
        try {
            $restoreReply = Invoke-WindowRequest $serialPort 67000901 'bench mode 7 mode=pos_vel'
            if ($restoreReply -match '^ok 67000901 bench mode accepted=1') {
                [void](Wait-WindowCompletion $serialPort 67000901 'mode')
                $modeRestored = $true
            }
        }
        catch { $transcriptLines.Add('restore_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $mitModeEntered -and $ownerReturnedToLcd -and
        -not $finalDisableCompleted) {
        try {
            $disableReply = Invoke-WindowRequest $serialPort 67000902 'bench disable 7'
            if ($disableReply -match '^ok 67000902 bench disable accepted=1') {
                [void](Wait-WindowCompletion $serialPort 67000902 'disable')
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
