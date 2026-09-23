<#
.SYNOPSIS
Checks disabled motor-7 feedback in MIT mode during one 24 V power window.

.DESCRIPTION
Requires a user-confirmed 24 V supply and the LCD-MIT-ADRC firmware with
`adrc probe`. It discovers parameters, changes only the volatile control mode,
sends one feedback query, restores POS_VEL, and disables motor 7. It never
sends ENABLE, MIT torque, ADRC acquire, or motion commands. A transcript is
saved even if the serial link or a safety gate fails.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed
)

$ErrorActionPreference = 'Stop'
$transcriptLines = [System.Collections.Generic.List[string]]::new()
$modeSwitchAttempted = $false
$modeRestored = $false
$finalDisableCompleted = $false
$completedReplies = @{}
$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 40
$serialPort.WriteTimeout = 500
$serialPort.NewLine = "`n"
$transcriptLines.Add('host_utc=' + [datetime]::UtcNow.ToString('o') +
    ' port=' + $PortName + ' power24v=on_user_report mode=mit_feedback_window')

function Receive-ReplyLine {
    <# Records one serial line and returns null for a bounded read timeout. #>
    param([System.IO.Ports.SerialPort]$Port)
    try { $replyLine = $Port.ReadLine().Trim() }
    catch [System.TimeoutException] { return $null }
    $script:transcriptLines.Add('< ' + $replyLine)
    if ($replyLine -match '^done (\d+) bench (mode|disable) result=') {
        $script:completedReplies[$Matches[1] + ':' + $Matches[2]] = $replyLine
    }
    return $replyLine
}

function Invoke-BenchRequest {
    <# Sends exactly one allowlisted command and waits for its matching reply. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('show state', 'show motor 7', 'show diag can',
            'adrc status', 'adrc gate motor=7', 'adrc discover motor=7',
            'adrc probe motor=7', 'adrc probe', 'bench mode 7 mode=mit',
            'bench mode 7 mode=pos_vel', 'bench disable 7')]
        [string]$Command
    )
    $requestText = "$RequestId $Command"
    $script:transcriptLines.Add('> ' + $requestText)
    $Port.Write($requestText + "`r`n")
    $deadline = [datetime]::UtcNow.AddMilliseconds(1500)
    while ([datetime]::UtcNow -lt $deadline) {
        $replyLine = Receive-ReplyLine $Port
        if ($null -ne $replyLine -and $replyLine -match "^(ok|error) $RequestId ") {
            return $replyLine
        }
    }
    throw "Request timed out: $requestText"
}

function Wait-BenchCompletion {
    <# Requires a completed asynchronous result for a mode or disable command. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('mode', 'disable')][string]$Operation
    )
    $completionKey = "$RequestId`:$Operation"
    $deadline = [datetime]::UtcNow.AddSeconds(4)
    while ([datetime]::UtcNow -lt $deadline) {
        if ($script:completedReplies.ContainsKey($completionKey)) {
            $completionReply = $script:completedReplies[$completionKey]
            if ($completionReply -match "^done $RequestId bench $Operation result=completed\b") {
                return $completionReply
            }
            throw "Bench operation failed: $completionReply"
        }
        $replyLine = Receive-ReplyLine $Port
    }
    throw "Bench completion timed out: $RequestId bench $Operation"
}

function Assert-DisabledLcdState {
    <# Rejects ownership, active motion, fault, or CAN bus-off before each phase. #>
    param([System.IO.Ports.SerialPort]$Port, [uint32]$RequestId)
    $armReply = Invoke-BenchRequest $Port $RequestId 'show state'
    $ownerReply = Invoke-BenchRequest $Port ([uint32]($RequestId + 1)) 'adrc status'
    $canReply = Invoke-BenchRequest $Port ([uint32]($RequestId + 2)) 'show diag can'
    if ($armReply -notmatch 'enabled=00 moving=0 .*fault=none' -or
        $ownerReply -notmatch 'owner=lcd\b' -or
        $canReply -notmatch 'busoff=0') {
        throw 'Safety state changed: expected disabled, LCD owner, and no CAN bus-off'
    }
}

try {
    if (-not $Power24vConfirmed) { throw 'User confirmation of 24 V ON is required' }
    if ($PortName -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
        throw "Serial port unavailable: $PortName"
    }
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    Assert-DisabledLcdState $serialPort 65000000
    $gateReply = Invoke-BenchRequest $serialPort 65000003 'adrc gate motor=7'
    if ($gateReply -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateReply -notmatch 'discovery_active=0') {
        throw 'Preflight stopped: LCD, CAN, or discovery is busy'
    }
    $discoveryReply = Invoke-BenchRequest $serialPort 65000004 'adrc discover motor=7'
    if ($discoveryReply -notmatch '^ok 65000004 adrc discover=accepted') {
        throw "Discovery rejected: $discoveryReply"
    }
    $discoveryComplete = $false
    for ($pollIndex = 0; $pollIndex -lt 20; ++$pollIndex) {
        Start-Sleep -Milliseconds 50
        $gateReply = Invoke-BenchRequest $serialPort ([uint32](65000005 + $pollIndex)) 'adrc gate motor=7'
        if ($gateReply -match 'discovery_active=0') {
            $discoveryComplete = $true
            break
        }
    }
    if (-not $discoveryComplete -or
        $gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'Original POS_VEL mode or full parameter discovery was not verified'
    }
    Assert-DisabledLcdState $serialPort 65000030

    $modeSwitchAttempted = $true
    $modeReply = Invoke-BenchRequest $serialPort 65000040 'bench mode 7 mode=mit'
    if ($modeReply -notmatch '^ok 65000040 bench mode accepted=1') {
        throw "MIT mode switch rejected: $modeReply"
    }
    [void](Wait-BenchCompletion $serialPort 65000040 'mode')
    $gateReply = Invoke-BenchRequest $serialPort 65000041 'adrc gate motor=7'
    if ($gateReply -notmatch 'mode=1 fields=1fff verified=40' -or
        $gateReply -notmatch 'lcd_idle=1 can_idle=1') {
        throw 'MIT mode readback or idle gate failed'
    }
    Assert-DisabledLcdState $serialPort 65000042
    $probeReply = Invoke-BenchRequest $serialPort 65000045 'adrc probe motor=7'
    if ($probeReply -notmatch '^ok 65000045 adrc probe=accepted') {
        throw "Diagnostic query rejected: $probeReply"
    }
    $probeStates = [System.Collections.Generic.List[string]]::new()
    for ($sampleIndex = 0; $sampleIndex -lt 12; ++$sampleIndex) {
        $probeStatus = Invoke-BenchRequest $serialPort ([uint32](65000046 + $sampleIndex)) 'adrc probe'
        $motorStatus = Invoke-BenchRequest $serialPort ([uint32](65000058 + $sampleIndex)) 'show motor 7'
        $probeStates.Add($probeStatus)
        if ($probeStatus -match 'state=(feedback|timeout|failed)\b') { break }
        Start-Sleep -Milliseconds 20
    }
    Assert-DisabledLcdState $serialPort 65000070

    $restoreReply = Invoke-BenchRequest $serialPort 65000080 'bench mode 7 mode=pos_vel'
    if ($restoreReply -notmatch '^ok 65000080 bench mode accepted=1') {
        throw "POS_VEL restore rejected: $restoreReply"
    }
    [void](Wait-BenchCompletion $serialPort 65000080 'mode')
    $modeRestored = $true
    $gateReply = Invoke-BenchRequest $serialPort 65000081 'adrc gate motor=7'
    if ($gateReply -notmatch 'mode=2 fields=1fff verified=40') {
        throw 'POS_VEL mode restore readback failed'
    }
    $disableReply = Invoke-BenchRequest $serialPort 65000082 'bench disable 7'
    if ($disableReply -notmatch '^ok 65000082 bench disable accepted=1') {
        throw "Final DISABLE rejected: $disableReply"
    }
    [void](Wait-BenchCompletion $serialPort 65000082 'disable')
    $finalDisableCompleted = $true
    Assert-DisabledLcdState $serialPort 65000083
    $probeSummary = if ($probeStates.Count -gt 0) { $probeStates[-1] } else { 'none' }
    Write-Output "ADRC_MODE_FEEDBACK_WINDOW_COMPLETE port=$PortName mode_restored=$modeRestored"
    Write-Output $probeSummary
    Write-Output $gateReply
}
catch {
    $transcriptLines.Add('host_error=' + $_.Exception.Message)
    Write-Error -ErrorAction Continue $_
    throw
}
finally {
    if ($serialPort.IsOpen -and $modeSwitchAttempted -and -not $modeRestored) {
        try {
            $restoreReply = Invoke-BenchRequest $serialPort 65000090 'bench mode 7 mode=pos_vel'
            if ($restoreReply -match '^ok 65000090 bench mode accepted=1') {
                [void](Wait-BenchCompletion $serialPort 65000090 'mode')
                $modeRestored = $true
            }
        }
        catch { $transcriptLines.Add('restore_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen -and $modeSwitchAttempted -and -not $finalDisableCompleted) {
        try {
            $disableReply = Invoke-BenchRequest $serialPort 65000091 'bench disable 7'
            if ($disableReply -match '^ok 65000091 bench disable accepted=1') {
                [void](Wait-BenchCompletion $serialPort 65000091 'disable')
            }
        }
        catch { $transcriptLines.Add('disable_error=' + $_.Exception.Message) }
    }
    if ($serialPort.IsOpen) { $serialPort.Close() }
    $serialPort.Dispose()
    $outputDirectory = Join-Path $PSScriptRoot '..\..\output\adrc\hardware\mode_feedback_probe_20260923'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $outputName = 'motor7_mode_feedback_' +
        [datetime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '.txt'
    $transcriptLines.Add('mode_restored=' + $modeRestored)
    $transcriptLines.Add('host_end_utc=' + [datetime]::UtcNow.ToString('o'))
    $transcriptLines | Set-Content -LiteralPath (
        Join-Path $outputDirectory $outputName) -Encoding utf8
}
