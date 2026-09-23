<#
.SYNOPSIS
Collects motor 7 feedback around one non-motion POS_VEL DISABLE command.

.DESCRIPTION
Requires an explicit user-confirmed 24 V power state. Reads status, performs
fresh parameter discovery, checks the original mode is POS_VEL,
then sends exactly one motor 7 DISABLE. It never switches mode or enables motion.
#>
param(
    [string]$PortName = 'COM4',
    [switch]$Power24vConfirmed
)

$ErrorActionPreference = 'Stop'
$transcriptLines = [System.Collections.Generic.List[string]]::new()
$transcriptLines.Add('host_utc=' + [datetime]::UtcNow.ToString('o') +
    ' port=' + $PortName + ' power24v=on_user_report mode=single_disable_feedback_probe')
$disableCompleted = $false
$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 40
$serialPort.WriteTimeout = 500
$serialPort.NewLine = "`n"

function Invoke-ProbeRequest {
    <# Sends one allowlisted request and records all interleaved replies. #>
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$RequestId,
        [ValidateSet('show state', 'show motor 7', 'show diag can', 'adrc status',
            'adrc gate motor=7', 'adrc discover motor=7',
            'bench disable 7')]
        [string]$Command
    )
    $requestText = "$RequestId $Command"
    $script:transcriptLines.Add('> ' + $requestText)
    $Port.Write($requestText + "`r`n")
    $deadline = [datetime]::UtcNow.AddMilliseconds(1200)
    while ([datetime]::UtcNow -lt $deadline) {
        try { $reply = $Port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        $script:transcriptLines.Add('< ' + $reply)
        if ($reply -match '^done 64000020 bench disable result=completed\b') {
            $script:disableCompleted = $true
        }
        if ($reply -match "^(ok|error) $RequestId ") { return $reply }
    }
    throw "Request timed out: $requestText"
}

try {
    if (-not $Power24vConfirmed) { throw '24 V confirmation is required' }
    if ($PortName -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
        throw "Serial port unavailable: $PortName"
    }
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    $stateBefore = Invoke-ProbeRequest $serialPort 64000000 'show state'
    $ownerBefore = Invoke-ProbeRequest $serialPort 64000001 'adrc status'
    $gateBefore = Invoke-ProbeRequest $serialPort 64000002 'adrc gate motor=7'
    $canBefore = Invoke-ProbeRequest $serialPort 64000003 'show diag can'
    if ($stateBefore -notmatch '^ok 64000000 show state .*enabled=00 moving=0 .*fault=none' -or
        $ownerBefore -notmatch '^ok 64000001 adrc status .*owner=lcd\b' -or
        $gateBefore -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateBefore -notmatch 'discovery_active=0' -or
        $canBefore -notmatch 'busoff=0') {
        throw 'Preflight stopped: motor, ownership, CAN, or discovery is not idle'
    }
    $discoveryReply = Invoke-ProbeRequest $serialPort 64000004 'adrc discover motor=7'
    if ($discoveryReply -notmatch '^ok 64000004 adrc discover=accepted') {
        throw "Discovery not admitted: $discoveryReply"
    }
    $discoveryComplete = $false
    for ($pollIndex = 0; $pollIndex -lt 20; ++$pollIndex) {
        Start-Sleep -Milliseconds 50
        $gateBefore = Invoke-ProbeRequest $serialPort ([uint32](64000005 + $pollIndex)) 'adrc gate motor=7'
        if ($gateBefore -match 'discovery_active=0') {
            $discoveryComplete = $true
            break
        }
    }
    if (-not $discoveryComplete) { throw 'Discovery did not finish in one second' }
    if ($gateBefore -notmatch 'lcd_idle=1 can_idle=1' -or
        $gateBefore -notmatch 'mode=2 fields=1fff verified=40' -or
        $gateBefore -notmatch 'discovery_active=0') {
        throw 'Preflight stopped: motor 7 original mode or discovery is not ready'
    }
    $stateBeforeDisable = Invoke-ProbeRequest $serialPort 64000019 'show state'
    if ($stateBeforeDisable -notmatch 'enabled=00 moving=0 .*fault=none') {
        throw 'Preflight stopped: motor state changed before DISABLE'
    }
    $disableReply = Invoke-ProbeRequest $serialPort 64000020 'bench disable 7'
    if ($disableReply -notmatch '^ok 64000020 bench disable accepted=1') {
        throw "DISABLE not admitted: $disableReply"
    }
    $shortLivedDisabledSeen = $false
    for ($sampleIndex = 0; $sampleIndex -lt 16; ++$sampleIndex) {
        $sample = Invoke-ProbeRequest $serialPort ([uint32](64000030 + $sampleIndex)) 'show motor 7'
        if ($sample -match 'state=disabled .*age_ms=(\d+)') {
            $feedbackAgeMs = [uint32]$Matches[1]
            if ($feedbackAgeMs -lt 150) { $shortLivedDisabledSeen = $true }
        }
        Start-Sleep -Milliseconds 25
    }
    $gateAfter = Invoke-ProbeRequest $serialPort 64000050 'adrc gate motor=7'
    $canAfter = Invoke-ProbeRequest $serialPort 64000052 'show diag can'
    $stateAfter = Invoke-ProbeRequest $serialPort 64000053 'show state'
    $ownerAfter = Invoke-ProbeRequest $serialPort 64000054 'adrc status'
    if ($stateAfter -notmatch 'enabled=00 moving=0 .*fault=none' -or
        $ownerAfter -notmatch 'owner=lcd\b' -or
        $canAfter -notmatch 'busoff=0') {
        throw 'Postflight stopped: state, owner, or CAN changed unexpectedly'
    }
    Write-Output "ADRC_DISABLE_FEEDBACK_PROBE_COMPLETE port=$PortName disable_done=$disableCompleted short_lived_disabled_seen=$shortLivedDisabledSeen"
    Write-Output $gateAfter
    Write-Output $canAfter
}
finally {
    if ($serialPort.IsOpen) { $serialPort.Close() }
    $serialPort.Dispose()
    $outputDirectory = Join-Path $PSScriptRoot '..\..\output\adrc\hardware\disable_feedback_probe_20260923'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $outputName = 'motor7_disable_feedback_' +
        [datetime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '.txt'
    $transcriptLines | Set-Content -LiteralPath (
        Join-Path $outputDirectory $outputName) -Encoding utf8
}
