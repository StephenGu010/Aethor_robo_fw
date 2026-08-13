<#
.SYNOPSIS
Safely checks and optionally moves selected unloaded S3519 motors over USB CDC.
.DESCRIPTION
Uses aethor-arm-ascii-v1 with CRC-16/CCITT-FALSE. The default path never enables
or moves motors; pass -RunMotion explicitly to execute the bounded motion stage.
#>

[CmdletBinding()]
param(
    [ValidatePattern('^COM\d+$')]
    [string]$PortName = 'COM7',
    [string]$MotorList = '1,3',
    [switch]$RunMotion,
    [ValidateRange(0.0001, 3.0)]
    [double]$DeltaDegrees = 0.2,
    [ValidateRange(0.0001, 3.0)]
    [double]$SpeedDegreesPerSecond = 1.0,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture

function Get-Crc16CcittFalse {
    <# Calculates CRC-16/CCITT-FALSE for one exact byte sequence. #>
    param([Parameter(Mandatory)][byte[]]$Data)

    [uint16]$crc = 0xFFFF
    foreach ($dataByte in $Data) {
        $crc = [uint16]($crc -bxor ([uint16]$dataByte -shl 8))
        for ($bitIndex = 0; $bitIndex -lt 8; $bitIndex++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = [uint16]((([uint32]$crc -shl 1) -bxor 0x1021) -band 0xFFFF)
            }
            else {
                $crc = [uint16](([uint32]$crc -shl 1) -band 0xFFFF)
            }
        }
    }
    return $crc
}

function ConvertTo-AethorFrame {
    <# Encodes one request body with an uppercase CRC and CRLF terminator. #>
    param([Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Body)

    $bodyBytes = [System.Text.Encoding]::ASCII.GetBytes($Body)
    $crc = Get-Crc16CcittFalse -Data $bodyBytes
    return "{0} *{1:X4}`r`n" -f $Body, $crc
}

function ConvertFrom-AethorFrame {
    <# Validates one received CRC frame and returns its body metadata. #>
    param([Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Line)

    $content = $Line.TrimEnd("`r", "`n")
    $separatorIndex = $content.LastIndexOf(' *')
    if (($separatorIndex -le 0) -or (($content.Length - $separatorIndex) -ne 6)) {
        throw "BAD_FRAME: $content"
    }
    $body = $content.Substring(0, $separatorIndex)
    $receivedCrcText = $content.Substring($separatorIndex + 2)
    [uint16]$receivedCrc = 0
    if (-not [uint16]::TryParse(
            $receivedCrcText,
            [System.Globalization.NumberStyles]::HexNumber,
            $invariantCulture,
            [ref]$receivedCrc)) {
        throw "BAD_FRAME_CRC_TEXT: $content"
    }
    $calculatedCrc = Get-Crc16CcittFalse `
        -Data ([System.Text.Encoding]::ASCII.GetBytes($body))
    if ($calculatedCrc -ne $receivedCrc) {
        throw ('BAD_CRC expected={0:X4} received={1:X4} body={2}' -f `
            $calculatedCrc, $receivedCrc, $body)
    }
    $tokens = $body.Split(' ')
    [uint32]$requestId = 0
    if (($tokens.Count -lt 2) -or
        (-not [uint32]::TryParse($tokens[1], [ref]$requestId))) {
        throw "BAD_RESPONSE_HEADER: $body"
    }
    return [pscustomobject]@{
        Body = $body
        Kind = $tokens[0]
        RequestId = $requestId
        Result = if ($tokens.Count -ge 3) { $tokens[2] } else { '' }
    }
}

function Get-MotorSelection {
    <# Validates a unique ascending 1..7 motor list and builds motion value lists. #>
    param(
        [Parameter(Mandatory)][string]$Text,
        [Parameter(Mandatory)][double]$Magnitude
    )

    $tokens = $Text.Split(',')
    if ($tokens.Count -eq 0) {
        throw 'MotorList must not be empty.'
    }
    $motorNumbers = New-Object System.Collections.Generic.List[int]
    $previousMotor = 0
    foreach ($token in $tokens) {
        [int]$motorNumber = 0
        if ((-not [int]::TryParse($token, [ref]$motorNumber)) -or
            ($motorNumber -lt 1) -or ($motorNumber -gt 7) -or
            ($motorNumber -le $previousMotor)) {
            throw 'MotorList must contain unique ascending motor numbers in 1..7.'
        }
        $motorNumbers.Add($motorNumber)
        $previousMotor = $motorNumber
    }
    $forwardValues = for ($index = 0; $index -lt $motorNumbers.Count; $index++) {
        $signedValue = if (($index % 2) -eq 0) { $Magnitude } else { -$Magnitude }
        $signedValue.ToString('0.0###', $invariantCulture)
    }
    $reverseValues = foreach ($value in $forwardValues) {
        (-[double]::Parse($value, $invariantCulture)).ToString('0.0###', $invariantCulture)
    }
    return [pscustomobject]@{
        Canonical = ($motorNumbers -join ',')
        Count = $motorNumbers.Count
        ForwardValues = ($forwardValues -join ',')
        ReverseValues = ($reverseValues -join ',')
    }
}

function New-AethorContext {
    <# Creates mutable request/session state around one optional serial port. #>
    param($SerialPort)

    return [pscustomobject]@{
        SerialPort = $SerialPort
        NextRequestId = [uint32]1
        SessionId = [uint32]0
        LastHeartbeatUtc = [datetime]::MinValue
        HeartbeatRequestIds = New-Object 'System.Collections.Generic.HashSet[uint32]'
    }
}

function Write-AethorBody {
    <# Sends one encoded request body and writes a timestamped transcript line. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Body
    )

    $frame = ConvertTo-AethorFrame -Body $Body
    Write-Host ('[{0:HH:mm:ss.fff}] > {1}' -f [datetime]::Now, $Body)
    $Context.SerialPort.Write($frame)
}

function Send-AethorRequest {
    <# Allocates one request ID, sends the canonical body, and returns the ID. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Operation,
        [string]$Fields = ''
    )

    $requestId = $Context.NextRequestId
    $Context.NextRequestId = [uint32]($Context.NextRequestId + 1)
    $body = "REQ $requestId $Operation"
    if ($Fields.Length -gt 0) {
        $body += " $Fields"
    }
    Write-AethorBody -Context $Context -Body $body
    return $requestId
}

function Send-AethorHeartbeatIfDue {
    <# Sends one session heartbeat when at least 200 ms have elapsed. #>
    param([Parameter(Mandatory)]$Context)

    if ($Context.SessionId -eq 0) {
        return
    }
    $now = [datetime]::UtcNow
    if (($now - $Context.LastHeartbeatUtc).TotalMilliseconds -lt 200) {
        return
    }
    $heartbeatId = Send-AethorRequest -Context $Context -Operation 'HEARTBEAT' `
        -Fields "session=$($Context.SessionId)"
    [void]$Context.HeartbeatRequestIds.Add([uint32]$heartbeatId)
    $Context.LastHeartbeatUtc = $now
}

function Read-AethorResponse {
    <# Reads and validates one response, returning null on the short read timeout. #>
    param([Parameter(Mandatory)]$Context)

    try {
        $line = $Context.SerialPort.ReadLine()
    }
    catch [System.TimeoutException] {
        return $null
    }
    $response = ConvertFrom-AethorFrame -Line $line
    Write-Host ('[{0:HH:mm:ss.fff}] < {1}' -f [datetime]::Now, $response.Body)
    return $response
}

function Wait-AethorRequest {
    <# Waits for a matching query or action terminal while servicing heartbeats. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][uint32]$RequestId,
        [Parameter(Mandatory)][ValidateSet('Query', 'Action')][string]$RequestType,
        [Parameter(Mandatory)][int]$TimeoutMilliseconds
    )

    $deadline = [datetime]::UtcNow.AddMilliseconds($TimeoutMilliseconds)
    $acknowledged = $false
    while ([datetime]::UtcNow -lt $deadline) {
        if ($RequestType -eq 'Action') {
            Send-AethorHeartbeatIfDue -Context $Context
        }
        $response = Read-AethorResponse -Context $Context
        if ($null -eq $response) {
            continue
        }
        if ($Context.HeartbeatRequestIds.Remove([uint32]$response.RequestId)) {
            continue
        }
        if ($response.RequestId -ne $RequestId) {
            continue
        }
        if ($response.Kind -eq 'ERR') {
            throw "DEVICE_ERROR: $($response.Body)"
        }
        if (($RequestType -eq 'Query') -and ($response.Kind -eq 'RSP')) {
            return $response
        }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'ACK')) {
            $acknowledged = $true
            continue
        }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'DONE')) {
            if (-not $acknowledged) {
                throw "DONE_WITHOUT_ACK: $($response.Body)"
            }
            if (($response.Result -ne 'COMPLETED') -and
                ($response.Result -ne 'STOPPED')) {
                throw "ACTION_FAILED: $($response.Body)"
            }
            return $response
        }
    }
    throw "REQUEST_TIMEOUT id=$RequestId type=$RequestType timeout_ms=$TimeoutMilliseconds"
}

function Invoke-AethorQuery {
    <# Sends one query and returns its matching validated RSP. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Operation,
        [string]$Fields = ''
    )

    $requestId = Send-AethorRequest -Context $Context -Operation $Operation -Fields $Fields
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Query -TimeoutMilliseconds 3000
}

function Invoke-AethorAction {
    <# Sends one action and waits for its ACK plus successful DONE. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Operation,
        [Parameter(Mandatory)][string]$Fields,
        [Parameter(Mandatory)][int]$TimeoutMilliseconds
    )

    $requestId = Send-AethorRequest -Context $Context -Operation $Operation -Fields $Fields
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Action -TimeoutMilliseconds $TimeoutMilliseconds
}

function Invoke-ScriptSelfTest {
    <# Verifies CRC vectors, frame rejection, selection mapping, and safe defaults. #>

    $testContext = New-AethorContext -SerialPort $null
    if (($testContext.NextRequestId -ne 1) -or ($testContext.SessionId -ne 0)) {
        throw 'CONTEXT_DEFAULTS_FAILED'
    }

    $referenceCrc = Get-Crc16CcittFalse `
        -Data ([System.Text.Encoding]::ASCII.GetBytes('123456789'))
    if ($referenceCrc -ne 0x29B1) {
        throw ('CRC_REFERENCE_FAILED: {0:X4}' -f $referenceCrc)
    }

    $formattedFrame = ConvertTo-AethorFrame -Body 'REQ 42 GET_JPOS'
    if ($formattedFrame -ne "REQ 42 GET_JPOS *6B48`r`n") {
        throw "FRAME_FORMAT_FAILED: $formattedFrame"
    }

    $validResponse = ConvertTo-AethorFrame -Body 'RSP 42 ok'
    $parsedResponse = ConvertFrom-AethorFrame -Line $validResponse
    if (($parsedResponse.Kind -ne 'RSP') -or ($parsedResponse.RequestId -ne 42)) {
        throw 'FRAME_PARSE_FAILED'
    }

    $badCrcRejected = $false
    try {
        ConvertFrom-AethorFrame -Line 'RSP 42 ok *0000' | Out-Null
    }
    catch {
        $badCrcRejected = $true
    }
    if (-not $badCrcRejected) {
        throw 'BAD_CRC_NOT_REJECTED'
    }

    $selection = Get-MotorSelection -Text '1,3' -Magnitude 0.2
    if (($selection.Canonical -ne '1,3') -or
        ($selection.ForwardValues -ne '0.2,-0.2') -or
        ($selection.ReverseValues -ne '-0.2,0.2')) {
        throw 'MOTOR_SELECTION_FAILED'
    }

    $invalidSelectionRejected = $false
    try {
        Get-MotorSelection -Text '3,1' -Magnitude 0.2 | Out-Null
    }
    catch {
        $invalidSelectionRejected = $true
    }
    if (-not $invalidSelectionRejected) {
        throw 'UNSORTED_MOTORS_NOT_REJECTED'
    }

    Write-Output ('SELF_TESTS_PASSED crc={0:X4} motors={1} run_motion_default={2}' -f `
        $referenceCrc, $selection.Canonical, [bool]$RunMotion)
}

if ($SelfTest) {
    Invoke-ScriptSelfTest
    exit 0
}

function Open-AethorSerialPort {
    <# Opens one exclusive ASCII USB CDC port with bounded read/write timeouts. #>
    param([Parameter(Mandatory)][string]$Name)

    $serialPort = [System.IO.Ports.SerialPort]::new(
        $Name,
        115200,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One)
    $serialPort.Encoding = [System.Text.Encoding]::ASCII
    $serialPort.NewLine = "`n"
    $serialPort.ReadTimeout = 50
    $serialPort.WriteTimeout = 500
    try {
        $serialPort.Open()
    }
    catch [System.UnauthorizedAccessException] {
        $serialPort.Dispose()
        throw "$Name is occupied. Close UartAssist's serial connection and retry."
    }
    $serialPort.DiscardInBuffer()
    $serialPort.DiscardOutBuffer()
    return $serialPort
}

function Initialize-AethorSession {
    <# Establishes HELLO and stores the runtime session identifier. #>
    param([Parameter(Mandatory)]$Context)

    $hello = Invoke-AethorQuery -Context $Context -Operation 'HELLO' `
        -Fields 'client=com7-dual-debug protocol=1'
    $sessionMatch = [regex]::Match($hello.Body, '(?:^| )session=(\d+)(?: |$)')
    if (-not $sessionMatch.Success) {
        throw "HELLO_SESSION_MISSING: $($hello.Body)"
    }
    $Context.SessionId = [uint32]::Parse($sessionMatch.Groups[1].Value)
    $Context.LastHeartbeatUtc = [datetime]::UtcNow
}

function Invoke-ReadOnlyStage {
    <# Runs handshake, queries, and selected-motor discovery without enabling motion. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)]$Selection
    )

    Initialize-AethorSession -Context $Context
    Invoke-AethorQuery -Context $Context -Operation 'SET_STREAM' `
        -Fields 'rate_hz=0 fields=jpos,jvel,state,motor' | Out-Null
    foreach ($queryOperation in @(
            'GET_INFO',
            'GET_CONFIG',
            'GET_STATE',
            'GET_MOTORS',
            'GET_DIAG')) {
        Invoke-AethorQuery -Context $Context -Operation $queryOperation | Out-Null
    }
    Invoke-AethorAction -Context $Context -Operation 'INIT_MOTORS' `
        -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 30000 | Out-Null
    Invoke-AethorQuery -Context $Context -Operation 'GET_MOTORS' | Out-Null
}

function Invoke-MotionStage {
    <# Clears, enables, moves out/back, stops, disables, and reads final diagnostics. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)]$Selection,
        [Parameter(Mandatory)][double]$Speed
    )

    $speedText = $Speed.ToString('0.0###', $invariantCulture)
    $speedValues = (@($speedText) * $Selection.Count) -join ','
    Invoke-AethorAction -Context $Context -Operation 'CLEAR_FAULT' `
        -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'ENABLE' `
        -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'MOVE_REL' `
        -Fields "motors=$($Selection.Canonical) delta_deg=$($Selection.ForwardValues) speed_deg_s=$speedValues" `
        -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'MOVE_REL' `
        -Fields "motors=$($Selection.Canonical) delta_deg=$($Selection.ReverseValues) speed_deg_s=$speedValues" `
        -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'STOP' `
        -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'DISABLE' `
        -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorQuery -Context $Context -Operation 'GET_MOTORS' | Out-Null
    Invoke-AethorQuery -Context $Context -Operation 'GET_DIAG' | Out-Null
}

function Invoke-BestEffortShutdown {
    <# Tries STOP then DISABLE without hiding the original failure. #>
    param(
        $Context,
        [Parameter(Mandatory)]$Selection
    )

    if (($null -eq $Context) -or ($Context.SessionId -eq 0) -or
        ($null -eq $Context.SerialPort) -or (-not $Context.SerialPort.IsOpen)) {
        return
    }
    foreach ($operation in @('STOP', 'DISABLE')) {
        try {
            Invoke-AethorAction -Context $Context -Operation $operation `
                -Fields "motors=$($Selection.Canonical)" -TimeoutMilliseconds 3000 | Out-Null
        }
        catch {
            Write-Warning "Best-effort $operation failed: $($_.Exception.Message)"
        }
    }
}

$selection = Get-MotorSelection -Text $MotorList -Magnitude $DeltaDegrees
$serialPort = $null
$context = $null
$shutdownRequired = $false
try {
    $serialPort = Open-AethorSerialPort -Name $PortName
    $context = New-AethorContext -SerialPort $serialPort
    Invoke-ReadOnlyStage -Context $context -Selection $selection
    if (-not $RunMotion) {
        Write-Output "READ_ONLY_DEBUG_PASSED port=$PortName motors=$($selection.Canonical)"
        exit 0
    }
    $shutdownRequired = $true
    Invoke-MotionStage -Context $context -Selection $selection `
        -Speed $SpeedDegreesPerSecond
    $shutdownRequired = $false
    Write-Output "MOTION_DEBUG_PASSED port=$PortName motors=$($selection.Canonical)"
}
catch {
    Write-Error "COM7_DUAL_MOTOR_DEBUG_FAILED: $($_.Exception.Message)"
    exit 1
}
finally {
    if ($shutdownRequired) {
        Invoke-BestEffortShutdown -Context $context -Selection $selection
    }
    if ($null -ne $serialPort) {
        if ($serialPort.IsOpen) {
            $serialPort.Close()
        }
        $serialPort.Dispose()
    }
}
