<#
.SYNOPSIS
Runs a guarded three-stage one-turn output-shaft test on S3519 motors 1 and 3.
.DESCRIPTION
Uses aethor-arm-ascii-v1 over COM7. Each output turn is split into 120 commands
of 3 degrees. Physical motion requires the explicit -RunMotion switch.
#>

[CmdletBinding()]
param(
    [ValidatePattern('^COM\d+$')]
    [string]$PortName = 'COM7',
    [switch]$RunMotion,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture
$stepDegrees = 3.0
$speedDegreesPerSecond = 3.0
$stepsPerTurn = 120

function Get-OneTurnStages {
    <# Returns the immutable, PMAX-safe three-stage output-turn sequence. #>

    return @(
        [pscustomobject]@{ Name = 'ID1'; Motors = '1'; Delta = 3.0; Steps = 120 },
        [pscustomobject]@{ Name = 'ID3'; Motors = '3'; Delta = 3.0; Steps = 120 },
        [pscustomobject]@{ Name = 'ID1_ID3'; Motors = '1,3'; Delta = -3.0; Steps = 120 }
    )
}

function Get-RepeatedValueList {
    <# Formats one numeric value once for every motor in an ascending motor list. #>
    param(
        [Parameter(Mandatory)][double]$Value,
        [Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$MotorList
    )

    $motorCount = $MotorList.Split(',').Count
    $formattedValue = $Value.ToString('0.0###', $invariantCulture)
    return (@($formattedValue) * $motorCount) -join ','
}

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
    <# Validates one received CRC frame and returns its parsed response metadata. #>
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

function Open-AethorSerialPort {
    <# Opens one exclusive ASCII USB CDC port with bounded read and write timeouts. #>
    param([Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Name)

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
        throw "$Name is occupied. Close every other serial monitor and retry."
    }
    $serialPort.DiscardInBuffer()
    $serialPort.DiscardOutBuffer()
    return $serialPort
}

function New-AethorContext {
    <# Creates mutable request and heartbeat state for one serial connection. #>
    param([Parameter(Mandatory)]$SerialPort)

    $invocationToken = [guid]::NewGuid().ToString('N').Substring(0, 8)
    [uint32]$initialRequestId = [Convert]::ToUInt32($invocationToken, 16)
    if ($initialRequestId -eq 0) {
        $initialRequestId = 1
    }
    return [pscustomobject]@{
        SerialPort = $SerialPort
        ClientName = "com7-one-turn-$invocationToken"
        NextRequestId = $initialRequestId
        SessionId = [uint32]0
        LastHeartbeatUtc = [datetime]::MinValue
        HeartbeatRequestIds = New-Object 'System.Collections.Generic.HashSet[uint32]'
    }
}

function Write-AethorBody {
    <# Sends one encoded request and optionally suppresses heartbeat transcript noise. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Body,
        [switch]$Quiet
    )

    if (-not $Quiet) {
        Write-Host ('[{0:HH:mm:ss.fff}] > {1}' -f [datetime]::Now, $Body)
    }
    $Context.SerialPort.Write((ConvertTo-AethorFrame -Body $Body))
}

function Send-AethorRequest {
    <# Allocates one request ID, sends a canonical request body, and returns its ID. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Operation,
        [string]$Fields = '',
        [switch]$Quiet
    )

    $requestId = $Context.NextRequestId
    $Context.NextRequestId = [uint32]($Context.NextRequestId + 1)
    $body = "REQ $requestId $Operation"
    if ($Fields.Length -gt 0) {
        $body += " $Fields"
    }
    Write-AethorBody -Context $Context -Body $body -Quiet:$Quiet
    return $requestId
}

function Send-AethorHeartbeatIfDue {
    <# Sends one quiet session heartbeat after at least 200 milliseconds. #>
    param([Parameter(Mandatory)]$Context)

    if ($Context.SessionId -eq 0) {
        return
    }
    $now = [datetime]::UtcNow
    if (($now - $Context.LastHeartbeatUtc).TotalMilliseconds -lt 200) {
        return
    }
    $heartbeatId = Send-AethorRequest -Context $Context -Operation 'HEARTBEAT' `
        -Fields "session=$($Context.SessionId)" -Quiet
    [void]$Context.HeartbeatRequestIds.Add([uint32]$heartbeatId)
    $Context.LastHeartbeatUtc = $now
}

function Read-AethorResponse {
    <# Reads and validates one response, returning null for the short serial timeout. #>
    param([Parameter(Mandatory)]$Context)

    try {
        $line = $Context.SerialPort.ReadLine()
    }
    catch [System.TimeoutException] {
        return $null
    }
    $response = ConvertFrom-AethorFrame -Line $line
    if (-not $Context.HeartbeatRequestIds.Contains([uint32]$response.RequestId)) {
        Write-Host ('[{0:HH:mm:ss.fff}] < {1}' -f [datetime]::Now, $response.Body)
    }
    return $response
}

function Wait-AethorRequest {
    <# Waits for one matching query or action terminal while servicing heartbeats. #>
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
    <# Sends one query and waits for its CRC-validated RSP. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Operation,
        [string]$Fields = ''
    )

    $requestId = Send-AethorRequest -Context $Context -Operation $Operation -Fields $Fields
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Query -TimeoutMilliseconds 3000
}

function Invoke-AethorAction {
    <# Sends one action and waits for its ACK followed by successful DONE. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Operation,
        [Parameter(Mandatory)][string]$Fields,
        [Parameter(Mandatory)][int]$TimeoutMilliseconds,
        [switch]$Quiet
    )

    $requestId = Send-AethorRequest -Context $Context -Operation $Operation `
        -Fields $Fields -Quiet:$Quiet
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Action -TimeoutMilliseconds $TimeoutMilliseconds
}

function Initialize-AethorSession {
    <# Establishes HELLO and stores the device-issued runtime session identifier. #>
    param([Parameter(Mandatory)]$Context)

    $hello = Invoke-AethorQuery -Context $Context -Operation 'HELLO' `
        -Fields "client=$($Context.ClientName) protocol=1"
    $sessionMatch = [regex]::Match($hello.Body, '(?:^| )session=(\d+)(?: |$)')
    if (-not $sessionMatch.Success) {
        throw "HELLO_SESSION_MISSING: $($hello.Body)"
    }
    $Context.SessionId = [uint32]::Parse($sessionMatch.Groups[1].Value)
    $Context.LastHeartbeatUtc = [datetime]::UtcNow
}

function Invoke-OneTurnStage {
    <# Initializes, enables, incrementally moves, stops, and disables one stage. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)]$Stage
    )

    $script:activeMotorList = $Stage.Motors
    $script:shutdownRequired = $true
    $deltaValues = Get-RepeatedValueList -Value $Stage.Delta -MotorList $Stage.Motors
    $speedValues = Get-RepeatedValueList `
        -Value $speedDegreesPerSecond -MotorList $Stage.Motors

    Write-Output ("STAGE_START name={0} motors={1} delta_deg={2} steps={3}" -f `
        $Stage.Name, $Stage.Motors, $Stage.Delta, $Stage.Steps)
    Invoke-AethorAction -Context $Context -Operation 'INIT_MOTORS' `
        -Fields "motors=$($Stage.Motors)" -TimeoutMilliseconds 30000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'CLEAR_FAULT' `
        -Fields "motors=$($Stage.Motors)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'ENABLE' `
        -Fields "motors=$($Stage.Motors)" -TimeoutMilliseconds 10000 | Out-Null

    for ($stepIndex = 1; $stepIndex -le $Stage.Steps; $stepIndex++) {
        Invoke-AethorAction -Context $Context -Operation 'MOVE_REL' `
            -Fields "motors=$($Stage.Motors) delta_deg=$deltaValues speed_deg_s=$speedValues" `
            -TimeoutMilliseconds 10000 -Quiet | Out-Null
        if (($stepIndex -eq 1) -or (($stepIndex % 10) -eq 0) -or
            ($stepIndex -eq $Stage.Steps)) {
            Write-Output ("STAGE_PROGRESS name={0} step={1}/{2} cumulative_deg={3}" -f `
                $Stage.Name,
                $stepIndex,
                $Stage.Steps,
                ($stepIndex * $Stage.Delta))
        }
    }

    Invoke-AethorAction -Context $Context -Operation 'STOP' `
        -Fields "motors=$($Stage.Motors)" -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorAction -Context $Context -Operation 'DISABLE' `
        -Fields "motors=$($Stage.Motors)" -TimeoutMilliseconds 10000 | Out-Null
    $script:shutdownRequired = $false

    $motorResponse = Invoke-AethorQuery -Context $Context -Operation 'GET_MOTORS'
    $diagnosticResponse = Invoke-AethorQuery -Context $Context -Operation 'GET_DIAG'
    Write-Output ("STAGE_FINAL_MOTORS name={0} {1}" -f $Stage.Name, $motorResponse.Body)
    Write-Output ("STAGE_FINAL_DIAG name={0} {1}" -f $Stage.Name, $diagnosticResponse.Body)
    Write-Output ("STAGE_COMPLETED name={0} motors={1} total_deg={2}" -f `
        $Stage.Name, $Stage.Motors, ($Stage.Steps * $Stage.Delta))
}

function Invoke-OneTurnSequence {
    <# Runs the initial read-only checks and all approved stages in strict order. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][object[]]$Stages
    )

    Initialize-AethorSession -Context $Context
    Invoke-AethorQuery -Context $Context -Operation 'SET_STREAM' `
        -Fields 'rate_hz=0 fields=jpos,jvel,state,motor' | Out-Null
    foreach ($queryOperation in @('GET_INFO', 'GET_STATE', 'GET_MOTORS', 'GET_DIAG')) {
        Invoke-AethorQuery -Context $Context -Operation $queryOperation | Out-Null
    }
    foreach ($stage in $Stages) {
        Invoke-OneTurnStage -Context $Context -Stage $stage
    }
}

function Invoke-BestEffortShutdown {
    <# Tries STOP then DISABLE for the active stage without hiding the first failure. #>
    param(
        $Context,
        [string]$MotorList
    )

    if (($null -eq $Context) -or ($Context.SessionId -eq 0) -or
        [string]::IsNullOrWhiteSpace($MotorList) -or
        ($null -eq $Context.SerialPort) -or (-not $Context.SerialPort.IsOpen)) {
        return
    }
    foreach ($operation in @('STOP', 'DISABLE')) {
        try {
            Invoke-AethorAction -Context $Context -Operation $operation `
                -Fields "motors=$MotorList" -TimeoutMilliseconds 3000 | Out-Null
            Write-Warning "Best-effort $operation completed for motors=$MotorList"
        }
        catch {
            Write-Warning "Best-effort $operation failed: $($_.Exception.Message)"
        }
    }
}

function Invoke-OneTurnSelfTest {
    <# Verifies CRC, frame parsing, stage totals, selection widths, and safe defaults. #>

    $stages = @(Get-OneTurnStages)
    if ($stages.Count -ne 3) {
        throw "STAGE_COUNT_FAILED actual=$($stages.Count)"
    }
    $expectedNames = @('ID1', 'ID3', 'ID1_ID3')
    $expectedMotors = @('1', '3', '1,3')
    $expectedTotals = @(360.0, 360.0, -360.0)
    for ($stageIndex = 0; $stageIndex -lt $stages.Count; $stageIndex++) {
        $stage = $stages[$stageIndex]
        $actualTotal = $stage.Delta * $stage.Steps
        if (($stage.Name -ne $expectedNames[$stageIndex]) -or
            ($stage.Motors -ne $expectedMotors[$stageIndex]) -or
            ($stage.Steps -ne $stepsPerTurn) -or
            ([math]::Abs($stage.Delta) -gt $stepDegrees) -or
            ([math]::Abs($actualTotal - $expectedTotals[$stageIndex]) -gt 0.0001)) {
            throw "STAGE_PLAN_FAILED index=$stageIndex"
        }
        $motorCount = $stage.Motors.Split(',').Count
        $speedCount = (Get-RepeatedValueList `
                -Value $speedDegreesPerSecond -MotorList $stage.Motors).Split(',').Count
        if ($speedCount -ne $motorCount) {
            throw "STAGE_SPEED_WIDTH_FAILED name=$($stage.Name)"
        }
    }

    $referenceCrc = Get-Crc16CcittFalse `
        -Data ([System.Text.Encoding]::ASCII.GetBytes('123456789'))
    if ($referenceCrc -ne 0x29B1) {
        throw ('CRC_REFERENCE_FAILED actual={0:X4}' -f $referenceCrc)
    }
    $roundTrip = ConvertFrom-AethorFrame `
        -Line (ConvertTo-AethorFrame -Body 'RSP 42 ok')
    if (($roundTrip.Kind -ne 'RSP') -or ($roundTrip.RequestId -ne 42) -or
        ($roundTrip.Result -ne 'ok')) {
        throw 'FRAME_ROUND_TRIP_FAILED'
    }
    if ($RunMotion.IsPresent) {
        throw 'SELF_TEST_MUST_NOT_RUN_MOTION'
    }

    Write-Output 'ONE_TURN_SELF_TESTS_PASSED stages=3 steps=360'
}

if ($SelfTest) {
    Invoke-OneTurnSelfTest
    exit 0
}
if (-not $RunMotion) {
    throw 'Physical motion is disabled. Pass -RunMotion after reviewing the design.'
}

$stages = @(Get-OneTurnStages)
$serialPort = $null
$context = $null
$script:activeMotorList = ''
$script:shutdownRequired = $false
$executionError = $null
$executionStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
try {
    $serialPort = Open-AethorSerialPort -Name $PortName
    $context = New-AethorContext -SerialPort $serialPort
    Invoke-OneTurnSequence -Context $context -Stages $stages
    Write-Output "OUTPUT_ONE_TURN_SEQUENCE_PASSED port=$PortName stages=$($stages.Count)"
}
catch {
    $executionError = $_
}
finally {
    if ($script:shutdownRequired) {
        Invoke-BestEffortShutdown -Context $context -MotorList $script:activeMotorList
    }
    if ($null -ne $serialPort) {
        if ($serialPort.IsOpen) {
            $serialPort.Close()
        }
        $serialPort.Dispose()
    }
    $executionStopwatch.Stop()
    Write-Output ('OUTPUT_ONE_TURN_ELAPSED seconds={0:F1}' -f `
        $executionStopwatch.Elapsed.TotalSeconds)
}
if ($null -ne $executionError) {
    Write-Error "COM7_OUTPUT_ONE_TURN_FAILED: $($executionError.Exception.Message)" `
        -ErrorAction Continue
    exit 1
}
