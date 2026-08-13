<#
.SYNOPSIS
Safely checks and optionally jogs selected unloaded S3519 motors over USB CDC.

.DESCRIPTION
Uses plain aethor-text-v1 lines. The default path identifies and initializes the
selected motors without enabling them; pass -RunMotion for bounded out/back jogs.
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

function ConvertTo-AethorTextLine {
    <# Adds the plain text protocol CRLF terminator. #>
    param([Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Body)

    if ($Body.Length -gt 160) {
        throw "REQUEST_TOO_LONG length=$($Body.Length)"
    }
    return "$Body`r`n"
}

function ConvertFrom-AethorTextLine {
    <# Parses one readable device line into matching metadata. #>
    param([Parameter(Mandatory)][ValidateNotNullOrEmpty()][string]$Line)

    $body = $Line.TrimEnd("`r", "`n")
    $tokens = $body.Split(' ', [System.StringSplitOptions]::RemoveEmptyEntries)
    [uint32]$requestId = 0
    if (($tokens.Count -lt 3) -or
        (-not [uint32]::TryParse($tokens[1], [ref]$requestId)) -or
        ($tokens[0] -notin @('ok', 'done', 'error', 'event', 'data'))) {
        throw "BAD_RESPONSE: $body"
    }
    $resultToken = $tokens | Where-Object { $_ -like 'result=*' } |
        Select-Object -First 1
    return [pscustomobject]@{
        Body = $body
        Kind = $tokens[0]
        RequestId = $requestId
        Result = if ($null -ne $resultToken) {
            $resultToken.Substring('result='.Length)
        }
        else { '' }
    }
}

function Get-MotorSelection {
    <# Validates one unique ascending comma list of motor numbers in 1..7. #>
    param([Parameter(Mandatory)][string]$Text)

    $motorNumbers = New-Object System.Collections.Generic.List[int]
    $previousMotor = 0
    foreach ($token in $Text.Split(',')) {
        [int]$motorNumber = 0
        if ((-not [int]::TryParse($token, [ref]$motorNumber)) -or
            ($motorNumber -lt 1) -or ($motorNumber -gt 7) -or
            ($motorNumber -le $previousMotor)) {
            throw 'MotorList must contain unique ascending motor numbers in 1..7.'
        }
        $motorNumbers.Add($motorNumber)
        $previousMotor = $motorNumber
    }
    if ($motorNumbers.Count -eq 0) {
        throw 'MotorList must not be empty.'
    }
    return [pscustomobject]@{
        Canonical = ($motorNumbers -join ',')
        Count = $motorNumbers.Count
    }
}

function New-AethorTextContext {
    <# Creates mutable request and keepalive state around one optional serial port. #>
    param($SerialPort)

    $randomToken = [guid]::NewGuid().ToString('N').Substring(0, 8)
    [uint32]$initialRequestId = [Convert]::ToUInt32($randomToken, 16)
    if ($initialRequestId -eq 0) {
        $initialRequestId = 1
    }
    return [pscustomobject]@{
        SerialPort = $SerialPort
        NextRequestId = $initialRequestId
        LastPingUtc = [datetime]::MinValue
        PingRequestIds = New-Object 'System.Collections.Generic.HashSet[uint32]'
    }
}

function Write-AethorTextBody {
    <# Writes one timestamped readable request to the selected port. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Body
    )

    Write-Host ('[{0:HH:mm:ss.fff}] > {1}' -f [datetime]::Now, $Body)
    $Context.SerialPort.Write((ConvertTo-AethorTextLine -Body $Body))
}

function Send-AethorTextRequest {
    <# Allocates a nonzero request ID and sends one complete command string. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Command
    )

    $requestId = $Context.NextRequestId
    $Context.NextRequestId = [uint32]($Context.NextRequestId + 1)
    Write-AethorTextBody -Context $Context -Body "$requestId $Command"
    return $requestId
}

function Send-AethorPingIfDue {
    <# Sends one keepalive ping after 200 ms during energized actions. #>
    param([Parameter(Mandatory)]$Context)

    $now = [datetime]::UtcNow
    if (($now - $Context.LastPingUtc).TotalMilliseconds -lt 200) {
        return
    }
    $pingId = Send-AethorTextRequest -Context $Context -Command 'ping'
    [void]$Context.PingRequestIds.Add([uint32]$pingId)
    $Context.LastPingUtc = $now
}

function Read-AethorTextResponse {
    <# Reads one response or returns null on the short serial timeout. #>
    param([Parameter(Mandatory)]$Context)

    try {
        $line = $Context.SerialPort.ReadLine()
    }
    catch [System.TimeoutException] {
        return $null
    }
    $response = ConvertFrom-AethorTextLine -Line $line
    Write-Host ('[{0:HH:mm:ss.fff}] < {1}' -f [datetime]::Now, $response.Body)
    return $response
}

function Wait-AethorTextRequest {
    <# Waits for one matching query or accepted-plus-done action lifecycle. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][uint32]$RequestId,
        [Parameter(Mandatory)][ValidateSet('Query', 'Action')][string]$RequestType,
        [Parameter(Mandatory)][int]$TimeoutMilliseconds,
        [switch]$KeepAlive
    )

    $deadline = [datetime]::UtcNow.AddMilliseconds($TimeoutMilliseconds)
    $accepted = $false
    while ([datetime]::UtcNow -lt $deadline) {
        if ($KeepAlive) {
            Send-AethorPingIfDue -Context $Context
        }
        $response = Read-AethorTextResponse -Context $Context
        if ($null -eq $response) {
            continue
        }
        if ($Context.PingRequestIds.Remove([uint32]$response.RequestId)) {
            continue
        }
        if (($response.Kind -in @('event', 'data')) -or
            ($response.RequestId -ne $RequestId)) {
            continue
        }
        if ($response.Kind -eq 'error') {
            throw "DEVICE_ERROR: $($response.Body)"
        }
        if (($RequestType -eq 'Query') -and ($response.Kind -eq 'ok')) {
            return $response
        }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'ok')) {
            $accepted = $true
            continue
        }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'done')) {
            if (-not $accepted) {
                throw "DONE_WITHOUT_ACCEPT: $($response.Body)"
            }
            if ($response.Result -notin @('completed', 'stopped')) {
                throw "ACTION_FAILED: $($response.Body)"
            }
            return $response
        }
    }
    throw "REQUEST_TIMEOUT id=$RequestId type=$RequestType timeout_ms=$TimeoutMilliseconds"
}

function Invoke-AethorTextQuery {
    <# Sends one readable query and waits for its matching ok response. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Command
    )

    $requestId = Send-AethorTextRequest -Context $Context -Command $Command
    return Wait-AethorTextRequest -Context $Context -RequestId $requestId `
        -RequestType Query -TimeoutMilliseconds 3000
}

function Invoke-AethorTextAction {
    <# Sends one action and waits for matching accepted and successful done lines. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)][string]$Command,
        [Parameter(Mandatory)][int]$TimeoutMilliseconds,
        [switch]$KeepAlive
    )

    $requestId = Send-AethorTextRequest -Context $Context -Command $Command
    return Wait-AethorTextRequest -Context $Context -RequestId $requestId `
        -RequestType Action -TimeoutMilliseconds $TimeoutMilliseconds `
        -KeepAlive:$KeepAlive
}

function Invoke-ScriptSelfTest {
    <# Verifies line framing, response parsing, selection rules, and safe defaults. #>

    $context = New-AethorTextContext -SerialPort $null
    $secondContext = New-AethorTextContext -SerialPort $null
    if (($context.NextRequestId -eq 0) -or
        ($context.NextRequestId -eq $secondContext.NextRequestId)) {
        throw 'CONTEXT_DEFAULTS_FAILED'
    }
    if ((ConvertTo-AethorTextLine -Body '42 show joints') -ne
        "42 show joints`r`n") {
        throw 'LINE_FORMAT_FAILED'
    }
    $parsed = ConvertFrom-AethorTextLine `
        -Line 'done 42 bench jog result=completed elapsed_ms=200 arrived=05'
    if (($parsed.Kind -ne 'done') -or ($parsed.RequestId -ne 42) -or
        ($parsed.Result -ne 'completed')) {
        throw 'LINE_PARSE_FAILED'
    }
    $selection = Get-MotorSelection -Text '1,3'
    if ($selection.Canonical -ne '1,3') {
        throw 'MOTOR_SELECTION_FAILED'
    }
    $invalidSelectionRejected = $false
    try {
        Get-MotorSelection -Text '3,1' | Out-Null
    }
    catch {
        $invalidSelectionRejected = $true
    }
    if (-not $invalidSelectionRejected) {
        throw 'UNSORTED_MOTORS_NOT_REJECTED'
    }
    Write-Output "SELF_TESTS_PASSED protocol=aethor-text-v1 motors=$($selection.Canonical) run_motion_default=$([bool]$RunMotion)"
}

function Open-AethorSerialPort {
    <# Opens one exclusive ASCII USB CDC port with bounded timeouts. #>
    param([Parameter(Mandatory)][string]$Name)

    $serialPort = [System.IO.Ports.SerialPort]::new(
        $Name, 115200, [System.IO.Ports.Parity]::None, 8,
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
        throw "$Name is occupied. Close the other serial connection and retry."
    }
    $serialPort.DiscardInBuffer()
    $serialPort.DiscardOutBuffer()
    return $serialPort
}

function Invoke-ReadOnlyStage {
    <# Identifies firmware and initializes selected motors without enabling them. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)]$Selection
    )

    Invoke-AethorTextQuery -Context $Context -Command 'hello' | Out-Null
    Invoke-AethorTextQuery -Context $Context -Command 'stream off' | Out-Null
    foreach ($command in @('show info', 'show config', 'show state',
            'show motors', 'show diag')) {
        Invoke-AethorTextQuery -Context $Context -Command $command | Out-Null
    }
    Invoke-AethorTextAction -Context $Context `
        -Command "bench init $($Selection.Canonical)" `
        -TimeoutMilliseconds 30000 | Out-Null
    Invoke-AethorTextQuery -Context $Context -Command 'show motors' | Out-Null
}

function Invoke-MotionStage {
    <# Clears, enables, jogs out/back, stops, and disables selected motors. #>
    param(
        [Parameter(Mandatory)]$Context,
        [Parameter(Mandatory)]$Selection,
        [Parameter(Mandatory)][double]$Delta,
        [Parameter(Mandatory)][double]$Speed
    )

    $deltaText = $Delta.ToString('0.0###', $invariantCulture)
    $reverseDeltaText = (-$Delta).ToString('0.0###', $invariantCulture)
    $speedText = $Speed.ToString('0.0###', $invariantCulture)
    Invoke-AethorTextAction -Context $Context `
        -Command "bench clear $($Selection.Canonical)" `
        -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorTextAction -Context $Context `
        -Command "bench enable $($Selection.Canonical)" `
        -TimeoutMilliseconds 10000 -KeepAlive | Out-Null
    Invoke-AethorTextAction -Context $Context `
        -Command "bench jog $($Selection.Canonical) delta=$deltaText speed=$speedText" `
        -TimeoutMilliseconds 10000 -KeepAlive | Out-Null
    Invoke-AethorTextAction -Context $Context `
        -Command "bench jog $($Selection.Canonical) delta=$reverseDeltaText speed=$speedText" `
        -TimeoutMilliseconds 10000 -KeepAlive | Out-Null
    Invoke-AethorTextAction -Context $Context `
        -Command "bench stop $($Selection.Canonical)" `
        -TimeoutMilliseconds 10000 -KeepAlive | Out-Null
    Invoke-AethorTextAction -Context $Context `
        -Command "bench disable $($Selection.Canonical)" `
        -TimeoutMilliseconds 10000 | Out-Null
    Invoke-AethorTextQuery -Context $Context -Command 'show motors' | Out-Null
    Invoke-AethorTextQuery -Context $Context -Command 'show diag' | Out-Null
}

function Invoke-BestEffortShutdown {
    <# Tries bench stop and disable without masking the original failure. #>
    param($Context, [Parameter(Mandatory)]$Selection)

    if (($null -eq $Context) -or ($null -eq $Context.SerialPort) -or
        (-not $Context.SerialPort.IsOpen)) {
        return
    }
    foreach ($operation in @('stop', 'disable')) {
        try {
            Invoke-AethorTextAction -Context $Context `
                -Command "bench $operation $($Selection.Canonical)" `
                -TimeoutMilliseconds 3000 -KeepAlive | Out-Null
        }
        catch {
            Write-Warning "Best-effort bench $operation failed: $($_.Exception.Message)"
        }
    }
}

if ($SelfTest) {
    Invoke-ScriptSelfTest
    exit 0
}

$selection = Get-MotorSelection -Text $MotorList
$serialPort = $null
$context = $null
$shutdownRequired = $false
try {
    $serialPort = Open-AethorSerialPort -Name $PortName
    $context = New-AethorTextContext -SerialPort $serialPort
    Invoke-ReadOnlyStage -Context $context -Selection $selection
    if (-not $RunMotion) {
        Write-Output "READ_ONLY_DEBUG_PASSED port=$PortName motors=$($selection.Canonical) protocol=aethor-text-v1"
        exit 0
    }
    $shutdownRequired = $true
    Invoke-MotionStage -Context $context -Selection $selection `
        -Delta $DeltaDegrees -Speed $SpeedDegreesPerSecond
    $shutdownRequired = $false
    Write-Output "MOTION_DEBUG_PASSED port=$PortName motors=$($selection.Canonical) protocol=aethor-text-v1"
}
catch {
    Write-Error "COM7_AETHOR_TEXT_DEBUG_FAILED: $($_.Exception.Message)"
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
