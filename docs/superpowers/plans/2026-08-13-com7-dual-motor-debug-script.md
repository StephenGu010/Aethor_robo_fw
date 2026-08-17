# COM7 Dual-Motor Debug Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Windows PowerShell 5.1 script that safely checks and optionally moves unloaded S3519 motors with CAN IDs 1 and 3 over COM7 using the current CRC-protected ASCII protocol.

**Architecture:** Keep the existing legacy hardware scripts unchanged and add one standalone script with three internal boundaries: pure protocol helpers, bounded serial request/heartbeat helpers, and the staged hardware workflow. A host wrapper invokes `-SelfTest` without opening COM7; real read-only and motion runs remain separate evidence gates.

**Tech Stack:** Windows PowerShell 5.1, `.NET System.IO.Ports.SerialPort`, `aethor-arm-ascii-v1`, CRC-16/CCITT-FALSE, existing Windows host test scripts.

---

## File map

- Create `Tests/host/test_com7_dual_motor_debug_script.ps1`: launches the target script in `-SelfTest` mode and checks the stable success contract.
- Create `Tests/hardware/debug_com7_motors_1_3.ps1`: owns protocol encoding/decoding, COM7 transactions, heartbeat scheduling, safe cleanup, and the staged CAN ID 1/3 workflow.
- Modify `Docs/handoffs/first-arm-software/verification.txt`: record only the reproducible self-test command after it passes; append real COM7 evidence only after a real run.
- Preserve `Tests/hardware/monitor_com7.ps1`: it is a legacy `#PING/#GETSTATE` monitor and is not called by the new script.

### Task 1: Add the failing host contract

**Files:**
- Create: `Tests/host/test_com7_dual_motor_debug_script.ps1`
- Test: `Tests/host/test_com7_dual_motor_debug_script.ps1`

- [ ] **Step 1: Write the failing test**

Create the following complete host wrapper:

```powershell
<#
.SYNOPSIS
Runs the COM7 dual-motor script self-tests without opening physical hardware.
#>

$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$debugScriptPath = Join-Path $repositoryRoot 'Tests\hardware\debug_com7_motors_1_3.ps1'

if (-not (Test-Path -LiteralPath $debugScriptPath -PathType Leaf)) {
    throw "Missing debug script: $debugScriptPath"
}

$selfTestOutput = & powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File $debugScriptPath -SelfTest 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "COM7 debug self-test failed with exit code $LASTEXITCODE`: $selfTestOutput"
}

$joinedOutput = $selfTestOutput -join "`n"
if ($joinedOutput -notmatch 'SELF_TESTS_PASSED') {
    throw "COM7 debug self-test did not publish its success marker: $joinedOutput"
}
if ($joinedOutput -notmatch 'crc=29B1') {
    throw "CRC reference vector was not verified: $joinedOutput"
}
if ($joinedOutput -notmatch 'motors=1,3') {
    throw "Default motor selection was not verified: $joinedOutput"
}
if ($joinedOutput -notmatch 'run_motion_default=False') {
    throw "Safe non-motion default was not verified: $joinedOutput"
}

Write-Output 'COM7_DEBUG_SCRIPT_TESTS_PASSED'
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

Expected: nonzero exit with `Missing debug script` because `debug_com7_motors_1_3.ps1` does not exist.

- [ ] **Step 3: Commit the red test**

```powershell
git add -- Tests/host/test_com7_dual_motor_debug_script.ps1
git commit -m "test: specify COM7 dual-motor debug contract"
```

### Task 2: Implement protocol helpers and self-test mode

**Files:**
- Create: `Tests/hardware/debug_com7_motors_1_3.ps1`
- Test: `Tests/host/test_com7_dual_motor_debug_script.ps1`

- [ ] **Step 1: Add the script shell and pure helpers**

Start the new script with a file-level help comment, parameters, and these complete functions:

```powershell
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
            } else {
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
    return '{0} *{1:X4}`r`n' -f $Body, $crc
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
    if ($tokens.Count -eq 0) { throw 'MotorList must not be empty.' }
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
```

- [ ] **Step 2: Add executable self-tests**

Append this complete self-test function and early exit:

```powershell
function Invoke-ScriptSelfTest {
    <# Verifies CRC vectors, frame rejection, selection mapping, and safe defaults. #>
    $referenceCrc = Get-Crc16CcittFalse `
        -Data ([System.Text.Encoding]::ASCII.GetBytes('123456789'))
    if ($referenceCrc -ne 0x29B1) { throw ('CRC_REFERENCE_FAILED: {0:X4}' -f $referenceCrc) }

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
    try { ConvertFrom-AethorFrame -Line 'RSP 42 ok *0000' | Out-Null }
    catch { $badCrcRejected = $true }
    if (-not $badCrcRejected) { throw 'BAD_CRC_NOT_REJECTED' }

    $selection = Get-MotorSelection -Text '1,3' -Magnitude 0.2
    if (($selection.Canonical -ne '1,3') -or
        ($selection.ForwardValues -ne '0.2,-0.2') -or
        ($selection.ReverseValues -ne '-0.2,0.2')) {
        throw 'MOTOR_SELECTION_FAILED'
    }

    $invalidSelectionRejected = $false
    try { Get-MotorSelection -Text '3,1' -Magnitude 0.2 | Out-Null }
    catch { $invalidSelectionRejected = $true }
    if (-not $invalidSelectionRejected) { throw 'UNSORTED_MOTORS_NOT_REJECTED' }

    Write-Output ('SELF_TESTS_PASSED crc={0:X4} motors={1} run_motion_default={2}' -f `
        $referenceCrc, $selection.Canonical, [bool]$RunMotion)
}

if ($SelfTest) {
    Invoke-ScriptSelfTest
    exit 0
}
```

- [ ] **Step 3: Run the host contract and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

Expected: `COM7_DEBUG_SCRIPT_TESTS_PASSED` and exit code 0. COM7 must remain unopened.

- [ ] **Step 4: Commit protocol helpers**

```powershell
git add -- Tests/hardware/debug_com7_motors_1_3.ps1
git commit -m "feat: add COM7 protocol self-tests"
```

### Task 3: Implement bounded serial transactions and heartbeat

**Files:**
- Modify: `Tests/hardware/debug_com7_motors_1_3.ps1`
- Test: `Tests/host/test_com7_dual_motor_debug_script.ps1`

- [ ] **Step 1: Extend self-test expectations before serial implementation**

In `Invoke-ScriptSelfTest`, add assertions that the context constructor starts at request ID 1 and has no session. Run the host test and expect `New-AethorContext` to be unrecognized.

```powershell
$testContext = New-AethorContext -SerialPort $null
if (($testContext.NextRequestId -ne 1) -or ($testContext.SessionId -ne 0)) {
    throw 'CONTEXT_DEFAULTS_FAILED'
}
```

- [ ] **Step 2: Add serial context, write, read, and wait functions**

Add these functions before `Invoke-ScriptSelfTest`:

```powershell
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
    if ($Fields.Length -gt 0) { $body += " $Fields" }
    Write-AethorBody -Context $Context -Body $body
    return $requestId
}

function Send-AethorHeartbeatIfDue {
    <# Sends one session heartbeat when at least 200 ms have elapsed. #>
    param([Parameter(Mandatory)]$Context)
    if ($Context.SessionId -eq 0) { return }
    $now = [datetime]::UtcNow
    if (($now - $Context.LastHeartbeatUtc).TotalMilliseconds -lt 200) { return }
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
    } catch [System.TimeoutException] {
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
        if ($RequestType -eq 'Action') { Send-AethorHeartbeatIfDue -Context $Context }
        $response = Read-AethorResponse -Context $Context
        if ($null -eq $response) { continue }
        if ($Context.HeartbeatRequestIds.Remove([uint32]$response.RequestId)) { continue }
        if ($response.RequestId -ne $RequestId) { continue }
        if ($response.Kind -eq 'ERR') { throw "DEVICE_ERROR: $($response.Body)" }
        if (($RequestType -eq 'Query') -and ($response.Kind -eq 'RSP')) { return $response }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'ACK')) {
            $acknowledged = $true
            continue
        }
        if (($RequestType -eq 'Action') -and ($response.Kind -eq 'DONE')) {
            if (-not $acknowledged) { throw "DONE_WITHOUT_ACK: $($response.Body)" }
            if (($response.Result -ne 'COMPLETED') -and ($response.Result -ne 'STOPPED')) {
                throw "ACTION_FAILED: $($response.Body)"
            }
            return $response
        }
    }
    throw "REQUEST_TIMEOUT id=$RequestId type=$RequestType timeout_ms=$TimeoutMilliseconds"
}

function Invoke-AethorQuery {
    <# Sends one query and returns its matching validated RSP. #>
    param($Context, [string]$Operation, [string]$Fields = '')
    $requestId = Send-AethorRequest -Context $Context -Operation $Operation -Fields $Fields
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Query -TimeoutMilliseconds 3000
}

function Invoke-AethorAction {
    <# Sends one action and waits for its ACK plus successful DONE. #>
    param($Context, [string]$Operation, [string]$Fields, [int]$TimeoutMilliseconds)
    $requestId = Send-AethorRequest -Context $Context -Operation $Operation -Fields $Fields
    return Wait-AethorRequest -Context $Context -RequestId $requestId `
        -RequestType Action -TimeoutMilliseconds $TimeoutMilliseconds
}
```

- [ ] **Step 3: Run the self-tests and existing protocol tests**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
```

Expected: `COM7_DEBUG_SCRIPT_TESTS_PASSED` and `PROTOCOL_TESTS_PASSED`.

- [ ] **Step 4: Commit serial transaction support**

```powershell
git add -- Tests/hardware/debug_com7_motors_1_3.ps1
git commit -m "feat: add CRC serial transactions and heartbeat"
```

### Task 4: Implement read-only and explicit-motion workflows

**Files:**
- Modify: `Tests/hardware/debug_com7_motors_1_3.ps1`
- Test: `Tests/host/test_com7_dual_motor_debug_script.ps1`

- [ ] **Step 1: Add the staged workflow**

Append these functions after the self-test early exit, followed by the main invocation:

```powershell
function Open-AethorSerialPort {
    <# Opens one exclusive ASCII USB CDC port with bounded read/write timeouts. #>
    param([Parameter(Mandatory)][string]$Name)
    $serialPort = [System.IO.Ports.SerialPort]::new(
        $Name, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $serialPort.Encoding = [System.Text.Encoding]::ASCII
    $serialPort.NewLine = "`n"
    $serialPort.ReadTimeout = 50
    $serialPort.WriteTimeout = 500
    try {
        $serialPort.Open()
    } catch [System.UnauthorizedAccessException] {
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
    if (-not $sessionMatch.Success) { throw "HELLO_SESSION_MISSING: $($hello.Body)" }
    $Context.SessionId = [uint32]::Parse($sessionMatch.Groups[1].Value)
    $Context.LastHeartbeatUtc = [datetime]::UtcNow
}

function Invoke-ReadOnlyStage {
    <# Runs handshake, queries, and selected-motor discovery without enabling motion. #>
    param($Context, $Selection)
    Initialize-AethorSession -Context $Context
    Invoke-AethorQuery $Context 'SET_STREAM' 'rate_hz=0 fields=jpos,jvel,state,motor' | Out-Null
    foreach ($queryOperation in @('GET_INFO', 'GET_CONFIG', 'GET_STATE', 'GET_MOTORS', 'GET_DIAG')) {
        Invoke-AethorQuery $Context $queryOperation | Out-Null
    }
    Invoke-AethorAction $Context 'INIT_MOTORS' "motors=$($Selection.Canonical)" 30000 | Out-Null
    Invoke-AethorQuery $Context 'GET_MOTORS' | Out-Null
}

function Invoke-MotionStage {
    <# Clears, enables, moves out/back, stops, disables, and reads final diagnostics. #>
    param($Context, $Selection, [double]$Speed)
    $speedText = $Speed.ToString('0.0###', $invariantCulture)
    $speedValues = (@($speedText) * $Selection.Count) -join ','
    Invoke-AethorAction $Context 'CLEAR_FAULT' "motors=$($Selection.Canonical)" 10000 | Out-Null
    Invoke-AethorAction $Context 'ENABLE' "motors=$($Selection.Canonical)" 10000 | Out-Null
    Invoke-AethorAction $Context 'MOVE_REL' `
        "motors=$($Selection.Canonical) delta_deg=$($Selection.ForwardValues) speed_deg_s=$speedValues" 10000 | Out-Null
    Invoke-AethorAction $Context 'MOVE_REL' `
        "motors=$($Selection.Canonical) delta_deg=$($Selection.ReverseValues) speed_deg_s=$speedValues" 10000 | Out-Null
    Invoke-AethorAction $Context 'STOP' "motors=$($Selection.Canonical)" 10000 | Out-Null
    Invoke-AethorAction $Context 'DISABLE' "motors=$($Selection.Canonical)" 10000 | Out-Null
    Invoke-AethorQuery $Context 'GET_MOTORS' | Out-Null
    Invoke-AethorQuery $Context 'GET_DIAG' | Out-Null
}

function Invoke-BestEffortShutdown {
    <# Tries STOP then DISABLE without hiding the original failure. #>
    param($Context, $Selection)
    if (($null -eq $Context) -or ($Context.SessionId -eq 0) -or
        ($null -eq $Context.SerialPort) -or (-not $Context.SerialPort.IsOpen)) { return }
    foreach ($operation in @('STOP', 'DISABLE')) {
        try {
            Invoke-AethorAction $Context $operation "motors=$($Selection.Canonical)" 3000 | Out-Null
        } catch {
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
} catch {
    Write-Error "COM7_DUAL_MOTOR_DEBUG_FAILED: $($_.Exception.Message)"
    exit 1
} finally {
    if ($shutdownRequired) { Invoke-BestEffortShutdown -Context $context -Selection $selection }
    if ($null -ne $serialPort) {
        if ($serialPort.IsOpen) { $serialPort.Close() }
        $serialPort.Dispose()
    }
}
```

- [ ] **Step 2: Run self-tests and syntax validation**

```powershell
$parseErrors = $null
[void][System.Management.Automation.Language.Parser]::ParseFile(
    (Resolve-Path .\Tests\hardware\debug_com7_motors_1_3.ps1),
    [ref]$null,
    [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { $parseErrors; exit 1 }
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

Expected: no parser errors and `COM7_DEBUG_SCRIPT_TESTS_PASSED`.

- [ ] **Step 3: Commit the staged hardware workflow**

```powershell
git add -- Tests/hardware/debug_com7_motors_1_3.ps1
git commit -m "feat: add staged COM7 dual-motor debugging"
```

### Task 5: Verify software without claiming hardware success

**Files:**
- Modify: `Docs/handoffs/first-arm-software/verification.txt`
- Test: all affected host checks

- [ ] **Step 1: Run fresh verification**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
git diff --check
```

Expected markers:

```text
COM7_DEBUG_SCRIPT_TESTS_PASSED
PROTOCOL_TESTS_PASSED
MOTOR_CORE_TESTS_PASSED
```

- [ ] **Step 2: Append software-only evidence**

Append exactly this evidence line, using the real execution date and preserving existing contents:

```text
- Tests/host/test_com7_dual_motor_debug_script.ps1: COM7_DEBUG_SCRIPT_TESTS_PASSED (software-only; COM7 motion not implied)
```

- [ ] **Step 3: Commit verification documentation**

```powershell
git add -- Docs/handoffs/first-arm-software/verification.txt
git commit -m "docs: record COM7 debug script self-test"
```

### Task 6: Execute the two real-hardware gates

**Files:**
- No source changes before the run
- Runtime evidence: console transcript supplied by the script

- [ ] **Step 1: Ensure UartAssist releases COM7**

Close only UartAssist's serial connection. Do not terminate the process automatically. Confirm COM7 still appears as a Windows serial device.

- [ ] **Step 2: Run the read-only gate**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File .\Tests\hardware\debug_com7_motors_1_3.ps1 `
    -PortName COM7 -MotorList '1,3'
```

Expected only after real responses: `READ_ONLY_DEBUG_PASSED port=COM7 motors=1,3`. If this fails, stop; do not run motion.

- [ ] **Step 3: Run the explicitly authorized unloaded-motion gate**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File .\Tests\hardware\debug_com7_motors_1_3.ps1 `
    -PortName COM7 -MotorList '1,3' -RunMotion `
    -DeltaDegrees 0.2 -SpeedDegreesPerSecond 1.0
```

Expected only after real ACK/DONE traffic: `MOTION_DEBUG_PASSED port=COM7 motors=1,3`. On any failure, preserve the exact transcript and report whether best-effort STOP/DISABLE succeeded.

- [ ] **Step 4: Record honest hardware evidence**

Only after the real run, append a dated result to `Docs/handoffs/first-arm-software/verification.txt` containing COM port, motor list, delta, speed, final marker, and any failure. Do not generalize a CAN ID 1/3 pass to J2 or J4-J7.

- [ ] **Step 5: Commit real evidence if obtained**

```powershell
git add -- Docs/handoffs/first-arm-software/verification.txt
git commit -m "test: record COM7 motors 1 and 3 commissioning"
```

## Plan self-review

- Spec coverage: safe default, explicit `-RunMotion`, COM7, motors 1/3, CRC, session, 250 ms heartbeat, ACK/DONE, STOP/DISABLE cleanup, no permanent motor writes, self-test, read-only gate, and hardware evidence are each assigned to a task.
- Placeholder scan: every implementation and verification step is concrete.
- Type consistency: request IDs and session IDs are `uint32`; CRC values are `uint16`; motor selection is a canonical comma string plus count and paired delta strings; all later tasks use the same property names.
