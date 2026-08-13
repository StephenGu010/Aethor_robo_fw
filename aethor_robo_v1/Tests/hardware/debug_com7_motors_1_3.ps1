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

function Invoke-ScriptSelfTest {
    <# Verifies CRC vectors, frame rejection, selection mapping, and safe defaults. #>

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

throw 'Hardware workflow is not implemented yet.'
