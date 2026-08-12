<#
.SYNOPSIS
Monitors STM32 firmware probes and state replies on the USB CDC virtual port.

.DESCRIPTION
Opens COM7 by default, timestamps every received line, sends one capability query,
and periodically requests controller state. The script uses only the Windows .NET
serial-port implementation and always closes the port when stopped with Ctrl+C.
#>

param(
    [Parameter()]
    [ValidatePattern('^COM\d+$')]
    [string]$PortName = 'COM7',

    [Parameter()]
    [ValidateRange(1200, 3000000)]
    [int]$BaudRate = 115200,

    [Parameter()]
    [ValidateRange(0, 60000)]
    [int]$QueryIntervalMs = 1000
)

$ErrorActionPreference = 'Stop'

function Write-ProbeMonitorLine {
    <# Writes one timestamped monitor event or firmware line. #>
    param(
        [Parameter(Mandatory)]
        [string]$Text
    )

    $timestamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff'
    Write-Host "[$timestamp] $Text"
}

function Send-ProbeMonitorCommand {
    <# Sends one bounded ASCII command terminated by LF. #>
    param(
        [Parameter(Mandatory)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory)]
        [ValidateLength(1, 190)]
        [string]$Command
    )

    $SerialPort.Write("$Command`n")
    Write-ProbeMonitorLine "tx $Command"
}

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($PortName -notin $availablePorts) {
    throw "Serial port $PortName is not present. Available: $($availablePorts -join ', ')"
}

$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serialPort.Encoding = [System.Text.Encoding]::ASCII
$serialPort.NewLine = "`n"
$serialPort.ReadTimeout = 100
$serialPort.WriteTimeout = 500
$nextQueryTime = [DateTime]::UtcNow

try {
    $serialPort.Open()
    $serialPort.DiscardInBuffer()
    $serialPort.DiscardOutBuffer()
    Write-ProbeMonitorLine "monitor_open port=$PortName baud=$BaudRate"
    Send-ProbeMonitorCommand -SerialPort $serialPort -Command '#GETCAPS'

    while ($true) {
        try {
            $receivedLine = $serialPort.ReadLine().TrimEnd("`r")
            if ($receivedLine.Length -gt 0) {
                Write-ProbeMonitorLine "rx $receivedLine"
            }
        }
        catch [System.TimeoutException] {
            # A short timeout keeps periodic queries and Ctrl+C responsive.
        }

        if (($QueryIntervalMs -gt 0) -and ([DateTime]::UtcNow -ge $nextQueryTime)) {
            Send-ProbeMonitorCommand -SerialPort $serialPort -Command '#GETSTATE'
            $nextQueryTime = [DateTime]::UtcNow.AddMilliseconds($QueryIntervalMs)
        }
    }
}
finally {
    if ($serialPort.IsOpen) {
        $serialPort.Close()
    }
    $serialPort.Dispose()
    Write-ProbeMonitorLine "monitor_closed port=$PortName"
}
