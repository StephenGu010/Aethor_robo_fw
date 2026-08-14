<#
.SYNOPSIS
Generates a C header from the protocol Golden Frames shared with PC-side tests.

.DESCRIPTION
Reads the versioned JSON contract and emits a build-only header consumed by the
host C test suite. The JSON remains the single editable source of test vectors.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$InputPath,

    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

function ConvertTo-CStringLiteral {
    <# Converts an arbitrary JSON string value into a quoted C string literal. #>
    param([AllowEmptyString()][string]$Value)

    $escapedValue = $Value.Replace('\', '\\').Replace('"', '\"')
    return '"' + $escapedValue + '"'
}

function ConvertTo-ProtocolStatusToken {
    <# Maps a JSON status name to the corresponding public C enum token. #>
    param([Parameter(Mandatory = $true)][string]$StatusName)

    switch ($StatusName)
    {
        'OK' { return 'ASCII_PROTOCOL_STATUS_OK' }
        'BAD_CRC' { return 'ASCII_PROTOCOL_STATUS_BAD_CRC' }
        'BAD_FRAME' { return 'ASCII_PROTOCOL_STATUS_BAD_FRAME' }
        default { throw "Unsupported Golden Frame status '$StatusName'." }
    }
}

function Write-GoldenHeader {
    <# Validates the JSON vectors and writes the deterministic generated header. #>
    $resolvedInputPath = (Resolve-Path -LiteralPath $InputPath).Path
    $goldenDocument = Get-Content -LiteralPath $resolvedInputPath -Raw -Encoding UTF8 | ConvertFrom-Json

    if ($goldenDocument.protocol -ne 'aethor-arm-ascii-v1')
    {
        throw "Unexpected protocol '$($goldenDocument.protocol)'."
    }

    $headerLines = @(
        '/**',
        ' * @file protocol_golden_vectors.h',
        ' * @brief Generated protocol vectors; edit the source JSON, not this file.',
        ' */',
        '',
        '#ifndef TESTS_PROTOCOL_GOLDEN_VECTORS_H',
        '#define TESTS_PROTOCOL_GOLDEN_VECTORS_H',
        '',
        '#include <stdint.h>',
        '',
        '#include "ascii_protocol.h"',
        '',
        '/** @brief Describes one shared firmware and PC protocol test vector. */',
        'typedef struct',
        '{',
        '    const char *name;',
        '    const char *body;',
        '    const char *crc16_text;',
        '    AsciiProtocolStatus expected_status;',
        '    uint32_t request_id;',
        '    const char *operation;',
        '    uint8_t field_count;',
        '    uint8_t use_crlf;',
        '} ProtocolGoldenVector;',
        '',
        'static const ProtocolGoldenVector protocol_golden_vectors[] =',
        '{'
    )

    foreach ($vector in $goldenDocument.vectors)
    {
        if ($vector.crc16 -notmatch '^[0-9A-F]{4}$')
        {
            throw "Vector '$($vector.name)' has an invalid CRC string."
        }

        $statusToken = ConvertTo-ProtocolStatusToken -StatusName $vector.expected_status
        $useCrlf = [int]($vector.line_ending -eq 'CRLF')
        $headerLines += ('    {{ {0}, {1}, {2}, {3}, {4}U, {5}, {6}U, {7}U }},' -f `
            (ConvertTo-CStringLiteral $vector.name),
            (ConvertTo-CStringLiteral $vector.body),
            (ConvertTo-CStringLiteral $vector.crc16),
            $statusToken,
            [uint32]$vector.request_id,
            (ConvertTo-CStringLiteral $vector.operation),
            [byte]$vector.field_count,
            $useCrlf)
    }

    $headerLines += @(
        '};',
        '',
        '#define PROTOCOL_GOLDEN_VECTOR_COUNT \',
        '    (sizeof(protocol_golden_vectors) / sizeof(protocol_golden_vectors[0]))',
        '',
        '#endif'
    )

    $outputDirectory = Split-Path -Parent $OutputPath
    if (-not (Test-Path -LiteralPath $outputDirectory))
    {
        New-Item -ItemType Directory -Path $outputDirectory | Out-Null
    }

    Set-Content -LiteralPath $OutputPath -Value $headerLines -Encoding UTF8
}

Write-GoldenHeader
