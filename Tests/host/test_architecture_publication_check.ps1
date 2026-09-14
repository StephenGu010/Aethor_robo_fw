<#
.SYNOPSIS
Verifies the normal-command publication checker against valid and invalid C source.
#>
$ErrorActionPreference = 'Stop'

function Invoke-PublicationCheckerRegression {
    <# Loads only the checker helper and verifies growth and ordering mutations. #>
    $checkerPath = Join-Path $PSScriptRoot 'check_phase0_architecture.ps1'
    $parseTokens = $null
    $parseErrors = $null
    $checkerAst = [System.Management.Automation.Language.Parser]::ParseFile(
        $checkerPath, [ref]$parseTokens, [ref]$parseErrors)
    $helper = $checkerAst.Find({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Test-NormalCommandPublicationOrder'
    }, $true)
    if ($null -eq $helper) { throw 'Publication checker helper is missing.' }
    . ([scriptblock]::Create($helper.Extent.Text))
    $sourcePath = Join-Path $PSScriptRoot '..\..\App\Protocol\protocol_engine.c'
    $sourceText = Get-Content -LiteralPath $sourcePath -Raw
    $functionMatch = [regex]::Match($sourceText,
        '(?ms)^static uint8_t protocol_engine_publish_normal_command\([^)]*\)\s*\{.*?^\}')
    if (-not $functionMatch.Success) { throw 'Publication function was not found.' }
    $functionText = $functionMatch.Value
    $cases = @(
        @{ Name = 'current source'; Text = $sourceText; Expected = $true },
        @{ Name = 'long admission logic'; Text = $functionText.Replace('uint8_t used_count', ('/*' + ('x' * 3000) + '*/' + "`n    uint8_t used_count")); Expected = $true },
        @{ Name = 'early publication'; Text = $functionText.Replace('++engine->command_write_sequence;', '').Replace('engine->commands[slot_index] = *command;', '++engine->command_write_sequence; engine->commands[slot_index] = *command;'); Expected = $false },
        @{ Name = 'missing barrier'; Text = $functionText.Replace('protocol_engine_compiler_barrier();', ''); Expected = $false },
        @{ Name = 'missing origin'; Text = $functionText.Replace('engine->active_motion_origin = command->origin;', ''); Expected = $false },
        @{ Name = 'missing epoch'; Text = $functionText.Replace('engine->active_motion_epoch = command->session_id;', ''); Expected = $false },
        @{ Name = 'duplicate sequence publication'; Text = $functionText.Replace('++engine->command_write_sequence;', '++engine->command_write_sequence; ++engine->command_write_sequence;'); Expected = $false },
        @{ Name = 'metadata in different function'; Text = ($functionText.Replace('engine->active_motion_request_id = command->request_id;', '') + "`n" + $functionText.Replace('protocol_engine_publish_normal_command', 'unrelated_function')); Expected = $false }
    )
    foreach ($testCase in $cases) {
        if ((Test-NormalCommandPublicationOrder -Text $testCase.Text) -ne $testCase.Expected) {
            throw ('Publication regression failed: ' + $testCase.Name)
        }
    }
    $stopHelper = $checkerAst.Find({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Test-StopCommandPublicationOrder'
    }, $true)
    if ($null -eq $stopHelper) { throw 'STOP publication helper is missing.' }
    . ([scriptblock]::Create($stopHelper.Extent.Text))
    $stopCases = @(
        @{ Name = 'current STOP'; Text = $sourceText; Expected = $true },
        @{ Name = 'missing STOP origin'; Text = $sourceText.Replace('engine->active_stop_origin = command->origin;', ''); Expected = $false },
        @{ Name = 'missing STOP epoch'; Text = $sourceText.Replace('engine->active_stop_epoch = command->session_id;', ''); Expected = $false },
        @{ Name = 'early STOP'; Text = $sourceText.Replace('++engine->stop_write_sequence;', '').Replace('engine->stop_command = *command;', '++engine->stop_write_sequence; engine->stop_command = *command;'); Expected = $false },
        @{ Name = 'duplicate STOP'; Text = $sourceText.Replace('++engine->stop_write_sequence;', '++engine->stop_write_sequence; ++engine->stop_write_sequence;'); Expected = $false }
    )
    foreach ($testCase in $stopCases) {
        if ((Test-StopCommandPublicationOrder -Text $testCase.Text) -ne $testCase.Expected) {
            throw ('STOP publication regression failed: ' + $testCase.Name)
        }
    }
    Write-Output ('PUBLICATION_CHECKER_REGRESSIONS_PASSED (' + ($cases.Count + $stopCases.Count) + ' cases)')
}

Invoke-PublicationCheckerRegression
