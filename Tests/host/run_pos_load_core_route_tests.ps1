<# @file run_pos_load_core_route_tests.ps1. Compile verbatim maintained Core POS terminal routing without hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Get-CoreRoutingBlock {
    <# Extract one complete balanced C routing statement from its unique maintained source anchor. #>
    param([string]$Source, [string]$Anchor)
    $position = $Source.IndexOf($Anchor, [StringComparison]::Ordinal)
    if ($position -lt 0 -or $Source.LastIndexOf($Anchor, [StringComparison]::Ordinal) -ne $position) {
        throw 'Core routing anchor missing or ambiguous.'
    }
    $opening = $Source.IndexOf('{', $position)
    $depth = 0
    for ($index = $opening; $index -lt $Source.Length; ++$index) {
        if ($Source[$index] -eq '{') { ++$depth }
        elseif ($Source[$index] -eq '}') {
            --$depth
            if ($depth -eq 0) { return $Source.Substring($position, $index - $position + 1) }
        }
    }
    throw 'Unbalanced maintained Core routing block.'
}

function Invoke-PosLoadCoreRoutingTests {
    <# Retain exact extracted blocks and run both strict-success and physical-fallback routing branches. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\pos_load_core'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $coreSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core\Src\freertos.c'))
    $emergencyBlock = Get-CoreRoutingBlock $coreSource 'while (aethor_app_pop_emergency_can_frame(&pendingFrame) != 0U)'
    $ordinaryBlock = Get-CoreRoutingBlock $coreSource 'if (aethor_app_next_can_frame(timestampUs,'
    $generated = "/** @file core_pos_route.inc. Verbatim maintained Core routing for offline boundary tests. */`n" +
        "/** @brief Run the maintained emergency loop. */`nstatic void route_emergency(void) { CanFrame pendingFrame;`n" +
        $emergencyBlock + "`n}`n/** @brief Run the maintained ordinary frame path. */`n" +
        "static void route_ordinary(void) { CanFrame pendingFrame; CanTxPriority pendingPriority; uint64_t timestampUs=1000U;`n" +
        $ordinaryBlock + "`n}`n"
    [IO.File]::WriteAllText((Join-Path $buildDirectory 'core_pos_route.inc'), $generated, [Text.UTF8Encoding]::new($false))
    $executable = Join-Path $buildDirectory 'pos_load_core_route.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    Push-Location $projectRoot
    try {
        & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1' `
            '-IApp' '-IApp/Config' '-IApp/Motor' '-IApp/Arm' '-IApp/Protocol' '-IApp/DebugUi' `
            ('-I' + $buildDirectory) 'Tests/host/pos_load_core_route_test.c' -o $executable
        if ($LASTEXITCODE -ne 0) { throw 'Actual Core POS routing compilation failed.' }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw 'Actual Core POS routing assertions failed.' }
    } finally { Pop-Location }
}

Invoke-PosLoadCoreRoutingTests
