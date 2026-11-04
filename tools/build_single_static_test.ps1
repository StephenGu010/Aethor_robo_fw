<#
.SYNOPSIS
Rebuilds the independent single-axis static LCD image and retains offline receipts.
.DESCRIPTION
Uses only the Keil command-line rebuild action in a hidden process. Fresh clean
logs and all four linked products are required, copied and SHA-256 verified.
The manifest describes software evidence; it never qualifies physical hardware.
#>
param(
    [string]$EvidenceDirectory = '',
    [string]$KeilPath = 'D:\application\keil\UV4\UV4.exe'
)
$ErrorActionPreference = 'Stop'

function Get-WorkspaceEvidencePath {
    <# Validate a new or empty evidence directory without permitting junction escapes. #>
    param([string]$WorkspaceRoot, [string]$RequestedPath)
    $workspacePrefix = $WorkspaceRoot.TrimEnd('\') + '\'
    $evidencePath = [IO.Path]::GetFullPath($RequestedPath)
    if (-not $evidencePath.StartsWith($workspacePrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'EvidenceDirectory must be a child directory inside this workspace.'
    }
    $ancestorPath = $evidencePath
    while ($ancestorPath -and $ancestorPath.Length -ge $WorkspaceRoot.Length) {
        if (Test-Path -LiteralPath $ancestorPath) {
            $ancestor = Get-Item -LiteralPath $ancestorPath -Force
            if (($ancestor.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw 'EvidenceDirectory must not traverse a reparse point.'
            }
            if (-not $ancestor.PSIsContainer) { throw 'EvidenceDirectory must resolve to directories.' }
        }
        $ancestorPath = [IO.Path]::GetDirectoryName($ancestorPath)
    }
    if ((Test-Path -LiteralPath $evidencePath) -and
        @(Get-ChildItem -LiteralPath $evidencePath -Force).Count -ne 0) {
        throw 'EvidenceDirectory already contains results; use a new or empty directory.'
    }
    return $evidencePath
}

function Assert-FreshBuildFile {
    <# Require a nonempty file written during this build, rejecting stale and future receipts. #>
    param([string]$Path, [DateTime]$StartedAt)
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw ('Missing build receipt: ' + $Path)
    }
    $receipt = Get-Item -LiteralPath $Path
    if ($receipt.Length -eq 0 -or $receipt.LastWriteTimeUtc -lt $StartedAt -or
        $receipt.LastWriteTimeUtc -gt [DateTime]::UtcNow) {
        throw ('Build receipt is empty or was not refreshed by this build: ' + $Path)
    }
}

function Copy-VerifiedBuildFile {
    <# Copy one explicit product and verify the source and retained copy share one hash. #>
    param([string]$SourcePath, [string]$DestinationPath)
    $sourceHash = (Get-FileHash -LiteralPath $SourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
    Copy-Item -LiteralPath $SourcePath -Destination $DestinationPath
    $copyHash = (Get-FileHash -LiteralPath $DestinationPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $sourceAfterCopy = (Get-FileHash -LiteralPath $SourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($copyHash -ne $sourceHash -or $sourceAfterCopy -ne $sourceHash) {
        throw ('Build receipt changed during copy: ' + $SourcePath)
    }
    $sourceItem = Get-Item -LiteralPath $SourcePath
    return [ordered]@{
        source = $SourcePath; copy = $DestinationPath; sha256 = $sourceHash
        bytes = $sourceItem.Length; last_write_time_utc = $sourceItem.LastWriteTimeUtc.ToString('o')
    }
}

function Invoke-SingleStaticKeilBuild {
    <# Execute exactly one rebuild, accepting only fresh clean software build evidence. #>
    param([string]$RequestedEvidenceDirectory, [string]$ExecutablePath)
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    $targetName = 'LCD-SingleStatic-Test'
    $projectPath = Join-Path $projectRoot ('MDK-ARM\' + $targetName + '.uvprojx')
    if (-not (Test-Path -LiteralPath $ExecutablePath -PathType Leaf)) { throw 'Keil UV4.exe was not found.' }
    if (-not (Test-Path -LiteralPath $projectPath -PathType Leaf)) { throw 'Independent single-static project was not found.' }
    if (-not $RequestedEvidenceDirectory) {
        $RequestedEvidenceDirectory = Join-Path $projectRoot (
            'output\static_trials_20261019\build_' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffffffZ'))
    } elseif (-not [IO.Path]::IsPathRooted($RequestedEvidenceDirectory)) {
        $RequestedEvidenceDirectory = Join-Path $projectRoot $RequestedEvidenceDirectory
    }
    $evidencePath = Get-WorkspaceEvidencePath -WorkspaceRoot $projectRoot -RequestedPath $RequestedEvidenceDirectory
    [void](New-Item -ItemType Directory -Path $evidencePath -Force)
    $logPath = Join-Path $evidencePath ($targetName + '.log')
    $buildArguments = @('-r', ('"' + $projectPath + '"'), '-t', $targetName,
                        '-j0', '-o', ('"' + $logPath + '"'))
    $projectHash = (Get-FileHash -LiteralPath $projectPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $startedAt = [DateTime]::UtcNow
    $manifest = [ordered]@{
        target = $targetName; project = $projectPath; project_sha256 = $projectHash
        builder_sha256 = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
        executable = $ExecutablePath; arguments = $buildArguments; started_at_utc = $startedAt.ToString('o')
        completed_at_utc = $null; exit_code = $null; passed = $false; failure = $null
        software_only = $true; hardware_qualified = $false; artifacts = [ordered]@{}
    }
    try {
        $process = Start-Process -FilePath $ExecutablePath -ArgumentList $buildArguments `
            -WindowStyle Hidden -PassThru -Wait
        $manifest.exit_code = $process.ExitCode
        if ($null -eq $process.ExitCode -or $process.ExitCode -ne 0) { throw 'Keil rebuild exit code was not zero.' }
        Assert-FreshBuildFile -Path $logPath -StartedAt $startedAt
        $log = Get-Content -LiteralPath $logPath -Raw
        $summaries = [regex]::Matches($log, '(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)')
        if ($summaries.Count -ne 1 -or $summaries[0].Groups[1].Value -ne '0' -or
            $summaries[0].Groups[2].Value -ne '0') {
            throw 'Keil rebuild log does not contain exactly one zero-error, zero-warning summary.'
        }
        if ((Get-FileHash -LiteralPath $projectPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $projectHash) {
            throw 'Keil rebuild changed the maintained project file.'
        }
        $productDirectory = Join-Path $projectRoot ('MDK-ARM\' + $targetName)
        foreach ($extension in @('axf', 'hex', 'map', 'htm')) {
            Assert-FreshBuildFile -Path (Join-Path $productDirectory ($targetName + '.' + $extension)) -StartedAt $startedAt
        }
        foreach ($extension in @('axf', 'hex', 'map', 'htm')) {
            $productName = $targetName + '.' + $extension
            $manifest.artifacts[$extension] = Copy-VerifiedBuildFile `
                -SourcePath (Join-Path $productDirectory $productName) -DestinationPath (Join-Path $evidencePath $productName)
        }
        $mapLogDirectory = Join-Path $projectRoot 'Tests\host\build\keil'
        [void](New-Item -ItemType Directory -Path $mapLogDirectory -Force)
        $manifest.artifacts['log'] = Copy-VerifiedBuildFile -SourcePath $logPath `
            -DestinationPath (Join-Path $mapLogDirectory ($targetName + '.log'))
        $manifest.passed = $true
    }
    catch {
        $manifest.failure = $_.Exception.Message
    }
    $manifest.completed_at_utc = [DateTime]::UtcNow.ToString('o')
    $manifestPath = Join-Path $evidencePath 'build_manifest.json'
    [IO.File]::WriteAllText($manifestPath, ($manifest | ConvertTo-Json -Depth 7) + [Environment]::NewLine,
        (New-Object Text.UTF8Encoding($false)))
    Write-Output ('SINGLE_STATIC_BUILD_MANIFEST=' + $manifestPath)
    if (-not $manifest.passed) { throw $manifest.failure }
    Write-Output ('SINGLE_STATIC_OFFLINE_BUILD_PASSED target=' + $targetName)
}

Invoke-SingleStaticKeilBuild -RequestedEvidenceDirectory $EvidenceDirectory -ExecutablePath $KeilPath
