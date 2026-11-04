<#
.SYNOPSIS
Run memory-only ADRC client/session checks in the existing MATLAB AutomationServer.
.DESCRIPTION
Invoke with Windows PowerShell 5.1. Never starts MATLAB, opens devices or changes
firmware. Preserves the desktop path and working directory and retains unique evidence.
#>
[CmdletBinding()]
param([string]$BlockedSoftwareReportPath = '', [switch]$IncludeCandidateModel)
$ErrorActionPreference = 'Stop'

function ConvertTo-SessionMatlabLiteral {
    <# Quote a path as a MATLAB literal without interpreting its characters as code. #>
    param([string]$Value)
    return "'" + $Value.Replace('\', '/').Replace("'", "''") + "'"
}

function Get-SessionInputHashes {
    <# Bind the client and runner sources to this exact memory-only test execution. #>
    param([string]$Root)
    $files = @(Get-ChildItem -LiteralPath (Join-Path $Root 'Models\AdrcClient') -File -Filter '*.m')
    $files += Get-Item -LiteralPath $PSCommandPath
    if ($BlockedSoftwareReportPath) { $files += Get-Item -LiteralPath $BlockedSoftwareReportPath }
    if ($IncludeCandidateModel) { $files += Get-ChildItem -LiteralPath (Join-Path $Root 'Models\Adrc') -File | Where-Object { $_.Extension -in @('.m', '.c') } }
    foreach ($file in ($files | Sort-Object FullName)) {
        [ordered]@{ path = $file.FullName; sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash }
    }
}

if ($PSVersionTable.PSVersion.Major -ne 5) {
    throw 'Use Windows PowerShell 5.1 -NoProfile -ExecutionPolicy Bypass -File tools/verify_adrc_client_session.ps1'
}
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$output = Join-Path $root ('output\adrc\client\session_' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void](New-Item -ItemType Directory -Path $output)
$sourceBefore = @(Get-SessionInputHashes $root)
$jobPath = Join-Path $output 'run_session_job.m'
$clientLiteral = ConvertTo-SessionMatlabLiteral (Join-Path $root 'Models\AdrcClient')
$outputLiteral = ConvertTo-SessionMatlabLiteral $output
$blockedReportLiteral = ConvertTo-SessionMatlabLiteral $BlockedSoftwareReportPath
$modelsLiteral = ConvertTo-SessionMatlabLiteral (Join-Path $root 'Models\Adrc')
$includeCandidateLiteral = if ($IncludeCandidateModel) { 'true' } else { 'false' }
$jobText = @"
% RUN_SESSION_JOB Keep all temporary variables local while testing without devices.
runSessionJob();
function runSessionJob()
%RUNSESSIONJOB Restore the desktop state and persist success or the complete failure.
originalPath = path; originalDirectory = pwd; originalRandom = rng; originalEnvironment = getenv('PATH');
fileGen = Simulink.fileGenControl('getConfig');
cleanup = onCleanup(@()restoreSession(originalPath, originalDirectory, originalRandom, originalEnvironment, fileGen));
outputDirectory = $outputLiteral;
status = struct('completed', false, 'success', false, 'hardwareOpened', false, 'error', '');
try
    addpath($clientLiteral);
    clear run_adrc_client_tests run_adrc_session;
    report = run_adrc_client_tests(outputDirectory, $blockedReportLiteral);
    clear run_adrc_identification_tests fit_adrc_pulses run_adrc_identified_session;
    [identificationReport, fixtureData] = run_adrc_identification_tests();
    saveJson(fullfile(outputDirectory, 'identification_report.json'), identificationReport);
    assert(identificationReport.passed && ~identificationReport.hardwareOpened);
    if $includeCandidateLiteral
        addpath($modelsLiteral);
        setenv('PATH', ['D:\application\WinLibs\mingw64\bin;' originalEnvironment]);
        clear run_adrc_pipeline build_adrc_models;
        candidateReport = run_adrc_identified_session([], fixtureData.training, fixtureData.validation, ...
            fixtureData.contract, struct([]), fullfile(outputDirectory, 'identified_candidate'), '');
        assert(candidateReport.offlineComplete && ~candidateReport.completed && ...
            strcmp(candidateReport.phase, 'blocked_no_hardware'), candidateReport.error);
    end
    saveJson(fullfile(outputDirectory, 'client_report.json'), report);
    assert(report.passed && ~report.hardwareOpened, 'Unexpected test result or hardware access.');
    status.success = true;
catch failure
    status.error = getReport(failure, 'extended', 'hyperlinks', 'off');
    disp(status.error);
end
clear cleanup;
restoredFileGen = Simulink.fileGenControl('getConfig');
status.sessionRestored = strcmp(path, originalPath) && strcmp(pwd, originalDirectory) && ...
    isequal(rng, originalRandom) && strcmp(getenv('PATH'), originalEnvironment) && ...
    strcmp(restoredFileGen.CacheFolder, fileGen.CacheFolder) && ...
    strcmp(restoredFileGen.CodeGenFolder, fileGen.CodeGenFolder) && ...
    isequal(restoredFileGen.CodeGenFolderStructure, fileGen.CodeGenFolderStructure);
status.completed = true;
saveJson(fullfile(outputDirectory, 'job_status.json'), status);
end
function restoreSession(originalPath, originalDirectory, originalRandom, originalEnvironment, fileGen)
%RESTORESESSION Restore file generation before its side effects on the search path.
rng(originalRandom); setenv('PATH', originalEnvironment);
Simulink.fileGenControl('set', 'CacheFolder', fileGen.CacheFolder, ...
    'CodeGenFolder', fileGen.CodeGenFolder, 'CodeGenFolderStructure', fileGen.CodeGenFolderStructure);
path(originalPath); cd(originalDirectory);
end
function saveJson(filePath, value)
%SAVEJSON Retain each test result with explicit UTF-8 encoding.
file = fopen(filePath, 'w', 'n', 'UTF-8'); assert(file >= 0);
cleanup = onCleanup(@()fclose(file));
fprintf(file, '%s', jsonencode(value));
end
"@
[IO.File]::WriteAllText($jobPath, $jobText, [Text.UTF8Encoding]::new($false))
$failureMessage = $null
try {
    $matlab = [Runtime.InteropServices.Marshal]::GetActiveObject('Matlab.Application')
    $console = $matlab.Execute('run(' + (ConvertTo-SessionMatlabLiteral $jobPath) + ');')
    [IO.File]::WriteAllText((Join-Path $output 'console.txt'), [string]$console, [Text.UTF8Encoding]::new($false))
    Write-Output $console
    $status = Get-Content -LiteralPath (Join-Path $output 'job_status.json') -Raw | ConvertFrom-Json
    if (-not $status.completed -or -not $status.success -or -not $status.sessionRestored -or $status.hardwareOpened) {
        throw 'Memory-only client/session verification did not pass; inspect job_status.json.'
    }
}
catch { $failureMessage = $_.Exception.Message }
$sourceAfter = @(Get-SessionInputHashes $root)
$unchanged = (($sourceBefore | ConvertTo-Json -Depth 4 -Compress) -ceq ($sourceAfter | ConvertTo-Json -Depth 4 -Compress))
$manifest = [ordered]@{ passed = ($null -eq $failureMessage -and $unchanged); hardwareOpened = $false
    sourcesUnchanged = $unchanged; sourceHashes = $sourceBefore; failure = $failureMessage
    artifacts = @(Get-ChildItem -LiteralPath $output -File -Recurse | ForEach-Object {
        [ordered]@{ path = $_.FullName; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    }) }
[IO.File]::WriteAllText((Join-Path $output 'manifest.json'), ($manifest | ConvertTo-Json -Depth 6), [Text.UTF8Encoding]::new($false))
Write-Output ('ADRC_CLIENT_SESSION_MANIFEST=' + (Join-Path $output 'manifest.json'))
if (-not $manifest.passed) { throw 'ADRC client/session verification failed.' }
