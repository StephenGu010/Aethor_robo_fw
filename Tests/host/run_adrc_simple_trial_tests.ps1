<# .SYNOPSIS
Compile deterministic pure simple-trial tests without accessing hardware.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$executable = Join-Path $buildDirectory 'adrc_simple_trial_tests.exe'
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
Push-Location $projectRoot
try {
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' '-O1' '-g' '-IApp/Adrc' 'App/Adrc/adrc_simple_trial.c' 'Tests/host/adrc_simple_trial_test_main.c' '-lm' '-o' $executable
    if ($LASTEXITCODE -ne 0) { throw 'Simple trial compilation failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'Simple trial assertion failed.' }
    # Resolve the exact generated snapshot referenced by the active Keil project.
    $projectFile = Join-Path $projectRoot 'MDK-ARM\LCD-MIT-ADRC.uvprojx'
    [xml]$projectDocument = Get-Content -LiteralPath $projectFile -Raw
    $controllerFiles = @($projectDocument.SelectNodes('//File[FileName="adrc_controller.c"]/FilePath'))
    if ($controllerFiles.Count -ne 1) { throw 'Expected exactly one active generated controller in Keil project.' }
    $generatedSource = (Resolve-Path (Join-Path (Split-Path $projectFile) $controllerFiles[0].InnerText)).Path
    $generatedDirectory = Split-Path $generatedSource
    $generatedHeader = Join-Path $generatedDirectory 'adrc_controller.h'
    if (-not (Test-Path -LiteralPath $generatedHeader -PathType Leaf)) { throw 'Active generated header missing.' }
    $manifestPath = Join-Path $generatedDirectory 'generation-manifest.json'
    $generationManifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    foreach ($generatedFile in $generationManifest.generated_files) {
        $generatedFilePath = Join-Path $generatedDirectory $generatedFile.name
        $actualHash = (Get-FileHash -LiteralPath $generatedFilePath -Algorithm SHA256).Hash
        if ($actualHash -ne $generatedFile.sha256) { throw "Active generated snapshot hash mismatch: $($generatedFile.name)" }
    }
    $includeNodes = @($projectDocument.SelectNodes('//Cads/VariousControls/IncludePath'))
    $activeIncludes = @($includeNodes | ForEach-Object { $_.InnerText.Split(';') } | Where-Object { $_ } | ForEach-Object {
        [System.IO.Path]::GetFullPath((Join-Path (Split-Path $projectFile) $_))
    })
    if ($activeIncludes -notcontains $generatedDirectory) { throw 'Generated source directory is absent from active Keil includes.' }
    $generatedExecutable = Join-Path $buildDirectory 'adrc_simple_trial_generated_tests.exe'
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' '-O1' '-g' '-DADRC_SIMPLE_USE_GENERATED=1' '-IApp/Adrc' "-I$generatedDirectory" 'App/Adrc/adrc_simple_trial.c' 'App/Adrc/adrc_generated_adapter.c' $generatedSource 'Tests/host/adrc_simple_trial_test_main.c' '-lm' '-o' $generatedExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Active generated simple-trial compilation failed.' }
    & $generatedExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Active generated simple-trial assertion failed.' }
    Write-Output "Active generated source: $generatedSource"
    Write-Output "Generated C SHA256: $((Get-FileHash -LiteralPath $generatedSource -Algorithm SHA256).Hash)"
    Write-Output "Generated header SHA256: $((Get-FileHash -LiteralPath $generatedHeader -Algorithm SHA256).Hash)"
    Write-Output "Manifest controller model SHA256 (generation provenance): $($generationManifest.model_hashes.adrc_controller)"
    Write-Output 'This run executes existing generated C against synthetic plants; it does not regenerate or test hardware.'
} finally { Pop-Location }
