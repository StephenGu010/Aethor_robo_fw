@echo off
rem Aethor Debug Studio local launcher; no automatic serial connection.
where pwsh.exe >nul 2>nul
if errorlevel 1 (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-Studio.ps1"
) else (
  pwsh.exe -NoProfile -File "%~dp0Start-Studio.ps1"
)
if errorlevel 1 pause
