@echo off
REM Wrapper to run the repository build script directly (no build.ps1 required)
setlocal
set SCRIPT_DIR=%~dp0
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\scripts\build_release.ps1" %*
endlocal
