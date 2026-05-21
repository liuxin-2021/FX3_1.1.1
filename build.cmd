@echo off
REM Wrapper to run Release\build.cmd from repository root
setlocal
set REPO_DIR=%~dp0
pushd "%REPO_DIR%Release"
if exist build.cmd (
    call build.cmd %*
) else (
    REM fallback to PowerShell script in scripts
    powershell -NoProfile -ExecutionPolicy Bypass -File "..\scripts\build_release.ps1" %*
)
popd
endlocal
