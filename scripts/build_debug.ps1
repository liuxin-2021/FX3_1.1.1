$ErrorActionPreference = 'Stop'


. "$PSScriptRoot\set_fx3_env.ps1"

$makeCmd = $null
$candidates = @()
$found = $false

# 1) Try PATH-resolved commands
$mk = Get-Command make -ErrorAction SilentlyContinue
if ($mk) { $makeCmd = $mk.Source; $found = $true }
if (-not $found) {
    $mk = Get-Command mingw32-make -ErrorAction SilentlyContinue
    if ($mk) { $makeCmd = $mk.Source; $found = $true }
}
if (-not $found) {
    $mk = Get-Command gmake -ErrorAction SilentlyContinue
    if ($mk) { $makeCmd = $mk.Source; $found = $true }
}

# 2) Probe common locations if still not found
if (-not $found) {
    $probeDirs = @(
        $(Join-Path $PSScriptRoot 'tools'),
        'C:\\msys64\\usr\\bin',
        'C:\\Program Files\\Git\\usr\\bin',
        'C:\\Program Files (x86)\\GnuWin32\\bin',
        'C:\\Program Files\\GnuWin32\\bin',
        'C:\\ProgramData\\chocolatey\\bin',
        'C:\\Strawberry\\c\\bin'
    )
    $names = @('make.exe','mingw32-make.exe','gmake.exe')
    foreach ($dir in $probeDirs) {
        foreach ($n in $names) {
            $p = Join-Path $dir $n
            if (Test-Path $p) { $makeCmd = $p; $found = $true; break }
        }
        if ($found) { break }
    }
}

if (-not $found) {
    Write-Host 'GNU make not found (make/mingw32-make/gmake)'
    Write-Host 'Please install GNU make, or place make.exe into scripts\\tools\\ and retry.'
    exit 127
}

Push-Location (Join-Path $PSScriptRoot '..' | Resolve-Path)
Push-Location .\Debug

& $makeCmd clean
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $makeCmd all
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

New-Item -ItemType Directory -Force -Path ..\out | Out-Null
Copy-Item -Force ios.elf, ios.img -Destination ..\out\

Pop-Location
Pop-Location

Write-Host 'Debug Build & Package completed.'