# Setup Cypress FX3 SDK and ARM GCC for current PowerShell session
# Usage: `.\scripts\set_fx3_env.ps1` or `powershell -ExecutionPolicy Bypass -File .\scripts\set_fx3_env.ps1`

# If you use FX3 SDK 1.3 keep this path; otherwise change to your installed SDK path
$fx3Sdk = "C:\\Program Files (x86)\\Cypress\\EZ-USB FX3 SDK\\1.3"

if (-not (Test-Path $fx3Sdk)) {
    Write-Warning "FX3 SDK not found: $fx3Sdk. Please edit this script or install FX3 SDK."
} else {
    $armBin = Join-Path $fx3Sdk "ARM GCC\\bin"
    if (Test-Path $armBin) {
        $env:PATH = "$armBin;" + $env:PATH
        Write-Host "Added to PATH: $armBin"
    } else {
        Write-Warning "ARM GCC bin path not found: $armBin"
    }

    $env:FX3_SDK = $fx3Sdk
    Write-Host "Set FX3_SDK=$fx3Sdk"
}

# 可选：将 elf2img 所在目录加入 PATH，便于手动调用
$elf2img = Join-Path $fx3Sdk "util\\elf2img"
if (Test-Path $elf2img) {
    $env:PATH = "$elf2img;" + $env:PATH
    Write-Host "Added to PATH: $elf2img"
}

Write-Host "FX3 environment initialized for current session."

# Add Git for Windows paths (user provided)
$gitBase = "C:\\Program Files\\Git"
if (Test-Path $gitBase) {
    $gitUsrBin = Join-Path $gitBase "usr\\bin"
    $gitBin = Join-Path $gitBase "bin"
    if (Test-Path $gitUsrBin) {
        $env:PATH = "$gitUsrBin;" + $env:PATH
        Write-Host "Added to PATH: $gitUsrBin (GNU make, coreutils)"
    }
    if (Test-Path $gitBin) {
        $env:PATH = "$gitBin;" + $env:PATH
        Write-Host "Added to PATH: $gitBin"
    }
}

# Optional: project-local tools folder (place a portable make.exe here if desired)
$localTools = Join-Path $PSScriptRoot "tools"
if (Test-Path $localTools) {
    $env:PATH = "$localTools;" + $env:PATH
    Write-Host "Added to PATH: $localTools"
}

# Optional: MSYS2 (provides GNU make via pacman)
$msysUsrBin = "C:\\msys64\\usr\\bin"
if (Test-Path $msysUsrBin) {
    $env:PATH = "$msysUsrBin;" + $env:PATH
    Write-Host "Added to PATH: $msysUsrBin (MSYS2)"
}

# Try to locate GNU make in common locations and add to PATH if found
$makeHintDirs = @(
    "C:\\msys64\\usr\\bin",
    "C:\\msys64\\mingw64\\bin",
    "C:\\msys64\\mingw32\\bin",
    "C:\\mingw64\\bin",
    "C:\\mingw32\\bin",
    "C:\\MinGW\\bin",
    "C:\\Program Files\\Git\\usr\\bin",
    "C:\\Program Files (x86)\\GnuWin32\\bin",
    "C:\\Program Files\\GnuWin32\\bin",
    "C:\\ProgramData\\chocolatey\\bin",
    "C:\\Strawberry\\c\\bin",
    (Join-Path $env:USERPROFILE 'scoop\\shims')
)

# Include any mingw-w64 distributions under Program Files automatically
try {
    $mw = "C:\\Program Files\\mingw-w64"
    if (Test-Path $mw) {
        Get-ChildItem -Path $mw -Directory -ErrorAction SilentlyContinue | ForEach-Object {
            $bin64 = Join-Path $_.FullName 'mingw64\\bin'
            $bin32 = Join-Path $_.FullName 'mingw32\\bin'
            if (Test-Path $bin64) { $makeHintDirs += $bin64 }
            if (Test-Path $bin32) { $makeHintDirs += $bin32 }
        }
    }
} catch {}

# FX3 SDK bundled Eclipse plugin may include a Windows make
try {
    $eclipsePlugins = Join-Path $fx3Sdk 'Eclipse\plugins'
    if (Test-Path $eclipsePlugins) {
        $cands = Get-ChildItem -Path $eclipsePlugins -Directory -Filter 'net.sourceforge.eclipsesdcc.win32*' -ErrorAction SilentlyContinue
        foreach ($c in $cands) {
            $mkDir = Join-Path $c.FullName 'os\win32\x86'
            $mkExe = Join-Path $mkDir 'make.exe'
            if (Test-Path $mkExe) {
                if (-not ($env:PATH -split ';' | Where-Object { $_ -eq $mkDir })) {
                    $env:PATH = "$mkDir;" + $env:PATH
                    Write-Host "Added to PATH: $mkDir (Eclipse bundled make)"
                }
                break
            }
        }
    }
} catch {}
foreach ($d in $makeHintDirs) {
    if (Test-Path $d) {
        $hasMake = (Test-Path (Join-Path $d "make.exe")) -or (Test-Path (Join-Path $d "mingw32-make.exe")) -or (Test-Path (Join-Path $d "gmake.exe"))
        if ($hasMake) {
            if (-not ($env:PATH -split ';' | Where-Object { $_ -eq $d })) {
                $env:PATH = "$d;" + $env:PATH
                Write-Host "Added to PATH: $d (found make)"
            }
            break
        }
    }
}
