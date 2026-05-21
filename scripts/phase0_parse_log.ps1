param(
    [Parameter(Mandatory = $true)]
    [string]$LogPath
)

if (-not (Test-Path $LogPath)) {
    Write-Error "Log file not found: $LogPath"
    exit 1
}

$runPattern = 'PHASE0-RUN\[(\d+)\]\s+power->read=(\d+)ms\s+read->stable=(\d+)ms\s+init=(\d+)ms\s+single=(\d+)ms\s+reg100=(\d+)ms\s+reg1000=(\d+)ms\s+fail100=(\d+)\s+fail1000=(\d+)\s+retries=(\d+)\s+timeouts=(\d+)'
$summaryPattern = 'PHASE0-SUMMARY'

$runs = @()
$summaries = @()

Get-Content -Path $LogPath | ForEach-Object {
    $line = $_

    if ($line -match $runPattern) {
        $runs += [pscustomobject]@{
            RunId = [int]$matches[1]
            PowerToReadMs = [int]$matches[2]
            ReadToStableMs = [int]$matches[3]
            InitMs = [int]$matches[4]
            SingleMs = [int]$matches[5]
            Reg100Ms = [int]$matches[6]
            Reg1000Ms = [int]$matches[7]
            Fail100 = [int]$matches[8]
            Fail1000 = [int]$matches[9]
            Retries = [int]$matches[10]
            Timeouts = [int]$matches[11]
            Raw = $line
        }
    }

    if ($line -match $summaryPattern) {
        $summaries += $line
    }
}

Write-Host "Runs parsed: $($runs.Count)"
if ($runs.Count -gt 0) {
    $last = $runs | Sort-Object RunId | Select-Object -Last 1
    Write-Host "Last run id: $($last.RunId)"

    $totalTimeouts = ($runs | Measure-Object -Property Timeouts -Sum).Sum
    $totalFail100 = ($runs | Measure-Object -Property Fail100 -Sum).Sum
    $totalFail1000 = ($runs | Measure-Object -Property Fail1000 -Sum).Sum

    Write-Host "Total timeouts: $totalTimeouts"
    Write-Host "Total fail100: $totalFail100"
    Write-Host "Total fail1000: $totalFail1000"

    $csvPath = [System.IO.Path]::ChangeExtension($LogPath, '.phase0.csv')
    $runs | Sort-Object RunId | Export-Csv -Path $csvPath -NoTypeInformation -Encoding UTF8
    Write-Host "Run table exported: $csvPath"
}

Write-Host "Summary lines found: $($summaries.Count)"
if ($summaries.Count -gt 0) {
    Write-Host "----- Last summary lines -----"
    $summaries | Select-Object -Last 8 | ForEach-Object { Write-Host $_ }
}
