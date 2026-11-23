<#
.SYNOPSIS
  One repeatable stream test: (re)starts the phone app with the given options, makes sure the
  ALVR dashboard and SteamVR run, streams for -Seconds, then saves the app/ALVR lines of logcat
  and ALVR's statistics into docs/logs/<date>/<Label>/ and prints a summary
  (tools/stream_summary.py).

  SteamVR is started once and reused across runs. -RestartSteamVr closes it gracefully first
  (steamvr-helpers.ps1); avoid restarting it more than needed.

.EXAMPLE
  .\stream-test.ps1 -Label stock -PhoneArgs '--es stream_profile stock' -Seconds 60
.EXAMPLE
  .\stream-test.ps1 -Label pacing -PhoneArgs '--es stream_profile stock --ez frame_pacing true'
#>
param(
    [Parameter(Mandatory = $true)][string]$Label,
    [string]$PhoneArgs = '',
    [int]$Seconds = 60,
    [switch]$RestartSteamVr,
    [switch]$Install
)
$ErrorActionPreference = 'Continue'
. (Join-Path $PSScriptRoot 'steamvr-helpers.ps1')
$ws = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..\..\..')).Path
$proj = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$streamer = Join-Path $ws 'alvr-streamer-v20.8.0'
$pkg = 'io.github.lennoxx360.phonepcvr'
$out = Join-Path $ws ("PhoneVR\docs\logs\" + (Get-Date -Format 'yyyy-MM-dd') + "\$Label")
New-Item -ItemType Directory -Force $out | Out-Null

if ($Install) {
    $apk = Get-ChildItem "$proj\app\build\outputs\apk\release\*.apk" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    adb install -r $apk.FullName | Out-Null
    Start-Sleep 2
}
if ($RestartSteamVr) {
    Close-SteamVrAndDashboard
}
if (-not (Get-Process -Name 'ALVR Dashboard' -ErrorAction SilentlyContinue)) {
    Start-Process -FilePath (Join-Path $streamer 'ALVR Dashboard.exe') -WorkingDirectory $streamer
    Start-Sleep 4
}

adb shell am force-stop $pkg | Out-Null
adb logcat -c
adb shell "am start -n $pkg/.HomeActivity --ez start_vr true $PhoneArgs" | Out-Null
$t0 = Get-Date
if (-not (Get-Process -Name 'vrserver' -ErrorAction SilentlyContinue)) {
    Start-Sleep 5
    Start-SteamVr
}
Start-Sleep -Seconds $Seconds

$raw = Join-Path $out 'logcat-full.tmp'
cmd /c "adb logcat -d > `"$raw`""
Copy-Item (Join-Path $streamer 'session_log.txt') (Join-Path $out 'session_log.txt') -ErrorAction SilentlyContinue
"phone args: $PhoneArgs" | Set-Content (Join-Path $out 'run.txt')
"started: $t0" | Add-Content (Join-Path $out 'run.txt')
# Keep only what the analysis uses (full logcat/session_log are 10-30 MB per run, ~99 % noise).
python (Join-Path $PSScriptRoot 'stream_summary.py') --trim $out $raw
Remove-Item $raw -ErrorAction SilentlyContinue
python (Join-Path $PSScriptRoot 'stream_summary.py') $out 10
