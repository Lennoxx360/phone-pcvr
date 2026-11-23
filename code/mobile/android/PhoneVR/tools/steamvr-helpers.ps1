# Shared helpers for the PC-side tools (dot-source this file).
#
# SteamVR and the ALVR dashboard are closed *gracefully*: their windows are asked to close, and
# we wait for the processes to exit. Force-killing SteamVR mid-run can leave its drivers and
# input hooks in a bad state (a long session of force-kills once ended with the
# keyboard unusable until a reboot), so Stop-Process -Force is only a last resort, and it says so.

function Close-SteamVrAndDashboard {
    param([int]$TimeoutSeconds = 20)

    $vrNames = 'vrmonitor', 'vrserver', 'vrcompositor', 'vrdashboard', 'vrwebhelper'
    $all = $vrNames + 'ALVR Dashboard'
    if (-not (Get-Process -Name $all -ErrorAction SilentlyContinue)) { return }

    # Closing the SteamVR status window (vrmonitor) shuts all of SteamVR down cleanly.
    foreach ($name in 'vrmonitor', 'ALVR Dashboard') {
        Get-Process -Name $name -ErrorAction SilentlyContinue | ForEach-Object { [void]$_.CloseMainWindow() }
    }

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline -and (Get-Process -Name $all -ErrorAction SilentlyContinue)) {
        Start-Sleep -Milliseconds 500
    }

    $left = Get-Process -Name $all -ErrorAction SilentlyContinue
    if ($left) {
        Write-Warning ("Still running after $TimeoutSeconds s, force-closing: " +
            (($left | Select-Object -ExpandProperty ProcessName -Unique) -join ', '))
        $left | Stop-Process -Force -ErrorAction SilentlyContinue
        Start-Sleep 2
    }
}

function Start-SteamVr {
    # steam:// URLs don't launch SteamVR on this PC; start it directly.
    $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam').SteamPath
    Start-Process (Join-Path $steam 'steamapps/common/SteamVR/bin/win64/vrstartup.exe')
}
