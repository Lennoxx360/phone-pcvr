<#
.SYNOPSIS
  Applies a Phone PCVR streamer profile to ALVR v20.8.0's session.json (the PC half of the
  phone app's "Streaming: latency & picture" profiles).

.DESCRIPTION
  Closes SteamVR and the ALVR dashboard gracefully (session.json may only be edited while
  they're closed; see steamvr-helpers.ps1),
  backs session.json up next to itself (session.json.bak-<profile>-<time>), then changes only the
  keys listed below. Run with -Profile Restore to put the newest backup back.

  Profiles:
    GoodWifiLowEndPc  Low PC load: 896-wide eyes, 20 Mbps constant, H.264, fastest NVENC settings.
                      Add -SoftwareEncoder on PCs without NVENC (e.g. GT 1030): x264 on the CPU.
    BadWifiHighEndPc  Weak link: adaptive bitrate (8-30 Mbps) that backs off when the network
                      lags, a small streamer-side frame queue and send buffer so a Wi-Fi stall
                      can't build up seconds of backlog, keyframe resend on loss.
    Balanced          The settings used on the dev setup (1072-wide, 30 Mbps constant, TCP).
    Restore           Copies the newest session.json.bak-* back.

  Never changes networking (adapters, hotspot, Wi-Fi). TCP stays TCP: UDP was tested on
  2026-11-22 and was worse on this setup.

.EXAMPLE
  .\alvr-pc-profile.ps1 -Profile BadWifiHighEndPc
.EXAMPLE
  .\alvr-pc-profile.ps1 -Profile GoodWifiLowEndPc -SoftwareEncoder
#>
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('GoodWifiLowEndPc', 'BadWifiHighEndPc', 'Balanced', 'Restore')]
    [string]$Profile,
    [string]$StreamerDir = '',
    [switch]$SoftwareEncoder,
    [switch]$NoClose,
    [switch]$WhatIf
)

$ErrorActionPreference = 'Stop'
if (-not $StreamerDir) {
    # The folder with "ALVR Dashboard.exe": the current folder, next to this script, or the
    # developer workspace (tools -> PhoneVR -> android -> mobile -> code -> repo -> workspace).
    $candidates = @(
        (Get-Location).Path,
        $PSScriptRoot,
        (Join-Path $PSScriptRoot '..'),
        (Join-Path $PSScriptRoot '..\..\..\..\..\..\alvr-streamer-v20.8.0'))
    $StreamerDir = $candidates | Where-Object { Test-Path (Join-Path $_ 'session.json') } | Select-Object -First 1
    if (-not $StreamerDir) {
        throw ('Can''t find ALVR''s session.json. Run the ALVR dashboard once, then pass its folder: ' +
            '-StreamerDir "C:\path\to\alvr_streamer_windows"')
    }
}
$StreamerDir = (Resolve-Path $StreamerDir).Path
$session = Join-Path $StreamerDir 'session.json'
if (-not (Test-Path $session)) { throw "No session.json in $StreamerDir" }

if (-not $NoClose -and -not $WhatIf) {
    # session.json is rewritten by the dashboard on exit, so both must be closed first.
    . (Join-Path $PSScriptRoot 'steamvr-helpers.ps1')
    Close-SteamVrAndDashboard
}

if ($Profile -eq 'Restore') {
    $bak = Get-ChildItem $StreamerDir -Filter 'session.json.bak-*' | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $bak) { throw 'No session.json.bak-* to restore.' }
    if ($WhatIf) { "Would restore $($bak.Name)"; return }
    Copy-Item $bak.FullName $session -Force
    "Restored $($bak.Name)"
    return
}

$json = Get-Content $session -Raw | ConvertFrom-Json
$v = $json.session_settings.video
$c = $json.session_settings.connection
$changes = New-Object System.Collections.Generic.List[string]

function Set-Value($obj, [string]$name, $value, [string]$label) {
    $old = $obj.$name
    $oldText = ($old | ConvertTo-Json -Compress -Depth 5)
    $newText = ($value | ConvertTo-Json -Compress -Depth 5)
    if ($oldText -ne $newText) {
        $obj.$name = $value
        $changes.Add("$label : $oldText -> $newText")
    }
}

function Set-Width([int]$width) {
    foreach ($key in 'transcoding_view_resolution', 'emulated_headset_view_resolution') {
        $r = $v.$key
        Set-Value $r 'variant' 'Absolute' "video.$key.variant"
        Set-Value $r.Absolute 'width' $width "video.$key.width"
    }
}

# Common to every profile: H.264 (fastest decode on phones), TCP, foveation off, and
# total frame width <= 4096 (the phone's H.264 decoder limit; 4288 gave broken-LCD blocks).
Set-Value $v.preferred_codec 'variant' 'H264' 'video.preferred_codec'
Set-Value $c.stream_protocol 'variant' 'Tcp' 'connection.stream_protocol'
Set-Value $v.foveated_encoding 'enabled' $false 'video.foveated_encoding.enabled'

switch ($Profile) {
    'GoodWifiLowEndPc' {
        Set-Width 896
        Set-Value $v.bitrate.mode 'variant' 'ConstantMbps' 'video.bitrate.mode'
        Set-Value $v.bitrate.mode 'ConstantMbps' 20 'video.bitrate.ConstantMbps'
        Set-Value $v.encoder_config.nvenc.quality_preset 'variant' 'P1' 'nvenc.quality_preset'
        Set-Value $v.encoder_config.nvenc.multi_pass 'variant' 'Disabled' 'nvenc.multi_pass'
        Set-Value $v.encoder_config.software 'force_software_encoding' ([bool]$SoftwareEncoder) 'software.force_software_encoding'
        Set-Value $v 'max_buffering_frames' 1.0 'video.max_buffering_frames'
        Set-Value $c 'max_queued_server_video_frames' 1024 'connection.max_queued_server_video_frames'
        Set-Value $c.server_send_buffer_bytes 'variant' 'Maximum' 'connection.server_send_buffer_bytes'
        Set-Value $c 'avoid_video_glitching' $false 'connection.avoid_video_glitching'
        Set-Value $c 'aggressive_keyframe_resend' $false 'connection.aggressive_keyframe_resend'
    }
    'BadWifiHighEndPc' {
        Set-Width 1072
        Set-Value $v.bitrate.mode 'variant' 'Adaptive' 'video.bitrate.mode'
        $a = $v.bitrate.mode.Adaptive
        Set-Value $a.max_bitrate_mbps 'enabled' $true 'bitrate.Adaptive.max_bitrate_mbps.enabled'
        Set-Value $a.max_bitrate_mbps 'content' 30 'bitrate.Adaptive.max_bitrate_mbps'
        Set-Value $a.min_bitrate_mbps 'enabled' $true 'bitrate.Adaptive.min_bitrate_mbps.enabled'
        Set-Value $a.min_bitrate_mbps 'content' 8 'bitrate.Adaptive.min_bitrate_mbps'
        Set-Value $a.max_network_latency_ms 'enabled' $true 'bitrate.Adaptive.max_network_latency_ms.enabled'
        Set-Value $a.max_network_latency_ms 'content' 20 'bitrate.Adaptive.max_network_latency_ms'
        Set-Value $v.encoder_config.nvenc.quality_preset 'variant' 'P1' 'nvenc.quality_preset'
        Set-Value $v.encoder_config.nvenc.multi_pass 'variant' 'QuarterResolution' 'nvenc.multi_pass'
        Set-Value $v.encoder_config.software 'force_software_encoding' $false 'software.force_software_encoding'
        Set-Value $v 'max_buffering_frames' 1.5 'video.max_buffering_frames'
        # Don't let a stall pile up frames on the PC: drop (and resend a keyframe) instead.
        Set-Value $c 'max_queued_server_video_frames' 6 'connection.max_queued_server_video_frames'
        Set-Value $c.server_send_buffer_bytes 'variant' 'Custom' 'connection.server_send_buffer_bytes.variant'
        Set-Value $c.server_send_buffer_bytes 'Custom' 250000 'connection.server_send_buffer_bytes.Custom'
        Set-Value $c 'avoid_video_glitching' $true 'connection.avoid_video_glitching'
        Set-Value $c 'aggressive_keyframe_resend' $true 'connection.aggressive_keyframe_resend'
    }
    'Balanced' {
        Set-Width 1072
        Set-Value $v.bitrate.mode 'variant' 'ConstantMbps' 'video.bitrate.mode'
        Set-Value $v.bitrate.mode 'ConstantMbps' 30 'video.bitrate.ConstantMbps'
        Set-Value $v.encoder_config.nvenc.quality_preset 'variant' 'P1' 'nvenc.quality_preset'
        Set-Value $v.encoder_config.nvenc.multi_pass 'variant' 'QuarterResolution' 'nvenc.multi_pass'
        Set-Value $v.encoder_config.software 'force_software_encoding' $false 'software.force_software_encoding'
        Set-Value $v 'max_buffering_frames' 2.0 'video.max_buffering_frames'
        Set-Value $c 'max_queued_server_video_frames' 1024 'connection.max_queued_server_video_frames'
        Set-Value $c.server_send_buffer_bytes 'variant' 'Maximum' 'connection.server_send_buffer_bytes.variant'
        Set-Value $c 'avoid_video_glitching' $false 'connection.avoid_video_glitching'
        Set-Value $c 'aggressive_keyframe_resend' $false 'connection.aggressive_keyframe_resend'
    }
}

if ($changes.Count -eq 0) { "Already on $Profile; nothing changed."; return }
"Profile $Profile :"
$changes | ForEach-Object { "  $_" }
if ($WhatIf) { '(WhatIf: not written)'; return }

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
Copy-Item $session (Join-Path $StreamerDir "session.json.bak-$Profile-$stamp")
# Depth 100: session.json nests deeply; the default depth (2) would truncate it.
# UTF-8 without a BOM: ALVR's JSON parser rejects a BOM (PowerShell 5.1's "utf8" adds one).
[System.IO.File]::WriteAllText($session, ($json | ConvertTo-Json -Depth 100), (New-Object System.Text.UTF8Encoding $false))
"Written. Backup: session.json.bak-$Profile-$stamp. Start the ALVR dashboard, then SteamVR from it."
