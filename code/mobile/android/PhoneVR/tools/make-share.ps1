# Builds a signed release APK and packs a zip you can send to people.
#   powershell -ExecutionPolicy Bypass -File tools\make-share.ps1 [-SkipBuild]
# Output: C:\coding_shit\phone-pcvr\share\PhonePCVR-v<version>.zip (+ the loose folder)
# Needs the release key from ~/.phonepcvr/signing.properties (see app/build.gradle).
param([switch]$SkipBuild)
$ErrorActionPreference = 'Stop'

. C:\coding_shit\phone-pcvr\tools\env.ps1
$proj = Split-Path $PSScriptRoot -Parent
$signing = if ($env:PHONEPCVR_SIGNING) { $env:PHONEPCVR_SIGNING } else { "$env:USERPROFILE\.phonepcvr\signing.properties" }
if (-not (Test-Path $signing)) { throw "No release key at $signing. Without it the APK would be signed with the public test key." }

$ver = (Get-Content "$proj\version.properties" | Where-Object { $_ -like 'VERSION_NAME=*' }) -replace '^VERSION_NAME=', ''

if (-not $SkipBuild) {
    Push-Location $proj
    try {
        & .\gradlew.bat :app:extractNdk :app:assembleRelease
        if ($LASTEXITCODE -ne 0) { throw "gradle failed ($LASTEXITCODE)" }
    } finally { Pop-Location }
}

$apk = Get-ChildItem "$proj\app\build\outputs\apk\release\*.apk" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $apk) { throw "No release APK found" }

# Check it's signed with our key, not the public PR key.
$apksigner = Get-ChildItem "$env:ANDROID_HOME\build-tools\*\apksigner.bat" | Sort-Object FullName -Descending | Select-Object -First 1
$certs = & $apksigner.FullName verify --print-certs $apk.FullName
if ($LASTEXITCODE -ne 0) { throw "apksigner verify failed" }
if (-not ($certs -match 'CN=Phone PCVR')) { throw "APK isn't signed with the Phone PCVR key:`n$certs" }

$outRoot = 'C:\coding_shit\phone-pcvr\share'
$name = "PhonePCVR-v$ver"
$dir = Join-Path $outRoot $name
if (Test-Path $dir) { Remove-Item -Recurse -Force $dir -Confirm:$false }
New-Item -ItemType Directory -Force $dir | Out-Null

Copy-Item $apk.FullName "$dir\$name.apk"
Copy-Item "$PSScriptRoot\share\HOW-TO-INSTALL.txt" $dir
(Get-Content "$PSScriptRoot\share\Install-to-phone.bat" -Raw) -replace '__APK__', "$name.apk" |
    Set-Content -Encoding ascii "$dir\Install-to-phone.bat"

# PC side: ALVR profile script (+ its helper) and the optional PC assist app.
$pc = New-Item -ItemType Directory -Force "$dir\PC"
Copy-Item "$PSScriptRoot\alvr-pc-profile.ps1", "$PSScriptRoot\steamvr-helpers.ps1" $pc
$assist = New-Item -ItemType Directory -Force "$pc\pc-assist"
$repo = Resolve-Path "$proj\..\..\..\.."
Copy-Item "$repo\pc-assist\pc_assist.py", "$repo\pc-assist\requirements.txt", "$repo\pc-assist\README.md" $assist

$sha = (Get-FileHash "$dir\$name.apk" -Algorithm SHA256).Hash.ToLower()
"$sha  $name.apk" | Set-Content -Encoding ascii "$dir\SHA256.txt"

$zip = "$outRoot\$name.zip"
if (Test-Path $zip) { Remove-Item $zip -Confirm:$false }
Compress-Archive -Path "$dir\*" -DestinationPath $zip

$mb = [math]::Round((Get-Item "$dir\$name.apk").Length / 1MB, 1)
Write-Host "`nDone: $zip"
Write-Host "APK:  $dir\$name.apk ($mb MB, sha256 $sha)"
