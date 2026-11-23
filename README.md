# Phone PCVR

Play SteamVR games on your phone in a cheap headset shell. Head tracking (6DoF) and hand
tracking both run on the phone using its camera, so you don't need controllers or base stations.

**Alpha.** Only tested on a Galaxy S24 Ultra so far. If you try it on another phone, please open
an issue and say how it went.

## What it does

- 6DoF head tracking: ARCore for position, gyro for rotation
- timewarp so turning your head doesn't feel laggy
- hand tracking with MediaPipe (experimental): pinch = trigger, fist = grip, left pinch and drag
  to walk
- PC assist (optional): runs the hand tracking on your PC instead, for older phones
- the PC side is just the normal ALVR streamer, so a weak GPU (GT 1030 class) is fine

## What you need

- Android phone with ARCore support (other phones get rotation only)
- a phone VR shell that leaves the back camera uncovered
- Windows PC with SteamVR and [ALVR Streamer v20.8.0](https://github.com/alvr-org/ALVR/releases/tag/v20.8.0)
  (exactly this version, others won't connect)
- good 5 GHz Wi-Fi. Windows Mobile hotspot with the phone connected straight to the PC works best

## Setup

1. Install SteamVR and unzip ALVR v20.8.0 on the PC. Run `ALVR Dashboard.exe` and go through
   its setup.
2. Install the APK from the Releases page on the phone.
3. Open Phone PCVR, tap Start VR, scan your headset's QR code (or skip).
4. In the ALVR dashboard click Trust next to the phone, then start SteamVR from there.

The release zip has a `HOW-TO-INSTALL.txt` with more detail, and a PowerShell script
(`PC/alvr-pc-profile.ps1`) that sets ALVR up for this app.

If you use hand tracking, turn off ALVR's own hand gestures in the dashboard
(Settings > Headset > Controllers > Gestures), otherwise both fire.

## Known issues

- Samsung phones drop to 60 Hz while the camera is on
- 6DoF pauses in the dark, facing blank walls, or with a hand right in front of the camera
- hands are only seen in front of the camera; low or far-right hands drop out
- the SteamVR dashboard laser doesn't always show up with the gaze pointer

## Building

Android Studio / Gradle project in `code/mobile/android/PhoneVR`. It needs the Android NDK,
Rust and cargo-ndk 3.x for the ALVR client library (`code/mobile/android/PhoneVR/ALVR`, a
modified ALVR v20 client) and a Cardboard SDK build. Then `gradlew :app:assembleRelease`.

## Credits

Based on [PhoneVR](https://github.com/PhoneVR-Developers/PhoneVR) and
[ALVR](https://github.com/alvr-org/ALVR). Uses ARCore, MediaPipe and the Cardboard SDK.
Licence: GPL-3.0 (ALVR parts MIT). Full credits and licences are in the app under About.
