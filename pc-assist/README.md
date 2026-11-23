# PC assist

Runs hand tracking on your PC instead of the phone. For older or slower phones, and for anyone
whose PC has CPU to spare. **Optional**: without it the phone tracks hands itself, and the PC
side of Phone PCVR still only needs to stream (GT 1030-class is fine).

How it works: the phone sends its hand-camera frames to this app as JPEG over Wi-Fi
(about 5-12 Mbit/s at quality 75). MediaPipe Hand Landmarker runs here and the landmarks go
back to the phone, which does everything else (3D placement, filtering, pinch, sending to
SteamVR through ALVR). The ALVR streamer is not modified.

## Setup (once)

Python 3.10-3.13, then in this folder:

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

The hand model (`hand_landmarker.task`) is read from the Android app's `assets` folder, which
the app's Gradle build downloads. Use `--model <path>` to point elsewhere.

## Use

1. On the phone: Settings → experimental → **Hands: PC assist** on (also in VR: ⚙ → Latency
   & picture), and hand tracking on.
2. On the PC: `.\.venv\Scripts\python pc_assist.py`. It finds the phone by itself when VR
   starts; or pass `--phone <phone IP>` (shown on the phone's home screen under Network).
3. The console prints frames/s, decode/inference time and Wi-Fi use every 5 s. The phone logs
   the same under logcat tag `PVR-Hands` ("PC assist via ...").

Windows may ask whether Python may use the network the first time (automatic discovery listens
for the phone's UDP announcement on port 9951). With `--phone` nothing listens on the PC; the PC
connects out to the phone on TCP 9950.

## Notes

- MediaPipe's Python package runs on the CPU on Windows (no GPU delegate there).
- At most 2 frames wait for a result at a time; a slow PC or network drops frames instead of
  adding delay. If the PC app stops, the phone switches back to its own hand tracking.
- `python test_pc_assist.py` checks the wire format and a loopback session (no MediaPipe needed).
  The wire format is documented in `PcAssistProtocol.java`.
