"""Phone PCVR - PC assist: runs hand tracking for the phone.

The phone (with "Hands: PC assist" on) sends its hand-camera frames as H.264 (hardware
encoder) or JPEG; this app runs
MediaPipe Hand Landmarker on them and sends the 21 landmarks per hand back. The phone does
everything else (3D placement, filtering, pinch, sending to SteamVR through ALVR), so the ALVR
streamer stays stock.

    python pc_assist.py                 # find the phone automatically (UDP beacon)
    python pc_assist.py --phone 192.168.137.131
    python pc_assist.py --fake          # no MediaPipe: answers "no hands" (protocol test)

Wire format: see PcAssistProtocol.java (phone side). Little endian, TCP, the PC connects.
"""

import argparse
import faulthandler
import os
import socket
import struct
import sys
import threading
import time
import traceback

VERSION = 2
PORT = 9950
BEACON_PORT = 9951
SKIPPED = 255
CODEC_JPEG, CODEC_H264 = 0, 1
FLAG_KEYFRAME = 1
# 'F', id, width, height, numHands, codec, flags, length
FRAME_HEADER = struct.Struct("<cIHHBBBI")
RESULT_HEADER = struct.Struct("<cIfB")  # 'R', id, pcInferMs, handCount
HAND = struct.Struct("<B" + "f" * (21 * 2) + "f" * (21 * 3))

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(HERE, "pc_assist.log")
_log_file = None


def log(msg):
    """Console + pc_assist.log (tracebacks too, so a failure can be diagnosed later)."""
    line = time.strftime("%H:%M:%S ") + msg
    print(msg, flush=True)
    if _log_file:
        _log_file.write(line + "\n")
        _log_file.flush()


def log_exception(what):
    log(f"{what}:\n{traceback.format_exc()}")
DEFAULT_MODEL = os.path.join(
    HERE, "..", "code", "mobile", "android", "PhoneVR", "app", "src", "main", "assets",
    "hand_landmarker.task")


def encode_result(frame_id, pc_ms, hands):
    """hands: list of (labelled_left, image[21][2], world[21][3]); None = frame skipped."""
    if hands is None:
        return RESULT_HEADER.pack(b"R", frame_id, pc_ms, SKIPPED)
    out = [RESULT_HEADER.pack(b"R", frame_id, pc_ms, len(hands))]
    for left, image, world in hands:
        flat = [v for p in image for v in p] + [v for p in world for v in p]
        out.append(HAND.pack(1 if left else 0, *flat))
    return b"".join(out)


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("phone closed the connection")
        buf += chunk
    return bytes(buf)


def find_phone(timeout_s):
    """Waits for the phone's UDP beacon; returns its IP or None."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", BEACON_PORT))
    s.settimeout(1.0)
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline:
            try:
                data, addr = s.recvfrom(256)
            except socket.timeout:
                continue
            parts = data.decode("ascii", "replace").split()
            if len(parts) == 3 and parts[0] == "PCVR-ASSIST":
                if int(parts[1]) != VERSION:
                    print(f"phone at {addr[0]} speaks version {parts[1]}, this app {VERSION}")
                    continue
                return addr[0]
    finally:
        s.close()
    return None


class H264Decoder:
    """FFmpeg H.264 decoder (PyAV), one frame out per packet in: single thread and low_delay,
    so nothing is held back for reordering (the phone's baseline stream has no B-frames)."""

    def __init__(self):
        import av
        self.ctx = av.CodecContext.create("h264", "r")
        self.ctx.thread_count = 1
        self.ctx.options = {"flags": "low_delay"}
        self.have_keyframe = False

    def decode(self, data, keyframe):
        """Returns an RGB numpy image, or None (no keyframe yet / decode error)."""
        import av
        if keyframe:
            self.have_keyframe = True
        if not self.have_keyframe:
            return None
        try:
            frames = self.ctx.decode(av.Packet(data))
        except av.error.FFmpegError:
            self.have_keyframe = False  # wait for the next keyframe
            return None
        if not frames:
            return None
        return frames[-1].to_ndarray(format="rgb24")


class Tracker:
    """MediaPipe Hand Landmarker (VIDEO mode). Recreated when the phone asks for 1 vs 2 hands."""

    def __init__(self, model, fake):
        self.fake = fake
        self.model = model
        self.num_hands = 0
        self.landmarker = None
        self.last_ts = -1
        if not fake:
            import cv2  # noqa: F401  (fail early with a clear message)
            import mediapipe  # noqa: F401
            if not os.path.exists(model):
                sys.exit(f"hand model not found: {model}\n"
                         "Build the Android app once (it downloads it) or pass --model.")

    def _create(self, num_hands):
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision
        opts = vision.HandLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=self.model),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=num_hands,
            # Same as the phone (HandTracker.java): strict detection, relaxed tracking.
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.3,
            min_tracking_confidence=0.3)
        if self.landmarker is not None:
            self.landmarker.close()
        self.landmarker = vision.HandLandmarker.create_from_options(opts)
        self.num_hands = num_hands
        print(f"MediaPipe ready ({num_hands} hand{'s' if num_hands > 1 else ''}, CPU)")

    @staticmethod
    def decode_jpeg(jpeg):
        import cv2
        import numpy as np
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        return None if bgr is None else cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def detect(self, rgb, num_hands):
        """rgb: decoded image (numpy HxWx3) or None. Returns (hands, infer_ms)."""
        if self.fake or rgb is None:
            return [], 0.0
        import mediapipe as mp
        if num_hands != self.num_hands:
            self._create(max(1, min(2, num_hands)))
        t1 = time.perf_counter()
        ts = int(time.monotonic() * 1000)
        if ts <= self.last_ts:
            ts = self.last_ts + 1
        self.last_ts = ts
        res = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
        t2 = time.perf_counter()
        hands = []
        for i in range(min(len(res.hand_landmarks), len(res.hand_world_landmarks), 2)):
            cats = res.handedness[i] if i < len(res.handedness) else []
            left = bool(cats) and cats[0].category_name == "Left"
            image = [(p.x, p.y) for p in res.hand_landmarks[i]]
            world = [(p.x, p.y, p.z) for p in res.hand_world_landmarks[i]]
            hands.append((left, image, world))
        return hands, (t2 - t1) * 1000


def serve(sock, tracker):
    """One phone connection. The reader thread decodes every frame as it arrives (H.264 needs
    every packet, in order) and keeps only the newest decoded image; MediaPipe runs on that.
    Frames that are replaced or can't be decoded are answered as skipped, so the phone's
    in-flight count stays right."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.sendall(b"H" + bytes([VERSION]))
    hello = recv_exact(sock, 2)
    if hello[:1] != b"P":
        raise ConnectionError("not a Phone PCVR phone")
    if hello[1] != VERSION:
        raise ConnectionError(f"phone speaks version {hello[1]}, this app {VERSION}")

    send_lock = threading.Lock()
    cond = threading.Condition()
    latest = {"frame": None, "closed": False}

    def send(data):
        with send_lock:
            sock.sendall(data)

    decoder = None

    def reader():
        nonlocal decoder
        try:
            while True:
                head = recv_exact(sock, FRAME_HEADER.size)
                tag, fid, w, h, num_hands, codec, flags, length = FRAME_HEADER.unpack(head)
                if tag != b"F" or length > 8_000_000:
                    raise ConnectionError(f"bad frame header {head!r}")
                data = recv_exact(sock, length)
                t_in = time.perf_counter()
                rgb = None
                if not tracker.fake:
                    try:
                        if codec == CODEC_H264:
                            if decoder is None:
                                decoder = H264Decoder()
                            rgb = decoder.decode(data, bool(flags & FLAG_KEYFRAME))
                        else:
                            rgb = Tracker.decode_jpeg(data)
                    except Exception:  # noqa: BLE001 - one bad frame must not stop the session
                        log_exception(f"decoding frame {fid} failed")
                        decoder = None  # start over at the next keyframe
                        rgb = None
                    if rgb is None:
                        send(encode_result(fid, 0.0, None))
                        stats["undecodable"] += 1
                        continue
                dec_ms = (time.perf_counter() - t_in) * 1000
                with cond:
                    old = latest["frame"]
                    latest["frame"] = (fid, w, h, num_hands, rgb, t_in, dec_ms, codec, length)
                    cond.notify()
                if old is not None:
                    send(encode_result(old[0], 0.0, None))
                    stats["skipped"] += 1
        except Exception as e:  # noqa: BLE001 - end the session cleanly, whatever happened
            if not isinstance(e, (ConnectionError, OSError)):
                log_exception("receive thread failed")
            with cond:
                latest["closed"] = True
                latest["error"] = str(e)
                cond.notify()

    stats = {"frames": 0, "skipped": 0, "undecodable": 0, "hands": 0, "decode": 0.0,
             "infer": 0.0, "bytes": 0, "wait": 0.0, "size": "", "codec": ""}
    t_stats = time.time()
    threading.Thread(target=reader, daemon=True).start()
    while True:
        with cond:
            while latest["frame"] is None and not latest["closed"]:
                cond.wait()
            if latest["closed"]:
                raise ConnectionError(latest.get("error", "closed"))
            fid, w, h, num_hands, rgb, t_in, dec_ms, codec, length = latest["frame"]
            latest["frame"] = None
        t0 = time.perf_counter()
        try:
            hands, inf_ms = tracker.detect(rgb, num_hands)
        except Exception:  # noqa: BLE001
            log_exception(f"hand tracking on frame {fid} failed")
            send(encode_result(fid, 0.0, None))
            continue
        send(encode_result(fid, dec_ms + inf_ms, hands))
        stats["frames"] += 1
        stats["hands"] += len(hands)
        stats["decode"] += dec_ms
        stats["infer"] += inf_ms
        stats["wait"] += (t0 - t_in) * 1000 - dec_ms
        stats["bytes"] += length
        stats["size"] = f"{w}x{h}"
        stats["codec"] = "H.264" if codec == CODEC_H264 else "JPEG"
        now = time.time()
        if now - t_stats >= 5:
            n = max(1, stats["frames"])
            dt = now - t_stats
            log(f"{stats['frames'] / dt:5.1f} frames/s {stats['size']} {stats['codec']:5s} "
                  f"{stats['bytes'] * 8 / 1e6 / dt:4.1f} Mbit/s  "
                  f"decode {stats['decode'] / n:4.1f} ms  infer {stats['infer'] / n:5.1f} ms  "
                  f"queued {stats['wait'] / n:4.1f} ms  hands/frame {stats['hands'] / n:.2f}  "
                  f"skipped {stats['skipped']}  undecodable {stats['undecodable']}")
            stats.update(frames=0, skipped=0, undecodable=0, hands=0, decode=0.0, infer=0.0,
                         bytes=0, wait=0.0)
            t_stats = now


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--phone", help="phone IP (default: find it via its UDP beacon)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="hand_landmarker.task")
    ap.add_argument("--fake", action="store_true", help="no MediaPipe; always 'no hands'")
    args = ap.parse_args()
    global _log_file
    _log_file = open(LOG_PATH, "a", encoding="utf-8")
    faulthandler.enable(_log_file)  # native crashes (MediaPipe, FFmpeg) leave a trace too
    log(f"--- pc_assist started (protocol v{VERSION}) ---")

    tracker = Tracker(os.path.normpath(args.model), args.fake)
    while True:
        ip = args.phone
        if not ip:
            print("Looking for the phone (turn on Hands + PC assist and start VR)...", flush=True)
            ip = find_phone(30)
            if not ip:
                continue
        try:
            sock = socket.create_connection((ip, args.port), timeout=5)
            sock.settimeout(None)
            log(f"Connected to the phone at {ip}. Hand tracking runs here now.")
            serve(sock, tracker)
        except KeyboardInterrupt:
            return
        except (ConnectionError, OSError) as e:
            if not isinstance(e, ConnectionRefusedError):
                log(f"Phone connection ended: {e}. Retrying in 2 s.")
            time.sleep(2)
        except Exception:  # noqa: BLE001 - keep serving whatever went wrong
            log_exception("session failed")
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
