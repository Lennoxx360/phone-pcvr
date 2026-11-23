"""Tests for pc_assist.py without MediaPipe: wire format (same golden bytes as the phone's
PcAssistProtocolTest.java) and a loopback session against a fake phone.

    python test_pc_assist.py
"""

import socket
import struct
import threading
import time
import unittest

import pc_assist as pa

# PcAssistProtocolTest.frameHeaderMatchesThePcApp: id 7, 960x720, 2 hands, H.264, keyframe, 1000 B
GOLDEN_FRAME_HEADER = "4607000000c003d002020101e8030000"


class WireFormat(unittest.TestCase):
    def test_frame_header_matches_phone(self):
        self.assertEqual(pa.FRAME_HEADER.pack(b"F", 7, 960, 720, 2, pa.CODEC_H264,
                                             pa.FLAG_KEYFRAME, 1000).hex(), GOLDEN_FRAME_HEADER)

    def test_skipped_matches_phone(self):
        self.assertEqual(pa.encode_result(9, 0.0, None).hex(), "5209000000" + "00000000" + "ff")

    def test_result_layout(self):
        image = [(i * 0.01, i * 0.02) for i in range(21)]
        world = [(i * 0.001, -i * 0.001, 0.5) for i in range(21)]
        wire = pa.encode_result(42, 12.5, [(True, image, world)])
        self.assertEqual(len(wire), pa.RESULT_HEADER.size + pa.HAND.size)
        tag, fid, ms, n = pa.RESULT_HEADER.unpack_from(wire)
        self.assertEqual((tag, fid, ms, n), (b"R", 42, 12.5, 1))
        hand = pa.HAND.unpack_from(wire, pa.RESULT_HEADER.size)
        self.assertEqual(hand[0], 1)
        self.assertAlmostEqual(hand[1 + 2 * 20 + 1], 0.4, places=5)   # image[20].y
        self.assertAlmostEqual(hand[1 + 42 + 3 * 20 + 2], 0.5, places=5)  # world[20].z


class Loopback(unittest.TestCase):
    """A fake phone (this test) and the real serve() loop with the fake tracker."""

    def test_session(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]

        def pc_side():
            s = socket.create_connection(("127.0.0.1", port))
            try:
                pa.serve(s, pa.Tracker(model="", fake=True))
            except (ConnectionError, OSError):
                pass

        threading.Thread(target=pc_side, daemon=True).start()
        phone, _ = server.accept()
        phone.settimeout(5)
        phone.sendall(b"P\x02")
        self.assertEqual(pa.recv_exact(phone, 2), b"H\x02")

        # Send 20 frames, at most 2 waiting for a result (like the phone).
        jpeg = b"\xff\xd8fakejpeg\xff\xd9"
        sent, answered, skipped, inflight = 0, set(), 0, set()
        deadline = time.time() + 5
        while (sent < 20 or inflight) and time.time() < deadline:
            while sent < 20 and len(inflight) < 2:
                sent += 1
                phone.sendall(pa.FRAME_HEADER.pack(b"F", sent, 960, 720, 1, pa.CODEC_JPEG, 0,
                                                   len(jpeg)) + jpeg)
                inflight.add(sent)
            tag, fid, ms, n = pa.RESULT_HEADER.unpack(pa.recv_exact(phone, pa.RESULT_HEADER.size))
            self.assertEqual(tag, b"R")
            if n == pa.SKIPPED:
                skipped += 1
            else:
                self.assertEqual(n, 0)  # fake tracker: no hands
                answered.add(fid)
            inflight.discard(fid)
        phone.close()
        server.close()
        self.assertEqual(sent, 20)
        self.assertEqual(len(answered) + skipped, 20, "every frame answered or skipped")
        self.assertGreater(len(answered), 0)


def baseline_stream(n=30, w=960, h=720, gop=10):
    """n packets of a baseline-profile, zero-latency H.264 stream (like the phone's), made with
    x264 from PyAV. Returns [(bytes, is_keyframe)]."""
    import av
    import numpy as np
    enc = av.CodecContext.create("libx264", "w")
    enc.width, enc.height, enc.pix_fmt = w, h, "yuv420p"
    from fractions import Fraction
    enc.time_base = Fraction(1, 30)
    enc.options = {"profile": "baseline", "tune": "zerolatency", "preset": "ultrafast",
                   "g": str(gop), "bf": "0"}
    out = []
    for i in range(n):
        img = np.zeros((h, w, 3), np.uint8)
        img[:, :, 0] = (i * 8) % 256
        img[100:300, 100 + i * 10:300 + i * 10] = 255
        f = av.VideoFrame.from_ndarray(img, format="rgb24").reformat(format="yuv420p")
        f.pts = i
        for pkt in enc.encode(f):
            out.append((bytes(pkt), pkt.is_keyframe))
    return out


class H264(unittest.TestCase):
    def test_one_frame_out_per_packet(self):
        stream = baseline_stream()
        self.assertEqual(len(stream), 30, "zerolatency x264 emits one packet per frame")
        dec = pa.H264Decoder()
        for i, (data, key) in enumerate(stream):
            rgb = dec.decode(data, key)
            self.assertIsNotNone(rgb, f"packet {i} gave no frame right away (decoder delay)")
            self.assertEqual(rgb.shape, (720, 960, 3))

    def test_join_mid_stream_waits_for_keyframe(self):
        stream = baseline_stream(gop=10)
        dec = pa.H264Decoder()
        got = [dec.decode(d, k) is not None for d, k in stream[3:]]
        self.assertFalse(any(got[:7]), "P-frames before the first keyframe are skipped")
        self.assertTrue(all(got[7:]), "decoding works from the keyframe on")


if __name__ == "__main__":
    unittest.main(verbosity=2)
