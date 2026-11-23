/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import java.io.DataInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Wire format between the phone and the PC assist app (pc-assist/pc_assist.py). TCP, little endian;
 * the PC connects to the phone on {@link #PORT}.
 *
 * <pre>
 * phone -> PC  hello   'P' u8 version
 * PC -> phone  hello   'H' u8 version
 * phone -> PC  frame   'F' u32 id, u16 width, u16 height, u8 numHands, u8 codec (0 JPEG,
 *                      1 H.264 Annex-B access unit), u8 flags (1 = keyframe), u32 length, data
 * PC -> phone  result  'R' u32 id, f32 pcInferMs, u8 handCount (255 = frame skipped), then per
 *                      hand: u8 labelledLeft, 21 x (f32 x, f32 y) normalized image landmarks,
 *                      21 x (f32 x, f32 y, f32 z) metric world landmarks
 * </pre>
 *
 * The phone also broadcasts {@code "PCVR-ASSIST <version> <port>"} over UDP to {@link #BEACON_PORT}
 * once a second so the PC can find it.
 */
public final class PcAssistProtocol {
    public static final int VERSION = 2;
    public static final int CODEC_JPEG = 0, CODEC_H264 = 1;
    public static final int FLAG_KEYFRAME = 1;
    public static final int PORT = 9950;
    public static final int BEACON_PORT = 9951;
    public static final int SKIPPED = 255;
    public static final int FRAME_HEADER_BYTES = 1 + 4 + 2 + 2 + 1 + 1 + 1 + 4;
    static final int HAND_BYTES = 1 + 21 * 2 * 4 + 21 * 3 * 4;

    private PcAssistProtocol() {}

    public static byte[] hello() {
        return new byte[] {'P', (byte) VERSION};
    }

    public static String beacon() {
        return "PCVR-ASSIST " + VERSION + " " + PORT;
    }

    /** Frame header; the frame's bytes follow it on the wire. */
    public static byte[] frameHeader(
            int id, int width, int height, int numHands, int codec, int flags, int length) {
        ByteBuffer b = ByteBuffer.allocate(FRAME_HEADER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        b.put((byte) 'F');
        b.putInt(id);
        b.putShort((short) width);
        b.putShort((short) height);
        b.put((byte) numHands);
        b.put((byte) codec);
        b.put((byte) flags);
        b.putInt(length);
        return b.array();
    }

    /** One hand-tracking result, in the same layout HandTracker passes to native code. */
    public static final class Result {
        public int frameId;
        public float pcInferMs;
        public boolean skipped;
        public int handCount;
        public final boolean[] labelledLeft = new boolean[2];
        public final float[] image2d = new float[2 * 21 * 2];
        public final float[] world3d = new float[2 * 21 * 3];
    }

    /** Reads the PC's hello; returns its version. */
    public static int readHello(DataInputStream in) throws IOException {
        int tag = in.readUnsignedByte();
        if (tag != 'H') throw new IOException("not a PC assist app (got " + tag + ")");
        return in.readUnsignedByte();
    }

    /** Reads one result message (blocking). */
    public static Result readResult(DataInputStream in) throws IOException {
        int tag = in.readUnsignedByte();
        if (tag != 'R') throw new IOException("unexpected message " + tag);
        byte[] head = new byte[4 + 4 + 1];
        in.readFully(head);
        ByteBuffer h = ByteBuffer.wrap(head).order(ByteOrder.LITTLE_ENDIAN);
        Result r = new Result();
        r.frameId = h.getInt();
        r.pcInferMs = h.getFloat();
        int n = h.get() & 0xff;
        if (n == SKIPPED) {
            r.skipped = true;
            return r;
        }
        if (n > 4) throw new IOException("bad hand count " + n);
        byte[] body = new byte[n * HAND_BYTES];
        in.readFully(body);
        ByteBuffer b = ByteBuffer.wrap(body).order(ByteOrder.LITTLE_ENDIAN);
        r.handCount = Math.min(n, 2);
        for (int hand = 0; hand < n; hand++) {
            boolean keep = hand < 2;
            boolean left = b.get() != 0;
            if (keep) r.labelledLeft[hand] = left;
            for (int j = 0; j < 21 * 2; j++) {
                float v = b.getFloat();
                if (keep) r.image2d[hand * 21 * 2 + j] = v;
            }
            for (int j = 0; j < 21 * 3; j++) {
                float v = b.getFloat();
                if (keep) r.world3d[hand * 21 * 3 + j] = v;
            }
        }
        return r;
    }

    /** Encodes a result (the PC's side; used by tests). */
    public static byte[] encodeResult(Result r) {
        int n = r.skipped ? 0 : r.handCount;
        ByteBuffer b =
                ByteBuffer.allocate(1 + 4 + 4 + 1 + n * HAND_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        b.put((byte) 'R');
        b.putInt(r.frameId);
        b.putFloat(r.pcInferMs);
        b.put((byte) (r.skipped ? SKIPPED : n));
        for (int hand = 0; hand < n; hand++) {
            b.put((byte) (r.labelledLeft[hand] ? 1 : 0));
            for (int j = 0; j < 21 * 2; j++) b.putFloat(r.image2d[hand * 21 * 2 + j]);
            for (int j = 0; j < 21 * 3; j++) b.putFloat(r.world3d[hand * 21 * 3 + j]);
        }
        return b.array();
    }
}
