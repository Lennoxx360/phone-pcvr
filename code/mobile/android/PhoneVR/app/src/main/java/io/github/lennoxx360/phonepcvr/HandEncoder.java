/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import android.media.Image;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * Hardware H.264 encoder for PC assist: hand-camera frames (YUV_420_888) in, one Annex-B access
 * unit per frame out. Baseline profile, no B-frames, low latency, a keyframe every second and on
 * request (a PC that joins mid-stream needs one); SPS/PPS are put in front of every keyframe.
 */
final class HandEncoder {
    interface Sink {
        /** Drain thread. data[0..length) is one frame; ptsUs as given to {@link #encode}. */
        void onPacket(long ptsUs, byte[] data, int length, boolean keyframe, float encodeMs);
    }

    private static final String TAG = "PVR-Assist";
    private static final String MIME = MediaFormat.MIMETYPE_VIDEO_AVC;

    final int width, height;
    private final MediaCodec codec;
    private final Sink sink;
    private final Thread drainThread;
    private volatile boolean running = true;
    private byte[] csd = new byte[0]; // SPS + PPS (drain thread)
    private byte[] out = new byte[256 * 1024];
    private final java.util.concurrent.ConcurrentHashMap<Long, Long> queuedAtNs =
            new java.util.concurrent.ConcurrentHashMap<>();

    HandEncoder(int width, int height, int bitrateBps, int fps, Sink sink) throws IOException {
        this.width = width;
        this.height = height;
        this.sink = sink;
        this.bitrate = bitrateBps;
        this.fps = fps;
        codec = MediaCodec.createEncoderByType(MIME);
        try {
            codec.configure(format(true), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
        } catch (RuntimeException e) {
            Log.w(TAG, "encoder rejected the low-latency settings, using plain ones", e);
            codec.reset();
            codec.configure(format(false), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
        }
        codec.start();
        drainThread = new Thread(this::drainLoop, "PVR-Assist-enc");
        drainThread.start();
        Log.i(
                TAG,
                String.format(
                        java.util.Locale.US,
                        "H.264 encoder %s: %dx%d, %.1f Mbit/s, %d fps",
                        codec.getName(),
                        width,
                        height,
                        bitrateBps / 1e6f,
                        fps));
    }

    private final int bitrate, fps;

    private MediaFormat format(boolean tuned) {
        MediaFormat f = MediaFormat.createVideoFormat(MIME, width, height);
        f.setInteger(
                MediaFormat.KEY_COLOR_FORMAT,
                MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible);
        f.setInteger(MediaFormat.KEY_BIT_RATE, bitrate > 0 ? bitrate : 4_000_000);
        f.setInteger(MediaFormat.KEY_FRAME_RATE, fps > 0 ? fps : 30);
        f.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1);
        if (tuned) {
            f.setInteger(
                    MediaFormat.KEY_PROFILE, MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline);
            f.setInteger(MediaFormat.KEY_LEVEL, MediaCodecInfo.CodecProfileLevel.AVCLevel31);
            f.setInteger(
                    MediaFormat.KEY_BITRATE_MODE,
                    MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR);
            f.setInteger(MediaFormat.KEY_PRIORITY, 0); // realtime
            f.setInteger(MediaFormat.KEY_LATENCY, 1);
            // Qualcomm encoders: low-latency mode (ignored elsewhere). Measured ~20 ms encode
            // without.
            f.setInteger("vendor.qti-ext-enc-low-latency.enable", 1);
            if (Build.VERSION.SDK_INT >= 29) {
                f.setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0);
                f.setInteger(MediaFormat.KEY_PREPEND_HEADER_TO_SYNC_FRAMES, 1);
            }
        }
        return f;
    }

    /** Worker thread. Copies the frame into the encoder; false if no input buffer was free. */
    boolean encode(Image src, long ptsUs) {
        int index;
        try {
            index = codec.dequeueInputBuffer(10_000);
        } catch (IllegalStateException e) {
            return false;
        }
        if (index < 0) return false;
        Image dst = codec.getInputImage(index);
        if (dst == null) {
            codec.queueInputBuffer(index, 0, 0, ptsUs, 0);
            return false;
        }
        Image.Plane[] s = src.getPlanes(), d = dst.getPlanes();
        boolean ok =
                copyYuvNative(
                        s[0].getBuffer(),
                        s[1].getBuffer(),
                        s[2].getBuffer(),
                        s[0].getRowStride(),
                        s[1].getRowStride(),
                        s[1].getPixelStride(),
                        d[0].getBuffer(),
                        d[1].getBuffer(),
                        d[2].getBuffer(),
                        d[0].getRowStride(),
                        d[1].getRowStride(),
                        d[1].getPixelStride(),
                        width,
                        height);
        queuedAtNs.put(ptsUs, System.nanoTime());
        codec.queueInputBuffer(index, 0, ok ? width * height * 3 / 2 : 0, ptsUs, 0);
        return ok;
    }

    /** Any thread: the next frame becomes a keyframe. */
    void requestKeyframe() {
        try {
            Bundle b = new Bundle();
            b.putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0);
            codec.setParameters(b);
        } catch (IllegalStateException e) {
            // encoder already released
        }
    }

    private void drainLoop() {
        MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();
        while (running) {
            int index;
            try {
                index = codec.dequeueOutputBuffer(info, 50_000);
            } catch (IllegalStateException e) {
                break;
            }
            if (index < 0) continue;
            ByteBuffer buf = codec.getOutputBuffer(index);
            if (buf != null && info.size > 0) {
                buf.position(info.offset);
                buf.limit(info.offset + info.size);
                if ((info.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
                    csd = new byte[info.size];
                    buf.get(csd);
                } else {
                    boolean key = (info.flags & MediaCodec.BUFFER_FLAG_KEY_FRAME) != 0;
                    int pre = key ? csd.length : 0;
                    if (out.length < pre + info.size) out = new byte[(pre + info.size) * 2];
                    if (pre > 0) System.arraycopy(csd, 0, out, 0, pre);
                    buf.get(out, pre, info.size);
                    Long t = queuedAtNs.remove(info.presentationTimeUs);
                    float ms = t != null ? (System.nanoTime() - t) / 1e6f : -1;
                    sink.onPacket(info.presentationTimeUs, out, pre + info.size, key, ms);
                }
            }
            codec.releaseOutputBuffer(index, false);
            if (queuedAtNs.size() > 30) queuedAtNs.clear(); // frames the encoder dropped
        }
    }

    /** Worker thread. */
    void release() {
        running = false;
        try {
            drainThread.join(200);
        } catch (InterruptedException ignored) {
        }
        try {
            codec.stop();
        } catch (IllegalStateException ignored) {
        }
        codec.release();
    }

    private static native boolean copyYuvNative(
            ByteBuffer sy,
            ByteBuffer su,
            ByteBuffer sv,
            int syRow,
            int suvRow,
            int suvPix,
            ByteBuffer dy,
            ByteBuffer du,
            ByteBuffer dv,
            int dyRow,
            int duvRow,
            int duvPix,
            int width,
            int height);
}
