/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import android.content.Context;
import android.graphics.Bitmap;
import android.media.Image;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import com.google.mediapipe.framework.image.BitmapImageBuilder;
import com.google.mediapipe.framework.image.MPImage;
import com.google.mediapipe.framework.image.MediaImageBuilder;
import com.google.mediapipe.tasks.components.containers.Category;
import com.google.mediapipe.tasks.components.containers.Landmark;
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark;
import com.google.mediapipe.tasks.core.BaseOptions;
import com.google.mediapipe.tasks.core.Delegate;
import com.google.mediapipe.tasks.vision.core.RunningMode;
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker;
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult;
import java.nio.ByteBuffer;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Runs MediaPipe Hand Landmarker on a background thread, either on the app's own HD camera stream
 * (ArCoreTracker shared camera mode, {@link #submitShared}) or on ARCore's CPU image (legacy). A
 * frame is only taken when the worker is idle, so a slow detection drops frames instead of building
 * a queue. Results go to native code, which places the hands in 3D and sends them to ALVR.
 */
public class HandTracker {
    public interface ResultSink {
        /**
         * Worker thread. Per hand: 21 normalized image landmarks (x, y in [0,1] of the CPU image),
         * 21 metric world landmarks (MediaPipe hand frame, origin at the hand centre), and whether
         * MediaPipe labelled it "Left" (it assumes a mirrored selfie image).
         */
        void onHands(
                long captureTimeNs,
                int handCount,
                boolean[] labelledLeft,
                float[] image2d,
                float[] world3d,
                float[] intrinsics,
                float[] cameraPose,
                float detectMs);
    }

    private static final String TAG = "PVR-Hands";
    private static final String MODEL = "hand_landmarker.task";

    private final Context context;
    private final ResultSink sink;
    private final HandlerThread thread = new HandlerThread("PVR-Hands");
    private final Handler handler;
    private final AtomicBoolean busy = new AtomicBoolean(false);
    private volatile boolean enabled = false;
    private boolean loadRequested = false;
    private volatile HandLandmarker landmarker;
    private String delegateName = "none";

    // Reused conversion buffers (worker thread only).
    private int[] argb;
    private Bitmap bitmap;

    public HandTracker(Context context, ResultSink sink) {
        this.context = context;
        this.sink = sink;
        thread.start();
        handler = new Handler(thread.getLooper());
    }

    /** UI thread. MediaPipe is only loaded the first time hand tracking is switched on. */
    public void setEnabled(boolean on) {
        enabled = on;
        if (on && !loadRequested) {
            loadRequested = true;
            handler.post(this::createLandmarker);
        }
        updatePcAssist();
    }

    // ---- PC assist: MediaPipe runs on the PC (pc-assist/pc_assist.py) ----
    // Frames go to the PC as JPEG; its landmarks come back and continue exactly like local ones
    // (pose lookup, then native code). Without a connected PC the phone runs MediaPipe itself.
    private volatile boolean pcAssistWanted = false;
    private volatile int pcAssistQuality = 75;
    private PcAssist pcAssist; // guarded by `this`

    private static final class RemoteFrame {
        long cameraTs, captureBootNs, t0Ns;
        float scale;
        int width, height;
        PoseSource poses;
    }

    private final java.util.concurrent.ConcurrentHashMap<Integer, RemoteFrame> remoteFrames =
            new java.util.concurrent.ConcurrentHashMap<>();

    private static final class JpegBuffer extends java.io.ByteArrayOutputStream {
        JpegBuffer() {
            super(128 * 1024);
        }

        byte[] bytes() {
            return buf;
        }
    }

    private final JpegBuffer jpeg = new JpegBuffer(); // worker thread

    // H.264 (hardware encoder) instead of JPEG: ~4x less Wi-Fi and no software JPEG encode.
    // Falls back to JPEG for good if the encoder can't be created.
    private volatile boolean pcAssistH264 = true;
    private volatile int pcAssistBitrate = 4_000_000;
    private HandEncoder encoder; // worker thread
    private boolean encoderFailed = false; // worker thread
    private final java.util.concurrent.ConcurrentHashMap<Long, Integer> ptsToId =
            new java.util.concurrent.ConcurrentHashMap<>();
    private volatile int remoteStatKeyframes;

    /** Any thread. quality: JPEG quality 1-100; mbps: H.264 bitrate. */
    public void setPcAssist(boolean on, int quality, boolean h264, float mbps) {
        pcAssistWanted = on;
        pcAssistQuality = Math.max(10, Math.min(100, quality));
        int bitrate = Math.round(Math.max(0.5f, Math.min(20f, mbps)) * 1_000_000);
        if (h264 != pcAssistH264 || bitrate != pcAssistBitrate) {
            pcAssistH264 = h264;
            pcAssistBitrate = bitrate;
            handler.post(this::releaseEncoder); // recreated with the new settings
        }
        updatePcAssist();
    }

    private synchronized void updatePcAssist() {
        boolean want = pcAssistWanted && enabled;
        if (want && pcAssist == null) {
            pcAssist =
                    new PcAssist(
                            new PcAssist.Listener() {
                                @Override
                                public void onResult(PcAssistProtocol.Result r) {
                                    handler.post(() -> deliverRemote(r));
                                }

                                @Override
                                public void onConnected() {
                                    HandEncoder e = encoder;
                                    if (e != null) e.requestKeyframe();
                                }
                            });
            pcAssist.start();
        } else if (!want && pcAssist != null) {
            pcAssist.stop();
            pcAssist = null;
            remoteFrames.clear();
            handler.post(this::releaseEncoder);
        }
    }

    private void releaseEncoder() {
        if (encoder != null) {
            encoder.release();
            encoder = null;
        }
        ptsToId.clear();
    }

    // Worker thread. Null = use JPEG.
    private HandEncoder encoderFor(PcAssist pa, int width, int height) {
        if (!pcAssistH264 || encoderFailed) return null;
        if (encoder != null && (encoder.width != width || encoder.height != height))
            releaseEncoder();
        if (encoder == null) {
            try {
                encoder =
                        new HandEncoder(
                                width,
                                height,
                                pcAssistBitrate,
                                30,
                                (ptsUs, data, length, key, encMs) -> {
                                    Integer id = ptsToId.remove(ptsUs);
                                    PcAssist p = activePcAssist();
                                    if (id == null || p == null) return;
                                    p.sendFrame(
                                            id,
                                            PcAssistProtocol.CODEC_H264,
                                            key ? PcAssistProtocol.FLAG_KEYFRAME : 0,
                                            width,
                                            height,
                                            numHands,
                                            data,
                                            0,
                                            length);
                                    remoteStatBytes += length;
                                    if (encMs >= 0) remoteStatEncMs += encMs;
                                    if (key) remoteStatKeyframes++;
                                });
                encoder.requestKeyframe();
            } catch (Exception e) {
                Log.w(TAG, "H.264 encoder unavailable, PC assist uses JPEG", e);
                encoderFailed = true;
                encoder = null;
            }
        }
        return encoder;
    }

    private synchronized PcAssist activePcAssist() {
        return pcAssist != null && pcAssist.isReady() ? pcAssist : null;
    }

    // Worker thread: encode and queue for the PC.
    private void processRemote(
            PcAssist pa,
            Image image,
            long cameraTs,
            long captureBootNs,
            float scale,
            PoseSource poses) {
        long t0 = SystemClock.elapsedRealtimeNanos();
        remoteStatFrames++;
        int id = pa.beginFrame();
        if (id < 0) {
            remoteStatDropped++; // too many frames waiting for the PC: skip before encoding
        } else {
            HandEncoder enc = encoderFor(pa, image.getWidth(), image.getHeight());
            remoteH264 = enc != null;
            if (enc != null) {
                long ptsUs = captureBootNs / 1000;
                ptsToId.put(ptsUs, id);
                if (!enc.encode(image, ptsUs)) {
                    ptsToId.remove(ptsUs);
                    pa.cancelFrame(id);
                    remoteStatDropped++;
                    id = -1;
                }
                remoteStatConvMs += (SystemClock.elapsedRealtimeNanos() - t0) / 1e6f;
                if (ptsToId.size() > 30) ptsToId.clear(); // outputs the encoder never gave
            } else {
                Bitmap bmp = yuvToBitmapFull(image);
                if (bmp == null) bmp = yuvToBitmap(image);
                long tConv = SystemClock.elapsedRealtimeNanos();
                jpeg.reset();
                bmp.compress(Bitmap.CompressFormat.JPEG, pcAssistQuality, jpeg);
                long tEnc = SystemClock.elapsedRealtimeNanos();
                pa.sendFrame(
                        id,
                        PcAssistProtocol.CODEC_JPEG,
                        0,
                        bmp.getWidth(),
                        bmp.getHeight(),
                        numHands,
                        jpeg.bytes(),
                        0,
                        jpeg.size());
                remoteStatConvMs += (tConv - t0) / 1e6f;
                remoteStatEncMs += (tEnc - tConv) / 1e6f;
                remoteStatBytes += jpeg.size();
            }
        }
        if (id >= 0) {
            RemoteFrame f = new RemoteFrame();
            f.cameraTs = cameraTs;
            f.captureBootNs = captureBootNs;
            f.t0Ns = t0;
            f.scale = scale;
            f.width = image.getWidth();
            f.height = image.getHeight();
            f.poses = poses;
            remoteFrames.put(id, f);
            if (remoteFrames.size() > 8) { // results that never came: forget the oldest
                long cutoff = t0 - 1_000_000_000L;
                remoteFrames.values().removeIf(old -> old.t0Ns < cutoff);
            }
        }
        logRemoteStats(pa);
    }

    // Worker thread: a result from the PC; same pose lookup and hand-off as a local result.
    private void deliverRemote(PcAssistProtocol.Result r) {
        RemoteFrame f = remoteFrames.remove(r.frameId);
        if (f == null) return;
        float[][] arK = new float[1][];
        float[] pose = f.poses.poseAt(f.cameraTs, arK, false);
        long waitStart = SystemClock.elapsedRealtime();
        while (pose == null && SystemClock.elapsedRealtime() - waitStart < POSE_WAIT_MAX_MS) {
            SystemClock.sleep(4);
            pose = f.poses.poseAt(f.cameraTs, arK, false);
        }
        if (pose == null) pose = f.poses.poseAt(f.cameraTs, arK, true);
        long now = SystemClock.elapsedRealtimeNanos();
        float roundTripMs = (now - f.t0Ns) / 1e6f;
        remoteStatResults++;
        remoteStatRoundTripMs += roundTripMs;
        remoteStatPcMs += r.pcInferMs;
        if (pose == null) {
            remoteStatNoPose++;
            return;
        }
        float[] k = arK[0];
        float[] intrinsics = {
            k[0] * f.scale, k[1] * f.scale, k[2] * f.scale, k[3] * f.scale, f.width, f.height
        };
        sink.onHands(
                f.captureBootNs,
                r.handCount,
                r.labelledLeft,
                r.image2d,
                r.world3d,
                intrinsics,
                pose,
                roundTripMs);
    }

    private long remoteStatStartMs = 0;
    private int remoteStatFrames, remoteStatDropped, remoteStatResults, remoteStatNoPose;
    private volatile float remoteStatConvMs, remoteStatEncMs, remoteStatRoundTripMs, remoteStatPcMs;
    private volatile long remoteStatBytes;
    private volatile boolean remoteH264 = false;

    private void logRemoteStats(PcAssist pa) {
        long now = SystemClock.elapsedRealtime();
        if (remoteStatStartMs == 0) remoteStatStartMs = now;
        if (now - remoteStatStartMs < 5000) return;
        float s = (now - remoteStatStartMs) / 1000f;
        int frames = Math.max(1, remoteStatFrames), res = Math.max(1, remoteStatResults);
        Log.i(
                TAG,
                String.format(
                        java.util.Locale.US,
                        "PC assist via %s: %.1f frames/s, %.1f results/s | %s: prepare %.1f ms,"
                                + " encode %.1f ms, %.0f KB, %.1f Mbit/s, keyframes %d | round"
                                + " trip %.1f ms (PC %.1f ms) | dropped %d, skipped %d, timeouts"
                                + " %d, no pose %d",
                        pa.peer(),
                        remoteStatFrames / s,
                        remoteStatResults / s,
                        remoteH264
                                ? String.format(
                                        java.util.Locale.US,
                                        "H.264 %.1f Mbit/s",
                                        pcAssistBitrate / 1e6f)
                                : "JPEG q" + pcAssistQuality,
                        remoteStatConvMs / frames,
                        remoteStatEncMs / frames,
                        remoteStatBytes / 1024f / frames,
                        remoteStatBytes * 8 / 1e6f / s,
                        remoteStatKeyframes,
                        remoteStatRoundTripMs / res,
                        remoteStatPcMs / res,
                        remoteStatDropped,
                        pa.skipped,
                        pa.timeouts,
                        remoteStatNoPose));
        remoteStatStartMs = now;
        remoteStatFrames = remoteStatDropped = remoteStatResults = remoteStatNoPose = 0;
        remoteStatConvMs = remoteStatEncMs = remoteStatRoundTripMs = remoteStatPcMs = 0;
        remoteStatBytes = 0;
        remoteStatKeyframes = 0;
        pa.skipped = pa.timeouts = 0;
    }

    private volatile int numHands = 2;
    private int createdNumHands = 2; // worker thread

    /**
     * Any thread. 1 when only one hand is used ("Which hands" = Left/Right only): MediaPipe then
     * stops running its palm detector to look for a second hand, which costs time every frame.
     */
    public void setNumHands(int n) {
        n = n <= 1 ? 1 : 2;
        if (n == numHands) return;
        numHands = n;
        handler.post(
                () -> {
                    // Not loaded yet, or already created with this count: nothing to redo.
                    if ((gpu == null && cpu == null) || createdNumHands == numHands) return;
                    if (gpu != null) gpu.close();
                    if (cpu != null) cpu.close();
                    gpu = cpu = landmarker = null;
                    gpuMs.clear();
                    cpuMs.clear();
                    benchUseCpu = false;
                    lastTimestampMs[0] = lastTimestampMs[1] = -1;
                    createLandmarker();
                });
    }

    public String delegateName() {
        return delegateName;
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Looks up ARCore's camera pose for a capture timestamp (see ArCoreTracker.poseAt). */
    public interface PoseSource {
        float[] poseAt(long cameraTs, float[][] intrinsicsOut, boolean finalTry);
    }

    /** Hand frame time minus the nearest ARCore frame time (diagnostics). */
    public interface MatchStats {
        float lastMatchDeltaMs();
    }

    private volatile MatchStats matchStats;

    public void setMatchStats(MatchStats m) {
        matchStats = m;
    }

    // ---- Own HD stream (ArCoreTracker shared camera mode) ----
    // Pose isn't known at capture time (ARCore processes the same frame on the render thread a
    // little later), so it is looked up after inference, which takes longer than that anyway.
    private static final long POSE_WAIT_MAX_MS = 60;
    private boolean mediaImageInput = true; // MediaPipe reads the YUV image directly
    private int statFrames, statNoPose;
    private float statConvMs, statInferMs, statPoseWaitMs;
    private long statStartMs = 0;

    /**
     * Camera thread. Takes ownership of the image. intrinsicsScale = this stream's width over
     * ARCore's CPU-image width (same aspect ratio), applied to ARCore's intrinsics.
     */
    public void submitShared(
            Image image,
            long cameraTs,
            long captureBootNs,
            float intrinsicsScale,
            PoseSource poses) {
        if (!busy.compareAndSet(false, true)) {
            image.close();
            return;
        }
        handler.post(
                () -> {
                    try {
                        processShared(image, cameraTs, captureBootNs, intrinsicsScale, poses);
                    } catch (Exception e) {
                        Log.w(TAG, "hand detection failed", e);
                    } finally {
                        image.close();
                        busy.set(false);
                    }
                });
    }

    private void processShared(
            Image image, long cameraTs, long captureBootNs, float scale, PoseSource poses) {
        PcAssist pa = activePcAssist();
        if (pa != null) {
            processRemote(pa, image, cameraTs, captureBootNs, scale, poses);
            return;
        }
        if (landmarker == null) return;
        long t0 = SystemClock.elapsedRealtimeNanos();
        HandLandmarker lm = landmarker;
        int which = lm == cpu ? 1 : 0;
        if (benchmarking) {
            lm = benchUseCpu ? cpu : gpu;
            which = benchUseCpu ? 1 : 0;
        }
        long tsMs = captureBootNs / 1_000_000L;
        if (tsMs <= lastTimestampMs[which]) tsMs = lastTimestampMs[which] + 1;
        lastTimestampMs[which] = tsMs;

        HandLandmarkerResult result = null;
        Bitmap bmp = null;
        long tConv = SystemClock.elapsedRealtimeNanos();
        long tInfer = tConv;
        if (mediaImageInput) {
            try {
                MPImage mp = new MediaImageBuilder(image).build();
                tInfer = SystemClock.elapsedRealtimeNanos();
                result = lm.detectForVideo(mp, tsMs);
            } catch (Exception e) {
                mediaImageInput = false;
                Log.w(TAG, "MediaPipe can't read the YUV image directly, converting instead", e);
                tsMs = ++lastTimestampMs[which];
            }
        }
        if (result == null) {
            bmp = yuvToBitmapFull(image);
            if (bmp == null) bmp = yuvToBitmap(image); // half size, Java
            tInfer = SystemClock.elapsedRealtimeNanos();
            result = lm.detectForVideo(new BitmapImageBuilder(bmp).build(), tsMs);
        }
        long tDone = SystemClock.elapsedRealtimeNanos();
        float convMs = (tInfer - tConv) / 1e6f, inferMs = (tDone - tInfer) / 1e6f;
        benchmarkStep(result, inferMs);

        // Camera pose for this exact capture.
        float[][] arK = new float[1][];
        float[] pose = poses.poseAt(cameraTs, arK, false);
        long waitStart = SystemClock.elapsedRealtime();
        while (pose == null && SystemClock.elapsedRealtime() - waitStart < POSE_WAIT_MAX_MS) {
            SystemClock.sleep(4);
            pose = poses.poseAt(cameraTs, arK, false);
        }
        if (pose == null) pose = poses.poseAt(cameraTs, arK, true);
        float poseWaitMs = SystemClock.elapsedRealtime() - waitStart;
        updateSharedStats(image, convMs, inferMs, poseWaitMs, pose == null);
        if (pose == null) return; // ARCore never saw this frame tracking: no 3D placement

        float[] k = arK[0];
        float[] intrinsics = {
            k[0] * scale,
            k[1] * scale,
            k[2] * scale,
            k[3] * scale,
            image.getWidth(),
            image.getHeight()
        };
        float detectMs = (tDone - t0) / 1e6f;
        deliver(result, captureBootNs, intrinsics, pose, detectMs, bmp);
    }

    private void updateSharedStats(
            Image image, float convMs, float inferMs, float poseWaitMs, boolean noPose) {
        long now = SystemClock.elapsedRealtime();
        if (statStartMs == 0) statStartMs = now;
        statFrames++;
        statConvMs += convMs;
        statInferMs += inferMs;
        statPoseWaitMs += poseWaitMs;
        if (noPose) statNoPose++;
        if (now - statStartMs >= 5000) {
            Log.i(
                    TAG,
                    String.format(
                            java.util.Locale.US,
                            "own stream %dx%d via %s on %s: %.1f frames/s, convert %.1f ms, infer"
                                    + " %.1f ms, pose wait %.1f ms, no pose %d, hand-ARCore"
                                    + " offset %.1f ms",
                            image.getWidth(),
                            image.getHeight(),
                            mediaImageInput
                                    ? "YUV direct"
                                    : nativeConvert
                                            ? "native RGBA (full size)"
                                            : "Java RGBA (half)",
                            delegateName,
                            statFrames * 1000f / (now - statStartMs),
                            statConvMs / statFrames,
                            statInferMs / statFrames,
                            statPoseWaitMs / statFrames,
                            statNoPose,
                            matchStats != null ? matchStats.lastMatchDeltaMs() : 0f));
            statStartMs = now;
            statFrames = statNoPose = 0;
            statConvMs = statInferMs = statPoseWaitMs = 0;
        }
    }

    /** Render thread: true if a new camera image should be handed over now. */
    public boolean wantsFrame() {
        return enabled && !busy.get() && (landmarker != null || activePcAssist() != null);
    }

    /**
     * Render thread. Takes ownership of the image (closes it). intrinsics: fx, fy, cx, cy, width,
     * height of the CPU image. cameraPose: ARCore physical camera pose tx, ty, tz, qx, qy, qz, qw.
     */
    public void submit(Image image, long captureTimeNs, float[] intrinsics, float[] cameraPose) {
        if (!busy.compareAndSet(false, true)) {
            image.close();
            return;
        }
        handler.post(
                () -> {
                    try {
                        process(image, captureTimeNs, intrinsics, cameraPose);
                    } catch (Exception e) {
                        Log.w(TAG, "hand detection failed", e);
                    } finally {
                        image.close();
                        busy.set(false);
                    }
                });
    }

    public void close() {
        pcAssistWanted = false;
        updatePcAssist();
        handler.post(this::releaseEncoder);
        handler.post(
                () -> {
                    if (gpu != null) gpu.close();
                    if (cpu != null) cpu.close();
                    gpu = cpu = landmarker = null;
                });
        thread.quitSafely();
    }

    private HandLandmarker create(Delegate delegate) {
        try {
            HandLandmarker.HandLandmarkerOptions options =
                    HandLandmarker.HandLandmarkerOptions.builder()
                            .setBaseOptions(
                                    BaseOptions.builder()
                                            .setModelAssetPath(MODEL)
                                            .setDelegate(delegate)
                                            .build())
                            .setRunningMode(RunningMode.VIDEO)
                            .setNumHands(createdNumHands = numHands)
                            // Detection stays strict (no phantom hands); keeping a hand once found
                            // is relaxed: at 0.5 it was lost in ~1/3 of frames on device, and the
                            // native HandFilter smooths the extra noise.
                            .setMinHandDetectionConfidence(0.5f)
                            .setMinHandPresenceConfidence(0.3f)
                            .setMinTrackingConfidence(0.3f)
                            .build();
            return HandLandmarker.createFromOptions(context, options);
        } catch (Exception e) {
            Log.w(TAG, "HandLandmarker on " + delegate + " failed", e);
            return null;
        }
    }

    // GPU vs CPU: on device the GPU took 40-45 ms per frame with a hand in view (it shares the
    // GPU with video decode and rendering). Both are loaded; frames with a hand alternate between
    // them until each has BENCH_SAMPLES timings, then the faster one is kept.
    private static final int BENCH_SAMPLES = 15;
    private HandLandmarker gpu, cpu;
    private final java.util.ArrayList<Float> gpuMs = new java.util.ArrayList<>();
    private final java.util.ArrayList<Float> cpuMs = new java.util.ArrayList<>();
    private boolean benchmarking = false;
    private boolean benchUseCpu = false;
    private final long[] lastTimestampMs = {-1, -1};

    private void createLandmarker() {
        gpu = create(Delegate.GPU);
        cpu = create(Delegate.CPU);
        if (gpu != null && cpu != null) {
            benchmarking = true;
            landmarker = gpu;
            delegateName = "benchmarking";
            Log.i(TAG, "HandLandmarker: GPU and CPU loaded, benchmarking");
        } else {
            landmarker = gpu != null ? gpu : cpu;
            delegateName = gpu != null ? "GPU" : cpu != null ? "CPU" : "none";
            Log.i(TAG, "HandLandmarker ready on " + delegateName);
        }
    }

    private static float median(java.util.ArrayList<Float> v) {
        java.util.ArrayList<Float> s = new java.util.ArrayList<>(v);
        java.util.Collections.sort(s);
        return s.get(s.size() / 2);
    }

    private void process(Image image, long captureTimeNs, float[] intrinsics, float[] cameraPose) {
        if (landmarker == null) return;
        long t0 = SystemClock.elapsedRealtimeNanos();
        Bitmap bmp = yuvToBitmap(image);
        HandLandmarker lm = landmarker;
        int which = lm == cpu ? 1 : 0;
        if (benchmarking) {
            lm = benchUseCpu ? cpu : gpu;
            which = benchUseCpu ? 1 : 0;
        }
        long tsMs = captureTimeNs / 1_000_000L;
        if (tsMs <= lastTimestampMs[which]) tsMs = lastTimestampMs[which] + 1; // must increase
        lastTimestampMs[which] = tsMs;
        MPImage mpImage = new BitmapImageBuilder(bmp).build();
        long tInfer = SystemClock.elapsedRealtimeNanos();
        HandLandmarkerResult result = lm.detectForVideo(mpImage, tsMs);
        float inferMs = (SystemClock.elapsedRealtimeNanos() - tInfer) / 1e6f;
        float detectMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6f;
        benchmarkStep(result, inferMs);
        deliver(result, captureTimeNs, intrinsics, cameraPose, detectMs, bmp);
    }

    // While benchmarking, frames with a hand alternate GPU/CPU; then the faster one is kept.
    private void benchmarkStep(HandLandmarkerResult result, float inferMs) {
        if (benchmarking && !result.landmarks().isEmpty()) {
            (benchUseCpu ? cpuMs : gpuMs).add(inferMs);
            benchUseCpu = !benchUseCpu;
            if (gpuMs.size() >= BENCH_SAMPLES && cpuMs.size() >= BENCH_SAMPLES) {
                float g = median(gpuMs), c = median(cpuMs);
                boolean useCpu = c < g;
                landmarker = useCpu ? cpu : gpu;
                (useCpu ? gpu : cpu).close();
                if (useCpu) gpu = null;
                else cpu = null;
                delegateName = useCpu ? "CPU" : "GPU";
                benchmarking = false;
                Log.i(
                        TAG,
                        String.format(
                                java.util.Locale.US,
                                "benchmark: GPU %.1f ms, CPU %.1f ms median inference -> %s",
                                g,
                                c,
                                delegateName));
            }
        }
    }

    private void deliver(
            HandLandmarkerResult result,
            long captureTimeNs,
            float[] intrinsics,
            float[] cameraPose,
            float detectMs,
            Bitmap bmp) {
        List<List<NormalizedLandmark>> hands2d = result.landmarks();
        List<List<Landmark>> hands3d = result.worldLandmarks();
        List<List<Category>> handedness = result.handedness();
        int n = Math.min(Math.min(hands2d.size(), hands3d.size()), 2);
        boolean[] left = new boolean[2];
        float[] image2d = new float[2 * 21 * 2];
        float[] world3d = new float[2 * 21 * 3];
        for (int h = 0; h < n; h++) {
            List<Category> cats = handedness.size() > h ? handedness.get(h) : null;
            left[h] = cats != null && !cats.isEmpty() && "Left".equals(cats.get(0).categoryName());
            for (int j = 0; j < 21; j++) {
                NormalizedLandmark p = hands2d.get(h).get(j);
                image2d[(h * 21 + j) * 2] = p.x();
                image2d[(h * 21 + j) * 2 + 1] = p.y();
                Landmark w = hands3d.get(h).get(j);
                world3d[(h * 21 + j) * 3] = w.x();
                world3d[(h * 21 + j) * 3 + 1] = w.y();
                world3d[(h * 21 + j) * 3 + 2] = w.z();
            }
        }
        sink.onHands(captureTimeNs, n, left, image2d, world3d, intrinsics, cameraPose, detectMs);
        if (SAVE_DEBUG_FRAMES && bmp != null) maybeSaveDebugFrame(bmp, n, left, image2d);
    }

    // Tuning aid: saves what MediaPipe saw to Android/data/<package>/files/hands/ every 3 s.
    private static final boolean SAVE_DEBUG_FRAMES = false;

    // Every few seconds, save what MediaPipe saw (with landmarks and labels) to
    // Android/data/<package>/files/hands/ so orientation and handedness can be checked by eye.
    private static final long DEBUG_FRAME_INTERVAL_MS = 3000;
    private static final int DEBUG_FRAMES_KEPT = 20;
    private long lastDebugFrameMs = 0;
    private int debugFrameIndex = 0;

    private void maybeSaveDebugFrame(Bitmap bmp, int n, boolean[] left, float[] image2d) {
        long nowMs = SystemClock.elapsedRealtime();
        if (nowMs - lastDebugFrameMs < DEBUG_FRAME_INTERVAL_MS) return;
        lastDebugFrameMs = nowMs;
        java.io.File dir = context.getExternalFilesDir("hands");
        if (dir == null) return;
        Bitmap copy = bmp.copy(Bitmap.Config.ARGB_8888, true);
        android.graphics.Canvas canvas = new android.graphics.Canvas(copy);
        android.graphics.Paint paint = new android.graphics.Paint();
        paint.setTextSize(14);
        for (int h = 0; h < n; h++) {
            // MediaPipe's label (it assumes a mirrored selfie image), drawn as it reported it.
            paint.setColor(left[h] ? 0xff00ff00 : 0xffff00ff);
            for (int j = 0; j < 21; j++) {
                float x = image2d[(h * 21 + j) * 2] * copy.getWidth();
                float y = image2d[(h * 21 + j) * 2 + 1] * copy.getHeight();
                canvas.drawCircle(x, y, j == 0 ? 4 : 2, paint);
            }
            canvas.drawText(
                    "MP:" + (left[h] ? "Left" : "Right"),
                    image2d[h * 21 * 2] * copy.getWidth(),
                    image2d[h * 21 * 2 + 1] * copy.getHeight() + 16,
                    paint);
        }
        java.io.File file =
                new java.io.File(dir, "frame_" + (debugFrameIndex++ % DEBUG_FRAMES_KEPT) + ".jpg");
        try (java.io.FileOutputStream out = new java.io.FileOutputStream(file)) {
            copy.compress(Bitmap.CompressFormat.JPEG, 85, out);
        } catch (java.io.IOException e) {
            Log.w(TAG, "debug frame save failed", e);
        }
        copy.recycle();
    }

    // Full-size YUV_420_888 -> RGBA in native code (alvr_main.cpp), into a reused bitmap. Null if
    // the native conversion isn't available, then the half-size Java path is used.
    private boolean nativeConvert = true;
    private Bitmap fullBitmap;

    private Bitmap yuvToBitmapFull(Image image) {
        if (!nativeConvert) return null;
        int w = image.getWidth(), h = image.getHeight();
        if (fullBitmap == null || fullBitmap.getWidth() != w || fullBitmap.getHeight() != h) {
            fullBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        }
        Image.Plane[] p = image.getPlanes();
        try {
            if (yuvToRgbaNative(
                    p[0].getBuffer(),
                    p[1].getBuffer(),
                    p[2].getBuffer(),
                    p[0].getRowStride(),
                    p[1].getRowStride(),
                    p[1].getPixelStride(),
                    w,
                    h,
                    fullBitmap)) {
                return fullBitmap;
            }
        } catch (UnsatisfiedLinkError e) {
            Log.w(TAG, "native YUV conversion missing", e);
        }
        nativeConvert = false;
        return null;
    }

    private static native boolean yuvToRgbaNative(
            ByteBuffer y,
            ByteBuffer u,
            ByteBuffer v,
            int yRowStride,
            int uvRowStride,
            int uvPixelStride,
            int width,
            int height,
            Bitmap out);

    // YUV_420_888 -> half-size ARGB bitmap (BT.601 full range), reusing buffers. MediaPipe scales
    // to ~200 px internally, so the full 640x480 conversion was wasted time (4x the pixels).
    // Normalized landmark coordinates are unaffected by the downscale.
    private Bitmap yuvToBitmap(Image image) {
        int w = image.getWidth() / 2, h = image.getHeight() / 2;
        if (argb == null || argb.length != w * h) {
            argb = new int[w * h];
            bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        }
        Image.Plane[] planes = image.getPlanes();
        ByteBuffer yBuf = planes[0].getBuffer(),
                uBuf = planes[1].getBuffer(),
                vBuf = planes[2].getBuffer();
        int yRow = planes[0].getRowStride(), yPix = planes[0].getPixelStride();
        int uvRow = planes[1].getRowStride(), uvPix = planes[1].getPixelStride();
        for (int y = 0; y < h; y++) {
            int yBase = (2 * y) * yRow, uvBase = y * uvRow;
            for (int x = 0; x < w; x++) {
                int Y = (yBuf.get(yBase + 2 * x * yPix) & 0xff);
                int uvIndex = uvBase + x * uvPix;
                int U = (uBuf.get(uvIndex) & 0xff) - 128;
                int V = (vBuf.get(uvIndex) & 0xff) - 128;
                int r = Y + ((91881 * V) >> 16);
                int g = Y - ((22554 * U + 46802 * V) >> 16);
                int b = Y + ((116130 * U) >> 16);
                r = r < 0 ? 0 : Math.min(r, 255);
                g = g < 0 ? 0 : Math.min(g, 255);
                b = b < 0 ? 0 : Math.min(b, 255);
                argb[y * w + x] = 0xff000000 | (r << 16) | (g << 8) | b;
            }
        }
        bitmap.setPixels(argb, 0, w, 0, 0, w, h);
        return bitmap;
    }
}
