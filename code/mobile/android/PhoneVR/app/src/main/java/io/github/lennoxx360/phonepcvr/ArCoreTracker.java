/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import android.app.Activity;
import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.PowerManager;
import android.os.SystemClock;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.SharedCamera;
import com.google.ar.core.TrackingFailureReason;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.core.exceptions.ResourceExhaustedException;
import com.google.ar.core.exceptions.UnavailableException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Locale;

/**
 * Runs an ARCore session next to the Cardboard head tracker and reports its pose.
 *
 * <p>Milestone 1: the pose is only displayed and logged, not sent to ALVR yet. Threading: {@link
 * #resume}, {@link #pause} and {@link #close} run on the UI thread; the {@code onSurface*} and
 * {@link #onDrawFrame} calls run on the GLSurfaceView render thread. The activity must pause the
 * GLSurfaceView before calling {@link #pause}, so update() never races with pause().
 */
public class ArCoreTracker {
    public interface StatusListener {
        /** Called on the render thread roughly 5 times a second. */
        void onArCoreStatus(String text);
    }

    public interface PoseListener {
        /**
         * Called on the render thread once per new ARCore camera frame. Pose is the
         * display-oriented camera pose (ARCore world from camera, OpenGL convention).
         *
         * @param timestampNs capture time on the CLOCK_BOOTTIME base (elapsedRealtimeNanos)
         */
        void onArCorePose(
                long timestampNs, boolean tracking, float[] translation, float[] rotation);
    }

    private static final String TAG = "PVR-ARCore";
    private static final long STATUS_INTERVAL_NS = 200_000_000L;
    private static final long LOG_INTERVAL_NS = 1_000_000_000L;

    private final Activity activity;
    private final StatusListener listener;
    private final PoseListener poseListener;
    private volatile HandTracker handTracker;

    public void setHandTracker(HandTracker tracker) {
        handTracker = tracker;
        if (tracker != null) tracker.setMatchStats(this::lastMatchDeltaMs);
    }

    private volatile Session session;
    private Session textureBoundSession;
    private boolean installRequested;
    private volatile String unavailableReason;

    // Why ARCore isn't tracking right now (e.g. "INSUFFICIENT_LIGHT"), or null while tracking.
    private volatile String failureReason;

    public String getFailureReason() {
        return failureReason;
    }

    // ---- Shared camera (hands get their own stream) ----
    // With hand tracking on, the app opens the camera itself (ARCore Shared Camera API) and builds
    // one capture session holding ARCore's streams plus a separate higher-resolution YUV stream for
    // MediaPipe. ARCore's own 640x480 CPU image (which hands used to get, halved to 320x240) is no
    // longer touched. Same physical camera: same 76x60 deg view, frame rate and exposure.
    private boolean wantSharedCamera = false;
    private boolean sharedMode = false; // decided when the session is created
    private SharedCamera sharedCamera;
    private String cameraId;
    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private ImageReader handReader;
    private volatile CameraDevice cameraDevice;
    private volatile CameraCaptureSession captureSession;
    private volatile boolean cameraOpening = false;
    private Size handSize;
    private int arCpuWidth; // ARCore CPU image size, which getImageIntrinsics() refers to
    private static final int HAND_STREAM_MAX_WIDTH = 1280;

    /**
     * Render thread (glView.queueEvent), so it can't race with session.update(). Switches between
     * ARCore owning the camera and the shared mode with the hands' own stream by recreating the
     * ARCore session; 6DoF re-acquires (a few seconds of "6DoF resuming").
     */
    public void setCameraMode(boolean shared) {
        Session s = session;
        if (s == null) {
            wantSharedCamera = shared; // used when the session is created
            return;
        }
        if (sharedMode == shared) return;
        Log.i(
                TAG,
                "switching camera mode to "
                        + (shared ? "shared (hands HD stream)" : "ARCore only"));
        s.pause();
        if (sharedMode) tearDownSharedCamera();
        session = null;
        s.close();
        textureBoundSession = null;
        synchronized (this) {
            java.util.Arrays.fill(ringPose, null);
        }
        wantSharedCamera = shared;
        resume();
    }

    private void tearDownSharedCamera() {
        closeSharedCamera();
        if (handReader != null) handReader.close();
        handReader = null;
        if (cameraThread != null) cameraThread.quitSafely();
        cameraThread = null;
        cameraHandler = null;
        sharedCamera = null;
    }

    /** UI thread, before the first resume(): give hands their own camera stream. */
    public void setSharedCameraForHands(boolean on) {
        wantSharedCamera = on;
    }

    /** Size of the hands' own stream, or null when hands use ARCore's CPU image. */
    public Size handStreamSize() {
        return sharedMode ? handSize : null;
    }

    // Recent ARCore physical-camera poses by camera timestamp, for matching hand frames (render
    // thread writes, camera/hand threads read).
    private static final int POSE_RING = 32;
    private final long[] ringTs = new long[POSE_RING];
    private final float[][] ringPose = new float[POSE_RING][];
    private final float[][] ringIntrinsics = new float[POSE_RING][];
    private final boolean[] ringTracking = new boolean[POSE_RING];
    private int ringNext = 0;

    private synchronized void recordPose(
            long ts, float[] pose, float[] intrinsics, boolean tracking) {
        ringTs[ringNext] = ts;
        ringPose[ringNext] = pose;
        ringIntrinsics[ringNext] = intrinsics;
        ringTracking[ringNext] = tracking;
        ringNext = (ringNext + 1) % POSE_RING;
    }

    /**
     * Pose (tx..qw) of the ARCore frame captured at cameraTs, or null if ARCore hasn't processed it
     * (yet) or wasn't tracking. out[0] gets ARCore's intrinsics for its CPU image.
     */
    synchronized float[] poseAt(long cameraTs, float[][] intrinsicsOut, boolean finalTry) {
        // Interpolate between the tracked ARCore frames just before and after the hand frame.
        // (In shared mode ARCore's frame timestamps don't match the camera frames 1:1, so an
        // exact match never happened on 2026-11-22.) Before ARCore has passed the hand frame,
        // return null so the caller waits; on the final try, take the nearest within 60 ms.
        int before = -1, after = -1;
        for (int i = 0; i < POSE_RING; i++) {
            if (ringPose[i] == null || !ringTracking[i]) continue;
            if (ringTs[i] <= cameraTs && (before < 0 || ringTs[i] > ringTs[before])) before = i;
            if (ringTs[i] >= cameraTs && (after < 0 || ringTs[i] < ringTs[after])) after = i;
        }
        int nearest =
                before < 0
                        ? after
                        : after < 0
                                ? before
                                : (cameraTs - ringTs[before] <= ringTs[after] - cameraTs
                                        ? before
                                        : after);
        if (nearest >= 0) lastMatchDeltaNs = cameraTs - ringTs[nearest];
        if (before >= 0 && after >= 0 && ringTs[after] - ringTs[before] <= 100_000_000L) {
            intrinsicsOut[0] = ringIntrinsics[before];
            if (after == before) return ringPose[before];
            float t = (cameraTs - ringTs[before]) / (float) (ringTs[after] - ringTs[before]);
            return lerpPose(ringPose[before], ringPose[after], t);
        }
        if (!finalTry || nearest < 0 || Math.abs(cameraTs - ringTs[nearest]) > 60_000_000L) {
            return null;
        }
        intrinsicsOut[0] = ringIntrinsics[nearest];
        return ringPose[nearest];
    }

    private volatile long lastMatchDeltaNs = 0;

    /** Hand frame time minus the nearest ARCore frame time at the last lookup (ms). */
    float lastMatchDeltaMs() {
        return lastMatchDeltaNs / 1e6f;
    }

    // tx ty tz qx qy qz qw: linear position, normalised linear rotation (fine within 100 ms).
    private static float[] lerpPose(float[] a, float[] b, float t) {
        float[] o = new float[7];
        for (int i = 0; i < 3; i++) o[i] = a[i] + t * (b[i] - a[i]);
        float dot = a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6];
        float sign = dot < 0 ? -1f : 1f;
        float n = 0;
        for (int i = 3; i < 7; i++) {
            o[i] = a[i] + t * (sign * b[i] - a[i]);
            n += o[i] * o[i];
        }
        n = (float) Math.sqrt(n);
        for (int i = 3; i < 7; i++) o[i] /= n;
        return o;
    }

    private int cameraTexture = 0;
    private int displayRotation = -1;
    private int displayWidth;
    private int displayHeight;

    // Render-thread state for the status readout.
    private long lastCameraTimestampNs;
    private long lastStatusNs;
    private long lastLogNs;
    private int cameraFramesSinceStatus;
    private float cameraFps;
    private float[] startPosition;
    private final StringBuilder sharedDeltas = new StringBuilder();
    private int sharedDeltasLogged = 0;

    public ArCoreTracker(Activity activity, StatusListener listener, PoseListener poseListener) {
        this.activity = activity;
        this.listener = listener;
        this.poseListener = poseListener;
    }

    // Switched off by the user (settings menu): camera closed, tracking falls back to 3DoF.
    private volatile boolean userDisabled = false;
    private boolean disableReported = false;

    /** UI thread, before the first resume(): start with ARCore off. */
    public void setInitiallyEnabled(boolean enabled) {
        userDisabled = !enabled;
    }

    /** Render thread (glView.queueEvent), so it can't race with session.update(). */
    public void setEnabled(boolean enabled) {
        if (enabled == !userDisabled) return;
        if (enabled) {
            userDisabled = false;
            resume();
        } else {
            userDisabled = true;
            disableReported = false;
            pause();
        }
    }

    /** UI thread. Creates the session on first call (may ask to install ARCore) and resumes it. */
    public void resume() {
        if (userDisabled) return;
        if (session == null) {
            try {
                ArCoreApk.Availability availability =
                        ArCoreApk.getInstance().checkAvailability(activity);
                if (availability.isUnsupported()) {
                    setUnavailable("this phone does not support ARCore (3DoF only)");
                    return;
                }
                switch (ArCoreApk.getInstance().requestInstall(activity, !installRequested)) {
                    case INSTALL_REQUESTED:
                        // ARCore install/update screen is showing; we get onResume() again after.
                        installRequested = true;
                        return;
                    case INSTALLED:
                        break;
                }
                sharedMode = wantSharedCamera;
                Session newSession =
                        sharedMode
                                ? new Session(activity, EnumSet.of(Session.Feature.SHARED_CAMERA))
                                : new Session(activity);
                Config config = new Config(newSession);
                // Never block the render loop waiting for a camera frame (camera runs at ~30 fps).
                config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE);
                config.setFocusMode(Config.FocusMode.FIXED);
                config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED);
                config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
                newSession.configure(config);
                session = newSession;
                unavailableReason = null;
                if (sharedMode) setUpSharedCamera(newSession);
            } catch (UnavailableUserDeclinedInstallationException e) {
                setUnavailable("ARCore install was declined (3DoF only)");
                return;
            } catch (UnavailableException e) {
                setUnavailable("ARCore unavailable: " + e.getClass().getSimpleName());
                return;
            } catch (RuntimeException e) {
                setUnavailable("ARCore failed to start: " + e.getMessage());
                return;
            }
        }
        if (sharedMode) {
            openSharedCamera(); // session.resume() follows once the capture session is running
            return;
        }
        try {
            session.resume();
        } catch (Exception e) {
            // Usually CameraNotAvailableException: another app holds the camera.
            setUnavailable("ARCore camera unavailable: " + e.getClass().getSimpleName());
        }
    }

    // ---- Shared camera setup (UI thread) and lifecycle (camera thread) ----

    private void setUpSharedCamera(Session s) {
        sharedCamera = s.getSharedCamera();
        cameraId = s.getCameraConfig().getCameraId();
        arCpuWidth = s.getCameraConfig().getImageSize().getWidth();
        Size cpu = s.getCameraConfig().getImageSize();
        handSize = pickHandStreamSize(cameraId, cpu);
        cameraThread = new HandlerThread("PVR-Camera");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
        handReader =
                ImageReader.newInstance(
                        handSize.getWidth(), handSize.getHeight(), ImageFormat.YUV_420_888, 3);
        handReader.setOnImageAvailableListener(this::onHandImage, cameraHandler);
        sharedCamera.setAppSurfaces(cameraId, Arrays.asList(handReader.getSurface()));
        Log.i(
                TAG,
                "shared camera "
                        + cameraId
                        + ": ARCore CPU image "
                        + cpu
                        + ", hands stream "
                        + handSize
                        + ", texture "
                        + s.getCameraConfig().getTextureSize()
                        + ", fps "
                        + s.getCameraConfig().getFpsRange());
    }

    // Largest YUV size with ARCore's CPU-image aspect ratio (so its intrinsics scale exactly), at
    // most HAND_STREAM_MAX_WIDTH wide.
    private Size pickHandStreamSize(String id, Size cpu) {
        Size best = cpu;
        try {
            CameraManager cm = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
            StreamConfigurationMap map =
                    cm.getCameraCharacteristics(id)
                            .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            float aspect = cpu.getWidth() / (float) cpu.getHeight();
            for (Size sz : map.getOutputSizes(ImageFormat.YUV_420_888)) {
                float a = sz.getWidth() / (float) sz.getHeight();
                if (Math.abs(a - aspect) > 0.01f || sz.getWidth() > HAND_STREAM_MAX_WIDTH) continue;
                if (sz.getWidth() > best.getWidth()) best = sz;
            }
        } catch (Exception e) {
            Log.w(TAG, "couldn't list camera sizes, hands use " + cpu, e);
        }
        return best;
    }

    private void openSharedCamera() {
        if (cameraDevice != null || cameraOpening || sharedCamera == null) return;
        cameraOpening = true;
        try {
            CameraManager cm = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
            cm.openCamera(
                    cameraId,
                    sharedCamera.createARDeviceStateCallback(deviceCallback, cameraHandler),
                    cameraHandler);
        } catch (SecurityException | android.hardware.camera2.CameraAccessException e) {
            cameraOpening = false;
            setUnavailable("camera open failed: " + e.getClass().getSimpleName());
        }
    }

    private final CameraDevice.StateCallback deviceCallback =
            new CameraDevice.StateCallback() {
                @Override
                public void onOpened(CameraDevice device) {
                    cameraDevice = device;
                    cameraOpening = false;
                    try {
                        List<Surface> surfaces = new ArrayList<>(sharedCamera.getArCoreSurfaces());
                        surfaces.add(handReader.getSurface());
                        device.createCaptureSession(
                                surfaces,
                                sharedCamera.createARSessionStateCallback(
                                        sessionCallback, cameraHandler),
                                cameraHandler);
                    } catch (Exception e) {
                        setUnavailable("camera session failed: " + e.getMessage());
                    }
                }

                @Override
                public void onDisconnected(CameraDevice device) {
                    device.close();
                    cameraDevice = null;
                    cameraOpening = false;
                }

                @Override
                public void onError(CameraDevice device, int error) {
                    Log.w(TAG, "camera error " + error);
                    device.close();
                    cameraDevice = null;
                    cameraOpening = false;
                }
            };

    private final CameraCaptureSession.StateCallback sessionCallback =
            new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(CameraCaptureSession cs) {
                    captureSession = cs;
                    CameraDevice device = cameraDevice;
                    Session s = session;
                    if (device == null || s == null) return;
                    try {
                        CaptureRequest.Builder b =
                                device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                        for (Surface surface : sharedCamera.getArCoreSurfaces())
                            b.addTarget(surface);
                        b.addTarget(handReader.getSurface());
                        // What ARCore needs for tracking: no stabilisation (it would move the image
                        // under ARCore's lens model), fixed focus, ARCore's frame rate.
                        b.set(
                                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                        b.set(
                                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                        b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
                        Float hyperfocal = hyperfocalDiopters();
                        if (hyperfocal != null)
                            b.set(CaptureRequest.LENS_FOCUS_DISTANCE, hyperfocal);
                        Range<Integer> fps = s.getCameraConfig().getFpsRange();
                        if (fps != null) b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps);
                        cs.setRepeatingRequest(b.build(), null, cameraHandler);
                        s.resume();
                        Log.i(TAG, "shared camera running, ARCore resumed");
                    } catch (Exception e) {
                        setUnavailable("shared camera start failed: " + e.getMessage());
                    }
                }

                @Override
                public void onConfigureFailed(CameraCaptureSession cs) {
                    setUnavailable("camera configuration failed (hands stream " + handSize + ")");
                }
            };

    private Float hyperfocalDiopters() {
        try {
            CameraManager cm = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
            return cm.getCameraCharacteristics(cameraId)
                    .get(CameraCharacteristics.LENS_INFO_HYPERFOCAL_DISTANCE);
        } catch (Exception e) {
            return null;
        }
    }

    private void closeSharedCamera() {
        CameraCaptureSession cs = captureSession;
        captureSession = null;
        if (cs != null) cs.close();
        CameraDevice d = cameraDevice;
        cameraDevice = null;
        if (d != null) d.close();
    }

    private String handClockLogged = null;

    private long handImageTsToBoot(long ts) {
        long nowBoot = SystemClock.elapsedRealtimeNanos();
        String base;
        long out;
        if (nowBoot - ts >= 0 && nowBoot - ts < 1_000_000_000L) {
            base = "boot clock";
            out = ts;
        } else {
            long nowMono = System.nanoTime();
            if (nowMono - ts >= 0 && nowMono - ts < 1_000_000_000L) {
                base = "monotonic clock (converted)";
                out = ts + (nowBoot - nowMono);
            } else {
                base = "unknown clock (using arrival time)";
                out = nowBoot;
            }
        }
        if (!base.equals(handClockLogged)) {
            handClockLogged = base;
            Log.i(
                    TAG,
                    "hands stream timestamps: " + base + ", age " + (nowBoot - out) / 1e6 + " ms");
        }
        return out;
    }

    // Camera thread: a frame of the hands' own stream.
    private void onHandImage(ImageReader reader) {
        Image image;
        try {
            image = reader.acquireLatestImage();
        } catch (IllegalStateException e) {
            return; // reader closed during a camera-mode switch
        }
        if (image == null) return;
        HandTracker hands = handTracker;
        if (hands == null || !hands.wantsFrame()) {
            image.close();
            return;
        }
        // Matching happens on the boot clock (ARCore's frame timestamps are on it, checked at
        // start); this stream's sensor timestamps may be on the boot or the monotonic clock.
        long bootTs = handImageTsToBoot(image.getTimestamp());
        float scale = handSize.getWidth() / (float) arCpuWidth;
        hands.submitShared(image, bootTs, bootTs, scale, this::poseAt);
    }

    /** UI thread, after the GLSurfaceView has been paused. */
    public void pause() {
        Session s = session;
        if (s != null) s.pause();
        if (sharedMode) closeSharedCamera();
    }

    /** UI thread. */
    public void close() {
        Session s = session;
        session = null;
        if (s != null) s.close();
        if (sharedMode) tearDownSharedCamera();
    }

    /** Render thread. A new GL context means the old camera texture is gone. */
    public void onSurfaceCreated() {
        int[] tex = new int[1];
        GLES20.glGenTextures(1, tex, 0);
        cameraTexture = tex[0];
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, cameraTexture);
        GLES20.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
                GLES20.GL_TEXTURE_WRAP_S,
                GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
                GLES20.GL_TEXTURE_WRAP_T,
                GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, 0);
        textureBoundSession = null;
    }

    /** Render thread. */
    public void onSurfaceChanged(int rotation, int width, int height) {
        displayRotation = rotation;
        displayWidth = width;
        displayHeight = height;
        textureBoundSession = null; // re-apply geometry on the next frame
    }

    /** Render thread, once per rendered frame, before the native renderer runs. */
    public void onDrawFrame() {
        long now = SystemClock.elapsedRealtimeNanos();
        if (userDisabled) {
            if (!disableReported) {
                // Tell the fusion tracking stopped: position freezes, gyro rotation continues.
                disableReported = true;
                poseListener.onArCorePose(now, false, new float[3], new float[] {0, 0, 0, 1});
            }
            if (now - lastStatusNs >= STATUS_INTERVAL_NS) {
                lastStatusNs = now;
                listener.onArCoreStatus("ARCore: OFF (3DoF, camera closed)\n" + thermalLine());
            }
            return;
        }
        Session s = session;
        if (s == null) {
            String reason = unavailableReason;
            if (reason != null && now - lastStatusNs >= STATUS_INTERVAL_NS) {
                lastStatusNs = now;
                listener.onArCoreStatus("ARCore: " + reason + "\n" + thermalLine());
            }
            return;
        }
        if (textureBoundSession != s && cameraTexture != 0) {
            s.setCameraTextureName(cameraTexture);
            if (displayRotation >= 0) {
                s.setDisplayGeometry(displayRotation, displayWidth, displayHeight);
            }
            textureBoundSession = s;
        }

        Frame frame;
        try {
            frame = s.update();
        } catch (Exception e) {
            if (now - lastLogNs >= LOG_INTERVAL_NS) {
                lastLogNs = now;
                Log.w(TAG, "session.update() failed", e);
            }
            return;
        }
        long frameTs = frame.getTimestamp();
        if (sharedMode && sharedDeltasLogged < 20 && frameTs != lastCameraTimestampNs) {
            sharedDeltas.append(' ').append((frameTs - lastCameraTimestampNs) / 100_000L / 10f);
            if (++sharedDeltasLogged == 20) {
                Log.i(TAG, "shared mode ARCore frame intervals (ms):" + sharedDeltas);
            }
        }
        // In shared mode ARCore sometimes delivers frames < 5 ms apart (seen as "60 fps" and
        // absurd turn rates in fusion on 2026-11-22). Only frames >= 5 ms apart reach fusion.
        boolean newFrame =
                sharedMode
                        ? Math.abs(frameTs - lastCameraTimestampNs) >= 5_000_000L
                        : frameTs != lastCameraTimestampNs;
        if (newFrame) {
            lastCameraTimestampNs = frameTs;
            cameraFramesSinceStatus++;
            Camera camera = frame.getCamera();
            Pose pose = camera.getDisplayOrientedPose();
            long captureNs = toBootTimeNs(frame.getTimestamp(), now);
            boolean tracking = camera.getTrackingState() == TrackingState.TRACKING;
            poseListener.onArCorePose(
                    captureNs, tracking, pose.getTranslation(), pose.getRotationQuaternion());

            HandTracker hands = handTracker;
            if (sharedMode) {
                // Hands run on their own stream; they look the camera pose up by timestamp.
                if (hands != null && hands.isEnabled()) {
                    CameraIntrinsics k = camera.getImageIntrinsics();
                    float[] f = k.getFocalLength(), c = k.getPrincipalPoint();
                    int[] dim = k.getImageDimensions();
                    Pose cam = camera.getPose(); // physical camera, image axes (not display)
                    float[] t = cam.getTranslation(), q = cam.getRotationQuaternion();
                    recordPose(
                            frame.getTimestamp(),
                            new float[] {t[0], t[1], t[2], q[0], q[1], q[2], q[3]},
                            new float[] {f[0], f[1], c[0], c[1], dim[0], dim[1]},
                            tracking);
                }
            } else if (tracking && hands != null && hands.wantsFrame()) {
                // Legacy: hand tracking reuses ARCore's CPU image (no second stream).
                try {
                    Image image = frame.acquireCameraImage();
                    CameraIntrinsics k = camera.getImageIntrinsics();
                    float[] f = k.getFocalLength(), c = k.getPrincipalPoint();
                    int[] dim = k.getImageDimensions();
                    Pose cam = camera.getPose(); // physical camera, image axes (not display)
                    float[] t = cam.getTranslation(), q = cam.getRotationQuaternion();
                    hands.submit(
                            image,
                            captureNs,
                            new float[] {f[0], f[1], c[0], c[1], dim[0], dim[1]},
                            new float[] {t[0], t[1], t[2], q[0], q[1], q[2], q[3]});
                } catch (NotYetAvailableException | ResourceExhaustedException e) {
                    // No CPU image this frame; try the next one.
                }
            }
        }

        if (now - lastStatusNs < STATUS_INTERVAL_NS) return;
        cameraFps = cameraFramesSinceStatus * 1e9f / (now - lastStatusNs);
        cameraFramesSinceStatus = 0;
        lastStatusNs = now;

        String text = describe(frame.getCamera());
        listener.onArCoreStatus(text);
        if (now - lastLogNs >= LOG_INTERVAL_NS) {
            lastLogNs = now;
            Log.i(TAG, text.replace("\n", " | "));
        }
    }

    private String describe(Camera camera) {
        TrackingState state = camera.getTrackingState();
        TrackingFailureReason why = camera.getTrackingFailureReason();
        failureReason =
                state == TrackingState.TRACKING
                        ? null
                        : (why == TrackingFailureReason.NONE ? state.name() : why.name());
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.US, "ARCore %s  cam %.0f fps", state, cameraFps));
        if (state != TrackingState.TRACKING) {
            TrackingFailureReason reason = camera.getTrackingFailureReason();
            if (reason != TrackingFailureReason.NONE) sb.append("\nlost: ").append(reason);
        }
        if (state != TrackingState.STOPPED) {
            // Display-oriented pose: OpenGL camera convention (looking down -Z), Y up, metres.
            Pose pose = camera.getDisplayOrientedPose();
            float[] t = pose.getTranslation();
            if (startPosition == null && state == TrackingState.TRACKING) startPosition = t;
            float[] q = pose.getRotationQuaternion(); // x, y, z, w
            sb.append(String.format(Locale.US, "\npos %+.2f %+.2f %+.2f m", t[0], t[1], t[2]));
            sb.append(
                    String.format(
                            Locale.US,
                            "\nyaw %+4.0f  pitch %+3.0f  roll %+4.0f",
                            yawDeg(q),
                            pitchDeg(q),
                            rollDeg(q)));
            if (startPosition != null) {
                float dx = t[0] - startPosition[0];
                float dy = t[1] - startPosition[1];
                float dz = t[2] - startPosition[2];
                sb.append(
                        String.format(
                                Locale.US,
                                "\nmoved %.2f m",
                                (float) Math.sqrt(dx * dx + dy * dy + dz * dz)));
            }
        }
        sb.append('\n').append(thermalLine());
        return sb.toString();
    }

    // ARCore's frame timestamp has no documented time base. On most phones it is the camera
    // sensor time = CLOCK_BOOTTIME; if it doesn't look like that, fall back to "now".
    private boolean timeBaseLogged;

    private long toBootTimeNs(long frameTimestampNs, long nowBootNs) {
        long age = nowBootNs - frameTimestampNs;
        boolean bootTime = age >= 0 && age < 500_000_000L;
        if (!timeBaseLogged) {
            timeBaseLogged = true;
            Log.i(
                    TAG,
                    "frame timestamp age "
                            + age / 1_000_000.0
                            + " ms -> "
                            + (bootTime ? "BOOTTIME base" : "unknown base, using now"));
        }
        return bootTime ? frameTimestampNs : nowBootNs;
    }

    private String thermalLine() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) return "thermal: n/a";
        PowerManager pm = (PowerManager) activity.getSystemService(Context.POWER_SERVICE);
        switch (pm.getCurrentThermalStatus()) {
            case PowerManager.THERMAL_STATUS_NONE:
                return "thermal: ok";
            case PowerManager.THERMAL_STATUS_LIGHT:
                return "thermal: LIGHT";
            case PowerManager.THERMAL_STATUS_MODERATE:
                return "thermal: MODERATE";
            case PowerManager.THERMAL_STATUS_SEVERE:
                return "thermal: SEVERE (throttling)";
            default:
                return "thermal: CRITICAL";
        }
    }

    private void setUnavailable(String reason) {
        unavailableReason = reason;
        Log.w(TAG, reason);
    }

    // Euler angles for display only, Y-up (yaw about Y, pitch about X, roll about Z).
    private static float yawDeg(float[] q) {
        return (float)
                Math.toDegrees(
                        Math.atan2(
                                2 * (q[3] * q[1] + q[0] * q[2]),
                                1 - 2 * (q[0] * q[0] + q[1] * q[1])));
    }

    private static float pitchDeg(float[] q) {
        float s = 2 * (q[3] * q[0] - q[1] * q[2]);
        return (float) Math.toDegrees(Math.asin(Math.max(-1f, Math.min(1f, s))));
    }

    private static float rollDeg(float[] q) {
        return (float)
                Math.toDegrees(
                        Math.atan2(
                                2 * (q[3] * q[2] + q[0] * q[1]),
                                1 - 2 * (q[0] * q[0] + q[2] * q[2])));
    }
}
