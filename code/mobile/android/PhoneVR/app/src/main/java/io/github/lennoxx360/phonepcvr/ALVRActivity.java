/* (C)2023 */
package io.github.lennoxx360.phonepcvr;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.media.AudioManager;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.BatteryManager;
import android.os.Build.VERSION;
import android.os.Build.VERSION_CODES;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Choreographer;
import android.view.Display;
import android.view.HapticFeedbackConstants;
import android.view.KeyEvent;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.PopupMenu;
import android.widget.TextView;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import java.util.Objects;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class ALVRActivity extends AppCompatActivity
        implements PopupMenu.OnMenuItemClickListener, BatteryLevelListener {

    static {
        System.loadLibrary("native-lib-alvr");
    }

    private static final String TAG = ALVRActivity.class.getSimpleName() + "-Java";

    // Permission request codes
    private static final int PERMISSIONS_REQUEST_CODE = 2;
    private static final int CAMERA_PERMISSION_REQUEST_CODE = 3;

    private static final float STREAM_FPS = 60f;

    /** Intent extra: open the Cardboard headset QR scanner once the stream screen is up. */
    public static final String EXTRA_SCAN_VIEWER = "scan_viewer";

    private volatile float targetRefreshRate = 0f;

    private GLSurfaceView glView;

    private ArCoreTracker arCore;
    private HandTracker handTracker;
    private View arOverlay;
    private TextView arTextLeft;
    private TextView arTextRight;
    private boolean cameraPermissionAsked = false;

    private final BatteryMonitor bMonitor = new BatteryMonitor(this);
    private final ManualAnnouncer announcer = new ManualAnnouncer();
    private String alvrHostname = "";

    // Headset-fit adjust mode: -1 = off, otherwise the index into Settings.FIT.
    private int fitIndex = -1;
    private View fitOverlay;
    private TextView fitTextLeft;
    private TextView fitTextRight;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private static final long FIT_IDLE_EXIT_MS = 12000;

    // ---- Low-latency frame pacing ----
    // GLSurfaceView normally renders as soon as a buffer is free, so frames queue up ~3.7
    // refreshes (62 ms measured) before they're shown. With pacing on, the renderer draws once per
    // vsync, starting pacingBudgetNs before the next one, so the queue stays nearly empty.
    private volatile boolean pacing = false;
    private volatile long pacingBudgetNs = 9_000_000L;
    private volatile long lastVsyncNs = 0; // System.nanoTime() base, from Choreographer
    private volatile long vsyncPeriodNs = 16_666_667L;
    private boolean vsyncCallbackPosted = false;
    private final Choreographer.FrameCallback vsyncCallback =
            new Choreographer.FrameCallback() {
                @Override
                public void doFrame(long frameTimeNanos) {
                    long last = lastVsyncNs;
                    if (last != 0) {
                        long d = frameTimeNanos - last;
                        if (d > 4_000_000L && d < 40_000_000L) {
                            vsyncPeriodNs += (d - vsyncPeriodNs) / 8;
                        }
                    }
                    lastVsyncNs = frameTimeNanos;
                    if (pacing) {
                        glView.requestRender();
                        Choreographer.getInstance().postFrameCallback(this);
                    } else {
                        vsyncCallbackPosted = false;
                    }
                }
            };
    private final Runnable fitIdleExit = () -> exitFitMode(true);

    public static class BatteryMonitor extends BroadcastReceiver {
        private final BatteryLevelListener listener;

        public BatteryMonitor(BatteryLevelListener listener) {
            this.listener = listener;
        }

        public void startMonitoring(Context context) {
            // Register BroadcastReceiver to monitor battery level changes
            IntentFilter filter = new IntentFilter(Intent.ACTION_BATTERY_CHANGED);
            context.registerReceiver(this, filter);
        }

        public void stopMonitoring(Context context) {
            // Unregister the BroadcastReceiver
            context.unregisterReceiver(this);
        }

        @Override
        public void onReceive(Context context, Intent intent) {
            if (Objects.equals(intent.getAction(), Intent.ACTION_BATTERY_CHANGED)) {
                // Get the current battery level and scale
                int level = intent.getIntExtra(BatteryManager.EXTRA_LEVEL, 0);
                int scale = intent.getIntExtra(BatteryManager.EXTRA_SCALE, 100);

                // Calculate the battery percentage as a float value between 0 and 1
                float batteryPercentage = (float) level / scale;

                // Get the charging state
                int chargingState = intent.getIntExtra(BatteryManager.EXTRA_PLUGGED, -1);
                boolean isCharging =
                        chargingState == BatteryManager.BATTERY_PLUGGED_AC
                                || chargingState == BatteryManager.BATTERY_PLUGGED_USB
                                || chargingState == BatteryManager.BATTERY_PLUGGED_WIRELESS;

                // Notify the listener with the current battery percentage and charging state
                if (listener != null) {
                    listener.onBatteryLevelChanged(batteryPercentage, isCharging);
                }
            }
        }
    }

    @Override
    public void onCreate(Bundle savedInstance) {
        super.onCreate(savedInstance);
        Log.d(TAG, "onCreate ALVRActivity");

        DisplayMetrics displayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
        int width = displayMetrics.widthPixels;
        int height = displayMetrics.heightPixels;

        // get refresh rate of the device
        Display display = getWindowManager().getDefaultDisplay();
        float refreshRate = display.getRefreshRate();
        Log.i(TAG, "Refresh rate: " + refreshRate);

        // The screen runs as fast as it can (see requestHighRefreshRate), but ALVR is told 60 Hz so
        // the stream stays at 60 fps; timewarp re-projects the extra display refreshes.
        initializeNative(width, height, STREAM_FPS);
        // Before the renderer starts, so the first lens build already has the fit applied.
        pushFit(false);
        alvrHostname = getHostnameNative();
        Log.i(TAG, "ALVR hostname: " + alvrHostname);
        getSharedPreferences("settings", MODE_PRIVATE)
                .edit()
                .putString("alvr_hostname", alvrHostname)
                .apply();

        setContentView(R.layout.activity_vr);
        glView = findViewById(R.id.surface_view);
        requestHighRefreshRate();

        arOverlay = findViewById(R.id.arcore_overlay);
        fitOverlay = findViewById(R.id.fit_overlay);
        fitTextLeft = findViewById(R.id.fit_text_left);
        fitTextRight = findViewById(R.id.fit_text_right);
        arTextLeft = findViewById(R.id.arcore_text_left);
        arTextRight = findViewById(R.id.arcore_text_right);
        problemOverlay = findViewById(R.id.problem_overlay);
        problemTextLeft = findViewById(R.id.problem_text_left);
        problemTextRight = findViewById(R.id.problem_text_right);
        setComfortGuardNative(settings().getBoolean(Settings.COMFORT_GUARD, true));
        arCore = new ArCoreTracker(this, this::showArCoreStatus, this::onArCorePose);
        arCore.setInitiallyEnabled(
                getSharedPreferences("settings", MODE_PRIVATE).getBoolean("arcore_6dof", true));
        handTracker = new HandTracker(this, this::setHandsNative);
        handTracker.setEnabled(
                getSharedPreferences("settings", MODE_PRIVATE).getBoolean("hand_tracking", false));
        arCore.setHandTracker(handTracker);
        // Decided once per VR start: the camera mode can't change while ARCore runs.
        arCore.setSharedCameraForHands(
                getSharedPreferences("settings", MODE_PRIVATE).getBoolean("hand_tracking", false)
                        && Settings.experimentalBool(settings(), Settings.HANDS_HD_CAMERA));
        updateArOverlayVisibility();
        setTimewarpEnabledNative(
                getSharedPreferences("settings", MODE_PRIVATE).getBoolean("timewarp", true));
        glView.setEGLContextClientVersion(3);
        Renderer renderer = new Renderer();
        glView.setRenderer(renderer);
        glView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        applyStreamExtras(getIntent());
        pushStreamOptions();

        setImmersiveSticky();
        View decorView = getWindow().getDecorView();
        decorView.setOnSystemUiVisibilityChangeListener(
                (visibility) -> {
                    if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                        setImmersiveSticky();
                    }
                });

        // Forces screen to max brightness.
        // get setting max_brightness boolean and set brightness to max if required
        SharedPreferences prefs = getSharedPreferences("settings", MODE_PRIVATE);
        if (prefs.getBoolean("max_brightness", true)) {
            WindowManager.LayoutParams layout = getWindow().getAttributes();
            layout.screenBrightness = 1.f;
            getWindow().setAttributes(layout);
        }

        // Prevents screen from dimming/locking.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    @Override
    public void onBatteryLevelChanged(float batteryPercentage, boolean isPlugged) {
        sendBatteryLevel(batteryPercentage, isPlugged);
        Log.d(TAG, "Battery level changed: " + batteryPercentage + ", isPlugged in? :" + isPlugged);
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.d(TAG, "Pausing ALVR Activity");
        pauseNative();
        stopPacing();
        glView.onPause();
        // Only after the render thread has stopped, so session.update() can't race with pause().
        arCore.pause();
        bMonitor.stopMonitoring(this);
        try {
            unregisterReceiver(liveSettingsReceiver);
        } catch (IllegalArgumentException ignored) {
        }
        announcer.stop();
        uiHandler.removeCallbacks(autoRecenter);
        uiHandler.removeCallbacks(problemPoll);
        gestures.cancel();
        exitFitMode(false);
        saveProfileIpd();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.d(TAG, "Resuming ALVR Activity");

        if (VERSION.SDK_INT < VERSION_CODES.Q && !isReadExternalStorageEnabled()) {
            final String[] permissions = new String[] {Manifest.permission.READ_EXTERNAL_STORAGE};
            ActivityCompat.requestPermissions(this, permissions, PERMISSIONS_REQUEST_CODE);

            return;
        }

        if (isCameraEnabled()) {
            arCore.resume();
        } else if (!cameraPermissionAsked) {
            cameraPermissionAsked = true;
            ActivityCompat.requestPermissions(
                    this,
                    new String[] {Manifest.permission.CAMERA},
                    CAMERA_PERMISSION_REQUEST_CODE);
        }

        glView.onResume();
        resumeNative();
        applyPacing();
        uiHandler.post(problemPoll);
        bMonitor.startMonitoring(this);
        registerLiveSettingsReceiver();

        if (settings().getBoolean(Settings.RECENTER_PENDING, false)
                || settings().getBoolean(Settings.RECENTER_ON_START, false)) {
            uiHandler.postDelayed(autoRecenter, AUTO_RECENTER_DELAY_MS);
        }

        SharedPreferences net = getSharedPreferences("settings", MODE_PRIVATE);
        if (net.getBoolean("network_manual", false)) {
            announcer.start(net.getString("pc_ip", ""), alvrHostname);
        }

        if (getIntent().getBooleanExtra(EXTRA_SCAN_VIEWER, false)) {
            getIntent().removeExtra(EXTRA_SCAN_VIEWER);
            switchViewerNative();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.d(TAG, "Destroying ALVR Activity");
        arCore.setHandTracker(null);
        handTracker.close();
        arCore.close();
        destroyNative();
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            setImmersiveSticky();
        }
    }

    private class Renderer implements GLSurfaceView.Renderer {
        @Override
        public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
            arCore.onSurfaceCreated();
            surfaceCreatedNative();
        }

        @Override
        public void onSurfaceChanged(GL10 gl10, int width, int height) {
            arCore.onSurfaceChanged(
                    getWindowManager().getDefaultDisplay().getRotation(), width, height);
            applySurfaceFrameRate();
            setScreenResolutionNative(width, height);
        }

        @Override
        public void onDrawFrame(GL10 gl10) {
            // ARCore first (camera texture + tracking): its pose only feeds the ~30 Hz position
            // filter, so with pacing it runs before the wait, off the latency-critical path.
            long arStart = SystemClock.elapsedRealtimeNanos();
            arCore.onDrawFrame();
            long arNs = SystemClock.elapsedRealtimeNanos() - arStart;
            if (pacing) waitForRenderSlot();
            renderNative(arNs);
        }
    }

    // ---- Problem banner ----

    private View problemOverlay;
    private TextView problemTextLeft;
    private TextView problemTextRight;
    private static final long PROBLEM_POLL_MS = 500;

    private final Runnable problemPoll =
            new Runnable() {
                @Override
                public void run() {
                    updateProblemBanner();
                    uiHandler.postDelayed(this, PROBLEM_POLL_MS);
                }
            };

    /** UI thread: shows why 6DoF is paused or the stream is behind; hides when all is well. */
    private void updateProblemBanner() {
        SharedPreferences p = settings();
        String text = null;
        if (p.getBoolean(Settings.PROBLEM_BANNER, true)) {
            int[] s = getProblemStateNative(); // guard, late ms, position hold state
            boolean sixDof = p.getBoolean(Settings.ARCORE_6DOF, true);
            if (s[0] == 1) {
                text = getString(R.string.banner_stream_late, s[1] / 1000f);
            } else if (sixDof && s[2] == 1) {
                text = getString(arCoreLostMessage(arCore.getFailureReason()));
            } else if (sixDof && s[2] == 2) {
                text = getString(R.string.banner_settling);
            } else if (sixDof && s[2] == 3) {
                text = getString(R.string.banner_hand_close);
            }
        }
        if (text == null) {
            problemOverlay.setVisibility(View.GONE);
            return;
        }
        problemTextLeft.setText(text);
        problemTextRight.setText(text);
        problemOverlay.setVisibility(View.VISIBLE);
    }

    private static int arCoreLostMessage(String reason) {
        if (reason == null) return R.string.banner_lost;
        switch (reason) {
            case "INSUFFICIENT_LIGHT":
                return R.string.banner_too_dark;
            case "INSUFFICIENT_FEATURES":
                return R.string.banner_no_features;
            case "EXCESSIVE_MOTION":
                return R.string.banner_fast_motion;
            case "CAMERA_UNAVAILABLE":
                return R.string.banner_camera_blocked;
            default:
                return R.string.banner_lost;
        }
    }

    /** GL thread: sleeps until pacingBudgetNs before the next vsync. */
    private void waitForRenderSlot() {
        long last = lastVsyncNs;
        long period = vsyncPeriodNs;
        if (last == 0 || period <= 0) return;
        long now = System.nanoTime();
        long next = last + period;
        while (next - pacingBudgetNs < now) next += period;
        long sleepNs = next - pacingBudgetNs - now;
        if (sleepNs > 500_000L && sleepNs < period) {
            try {
                Thread.sleep(sleepNs / 1_000_000L, (int) (sleepNs % 1_000_000L));
            } catch (InterruptedException ignored) {
            }
        }
    }

    /** UI thread: switches the render loop between continuous and vsync-paced. */
    private void applyPacing() {
        SharedPreferences p = settings();
        pacing = Settings.streamBool(p, Settings.FRAME_PACING);
        pacingBudgetNs = (long) (Settings.tuneValue(p, Settings.PACING_BUDGET_MS) * 1_000_000L);
        glView.setRenderMode(
                pacing
                        ? GLSurfaceView.RENDERMODE_WHEN_DIRTY
                        : GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        if (pacing && !vsyncCallbackPosted) {
            vsyncCallbackPosted = true;
            Choreographer.getInstance().postFrameCallback(vsyncCallback);
        }
        Log.i(
                TAG,
                "Frame pacing " + (pacing ? "on, budget " + pacingBudgetNs / 1e6 + " ms" : "off"));
    }

    private void stopPacing() {
        pacing = false;
        if (vsyncCallbackPosted) {
            Choreographer.getInstance().removeFrameCallback(vsyncCallback);
            vsyncCallbackPosted = false;
        }
    }

    /**
     * Testing hook (adb): {@code am start -n <pkg>/.ALVRActivity --es stream_profile <id>} picks a
     * profile; {@code --ez <toggle key> true|false} and {@code --ef <tune key> <value>} set single
     * options (keys as in Settings.kt). Saved like a change in the ⚙ menu.
     */
    /**
     * Testing hook: change stream options while streaming, e.g. {@code adb shell am broadcast -a
     * io.github.lennoxx360.phonepcvr.STREAM_OPTIONS --ef overscan_deg 7}. Same extras as
     * applyStreamExtras. Only senders holding android.permission.DUMP (adb shell) can use it.
     */
    public static final String ACTION_STREAM_OPTIONS =
            "io.github.lennoxx360.phonepcvr.STREAM_OPTIONS";

    private final BroadcastReceiver liveSettingsReceiver =
            new BroadcastReceiver() {
                @Override
                public void onReceive(Context context, Intent intent) {
                    applyStreamExtras(intent);
                    onStreamSettingsChanged();
                    Log.i(TAG, "Stream options changed live: " + intent.getExtras());
                }
            };

    private void registerLiveSettingsReceiver() {
        IntentFilter f = new IntentFilter(ACTION_STREAM_OPTIONS);
        if (VERSION.SDK_INT >= 33) {
            registerReceiver(
                    liveSettingsReceiver,
                    f,
                    Manifest.permission.DUMP,
                    uiHandler,
                    Context.RECEIVER_EXPORTED);
        } else {
            registerReceiver(liveSettingsReceiver, f, Manifest.permission.DUMP, uiHandler);
        }
    }

    private void applyStreamExtras(Intent intent) {
        if (intent == null || intent.getExtras() == null) return;
        SharedPreferences p = settings();
        String profile = intent.getStringExtra(Settings.STREAM_PROFILE);
        if (profile != null) Settings.applyProfile(p, profile);
        SharedPreferences.Editor e = p.edit();
        boolean custom = false;
        for (Settings.Toggle t : Settings.STREAM) {
            if (intent.hasExtra(t.getKey())) {
                e.putBoolean(t.getKey(), intent.getBooleanExtra(t.getKey(), t.getDefault()));
                custom = true;
            }
        }
        for (Settings.Tune t : Settings.TUNE) {
            if (intent.hasExtra(t.getKey())) {
                e.putFloat(t.getKey(), intent.getFloatExtra(t.getKey(), t.getDefault()));
                custom = true;
            }
        }
        for (Settings.Toggle t : Settings.EXPERIMENTAL) {
            if (intent.hasExtra(t.getKey()))
                e.putBoolean(t.getKey(), intent.getBooleanExtra(t.getKey(), t.getDefault()));
        }
        for (Settings.Tune t : Settings.EXPERIMENTAL_TUNE) {
            if (intent.hasExtra(t.getKey()))
                e.putFloat(t.getKey(), intent.getFloatExtra(t.getKey(), t.getDefault()));
        }
        if (custom) e.putString(Settings.STREAM_PROFILE, Settings.PROFILE_CUSTOM);
        e.commit();
    }

    /** Sends the latency / picture options to native and the ALVR fork. Any thread. */
    private void pushStreamOptions() {
        SharedPreferences p = settings();
        boolean latest = Settings.streamBool(p, Settings.LATEST_FRAME);
        boolean hide = Settings.streamBool(p, Settings.HIDE_CORRUPTED);
        boolean limit = Settings.streamBool(p, Settings.LIMIT_PREDICTION);
        setStreamOptionsNative(
                latest,
                Settings.tuneValue(p, Settings.BUFFER_FRAMES),
                Math.round(Settings.tuneValue(p, Settings.DECODER_WAIT_MS)),
                hide ? 1 : -1,
                limit ? Math.round(Settings.tuneValue(p, Settings.PREDICTION_CAP_MS)) : 0,
                Math.round(Settings.tuneValue(p, Settings.POSITION_PREDICTION_MS)),
                Settings.tuneValue(p, Settings.OVERSCAN_DEG),
                Settings.streamBool(p, Settings.TW_STRETCH_EDGES),
                (Settings.streamBool(p, Settings.TW_AIM_MEASURED)
                                || Settings.streamBool(p, Settings.FRAME_PACING))
                        ? 1
                        : 0,
                Settings.tuneValue(p, Settings.TW_PREDICTION_PCT) / 100f,
                Settings.streamBool(p, Settings.TW_DAMPED));
        setPositionalTimewarpNative(
                Settings.experimentalBool(p, Settings.TW_POSITIONAL),
                Settings.tuneValue(p, Settings.TW_PLANE_M));
        setAirJoystickNative(
                Settings.experimentalBool(p, Settings.HANDS_AIR_JOYSTICK),
                Settings.tuneValue(p, Settings.JOY_DEADZONE_CM),
                Settings.tuneValue(p, Settings.JOY_FULL_CM));
        if (handTracker != null)
            handTracker.setPcAssist(
                    Settings.experimentalBool(p, Settings.HANDS_PC_ASSIST),
                    Math.round(Settings.tuneValue(p, Settings.PC_ASSIST_QUALITY)),
                    Settings.experimentalBool(p, Settings.PC_ASSIST_H264),
                    Settings.tuneValue(p, Settings.PC_ASSIST_MBPS));
        int handsWhich = Math.round(Settings.tuneValue(p, Settings.HANDS_WHICH));
        setHandModeNative(handsWhich);
        if (handTracker != null) handTracker.setNumHands(handsWhich == 0 ? 2 : 1);
        setPhoneGesturesNative(
                Settings.experimentalBool(p, Settings.HANDS_PHONE_GESTURES),
                Settings.tuneValue(p, Settings.PINCH_PRESS_MM),
                Settings.tuneValue(p, Settings.PINCH_RELEASE_MM),
                Settings.tuneValue(p, Settings.GRIP_PRESS_MM),
                Settings.tuneValue(p, Settings.GRIP_RELEASE_MM));
    }

    private static final int MENU_GROUP_PROFILES = 101;
    private static final int MENU_GROUP_STREAM = 102;
    private static final int MENU_GROUP_EXPERIMENTAL = 103;

    /** ⚙ → Latency & picture: profiles (radio) and every on/off option. */
    private void addLatencyMenu(android.view.Menu menu) {
        SubMenu sub = menu.addSubMenu(android.view.Menu.NONE, View.NO_ID, 1, R.string.latency_menu);
        SharedPreferences p = settings();
        String current = Settings.currentProfile(p);
        for (int i = 0; i < Settings.PROFILE_IDS.size(); i++) {
            String id = Settings.PROFILE_IDS.get(i);
            MenuItem item = sub.add(MENU_GROUP_PROFILES, 1000 + i, i, Settings.profileTitle(id));
            item.setChecked(id.equals(current));
        }
        sub.setGroupCheckable(MENU_GROUP_PROFILES, true, true);
        for (int i = 0; i < Settings.STREAM.size(); i++) {
            Settings.Toggle t = Settings.STREAM.get(i);
            MenuItem item = sub.add(MENU_GROUP_STREAM, 2000 + i, 100 + i, t.getTitle());
            item.setCheckable(true);
            item.setChecked(p.getBoolean(t.getKey(), t.getDefault()));
        }
        for (int i = 0; i < Settings.EXPERIMENTAL.size(); i++) {
            Settings.Toggle t = Settings.EXPERIMENTAL.get(i);
            MenuItem item = sub.add(MENU_GROUP_EXPERIMENTAL, 3000 + i, 200 + i, t.getTitle());
            item.setCheckable(true);
            item.setChecked(p.getBoolean(t.getKey(), t.getDefault()));
        }
    }

    /** Returns true if it was a Latency & picture item. */
    private boolean onLatencyMenuItem(MenuItem item) {
        if (item.getGroupId() == MENU_GROUP_PROFILES) {
            String id = Settings.PROFILE_IDS.get(item.getItemId() - 1000);
            Settings.applyProfile(settings(), id);
            onStreamSettingsChanged();
            showMessage(getString(Settings.profileTitle(id)));
            return true;
        }
        if (item.getGroupId() == MENU_GROUP_STREAM) {
            Settings.Toggle t = Settings.STREAM.get(item.getItemId() - 2000);
            boolean on = !item.isChecked();
            item.setChecked(on);
            settings()
                    .edit()
                    .putBoolean(t.getKey(), on)
                    .putString(Settings.STREAM_PROFILE, Settings.PROFILE_CUSTOM)
                    .apply();
            onStreamSettingsChanged();
            showMessage(getString(t.getTitle()) + (on ? ": on" : ": off"));
            return true;
        }
        if (item.getGroupId() == MENU_GROUP_EXPERIMENTAL) {
            Settings.Toggle t = Settings.EXPERIMENTAL.get(item.getItemId() - 3000);
            boolean on = !item.isChecked();
            item.setChecked(on);
            settings().edit().putBoolean(t.getKey(), on).apply();
            onStreamSettingsChanged();
            showMessage(getString(t.getTitle()) + (on ? ": on" : ": off"));
            return true;
        }
        return false;
    }

    private void onStreamSettingsChanged() {
        pushStreamOptions();
        applyPacing();
    }

    // Called from activity_vr.xml
    public void closeSample(View view) {
        Log.d(TAG, "Leaving VR");
        finish();
    }

    // Called from activity_vr.xml
    public void showSettings(View view) {
        PopupMenu popup = new PopupMenu(this, view);
        MenuInflater inflater = popup.getMenuInflater();
        inflater.inflate(R.menu.settings_menu, popup.getMenu());

        MenuItem toggleBrightness = popup.getMenu().findItem(R.id.max_brightness_toggle);
        // get setting max_brightness boolean from settings
        SharedPreferences prefs = getSharedPreferences("settings", MODE_PRIVATE);
        boolean isChecked = prefs.getBoolean("max_brightness", true);
        toggleBrightness.setChecked(isChecked);
        popup.getMenu()
                .findItem(R.id.arcore_overlay_toggle)
                .setChecked(prefs.getBoolean("arcore_overlay", false));
        popup.getMenu()
                .findItem(R.id.timewarp_toggle)
                .setChecked(prefs.getBoolean("timewarp", true));
        popup.getMenu()
                .findItem(R.id.arcore_6dof_toggle)
                .setChecked(prefs.getBoolean("arcore_6dof", true));
        popup.getMenu()
                .findItem(R.id.gaze_pointer_toggle)
                .setChecked(prefs.getBoolean(Settings.GAZE_POINTER, false));
        popup.getMenu()
                .findItem(R.id.hand_tracking_toggle)
                .setChecked(prefs.getBoolean("hand_tracking", false));
        popup.getMenu()
                .findItem(R.id.hands_which_cycle)
                .setTitle(getString(R.string.hands_which_menu, handsWhichLabel()));

        addLatencyMenu(popup.getMenu());
        popup.setOnMenuItemClickListener(this);
        popup.show();
    }

    @Override
    public boolean onMenuItemClick(MenuItem item) {
        if (onLatencyMenuItem(item)) {
            return true;
        } else if (item.getItemId() == R.id.switch_viewer) {
            switchViewerNative();
            return true;
        } else if (item.getItemId() == R.id.gaze_pointer_toggle) {
            item.setChecked(!item.isChecked());
            settings().edit().putBoolean(Settings.GAZE_POINTER, item.isChecked()).apply();
            pushFit(true);
            updateTriggerMode();
            return true;
        } else if (item.getItemId() == R.id.steamvr_menu) {
            toggleSteamVrMenu();
            return true;
        } else if (item.getItemId() == R.id.reset_play_space) {
            resetPlaySpace();
            return true;
        } else if (item.getItemId() == R.id.adjust_fit) {
            enterFitMode();
            return true;
        } else if (item.getItemId() == R.id.back_to_home) {
            finish();
            return true;
        } else if (item.getItemId() == R.id.max_brightness_toggle) {
            // Save app setting boolean max_brightness == true
            item.setChecked(!item.isChecked());
            SharedPreferences.Editor editor = getSharedPreferences("settings", MODE_PRIVATE).edit();
            editor.putBoolean("max_brightness", item.isChecked());
            editor.apply();
            return true;
        } else if (item.getItemId() == R.id.arcore_overlay_toggle) {
            item.setChecked(!item.isChecked());
            getSharedPreferences("settings", MODE_PRIVATE)
                    .edit()
                    .putBoolean("arcore_overlay", item.isChecked())
                    .apply();
            updateArOverlayVisibility();
            return true;
        } else if (item.getItemId() == R.id.timewarp_toggle) {
            item.setChecked(!item.isChecked());
            getSharedPreferences("settings", MODE_PRIVATE)
                    .edit()
                    .putBoolean("timewarp", item.isChecked())
                    .apply();
            setTimewarpEnabledNative(item.isChecked());
            return true;
        } else if (item.getItemId() == R.id.arcore_6dof_toggle) {
            item.setChecked(!item.isChecked());
            final boolean on = item.isChecked();
            getSharedPreferences("settings", MODE_PRIVATE)
                    .edit()
                    .putBoolean("arcore_6dof", on)
                    .apply();
            glView.queueEvent(() -> arCore.setEnabled(on));
            return true;
        } else if (item.getItemId() == R.id.hand_tracking_toggle) {
            item.setChecked(!item.isChecked());
            getSharedPreferences("settings", MODE_PRIVATE)
                    .edit()
                    .putBoolean("hand_tracking", item.isChecked())
                    .apply();
            handTracker.setEnabled(item.isChecked());
            // Hands on: the app takes over the camera so hands get their own HD stream; hands
            // off: ARCore opens the camera itself again (the head-tracking setup is unchanged).
            final boolean shared =
                    item.isChecked()
                            && Settings.experimentalBool(settings(), Settings.HANDS_HD_CAMERA);
            glView.queueEvent(() -> arCore.setCameraMode(shared));
            return true;
        } else if (item.getItemId() == R.id.hands_which_cycle) {
            SharedPreferences p = settings();
            int next = (Math.round(Settings.tuneValue(p, Settings.HANDS_WHICH)) + 1) % 3;
            p.edit().putFloat(Settings.HANDS_WHICH, next).apply();
            pushStreamOptions();
            showMessage(getString(R.string.hands_which_menu, handsWhichLabel()));
            return true;
        }
        return false;
    }

    private String handsWhichLabel() {
        for (Settings.Tune t : Settings.EXPERIMENTAL_TUNE) {
            if (t.getKey().equals(Settings.HANDS_WHICH)) return t.format(this, settings());
        }
        return "";
    }

    // ---- Headset fit ----

    private SharedPreferences settings() {
        return getSharedPreferences(Settings.PREFS, MODE_PRIVATE);
    }

    /** Short side of the physical screen in metres (for moving CENTER-aligned profiles). */
    private float screenHeightMeters() {
        DisplayMetrics m = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getRealMetrics(m);
        boolean landscape = m.widthPixels > m.heightPixels;
        float px = landscape ? m.heightPixels : m.widthPixels;
        float dpi = landscape ? m.ydpi : m.xdpi;
        return dpi > 0 ? px / dpi * 0.0254f : 0f;
    }

    /** Sends the saved fit to the native lens setup (on the GL thread once it's running). */
    private void pushFit(boolean onGlThread) {
        SharedPreferences p = settings();
        final float ipd = Settings.fitValue(p, Settings.FitKind.IPD);
        final float size = Settings.fitValue(p, Settings.FitKind.SIZE);
        final float fov = Settings.fitValue(p, Settings.FitKind.FOV);
        final float vertical = Settings.fitValue(p, Settings.FitKind.VERTICAL);
        final float stereo = Settings.fitValue(p, Settings.FitKind.STEREO);
        final float screenH = screenHeightMeters();
        setGazePointerNative(
                p.getBoolean(Settings.GAZE_POINTER, false),
                Settings.fitValue(p, Settings.FitKind.GAZE_PITCH));
        if (onGlThread) {
            glView.queueEvent(() -> setViewerFitNative(ipd, size, fov, vertical, stereo, screenH));
        } else {
            setViewerFitNative(ipd, size, fov, vertical, stereo, screenH);
        }
    }

    private void saveProfileIpd() {
        float[] info = getViewerInfoNative();
        if (info != null && info.length > 0 && info[0] > 0) {
            settings().edit().putFloat(Settings.PROFILE_IPD_MM, info[0]).apply();
        }
    }

    private void enterFitMode() {
        saveProfileIpd();
        fitIndex = 0;
        // Gaze pointer on: start at "Gaze laser angle", the setting gaze users need (hold Vol- goes
        // back to the lens settings).
        if (gazeOn()) {
            for (int i = 0; i < Settings.FIT.size(); i++) {
                if (Settings.FIT.get(i).getKind() == Settings.FitKind.GAZE_PITCH) fitIndex = i;
            }
        }
        showFitOverlay();
        updateTriggerMode();
    }

    private void nextFitItem() {
        if (fitIndex < 0) {
            enterFitMode();
            return;
        }
        fitIndex++;
        if (fitIndex >= Settings.FIT.size()) {
            exitFitMode(true);
        } else {
            showFitOverlay();
        }
    }

    private void stepFit(int direction) {
        if (fitIndex < 0) return;
        Settings.FIT.get(fitIndex).step(settings(), direction);
        pushFit(true);
        showFitOverlay();
    }

    private void showFitOverlay() {
        Settings.Fit fit = Settings.FIT.get(fitIndex);
        boolean last = fitIndex == Settings.FIT.size() - 1;
        String text =
                getString(
                        R.string.fit_overlay,
                        fitIndex + 1,
                        Settings.FIT.size(),
                        getString(fit.getTitle()),
                        fit.format(this, settings()),
                        getString(last ? R.string.fit_overlay_done : R.string.fit_overlay_next));
        fitTextLeft.setText(text);
        fitTextRight.setText(text);
        fitOverlay.setVisibility(View.VISIBLE);
        uiHandler.removeCallbacks(fitIdleExit);
        uiHandler.postDelayed(fitIdleExit, FIT_IDLE_EXIT_MS);
    }

    private void exitFitMode(boolean showSaved) {
        uiHandler.removeCallbacks(fitIdleExit);
        if (fitIndex < 0) return;
        fitIndex = -1;
        updateTriggerMode();
        if (showSaved) {
            showMessage(getString(R.string.fit_saved));
        } else {
            fitOverlay.setVisibility(View.GONE);
        }
    }

    /** A short message in both eyes (uses the fit overlay when it isn't in use). */
    private void showMessage(String text) {
        showMessage(text, 1500);
    }

    private void showMessage(String text, long durationMs) {
        if (fitIndex >= 0) return;
        fitTextLeft.setText(text);
        fitTextRight.setText(text);
        fitOverlay.setVisibility(View.VISIBLE);
        uiHandler.removeCallbacks(hideMessage);
        uiHandler.postDelayed(hideMessage, durationMs);
    }

    private final Runnable hideMessage =
            () -> {
                if (fitIndex < 0) fitOverlay.setVisibility(View.GONE);
            };

    // ---- Play space ----

    private void resetPlaySpace() {
        recenterNative();
        buzz();
        showMessage(getString(R.string.play_space_reset_done));
    }

    // Queued from the home screen, or "reset on every start": wait until the headset is on and
    // ARCore has settled (settle is 3 s), then reset.
    private static final long AUTO_RECENTER_DELAY_MS = 6000;
    private final Runnable autoRecenter =
            () -> {
                settings().edit().putBoolean(Settings.RECENTER_PENDING, false).apply();
                resetPlaySpace();
            };

    // ---- SteamVR menu ----

    /** Taps the left controller menu button: opens/closes the SteamVR dashboard. */
    private void toggleSteamVrMenu() {
        sendSteamVrMenuNative(true);
        uiHandler.postDelayed(() -> sendSteamVrMenuNative(false), 150);
        buzz();
        if (gazeOn()) {
            // SteamVR gives the dashboard laser to a controller once it clicks. Click once with the
            // gaze controller after the menu has opened so the laser appears without a manual
            // click.
            uiHandler.postDelayed(() -> sendTriggerNative(true), 600);
            uiHandler.postDelayed(() -> sendTriggerNative(false), 700);
            showMessage(getString(R.string.gaze_controls), 3000);
        } else {
            showMessage(getString(R.string.steamvr_menu_sent));
        }
    }

    private void buzz() {
        glView.performHapticFeedback(
                HapticFeedbackConstants.LONG_PRESS,
                HapticFeedbackConstants.FLAG_IGNORE_GLOBAL_SETTING);
    }

    private static boolean isVolumeKey(int keyCode) {
        return keyCode == KeyEvent.KEYCODE_VOLUME_UP || keyCode == KeyEvent.KEYCODE_VOLUME_DOWN;
    }

    private static boolean isSelectKey(int keyCode) {
        return keyCode == KeyEvent.KEYCODE_ENTER
                || keyCode == KeyEvent.KEYCODE_DPAD_CENTER
                || keyCode == KeyEvent.KEYCODE_BUTTON_A;
    }

    // Volume keys, one gesture per action (VolumeGestures decides; nothing overlaps):
    //                Normal          Gaze pointer on      Headset fit
    //   Tap Vol+     volume up       SteamVR menu         value +
    //   Tap Vol-     volume down     trigger (click)      value -
    //   Hold Vol+    headset fit     headset fit          next setting
    //   Hold Vol-    SteamVR menu    trigger (hold/drag)  previous setting
    //   Tap both     reset play space                     save & exit
    //   Hold both    gaze pointer on/off                  save & exit
    // Phones don't give apps the power key. Gamepad/keyboard: Enter/A = next, d-pad = value.
    private boolean gazeOn() {
        return settings().getBoolean(Settings.GAZE_POINTER, false);
    }

    private final VolumeGestures gestures = new VolumeGestures(this::onGesture);

    /** Vol- is the trigger only while the gaze pointer is on and the fit isn't being adjusted. */
    private void updateTriggerMode() {
        gestures.setTriggerMode(gazeOn() && fitIndex < 0);
    }

    private void toggleGazePointer() {
        boolean on = !gazeOn();
        settings().edit().putBoolean(Settings.GAZE_POINTER, on).apply();
        pushFit(true);
        updateTriggerMode();
        buzz();
        showMessage(getString(on ? R.string.gaze_on_msg : R.string.gaze_off_msg), on ? 4000 : 1500);
    }

    private void prevFitItem() {
        if (fitIndex > 0) {
            fitIndex--;
            showFitOverlay();
        }
    }

    private void onGesture(VolumeGestures.Gesture g) {
        if (fitIndex >= 0) {
            switch (g) {
                case TAP_UP:
                    stepFit(+1);
                    break;
                case TAP_DOWN:
                    stepFit(-1);
                    break;
                case HOLD_UP:
                    buzz();
                    nextFitItem();
                    break;
                case HOLD_DOWN:
                    buzz();
                    prevFitItem();
                    break;
                case TAP_BOTH:
                case HOLD_BOTH:
                    exitFitMode(true);
                    break;
                default:
                    break;
            }
            updateTriggerMode();
            return;
        }
        boolean gaze = gazeOn();
        switch (g) {
            case TAP_UP:
                if (gaze) toggleSteamVrMenu();
                else adjustVolume(+1);
                break;
            case TAP_DOWN:
                adjustVolume(-1); // not reached in gaze mode: Vol- is the trigger there
                break;
            case HOLD_UP:
                buzz();
                enterFitMode();
                break;
            case HOLD_DOWN:
                toggleSteamVrMenu();
                break;
            case TAP_BOTH:
                resetPlaySpace();
                break;
            case HOLD_BOTH:
                toggleGazePointer();
                break;
            case TRIGGER_PRESS:
                sendTriggerNative(true);
                break;
            case TRIGGER_RELEASE:
                sendTriggerNative(false);
                break;
        }
        updateTriggerMode();
    }

    private void adjustVolume(int direction) {
        AudioManager audio = (AudioManager) getSystemService(AUDIO_SERVICE);
        audio.adjustStreamVolume(
                AudioManager.STREAM_MUSIC,
                direction > 0 ? AudioManager.ADJUST_RAISE : AudioManager.ADJUST_LOWER,
                0);
    }

    private boolean volumeKeysEnabled() {
        return settings().getBoolean(Settings.FIT_VOLUME_KEYS, true);
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (isVolumeKey(keyCode) && volumeKeysEnabled()) {
            if (event.getRepeatCount() == 0) {
                updateTriggerMode();
                long t = event.getEventTime();
                gestures.onDown(
                        keyCode == KeyEvent.KEYCODE_VOLUME_UP
                                ? VolumeGestures.UP
                                : VolumeGestures.DOWN,
                        t);
                uiHandler.postAtTime(
                        () -> gestures.onTimer(SystemClock.uptimeMillis()),
                        t + VolumeGestures.CHORD_MS);
                uiHandler.postAtTime(
                        () -> gestures.onTimer(SystemClock.uptimeMillis()),
                        t + VolumeGestures.LONG_MS);
            }
            return true;
        }
        if (isSelectKey(keyCode) && event.getRepeatCount() == 0) {
            nextFitItem();
            updateTriggerMode();
            return true;
        }
        if (fitIndex >= 0
                && (keyCode == KeyEvent.KEYCODE_DPAD_RIGHT
                        || keyCode == KeyEvent.KEYCODE_DPAD_UP)) {
            stepFit(+1);
            return true;
        }
        if (fitIndex >= 0
                && (keyCode == KeyEvent.KEYCODE_DPAD_LEFT
                        || keyCode == KeyEvent.KEYCODE_DPAD_DOWN)) {
            stepFit(-1);
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }

    @Override
    public boolean onKeyUp(int keyCode, KeyEvent event) {
        if (isVolumeKey(keyCode) && volumeKeysEnabled()) {
            gestures.onUp(
                    keyCode == KeyEvent.KEYCODE_VOLUME_UP ? VolumeGestures.UP : VolumeGestures.DOWN,
                    event.getEventTime());
            return true;
        }
        return super.onKeyUp(keyCode, event);
    }

    // Ask for the fastest display mode at the current resolution (120 Hz on an S24). Phones keep
    // ordinary apps at 60 Hz otherwise. Both the window mode and the surface frame rate are set,
    // since vendors honour one or the other.
    private void requestHighRefreshRate() {
        if (VERSION.SDK_INT < VERSION_CODES.M) return;
        Display display = getWindowManager().getDefaultDisplay();
        Display.Mode current = display.getMode();
        Display.Mode best = current;
        for (Display.Mode mode : display.getSupportedModes()) {
            if (mode.getPhysicalWidth() == current.getPhysicalWidth()
                    && mode.getPhysicalHeight() == current.getPhysicalHeight()
                    && mode.getRefreshRate() > best.getRefreshRate()) {
                best = mode;
            }
        }
        WindowManager.LayoutParams lp = getWindow().getAttributes();
        lp.preferredDisplayModeId = best.getModeId();
        lp.preferredRefreshRate = best.getRefreshRate();
        getWindow().setAttributes(lp);
        Log.i(
                TAG,
                "Requesting display mode "
                        + best.getModeId()
                        + " at "
                        + best.getRefreshRate()
                        + " Hz (was "
                        + current.getRefreshRate()
                        + " Hz)");

        targetRefreshRate = best.getRefreshRate();

        // Android 15+ (adaptive refresh): each view votes, and a plain view votes "normal" (60 Hz
        // on this phone). View.setRequestedFrameRate is API 35; compileSdk is 34, so reflection.
        if (VERSION.SDK_INT >= 35) {
            try {
                View.class
                        .getMethod("setRequestedFrameRate", float.class)
                        .invoke(glView, targetRefreshRate);
                Log.i(TAG, "View.setRequestedFrameRate(" + targetRefreshRate + ") ok");
            } catch (Exception e) {
                Log.w(TAG, "View.setRequestedFrameRate failed", e);
            }
        }
    }

    // Render thread, once the GL surface exists: vote for the high rate on the surface itself.
    private void applySurfaceFrameRate() {
        if (VERSION.SDK_INT < VERSION_CODES.R || targetRefreshRate <= 0) return;
        Surface surface = glView.getHolder().getSurface();
        if (surface == null || !surface.isValid()) return;
        try {
            if (VERSION.SDK_INT >= VERSION_CODES.S) {
                surface.setFrameRate(
                        targetRefreshRate,
                        Surface.FRAME_RATE_COMPATIBILITY_FIXED_SOURCE,
                        Surface.CHANGE_FRAME_RATE_ALWAYS);
            } else {
                surface.setFrameRate(targetRefreshRate, Surface.FRAME_RATE_COMPATIBILITY_DEFAULT);
            }
            Log.i(TAG, "Surface.setFrameRate(" + targetRefreshRate + ") ok");
        } catch (Exception e) {
            Log.w(TAG, "Surface.setFrameRate failed", e);
        }
    }

    private boolean isCameraEnabled() {
        return ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
                == PackageManager.PERMISSION_GRANTED;
    }

    private void updateArOverlayVisibility() {
        boolean show =
                getSharedPreferences("settings", MODE_PRIVATE).getBoolean("arcore_overlay", false);
        arOverlay.setVisibility(show ? View.VISIBLE : View.GONE);
    }

    // Render thread: forward every ARCore camera pose to the native pose fusion.
    private void onArCorePose(
            long timestampNs, boolean tracking, float[] translation, float[] rotation) {
        setArCorePoseNative(
                timestampNs,
                tracking,
                translation[0],
                translation[1],
                translation[2],
                rotation[0],
                rotation[1],
                rotation[2],
                rotation[3]);
    }

    // Render thread -> UI thread.
    private int statusCount = 0;

    private void showArCoreStatus(String arText) {
        String fusionText = getFusionStatusNative();
        String text = arText + "\n" + fusionText;
        if (statusCount++ % 5 == 0) Log.i("PVR-Fusion", fusionText.replace("\n", " | "));
        runOnUiThread(
                () -> {
                    if (arOverlay.getVisibility() != View.VISIBLE) return;
                    arTextLeft.setText(text);
                    arTextRight.setText(text);
                });
    }

    private boolean isReadExternalStorageEnabled() {
        return ActivityCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
                == PackageManager.PERMISSION_GRANTED;
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE) {
            // Granted: onResume() runs again after the dialog and starts ARCore.
            if (!isCameraEnabled()) {
                Toast.makeText(this, R.string.camera_permission_needed, Toast.LENGTH_LONG).show();
            }
            return;
        }
        if (!isReadExternalStorageEnabled()) {
            Toast.makeText(this, R.string.read_storage_permission, Toast.LENGTH_LONG).show();
            if (!ActivityCompat.shouldShowRequestPermissionRationale(
                    this, Manifest.permission.READ_EXTERNAL_STORAGE)) {
                // Permission denied with checking "Do not ask again". Note that in Android R
                // "Do not ask again" is not available anymore.
                Intent intent = new Intent();
                intent.setAction(android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                intent.setData(Uri.fromParts("package", getPackageName(), null));
                startActivity(intent);
            }
            finish();
        }
    }

    private void setImmersiveSticky() {
        getWindow()
                .getDecorView()
                .setSystemUiVisibility(
                        View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                                | View.SYSTEM_UI_FLAG_FULLSCREEN
                                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    private native void initializeNative(
            int screenWidth, int screenHeight, float screenRefreshRate);

    private native void destroyNative();

    private native String getHostnameNative();

    private native void setViewerFitNative(
            float ipdDeltaMm,
            float sizePercent,
            float fovPercent,
            float verticalMm,
            float stereoDeltaMm,
            float screenHeightM);

    private native float[] getViewerInfoNative();

    private native void recenterNative();

    private native void sendSteamVrMenuNative(boolean pressed);

    private native void setGazePointerNative(boolean on, float pitchDeg);

    private native void sendTriggerNative(boolean pressed);

    private native void resumeNative();

    private native void pauseNative();

    private native void surfaceCreatedNative();

    private native void setScreenResolutionNative(int width, int height);

    private native void renderNative(long arCoreNs);

    private native void setComfortGuardNative(boolean on);

    private native void setPositionalTimewarpNative(boolean on, float planeDistM);

    private native void setAirJoystickNative(boolean on, float deadzoneCm, float fullCm);

    /** 0 both hands, 1 left only, 2 right only. */
    private native void setHandModeNative(int mode);

    private native void setPhoneGesturesNative(
            boolean on,
            float pinchPressMm,
            float pinchReleaseMm,
            float gripPressMm,
            float gripReleaseMm);

    private native int[] getProblemStateNative();

    private native void setStreamOptionsNative(
            boolean latestFrameOnly,
            float maxBufferingFrames,
            int decoderWaitMs,
            int hideCorrupted,
            int lookaheadCapMs,
            int positionPredictionMs,
            float fovMarginDeg,
            boolean stretchEdges,
            int aimMode,
            float predictionScale,
            boolean damped);

    private native void switchViewerNative();

    private native void sendBatteryLevel(float level, boolean plugged);

    private native void setArCorePoseNative(
            long timestampNs,
            boolean tracking,
            float tx,
            float ty,
            float tz,
            float qx,
            float qy,
            float qz,
            float qw);

    private native String getFusionStatusNative();

    private native void setTimewarpEnabledNative(boolean enabled);

    // MediaPipe worker thread (see HandTracker.ResultSink).
    private native void setHandsNative(
            long captureTimeNs,
            int handCount,
            boolean[] labelledLeft,
            float[] image2d,
            float[] world3d,
            float[] intrinsics,
            float[] cameraPose,
            float detectMs);
}
