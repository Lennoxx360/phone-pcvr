/* (C)2026 */
package io.github.lennoxx360.phonepcvr

import android.content.Context
import android.content.SharedPreferences
import androidx.annotation.StringRes
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

/** User-facing settings. Keys are shared with ALVRActivity's in-VR ⚙ menu and volume keys. */
object Settings {
    const val PREFS = "settings"

    const val ARCORE_6DOF = "arcore_6dof"
    const val TIMEWARP = "timewarp"
    const val HAND_TRACKING = "hand_tracking"
    const val MAX_BRIGHTNESS = "max_brightness"
    const val ARCORE_OVERLAY = "arcore_overlay"
    /** Show the head-tracked lobby instead of frames that arrive badly late (seconds behind). */
    const val COMFORT_GUARD = "comfort_guard"
    /** Short in-VR message when 6DoF pauses or the stream falls behind, and why. */
    const val PROBLEM_BANNER = "problem_banner"
    /** Fake right controller that follows the head; Vol- = trigger. */
    const val GAZE_POINTER = "gaze_pointer"

    /** Network: false = ALVR's automatic broadcast discovery only. */
    const val NETWORK_MANUAL = "network_manual"
    /** PC IP for manual mode; discovery packets are also sent straight to it. */
    const val PC_IP = "pc_ip"
    /** Name ALVR shows for this phone in the dashboard; saved by ALVRActivity. */
    const val ALVR_HOSTNAME = "alvr_hostname"

    /** One-shot: reset the play space a few seconds after the stream screen next opens. */
    const val RECENTER_PENDING = "recenter_pending"
    /** Reset the play space every time the stream screen opens. */
    const val RECENTER_ON_START = "recenter_on_start"

    /** Volume keys adjust the headset fit while streaming (hold to start). */
    const val FIT_VOLUME_KEYS = "fit_volume_keys"
    /** Lens IPD of the scanned headset profile in mm; saved by ALVRActivity, for display. */
    const val PROFILE_IPD_MM = "profile_ipd_mm"

    data class Toggle(
        val key: String,
        val default: Boolean,
        @StringRes val title: Int,
        @StringRes val desc: Int
    )

    @JvmField
    val TRACKING =
        listOf(
            Toggle(ARCORE_6DOF, true, R.string.arcore_6dof, R.string.arcore_6dof_desc),
            Toggle(TIMEWARP, true, R.string.timewarp, R.string.timewarp_desc),
            Toggle(GAZE_POINTER, false, R.string.gaze_pointer, R.string.gaze_pointer_desc),
            Toggle(HAND_TRACKING, false, R.string.hand_tracking, R.string.hand_tracking_desc),
        )

    @JvmField
    val DISPLAY =
        listOf(
            Toggle(MAX_BRIGHTNESS, true, R.string.max_brightness, R.string.max_brightness_desc),
            Toggle(COMFORT_GUARD, true, R.string.comfort_guard, R.string.comfort_guard_desc),
            Toggle(PROBLEM_BANNER, true, R.string.problem_banner, R.string.problem_banner_desc),
            Toggle(ARCORE_OVERLAY, false, R.string.arcore_overlay, R.string.arcore_overlay_desc),
        )

    enum class FitKind {
        IPD,
        SIZE,
        FOV,
        VERTICAL,
        STEREO,
        GAZE_PITCH
    }

    /** One headset-fit adjustment, stored as a float in prefs. Applied on the QR profile. */
    data class Fit(
        val kind: FitKind,
        val key: String,
        @StringRes val title: Int,
        @StringRes val desc: Int,
        val default: Float,
        val step: Float,
        val min: Float,
        val max: Float
    ) {
        fun get(prefs: SharedPreferences): Float = prefs.getFloat(key, default)

        /** Steps up (+1) or down (-1), clamps, saves, and returns the new value. */
        fun step(prefs: SharedPreferences, direction: Int): Float {
            val raw = get(prefs) + direction * step
            // Snap to the step grid so repeated float adds don't drift.
            val snapped = (raw / step).roundToInt() * step
            val v = snapped.coerceIn(min, max)
            prefs.edit().putFloat(key, v).apply()
            return v
        }

        fun format(context: Context, prefs: SharedPreferences): String {
            val v = get(prefs)
            val profileIpd = prefs.getFloat(PROFILE_IPD_MM, 0f)
            return when (kind) {
                FitKind.IPD ->
                    if (profileIpd > 0)
                        context.getString(R.string.fit_ipd_value, mm(profileIpd + v), signedMm(v))
                    else signedMm(v)
                FitKind.SIZE,
                FitKind.FOV -> "${v.roundToInt()} %"
                FitKind.GAZE_PITCH ->
                    if (abs(v) < 0.5f) context.getString(R.string.gaze_pitch_centered)
                    else String.format(Locale.US, "%+.0f°", v)
                FitKind.VERTICAL ->
                    when {
                        abs(v) < 0.01f -> context.getString(R.string.fit_centered)
                        v > 0 -> context.getString(R.string.fit_up, mm(v))
                        else -> context.getString(R.string.fit_down, mm(-v))
                    }
                FitKind.STEREO ->
                    when {
                        abs(v) < 0.01f -> context.getString(R.string.fit_stereo_same)
                        profileIpd > 0 ->
                            "${mm(profileIpd + prefs.getFloat(FIT[0].key, 0f) + v)} (${signedMm(v)})"
                        else -> signedMm(v)
                    }
            }
        }

        private fun mm(v: Float) = String.format(Locale.US, "%.1f mm", v)

        private fun signedMm(v: Float) = String.format(Locale.US, "%+.1f mm", v)
    }

    @JvmField
    val FIT =
        listOf(
            Fit(
                FitKind.IPD,
                "fit_ipd_mm",
                R.string.fit_ipd,
                R.string.fit_ipd_desc,
                0f,
                0.5f,
                -15f,
                15f),
            Fit(
                FitKind.SIZE,
                "fit_size_pct",
                R.string.fit_size,
                R.string.fit_size_desc,
                100f,
                1f,
                60f,
                150f),
            Fit(
                FitKind.FOV,
                "fit_fov_pct",
                R.string.fit_fov,
                R.string.fit_fov_desc,
                100f,
                2f,
                50f,
                150f),
            Fit(
                FitKind.VERTICAL,
                "fit_vertical_mm",
                R.string.fit_vertical,
                R.string.fit_vertical_desc,
                0f,
                0.5f,
                -20f,
                20f),
            Fit(
                FitKind.STEREO,
                "fit_stereo_mm",
                R.string.fit_stereo,
                R.string.fit_stereo_desc,
                0f,
                0.5f,
                -20f,
                20f),
            Fit(
                FitKind.GAZE_PITCH,
                "gaze_pitch_deg",
                R.string.gaze_pitch,
                R.string.gaze_pitch_desc,
                0f,
                2f,
                -60f,
                60f),
        )

    // ---- Streaming: latency & artifacts ----
    // Every option defaults to the previous behaviour ("Stock"), so nothing changes until a
    // profile is picked or an option is switched. Each one is also in the in-VR ⚙ menu to A/B.

    /** Which bundle of the options below is active; "custom" once any option is changed. */
    const val STREAM_PROFILE = "stream_profile"
    const val PROFILE_STOCK = "stock"
    const val PROFILE_LOW_END_PC = "good_wifi_low_end_pc"
    const val PROFILE_BAD_WIFI = "bad_wifi_high_end_pc"
    const val PROFILE_CUSTOM = "custom"

    /** Render each frame just before the screen refresh instead of 3+ frames ahead. */
    const val FRAME_PACING = "frame_pacing"
    /** Show the newest decoded frame, dropping older queued ones (ALVR fork). */
    const val LATEST_FRAME = "latest_frame_only"
    /** Freeze on the last good frame instead of showing a smeared picture after a lost frame. */
    const val HIDE_CORRUPTED = "hide_corrupted"
    /** Cap how far ahead poses are predicted when the network spikes. */
    const val LIMIT_PREDICTION = "limit_prediction"
    /** Aim timewarp at the measured screen delay instead of the 2.5-refresh guess. */
    const val TW_AIM_MEASURED = "tw_aim_measured"
    /** Shorten the timewarp's gyro prediction while a head turn slows down. */
    const val TW_DAMPED = "tw_damped"
    /** Repeat the image's edge pixels instead of black where timewarp runs out of picture. */
    const val TW_STRETCH_EDGES = "tw_stretch_edges"

    @JvmField
    val STREAM =
        listOf(
            Toggle(FRAME_PACING, false, R.string.frame_pacing, R.string.frame_pacing_desc),
            Toggle(LATEST_FRAME, false, R.string.latest_frame, R.string.latest_frame_desc),
            Toggle(HIDE_CORRUPTED, false, R.string.hide_corrupted, R.string.hide_corrupted_desc),
            Toggle(
                LIMIT_PREDICTION, false, R.string.limit_prediction, R.string.limit_prediction_desc),
            Toggle(TW_AIM_MEASURED, false, R.string.tw_aim_measured, R.string.tw_aim_measured_desc),
            Toggle(TW_DAMPED, false, R.string.tw_damped, R.string.tw_damped_desc),
            Toggle(
                TW_STRETCH_EDGES, false, R.string.tw_stretch_edges, R.string.tw_stretch_edges_desc),
        )

    /** A numeric streaming option, stored as a float. */
    data class Tune(
        val key: String,
        @StringRes val title: Int,
        @StringRes val desc: Int,
        val default: Float,
        val step: Float,
        val min: Float,
        val max: Float,
        val unit: String,
        /** Shown instead of the number at this value (e.g. "PC setting"), or 0 for none. */
        @StringRes val specialLabel: Int = 0,
        val specialAt: Float = Float.NaN,
        /** Part of the profiles (changing it makes the profile "Custom"). */
        val profileManaged: Boolean = true,
        /** A choice instead of a number: label for value 0, 1, 2, ... (step 1, min 0). */
        val labels: List<Int>? = null
    ) {
        fun get(prefs: SharedPreferences): Float = prefs.getFloat(key, default)

        fun step(prefs: SharedPreferences, direction: Int): Float {
            val raw = get(prefs) + direction * step
            val snapped = (raw / step).roundToInt() * step
            val v = snapped.coerceIn(min, max)
            val e = prefs.edit().putFloat(key, v)
            if (profileManaged) e.putString(STREAM_PROFILE, PROFILE_CUSTOM)
            e.apply()
            return v
        }

        fun format(context: Context, prefs: SharedPreferences): String {
            val v = get(prefs)
            if (labels != null)
                return context.getString(labels[v.roundToInt().coerceIn(0, labels.size - 1)])
            if (specialLabel != 0 && abs(v - specialAt) < step / 2)
                return context.getString(specialLabel)
            val digits = if (step < 1f) 1 else 0
            return String.format(Locale.US, "%.${digits}f %s", v, unit).trim()
        }
    }

    const val BUFFER_FRAMES = "buffer_frames"
    const val DECODER_WAIT_MS = "decoder_wait_ms"
    const val PREDICTION_CAP_MS = "prediction_cap_ms"
    const val POSITION_PREDICTION_MS = "position_prediction_ms"
    const val OVERSCAN_DEG = "overscan_deg"
    const val TW_PREDICTION_PCT = "tw_prediction_pct"
    const val PACING_BUDGET_MS = "pacing_budget_ms"

    @JvmField
    val TUNE =
        listOf(
            Tune(
                BUFFER_FRAMES,
                R.string.buffer_frames,
                R.string.buffer_frames_desc,
                -1f,
                1f,
                -1f,
                3f,
                "spare",
                R.string.pc_setting,
                -1f),
            Tune(
                DECODER_WAIT_MS,
                R.string.decoder_wait,
                R.string.decoder_wait_desc,
                0f,
                1f,
                0f,
                20f,
                "ms"),
            Tune(
                PREDICTION_CAP_MS,
                R.string.prediction_cap,
                R.string.prediction_cap_desc,
                60f,
                5f,
                20f,
                150f,
                "ms"),
            Tune(
                POSITION_PREDICTION_MS,
                R.string.position_prediction,
                R.string.position_prediction_desc,
                150f,
                10f,
                0f,
                200f,
                "ms"),
            Tune(OVERSCAN_DEG, R.string.overscan, R.string.overscan_desc, 0f, 1f, 0f, 15f, "°"),
            Tune(
                TW_PREDICTION_PCT,
                R.string.tw_prediction,
                R.string.tw_prediction_desc,
                100f,
                10f,
                0f,
                120f,
                "%"),
            Tune(
                PACING_BUDGET_MS,
                R.string.pacing_budget,
                R.string.pacing_budget_desc,
                11f,
                0.5f,
                3f,
                15f,
                "ms"),
        )

    /** Option values per profile. Missing keys fall back to the option's default (= stock). */
    private val PROFILES: Map<String, Map<String, Any>> =
        mapOf(
            PROFILE_STOCK to emptyMap(),
            // Low-end PC, good Wi-Fi: frames arrive on time, so take the lowest-latency path. The
            // PC side (lower resolution and bitrate, software encoder) is in
            // tools/alvr-pc-profile.ps1.
            PROFILE_LOW_END_PC to
                mapOf(
                    FRAME_PACING to true,
                    LATEST_FRAME to true,
                    HIDE_CORRUPTED to true,
                    LIMIT_PREDICTION to true,
                    TW_AIM_MEASURED to true,
                    TW_DAMPED to true,
                    TW_STRETCH_EDGES to true,
                    BUFFER_FRAMES to 0f,
                    DECODER_WAIT_MS to 4f,
                    PREDICTION_CAP_MS to 60f,
                    POSITION_PREDICTION_MS to 100f,
                    OVERSCAN_DEG to 4f,
                    TW_PREDICTION_PCT to 100f,
                    PACING_BUDGET_MS to 11f),
            // High-end PC, bad Wi-Fi: frames arrive late and in bursts. Keep a small buffer to
            // smooth them, freeze instead of smearing on a lost frame, more overscan for the
            // bigger corrections, and a firm prediction cap so lag spikes don't throw the view.
            PROFILE_BAD_WIFI to
                mapOf(
                    FRAME_PACING to true,
                    LATEST_FRAME to false,
                    HIDE_CORRUPTED to true,
                    LIMIT_PREDICTION to true,
                    TW_AIM_MEASURED to true,
                    TW_DAMPED to true,
                    TW_STRETCH_EDGES to true,
                    BUFFER_FRAMES to 1f,
                    DECODER_WAIT_MS to 8f,
                    PREDICTION_CAP_MS to 70f,
                    POSITION_PREDICTION_MS to 80f,
                    OVERSCAN_DEG to 7f,
                    TW_PREDICTION_PCT to 100f,
                    PACING_BUDGET_MS to 11f),
        )

    @JvmField val PROFILE_IDS = listOf(PROFILE_STOCK, PROFILE_LOW_END_PC, PROFILE_BAD_WIFI)

    @JvmStatic
    fun profileTitle(id: String): Int =
        when (id) {
            PROFILE_LOW_END_PC -> R.string.profile_low_end_pc
            PROFILE_BAD_WIFI -> R.string.profile_bad_wifi
            PROFILE_CUSTOM -> R.string.profile_custom
            else -> R.string.profile_stock
        }

    @JvmStatic
    fun profileDesc(id: String): Int =
        when (id) {
            PROFILE_LOW_END_PC -> R.string.profile_low_end_pc_desc
            PROFILE_BAD_WIFI -> R.string.profile_bad_wifi_desc
            PROFILE_CUSTOM -> R.string.profile_custom_desc
            else -> R.string.profile_stock_desc
        }

    @JvmStatic
    fun currentProfile(prefs: SharedPreferences): String =
        prefs.getString(STREAM_PROFILE, PROFILE_STOCK) ?: PROFILE_STOCK

    /** Writes every streaming option for this profile. */
    @JvmStatic
    fun applyProfile(prefs: SharedPreferences, id: String) {
        val values = PROFILES[id] ?: return
        val e = prefs.edit()
        for (t in STREAM) e.putBoolean(t.key, (values[t.key] as? Boolean) ?: t.default)
        for (t in TUNE) e.putFloat(t.key, (values[t.key] as? Float) ?: t.default)
        e.putString(STREAM_PROFILE, id)
        e.apply()
    }

    @JvmStatic fun isStreamToggle(key: String) = STREAM.any { it.key == key }

    @JvmStatic
    fun streamBool(prefs: SharedPreferences, key: String): Boolean =
        prefs.getBoolean(key, STREAM.first { it.key == key }.default)

    @JvmStatic
    fun tuneValue(prefs: SharedPreferences, key: String): Float =
        (TUNE + EXPERIMENTAL_TUNE).first { it.key == key }.get(prefs)

    // ---- Experimental tracking options ----
    // One change per option, each off by default and never touched by the profiles, so they can
    // be tried one at a time in the headset (see the "tracking changes need toggles" rule).

    /** Positional timewarp: shift the picture for walking/leaning/tilting at display time. */
    const val TW_POSITIONAL = "tw_positional"
    /** Distance of the plane positional timewarp assumes the scene is on. */
    const val TW_PLANE_M = "tw_plane_m"
    /**
     * Hands get their own higher-resolution camera stream (the app opens the camera and shares it
     * with ARCore) instead of ARCore's 640x480 image. Only used while hand tracking is on; applies
     * on the next start of VR.
     */
    const val HANDS_HD_CAMERA = "hands_hd_camera"
    /**
     * Pinch = trigger and fist = grip decided on the phone (hysteresis + debounce, no clicks from a
     * hand that left the camera's view), sent as clean on/off buttons. Turn ALVR's own gesture
     * emulation off on the PC so it doesn't fire as well.
     */
    const val HANDS_PHONE_GESTURES = "hands_phone_gestures"
    /** Left pinch and drag = left thumbstick (walk); a quick left pinch is still a click. */
    const val HANDS_AIR_JOYSTICK = "hands_air_joystick"
    const val HANDS_PC_ASSIST = "hands_pc_assist"
    const val PC_ASSIST_QUALITY = "pc_assist_quality"
    const val PC_ASSIST_H264 = "pc_assist_h264"
    const val PC_ASSIST_MBPS = "pc_assist_mbps"
    const val JOY_DEADZONE_CM = "joy_deadzone_cm"
    const val JOY_FULL_CM = "joy_full_cm"
    const val HANDS_WHICH = "hands_which"
    const val PINCH_PRESS_MM = "pinch_press_mm"
    const val PINCH_RELEASE_MM = "pinch_release_mm"
    const val GRIP_PRESS_MM = "grip_press_mm"
    const val GRIP_RELEASE_MM = "grip_release_mm"

    @JvmField
    val EXPERIMENTAL =
        listOf(
            Toggle(TW_POSITIONAL, false, R.string.tw_positional, R.string.tw_positional_desc),
            Toggle(HANDS_HD_CAMERA, true, R.string.hands_hd_camera, R.string.hands_hd_camera_desc),
            Toggle(
                HANDS_PHONE_GESTURES,
                true,
                R.string.hands_phone_gestures,
                R.string.hands_phone_gestures_desc),
            Toggle(
                HANDS_AIR_JOYSTICK,
                true,
                R.string.hands_air_joystick,
                R.string.hands_air_joystick_desc),
            Toggle(HANDS_PC_ASSIST, false, R.string.hands_pc_assist, R.string.hands_pc_assist_desc),
            Toggle(PC_ASSIST_H264, true, R.string.pc_assist_h264, R.string.pc_assist_h264_desc),
        )

    @JvmField
    val EXPERIMENTAL_TUNE =
        listOf(
            Tune(
                PC_ASSIST_MBPS,
                R.string.pc_assist_mbps,
                R.string.pc_assist_mbps_desc,
                4f,
                0.5f,
                1f,
                12f,
                "Mbit/s",
                profileManaged = false),
            Tune(
                PC_ASSIST_QUALITY,
                R.string.pc_assist_quality,
                R.string.pc_assist_quality_desc,
                75f,
                5f,
                40f,
                95f,
                "",
                profileManaged = false),
            Tune(
                HANDS_WHICH,
                R.string.hands_which,
                R.string.hands_which_desc,
                0f,
                1f,
                0f,
                2f,
                "",
                profileManaged = false,
                labels =
                    listOf(
                        R.string.hands_which_both,
                        R.string.hands_which_left,
                        R.string.hands_which_right)),
            Tune(
                TW_PLANE_M,
                R.string.tw_plane,
                R.string.tw_plane_desc,
                2f,
                0.5f,
                0.5f,
                6f,
                "m",
                profileManaged = false),
            Tune(
                JOY_DEADZONE_CM,
                R.string.joy_deadzone,
                R.string.joy_deadzone_desc,
                2f,
                0.5f,
                0.5f,
                6f,
                "cm",
                profileManaged = false),
            Tune(
                JOY_FULL_CM,
                R.string.joy_full,
                R.string.joy_full_desc,
                12f,
                1f,
                4f,
                25f,
                "cm",
                profileManaged = false),
            Tune(
                PINCH_PRESS_MM,
                R.string.pinch_press,
                R.string.pinch_press_desc,
                20f,
                1f,
                8f,
                40f,
                "mm",
                profileManaged = false),
            Tune(
                PINCH_RELEASE_MM,
                R.string.pinch_release,
                R.string.pinch_release_desc,
                38f,
                1f,
                15f,
                80f,
                "mm",
                profileManaged = false),
            Tune(
                GRIP_PRESS_MM,
                R.string.grip_press,
                R.string.grip_press_desc,
                40f,
                1f,
                15f,
                80f,
                "mm",
                profileManaged = false),
            Tune(
                GRIP_RELEASE_MM,
                R.string.grip_release,
                R.string.grip_release_desc,
                55f,
                1f,
                20f,
                100f,
                "mm",
                profileManaged = false),
        )

    @JvmStatic
    fun experimentalBool(prefs: SharedPreferences, key: String): Boolean =
        prefs.getBoolean(key, EXPERIMENTAL.first { it.key == key }.default)

    @JvmStatic
    fun fitValue(prefs: SharedPreferences, kind: FitKind): Float =
        FIT.first { it.kind == kind }.get(prefs)

    @JvmStatic
    fun resetFit(prefs: SharedPreferences) {
        val e = prefs.edit()
        for (f in FIT) e.remove(f.key)
        e.apply()
    }
}
