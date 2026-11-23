#include "alvr_client_core.h"
#include "cardboard.h"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <android/bitmap.h>
#include <android/log.h>
#include <deque>
#include <jni.h>
#include <map>
#include <thread>
#include <unistd.h>
#include <vector>

#include "nlohmann/json.hpp"
#include "pose_fusion.h"
#include "viewer_params.h"
#include "timewarp.h"
#include "hand_tracking.h"
#include "stream_tuning.h"
#include "pinch.h"
#include <mutex>
#include "utils.h"

using namespace nlohmann;

uint64_t HEAD_ID = alvr_path_string_to_id("/user/head");
uint64_t HAND_LEFT_ID = alvr_path_string_to_id("/user/hand/left");
uint64_t HAND_RIGHT_ID = alvr_path_string_to_id("/user/hand/right");

// Log raw ARCore + gyro poses at 30 Hz as PVR-Raw (for tools/calibrate_pivot.py).
const bool LOG_RAW_POSES = false;

// Hands older than this are not sent (SteamVR then shows them as lost).
const int64_t HAND_MAX_AGE_NS = 300'000'000;
// Freeze head position while a hand is this close to the camera or covers this much of the
// image (its motion corrupts ARCore's tracking), and for a while after.
const float HAND_HOLD_DISTANCE_M = 0.30f;
const float HAND_HOLD_IMAGE_FRACTION = 0.25f;
const int64_t HAND_HOLD_AFTER_NS = 400'000'000;

// Note: the Cardboard SDK cannot estimate display time and an heuristic is used instead.
const uint64_t VSYNC_QUEUE_INTERVAL_NS = 50e6;
const float FLOOR_HEIGHT = 1.5;
const int MAXIMUM_TRACKING_FRAMES = 360;

struct NativeContext {
    JavaVM *javaVm = nullptr;
    jobject javaContext = nullptr;

    CardboardHeadTracker *headTracker = nullptr;
    CardboardLensDistortion *lensDistortion = nullptr;
    CardboardDistortionRenderer *distortionRenderer = nullptr;

    int screenWidth = 0;
    int screenHeight = 0;

    bool renderingParamsChanged = true;
    bool glContextRecreated = false;

    bool running = false;
    bool streaming = false;
    std::thread inputThread;

    // Une one texture per eye, no need for swapchains.
    GLuint lobbyTextures[2] = {0, 0};
    GLuint streamTextures[2] = {0, 0};

    float eyeOffsets[2] = {0.0, 0.0};

    // Headset fit (IPD, image size, FOV, vertical offset) on top of the QR profile. GL thread
    // only: Java sets it through GLSurfaceView.queueEvent (or before the renderer starts).
    viewer::Adjust fitAdjust;
    float screenHeightM = 0;
    bool fitChanged = false;
    viewer::Info viewerBase;   // the unadjusted profile, for the UI
    AlvrFov fovArr[2] = {};
    AlvrViewParams viewParams[2] = {};
    AlvrDeviceMotion deviceMotion = {};

    // ARCore position + gyro rotation.
    fusion::PoseFusion poseFusion;

    // Rotational timewarp. Render thread only, except the enabled flag.
    timewarp::Timewarp timewarp;
    std::atomic<bool> timewarpEnabled{true};
    // Gaze pointer: a fake right controller that follows the head (replaces the tracked right
    // hand while on). Pitch trims the angle between the sent pose and SteamVR's laser.
    std::atomic<bool> gazePointer{false};
    std::atomic<float> gazePitchDeg{0};
    AlvrViewParams lastFrameViewParams[2] = {};   // render-time views of the last stream frame
    bool haveFrame = false;

    // Latency / artifact options. Set from Java (any thread), read where used.
    // Cap on ALVR's lookahead for the poses sent to the PC (0 = no cap).
    std::atomic<int64_t> lookaheadCapNs{0};
    // Extra FOV per edge the PC renders, so timewarp rotates into real pixels, not black.
    std::atomic<float> fovMarginRad{0};
    // Timewarp: repeat edge pixels instead of black outside the rendered image.
    std::atomic<bool> stretchEdges{false};
    // Timewarp aim: 0 = 2.5 refreshes (legacy guess), 1 = measured swap->photons + half a scan.
    std::atomic<int> twAimMode{0};
    // Fraction of the aim time the gyro is extrapolated (1 = full).
    std::atomic<float> twPredictionScale{1};
    // Cut the extrapolation while the head is slowing down (no overshoot when a turn stops).
    std::atomic<bool> twDamped{false};
    tuning::DampedPrediction dampedPrediction;   // render thread
    float lastTwHorizonMs = 0;
    // Positional timewarp (stream_tuning.h::planarReproject): off by default.
    std::atomic<bool> twPositional{false};
    std::atomic<float> twPlaneDistM{2.0f};
    float twPosSumCm = 0, twPosMaxCm = 0;   // per-second stats, render thread

    // Video pipeline stats from the ALVR fork, per second (render thread).
    AlvrPvrStats lastPvrStats = {};
    int64_t netStatsStartNs = 0;
    int renderedFrames = 0;
    double renderCpuMsSum = 0, renderCpuMsMax = 0, arCpuMsSum = 0, arCpuMsMax = 0;
    // How late new frames are: now minus the time they were predicted to be shown. A few ms
    // normally; hundreds of ms = the streamer is queueing frames faster than the link drains them.
    double lateMsSum = 0, lateMsMax = -1e9;
    int lateSamples = 0;
    char netStatus[400] = "video: no stream yet";

    // Comfort guard (stream_tuning.h): shows the head-tracked lobby instead of badly late frames.
    tuning::ComfortGuard comfortGuard;   // render thread
    std::atomic<bool> comfortGuardEnabled{true};
    std::atomic<bool> guardActive{false};   // read by Java for the in-VR banner
    std::atomic<int> guardLateMs{0};
    // Per-second stats for the status line.
    int64_t twStatsStartNs = 0;
    int twNewFrames = 0, twRepeats = 0, twSamples = 0;
    float twAngleSumDeg = 0, twAngleMaxDeg = 0;
    char twStatus[320] = "timewarp: no stream yet";

    // Hand tracking. Written by the MediaPipe worker, read by the input thread.
    std::mutex handsMutex;
    hands::HandObservation hands[2];   // 0 = left, 1 = right, ARCore world (latest raw, for stats)
    hands::HandFilter handFilter;      // smoothed + predicted tracks actually sent to ALVR
    hands::AimFilter aimFilters[2];    // controller pointing per side (input thread only)
    int64_t handStatsStartNs = 0;
    int handResults = 0, handsSeen[2] = {0, 0};
    float detectMsSum = 0, handAgeMsSum = 0;
    char handStatus[300] = "hands: waiting for ARCore camera frames";
    // Phone-side pinch (trigger) and grip (pinch.h), decided per real camera observation in
    // setHandsNative (under handsMutex); the input thread sends button changes.
    std::atomic<bool> phoneGestures{true};
    pinch::Hysteresis pinchDet[2], gripDet[2];
    bool sentTrigger[2] = {false, false}, sentGrip[2] = {false, false};   // input thread
    // Air joystick (pinch.h): left pinch-and-drag = left thumbstick (input thread state).
    std::atomic<bool> airJoystick{true};
    std::atomic<float> joyDeadzoneM{0.02f}, joyFullM{0.12f};
    pinch::AirJoystick airJoy;
    float joySentX = 0, joySentY = 0;
    bool joyTouchSent = false;
    int64_t tapReleaseNs = 0;   // left trigger tap: release time, 0 = none
    // Boot time a fresh detection of each side last *arrived* (not its capture time: results
    // arrive ~100 ms after capture, ~60 ms apart).
    int64_t handResultNs[2] = {0, 0};
    // Pointer freeze while pinched: recent aim orientations per side (input thread).
    static constexpr int kAimHistory = 64;
    AlvrQuat aimHist[2][kAimHistory] = {};
    int64_t aimHistNs[2][kAimHistory] = {};
    int aimHistNext[2] = {0, 0};
    bool aimFrozen[2] = {false, false};
    AlvrQuat frozenAim[2] = {};
    // Per-second min/max of the gesture distances (metres), per side.
    float pinchMinM[2] = {9, 9}, pinchMaxM[2] = {0, 0}, gripMinM[2] = {9, 9}, gripMaxM[2] = {0, 0};

    NativeContext() {
        memset(&fovArr, 0, (sizeof(fovArr)) / sizeof(int));
        memset(&viewParams, 0, (sizeof(viewParams)) / sizeof(int));
        memset(&deviceMotion, 0, (sizeof(deviceMotion)) / sizeof(int));
    }
};

NativeContext CTX = {};

int64_t GetBootTimeNano() {
    struct timespec res = {};
    clock_gettime(CLOCK_BOOTTIME, &res);
    return (res.tv_sec * 1e9) + res.tv_nsec;
}

// Inverse unit quaternion
AlvrQuat inverseQuat(AlvrQuat q) { return {-q.x, -q.y, -q.z, q.w}; }

void cross(float a[3], float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void quatVecMultiply(AlvrQuat q, float v[3], float out[3]) {
    float rv[3], rrv[3];
    float r[3] = {q.x, q.y, q.z};
    cross(r, v, rv);
    cross(r, rv, rrv);
    for (int i = 0; i < 3; i++) {
        out[i] = v[i] + 2 * (q.w * rv[i] + rrv[i]);
    }
}

void offsetPosWithQuat(AlvrQuat q, float offset[3], float outPos[3]) {
    float rotatedOffset[3];
    quatVecMultiply(q, offset, rotatedOffset);

    outPos[0] -= rotatedOffset[0];
    outPos[1] -= rotatedOffset[1];
    outPos[2] -= rotatedOffset[2];
}

AlvrFov getFov(CardboardEye eye) {
    float f[4];
    CardboardLensDistortion_getFieldOfView(CTX.lensDistortion, eye, f);

    AlvrFov fov = {};
    fov.left = -f[0];
    fov.right = f[1];
    fov.up = f[3];
    fov.down = -f[2];

    return fov;
}

// Cardboard gyro orientation, world from head.
AlvrQuat getGyroOrientation(int64_t timestampNs) {
    float pos[3];
    float q[4];
    CardboardHeadTracker_getPose(CTX.headTracker, timestampNs, kLandscapeLeft, pos, q);

    // Cardboard returns head-from-world.
    auto inverseOrientation = AlvrQuat{q[0], q[1], q[2], q[3]};
    return inverseQuat(inverseOrientation);
}

// Head pose sent to ALVR at timestampNs: gyro rotation + predicted ARCore position (3DoF until
// ARCore tracks), plus velocities.
const int64_t ANGULAR_VELOCITY_DT_NS = 10'000'000;

fusion::FusedPose getFusedPose(uint64_t timestampNs) {
    auto t = (int64_t) timestampNs;
    fusion::FusedPose fused =
        CTX.poseFusion.fuse(t,
                            getGyroOrientation(t),
                            getGyroOrientation(t - ANGULAR_VELOCITY_DT_NS),
                            ANGULAR_VELOCITY_DT_NS * 1e-9f);
    fused.pose.position[1] += FLOOR_HEIGHT;
    return fused;
}

AlvrPose getPose(uint64_t timestampNs) { return getFusedPose(timestampNs).pose; }

// How long after the timewarp pass the image reaches the screen: GLSurfaceView buffering plus
// scan-out, taken as 2.5 display refreshes of the measured render-loop interval (~21 ms at 120 Hz,
// ~42 ms at 60 Hz). The warp targets the head orientation at that moment.
const float TIMEWARP_DISPLAY_LATENCY_FRAMES = 2.5f;
int64_t lastRenderCallNs = 0;
float renderIntervalS = 1.f / 60.f;   // smoothed render-loop interval

void measureRenderInterval(int64_t nowNs) {
    if (lastRenderCallNs) {
        float dt = (nowNs - lastRenderCallNs) * 1e-9f;
        if (dt > 0.004f && dt < 0.05f)
            renderIntervalS += 0.05f * (dt - renderIntervalS);
    }
    lastRenderCallNs = nowNs;
}

// Real swap -> photons time, from EGL_ANDROID_get_frame_timestamps. 0 until measured.
// Measured 62 ms at 60 Hz on the S24U, against the 42 ms the legacy aim assumes.
std::atomic<int64_t> measuredDisplayLatencyNs{0};
bool frameTimestampsSupported = true;

struct PendingFrame {
    EGLuint64KHR id;
    int64_t swapRequestNs;
};
std::deque<PendingFrame> pendingFrames;

// Render thread, right before onDrawFrame returns (GLSurfaceView swaps right after).
void trackDisplayLatency() {
    static auto getNextId =
        (PFNEGLGETNEXTFRAMEIDANDROIDPROC) eglGetProcAddress("eglGetNextFrameIdANDROID");
    static auto getTimes =
        (PFNEGLGETFRAMETIMESTAMPSANDROIDPROC) eglGetProcAddress("eglGetFrameTimestampsANDROID");
    static EGLSurface enabledFor = EGL_NO_SURFACE;
    if (!getNextId || !getTimes) {
        if (frameTimestampsSupported)
            info("PVR-Display: EGL_ANDROID_get_frame_timestamps not available");
        frameTimestampsSupported = false;
        return;
    }
    EGLDisplay dpy = eglGetCurrentDisplay();
    EGLSurface surf = eglGetCurrentSurface(EGL_DRAW);
    if (surf == EGL_NO_SURFACE)
        return;
    if (surf != enabledFor) {   // new surface: turn timestamps on
        EGLBoolean ok = eglSurfaceAttrib(dpy, surf, EGL_TIMESTAMPS_ANDROID, EGL_TRUE);
        info("PVR-Display: frame timestamps enabled: %s", ok ? "yes" : "no");
        enabledFor = surf;
        pendingFrames.clear();
        return;
    }
    EGLuint64KHR id;
    if (getNextId(dpy, surf, &id))
        pendingFrames.push_back({id, GetBootTimeNano()});
    const EGLint names[] = {EGL_DISPLAY_PRESENT_TIME_ANDROID};
    while (pendingFrames.size() > 6) {
        PendingFrame f = pendingFrames.front();
        pendingFrames.pop_front();
        EGLnsecsANDROID v[1] = {};
        if (!getTimes(dpy, surf, f.id, 1, names, v) || v[0] <= 0)
            continue;   // EGL_TIMESTAMP_PENDING / INVALID
        // DISPLAY_PRESENT is CLOCK_MONOTONIC; convert to BOOTTIME like the rest of the code.
        timespec mono{}, boot{};
        clock_gettime(CLOCK_MONOTONIC, &mono);
        clock_gettime(CLOCK_BOOTTIME, &boot);
        int64_t bootMinusMono =
            (boot.tv_sec - mono.tv_sec) * 1000000000LL + (boot.tv_nsec - mono.tv_nsec);
        int64_t lat = (int64_t) v[0] + bootMinusMono - f.swapRequestNs;
        if (lat > 0 && lat < 200000000) {
            int64_t old = measuredDisplayLatencyNs.load();
            measuredDisplayLatencyNs = old ? old + (lat - old) / 16 : lat;
        }
    }
}

// How far ahead of "now" (taken just before the warp) the warped image is seen.
int64_t timewarpDisplayLatencyNs() {
    int64_t m = measuredDisplayLatencyNs.load();
    if (CTX.twAimMode.load() == 1 && m > 0) {
        // Measured swap->photons plus half a refresh of scan-out to the middle of the panel.
        return m + (int64_t) (0.5f * renderIntervalS * 1e9f);
    }
    return (int64_t) (TIMEWARP_DISPLAY_LATENCY_FRAMES * renderIntervalS * 1e9f);
}

// The display-time head orientation for the warp (output world), with the prediction options.
AlvrQuat timewarpDisplayOrientation(int64_t nowNs, int64_t *targetNsOut = nullptr) {
    AlvrQuat current = getGyroOrientation(nowNs);
    float damp = CTX.dampedPrediction.update(nowNs, current);
    float scale = CTX.twPredictionScale.load();
    if (CTX.twDamped.load())
        scale *= damp;
    int64_t horizon = (int64_t) (timewarpDisplayLatencyNs() * scale);
    CTX.lastTwHorizonMs = horizon * 1e-6f;
    if (targetNsOut)
        *targetNsOut = nowNs + horizon;
    return CTX.poseFusion.orientationFor(getGyroOrientation(nowNs + horizon));
}

// Positional timewarp: for each eye, where the eye will be at targetNs relative to where the
// frame was rendered, in the render eye's frame (metres), clamped. False before ARCore's first
// pose (3DoF), then rotation-only timewarp is used.
bool positionalEyeOffsets(int64_t targetNs, float out[2][3]) {
    AlvrPose head = {};
    if (!CTX.poseFusion.poseFor(targetNs, getGyroOrientation(targetNs), head))
        return false;
    head.position[1] += FLOOR_HEIGHT;   // same as getFusedPose()
    float maxCm = 0;
    for (int eye = 0; eye < 2; eye++) {
        float eyePos[3] = {head.position[0], head.position[1], head.position[2]};
        float headToEye[3] = {CTX.eyeOffsets[eye], 0.0, 0.0};
        offsetPosWithQuat(head.orientation, headToEye, eyePos);   // as updateViewConfigs()
        const AlvrPose &r = CTX.lastFrameViewParams[eye].pose;
        float world[3];
        for (int i = 0; i < 3; i++)
            world[i] = eyePos[i] - r.position[i];
        fusion::rotate(fusion::conj(r.orientation), world, out[eye]);
        float len = std::sqrt(out[eye][0] * out[eye][0] + out[eye][1] * out[eye][1] +
                              out[eye][2] * out[eye][2]);
        if (len > tuning::kMaxPositionalOffsetM)
            for (int i = 0; i < 3; i++)
                out[eye][i] *= tuning::kMaxPositionalOffsetM / len;
        maxCm = std::max(maxCm, std::min(len, tuning::kMaxPositionalOffsetM) * 100.f);
    }
    CTX.twPosSumCm += maxCm;
    CTX.twPosMaxCm = std::max(CTX.twPosMaxCm, maxCm);
    return true;
}

void updateTimewarpStats(int64_t nowNs, bool newFrame, AlvrQuat rendered, AlvrQuat displayed) {
    if (newFrame)
        CTX.twNewFrames++;
    else
        CTX.twRepeats++;
    AlvrQuat d = fusion::normalize(fusion::mul(fusion::conj(rendered), displayed));
    float deg = fusion::angleOf(d) * 180.f / (float) M_PI;
    CTX.twAngleSumDeg += deg;
    CTX.twAngleMaxDeg = std::max(CTX.twAngleMaxDeg, deg);
    CTX.twSamples++;
    if (CTX.twStatsStartNs == 0)
        CTX.twStatsStartNs = nowNs;
    double elapsed = (nowNs - CTX.twStatsStartNs) * 1e-9;
    if (elapsed >= 1.0) {
        snprintf(CTX.twStatus,
                 sizeof(CTX.twStatus),
                 "timewarp ON  render %.0f Hz  new %.0f/s  re-warped %.0f/s  aim +%.0f ms (%s)  "
                 "disp %.0f ms\n"
                 "correction avg %.1f max %.1f deg  pred x%.2f%s  turn %.0f deg/s  pos %s %.1f/%.1f cm",
                 1.f / renderIntervalS,
                 CTX.twNewFrames / elapsed,
                 CTX.twRepeats / elapsed,
                 CTX.lastTwHorizonMs,
                 CTX.twAimMode.load() == 1 ? "measured" : "2.5 refr",
                 measuredDisplayLatencyNs.load() * 1e-6,
                 CTX.twAngleSumDeg / std::max(1, CTX.twSamples),
                 CTX.twAngleMaxDeg,
                 CTX.twPredictionScale.load(),
                 CTX.twDamped.load() ? " damped" : "",
                 CTX.dampedPrediction.rateDegS(),
                 CTX.twPositional.load() ? "ON" : "off",
                 CTX.twPosSumCm / std::max(1, CTX.twSamples),
                 CTX.twPosMaxCm);
        CTX.twStatsStartNs = nowNs;
        CTX.twNewFrames = CTX.twRepeats = CTX.twSamples = 0;
        CTX.twAngleSumDeg = CTX.twAngleMaxDeg = 0;
        CTX.twPosSumCm = CTX.twPosMaxCm = 0;
    }
}

void updateViewConfigs(uint64_t targetTimestampNs = 0) {
    if (!targetTimestampNs)
        targetTimestampNs = GetBootTimeNano() + alvr_get_head_prediction_offset_ns();

    fusion::FusedPose fused = getFusedPose(targetTimestampNs);
    AlvrPose headPose = fused.pose;

    CTX.deviceMotion.device_id = HEAD_ID;
    CTX.deviceMotion.pose = headPose;
    for (int i = 0; i < 3; i++) {
        CTX.deviceMotion.linear_velocity[i] = fused.linearVelocity[i];
        CTX.deviceMotion.angular_velocity[i] = fused.angularVelocity[i];
    }

    float headToEye[3] = {CTX.eyeOffsets[kLeft], 0.0, 0.0};

    CTX.viewParams[kLeft].pose = headPose;
    offsetPosWithQuat(headPose.orientation, headToEye, CTX.viewParams[kLeft].pose.position);
    float margin = CTX.fovMarginRad.load();
    CTX.viewParams[kLeft].fov = tuning::expandFov(CTX.fovArr[kLeft], margin);

    headToEye[0] = CTX.eyeOffsets[kRight];
    CTX.viewParams[kRight].pose = headPose;
    offsetPosWithQuat(headPose.orientation, headToEye, CTX.viewParams[kRight].pose.position);
    CTX.viewParams[kRight].fov = tuning::expandFov(CTX.fovArr[kRight], margin);
}

// Gaze pointer pose: a controller below-left of the eyes, aimed at the point kAimDistM straight
// ahead, so the laser dot sits in the centre of the view for menus at that distance.
// It is the LEFT controller: on this setup SteamVR never assigns ALVR's right controller the
// RightHand role (role Invalid, checked via IVRSystem), so the dashboard draws no
// laser for it. The left one has its role, and its menu button is what opens the dashboard.
AlvrPose gazeControllerPose(const AlvrPose &head) {
    const float kOffsetHead[3] = {-0.12f, -0.18f, -0.12f};   // right, down, forward (m)
    const float kAimDistM = 2.0f;
    AlvrPose out = {};
    float off[3], fwd0[3] = {0, 0, -1}, f[3], up0[3] = {0, 1, 0}, up[3];
    fusion::rotate(head.orientation, kOffsetHead, off);
    fusion::rotate(head.orientation, fwd0, f);
    fusion::rotate(head.orientation, up0, up);
    float dir[3];
    for (int i = 0; i < 3; i++) {
        out.position[i] = head.position[i] + off[i];
        dir[i] = head.position[i] + f[i] * kAimDistM - out.position[i];
    }
    hands::normalize3(dir);
    AlvrQuat q = hands::jointOrientation(dir, up);
    float half = CTX.gazePitchDeg.load() * (float) M_PI / 180.f / 2;
    AlvrQuat pitch = {std::sin(half), 0, 0, std::cos(half)};   // about the controller's X
    out.orientation = fusion::normalize(fusion::mul(q, pitch));
    return out;
}

// Input thread, caller holds handsMutex. While a pinch is held, the pointer keeps the direction it had just before the
// fingers started closing (~150 ms earlier), so the click lands where the user aimed (as on Quest).
AlvrQuat pinchAim(int side, int64_t nowNs, AlvrQuat aim) {
    int &next = CTX.aimHistNext[side];
    CTX.aimHist[side][next] = aim;
    CTX.aimHistNs[side][next] = nowNs;
    next = (next + 1) % NativeContext::kAimHistory;
    // Called from the hands loop in inputThread, which already holds handsMutex.
    bool down = CTX.phoneGestures.load() && CTX.pinchDet[side].down();
    if (!down) {
        CTX.aimFrozen[side] = false;
        return aim;
    }
    if (!CTX.aimFrozen[side]) {
        int64_t want = nowNs - 150'000'000;
        AlvrQuat best = aim;
        int64_t bestD = INT64_MAX;
        for (int i = 0; i < NativeContext::kAimHistory; i++) {
            int64_t t = CTX.aimHistNs[side][i];
            if (t == 0)
                continue;
            int64_t d = t > want ? t - want : want - t;
            if (d < bestD) {
                bestD = d;
                best = CTX.aimHist[side][i];
            }
        }
        CTX.frozenAim[side] = best;
        CTX.aimFrozen[side] = true;
    }
    return CTX.frozenAim[side];
}

// Input thread: left pinch-and-drag -> left thumbstick. Returns the left trigger state to send
// (only a short pulse after a tap).
bool updateAirJoystick(bool pinchDown, const AlvrPose *leftSkeleton, int64_t nowNs) {
    static const uint64_t STICK_X = alvr_path_string_to_id("/user/hand/left/input/thumbstick/x");
    static const uint64_t STICK_Y = alvr_path_string_to_id("/user/hand/left/input/thumbstick/y");
    static const uint64_t STICK_TOUCH = alvr_path_string_to_id("/user/hand/left/input/thumbstick/touch");
    CTX.airJoy.deadzoneM = CTX.joyDeadzoneM.load();
    CTX.airJoy.fullM = std::max(CTX.joyFullM.load(), CTX.airJoy.deadzoneM + 0.01f);
    float point[3] = {0, 0, 0};
    bool down = pinchDown && leftSkeleton;
    if (down) {   // between thumb tip (joint 5) and index tip (joint 10)
        for (int i = 0; i < 3; i++)
            point[i] = 0.5f * (leftSkeleton[5].position[i] + leftSkeleton[10].position[i]);
    }
    float yaw = fusion::twistYaw(CTX.deviceMotion.pose.orientation);
    pinch::AirJoystick::Out o = CTX.airJoy.update(nowNs, down, point, yaw);
    if (o.active != CTX.joyTouchSent) {
        AlvrButtonValue b = {};
        b.tag = ALVR_BUTTON_VALUE_BINARY;
        b.binary = o.active;
        alvr_send_button(STICK_TOUCH, b);
        CTX.joyTouchSent = o.active;
    }
    bool zeroNow = o.x == 0 && o.y == 0, zeroSent = CTX.joySentX == 0 && CTX.joySentY == 0;
    if (std::fabs(o.x - CTX.joySentX) > 0.02f || std::fabs(o.y - CTX.joySentY) > 0.02f ||
        zeroNow != zeroSent) {
        AlvrButtonValue v = {};
        v.tag = ALVR_BUTTON_VALUE_SCALAR;
        v.scalar = o.x;
        alvr_send_button(STICK_X, v);
        v.scalar = o.y;
        alvr_send_button(STICK_Y, v);
        if (zeroNow != zeroSent)
            info("Air joystick %s", zeroNow ? "released" : "moving");
        CTX.joySentX = o.x;
        CTX.joySentY = o.y;
    }
    if (o.tap)
        CTX.tapReleaseNs = nowNs + 80'000'000;   // short left trigger click
    if (CTX.tapReleaseNs && nowNs >= CTX.tapReleaseNs)
        CTX.tapReleaseNs = 0;
    return CTX.tapReleaseNs != 0;
}

// Input thread: sends trigger / grip changes for the hands (phone-side gestures). A hand not seen
// by the camera for 150 ms releases everything. The gaze pointer owns the left trigger.
void sendHandButtons(const AlvrPose *skeletons[2]) {
    static const uint64_t TRIG_VALUE[2] = {alvr_path_string_to_id("/user/hand/left/input/trigger/value"),
                                           alvr_path_string_to_id("/user/hand/right/input/trigger/value")};
    static const uint64_t TRIG_TOUCH[2] = {alvr_path_string_to_id("/user/hand/left/input/trigger/touch"),
                                           alvr_path_string_to_id("/user/hand/right/input/trigger/touch")};
    static const uint64_t SQZ_VALUE[2] = {alvr_path_string_to_id("/user/hand/left/input/squeeze/value"),
                                          alvr_path_string_to_id("/user/hand/right/input/squeeze/value")};
    static const uint64_t SQZ_CLICK[2] = {alvr_path_string_to_id("/user/hand/left/input/squeeze/click"),
                                          alvr_path_string_to_id("/user/hand/right/input/squeeze/click")};
    bool on = CTX.phoneGestures.load();
    int64_t nowNs = GetBootTimeNano();
    bool joy = on && CTX.airJoystick.load() && !CTX.gazePointer.load();
    for (int side = 0; side < 2; side++) {
        bool trig = false, grip = false;
        {
            std::lock_guard<std::mutex> lock(CTX.handsMutex);
            // Release only when the hand has really gone (no detection arriving for 300 ms).
            // This used the capture time with a 150 ms limit, which results are already ~100 ms
            // past on arrival: at 17 Hz it released between every two frames, so the 2-frame
            // press never completed (tips 3-10 mm apart, zero clicks).
            if (CTX.handFilter.lastSeenS(side) < 0 ||
                nowNs - CTX.handResultNs[side] > 300'000'000) {
                CTX.pinchDet[side].release();
                CTX.gripDet[side].release();
            }
            trig = on && CTX.pinchDet[side].down();
            grip = on && CTX.gripDet[side].down();
        }
        if (side == 0 && CTX.gazePointer.load())
            trig = grip = false;   // left controller is the gaze pointer
        if (side == 0 && (joy || CTX.joyTouchSent)) {
            // Left pinch drives the air joystick; a quick still pinch is a short trigger click.
            trig = updateAirJoystick(joy && trig, skeletons[0], nowNs);
        }
        if (trig != CTX.sentTrigger[side]) {
            AlvrButtonValue v = {};
            v.tag = ALVR_BUTTON_VALUE_SCALAR;
            v.scalar = trig ? 1.f : 0.f;
            AlvrButtonValue b = {};
            b.tag = ALVR_BUTTON_VALUE_BINARY;
            b.binary = trig;
            alvr_send_button(TRIG_TOUCH[side], b);
            alvr_send_button(TRIG_VALUE[side], v);
            CTX.sentTrigger[side] = trig;
            info("Hand %s pinch %s", side ? "right" : "left", trig ? "DOWN" : "up");
        }
        if (grip != CTX.sentGrip[side]) {
            AlvrButtonValue v = {};
            v.tag = ALVR_BUTTON_VALUE_SCALAR;
            v.scalar = grip ? 1.f : 0.f;
            AlvrButtonValue b = {};
            b.tag = ALVR_BUTTON_VALUE_BINARY;
            b.binary = grip;
            alvr_send_button(SQZ_CLICK[side], b);
            alvr_send_button(SQZ_VALUE[side], v);
            CTX.sentGrip[side] = grip;
            info("Hand %s grip %s", side ? "right" : "left", grip ? "DOWN" : "up");
        }
    }
}

// Once a second (input thread): what hand data actually went to SteamVR, per side. Palm position
// in the head frame (cm: right, up, forward) and how often a side's palm jumped more than 15 cm
// between two sends, which is what a hand flipping between the left and right controller does.
void logHandsSent(const AlvrPose *skeletons[2], int64_t nowNs) {
    static int64_t startNs = 0;
    static int sent[2] = {0, 0}, jumps[2] = {0, 0};
    static float sum[2][3] = {}, last[2][3] = {};
    static bool haveLast[2] = {false, false};
    const AlvrPose &head = CTX.deviceMotion.pose;
    for (int side = 0; side < 2; side++) {
        if (!skeletons[side]) {
            haveLast[side] = false;
            continue;
        }
        const float *p = skeletons[side][0].position;
        if (haveLast[side]) {
            float d2 = 0;
            for (int i = 0; i < 3; i++)
                d2 += (p[i] - last[side][i]) * (p[i] - last[side][i]);
            if (d2 > 0.15f * 0.15f)
                jumps[side]++;
        }
        float rel[3], local[3];
        for (int i = 0; i < 3; i++) {
            rel[i] = p[i] - head.position[i];
            last[side][i] = p[i];
        }
        haveLast[side] = true;
        fusion::rotate(fusion::conj(head.orientation), rel, local);
        sum[side][0] += local[0];
        sum[side][1] += local[1];
        sum[side][2] += -local[2];
        sent[side]++;
    }
    if (startNs == 0)
        startNs = nowNs;
    double elapsed = (nowNs - startNs) * 1e-9;
    if (elapsed < 1.0)
        return;
    if (sent[0] || sent[1]) {
        int votes[2][2];
        uint32_t dups, merged;
        {
            std::lock_guard<std::mutex> lock(CTX.handsMutex);
            CTX.handFilter.takeLabelVotes(0, votes[0]);
            CTX.handFilter.takeLabelVotes(1, votes[1]);
            dups = CTX.handFilter.duplicates();
            merged = CTX.handFilter.mergedTracks();
        }
        char side[2][128];
        for (int s = 0; s < 2; s++) {
            if (!sent[s]) {
                snprintf(side[s], sizeof(side[s]), "--");
                continue;
            }
            snprintf(side[s],
                     sizeof(side[s]),
                     "%.0f/s palm rt%+.0f up%+.0f fwd%+.0f cm jumps %d MP-label L%d/R%d",
                     sent[s] / elapsed,
                     sum[s][0] / sent[s] * 100,
                     sum[s][1] / sent[s] * 100,
                     sum[s][2] / sent[s] * 100,
                     jumps[s],
                     votes[s][0],
                     votes[s][1]);
        }
        __android_log_print(ANDROID_LOG_INFO,
                            "PVR-HandsOut",
                            "to SteamVR: L %s | R %s | duplicates %u merged %u",
                            side[0],
                            side[1],
                            dups,
                            merged);
    }
    startNs = nowNs;
    for (int s = 0; s < 2; s++) {
        sent[s] = jumps[s] = 0;
        sum[s][0] = sum[s][1] = sum[s][2] = 0;
    }
}

void inputThread() {
    auto deadline = std::chrono::steady_clock::now();

    info("inputThread: thread staring...");
    while (CTX.streaming) {

        auto lookaheadNs = (uint64_t) tuning::capLookahead(
            (int64_t) alvr_get_head_prediction_offset_ns(), CTX.lookaheadCapNs.load());
        CTX.poseFusion.noteAlvrLookahead((int64_t) lookaheadNs);
        auto targetTimestampNs = GetBootTimeNano() + lookaheadNs;
        updateViewConfigs(targetTimestampNs);

        // Hands: latest MediaPipe result per side, moved into the output world and sent as both
        // a 26-joint skeleton and a palm device motion (what ALVR's Quest client sends).
        AlvrDeviceMotion motions[3] = {CTX.deviceMotion};
        int motionCount = 1;
        AlvrPose skeletons[2][26];
        const AlvrPose *skeletonPtrs[2] = {nullptr, nullptr};
        {
            int64_t now = GetBootTimeNano();
            std::lock_guard<std::mutex> lock(CTX.handsMutex);
            (void) now;
            for (int side = 0; side < 2; side++) {
                float arPts[21][3], pts[21][3];
                if (!CTX.handFilter.predict(side, (int64_t) targetTimestampNs, arPts)) {
                    CTX.aimFilters[side].reset();
                    continue;
                }
                for (int i = 0; i < 21; i++) {
                    CTX.poseFusion.arToOutput(arPts[i], pts[i]);
                    pts[i][1] += FLOOR_HEIGHT;
                }
                // A registered pinch shows as touching fingers in game.
                if (CTX.phoneGestures.load() && CTX.pinchDet[side].down())
                    hands::snapPinch(pts, 1.f);
                hands::toOpenXrSkeleton(pts, side == 0, skeletons[side]);
                // Controller = palm position, aimed along shoulder -> knuckles (steady, natural
                // pointing). The palm joint gets the same orientation so SteamVR's finger
                // skeleton stays correct relative to the controller.
                skeletons[side][0].orientation =
                    CTX.aimFilters[side].update(CTX.deviceMotion.pose.position,
                                                CTX.deviceMotion.pose.orientation,
                                                pts[hands::MP_INDEX_MCP],
                                                side == 0,
                                                targetTimestampNs * 1e-9);
                skeletons[side][0].orientation =
                    pinchAim(side, (int64_t) targetTimestampNs, skeletons[side][0].orientation);
                skeletonPtrs[side] = skeletons[side];
                AlvrDeviceMotion &m = motions[motionCount++];
                m = {};
                m.device_id = side == 0 ? HAND_LEFT_ID : HAND_RIGHT_ID;
                m.pose = skeletons[side][0];   // palm
            }
        }
        logHandsSent(skeletonPtrs, (int64_t) targetTimestampNs);
        sendHandButtons(skeletonPtrs);

        if (CTX.gazePointer) {
            for (int i = 1; i < motionCount; i++) {
                if (motions[i].device_id == HAND_LEFT_ID) {
                    motions[i] = motions[--motionCount];
                    break;
                }
            }
            skeletonPtrs[0] = nullptr;   // controller, not a tracked hand
            AlvrDeviceMotion &m = motions[motionCount++];
            m = {};
            m.device_id = HAND_LEFT_ID;
            m.pose = gazeControllerPose(CTX.deviceMotion.pose);
        }

        alvr_send_tracking(targetTimestampNs,
                           CTX.viewParams,
                           motions,
                           motionCount,
                           (skeletonPtrs[0] || skeletonPtrs[1]) ? skeletonPtrs : nullptr,
                           nullptr);

        deadline += std::chrono::nanoseconds((uint64_t) (1e9 / 60.f / 3));
        std::this_thread::sleep_until(deadline);
    }
}

// PC assist (HandEncoder.java): copies a YUV_420_888 camera frame into the H.264 encoder's input
// image, whatever the row and pixel strides of either side are.
extern "C" JNIEXPORT jboolean JNICALL
Java_io_github_lennoxx360_phonepcvr_HandEncoder_copyYuvNative(JNIEnv *env,
                                                              jclass,
                                                              jobject syBuf,
                                                              jobject suBuf,
                                                              jobject svBuf,
                                                              jint syRow,
                                                              jint suvRow,
                                                              jint suvPix,
                                                              jobject dyBuf,
                                                              jobject duBuf,
                                                              jobject dvBuf,
                                                              jint dyRow,
                                                              jint duvRow,
                                                              jint duvPix,
                                                              jint width,
                                                              jint height) {
    auto *sy = (const uint8_t *) env->GetDirectBufferAddress(syBuf);
    auto *su = (const uint8_t *) env->GetDirectBufferAddress(suBuf);
    auto *sv = (const uint8_t *) env->GetDirectBufferAddress(svBuf);
    auto *dy = (uint8_t *) env->GetDirectBufferAddress(dyBuf);
    auto *du = (uint8_t *) env->GetDirectBufferAddress(duBuf);
    auto *dv = (uint8_t *) env->GetDirectBufferAddress(dvBuf);
    if (!sy || !su || !sv || !dy || !du || !dv)
        return JNI_FALSE;
    for (int row = 0; row < height; row++)
        memcpy(dy + row * dyRow, sy + row * syRow, width);
    int cw = width / 2, ch = height / 2;
    for (int row = 0; row < ch; row++) {
        const uint8_t *ur = su + row * suvRow, *vr = sv + row * suvRow;
        uint8_t *uo = du + row * duvRow, *vo = dv + row * duvRow;
        if (suvPix == 1 && duvPix == 1) {
            memcpy(uo, ur, cw);
            memcpy(vo, vr, cw);
            continue;
        }
        for (int x = 0; x < cw; x++) {
            uo[x * duvPix] = ur[x * suvPix];
            vo[x * duvPix] = vr[x * suvPix];
        }
    }
    return JNI_TRUE;
}

// Hand tracking (HandTracker.java, own HD stream): YUV_420_888 planes -> RGBA bitmap, full size,
// BT.601 full range (same maths as the Java fallback). ~10x faster than the Java loop.
extern "C" JNIEXPORT jboolean JNICALL
Java_io_github_lennoxx360_phonepcvr_HandTracker_yuvToRgbaNative(JNIEnv *env,
                                                                jclass,
                                                                jobject yBuf,
                                                                jobject uBuf,
                                                                jobject vBuf,
                                                                jint yRow,
                                                                jint uvRow,
                                                                jint uvPix,
                                                                jint width,
                                                                jint height,
                                                                jobject bitmap) {
    auto *y = (const uint8_t *) env->GetDirectBufferAddress(yBuf);
    auto *u = (const uint8_t *) env->GetDirectBufferAddress(uBuf);
    auto *v = (const uint8_t *) env->GetDirectBufferAddress(vBuf);
    if (!y || !u || !v)
        return JNI_FALSE;
    AndroidBitmapInfo bi;
    if (AndroidBitmap_getInfo(env, bitmap, &bi) != ANDROID_BITMAP_RESULT_SUCCESS ||
        bi.format != ANDROID_BITMAP_FORMAT_RGBA_8888 || (int) bi.width != width ||
        (int) bi.height != height)
        return JNI_FALSE;
    void *pixels = nullptr;
    if (AndroidBitmap_lockPixels(env, bitmap, &pixels) != ANDROID_BITMAP_RESULT_SUCCESS)
        return JNI_FALSE;
    for (int row = 0; row < height; row++) {
        const uint8_t *yr = y + row * yRow;
        const uint8_t *ur = u + (row / 2) * uvRow;
        const uint8_t *vr = v + (row / 2) * uvRow;
        auto *out = (uint32_t *) ((uint8_t *) pixels + row * bi.stride);
        for (int x = 0; x < width; x++) {
            int Y = yr[x];
            int U = ur[(x / 2) * uvPix] - 128;
            int V = vr[(x / 2) * uvPix] - 128;
            int r = Y + ((91881 * V) >> 16);
            int g = Y - ((22554 * U + 46802 * V) >> 16);
            int b = Y + ((116130 * U) >> 16);
            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);
            out[x] = 0xff000000u | ((uint32_t) b << 16) | ((uint32_t) g << 8) | (uint32_t) r;
        }
    }
    AndroidBitmap_unlockPixels(env, bitmap);
    return JNI_TRUE;
}

extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *) {
    CTX.javaVm = vm;
    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_initializeNative(
    JNIEnv *env, jobject obj, jint screenWidth, jint screenHeight, jfloat refreshRate) {
    CTX.javaContext = env->NewGlobalRef(obj);

    uint32_t viewWidth = std::max(screenWidth, screenHeight) / 2;
    uint32_t viewHeight = std::min(screenWidth, screenHeight);

    alvr_initialize_android_context((void *) CTX.javaVm, (void *) CTX.javaContext);

    float refreshRatesBuffer[1] = {refreshRate};

    AlvrClientCapabilities caps = {};
    caps.default_view_height = viewHeight;
    caps.default_view_width = viewWidth;
    caps.external_decoder = false;
    caps.refresh_rates = refreshRatesBuffer;
    caps.refresh_rates_count = 1;
    caps.foveated_encoding =
        true;   // By default disable FFE (can be force-enabled by Server Settings
    caps.encoder_high_profile = true;
    caps.encoder_10_bits = true;
    caps.encoder_av1 = true;

    alvr_initialize(caps);

    Cardboard_initializeAndroid(CTX.javaVm, CTX.javaContext);
    CTX.headTracker = CardboardHeadTracker_create();
}

// The name this phone announces to the ALVR dashboard (e.g. "1234.client"). Valid after
// initializeNative().
extern "C" JNIEXPORT jstring JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_getHostnameNative(JNIEnv *env, jobject) {
    char buf[128] = {};
    if (alvr_hostname(nullptr) <= sizeof(buf)) {
        alvr_hostname(buf);
    }
    return env->NewStringUTF(buf);
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_destroyNative(JNIEnv *,
                                                                                        jobject) {
    alvr_destroy_opengl();
    alvr_destroy();

    CardboardHeadTracker_destroy(CTX.headTracker);
    CTX.headTracker = nullptr;
    CardboardLensDistortion_destroy(CTX.lensDistortion);
    CTX.lensDistortion = nullptr;
    CardboardDistortionRenderer_destroy(CTX.distortionRenderer);
    CTX.distortionRenderer = nullptr;
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_resumeNative(JNIEnv *,
                                                                                       jobject) {
    CardboardHeadTracker_resume(CTX.headTracker);

    CTX.renderingParamsChanged = true;

    uint8_t *buffer;
    int size;
    CardboardQrCode_getSavedDeviceParams(&buffer, &size);
    if (size == 0) {
        CardboardQrCode_scanQrCodeAndSaveDeviceParams();
    }
    CardboardQrCode_destroy(buffer);

    CTX.running = true;

    alvr_resume();
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_pauseNative(JNIEnv *,
                                                                                      jobject) {
    alvr_pause();

    if (CTX.running) {
        CTX.running = false;
    }

    CardboardHeadTracker_pause(CTX.headTracker);
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_surfaceCreatedNative(JNIEnv *, jobject) {
    alvr_initialize_opengl();

    CTX.glContextRecreated = true;
    CTX.timewarp.forgetGlObjects();   // the old context took them with it
    CTX.haveFrame = false;
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setScreenResolutionNative(
    JNIEnv *, jobject, jint width, jint height) {
    CTX.screenWidth = width;
    CTX.screenHeight = height;

    CTX.renderingParamsChanged = true;
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_sendBatteryLevel(
    JNIEnv *, jobject, jfloat level, jboolean plugged) {
    alvr_send_battery(HEAD_ID, level, plugged);
}

// (Re)creates the lens distortion from the saved QR profile plus the headset-fit adjustments.
// Only touches Cardboard objects, so it is safe to run mid-stream when the fit changes.
bool rebuildLens(bool recreateRenderer) {
    uint8_t *buffer = nullptr;
    int size = 0;
    CardboardQrCode_getSavedDeviceParams(&buffer, &size);
    if (size == 0) {
        CardboardQrCode_destroy(buffer);
        return false;
    }

    std::vector<uint8_t> adjusted;
    viewer::Info base;
    bool ok = viewer::apply(buffer, size, CTX.fitAdjust, CTX.screenHeightM, adjusted, &base);
    if (ok) {
        CTX.viewerBase = base;
        info("Viewer profile: ipd %.1f mm, screen-lens %.1f mm, fov %.0f/%.0f/%.0f/%.0f; fit ipd "
             "%+.1f mm size x%.2f fov x%.2f vertical %+.1f mm 3D %+.1f mm",
             base.interLens * 1000,
             base.screenToLens * 1000,
             base.fov[0],
             base.fov[1],
             base.fov[2],
             base.fov[3],
             CTX.fitAdjust.ipdDeltaM * 1000,
             CTX.fitAdjust.sizeScale,
             CTX.fitAdjust.fovScale,
             CTX.fitAdjust.verticalM * 1000,
             CTX.fitAdjust.stereoDeltaM * 1000);
    } else {
        error("Couldn't decode the viewer profile; headset fit adjustments are ignored");
    }

    if (CTX.lensDistortion) {
        CardboardLensDistortion_destroy(CTX.lensDistortion);
        CTX.lensDistortion = nullptr;
    }
    CTX.lensDistortion = ok ? CardboardLensDistortion_create(
                                  adjusted.data(), int(adjusted.size()), CTX.screenWidth, CTX.screenHeight)
                            : CardboardLensDistortion_create(
                                  buffer, size, CTX.screenWidth, CTX.screenHeight);
    CardboardQrCode_destroy(buffer);

    if (recreateRenderer || !CTX.distortionRenderer) {
        if (CTX.distortionRenderer) {
            CardboardDistortionRenderer_destroy(CTX.distortionRenderer);
            CTX.distortionRenderer = nullptr;
        }
        const CardboardOpenGlEsDistortionRendererConfig config{kGlTexture2D};
        CTX.distortionRenderer = CardboardOpenGlEs2DistortionRenderer_create(&config);
    }

    for (int eye = 0; eye < 2; eye++) {
        CardboardMesh mesh;
        CardboardLensDistortion_getDistortionMesh(CTX.lensDistortion, (CardboardEye) eye, &mesh);
        CardboardDistortionRenderer_setMesh(CTX.distortionRenderer, &mesh, (CardboardEye) eye);

        float matrix[16] = {};
        CardboardLensDistortion_getEyeFromHeadMatrix(
            CTX.lensDistortion, (CardboardEye) eye, matrix);
        // matrix[12] is +-(lens IPD)/2; the 3D-depth adjustment widens only the render cameras.
        float sign = matrix[12] < 0 ? -1.0f : 1.0f;
        CTX.eyeOffsets[eye] = matrix[12] + sign * CTX.fitAdjust.stereoDeltaM / 2;
    }

    CTX.fovArr[kLeft] = getFov(kLeft);
    CTX.fovArr[kRight] = getFov(kRight);
    return true;
}

// Head-tracked lobby (ALVR's grid + HUD text) into the lobby textures. Render thread.
void renderLobby(CardboardEyeTextureDescription viewsDescs[2]) {
    AlvrPose pose = getPose(GetBootTimeNano() + VSYNC_QUEUE_INTERVAL_NS);
    AlvrViewInput viewInputs[2] = {};
    for (int eye = 0; eye < 2; eye++) {
        float headToEye[3] = {CTX.eyeOffsets[eye], 0.0, 0.0};
        // offset head pos to Eye Position
        for (int i = 0; i < 3; i++)
            viewInputs[eye].pose.position[i] = pose.position[i];
        offsetPosWithQuat(pose.orientation, headToEye, viewInputs[eye].pose.position);
        viewInputs[eye].pose.orientation = pose.orientation;
        viewInputs[eye].fov = getFov((CardboardEye) eye);
        viewInputs[eye].swapchain_index = 0;
    }
    alvr_render_lobby_opengl(viewInputs);
    viewsDescs[0].texture = CTX.lobbyTextures[0];
    viewsDescs[1].texture = CTX.lobbyTextures[1];
}

// Feeds one new frame's lateness to the comfort guard; switches the lobby message on changes.
void updateComfortGuard(int64_t nowNs, double lateMs) {
    bool now =CTX.comfortGuard.onFrame(nowNs, lateMs) && CTX.comfortGuardEnabled.load();
    CTX.guardLateMs = (int) CTX.comfortGuard.lateMs();
    if (now != CTX.guardActive.load()) {
        CTX.guardActive = now;
        if (now) {
            alvr_update_hud_message_opengl(
                "The stream is falling behind (frames arrive late).\n"
                "Paused here so it doesn't make you sick.\n\n"
                "PC: run tools/alvr-pc-profile.ps1 -Profile BadWifiHighEndPc\n"
                "or use the PC hotspot / a USB cable.");
            info("Comfort guard ON: frames %.0f ms late", CTX.comfortGuard.lateMs());
        } else {
            alvr_update_hud_message_opengl("");
            info("Comfort guard OFF: frames on time again");
        }
    }
}

// Once a second (render thread): the ALVR fork's video counters as rates, plus render timing.
// Shown in the debug overlay and logged as PVR-Net, so stalls can be traced to network (frames
// not arriving), decoder (drops/backlog) or display (re-warps with frames available).
void updateNetStats(int64_t renderStartNs, int64_t arCoreNs) {
    int64_t now = GetBootTimeNano();
    double renderMs = (now - renderStartNs) * 1e-6, arMs = arCoreNs * 1e-6;
    CTX.renderedFrames++;
    CTX.renderCpuMsSum += renderMs;
    CTX.renderCpuMsMax = std::max(CTX.renderCpuMsMax, renderMs);
    CTX.arCpuMsSum += arMs;
    CTX.arCpuMsMax = std::max(CTX.arCpuMsMax, arMs);
    if (CTX.netStatsStartNs == 0) {
        CTX.netStatsStartNs = now;
        alvr_pvr_get_stats(&CTX.lastPvrStats);
        return;
    }
    double elapsed = (now - CTX.netStatsStartNs) * 1e-9;
    if (elapsed < 1.0)
        return;
    AlvrPvrStats st = {};
    alvr_pvr_get_stats(&st);
    const AlvrPvrStats &o = CTX.lastPvrStats;
    auto rate = [&](uint64_t a, uint64_t b) { return (double) (a - b) / elapsed; };
    uint64_t gaps = st.arrival_gap_count - o.arrival_gap_count;
    uint64_t decoded = st.frames_decoded_out - o.frames_decoded_out;
    uint64_t presented = st.frames_presented - o.frames_presented;
    snprintf(CTX.netStatus,
             sizeof(CTX.netStatus),
             "video in %.0f/s (gap avg %.1f max %.0f ms) dec %.0f/s %.1f ms  shown %.0f/s "
             "queue %.1f ms (max %u)\n"
             "late %+.0f ms (max %+.0f)  pose-fallback %.0f/s\n"
             "drops: stale %.0f/s decoder-full %.0f corrupt %.0f lossy %.0f  IDR req %.0f got %.0f"
             "  | cpu ar %.1f/%.1f render %.1f/%.1f ms",
             rate(st.frames_received, o.frames_received),
             gaps ? (st.arrival_gap_sum_us - o.arrival_gap_sum_us) / 1000.0 / gaps : 0.0,
             st.arrival_gap_max_us / 1000.0,
             rate(st.frames_decoded_out, o.frames_decoded_out),
             decoded ? (st.decode_sum_us - o.decode_sum_us) / 1000.0 / decoded : 0.0,
             rate(st.frames_presented, o.frames_presented),
             presented ? (st.queue_sum_us - o.queue_sum_us) / 1000.0 / presented : 0.0,
             (unsigned) st.queue_len_max,
             CTX.lateSamples ? CTX.lateMsSum / CTX.lateSamples : 0.0,
             CTX.lateSamples ? CTX.lateMsMax : 0.0,
             rate(st.pose_fallbacks, o.pose_fallbacks),
             rate(st.frames_dropped_stale, o.frames_dropped_stale),
             rate(st.frames_dropped_decoder_full, o.frames_dropped_decoder_full),
             rate(st.frames_dropped_corrupt, o.frames_dropped_corrupt),
             rate(st.frames_with_packet_loss, o.frames_with_packet_loss),
             rate(st.idr_requests, o.idr_requests),
             rate(st.idr_received, o.idr_received),
             CTX.arCpuMsSum / CTX.renderedFrames,
             CTX.arCpuMsMax,
             CTX.renderCpuMsSum / CTX.renderedFrames,
             CTX.renderCpuMsMax);
    if (CTX.streaming) {
        __android_log_print(ANDROID_LOG_INFO, "PVR-Net", "%s", CTX.netStatus);
        __android_log_print(ANDROID_LOG_INFO,
                            "PVR-Net",
                            "%s | lookahead %.0f ms (alvr %.0f)",
                            CTX.twStatus,
                            (double) tuning::capLookahead(
                                (int64_t) alvr_get_head_prediction_offset_ns(),
                                CTX.lookaheadCapNs.load()) *
                                1e-6,
                            alvr_get_head_prediction_offset_ns() * 1e-6);
    }
    CTX.lastPvrStats = st;
    CTX.netStatsStartNs = now;
    CTX.renderedFrames = 0;
    CTX.renderCpuMsSum = CTX.renderCpuMsMax = CTX.arCpuMsSum = CTX.arCpuMsMax = 0;
    CTX.lateMsSum = 0;
    CTX.lateMsMax = -1e9;
    CTX.lateSamples = 0;
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setPositionalTimewarpNative(JNIEnv *,
                                                                           jobject,
                                                                           jboolean on,
                                                                           jfloat planeDistM) {
    CTX.twPositional = on;
    CTX.twPlaneDistM = std::clamp((float) planeDistM, 0.5f, 10.f);
    info("Positional timewarp %s (plane %.1f m)", on ? "on" : "off", planeDistM);
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setHandModeNative(JNIEnv *, jobject, jint mode) {
    std::lock_guard<std::mutex> lock(CTX.handsMutex);
    int m = mode < 0 || mode > 2 ? 0 : (int) mode;
    if (m != CTX.handFilter.config.handMode)
        info("Hands: %s", m == 0 ? "both" : m == 1 ? "left only" : "right only");
    CTX.handFilter.config.handMode = m;
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setPhoneGesturesNative(JNIEnv *,
                                                                       jobject,
                                                                       jboolean on,
                                                                       jfloat pinchPressMm,
                                                                       jfloat pinchReleaseMm,
                                                                       jfloat gripPressMm,
                                                                       jfloat gripReleaseMm) {
    std::lock_guard<std::mutex> lock(CTX.handsMutex);
    CTX.phoneGestures = on;
    for (int s = 0; s < 2; s++) {
        CTX.pinchDet[s].pressBelowM = pinchPressMm / 1000.f;
        CTX.pinchDet[s].releaseAboveM = std::max(pinchReleaseMm, pinchPressMm + 5.f) / 1000.f;
        CTX.pinchDet[s].releaseNowAboveM = CTX.pinchDet[s].releaseAboveM + 0.017f;
        CTX.gripDet[s].pressBelowM = gripPressMm / 1000.f;
        CTX.gripDet[s].releaseAboveM = std::max(gripReleaseMm, gripPressMm + 5.f) / 1000.f;
        CTX.gripDet[s].releaseNowAboveM = CTX.gripDet[s].releaseAboveM + 0.02f;
        if (!on) {
            CTX.pinchDet[s].release();
            CTX.gripDet[s].release();
        }
    }
    info("Phone gestures %s: pinch %.0f/%.0f mm, grip %.0f/%.0f mm (press/release)",
         on ? "on" : "off",
         pinchPressMm,
         pinchReleaseMm,
         gripPressMm,
         gripReleaseMm);
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setAirJoystickNative(
    JNIEnv *, jobject, jboolean on, jfloat deadzoneCm, jfloat fullCm) {
    CTX.airJoystick = on;
    CTX.joyDeadzoneM = deadzoneCm / 100.f;
    CTX.joyFullM = fullCm / 100.f;
    info("Air joystick %s: dead zone %.1f cm, full tilt %.1f cm", on ? "on" : "off", deadzoneCm, fullCm);
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setComfortGuardNative(JNIEnv *, jobject, jboolean on) {
    CTX.comfortGuardEnabled = on;
    if (!on)
        CTX.guardActive = false;   // takes effect on the next frame
}

// For the in-VR problem banner (UI thread): {guard active 0/1, smoothed lateness ms,
// position hold state 0 tracking / 1 ARCore lost / 2 settling / 3 hand close / -1 no ARCore}.
extern "C" JNIEXPORT jintArray JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_getProblemStateNative(JNIEnv *env, jobject) {
    jint v[3] = {CTX.guardActive.load() ? 1 : 0,
                 CTX.streaming ? CTX.guardLateMs.load() : 0,
                 CTX.poseFusion.holdState()};
    jintArray out = env->NewIntArray(3);
    env->SetIntArrayRegion(out, 0, 3, v);
    return out;
}

// Latency / artifact options from Java (any thread). See Settings.kt for what each one does.
extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setStreamOptionsNative(JNIEnv *,
                                                                       jobject,
                                                                       jboolean latestFrameOnly,
                                                                       jfloat maxBufferingFrames,
                                                                       jint decoderWaitMs,
                                                                       jint hideCorrupted,
                                                                       jint lookaheadCapMs,
                                                                       jint positionPredictionMs,
                                                                       jfloat fovMarginDeg,
                                                                       jboolean stretchEdges,
                                                                       jint aimMode,
                                                                       jfloat predictionScale,
                                                                       jboolean damped) {
    AlvrPvrOptions o = {};
    o.latest_frame_only = latestFrameOnly;
    o.max_buffering_frames = maxBufferingFrames;
    o.decoder_input_wait_ms = (uint32_t) std::max(0, (int) decoderWaitMs);
    o.hide_corrupted_frames = hideCorrupted;
    o.low_latency_decoder = true;
    alvr_pvr_set_options(&o);

    CTX.lookaheadCapNs = (int64_t) lookaheadCapMs * 1000000LL;
    CTX.poseFusion.setMaxPrediction(positionPredictionMs * 1e-3f);
    CTX.fovMarginRad = fovMarginDeg * (float) M_PI / 180.f;
    CTX.stretchEdges = stretchEdges;
    CTX.twAimMode = aimMode;
    CTX.twPredictionScale = std::clamp((float) predictionScale, 0.f, 1.5f);
    CTX.twDamped = damped;
    info("Stream options: latest-frame %d buffering %.1f decoder-wait %d ms hide-corrupt %d | "
         "lookahead cap %d ms pos-pred %d ms | overscan %.1f deg stretch %d aim %d pred x%.2f "
         "damped %d",
         (int) latestFrameOnly,
         maxBufferingFrames,
         (int) decoderWaitMs,
         (int) hideCorrupted,
         (int) lookaheadCapMs,
         (int) positionPredictionMs,
         fovMarginDeg,
         (int) stretchEdges,
         (int) aimMode,
         predictionScale,
         (int) damped);
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_renderNative(
    JNIEnv *, jobject, jlong arCoreNs) {
    int64_t renderStartNs = GetBootTimeNano();
    measureRenderInterval(renderStartNs);
    try {
        if (CTX.renderingParamsChanged || CTX.fitChanged) {
            if (!rebuildLens(CTX.renderingParamsChanged)) {
                return;
            }
            CTX.fitChanged = false;
        }

        // Note: if GL context is recreated, old resources are already freed.
        if (CTX.renderingParamsChanged && !CTX.glContextRecreated) {
            info("Pausing ALVR since glContext is not recreated, deleting textures");
            alvr_pause_opengl();

            GL(glDeleteTextures(2, CTX.lobbyTextures));
        }

        if (CTX.renderingParamsChanged || CTX.glContextRecreated) {
            info("Rebuilding, binding textures, Resuming ALVR since glContextRecreated %b, "
                 "renderingParamsChanged %b",
                 CTX.renderingParamsChanged,
                 CTX.glContextRecreated);
            GL(glGenTextures(2, CTX.lobbyTextures));

            for (auto &lobbyTexture : CTX.lobbyTextures) {
                GL(glBindTexture(GL_TEXTURE_2D, lobbyTexture));
                GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
                GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
                GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
                GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
                GL(glTexImage2D(GL_TEXTURE_2D,
                                0,
                                GL_RGB,
                                CTX.screenWidth / 2,
                                CTX.screenHeight,
                                0,
                                GL_RGB,
                                GL_UNSIGNED_BYTE,
                                nullptr));
            }

            const uint32_t *targetViews[2] = {(uint32_t *) &CTX.lobbyTextures[0],
                                              (uint32_t *) &CTX.lobbyTextures[1]};
            alvr_resume_opengl(CTX.screenWidth / 2, CTX.screenHeight, targetViews, 1, true);

            CTX.renderingParamsChanged = false;
            CTX.glContextRecreated = false;
        }

        AlvrEvent event;
        while (alvr_poll_event(&event)) {
            if (event.tag == ALVR_EVENT_HUD_MESSAGE_UPDATED) {
                auto message_length = alvr_hud_message(nullptr);
                auto message_buffer = std::vector<char>(message_length);

                alvr_hud_message(&message_buffer[0]);
                info("ALVR Poll Event: HUD Message Update - %s", &message_buffer[0]);

                if (message_length > 0)
                    alvr_update_hud_message_opengl(&message_buffer[0]);
            }
            if (event.tag == ALVR_EVENT_STREAMING_STARTED) {
                info("ALVR Poll Event: ALVR_EVENT_STREAMING_STARTED, generating and binding "
                     "textures...");
                auto config = event.STREAMING_STARTED;

                auto settings_len = alvr_get_settings_json(nullptr);
                auto settings_buffer = std::vector<char>(settings_len);
                alvr_get_settings_json(&settings_buffer[0]);

                info("Got settings from ALVR Server - %s", &settings_buffer[0]);
                if (settings_len > 900)   // to workthough logcat buffer limit
                    info("Got settings from ALVR Server - %s", &settings_buffer[900]);
                json settings_json = json::parse(&settings_buffer[0]);

                GL(glGenTextures(2, CTX.streamTextures));

                for (auto &streamTexture : CTX.streamTextures) {
                    GL(glBindTexture(GL_TEXTURE_2D, streamTexture));
                    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
                    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
                    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
                    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
                    GL(glTexImage2D(GL_TEXTURE_2D,
                                    0,
                                    GL_RGB,
                                    config.view_width,
                                    config.view_height,
                                    0,
                                    GL_RGB,
                                    GL_UNSIGNED_BYTE,
                                    nullptr));
                }

                CTX.fovArr[0] = getFov((CardboardEye) 0);
                CTX.fovArr[1] = getFov((CardboardEye) 1);

                info("ALVR Poll Event: ALVR_EVENT_STREAMING_STARTED, View configs updated...");

                auto leftIntHandle = (uint32_t) CTX.streamTextures[0];
                auto rightIntHandle = (uint32_t) CTX.streamTextures[1];
                const uint32_t *textureHandles[2] = {&leftIntHandle, &rightIntHandle};

                auto render_config = AlvrStreamConfig{};
                render_config.view_resolution_width = config.view_width;
                render_config.view_resolution_height = config.view_height;
                render_config.swapchain_textures = textureHandles;
                render_config.swapchain_length = 1;

                render_config.enable_foveation = false;
                if (!settings_json["video"].is_null()) {
                    if (!settings_json["video"]["foveated_encoding"].is_null()) {
                        info("settings_json.video.foveated_encoding is %s",
                             settings_json["video"]["foveated_encoding"].dump().c_str());

                        // Foveated encoding would be a "Enabled": {Array} or "Disabled" String
                        if (!settings_json["video"]["foveated_encoding"].is_string()) {
                            render_config.enable_foveation = true;
                            render_config.foveation_center_size_x =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["center_size_x"];
                            render_config.foveation_center_size_y =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["center_size_y"];
                            render_config.foveation_center_shift_x =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["center_shift_x"];
                            render_config.foveation_center_shift_y =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["center_shift_y"];
                            render_config.foveation_edge_ratio_x =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["edge_ratio_x"];
                            render_config.foveation_edge_ratio_y =
                                settings_json["video"]["foveated_encoding"]["Enabled"]
                                             ["edge_ratio_y"];
                        } else
                            info("foveated_encoding is Disabled");
                    } else
                        error("settings_json doesn't have a video.foveated_encoding key");
                } else
                    error("settings_json doesn't have a video key");

                info("Settings for foveation:");
                info("render_config.enable_foveation: %b", render_config.enable_foveation);
                info("render_config.foveation_center_size_x: %f",
                     render_config.foveation_center_size_x);
                info("render_config.foveation_center_size_y: %f",
                     render_config.foveation_center_size_y);
                info("render_config.foveation_center_shift_x: %f",
                     render_config.foveation_center_shift_x);
                info("render_config.foveation_center_shift_y: %f",
                     render_config.foveation_center_shift_y);
                info("render_config.foveation_edge_ratio_x: %f",
                     render_config.foveation_edge_ratio_x);
                info("render_config.foveation_edge_ratio_y: %f",
                     render_config.foveation_edge_ratio_y);

                alvr_start_stream_opengl(render_config);

                CTX.haveFrame = false;
                CTX.comfortGuard.reset();
                CTX.guardActive = false;
                if (!CTX.timewarp.create((int) config.view_width, (int) config.view_height))
                    error("timewarp: GL setup failed, showing frames unwarped");

                info("ALVR Poll Event: ALVR_EVENT_STREAMING_STARTED, opengl stream started and "
                     "input "
                     "Thread started...");
                CTX.streaming = true;
                CTX.inputThread = std::thread(inputThread);

            } else if (event.tag == ALVR_EVENT_STREAMING_STOPPED) {
                info("ALVR Poll Event: ALVR_EVENT_STREAMING_STOPPED, Waiting for inputThread to "
                     "join...");
                CTX.streaming = false;
                CTX.inputThread.join();

                GL(glDeleteTextures(2, CTX.streamTextures));
                CTX.timewarp.destroy();
                CTX.haveFrame = false;
                CTX.comfortGuard.reset();
                CTX.guardActive = false;
                info("ALVR Poll Event: ALVR_EVENT_STREAMING_STOPPED, Stream stopped deleted "
                     "textures.");
            }
        }

        CardboardEyeTextureDescription viewsDescs[2] = {};
        for (auto &viewsDesc : viewsDescs) {
            viewsDesc.left_u = 0.0;
            viewsDesc.right_u = 1.0;
            viewsDesc.top_v = 1.0;
            viewsDesc.bottom_v = 0.0;
        }

        if (CTX.streaming) {
            void *streamHardwareBuffer = nullptr;

            // alvr_get_frame writes one view per eye (stock PhoneVR passed a single struct and
            // overflowed the stack). The views are the poses this frame was rendered for.
            AlvrViewParams frameViewParams[2] = {};
            auto timestampNs = alvr_get_frame(frameViewParams, &streamHardwareBuffer);
            bool newFrame = timestampNs != -1;
            bool useTimewarp = CTX.timewarpEnabled && CTX.timewarp.ready();

            // Without timewarp a refresh with no new frame draws nothing (stock behaviour).
            // With it, the last frame is re-warped to the current head orientation. The comfort
            // guard's lobby is drawn every refresh.
            if (!newFrame && !(useTimewarp && CTX.haveFrame) && !CTX.guardActive.load()) {
                return;
            }

            if (newFrame) {
                uint32_t swapchainIndices[2] = {0, 0};
                alvr_render_stream_opengl(streamHardwareBuffer, swapchainIndices);
                alvr_report_submit(timestampNs, 0);
                double lateMs = (GetBootTimeNano() - timestampNs) * 1e-6;
                CTX.lateMsSum += lateMs;
                CTX.lateMsMax = std::max(CTX.lateMsMax, lateMs);
                CTX.lateSamples++;
                updateComfortGuard(GetBootTimeNano(), lateMs);
                CTX.lastFrameViewParams[0] = frameViewParams[0];
                CTX.lastFrameViewParams[1] = frameViewParams[1];
                CTX.haveFrame = true;
            }

            if (CTX.guardActive.load()) {
                // Frames keep being consumed above (so the backlog drains and lateness is
                // measured), but the user sees the head-tracked lobby with the explanation.
                renderLobby(viewsDescs);
            } else if (useTimewarp) {
                int64_t now = GetBootTimeNano(), targetNs = now;
                AlvrQuat displayOrientation = timewarpDisplayOrientation(now, &targetNs);
                bool stretch = CTX.stretchEdges.load();
                float eyeOffsets[2][3] = {};
                float planeDist = CTX.twPlaneDistM.load();
                bool positional = CTX.twPositional.load() && planeDist > 0 &&
                                  positionalEyeOffsets(targetNs, eyeOffsets);
                for (int eye = 0; eye < 2; eye++) {
                    // The frame's own FOV (what the PC rendered, incl. overscan) is the source;
                    // the lens FOV is the output.
                    viewsDescs[eye].texture =
                        CTX.timewarp.warp(eye,
                                          CTX.streamTextures[eye],
                                          CTX.lastFrameViewParams[eye].fov,
                                          CTX.fovArr[eye],
                                          CTX.lastFrameViewParams[eye].pose.orientation,
                                          displayOrientation,
                                          stretch,
                                          eyeOffsets[eye],
                                          positional ? planeDist : 0.f);
                }
                updateTimewarpStats(now,
                                    newFrame,
                                    CTX.lastFrameViewParams[0].pose.orientation,
                                    displayOrientation);
            } else {
                viewsDescs[0].texture = CTX.streamTextures[0];
                viewsDescs[1].texture = CTX.streamTextures[1];
                // Overscanned frame: show only the part the lens covers.
                for (int eye = 0; eye < 2; eye++) {
                    tuning::UvRect uv =
                        tuning::cropUv(CTX.lastFrameViewParams[eye].fov, CTX.fovArr[eye]);
                    if (CTX.haveFrame && uv.u1 > uv.u0 && uv.v1 > uv.v0) {
                        viewsDescs[eye].left_u = uv.u0;
                        viewsDescs[eye].right_u = uv.u1;
                        viewsDescs[eye].bottom_v = uv.v0;
                        viewsDescs[eye].top_v = uv.v1;
                    }
                }
                if (newFrame)
                    snprintf(CTX.twStatus, sizeof(CTX.twStatus), "timewarp: OFF");
            }
        } else {
            renderLobby(viewsDescs);
        }

        // info("nativeRendered: Rendering to Display...");
        CardboardDistortionRenderer_renderEyeToDisplay(CTX.distortionRenderer,
                                                       0,
                                                       0,
                                                       0,
                                                       CTX.screenWidth,
                                                       CTX.screenHeight,
                                                       &viewsDescs[0],
                                                       &viewsDescs[1]);
        trackDisplayLatency();   // GLSurfaceView swaps right after onDrawFrame returns
        updateNetStats(renderStartNs, arCoreNs);
    } catch (const json::exception &e) {
        error(std::string(std::string(__FUNCTION__) + std::string(__FILE_NAME__) +
                          std::string(e.what()))
                  .c_str());
    }
}

// Press/release the left controller's menu button. ALVR emulates Quest Touch controllers, where
// SteamVR toggles its dashboard (and game overlays like VRChat's) with the left menu button.
extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_sendSteamVrMenuNative(JNIEnv *,
                                                                       jobject,
                                                                       jboolean pressed) {
    // (The right controller's system button doesn't toggle the dashboard.)
    static const uint64_t LEFT_MENU_CLICK_ID =
        alvr_path_string_to_id("/user/hand/left/input/menu/click");
    AlvrButtonValue value = {};
    value.tag = ALVR_BUTTON_VALUE_BINARY;
    value.binary = pressed;
    alvr_send_button(LEFT_MENU_CLICK_ID, value);
    info("SteamVR menu button %s", pressed ? "down" : "up");
}

extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setGazePointerNative(
    JNIEnv *, jobject, jboolean on, jfloat pitchDeg) {
    CTX.gazePointer = on;
    CTX.gazePitchDeg = pitchDeg;
    info("Gaze pointer %s (pitch %.1f deg)", on ? "on" : "off", pitchDeg);
}

// Gaze-pointer trigger on the left (gaze) controller: full pull / release.
extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_sendTriggerNative(JNIEnv *, jobject, jboolean pressed) {
    static const uint64_t VALUE_ID = alvr_path_string_to_id("/user/hand/left/input/trigger/value");
    static const uint64_t TOUCH_ID = alvr_path_string_to_id("/user/hand/left/input/trigger/touch");
    AlvrButtonValue touch = {};
    touch.tag = ALVR_BUTTON_VALUE_BINARY;
    touch.binary = pressed;
    AlvrButtonValue value = {};
    value.tag = ALVR_BUTTON_VALUE_SCALAR;
    value.scalar = pressed ? 1.0f : 0.0f;
    alvr_send_button(TOUCH_ID, touch);
    alvr_send_button(VALUE_ID, value);
    info("Gaze trigger %s", pressed ? "down" : "up");
}

// Reset play space: current position and heading become the centre and forward. Any thread.
extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_recenterNative(JNIEnv *, jobject) {
    CTX.poseFusion.recenter();
    info("Play space reset");
}

// GL thread (or before the renderer starts). Values as shown in the UI.
extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setViewerFitNative(JNIEnv *,
                                                                    jobject,
                                                                    jfloat ipdDeltaMm,
                                                                    jfloat sizePercent,
                                                                    jfloat fovPercent,
                                                                    jfloat verticalMm,
                                                                    jfloat stereoDeltaMm,
                                                                    jfloat screenHeightM) {
    CTX.fitAdjust.ipdDeltaM = ipdDeltaMm / 1000;
    CTX.fitAdjust.sizeScale = sizePercent / 100;
    CTX.fitAdjust.fovScale = fovPercent / 100;
    CTX.fitAdjust.verticalM = verticalMm / 1000;
    CTX.fitAdjust.stereoDeltaM = stereoDeltaMm / 1000;
    CTX.screenHeightM = screenHeightM;
    CTX.fitChanged = true;
}

// The unadjusted headset profile: {lens IPD mm, screen-to-lens mm}, or null before it's loaded.
extern "C" JNIEXPORT jfloatArray JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_getViewerInfoNative(JNIEnv *env, jobject) {
    viewer::Info base = CTX.viewerBase;
    if (!base.ok)
        return nullptr;
    float values[2] = {base.interLens * 1000, base.screenToLens * 1000};
    jfloatArray out = env->NewFloatArray(2);
    env->SetFloatArrayRegion(out, 0, 2, values);
    return out;
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_switchViewerNative(JNIEnv *, jobject) {
    CardboardQrCode_scanQrCodeAndSaveDeviceParams();
}

// Called on the GL thread for every ARCore camera frame. timestampNs is CLOCK_BOOTTIME.
extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setArCorePoseNative(
    JNIEnv *,
    jobject,
    jlong timestampNs,
    jboolean tracking,
    jfloat tx,
    jfloat ty,
    jfloat tz,
    jfloat qx,
    jfloat qy,
    jfloat qz,
    jfloat qw) {
    if (!CTX.headTracker)
        return;
    float position[3] = {tx, ty, tz};
    CTX.poseFusion.noteCameraAge(GetBootTimeNano() - (int64_t) timestampNs);
    AlvrQuat gyro = getGyroOrientation(timestampNs);
    CTX.poseFusion.onArCoreFrame(timestampNs, tracking, position, AlvrQuat{qx, qy, qz, qw}, gyro);

    // Raw 30 Hz samples for offline camera->pivot calibration (tools/calibrate_pivot.py). Off
    // by default; set LOG_RAW_POSES to record a calibration.
    // Columns: t_ns tracking cam_x cam_y cam_z ar_qx ar_qy ar_qz ar_qw gyro_qx gyro_qy gyro_qz gyro_qw
    if (LOG_RAW_POSES)
        __android_log_print(ANDROID_LOG_INFO,
                        "PVR-Raw",
                        "%lld %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
                        (long long) timestampNs,
                        tracking ? 1 : 0,
                        tx,
                        ty,
                        tz,
                        qx,
                        qy,
                        qz,
                        qw,
                        gyro.x,
                        gyro.y,
                        gyro.z,
                        gyro.w);
}

extern "C" JNIEXPORT jstring JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_getFusionStatusNative(JNIEnv *env, jobject) {
    char buf[2200];
    CTX.poseFusion.status(buf, sizeof(buf));
    // Same thread as the renderer, so twStatus / netStatus are safe to read here.
    size_t used = strlen(buf);
    snprintf(buf + used, sizeof(buf) - used, "\n%s", CTX.twStatus);
    used = strlen(buf);
    snprintf(buf + used, sizeof(buf) - used, "\n%s", CTX.netStatus);
    {
        std::lock_guard<std::mutex> lock(CTX.handsMutex);
        used = strlen(buf);
        snprintf(buf + used, sizeof(buf) - used, "\n%s", CTX.handStatus);
    }
    return env->NewStringUTF(buf);
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setTimewarpEnabledNative(JNIEnv *, jobject, jboolean on) {
    CTX.timewarpEnabled = on;
}

// MediaPipe worker thread: one hand-landmarker result (0-2 hands) for an ARCore camera frame.
extern "C" JNIEXPORT void JNICALL Java_io_github_lennoxx360_phonepcvr_ALVRActivity_setHandsNative(
    JNIEnv *env,
    jobject,
    jlong captureNs,
    jint handCount,
    jbooleanArray labelledLeftArr,
    jfloatArray image2dArr,
    jfloatArray world3dArr,
    jfloatArray intrinsicsArr,
    jfloatArray cameraPoseArr,
    jfloat detectMs) {
    jboolean labelledLeft[2];
    float image2d[2 * 21 * 2], world3d[2 * 21 * 3], k[6], cam[7];
    env->GetBooleanArrayRegion(labelledLeftArr, 0, 2, labelledLeft);
    env->GetFloatArrayRegion(image2dArr, 0, 2 * 21 * 2, image2d);
    env->GetFloatArrayRegion(world3dArr, 0, 2 * 21 * 3, world3d);
    env->GetFloatArrayRegion(intrinsicsArr, 0, 6, k);
    env->GetFloatArrayRegion(cameraPoseArr, 0, 7, cam);
    hands::Intrinsics intr = {k[0], k[1], k[2], k[3], k[4], k[5]};
    float camT[3] = {cam[0], cam[1], cam[2]};
    AlvrQuat camQ = {cam[3], cam[4], cam[5], cam[6]};

    hands::HandObservation found[2];   // in detection order; the HandFilter assigns sides
    AlvrPose head = CTX.deviceMotion.pose;   // last head pose sent (output world + floor)
    for (int h = 0; h < handCount && h < 2; h++) {
        hands::HandObservation obs;
        if (!hands::placeHand((const float(*)[3]) & world3d[h * 21 * 3],
                              (const float(*)[2]) & image2d[h * 21 * 2],
                              intr,
                              camT,
                              camQ,
                              obs))
            continue;
        // MediaPipe's label flips with how the hand is held (a left hand from the back looks
        // like a right hand from the palm); it only breaks ties. Handedness comes from where the
        // hand is and which way it points relative to the head.
        obs.isLeft = labelledLeft[h];
        float wrist[3], mcp[3];
        CTX.poseFusion.arToOutput(obs.points[hands::MP_WRIST], wrist);
        CTX.poseFusion.arToOutput(obs.points[hands::MP_MIDDLE_MCP], mcp);
        wrist[1] += FLOOR_HEIGHT;
        mcp[1] += FLOOR_HEIGHT;
        obs.leftScore = hands::leftHandScore(head.position, head.orientation, wrist, mcp);
        obs.captureNs = (int64_t) captureNs;
        found[h] = obs;

        // A hand close to the camera fills the image and drags ARCore's tracking with it:
        // stop trusting ARCore position until shortly after it's gone.
        float minX = 1, minY = 1, maxX = 0, maxY = 0;
        for (int i = 0; i < 21; i++) {
            minX = std::min(minX, image2d[(h * 21 + i) * 2]);
            maxX = std::max(maxX, image2d[(h * 21 + i) * 2]);
            minY = std::min(minY, image2d[(h * 21 + i) * 2 + 1]);
            maxY = std::max(maxY, image2d[(h * 21 + i) * 2 + 1]);
        }
        float area = std::max(0.f, maxX - minX) * std::max(0.f, maxY - minY);
        if (obs.cameraDistanceM < HAND_HOLD_DISTANCE_M || area > HAND_HOLD_IMAGE_FRACTION)
            CTX.poseFusion.holdPosition((int64_t) captureNs + HAND_HOLD_AFTER_NS);
    }

    int64_t now = GetBootTimeNano();
    std::lock_guard<std::mutex> lock(CTX.handsMutex);
    CTX.handFilter.onDetections((int64_t) captureNs, 2, found);
    for (int k = 0; k < 2; k++) {   // stats per side, by the geometric score
        if (!found[k].valid)
            continue;
        int side = found[k].leftScore > 0 ? 0 : 1;
        CTX.hands[side] = found[k];
        CTX.handsSeen[side]++;
    }
    // Pinch comes straight from this frame's detection (the filtered skeleton lags and its
    // shape smoothing keeps touching tips apart); grip from the filtered skeleton.
    float rawPinch[2] = {-1, -1};
    for (int h = 0; h < 2; h++) {
        int side = CTX.handFilter.detectionSide(h);
        if (side >= 0 && found[h].valid) {
            // In a fist the thumb lies against the curled index finger: tips only, so grabbing
            // doesn't also click.
            rawPinch[side] = CTX.gripDet[side].down() ? found[h].pinchTipM : found[h].pinchM;
        }
    }
    for (int side = 0; side < 2; side++) {
        float pts[21][3];
        if (!CTX.handFilter.predict(side, (int64_t) captureNs, pts))
            continue;
        AlvrPose skel[26];
        hands::toOpenXrSkeleton(pts, side == 0, skel);
        hands::GestureDistances g = hands::measureGestures(skel);
        // Only a detection from this camera frame counts (not a held/coasted hand).
        bool fresh = std::fabs(CTX.handFilter.lastSeenS(side) - captureNs * 1e-9) < 1e-6;
        if (rawPinch[side] >= 0)
            g.pinchM = rawPinch[side];
        if (fresh)
            CTX.handResultNs[side] = now;
        if (CTX.phoneGestures.load()) {
            CTX.pinchDet[side].update((int64_t) captureNs, g.pinchM, fresh && rawPinch[side] >= 0);
            CTX.gripDet[side].update((int64_t) captureNs, g.gripM, fresh);
        }
        // Every detection: which pinch cue saw what, and how the hand was held (for tuning).
        for (int h = 0; h < 2; h++) {
            if (fresh && found[h].valid && CTX.handFilter.detectionSide(h) == side) {
                const hands::HandObservation &o = found[h];
                __android_log_print(ANDROID_LOG_INFO,
                                    "PVR-Pinch",
                                    "%c image %3.0f shape %3.0f -> %3.0f mm  grip %3.0f  palm %2.0f deg  "
                                    "dist %2.0f cm  %s",
                                    side ? 'R' : 'L',
                                    o.pinchImageM * 1000,
                                    o.pinchShapeM * 1000,
                                    o.pinchM * 1000,
                                    g.gripM * 1000,
                                    o.palmFacingDeg,
                                    o.cameraDistanceM * 100,
                                    CTX.pinchDet[side].down() ? "DOWN" : "-");
            }
        }
        CTX.pinchMinM[side] = std::min(CTX.pinchMinM[side], g.pinchM);
        CTX.pinchMaxM[side] = std::max(CTX.pinchMaxM[side], g.pinchM);
        CTX.gripMinM[side] = std::min(CTX.gripMinM[side], g.gripM);
        CTX.gripMaxM[side] = std::max(CTX.gripMaxM[side], g.gripM);
    }
    CTX.handResults++;
    CTX.detectMsSum += detectMs;
    CTX.handAgeMsSum += (now - (int64_t) captureNs) * 1e-6f;
    if (CTX.handStatsStartNs == 0)
        CTX.handStatsStartNs = now;
    double elapsed = (now - CTX.handStatsStartNs) * 1e-9;
    if (elapsed >= 1.0) {
        // Where each hand is relative to the head (head frame: right, up, forward), in cm.
        char sideText[2][112];
        for (int side = 0; side < 2; side++) {
            const hands::HandObservation &hnd = CTX.hands[side];
            if (!hnd.valid || now - hnd.captureNs > HAND_MAX_AGE_NS) {
                snprintf(sideText[side], sizeof(sideText[side]), "--");
                continue;
            }
            float wrist[3], d[3], local[3];
            CTX.poseFusion.arToOutput(hnd.points[0], wrist);
            wrist[1] += FLOOR_HEIGHT;
            for (int i = 0; i < 3; i++)
                d[i] = wrist[i] - head.position[i];
            fusion::rotate(fusion::conj(head.orientation), d, local);
            snprintf(sideText[side],
                     sizeof(sideText[side]),
                     "%.0f%% rt%+.0f up%+.0f fwd%+.0f cam%.0f pinch %.0f-%.0f grip %.0f-%.0f",
                     100.0 * CTX.handsSeen[side] / std::max(1, CTX.handResults),
                     local[0] * 100,
                     local[1] * 100,
                     -local[2] * 100,
                     hnd.cameraDistanceM * 100,
                     CTX.pinchMinM[side] * 1000,
                     CTX.pinchMaxM[side] * 1000,
                     CTX.gripMinM[side] * 1000,
                     CTX.gripMaxM[side] * 1000);
        }
        for (int side = 0; side < 2; side++) {
            CTX.pinchMinM[side] = CTX.gripMinM[side] = 9;
            CTX.pinchMaxM[side] = CTX.gripMaxM[side] = 0;
        }
        snprintf(CTX.handStatus,
                 sizeof(CTX.handStatus),
                 "hands %.0f/s det %.0f ms age %.0f ms rej %u | L %s | R %s",
                 CTX.handResults / elapsed,
                 CTX.detectMsSum / std::max(1, CTX.handResults),
                 CTX.handAgeMsSum / std::max(1, CTX.handResults),
                 (unsigned) CTX.handFilter.rejected(),
                 sideText[0],
                 sideText[1]);
        CTX.handStatsStartNs = now;
        CTX.handResults = CTX.handsSeen[0] = CTX.handsSeen[1] = 0;
        CTX.detectMsSum = CTX.handAgeMsSum = 0;
    }
}
