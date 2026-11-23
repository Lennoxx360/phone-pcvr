// Host-side tests for app/src/main/cpp/pose_fusion.h (not part of the Gradle build).
//
// Run from code/mobile/android/PhoneVR (needs the ALVR header from a client-lib build):
//   clang++ -std=c++17 -I app/src/main/cpp -I ALVR/build/alvr_client_core \
//       app/src/test/cpp/pose_fusion_test.cpp -o pose_fusion_test && ./pose_fusion_test

#define _USE_MATH_DEFINES
#include "pose_fusion.h"
#include <cmath>
#include <cstdio>
#include <random>

using namespace fusion;

static int fails = 0;

static void check(const char *name, float got, float want, float tol) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok)
        fails++;
    printf("%-44s got %+9.4f want %+9.4f  %s\n", name, got, want, ok ? "ok" : "FAIL");
}

static AlvrQuat axisAngle(float x, float y, float z, float deg) {
    float a = deg * (float) M_PI / 180 / 2, s = std::sin(a);
    return {x * s, y * s, z * s, std::cos(a)};
}

static float quatDot(AlvrQuat a, AlvrQuat b) {
    return std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

static const int64_t FRAME_NS = 33'333'333;   // ARCore ~30 fps

// Camera position for a head centre `head` with orientation q, for a phone whose camera sits
// where the default Config says it does.
static void cameraFor(const float head[3], AlvrQuat q, float cam[3]) {
    const Config config;
    float w[3];
    rotate(q, config.cameraInHead, w);
    for (int i = 0; i < 3; i++)
        cam[i] = head[i] + w[i];
}

static FusedPose fuseAt(PoseFusion &f, int64_t t, AlvrQuat gyro) {
    return f.fuse(t, gyro, gyro, 0.01f);
}

static void testAlignmentAndOffset() {
    AlvrQuat headAr =
        normalize(mul(mul(axisAngle(0, 1, 0, 40), axisAngle(1, 0, 0, -15)), axisAngle(0, 0, 1, 5)));
    // Gyro world = ARCore world rotated by -30 deg yaw (ARCore-from-gyro offset is +30).
    AlvrQuat gyro = normalize(mul(axisAngle(0, 1, 0, -30), headAr));
    float head[3] = {1, 0.2f, -2}, cam[3];
    cameraFor(head, headAr, cam);

    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    f.onArCoreFrame(0, true, cam, headAr, gyro);
    FusedPose p = fuseAt(f, 0, gyro);
    check("static: head x", p.pose.position[0], 1, 1e-4f);
    check("static: head y", p.pose.position[1], 0.2f, 1e-4f);
    check("static: head z", p.pose.position[2], -2, 1e-4f);
    check("static: fused rot == ARCore rot", quatDot(p.pose.orientation, headAr), 1, 1e-5f);
    check("static: yaw offset deg",
          twistYaw(normalize(mul(headAr, conj(gyro)))) * 180 / (float) M_PI,
          30,
          0.01f);
    check("static: tilt err deg", tiltDeg(normalize(mul(headAr, conj(gyro)))), 0, 0.1f);

    // Gyro drifts +2 deg in yaw while still: offset converges, output stays on ARCore.
    AlvrQuat drifted = normalize(mul(axisAngle(0, 1, 0, 2), gyro));
    int64_t t = 0;
    for (int i = 0; i < 300; i++)
        f.onArCoreFrame(t += FRAME_NS, true, cam, headAr, drifted);
    check("drift: corrected after 10 s", quatDot(fuseAt(f, t, drifted).pose.orientation, headAr), 1, 1e-4f);

    // Tracking lost: position frozen, no extrapolation.
    float far[3] = {5, 5, 5};
    f.onArCoreFrame(t += FRAME_NS, false, far, headAr, drifted);
    check("lost: position frozen", fuseAt(f, t + 100'000'000, drifted).pose.position[0], 1, 1e-3f);
}

static void testNoFixPassthrough() {
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    AlvrQuat gyro = axisAngle(0, 1, 0, 25);
    FusedPose p = fuseAt(f, 0, gyro);
    check("no fix: position zero", p.pose.position[0], 0, 0);
    check("no fix: rotation == gyro", quatDot(p.pose.orientation, gyro), 1, 1e-6f);
}

static void testPredictionWhileWalking() {
    // Walk along -z at 1 m/s; predict 60 ms past the last camera frame.
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    f.config.sendVelocities = true;   // to check the velocity estimate
    AlvrQuat q = {0, 0, 0, 1};
    int64_t t = 0;
    float head[3] = {0, 0, 0}, cam[3];
    for (int i = 0; i < 90; i++) {
        t = i * FRAME_NS;
        head[2] = -1.0f * (float) (t * 1e-9);
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
    }
    int64_t target = t + 60'000'000;
    float truth = -1.0f * (float) (target * 1e-9);
    FusedPose p = fuseAt(f, target, q);
    check("walk 1 m/s: predicted z (+60 ms), 1 cm tol", p.pose.position[2], truth, 0.01f);
    check("walk 1 m/s: velocity z", p.linearVelocity[2], -1, 0.05f);
}

static void testJitterSmoothing() {
    // Standing still with 3 mm RMS ARCore noise: output must be much steadier.
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    AlvrQuat q = {0, 0, 0, 1};
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0, 0.003f);
    float prev[3] = {0, 0, 0};
    double sumIn = 0, sumOut = 0;
    float prevIn[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < 300; i++) {
        int64_t t = i * FRAME_NS;
        float head[3] = {noise(rng), 1.0f + noise(rng), noise(rng)}, cam[3];
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        FusedPose p = fuseAt(f, t + 40'000'000, q);
        if (i > 30) {
            for (int k = 0; k < 3; k++) {
                sumIn += (head[k] - prevIn[k]) * (head[k] - prevIn[k]);
                sumOut += (p.pose.position[k] - prev[k]) * (p.pose.position[k] - prev[k]);
            }
            n++;
        }
        for (int k = 0; k < 3; k++) {
            prev[k] = p.pose.position[k];
            prevIn[k] = head[k];
        }
    }
    float rmsIn = (float) std::sqrt(sumIn / n) * 1000, rmsOut = (float) std::sqrt(sumOut / n) * 1000;
    printf("    still: frame-to-frame step RMS in %.2f mm, out %.2f mm\n", rmsIn, rmsOut);
    check("still: output step < 1/3 of input step", rmsOut < rmsIn / 3 ? 1.f : 0.f, 1, 0);
}

static void testYawNotUpdatedWhileTurningFast() {
    // A constant 3 deg camera/gyro yaw disagreement that only appears while turning fast
    // (as a timing mismatch would) must not drag the offset.
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    int64_t t = 0;
    float cam[3] = {0, 0, 0};
    AlvrQuat q = {0, 0, 0, 1};
    for (int i = 0; i < 30; i++)
        f.onArCoreFrame(t += FRAME_NS, true, cam, q, q);   // settle at offset 0
    for (int i = 0; i < 60; i++) {
        AlvrQuat gyro = axisAngle(0, 1, 0, 120.f * (float) (i * FRAME_NS * 1e-9));   // 120 deg/s
        AlvrQuat ar = normalize(mul(axisAngle(0, 1, 0, 3), gyro));
        f.onArCoreFrame(t += FRAME_NS, true, cam, ar, gyro);
    }
    AlvrQuat gyro = axisAngle(0, 1, 0, 10);
    float yawOut = twistYaw(fuseAt(f, t, gyro).pose.orientation) * 180 / (float) M_PI;
    check("fast turn: yaw offset not dragged (out yaw)", yawOut, 10, 0.2f);
}

static void testArCoreWorldJump() {
    // Replays what happened on device: mid-walk, ARCore re-localises and its world snaps by
    // 31 deg yaw and 0.78 m. The output must stay continuous and keep tracking real motion.
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    f.config.sendVelocities = true;   // to check the walking direction afterwards
    f.config.bleedBaseMps = f.config.bleedSpeedFraction = 0;   // continuity only; bleed tested separately
    AlvrQuat gyro = {0, 0, 0, 1};   // the user faces the same real direction throughout
    AlvrQuat snapQ = axisAngle(0, 1, 0, 31);
    float snapT[3] = {0.69f, 0.31f, 0.13f};
    int64_t t = 0;
    float real[3] = {0, 0, 0};
    FusedPose last = {};
    float maxStep = 0;
    float maxRotStepDeg = 0;
    bool jumped = false;
    for (int i = 0; i < 150; i++) {
        t = i * FRAME_NS;
        real[2] = -0.3f * (float) (t * 1e-9);   // walking forward at 0.3 m/s
        jumped = i >= 60;
        // ARCore world = real world, after frame 60 rotated + shifted.
        float head[3];
        AlvrQuat q = gyro;
        if (jumped) {
            rotate(snapQ, real, head);
            for (int k = 0; k < 3; k++)
                head[k] += snapT[k];
            q = snapQ;
        } else {
            for (int k = 0; k < 3; k++)
                head[k] = real[k];
        }
        float cam[3];
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, gyro);
        FusedPose p = fuseAt(f, t, gyro);
        if (i > 30) {
            float step = 0;
            for (int k = 0; k < 3; k++)
                step += (p.pose.position[k] - last.pose.position[k]) *
                        (p.pose.position[k] - last.pose.position[k]);
            maxStep = std::fmax(maxStep, std::sqrt(step));
            maxRotStepDeg = std::fmax(
                maxRotStepDeg,
                angleOf(normalize(mul(p.pose.orientation, conj(last.pose.orientation)))) * 180 /
                    (float) M_PI);
        }
        last = p;
    }
    // Walking 0.3 m/s at 30 fps is 1 cm per frame; anything bigger is a visible jump.
    check("world jump: max position step per frame (m)", maxStep, 0.01f, 0.006f);
    check("world jump: max rotation step per frame (deg)", maxRotStepDeg, 0, 0.5f);
    check("world jump: still walking forward (-z) after", last.linearVelocity[2], -0.3f, 0.05f);
    check("world jump: no sideways drift after", last.linearVelocity[0], 0, 0.05f);
}

// Nodding the head (pitch about the neck, +-25 deg at 1.5 Hz) with 3 mm ARCore noise, frames
// arriving 50 ms late and ALVR asking for 100 ms of lookahead. Returns the RMS error (m) of the
// output eye position against the true eye position at the display time.
static float noddingError(bool neckModel, float *jitterOut) {
    PoseFusion f;
    f.config.settleS = 0;   // tested separately
    if (!neckModel)
        for (float &e : f.config.eyeFromNeck)
            e = 0;   // old behaviour: track the eye point directly
    const float neck[3] = {0, 1.0f, 0};
    const float eyeFromNeck[3] = {0, 0.075f, -0.08f};   // the "real" head
    std::mt19937 rng(7);
    std::normal_distribution<float> noise(0, 0.003f);
    auto headAt = [&](double s) {
        return axisAngle(1, 0, 0, 25.f * (float) std::sin(2 * M_PI * 1.5 * s));
    };
    auto eyeAt = [&](AlvrQuat q, float eye[3]) {
        float r[3];
        rotate(q, eyeFromNeck, r);
        for (int i = 0; i < 3; i++)
            eye[i] = neck[i] + r[i];
    };
    double sumErr = 0, sumJit = 0;
    int n = 0;
    float prevOut[3] = {0, 0, 0}, prevTruth[3] = {0, 0, 0};
    // Render loop at 60 Hz; camera frames every other render frame, 50 ms after capture.
    for (int k = 0; k < 600; k++) {
        double now = k / 60.0;
        if (k % 2 == 0 && now >= 0.05) {
            double cap = now - 0.05;
            AlvrQuat q = headAt(cap);
            float eye[3], cam[3];
            eyeAt(q, eye);
            cameraFor(eye, q, cam);
            for (float &c : cam)
                c += noise(rng);
            f.onArCoreFrame((int64_t) (cap * 1e9), true, cam, q, q);
        }
        double target = now + 0.1;
        AlvrQuat qT = headAt(target);   // Cardboard predicts rotation from the gyro
        FusedPose p = f.fuse((int64_t) (target * 1e9), qT, qT, 0.01f);
        float truth[3];
        eyeAt(qT, truth);
        if (k > 120) {
            for (int i = 0; i < 3; i++) {
                float e = p.pose.position[i] - truth[i];
                sumErr += e * e;
                float j = (p.pose.position[i] - prevOut[i]) - (truth[i] - prevTruth[i]);
                sumJit += j * j;
            }
            n++;
        }
        for (int i = 0; i < 3; i++) {
            prevOut[i] = p.pose.position[i];
            prevTruth[i] = truth[i];
        }
    }
    *jitterOut = (float) std::sqrt(sumJit / n);
    return (float) std::sqrt(sumErr / n);
}

static void testNoddingNeckModel() {
    float jitOld, jitNew;
    float errOld = noddingError(false, &jitOld);
    float errNew = noddingError(true, &jitNew);
    printf("    nodding: eye error RMS old %.1f mm -> neck model %.1f mm; "
           "frame jitter old %.2f -> %.2f mm\n",
           errOld * 1000,
           errNew * 1000,
           jitOld * 1000,
           jitNew * 1000);
    check("nodding: neck model error < 5 mm", errNew < 0.005f ? 1.f : 0.f, 1, 0);
    check("nodding: neck model error < 1/3 of old", errNew < errOld / 3 ? 1.f : 0.f, 1, 0);
    check("nodding: neck model jitter < 1/3 of old", jitNew < jitOld / 3 ? 1.f : 0.f, 1, 0);
}

// Replays the hand-tracking test on device: the user stands still, ARCore drifts ~1.2 m over its
// first 3 s (hand in front of the camera), then snaps back. The output must not move.
static void testStartupDriftThenCorrection() {
    PoseFusion f;   // default settleS
    AlvrQuat q = {0, 0, 0, 1};
    float still[3] = {0, 0, 0}, cam[3];
    float maxDisp = 0, finalDisp = 0;
    bool haveRef = false;
    float ref[3] = {0, 0, 0};
    for (int i = 0; i < 300; i++) {   // 10 s
        int64_t t = i * FRAME_NS;
        double s = t * 1e-9;
        float head[3] = {0, 0, 0};
        if (s < 3.0)
            head[0] = 0.4f * (float) s;   // bogus drift, 0.4 m/s
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        FusedPose p = fuseAt(f, t, q);
        if (!haveRef && s >= 3.0) {
            for (int k = 0; k < 3; k++)
                ref[k] = p.pose.position[k];
            haveRef = true;
        }
        if (haveRef) {
            float d = std::sqrt((p.pose.position[0] - ref[0]) * (p.pose.position[0] - ref[0]) +
                                (p.pose.position[2] - ref[2]) * (p.pose.position[2] - ref[2]));
            maxDisp = std::fmax(maxDisp, d);
            finalDisp = d;
        }
        (void) still;
    }
    check("startup drift: output moves < 2 cm after settle", maxDisp, 0, 0.02f);
    check("startup drift: final displacement (m)", finalDisp, 0, 0.02f);
}

static void testHandHoldAbsorbsDrift() {
    // Tracking fine, then a hand covers the camera for 2 s while ARCore drifts 0.5 m; the view
    // must stay put during the hold and not snap when it ends.
    PoseFusion f;
    f.config.settleS = 0;
    f.config.bleedBaseMps = f.config.bleedSpeedFraction = 0;
    AlvrQuat q = {0, 0, 0, 1};
    float maxStep = 0, prev[3] = {0, 0, 0};
    for (int i = 0; i < 180; i++) {   // 6 s
        int64_t t = i * FRAME_NS;
        double s = t * 1e-9;
        float head[3] = {0, 0, 0}, cam[3];
        if (s >= 2.0)
            head[0] = 0.5f * (float) std::min(1.0, (s - 2.0) / 2.0);   // drifts during 2..4 s
        if (i == 60)
            f.holdPosition((int64_t) (4.1e9));   // hand close from 2 s to 4.1 s
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        FusedPose p = fuseAt(f, t, q);
        if (i > 0)
            maxStep = std::fmax(maxStep, std::fabs(p.pose.position[0] - prev[0]));
        for (int k = 0; k < 3; k++)
            prev[k] = p.pose.position[k];
    }
    check("hand hold: largest frame step (m)", maxStep, 0, 0.005f);
    check("hand hold: view did not follow the 0.5 m drift", prev[0], 0, 0.01f);
}

// Head held still at x = 0, then ARCore's world jumps by jumpM along x at 1 s. Returns the
// output x after `seconds` and the largest per-frame output step after the jump.
static float stillThenJump(float jumpM, float seconds, float *maxStepOut) {
    PoseFusion f;
    f.config.settleS = 0;
    AlvrQuat q = {0, 0, 0, 1};
    float head[3] = {0, 0, 0}, cam[3];
    float lastX = 0, maxStep = 0;
    int frames = (int) (seconds * 30);
    for (int i = 0; i < frames; i++) {
        int64_t t = i * FRAME_NS;
        head[0] = i < 30 ? 0.f : jumpM;
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        float x = fuseAt(f, t, q).pose.position[0];
        if (i > 31)
            maxStep = std::fmax(maxStep, std::fabs(x - lastX));
        lastX = x;
    }
    *maxStepOut = maxStep;
    return lastX;
}

static void testBleedReturnsToArCore() {
    // A small absorbed jump that was really ARCore correcting itself: the output should
    // return to ARCore's world over time instead of staying displaced forever.
    float maxStep;
    float x = stillThenJump(0.25f, 50, &maxStep);
    check("bleed: output back at ARCore position after 50 s", x, 0.25f, 0.02f);
    check("bleed: never moves faster than ~1.5 cm/s (per frame m)", maxStep, 0, 0.0008f);
}

static void testBigJumpDoesNotSlide() {
    // ARCore relocalising by metres (seen on device: 8 m). Bleeding all of it would slide the
    // view for minutes; only the first 0.3 m may bleed, the rest is a permanent re-anchor.
    float maxStep;
    float x = stillThenJump(4.0f, 120, &maxStep);
    check("big jump: view moved at most the bleed cap", x, 0.3f, 0.02f);
    check("big jump: still slow (per frame m)", maxStep, 0, 0.0008f);
}

static void testRecenter() {
    // Standing at (1, 0, 2) facing 90° left (+yaw), reset the play space: the head should be at
    // the origin facing forward (-z), keep its height, and walking "forward" should go along -z.
    PoseFusion f;
    f.config.settleS = 0;
    f.config.bleedBaseMps = f.config.bleedSpeedFraction = 0;
    AlvrQuat q = axisAngle(0, 1, 0, 90);
    float head[3] = {1, 0.2f, 2}, cam[3];
    int64_t t = 0;
    for (int i = 0; i < 60; i++, t += FRAME_NS) {
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        fuseAt(f, t, q);
    }
    FusedPose before = fuseAt(f, t, q);
    f.recenter();
    FusedPose after = fuseAt(f, t, q);
    check("recenter: x at origin", after.pose.position[0], 0, 0.01f);
    check("recenter: z at origin", after.pose.position[2], 0, 0.01f);
    check("recenter: height kept", after.pose.position[1], before.pose.position[1], 0.001f);
    check("recenter: facing forward (yaw deg)", twistYaw(after.pose.orientation) * 180 / M_PI, 0, 0.5f);

    // Walk 0.5 m in the facing direction (+90° yaw faces -x in the ARCore world).
    for (int i = 0; i < 90; i++, t += FRAME_NS) {
        if (i < 30)
            head[0] -= 0.5f / 30;
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
        fuseAt(f, t, q);
    }
    FusedPose walked = fuseAt(f, t, q);
    check("recenter: walking forward goes -z", walked.pose.position[2], -0.5f, 0.03f);
    check("recenter: no sideways error", walked.pose.position[0], 0, 0.03f);
    float ar[3] = {head[0], head[1], head[2]}, out[3];
    f.arToOutput(ar, out);
    check("recenter: hands use the same frame (z)", out[2], walked.pose.position[2], 0.12f);
}

static void testPoseForMatchesFuse() {
    // poseFor() (positional timewarp, every refresh) must give exactly what fuse() sends,
    // including the play-space reset, while walking and turned.
    PoseFusion f;
    f.config.settleS = 0;
    AlvrQuat q = axisAngle(0, 1, 0, 30);
    float head[3] = {0.2f, 1.6f, 0}, cam[3];
    int64_t t = 0;
    for (int i = 0; i < 90; i++) {
        t = i * FRAME_NS;
        head[2] = -0.5f * (float) (t * 1e-9);
        cameraFor(head, q, cam);
        f.onArCoreFrame(t, true, cam, q, q);
    }
    for (int pass = 0; pass < 2; pass++) {
        int64_t target = t + 50'000'000;
        AlvrQuat g = axisAngle(0, 1, 0, 34);
        AlvrPose p = {};
        bool ok = f.poseFor(target, g, p);
        FusedPose ref = fuseAt(f, target, g);
        float err = 0;
        for (int k = 0; k < 3; k++)
            err = std::fmax(err, std::fabs(p.position[k] - ref.pose.position[k]));
        check(pass ? "poseFor == fuse after recenter: position" : "poseFor == fuse: position",
              ok ? err : 1.f, 0, 1e-6f);
        check(pass ? "poseFor == fuse after recenter: orientation" : "poseFor == fuse: orientation",
              quatDot(p.orientation, ref.pose.orientation), 1, 1e-6f);
        f.recenter();
    }
}

int main() {
    testPoseForMatchesFuse();
    testAlignmentAndOffset();
    testNoFixPassthrough();
    testPredictionWhileWalking();
    testJitterSmoothing();
    testYawNotUpdatedWhileTurningFast();
    testArCoreWorldJump();
    testNoddingNeckModel();
    testStartupDriftThenCorrection();
    testHandHoldAbsorbsDrift();
    testBleedReturnsToArCore();
    testBigJumpDoesNotSlide();
    testRecenter();
    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
