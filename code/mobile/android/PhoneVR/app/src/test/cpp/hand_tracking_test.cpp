// Host-side tests for app/src/main/cpp/hand_tracking.h (not part of the Gradle build).
//
// Run from code/mobile/android/PhoneVR:
//   clang++ -std=c++17 -I app/src/main/cpp -I ALVR/build/alvr_client_core \
//       app/src/test/cpp/hand_tracking_test.cpp -o hand_tracking_test && ./hand_tracking_test

#define _USE_MATH_DEFINES
#include "hand_tracking.h"
#include <cmath>
#include <cstdio>
#include <random>

using namespace hands;

static int fails = 0;
static void check(const char *name, float got, float want, float tol) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok)
        fails++;
    printf("%-50s got %+9.4f want %+9.4f  %s\n", name, got, want, ok ? "ok" : "FAIL");
}

static AlvrQuat axisAngle(float x, float y, float z, float deg) {
    float a = deg * (float) M_PI / 180 / 2, s = std::sin(a);
    return {x * s, y * s, z * s, std::cos(a)};
}

static float quatDot(AlvrQuat a, AlvrQuat b) {
    return std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

// A right hand, palm down, fingers pointing forward (-z), thumb on the left (-x), in a y-up
// world frame. Metres, wrist at the origin.
static void rightHandPalmDown(float p[21][3]) {
    const float pts[21][3] = {
        {0, 0, 0},                                                                 // wrist
        {-0.03f, -0.01f, -0.02f}, {-0.05f, -0.01f, -0.045f}, {-0.065f, -0.01f, -0.07f},
        {-0.075f, -0.01f, -0.09f},                                                  // thumb
        {-0.025f, 0, -0.09f}, {-0.027f, 0, -0.13f}, {-0.028f, 0, -0.155f},
        {-0.029f, 0, -0.175f},                                                      // index
        {0, 0, -0.095f}, {0, 0, -0.14f}, {0, 0, -0.17f}, {0, 0, -0.19f},            // middle
        {0.02f, 0, -0.09f}, {0.022f, 0, -0.13f}, {0.023f, 0, -0.155f},
        {0.024f, 0, -0.175f},                                                       // ring
        {0.04f, 0, -0.08f}, {0.043f, 0, -0.105f}, {0.045f, 0, -0.125f},
        {0.046f, 0, -0.14f},                                                        // pinky
    };
    for (int i = 0; i < 21; i++)
        for (int k = 0; k < 3; k++)
            p[i][k] = pts[i][k];
}

// The hand is turned to every orientation in front of the camera (OpenCV axes, 45 cm away),
// projected to the image with 2 px noise, and fed with its exact metric shape (worst/best
// cases for MediaPipe are covered separately). Returns {min, max} pinch over orientations.
struct PinchRange {
    float lo = 9, hi = 0;
    int samples = 0;
};
static PinchRange pinchOverOrientations(const float shape[21][3],
                                        bool occludeThumbInImage,
                                        float *tipOnlyMax = nullptr) {
    Intrinsics k = {700, 700, 480, 360, 960, 720};
    std::mt19937 rng(11);
    std::normal_distribution<float> px(0, 2.f);
    PinchRange r;
    float tipMax = 0;
    for (int a = 0; a < 12; a++)
        for (int b = 0; b < 7; b++)
            for (int c = 0; c < 6; c++) {
                float yaw = a * 30 * 0.0174533f, pitch = (b * 30 - 90) * 0.0174533f,
                      roll = c * 60 * 0.0174533f;
                float cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch),
                      sp = std::sin(pitch), cr = std::cos(roll), sr = std::sin(roll);
                // R = Ry(yaw) * Rx(pitch) * Rz(roll)
                float R[3][3] = {
                    {cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr, sy * cp},
                    {cp * sr, cp * cr, -sp},
                    {-sy * cr + cy * sp * sr, sy * sr + cy * sp * cr, cy * cp},
                };
                float world[21][3], img[21][2];
                float centre[3] = {0, 0, 0};
                for (int i = 0; i < 21; i++) {
                    for (int q = 0; q < 3; q++)
                        world[i][q] = R[q][0] * shape[i][0] + R[q][1] * shape[i][1] +
                                      R[q][2] * shape[i][2];
                    for (int q = 0; q < 3; q++)
                        centre[q] += world[i][q] / 21;
                }
                for (int i = 0; i < 21; i++) {
                    for (int q = 0; q < 3; q++)
                        world[i][q] -= centre[q];
                    float X = world[i][0], Y = world[i][1], Z = world[i][2] + 0.45f;
                    img[i][0] = (k.fx * X / Z + k.cx + px(rng)) / k.width;
                    img[i][1] = (k.fy * Y / Z + k.cy + px(rng)) / k.height;
                }
                if (occludeThumbInImage) {   // hidden thumb tip: MediaPipe's 2D guess is 3 cm off
                    img[MP_THUMB_TIP][0] += k.fx * 0.03f / 0.45f / k.width;
                    img[MP_THUMB_TIP][1] -= k.fy * 0.01f / 0.45f / k.height;
                }
                float camT[3] = {0, 0, 0};
                AlvrQuat camQ = {0, 0, 0, 1};
                HandObservation o;
                if (!placeHand(world, img, k, camT, camQ, o, true))
                    continue;
                r.lo = std::fmin(r.lo, o.pinchM);
                r.hi = std::fmax(r.hi, o.pinchM);
                tipMax = std::fmax(tipMax, o.pinchTipM);
                r.samples++;
            }
    if (tipOnlyMax)
        *tipOnlyMax = tipMax;
    return r;
}

static void testPinchFromAnyAngle() {
    float open[21][3], pad[21][3], half[21][3];
    rightHandPalmDown(open);
    // Pad pinch: thumb tip on the index fingertip's pad, 60% of the way from DIP to tip and
    // 4 mm below it, so the landmark tips are ~1 cm apart.
    for (int i = 0; i < 21; i++)
        for (int q = 0; q < 3; q++)
            pad[i][q] = half[i][q] = open[i][q];
    for (int q = 0; q < 3; q++)
        pad[MP_THUMB_TIP][q] = open[MP_INDEX_DIP][q] + 0.6f * (open[MP_INDEX_TIP][q] - open[MP_INDEX_DIP][q]);
    pad[MP_THUMB_TIP][1] -= 0.004f;
    for (int q = 0; q < 3; q++)   // thumb IP half way back towards its old place
        pad[MP_THUMB_IP][q] = 0.5f * (open[MP_THUMB_IP][q] + pad[MP_THUMB_TIP][q]);
    // Half-open hand: thumb tip 5 cm from the index tip.
    for (int q = 0; q < 3; q++)
        half[MP_THUMB_TIP][q] = open[MP_INDEX_TIP][q];
    half[MP_THUMB_TIP][0] -= 0.05f;

    float tipMax = 0;
    PinchRange rp = pinchOverOrientations(pad, false, &tipMax);
    printf("    pad pinch, %d orientations: %.1f-%.1f mm (tips only up to %.1f mm)\n",
           rp.samples, rp.lo * 1000, rp.hi * 1000, tipMax * 1000);
    check("pad pinch clicks at every angle (< 25 mm)", rp.hi < 0.025f ? 1.f : 0.f, 1, 0);
    check("pad pinch: tips alone would miss it (> 8 mm)", tipMax > 0.008f ? 1.f : 0.f, 1, 0);
    PinchRange ro = pinchOverOrientations(pad, true);
    printf("    pad pinch, thumb hidden in the image: %.1f-%.1f mm\n", ro.lo * 1000, ro.hi * 1000);
    check("hidden thumb: 3D shape still sees the pinch", ro.hi < 0.025f ? 1.f : 0.f, 1, 0);
    PinchRange rh = pinchOverOrientations(half, false);
    printf("    half-open hand (5 cm): %.1f-%.1f mm\n", rh.lo * 1000, rh.hi * 1000);
    check("half-open hand never clicks (> 25 mm)", rh.lo > 0.025f ? 1.f : 0.f, 1, 0);
    PinchRange rw = pinchOverOrientations(open, false);
    printf("    open hand: %.1f-%.1f mm\n", rw.lo * 1000, rw.hi * 1000);
    check("open hand always released (> 45 mm)", rw.lo > 0.045f ? 1.f : 0.f, 1, 0);
}

static void testTranslationSolve() {
    // Hand shape in OpenCV camera axes (x right, y down, z forward), centred near 0.
    float hand[21][3], w[21][3];
    rightHandPalmDown(hand);
    for (int i = 0; i < 21; i++) {   // y-up/-z-forward world -> OpenCV axes, centred
        w[i][0] = hand[i][0];
        w[i][1] = -hand[i][2] - 0.1f;   // fingers "up" in the image
        w[i][2] = -hand[i][1];
    }
    Intrinsics k = {480, 480, 320, 240, 640, 480};
    const float tTrue[3] = {0.05f, 0.08f, 0.35f};
    float img[21][2];
    std::mt19937 rng(3);
    std::normal_distribution<float> px(0, 1.0f);   // 1 px landmark noise
    for (int i = 0; i < 21; i++) {
        float X = w[i][0] + tTrue[0], Y = w[i][1] + tTrue[1], Z = w[i][2] + tTrue[2];
        img[i][0] = (k.fx * X / Z + k.cx + px(rng)) / k.width;
        img[i][1] = (k.fy * Y / Z + k.cy + px(rng)) / k.height;
    }
    float t[3];
    bool ok = solveTranslation(w, img, k, t);
    check("solve: succeeded", ok ? 1.f : 0.f, 1, 0);
    check("solve: tx (m)", t[0], tTrue[0], 0.005f);
    check("solve: ty (m)", t[1], tTrue[1], 0.005f);
    check("solve: tz = distance (m), 1 px noise", t[2], tTrue[2], 0.02f);

    // Camera at the ARCore origin looking down -z (identity pose): an OpenCV point straight ahead
    // at z = 0.35 must land at (0, 0, -0.35) in the ARCore world.
    float ahead[3] = {0, 0, 0.35f}, outW[3], camT[3] = {0, 0, 0};
    cameraToArWorld(ahead, camT, AlvrQuat{0, 0, 0, 1}, outW);
    check("camera->world: straight ahead is -z", outW[2], -0.35f, 1e-5f);
    float below[3] = {0, 0.1f, 0.35f};   // OpenCV +y is down
    cameraToArWorld(below, camT, AlvrQuat{0, 0, 0, 1}, outW);
    check("camera->world: image-down is world-down", outW[1], -0.1f, 1e-5f);
}

static void axes(AlvrQuat q, float x[3], float y[3], float z[3]) {
    float ex[3] = {1, 0, 0}, ey[3] = {0, 1, 0}, ez[3] = {0, 0, 1};
    fusion::rotate(q, ex, x);
    fusion::rotate(q, ey, y);
    fusion::rotate(q, ez, z);
}

static void testImageAnchoredPinch() {
    // True hand 45 cm in front of the camera (OpenCV axes), thumb tip touching index tip.
    // MediaPipe's metric shape is biased: its thumb tip sits 2.5 cm off (as seen on device).
    // Image-anchored placement must put the tips back together; the rigid shape can't.
    Intrinsics k = {500, 500, 480, 360, 960, 720};
    float hand[21][3], truth[21][3], world[21][3], img[21][2];
    rightHandPalmDown(hand);
    for (int i = 0; i < 21; i++) {   // fingers up in the image, palm facing the camera
        truth[i][0] = hand[i][0];
        truth[i][1] = -hand[i][2] - 0.1f;
        truth[i][2] = -hand[i][1] + 0.45f;
    }
    for (int c = 0; c < 3; c++)   // thumb tip (4) touches index tip (8)
        truth[4][c] = truth[8][c];
    float centre[3] = {0, 0, 0};
    for (int i = 0; i < 21; i++)
        for (int c = 0; c < 3; c++)
            centre[c] += truth[i][c] / 21;
    for (int i = 0; i < 21; i++) {
        img[i][0] = (k.fx * truth[i][0] / truth[i][2] + k.cx) / k.width;
        img[i][1] = (k.fy * truth[i][1] / truth[i][2] + k.cy) / k.height;
        for (int c = 0; c < 3; c++)
            world[i][c] = truth[i][c] - centre[c];
    }
    world[4][0] -= 0.025f;   // MediaPipe's 3D thumb tip: 2.5 cm off
    float camT[3] = {0, 0, 0};
    AlvrQuat camQ = {0, 0, 0, 1};
    HandObservation anchored, rigid;
    placeHand(world, img, k, camT, camQ, anchored, true);
    placeHand(world, img, k, camT, camQ, rigid, false);
    auto tipGap = [](const HandObservation &o) {
        float d[3];
        sub(o.points[4], o.points[8], d);
        return std::sqrt(dot(d, d));
    };
    printf("    touching tips: image-anchored %.1f mm, rigid shape %.1f mm\n",
           tipGap(anchored) * 1000,
           tipGap(rigid) * 1000);
    check("image-anchored: touching fingertips < 5 mm apart", tipGap(anchored), 0, 0.005f);
    check("rigid shape keeps the 2.5 cm error (why pinches failed)", tipGap(rigid), 0.025f, 0.006f);
    check("raw pinch distance of touching tips < 5 mm", anchored.pinchM, 0, 0.005f);
    {
        // MediaPipe's depth for the thumb tip 2 cm off: 3/4 of that counts for the pinch.
        float w2[21][3];
        for (int i = 0; i < 21; i++)
            for (int c = 0; c < 3; c++)
                w2[i][c] = world[i][c];
        w2[4][0] += 0.025f;   // undo the sideways error
        w2[4][2] += 0.02f;
        HandObservation o;
        placeHand(w2, img, k, camT, camQ, o, true);
        printf("    2 cm depth error on the thumb tip: raw pinch %.1f mm\n", o.pinchM * 1000);
        check("depth error counts 3/4 in the raw pinch", o.pinchM, 0.015f, 0.003f);
    }
    {
        // Snap: tips 3 cm apart while pinched -> sent touching; the distal joints follow half way.
        float pts[21][3];
        for (int i = 0; i < 21; i++)
            for (int c = 0; c < 3; c++)
                pts[i][c] = rigid.points[i][c];
        pts[MP_THUMB_TIP][0] = pts[MP_INDEX_TIP][0] - 0.03f;
        pts[MP_THUMB_TIP][1] = pts[MP_INDEX_TIP][1];
        pts[MP_THUMB_TIP][2] = pts[MP_INDEX_TIP][2];
        float ipBefore = pts[MP_THUMB_IP][0], wristBefore = pts[MP_WRIST][0];
        snapPinch(pts, 1.f);
        float d[3];
        sub(pts[MP_THUMB_TIP], pts[MP_INDEX_TIP], d);
        check("snap: pinched tips sent touching", std::sqrt(dot(d, d)), 0, 1e-4f);
        check("snap: thumb IP follows half the tip move", pts[MP_THUMB_IP][0] - ipBefore, 0.0075f, 1e-4f);
        check("snap: wrist untouched", pts[MP_WRIST][0] - wristBefore, 0, 1e-6f);
    }
    // Wrist lands where it really is (camera at origin, identity pose -> GL axes).
    float wristGl[3] = {truth[0][0], -truth[0][1], -truth[0][2]};
    float e[3];
    sub(anchored.points[0], wristGl, e);
    check("image-anchored: wrist position within 1 cm", std::sqrt(dot(e, e)), 0, 0.01f);
}

static void testSkeleton() {
    float mp[21][3];
    rightHandPalmDown(mp);
    AlvrPose s[26];
    toOpenXrSkeleton(mp, false, s);
    float x[3], y[3], z[3];

    // Index proximal (7) -> intermediate (8): -Z must point along the bone (forward, -z world).
    axes(s[7].orientation, x, y, z);
    float bone[3] = {s[8].position[0] - s[7].position[0],
                     s[8].position[1] - s[7].position[1],
                     s[8].position[2] - s[7].position[2]};
    normalize3(bone);
    check("skeleton: index proximal -Z along bone", -dot(z, bone), 1, 0.01f);
    check("skeleton: right palm-down +Y is up (dorsal)", y[1], 1, 0.05f);
    check("skeleton: index tip position = MP index tip x", s[10].position[0], mp[8][0], 1e-6f);
    check("skeleton: thumb tip position = MP thumb tip z", s[5].position[2], mp[4][2], 1e-6f);
    check("skeleton: palm between wrist and middle MCP (z)", s[0].position[2], mp[9][2] / 2, 1e-6f);
    axes(s[0].orientation, x, y, z);
    // -Z toward the fingers (world -z) means the +Z axis points to world +z.
    check("skeleton: palm -Z points forward", z[2], 1, 0.01f);
    // Right-handed basis: X = Y x Z.
    float c[3];
    cross(y, z, c);
    check("skeleton: basis right-handed", dot(c, x), 1, 1e-4f);

    // Mirror to a left hand (x -> -x): dorsal must still point up.
    float left[21][3];
    for (int i = 0; i < 21; i++) {
        left[i][0] = -mp[i][0];
        left[i][1] = mp[i][1];
        left[i][2] = mp[i][2];
    }
    toOpenXrSkeleton(left, true, s);
    axes(s[7].orientation, x, y, z);
    check("skeleton: left palm-down +Y is up (dorsal)", y[1], 1, 0.05f);
}

static HandObservation handAt(float x, float y, float z, bool isLeft, int64_t t) {
    HandObservation o;
    float shape[21][3];
    rightHandPalmDown(shape);
    for (int i = 0; i < 21; i++) {
        o.points[i][0] = shape[i][0] + x;
        o.points[i][1] = shape[i][1] + y;
        o.points[i][2] = shape[i][2] + z;
    }
    o.valid = true;
    o.isLeft = isLeft;
    o.captureNs = t;
    // Camera straight behind the hand (looking down -z), so z is the depth axis as on device.
    o.cameraPos[0] = x;
    o.cameraPos[1] = y;
    o.cameraPos[2] = 0;
    return o;
}

static const int64_t DET_NS = 66'666'666;   // ~15 detections/s, as measured on device

static void testFilterSmoothsStillHand() {
    HandFilter f;
    std::mt19937 rng(5);
    std::normal_distribution<float> n(0, 0.008f);   // 8 mm depth-ish noise
    float prev = 0, sumIn = 0, sumOut = 0, prevIn = 0;
    for (int i = 0; i < 90; i++) {
        int64_t t = i * DET_NS;
        float z = -0.45f + n(rng);
        HandObservation o = handAt(0.1f, 1.2f, z, true, t);
        f.onDetections(t, 1, &o);
        float out[21][3];
        f.predict(0, t + 100'000'000, out);
        if (i > 15) {
            sumIn += (z - prevIn) * (z - prevIn);
            sumOut += (out[0][2] - prev) * (out[0][2] - prev);
        }
        prev = out[0][2];
        prevIn = z;
    }
    printf("    still hand: step RMS in %.1f mm, out %.1f mm\n",
           std::sqrt(sumIn / 74) * 1000,
           std::sqrt(sumOut / 74) * 1000);
    check("filter: still hand output 3x steadier", sumOut < sumIn / 9 ? 1.f : 0.f, 1, 0);
}

static void testFilterPredictsMovingHand() {
    HandFilter f;
    for (int i = 0; i < 30; i++) {   // hand moving right at 0.5 m/s
        int64_t t = i * DET_NS;
        HandObservation o = handAt(0.5f * (float) (t * 1e-9), 1.2f, -0.45f, false, t);
        f.onDetections(t, 1, &o);
    }
    int64_t last = 29 * DET_NS, target = last + 100'000'000;
    float out[21][3];
    f.predict(1, target, out);
    float truth = 0.5f * (float) (target * 1e-9);
    // maxPredictionS (0.12 s) covers this 0.1 s look-ahead plus some of the filter lag.
    check("filter: moving hand predicted ahead (wrist x, 3 cm tol)", out[0][0], truth, 0.03f);
}

static void testFilterHoldsThenDrops() {
    HandFilter f;
    HandObservation o = handAt(0, 1.2f, -0.45f, true, 0);
    f.onDetections(0, 1, &o);
    float out[21][3];
    bool held = f.predict(0, 300'000'000, out);   // 0.3 s after capture, no new detections
    bool dropped = !f.predict(0, 600'000'000, out);
    check("filter: hand held through a short dropout", held ? 1.f : 0.f, 1, 0);
    check("filter: hand dropped after a long one", dropped ? 1.f : 0.f, 1, 0);
}

static void testFilterStickyHandedness() {
    // One hand moving continuously while MediaPipe's label flips every other frame.
    HandFilter f;
    int leftHits = 0, rightHits = 0;
    for (int i = 0; i < 30; i++) {
        int64_t t = i * DET_NS;
        HandObservation o = handAt(-0.15f + 0.002f * i, 1.2f, -0.45f, i % 2 == 0, t);
        f.onDetections(t, 1, &o);
        float out[21][3];
        if (f.predict(0, t, out))
            leftHits++;
        if (f.predict(1, t, out))
            rightHits++;
    }
    check("sticky: hand stays left despite label flips", (float) leftHits, 30, 0);
    check("sticky: no phantom right hand", (float) rightHits, 0, 0);
}

static void testNoDoubleHandAcrossGaps() {
    // 2026-11-22 log: one hand ~15 cm left of centre (borderline geometric score), MediaPipe's
    // label flipping, detections with 0.2 s gaps. It was sent as both hands at once.
    HandFilter f;
    int both = 0, leftFrames = 0, rightFrames = 0;
    int64_t t = 0;
    for (int i = 0; i < 60; i++) {
        t += (i % 4 == 3) ? 200'000'000 : DET_NS;   // every 4th gap is a 0.2 s dropout
        HandObservation o = handAt(-0.15f, 1.2f, -0.45f, i % 3 != 0, t);
        f.onDetections(t, 1, &o);
        for (int k = 0; k < 6; k++) {   // input thread samples in between
            int64_t q = t + k * 11'000'000;
            float out[21][3];
            bool l = f.predict(0, q, out), r = f.predict(1, q, out);
            both += (l && r) ? 1 : 0;
            leftFrames += l ? 1 : 0;
            rightFrames += r ? 1 : 0;
        }
    }
    check("one hand + gaps + label flips: never on both sides", (float) both, 0, 0);
    check("one hand + gaps + label flips: stays on one side",
          (float) std::min(leftFrames, rightFrames),
          0,
          0);
}

static void testDuplicateDetectionSameFrame() {
    // MediaPipe reports the same hand twice in one frame (wrists 2 cm apart, opposite labels).
    HandFilter f;
    int both = 0;
    for (int i = 0; i < 20; i++) {
        int64_t t = i * DET_NS;
        HandObservation two[2] = {handAt(-0.15f, 1.2f, -0.45f, true, t),
                                  handAt(-0.13f, 1.2f, -0.45f, false, t)};
        f.onDetections(t, 2, two);
        float out[21][3];
        both += (f.predict(0, t, out) && f.predict(1, t, out)) ? 1 : 0;
    }
    check("duplicate detection in one frame: one hand only", (float) both, 0, 0);
}

static void testTwoRealHandsStayTwo() {
    // Two real hands 30 cm apart, both seen every frame: both tracks stay.
    HandFilter f;
    int both = 0;
    for (int i = 0; i < 20; i++) {
        int64_t t = i * DET_NS;
        HandObservation two[2] = {handAt(-0.15f, 1.2f, -0.45f, true, t),
                                  handAt(0.15f, 1.2f, -0.45f, false, t)};
        f.onDetections(t, 2, two);
        float out[21][3];
        both += (f.predict(0, t, out) && f.predict(1, t, out)) ? 1 : 0;
    }
    check("two real hands: both tracked", (float) both, 20, 0);
}

static void testSwitchingHandsInSameSpot() {
    // Right hand held out, then dropped; the left hand appears in the same spot, labelled Left.
    // Within reacquireS (1.5 s) that looks exactly like the same hand coming back (the common
    // case with this camera's narrow view), so it keeps the right side; after that, label and
    // geometry decide again. (Trade-off chosen 2026-11-22; "Left/Right only" modes avoid it.)
    HandFilter f;
    for (int i = 0; i < 15; i++) {
        int64_t t = i * DET_NS;
        HandObservation r = handAt(-0.15f, 1.2f, -0.45f, false, t);
        f.onDetections(t, 1, &r);
    }
    int64_t t0 = 15 * DET_NS + 1'600'000'000;
    int leftHits = 0;
    for (int i = 0; i < 15; i++) {
        int64_t t = t0 + i * DET_NS;
        HandObservation l = handAt(-0.15f, 1.2f, -0.45f, true, t);
        f.onDetections(t, 1, &l);
        float out[21][3];
        if (f.predict(0, t, out))
            leftHits++;
    }
    check("switch hands after 1.6 s: left hand drives the left track", (float) leftHits, 15, 0);
}

static void testLoneHandReacquiredAcrossCentre() {
    // 2026-11-22 18:55:33: a lone left hand, lost for 0.5 s, came back right of centre with
    // MediaPipe saying "Right" and a geometric score pointing right. It must stay left.
    HandFilter f;
    int64_t t = 0;
    for (int i = 0; i < 20; i++) {
        t += DET_NS;
        HandObservation o = handAt(-0.15f, 1.2f, -0.45f, true, t);
        o.leftScore = 1.0f;
        f.onDetections(t, 1, &o);
    }
    t += 500'000'000;
    int rightHits = 0, leftHits = 0;
    for (int i = 0; i < 15; i++) {
        t += DET_NS;
        HandObservation o = handAt(0.12f, 1.2f, -0.45f, false, t);
        o.leftScore = -1.2f;
        f.onDetections(t, 1, &o);
        float out[21][3];
        rightHits += f.predict(1, t, out) ? 1 : 0;
        leftHits += f.predict(0, t, out) ? 1 : 0;
    }
    check("lone left hand back right of centre: stays left", (float) leftHits, 15, 0);
    check("lone left hand back right of centre: no right hand", (float) rightHits, 0, 0);
}

static void testLeftOnlyMode() {
    HandFilter f;
    f.config.handMode = 1;
    int leftHits = 0, rightHits = 0;
    for (int i = 0; i < 20; i++) {
        int64_t t = i * DET_NS;
        HandObservation o = handAt(0.2f, 1.2f, -0.45f, false, t);   // far right, labelled Right
        o.leftScore = -3.f;
        f.onDetections(t, 1, &o);
        float out[21][3];
        leftHits += f.predict(0, t, out) ? 1 : 0;
        rightHits += f.predict(1, t, out) ? 1 : 0;
    }
    check("left-only mode: every hand is the left hand", (float) leftHits, 20, 0);
    check("left-only mode: never a right hand", (float) rightHits, 0, 0);
}

static void testDepthNoiseSuppressed() {
    // Depth (along the ray) noise of 15 mm vs sideways noise of 3 mm: the wrist must be steady.
    HandFilter f;
    std::mt19937 rng(9);
    std::normal_distribution<float> nd(0, 0.015f), ns(0, 0.003f);
    float prev[3] = {0, 0, 0};
    double sum = 0;
    int n = 0;
    for (int i = 0; i < 120; i++) {
        int64_t t = i * DET_NS;
        HandObservation o = handAt(-0.1f + ns(rng), 1.2f + ns(rng), -0.45f + nd(rng), true, t);
        o.cameraPos[0] = -0.1f;
        o.cameraPos[1] = 1.2f;
        f.onDetections(t, 1, &o);
        float out[21][3];
        f.predict(0, t, out);
        if (i > 20) {
            float d2 = 0;
            for (int k = 0; k < 3; k++)
                d2 += (out[0][k] - prev[k]) * (out[0][k] - prev[k]);
            sum += d2;
            n++;
        }
        for (int k = 0; k < 3; k++)
            prev[k] = out[0][k];
    }
    float rms = (float) std::sqrt(sum / n) * 1000;
    printf("    depth noise 15 mm: wrist step RMS %.1f mm\n", rms);
    check("depth noise 15 mm -> wrist step RMS < 3 mm", rms < 3.f ? 1.f : 0.f, 1, 0);
}

static void testPinchClosesQuickly() {
    // Fingers close fast while the wrist stays put: the finger shape must follow within ~2
    // detections (the old all-joint filter lagged pinches).
    HandFilter f;
    int64_t t = 0;
    for (int i = 0; i < 15; i++) {
        t += DET_NS;
        HandObservation o = handAt(-0.1f, 1.2f, -0.45f, true, t);
        f.onDetections(t, 1, &o);
    }
    float out[21][3];
    for (int i = 0; i < 3; i++) {
        t += DET_NS;
        HandObservation o = handAt(-0.1f, 1.2f, -0.45f, true, t);
        for (int c = 0; c < 3; c++)
            o.points[4][c] = o.points[8][c];   // thumb tip onto index tip
        f.onDetections(t, 1, &o);
    }
    f.predict(0, t, out);
    float d[3];
    sub(out[4], out[8], d);
    float gap = std::sqrt(dot(d, d));
    printf("    pinch after 3 detections: tip gap %.1f mm\n", gap * 1000);
    check("pinch: tips within 15 mm after 3 detections", gap < 0.015f ? 1.f : 0.f, 1, 0);
}


// A hand at (x, y, z) pointing along `yawDeg` (0 = straight ahead, + = to the right).
static HandObservation pointingHand(float x, float y, float z, float yawDeg, bool label, int64_t t,
                                    const float head[3]) {
    HandObservation o = handAt(x, y, z, label, t);
    float a = yawDeg * (float) M_PI / 180, c = std::cos(a), s = std::sin(a);
    float base[21][3];
    rightHandPalmDown(base);
    for (int i = 0; i < 21; i++) {   // rotate the shape about the wrist, around +y
        float bx = base[i][0], bz = base[i][2];
        o.points[i][0] = x + bx * c - bz * s;
        o.points[i][2] = z + bx * s + bz * c;
    }
    AlvrQuat q = {0, 0, 0, 1};
    o.leftScore = leftHandScore(head, q, o.points[MP_WRIST], o.points[MP_MIDDLE_MCP]);
    return o;
}

static void testGeometricHandedness() {
    float head[3] = {0, 1.6f, 0};
    AlvrQuat q = {0, 0, 0, 1};
    // Scores for typical poses (hands ~45 cm in front, chest height).
    HandObservation lo = pointingHand(-0.18f, 1.35f, -0.45f, 15, false, 0, head);
    HandObservation ro = pointingHand(0.12f, 1.35f, -0.45f, -15, true, 0, head);
    check("handed: left hand left of head, pointing in -> left", lo.leftScore > 0.5f ? 1.f : 0.f, 1, 0);
    check("handed: right hand right of head, pointing in -> right", ro.leftScore < -0.5f ? 1.f : 0.f, 1, 0);
    (void) q;

    // Replay of the failed test: only the LEFT hand, ~20 cm left of the head, MediaPipe's label
    // random. Every frame must drive the left track, never the right.
    HandFilter f;
    std::mt19937 rng(11);
    std::uniform_int_distribution<int> coin(0, 1);
    std::normal_distribution<float> n(0, 0.03f);
    int leftHits = 0, rightHits = 0;
    for (int i = 0; i < 150; i++) {
        int64_t t = i * DET_NS;
        HandObservation o =
            pointingHand(-0.20f + n(rng), 1.35f + n(rng), -0.50f, 10 + 10 * n(rng), coin(rng), t, head);
        f.onDetections(t, 1, &o);
        float out[21][3];
        if (f.predict(0, t, out))
            leftHits++;
        if (f.predict(1, t, out))
            rightHits++;
    }
    check("handed: lone left hand, random labels -> always left", (float) leftHits, 150, 0);
    check("handed: lone left hand -> never a right hand", (float) rightHits, 0, 0);

    // Both hands at once, labels swapped: the more-left one is left.
    HandFilter g;
    for (int i = 0; i < 20; i++) {
        int64_t t = i * DET_NS;
        HandObservation two[2] = {pointingHand(0.12f, 1.35f, -0.45f, -15, true, t, head),
                                  pointingHand(-0.18f, 1.35f, -0.45f, 15, false, t, head)};
        g.onDetections(t, 2, two);
    }
    float l[21][3], r[21][3];
    bool okL = g.predict(0, 19 * DET_NS, l), okR = g.predict(1, 19 * DET_NS, r);
    check("handed: two hands -> left track holds the left hand (wrist x)",
          okL ? l[MP_WRIST][0] : 99.f,
          -0.18f,
          0.02f);
    check("handed: two hands -> right track holds the right hand (wrist x)",
          okR ? r[MP_WRIST][0] : 99.f,
          0.12f,
          0.02f);
}

static void testFilterRejectsDepthOutlier() {
    HandFilter f;
    for (int i = 0; i < 10; i++) {
        int64_t t = i * DET_NS;
        HandObservation o = handAt(0, 1.2f, -0.45f, true, t);
        f.onDetections(t, 1, &o);
    }
    int64_t t = 10 * DET_NS;
    HandObservation bad = handAt(0, 1.2f, -0.95f, true, t);   // depth suddenly 50 cm off
    f.onDetections(t, 1, &bad);
    float out[21][3];
    f.predict(0, t, out);
    // The depth part of one step is capped (5 cm) and smoothed slowly: a glitch barely moves it.
    check("outlier: 50 cm depth jump barely moves the wrist (z)", out[0][2], -0.45f, 0.015f);
}

static void testGestureDistances() {
    float mp[21][3];
    rightHandPalmDown(mp);
    AlvrPose s[26];
    toOpenXrSkeleton(mp, false, s);
    GestureDistances open = measureGestures(s);
    // Pinch: move the index tip onto the thumb tip.
    for (int k = 0; k < 3; k++)
        mp[8][k] = mp[4][k] + (k == 0 ? 0.015f : 0);
    toOpenXrSkeleton(mp, false, s);
    GestureDistances pinched = measureGestures(s);
    check("gesture: open hand pinch distance (m)", open.pinchM, 0.093f, 0.01f);
    check("gesture: pinched distance 15 mm", pinched.pinchM, 0.015f, 1e-4f);
    check("gesture: open hand grip distance > 7 cm", open.gripM > 0.07f ? 1.f : 0.f, 1, 0);
}

static void testAimRay() {
    // Head at (0, 1.6, 0) facing -z. Right hand straight out in front of the right shoulder:
    // the ray must point straight ahead and the controller stay upright.
    AimFilter a;
    float head[3] = {0, 1.6f, 0};
    AlvrQuat q = {0, 0, 0, 1};
    float hand[3] = {0.17f, 1.38f, -0.55f};
    AlvrQuat o = a.update(head, q, hand, false, 0);
    float x[3], y[3], z[3];
    axes(o, x, y, z);
    check("aim: straight-out arm points forward (-Z . -z)", z[2], 1, 0.01f);
    check("aim: controller upright (+Y . up)", y[1], 1, 0.01f);

    // Hand up and to the left of the right shoulder: ray points up-left.
    a.reset();
    float hand2[3] = {-0.1f, 1.7f, -0.5f};
    o = a.update(head, q, hand2, false, 0);
    axes(o, x, y, z);
    check("aim: hand up-left -> ray has upward part", -z[1] > 0.3f ? 1.f : 0.f, 1, 0);
    check("aim: hand up-left -> ray has leftward part", -z[0] < -0.2f ? 1.f : 0.f, 1, 0);

    // Nodding the head must not move the ray (shoulder follows heading only).
    a.reset();
    AlvrQuat nod = axisAngle(1, 0, 0, -30);
    AlvrQuat o1 = a.update(head, q, hand, false, 0);
    a.reset();
    AlvrQuat o2 = a.update(head, nod, hand, false, 0);
    check("aim: nodding doesn't swing the ray", quatDot(o1, o2), 1, 1e-4f);

    // The real chain on device: MediaPipe detections at ~15/s with 8 mm noise -> HandFilter
    // (smooth + predict) -> AimFilter at the 60 Hz send rate.
    auto chain = [&](auto handAtTime, float *maxStepDeg, float *finalYawDeg) {
        HandFilter hf;
        AimFilter af;
        std::mt19937 rng(9);
        std::normal_distribution<float> n(0, 0.008f);
        AlvrQuat prev = {0, 0, 0, 1};
        *maxStepDeg = 0;
        int64_t nextDet = 0;
        for (int i = 0; i < 120; i++) {   // 2 s at 60 Hz
            int64_t t = (int64_t) (i * 1e9 / 60);
            if (t >= nextDet) {
                float p[3];
                handAtTime(t * 1e-9, p);
                HandObservation o = handAt(p[0] + n(rng), p[1] + n(rng), p[2] + n(rng), false, t);
                hf.onDetections(t, 1, &o);
                nextDet += DET_NS;
            }
            float pts[21][3];
            if (!hf.predict(1, t + 50'000'000, pts))
                continue;
            AlvrQuat cur = af.update(head, q, pts[MP_INDEX_MCP], false, t * 1e-9);
            if (i > 30)
                *maxStepDeg = std::fmax(
                    *maxStepDeg,
                    fusion::angleOf(fusion::normalize(fusion::mul(cur, fusion::conj(prev)))) *
                        180 / (float) M_PI);
            prev = cur;
        }
        const float *d = af.direction();
        *finalYawDeg = std::atan2(d[0], -d[2]) * 180 / (float) M_PI;
    };
    float jit, yaw;
    chain([&](double, float p[3]) { p[0] = 0.17f; p[1] = 1.38f; p[2] = -0.55f; }, &jit, &yaw);
    printf("    aim chain: still hand, max ray step %.2f deg/frame\n", jit);
    check("aim chain: still hand -> ray step < 0.5 deg/frame", jit < 0.5f ? 1.f : 0.f, 1, 0);

    // A deliberate 30 deg swing over 0.5 s, then held: the ray must arrive by the end.
    float baseYaw;
    chain([&](double, float p[3]) { p[0] = 0.17f; p[1] = 1.38f; p[2] = -0.55f; }, &jit, &baseYaw);
    chain(
        [&](double t, float p[3]) {
            float s = (float) std::fmin(1.0, std::fmax(0.0, (t - 0.5) / 0.5));
            float ang = 30.f * s * (float) M_PI / 180;
            p[0] = 0.17f + 0.55f * std::sin(ang);
            p[1] = 1.38f;
            p[2] = -0.55f * std::cos(ang);
        },
        &jit,
        &yaw);
    // Expected: the ray runs shoulder -> knuckles (index MCP), not through the swing's pivot.
    float shape[21][3];
    rightHandPalmDown(shape);
    auto rayYaw = [&](float wx, float wz) {
        float kx = wx + shape[MP_INDEX_MCP][0], kz = wz + shape[MP_INDEX_MCP][2];
        float sx = 0.17f, sz = 0.08f;   // right shoulder for a head at the origin facing -z
        return std::atan2(kx - sx, -(kz - sz)) * 180 / (float) M_PI;
    };
    float ang = 30.f * (float) M_PI / 180;
    float expected = rayYaw(0.17f + 0.55f * std::sin(ang), -0.55f * std::cos(ang)) -
                     rayYaw(0.17f, -0.55f);
    check("aim chain: follows a swing to its end (deg change, 1 s after)",
          yaw - baseYaw,
          expected,
          1.5f);
}


int main() {
    testLoneHandReacquiredAcrossCentre();
    testLeftOnlyMode();
    testDepthNoiseSuppressed();
    testPinchClosesQuickly();
    testImageAnchoredPinch();
    testPinchFromAnyAngle();
    testAimRay();
    testGestureDistances();
    testTranslationSolve();
    testSkeleton();
    testFilterSmoothsStillHand();
    testFilterPredictsMovingHand();
    testFilterHoldsThenDrops();
    testFilterStickyHandedness();
    testNoDoubleHandAcrossGaps();
    testDuplicateDetectionSameFrame();
    testTwoRealHandsStayTwo();
    testSwitchingHandsInSameSpot();
    testGeometricHandedness();
    testFilterRejectsDepthOutlier();
    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
