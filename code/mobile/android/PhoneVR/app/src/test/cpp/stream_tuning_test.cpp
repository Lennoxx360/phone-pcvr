// Host-side tests for app/src/main/cpp/stream_tuning.h (not part of the Gradle build).
//
// Run from code/mobile/android/PhoneVR (needs the ALVR header from a client-lib build):
//   clang++ -std=c++17 -I app/src/main/cpp -I ALVR/build/alvr_client_core \
//       app/src/test/cpp/stream_tuning_test.cpp -o stream_tuning_test && ./stream_tuning_test

#define _USE_MATH_DEFINES
#include "stream_tuning.h"
#include <cmath>
#include <cstdio>

using namespace tuning;

static int fails = 0;

static void check(const char *name, float got, float want, float tol) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok)
        fails++;
    printf("%-52s got %+9.4f want %+9.4f  %s\n", name, got, want, ok ? "ok" : "FAIL");
}

static AlvrQuat yaw(float deg) {
    float a = deg * (float) M_PI / 180 / 2;
    return {0, std::sin(a), 0, std::cos(a)};
}

static const float D = (float) M_PI / 180;

static void testExpandAndCrop() {
    AlvrFov lens = {-50 * D, 45 * D, 48 * D, -52 * D};   // left, right, up, down
    AlvrFov wide = expandFov(lens, 5 * D);
    check("expand: left", wide.left / D, -55, 1e-3f);
    check("expand: down", wide.down / D, -57, 1e-3f);
    // Crop of the wider image back to the lens: left edge at the lens's left tangent.
    UvRect uv = cropUv(wide, lens);
    float tl = std::tan(lens.left), sl = std::tan(wide.left), sr = std::tan(wide.right);
    check("crop: u0", uv.u0, (tl - sl) / (sr - sl), 1e-5f);
    check("crop: inside 0..1", (uv.u0 > 0 && uv.u1 < 1 && uv.v0 > 0 && uv.v1 < 1) ? 1.f : 0.f, 1, 0);
    UvRect same = cropUv(lens, lens);
    check("crop: no overscan u0", same.u0, 0, 1e-6f);
    check("crop: no overscan v1", same.v1, 1, 1e-6f);
    AlvrFov huge = expandFov(lens, 60 * D);
    check("expand: clamped below 90 deg", huge.right < 1.5f ? 1.f : 0.f, 1, 0);
}

static void testDampedPrediction() {
    DampedPrediction d;
    const int64_t dt = 16'666'667;
    int64_t t = 0;
    // Still: full prediction (nothing to overshoot, and no noise amplification).
    float k = 1;
    for (int i = 0; i < 30; i++)
        k = d.update(t += dt, yaw(0));
    check("still head keeps full prediction", k, 1, 1e-6f);

    // Steady 120 deg/s turn: full prediction.
    float a = 0;
    for (int i = 0; i < 30; i++)
        k = d.update(t += dt, yaw(a += 2.0f));
    check("steady turn keeps full prediction", k, 1, 0.02f);
    check("turn rate measured", d.rateDegS(), 120, 3);

    // Abrupt slow-down to 30 deg/s: prediction cut hard (rate/peak)^2 ~ 0.06 within a few frames.
    for (int i = 0; i < 4; i++)
        k = d.update(t += dt, yaw(a += 0.5f));
    check("slowing turn damps prediction", k < 0.3f ? 1.f : 0.f, 1, 0);

    // Holding the slower speed: the peak decays, prediction recovers.
    for (int i = 0; i < 40; i++)
        k = d.update(t += dt, yaw(a += 0.5f));
    check("steady again: prediction recovers", k, 1, 0.05f);

    // Speeding up never damps.
    for (int i = 0; i < 10; i++)
        k = d.update(t += dt, yaw(a += 0.5f + i * 0.3f));
    check("speeding up keeps full prediction", k, 1, 0.05f);
}

static void testCap() {
    check("cap off passes through", (float) capLookahead(118'000'000, 0) / 1e6f, 118, 1e-3f);
    check("cap limits spike", (float) capLookahead(118'000'000, 60'000'000) / 1e6f, 60, 1e-3f);
    check("cap keeps normal value", (float) capLookahead(40'000'000, 60'000'000) / 1e6f, 40, 1e-3f);
}

static void testSlerp() {
    AlvrQuat h = slerp(yaw(0), yaw(40), 0.5f);
    check("slerp halfway", qangle(yaw(0), h) / D, 20, 1e-2f);
    check("qangle", qangle(yaw(10), yaw(-15)) / D, 25, 1e-2f);
}

static void testComfortGuard() {
    ComfortGuard g;
    const int64_t dt = 16'666'667;
    int64_t t = 0;
    bool on = false;
    for (int i = 0; i < 60; i++)
        on = g.onFrame(t += dt, 15);
    check("guard: on-time frames stay off", on ? 1.f : 0.f, 0, 0);
    // One late spike (a single 400 ms frame) must not trip it.
    on = g.onFrame(t += dt, 400);
    for (int i = 0; i < 5; i++)
        on = g.onFrame(t += dt, 15);
    check("guard: single spike ignored", on ? 1.f : 0.f, 0, 0);
    // A backlog (seconds late) trips it within a few frames.
    int frames = 0;
    while (!g.onFrame(t += dt, 4000) && frames < 60)
        frames++;
    check("guard: backlog trips within 5 frames", frames <= 5 ? 1.f : 0.f, 1, 0);
    // Recovering: stays on until lateness is low for a full second.
    for (int i = 0; i < 20; i++)
        on = g.onFrame(t += dt, 15);
    check("guard: holds right after recovery", on ? 1.f : 0.f, 1, 0);
    for (int i = 0; i < 70; i++)
        on = g.onFrame(t += dt, 15);
    check("guard: releases after 1 s of good frames", on ? 1.f : 0.f, 0, 0);
    check("guard: counted one entry", (float) g.entries(), 1, 0);
}

static void testPlanarReproject() {
    const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float zero[3] = {0, 0, 0};
    float t[2];
    // No offset: identical to rotation-only for any plane distance.
    const float d1[3] = {0.3f, -0.2f, -1};
    planarReproject(I, zero, 2.0f, d1, t);
    check("planar: no offset = rotation-only (x)", t[0], 0.3f, 1e-6f);
    check("planar: no offset = rotation-only (y)", t[1], -0.2f, 1e-6f);
    // Eye moved 10 cm right: straight ahead now sees the plane point 10 cm right of centre.
    const float right[3] = {0.1f, 0, 0}, ahead[3] = {0, 0, -1};
    planarReproject(I, right, 2.0f, ahead, t);
    check("planar: 10 cm right, plane 2 m -> tan +0.05", t[0], 0.05f, 1e-6f);
    // Eye moved 50 cm forward toward a 2 m plane: things look bigger (display tan 0.3 -> 0.225).
    const float fwd[3] = {0, 0, -0.5f}, d2[3] = {0.3f, 0, -1};
    planarReproject(I, fwd, 2.0f, d2, t);
    check("planar: 50 cm forward magnifies", t[0], 0.225f, 1e-6f);
    // Plane distance 0 = rotation only, offset ignored.
    planarReproject(I, right, 0.0f, ahead, t);
    check("planar: depth 0 = rotation only", t[0], 0, 1e-6f);
    // Ray pointing backwards fails cleanly.
    const float back[3] = {0, 0, 1};
    check("planar: backward ray rejected", planarReproject(I, zero, 2.0f, back, t) ? 1.f : 0.f, 0, 0);
}

int main() {
    testComfortGuard();
    testPlanarReproject();
    testExpandAndCrop();
    testDampedPrediction();
    testCap();
    testSlerp();
    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
