// Host-side tests for app/src/main/cpp/pinch.h (not part of the Gradle build).
//
// Run from code/mobile/android/PhoneVR:
//   clang++ -std=c++17 -I app/src/main/cpp app/src/test/cpp/pinch_test.cpp -o pinch_test && ./pinch_test

#include "pinch.h"
#include <cmath>
#include <cstdio>
#include <random>

using namespace pinch;

static int fails = 0;

static void check(const char *name, float got, float want, float tol) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok)
        fails++;
    printf("%-52s got %+9.4f want %+9.4f  %s\n", name, got, want, ok ? "ok" : "FAIL");
}

static const int64_t DT = 33'333'333;   // ~30 hand updates/s

int main() {
    {
        // Relaxed hand with tracking noise around 4-6 cm: never clicks.
        Hysteresis h;
        std::mt19937 rng(1);
        std::normal_distribution<float> n(0.050f, 0.008f);
        int64_t t = 0;
        for (int i = 0; i < 3000; i++)
            h.update(t += DT, n(rng), true);
        check("relaxed hand (5 cm +- 8 mm, 100 s): no clicks", (float) h.presses(), 0, 0);
    }
    {
        // Single noisy frames below the press threshold don't click (debounce).
        Hysteresis h;
        int64_t t = 0;
        for (int i = 0; i < 300; i++)
            h.update(t += DT, (i % 10 == 0) ? 0.015f : 0.045f, true);
        check("one-frame dips to 15 mm: no clicks", (float) h.presses(), 0, 0);
    }
    {
        // A real pinch: down after ~35 ms below 2.5 cm, stays down through jitter up to 4 cm,
        // up once clearly open.
        Hysteresis h;
        int64_t t = 0;
        bool d = false;
        for (int i = 0; i < 3; i++)
            d = h.update(t += DT, 0.012f, true);
        check("pinch held 100 ms: down", d ? 1.f : 0.f, 1, 0);
        for (int i = 0; i < 30; i++)
            d = h.update(t += DT, (i % 2) ? 0.035f : 0.018f, true);
        check("jitter 18-35 mm while pinched: stays down", d ? 1.f : 0.f, 1, 0);
        for (int i = 0; i < 3; i++)
            d = h.update(t += DT, 0.06f, true);
        check("opened to 6 cm: up", d ? 1.f : 0.f, 0, 0);
        check("exactly one click", (float) h.presses(), 1, 0);
    }
    {
        // Letting go: one frame at 40 mm while held (noise) keeps it; two frames at 40 mm
        // release; clearly open (6 cm) releases on the first frame.
        Hysteresis h;
        int64_t t = 0;
        bool d = false;
        for (int i = 0; i < 3; i++)
            d = h.update(t += DT, 0.008f, true);
        d = h.update(t += DT, 0.040f, true);
        d = h.update(t += DT, 0.010f, true);
        check("one noisy frame at 40 mm: still held", d ? 1.f : 0.f, 1, 0);
        d = h.update(t += DT, 0.040f, true);
        d = h.update(t += DT, 0.041f, true);
        d = h.update(t += DT, 0.042f, true);
        check("opened to ~40 mm for 66 ms: released", d ? 1.f : 0.f, 0, 0);
        for (int i = 0; i < 3; i++)
            d = h.update(t += DT, 0.008f, true);
        d = h.update(t += DT, 0.060f, true);
        check("opened to 6 cm: released on the first frame", d ? 1.f : 0.f, 0, 0);
        h = Hysteresis();
        for (int i = 0; i < 10; i++)
            d = h.update(t += DT, 0.023f, true);
        check("loose pinch at 23 mm: no click (firm pinch needed)", d ? 1.f : 0.f, 0, 0);
    }
    {
        // Hand leaves the camera while pinched: released, not stuck or re-clicked.
        Hysteresis h;
        int64_t t = 0;
        for (int i = 0; i < 4; i++)
            h.update(t += DT, 0.01f, true);
        bool d = h.update(t += DT, 0.01f, false);
        check("one missed frame while pinched: still down", d ? 1.f : 0.f, 1, 0);
        for (int i = 0; i < 4; i++)
            d = h.update(t += DT, 0.01f, false);
        check("hand missing 160 ms mid-pinch: held", d ? 1.f : 0.f, 1, 0);
        for (int i = 0; i < 6; i++)
            d = h.update(t += DT, 0.01f, false);
        check("hand missing 330 ms: released", d ? 1.f : 0.f, 0, 0);
        check("no second click", (float) h.presses(), 1, 0);
    }
    {
        // 17 hand frames/s (as measured on device): a pinch clicks on its 2nd frame.
        Hysteresis h;
        int64_t t = 0;
        const int64_t DT17 = 58'000'000;
        bool d = h.update(t += DT17, 0.008f, true);
        check("17 Hz: first frame touching: not yet", d ? 1.f : 0.f, 0, 0);
        d = h.update(t += DT17, 0.009f, true);
        check("17 Hz: second frame touching: down", d ? 1.f : 0.f, 1, 0);
    }
    {
        // Air joystick, facing -z (yaw 0). Hold still for the 100 ms centring window, then move
        // and let the smoothing and ramp settle.
        AirJoystick j;
        float p[3] = {0, 1.2f, -0.4f};
        int64_t t = 0;
        AirJoystick::Out o;
        for (int i = 0; i < 5; i++)
            o = j.update(t += DT, true, p, 0);
        check("joystick: centring window -> no tilt", std::fabs(o.x) + std::fabs(o.y), 0, 1e-6f);
        p[2] -= 0.07f;
        for (int i = 0; i < 40; i++)
            o = j.update(t += DT, true, p, 0);
        // (7 - 2) / (12 - 2) = 0.5 of the way, ^1.6 curve: a gentle walk.
        check("joystick: 7 cm forward -> y 0.33 (gentle curve)", o.y, 0.33f, 0.01f);
        check("joystick: 7 cm forward -> x 0", o.x, 0, 1e-4f);
        p[2] -= 0.20f;
        float y300 = 0;
        for (int i = 0; i < 40; i++) {
            o = j.update(t += DT, true, p, 0);
            if (i == 8)
                y300 = o.y;
        }
        check("joystick: speeds up gradually (< 0.9 after 0.3 s)", y300 < 0.9f ? 1.f : 0.f, 1, 0);
        check("joystick: far forward clamps to 1", o.y, 1, 1e-4f);
        p[2] += 0.27f;   // back to the centre
        o = j.update(t += DT, true, p, 0);
        float y1 = o.y;
        for (int i = 0; i < 8; i++)
            o = j.update(t += DT, true, p, 0);
        check("joystick: slows down at once (first frame drops)", y1 < 0.95f ? 1.f : 0.f, 1, 0);
        check("joystick: back at centre -> stopped within 0.3 s", o.y, 0, 1e-4f);
        o = j.update(t += DT, false, p, 0);
        check("joystick: release -> zero", std::fabs(o.x) + std::fabs(o.y), 0, 0);
        check("joystick: drag release is not a tap", o.tap ? 1.f : 0.f, 0, 0);
    }
    {
        // Facing +x (yaw -90 deg): moving the hand along +x is "forward".
        AirJoystick j;
        float p[3] = {0, 1.2f, 0};
        int64_t t = 0;
        AirJoystick::Out o;
        for (int i = 0; i < 5; i++)
            j.update(t += DT, true, p, (float) (-M_PI / 2));
        p[0] += 0.14f;
        for (int i = 0; i < 40; i++)
            o = j.update(t += DT, true, p, (float) (-M_PI / 2));
        check("joystick: head-relative forward (y)", o.y, 1, 1e-3f);
        check("joystick: head-relative forward (x)", o.x, 0, 1e-3f);
    }
    {
        // Jitter (6 mm per axis) never moves you; a quick still pinch is a tap.
        AirJoystick j;
        std::mt19937 rng(3);
        std::normal_distribution<float> n(0, 0.006f);
        int64_t t = 0;
        float maxMag = 0;
        for (int i = 0; i < 9; i++) {
            float p[3] = {n(rng), 1.2f, -0.4f + n(rng)};
            AirJoystick::Out o = j.update(t += DT, true, p, 0);
            maxMag = std::max(maxMag, std::fabs(o.x) + std::fabs(o.y));
        }
        check("joystick: 6 mm jitter -> no movement", maxMag, 0, 0);
        float p[3] = {0, 1.2f, -0.4f};
        AirJoystick::Out o = j.update(t += DT, false, p, 0);
        check("joystick: quick still pinch -> tap", o.tap ? 1.f : 0.f, 1, 0);
    }
    {
        // Long jitter test: 30 s of a held pinch with 6 mm jitter never walks.
        AirJoystick j;
        std::mt19937 rng(7);
        std::normal_distribution<float> n(0, 0.006f);
        int64_t t = 0;
        int moving = 0;
        for (int i = 0; i < 900; i++) {
            float p[3] = {n(rng), 1.2f, -0.4f + n(rng)};
            AirJoystick::Out o = j.update(t += DT, true, p, 0);
            if (std::fabs(o.x) + std::fabs(o.y) > 0) moving++;
        }
        check("joystick: 30 s held still with jitter -> never moves", (float) moving, 0, 0);
    }
    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
