// Host-side tests for app/src/main/cpp/viewer_params.h (not part of the Gradle build).
//
// Run from code/mobile/android/PhoneVR:
//   clang++ -std=c++17 -I app/src/main/cpp app/src/test/cpp/viewer_params_test.cpp \
//       -o viewer_params_test && ./viewer_params_test

#include "viewer_params.h"
#include <cmath>
#include <cstdio>
#include <string>

using namespace viewer;

static int fails = 0;

static void check(const char *name, float got, float want, float tol = 1e-5f) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok)
        fails++;
    printf("%-40s got %+10.5f want %+10.5f  %s\n", name, got, want, ok ? "ok" : "FAIL");
}

// Builds a DeviceParams like a real profile: vendor/model strings, floats, packed FOV,
// distortion coefficients and enums, in proto field order.
static std::vector<uint8_t> makeProfile(int alignment, bool packedFov = true, bool withTray = true) {
    using namespace detail;
    std::vector<uint8_t> out;
    auto str = [&](int field, const std::string &s) {
        writeVarint(out, (uint64_t(field) << 3) | 2);
        writeVarint(out, s.size());
        out.insert(out.end(), s.begin(), s.end());
    };
    auto packed = [&](int field, std::initializer_list<float> fs) {
        writeVarint(out, (uint64_t(field) << 3) | 2);
        writeVarint(out, fs.size() * 4);
        for (float f : fs) {
            uint8_t b[4];
            memcpy(b, &f, 4);
            out.insert(out.end(), b, b + 4);
        }
    };
    str(1, "Samsung");
    str(2, "Gear VR");
    writeFloatField(out, 3, 0.039f);
    writeFloatField(out, 4, 0.062f);
    if (packedFov) {
        packed(5, {50, 50, 50, 50});
    } else {
        for (int i = 0; i < 4; i++)
            writeFloatField(out, 5, 50);
    }
    if (withTray)
        writeFloatField(out, 6, 0.035f);
    packed(7, {0.21f, 0.04f});
    writeVarint(out, 11u << 3);
    writeVarint(out, alignment);
    writeVarint(out, 12u << 3);
    writeVarint(out, 3);   // primary_button = INDIRECT_TOUCH, must survive
    return out;
}

static Info roundTrip(const std::vector<uint8_t> &in, const Adjust &a, float screenH = 0.07f) {
    std::vector<uint8_t> out;
    Info base, after;
    if (!apply(in.data(), int(in.size()), a, screenH, out, &base) ||
        !parse(out.data(), int(out.size()), after)) {
        fails++;
        printf("apply/parse failed\n");
    }
    return after;
}

int main() {
    auto bottom = makeProfile(0);

    Info i = roundTrip(bottom, Adjust{});
    check("identity: screen_to_lens", i.screenToLens, 0.039f);
    check("identity: inter_lens", i.interLens, 0.062f);
    check("identity: tray", i.trayToLens, 0.035f);
    check("identity: fov[3]", i.fov[3], 50);
    check("identity: alignment", i.alignment, 0);

    Adjust a;
    a.ipdDeltaM = 0.0015f;
    a.sizeScale = 1.1f;
    a.fovScale = 1.2f;
    a.verticalM = 0.002f;
    i = roundTrip(bottom, a);
    check("ipd +1.5 mm", i.interLens, 0.0635f);
    check("size x1.1", i.screenToLens, 0.0429f);
    check("fov x1.2", i.fov[0], 60);
    check("bottom tray +2 mm", i.trayToLens, 0.037f);

    auto top = makeProfile(2);
    i = roundTrip(top, a);
    check("top tray moves the other way", i.trayToLens, 0.033f);
    check("top alignment kept", i.alignment, 2);

    auto center = makeProfile(1, true, false);
    i = roundTrip(center, a, 0.07f);
    check("center -> bottom", i.alignment, 0);
    check("center tray = h/2 + border + 2 mm", i.trayToLens, 0.035f + 0.003f + 0.002f);
    i = roundTrip(center, Adjust{}, 0.07f);
    check("center untouched without offset", i.alignment, 1);

    auto unpacked = makeProfile(0, false);
    i = roundTrip(unpacked, Adjust{});
    check("unpacked fov read", i.fov[2], 50);

    Adjust extreme;
    extreme.ipdDeltaM = 1;
    extreme.fovScale = 5;
    i = roundTrip(bottom, extreme);
    check("ipd clamped", i.interLens, 0.1f);
    check("fov clamped", i.fov[1], 85);

    // Untouched fields survive (vendor string, distortion, primary button).
    std::vector<uint8_t> out;
    apply(bottom.data(), int(bottom.size()), a, 0.07f, out);
    std::string s(out.begin(), out.end());
    check("vendor kept", s.find("Gear VR") != std::string::npos, 1);
    check("button kept", s.find(std::string("\x60\x03", 2)) != std::string::npos, 1);

    uint8_t junk[] = {0x0a, 0x50, 0x01};   // length runs past the end
    Info bad;
    check("malformed rejected", parse(junk, sizeof(junk), bad), 0);

    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
