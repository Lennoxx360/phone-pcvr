#ifndef PHONEVR_STREAM_TUNING_H
#define PHONEVR_STREAM_TUNING_H

// Small, host-testable pieces of the latency/artifact work (2026-11-22):
//  - FOV overscan: the PC renders a slightly wider image so timewarp has pixels to rotate into
//    instead of black (or stretched) edges;
//  - UV crop: shows the display part of an overscanned image when timewarp is off;
//  - damped rotation prediction: shortens the timewarp's gyro extrapolation while the head is
//    slowing down, so stopping a turn doesn't overshoot ("the view runs ahead of my head");
//  - prediction caps: bounds how far ahead poses are extrapolated when ALVR's latency estimate
//    spikes on a bad network.

#include "alvr_client_core.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tuning {

// Widens every edge of fov by marginRad (angles; left/down are negative).
inline AlvrFov expandFov(AlvrFov fov, float marginRad) {
    const float maxHalf = 1.45f;   // ~83 deg: keep tan() finite
    AlvrFov out = fov;
    out.left = std::max(fov.left - marginRad, -maxHalf);
    out.right = std::min(fov.right + marginRad, maxHalf);
    out.down = std::max(fov.down - marginRad, -maxHalf);
    out.up = std::min(fov.up + marginRad, maxHalf);
    return out;
}

struct UvRect {
    float u0, u1, v0, v1;   // u0 = left edge, v0 = bottom edge, in the source image's 0..1 range
};

// The part of an image rendered with sourceFov that covers displayFov (same orientation).
inline UvRect cropUv(AlvrFov sourceFov, AlvrFov displayFov) {
    float sl = std::tan(sourceFov.left), sr = std::tan(sourceFov.right);
    float sd = std::tan(sourceFov.down), su = std::tan(sourceFov.up);
    float dl = std::tan(displayFov.left), dr = std::tan(displayFov.right);
    float dd = std::tan(displayFov.down), du = std::tan(displayFov.up);
    UvRect r;
    r.u0 = (dl - sl) / (sr - sl);
    r.u1 = (dr - sl) / (sr - sl);
    r.v0 = (dd - sd) / (su - sd);
    r.v1 = (du - sd) / (su - sd);
    return r;
}

// Unit quaternion helpers (x, y, z, w).
inline AlvrQuat qmul(AlvrQuat a, AlvrQuat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline float qangle(AlvrQuat a, AlvrQuat b) {   // radians between two orientations
    AlvrQuat d = qmul({-a.x, -a.y, -a.z, a.w}, b);
    return 2 * std::acos(std::fmin(1.f, std::fabs(d.w)));
}

inline AlvrQuat slerp(AlvrQuat a, AlvrQuat b, float t) {
    float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0) {
        b = {-b.x, -b.y, -b.z, -b.w};
        dot = -dot;
    }
    if (dot > 0.9995f) {
        AlvrQuat r = {a.x + t * (b.x - a.x),
                      a.y + t * (b.y - a.y),
                      a.z + t * (b.z - a.z),
                      a.w + t * (b.w - a.w)};
        float n = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
        return {r.x / n, r.y / n, r.z / n, r.w / n};
    }
    float th = std::acos(dot), s = std::sin(th);
    float ka = std::sin((1 - t) * th) / s, kb = std::sin(t * th) / s;
    return {ka * a.x + kb * b.x, ka * a.y + kb * b.y, ka * a.z + kb * b.z, ka * a.w + kb * b.w};
}

// Tracks the head turn rate from the current (unpredicted) gyro orientation sampled once per
// render, and says how much of the gyro extrapolation to keep. While the turn rate is falling the
// extrapolation (which assumes constant speed) overshoots, so it is scaled by
// (rate now / recent peak rate)^2. Steady turns and speed-ups keep 100 %.
class DampedPrediction {
  public:
    // Returns 0..1.
    float update(int64_t tNs, AlvrQuat orientationNow) {
        if (haveLast_ && tNs > lastT_) {
            float dt = (tNs - lastT_) * 1e-9f;
            if (dt > 0.002f && dt < 0.1f) {
                float rate = qangle(last_, orientationNow) / dt;   // rad/s
                rate_ += 0.5f * (rate - rate_);                     // light smoothing
                // Peak decays with ~120 ms time constant so a slow-down is detected quickly
                // but a stop-start doesn't keep damping forever.
                peak_ = std::max(rate_, peak_ * std::exp(-dt / 0.12f));
            }
        }
        last_ = orientationNow;
        lastT_ = tNs;
        haveLast_ = true;
        if (peak_ < kMinRate)
            return scale_ = 1.f;   // nearly still: nothing to overshoot
        float k = std::clamp(rate_ / peak_, 0.f, 1.f);
        return scale_ = k * k;
    }

    float rateDegS() const { return rate_ * 57.2958f; }
    float scale() const { return scale_; }
    void reset() {
        haveLast_ = false;
        rate_ = peak_ = 0;
        scale_ = 1;
    }

  private:
    static constexpr float kMinRate = 0.35f;   // ~20 deg/s
    AlvrQuat last_ = {0, 0, 0, 1};
    int64_t lastT_ = 0;
    bool haveLast_ = false;
    float rate_ = 0, peak_ = 0, scale_ = 1;
};

// ALVR's lookahead (its average total pipeline latency) balloons during network stalls: 118 ms
// was logged on 2026-11-22. Poses extrapolated that far overshoot whenever the head changes
// direction. capNs <= 0 disables the cap.
inline int64_t capLookahead(int64_t alvrLookaheadNs, int64_t capNs) {
    if (capNs <= 0)
        return alvrLookaheadNs;
    return std::min(alvrLookaheadNs, capNs);
}

// Positional timewarp (planar reprojection). Rotation-only timewarp can't show position changes
// (walking, leaning, the eye shift of a head tilt) until a new PC frame arrives. Without depth,
// assume the scene sits on a plane planeDistM in front of the render eye: cast the display ray
// from where the eye is now (`offset`, render-eye frame, metres) and look up where it hits that
// plane in the rendered image. Exact for the plane, approximate nearer/farther (parallax error
// ~ offset * (1/z - 1/planeDist)). Mirrors the timewarp.h shader; M is column-major
// displayToRender. Returns false when the ray doesn't hit the plane in front.
inline bool planarReproject(
    const float M[9], const float offset[3], float planeDistM, const float d[3], float tanOut[2]) {
    float r[3];
    for (int i = 0; i < 3; i++)
        r[i] = M[i] * d[0] + M[3 + i] * d[1] + M[6 + i] * d[2];
    if (r[2] > -1e-4f)
        return false;
    if (planeDistM <= 0) {   // rotation only
        tanOut[0] = r[0] / -r[2];
        tanOut[1] = r[1] / -r[2];
        return true;
    }
    float s = (-planeDistM - offset[2]) / r[2];
    float q[3] = {offset[0] + s * r[0], offset[1] + s * r[1], offset[2] + s * r[2]};
    if (s <= 0 || q[2] > -1e-3f)
        return false;
    tanOut[0] = q[0] / -q[2];
    tanOut[1] = q[1] / -q[2];
    return true;
}

// Largest eye offset positional timewarp applies; beyond this the frame's pose is too far off
// (e.g. a stall) and a big planar shift would look worse than none.
constexpr float kMaxPositionalOffsetM = 0.25f;

// Comfort guard. On a link that can't keep up, the streamer queues frames and they arrive
// seconds late (4-10 s on 2026-11-22). Timewarp then swings a seconds-old world by up to 77 deg
// to follow the head, which made the user sick. While frames are badly late, the app shows the
// head-tracked lobby with an explanation instead.
// Enter: smoothed lateness above enterMs. Leave: below exitMs continuously for exitHoldNs.
class ComfortGuard {
  public:
    float enterMs = 300.f;
    float exitMs = 120.f;
    int64_t exitHoldNs = 1'000'000'000;

    // One new frame, lateMs = now - its target display time. Returns active().
    bool onFrame(int64_t nowNs, double lateMs) {
        lateEmaMs_ = haveLate_ ? lateEmaMs_ + 0.3 * (lateMs - lateEmaMs_) : lateMs;
        haveLate_ = true;
        if (!active_) {
            if (lateEmaMs_ > enterMs) {
                active_ = true;
                goodSinceNs_ = -1;
                entries_++;
            }
        } else if (lateEmaMs_ < exitMs) {
            if (goodSinceNs_ < 0)
                goodSinceNs_ = nowNs;
            else if (nowNs - goodSinceNs_ >= exitHoldNs)
                active_ = false;
        } else {
            goodSinceNs_ = -1;
        }
        return active_;
    }

    bool active() const { return active_; }
    double lateMs() const { return lateEmaMs_; }
    int entries() const { return entries_; }
    void reset() {
        active_ = haveLate_ = false;
        lateEmaMs_ = 0;
        goodSinceNs_ = -1;
    }

  private:
    bool active_ = false, haveLate_ = false;
    double lateEmaMs_ = 0;
    int64_t goodSinceNs_ = -1;
    int entries_ = 0;
};

}   // namespace tuning

#endif   // PHONEVR_STREAM_TUNING_H
