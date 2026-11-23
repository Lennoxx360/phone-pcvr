#ifndef PHONEVR_PINCH_H
#define PHONEVR_PINCH_H

// Phone-side pinch (trigger) and grip decisions with hysteresis and debounce.
//
// ALVR's server-side gesture emulation turns the thumb-index distance into an analog trigger that
// starts rising at 6.4 cm (centre to centre) and has only a 50 ms on-delay, so a relaxed hand kept
// firing triggers. Here a button goes down only when the distance stays below
// pressBelowM for pressHoldNs, and up only when it stays above releaseAboveM for releaseHoldNs.
// Decisions are made on real camera observations only: a hand that is merely being held/coasted
// after leaving the camera's view releases instead of clicking.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace pinch {

class Hysteresis {
  public:
    // VRChat testing: held pinches read 3-30 mm, relaxed hands 30-80 mm, and a release
    // at 45 mm kept a press held ~0.8 s after the fingers had opened to ~40 mm.
    float pressBelowM = 0.020f;         // firm pinch to click (was 25 mm: trigger happy)
    float releaseAboveM = 0.038f;       // let go above 38 mm for releaseHoldNs...
    float releaseNowAboveM = 0.055f;    // ...or at once when clearly open
    int64_t pressHoldNs = 50'000'000;   // 2 frames at 17 Hz, 3 at 30 Hz
    int64_t releaseHoldNs = 50'000'000; // one noisy frame above 38 mm doesn't drop the hold
    // Frames where MediaPipe misses the hand (common mid-pinch) don't release a held pinch
    // until they've lasted this long.
    int64_t missGraceNs = 250'000'000;

    // One observation at tNs. valid = false (no fresh observation of the hand in this frame)
    // releases once misses have lasted missGraceNs.
    bool update(int64_t tNs, float distanceM, bool valid) {
        if (!valid) {
            since_ = -1;
            if (!down_)
                return down_;
            if (missSince_ < 0)
                missSince_ = tNs;
            if (tNs - missSince_ >= missGraceNs) {
                down_ = false;
                missSince_ = -1;
            }
            return down_;
        }
        missSince_ = -1;
        if (down_ && distanceM > releaseNowAboveM) {
            down_ = false;
            since_ = -1;
            return down_;
        }
        bool wantsChange = down_ ? distanceM > releaseAboveM : distanceM < pressBelowM;
        if (!wantsChange) {
            since_ = -1;
            return down_;
        }
        if (since_ < 0)
            since_ = tNs;
        if (tNs - since_ >= (down_ ? releaseHoldNs : pressHoldNs)) {
            down_ = !down_;
            since_ = -1;
            presses_ += down_ ? 1 : 0;
        }
        return down_;
    }

    void release() {
        down_ = false;
        since_ = -1;
        missSince_ = -1;
    }
    bool down() const { return down_; }
    int presses() const { return presses_; }

  private:
    bool down_ = false;
    int64_t since_ = -1;
    int64_t missSince_ = -1;
    int presses_ = 0;
};

// Air joystick: pinch and hold, then move the hand = thumbstick (for walking with hand tracking).
// The pinch point where the pinch started is the stick's centre. The hand's offset from it,
// horizontally in the head's yaw frame (x right, y forward), gives the stick: nothing inside
// deadzoneM (tracking jitter), full tilt at fullM. A quick pinch that never left the deadzone
// (released within tapMaxNs) is a tap: the caller sends a short trigger click instead.
class AirJoystick {
  public:
    float deadzoneM = 0.02f;
    float fullM = 0.12f;
    int64_t tapMaxNs = 350'000'000;
    int64_t anchorNs = 100'000'000;   // centre = average over the first 100 ms of the pinch
    float smoothS = 0.12f;            // pinch-point smoothing time constant
    // Gentle start: stick = curve^curvePower (small hand moves = slow walk), and the stick
    // can grow by at most rampPerS per second (0 -> full in ~0.6 s). Slowing down is instant.
    float curvePower = 1.6f;
    float rampPerS = 1.7f;

    struct Out {
        float x = 0, y = 0;     // stick, each -1..1, length <= 1
        bool active = false;    // pinch held (stick touched)
        bool tap = false;       // true once, on the release of a tap
    };

    // pinchPoint: output world (metres, y up). headYawRad: head yaw about +y (0 = facing -z).
    Out update(int64_t tNs, bool pinchDown, const float pinchPoint[3], float headYawRad) {
        Out o;
        if (!pinchDown) {
            if (held_ && !moved_ && tNs - startNs_ <= tapMaxNs)
                o.tap = true;
            held_ = false;
            return o;
        }
        if (!held_) {
            held_ = true;
            moved_ = false;
            mag_ = 0;
            startNs_ = lastNs_ = tNs;
            anchorSamples_ = 0;
            for (int i = 0; i < 3; i++)
                anchor_[i] = smooth_[i] = pinchPoint[i];
        }
        o.active = true;
        // Time-based smoothing of the pinch point (tracking jitter). Moving back towards the
        // centre is smoothed 4x less, so stopping is quick while speeding up stays gentle.
        float dt = (tNs - lastNs_) * 1e-9f;
        lastNs_ = tNs;
        float rawOff2 = 0, smoothOff2 = 0;
        for (int i = 0; i < 3; i += 2) {   // horizontal (x, z)
            rawOff2 += (pinchPoint[i] - anchor_[i]) * (pinchPoint[i] - anchor_[i]);
            smoothOff2 += (smooth_[i] - anchor_[i]) * (smooth_[i] - anchor_[i]);
        }
        bool returning = tNs - startNs_ > anchorNs && rawOff2 < smoothOff2;
        float tau = returning ? smoothS * 0.25f : smoothS;
        float a = dt > 0 ? 1.f - std::exp(-dt / tau) : 0.f;
        for (int i = 0; i < 3; i++)
            smooth_[i] += a * (pinchPoint[i] - smooth_[i]);
        // The centre is the average over the first anchorS of the pinch, not one noisy sample.
        if (tNs - startNs_ <= anchorNs) {
            anchorSamples_++;
            for (int i = 0; i < 3; i++)
                anchor_[i] += (smooth_[i] - anchor_[i]) / anchorSamples_;
            return o;
        }
        // World offset -> head yaw frame: right = (cos, -sin), forward = (-sin, -cos) in (x, z).
        float dx = smooth_[0] - anchor_[0], dz = smooth_[2] - anchor_[2];
        float c = std::cos(headYawRad), s = std::sin(headYawRad);
        float right = dx * c - dz * s;
        float fwd = -dx * s - dz * c;
        float len = std::sqrt(right * right + fwd * fwd);
        if (len > deadzoneM)
            moved_ = true;
        float want = 0;
        if (moved_ && len > deadzoneM)
            want = std::pow(std::min(1.f, (len - deadzoneM) / (fullM - deadzoneM)), curvePower);
        mag_ = want < mag_ ? want : std::min(want, mag_ + rampPerS * dt);
        if (mag_ <= 0)
            return o;
        float mag = mag_;
        o.x = right / len * mag;
        o.y = fwd / len * mag;
        return o;
    }

    bool held() const { return held_; }

  private:
    bool held_ = false, moved_ = false;
    int64_t startNs_ = 0, lastNs_ = 0;
    int anchorSamples_ = 0;
    float mag_ = 0;
    float anchor_[3] = {0, 0, 0}, smooth_[3] = {0, 0, 0};
};

}   // namespace pinch

#endif   // PHONEVR_PINCH_H
