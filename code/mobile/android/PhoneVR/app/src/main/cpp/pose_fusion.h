#ifndef PHONEVR_POSE_FUSION_H
#define PHONEVR_POSE_FUSION_H

// Fuses the Cardboard gyro orientation (high rate, drifts in yaw) with the ARCore camera pose
// (~30 Hz, drift-free, gives position) into one head pose for ALVR.
//
// Frames: both trackers report "world from head" in the OpenGL convention (x right, y up,
// looking down -z, metres), with y aligned to gravity. Their worlds therefore differ only by a
// rotation about y (yaw). The output is expressed in the ARCore world:
//  - position: ARCore head position, smoothed and extrapolated to the display time with a
//    One Euro filter (heavy smoothing when still, little lag when moving);
//  - orientation: the gyro orientation (already predicted by Cardboard) rotated by the filtered
//    yaw offset between the two worlds. Re-estimating that offset on every ARCore frame also
//    removes gyro yaw drift.
// ARCore sometimes re-localises and snaps its whole world (seen on device: 31 deg + 0.8 m in one
// frame). Such jumps are absorbed into an "anchor" (yaw + translation from the ARCore world to
// the output world) so the user's view stays continuous instead of teleporting or swinging.

#include "alvr_client_core.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>

namespace fusion {

inline AlvrQuat mul(AlvrQuat a, AlvrQuat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline AlvrQuat conj(AlvrQuat q) { return {-q.x, -q.y, -q.z, q.w}; }

inline AlvrQuat normalize(AlvrQuat q) {
    float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-9f)
        return {0, 0, 0, 1};
    return {q.x / n, q.y / n, q.z / n, q.w / n};
}

inline AlvrQuat yawQuat(float rad) { return {0, std::sin(rad / 2), 0, std::cos(rad / 2)}; }

// out = q * v * q^-1
inline void rotate(AlvrQuat q, const float v[3], float out[3]) {
    float r[3] = {q.x, q.y, q.z};
    float rv[3] = {r[1] * v[2] - r[2] * v[1], r[2] * v[0] - r[0] * v[2], r[0] * v[1] - r[1] * v[0]};
    float rrv[3] = {
        r[1] * rv[2] - r[2] * rv[1], r[2] * rv[0] - r[0] * rv[2], r[0] * rv[1] - r[1] * rv[0]};
    for (int i = 0; i < 3; i++)
        out[i] = v[i] + 2 * (q.w * rv[i] + rrv[i]);
}

inline float wrapPi(float a) {
    while (a > (float) M_PI)
        a -= 2 * (float) M_PI;
    while (a < -(float) M_PI)
        a += 2 * (float) M_PI;
    return a;
}

// Rotation angle of q in radians, 0..pi.
inline float angleOf(AlvrQuat q) { return 2 * std::acos(std::fmin(1.f, std::fabs(q.w))); }

// Swing-twist split about y: the yaw part of q, in radians.
inline float twistYaw(AlvrQuat q) { return wrapPi(2 * std::atan2(q.y, q.w)); }

// Angle of the non-yaw (tilt) part of q, in degrees. ~0 when two gravity-aligned frames agree.
inline float tiltDeg(AlvrQuat q) {
    AlvrQuat twist = normalize({0, q.y, 0, q.w});
    AlvrQuat swing = mul(q, conj(twist));
    return angleOf(swing) * 180.f / (float) M_PI;
}

// World-frame angular velocity (rad/s) that takes orientation a to b in dt seconds.
inline void angularVelocity(AlvrQuat a, AlvrQuat b, float dt, float out[3]) {
    AlvrQuat d = normalize(mul(b, conj(a)));
    if (d.w < 0)
        d = {-d.x, -d.y, -d.z, -d.w};
    float s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    float angle = 2 * std::atan2(s, d.w);
    for (int i = 0; i < 3; i++)
        out[i] = (s > 1e-9f && dt > 0) ? (&d.x)[i] / s * angle / dt : 0.f;
}

// One Euro filter (Casiez et al. 2012) on a 3D point, also yielding a smoothed velocity.
// Unlike the textbook version, velocity comes from consecutive *raw* samples (the textbook
// derivative against the filtered value over-reads by lag/dt at constant speed), and lagS()
// reports the filter's current time lag so a predictor can add v * lagS() back.
class OneEuro3 {
  public:
    float minCutoffHz = 1.0f;   // smoothing when still: lower = smoother, more lag
    float beta = 4.0f;          // how fast the cutoff rises with speed (per m/s)
    float derivCutoffHz = 2.0f; // smoothing of the velocity estimate

    void reset() { initialized_ = false; }

    void update(const float x[3], double t) {
        if (!initialized_) {
            for (int i = 0; i < 3; i++) {
                value_[i] = x[i];
                prevRaw_[i] = x[i];
                velocity_[i] = 0;
            }
            lastT_ = t;
            lagS_ = 0;
            initialized_ = true;
            return;
        }
        float dt = (float) (t - lastT_);
        if (dt <= 1e-4f)
            return;
        lastT_ = t;
        float aD = alpha(derivCutoffHz, dt);
        float speed = 0;
        for (int i = 0; i < 3; i++) {
            float rawVel = (x[i] - prevRaw_[i]) / dt;
            prevRaw_[i] = x[i];
            velocity_[i] += aD * (rawVel - velocity_[i]);
            speed += velocity_[i] * velocity_[i];
        }
        float cutoff = minCutoffHz + beta * std::sqrt(speed);
        float a = alpha(cutoff, dt);
        for (int i = 0; i < 3; i++)
            value_[i] += a * (x[i] - value_[i]);
        // Steady-state lag of this discrete low-pass behind a ramp: dt * (1 - a) / a.
        lagS_ = dt * (1 - a) / a;
    }

    float lagS() const { return lagS_; }

    bool initialized() const { return initialized_; }
    const float *value() const { return value_; }
    const float *velocity() const { return velocity_; }
    double time() const { return lastT_; }
    void zeroVelocity() {
        for (float &v : velocity_)
            v = 0;
    }

  private:
    static float alpha(float cutoffHz, float dt) {
        float tau = 1.f / (2 * (float) M_PI * cutoffHz);
        return 1.f / (1.f + tau / dt);
    }
    bool initialized_ = false;
    float value_[3] = {0, 0, 0};
    float prevRaw_[3] = {0, 0, 0};
    float velocity_[3] = {0, 0, 0};
    double lastT_ = 0;
    float lagS_ = 0;
};

struct Config {
    // Phone camera position relative to the head centre (between the eyes), in head coordinates
    // (x right, y up, -z forward), metres. Depends on the shell.
    // Calibrated for a Galaxy S24 Ultra in a Gear VR shell (tools/calibrate_pivot.py on a
    // 24 s head-rotation recording): camera at (-0.045, +0.105, -0.137) m from the neck pivot,
    // i.e. this value + eyeFromNeck. The camera is 4.5 cm left of centre.
    float cameraInHead[3] = {-0.045f, 0.030f, -0.057f};
    // Neck model: eye centre relative to the neck pivot the head rotates about, head coordinates.
    // ARCore tracks the neck (which barely moves when you tilt or turn), and the eye position is
    // rebuilt from it with the gyro's full-rate, predicted rotation. Without this, every head
    // tilt is an eye translation that ARCore sees late at 30 Hz and prediction overshoots.
    float eyeFromNeck[3] = {0.f, 0.075f, -0.08f};
    // Per-ARCore-frame blend toward the newly measured yaw offset (~30 Hz -> ~1.7 s time constant).
    float yawFilterAlpha = 0.02f;
    // Above this head turn rate the yaw offset is not updated: a few ms of timestamp mismatch
    // between camera and gyro would bias it.
    float yawUpdateMaxTurnRateDegS = 45.f;
    // Upper bound on how far ahead position is extrapolated.
    float maxPredictionS = 0.15f;
    // Prediction fades in with speed: none below the low bound (the velocity estimate is mostly
    // ARCore noise there, and extrapolating it would add jitter), full above the high bound.
    float predictionFadeInSpeed[2] = {0.06f, 0.20f};   // m/s
    // Prediction also fades out with head turn rate: during fast rotation most apparent position
    // motion is lever-arm error (camera/neck offsets not exactly right), not real translation.
    float predictionFadeOutTurnDegS[2] = {60.f, 180.f};
    // Send linear/angular velocity to ALVR. Off: the pose is already predicted to the display
    // time, and with it on head tilts were badly jittery on device (suspected double prediction
    // or a frame mismatch in SteamVR); stock PhoneVR never sent them either.
    bool sendVelocities = false;
    // ARCore world-jump detection. A frame-to-frame head step above reanchorStepM (3 m/s at
    // 30 fps) is treated as a jump, not motion. A yaw-offset mismatch above reanchorYawDeg that
    // lasts reanchorYawFrames frames while turning slowly is a world rotation, not gyro drift.
    float reanchorStepM = 0.10f;
    float reanchorYawDeg = 6.f;
    int reanchorYawFrames = 3;
    // ARCore position is ignored for this long after tracking (re)starts: on device it drifted
    // ~1.2 m in the first seconds before correcting itself.
    float settleS = 3.f;
    // Absorbed position jumps are bled back toward ARCore's (corrected) world so a jump that was
    // ARCore fixing its own drift can never leave the user permanently displaced. Rate =
    // base + fraction of the current head speed, so most of it hides inside real motion.
    float bleedBaseMps = 0.015f;
    float bleedSpeedFraction = 0.15f;
    // At most this much absorbed offset is bled back; anything beyond it is kept permanently.
    // ARCore's own drift fixes are centimetres to tens of centimetres. Multi-metre jumps happen
    // when it relocalises (e.g. the camera was covered while the phone went into the headset),
    // and bleeding those made the view slide through the world for minutes
    // (8 m absorbed). The play-space origin is arbitrary, so keeping a big offset costs nothing.
    float bleedMaxM = 0.3f;
};

struct FusedPose {
    AlvrPose pose;
    float linearVelocity[3];
    float angularVelocity[3];
};

class PoseFusion {
  public:
    Config config;
    OneEuro3 positionFilter;

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        haveYaw_ = false;
        tracking_ = false;
        yaw_ = 0;
        positionFilter.reset();
        haveLastGyro_ = false;
        anchorYaw_ = 0;
        for (int i = 0; i < 3; i++)
            anchorT_[i] = absorbedT_[i] = 0;
        haveArHead_ = false;
        yawDevFrames_ = 0;
        trackingSinceT_ = -1;
        holdUntilT_ = 0;
        absorbNextStep_ = false;
        haveBleedT_ = false;
    }

    // One ARCore camera frame, captured at timestampNs (CLOCK_BOOTTIME). arPosition/arOrientation:
    // ARCore display-oriented camera pose (ARCore world from camera). gyroAtFrame: Cardboard
    // world-from-head orientation at the same moment.
    void onArCoreFrame(int64_t timestampNs,
                       bool tracking,
                       const float arPosition[3],
                       AlvrQuat arOrientation,
                       AlvrQuat gyroAtFrame) {
        std::lock_guard<std::mutex> lock(mutex_);
        double t = timestampNs * 1e-9;

        float turnRateDegS = 0;
        if (haveLastGyro_ && t > lastGyroT_) {
            float w[3];
            angularVelocity(lastGyro_, gyroAtFrame, (float) (t - lastGyroT_), w);
            turnRateDegS =
                std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * 180.f / (float) M_PI;
        }
        lastGyro_ = gyroAtFrame;
        lastGyroT_ = t;
        haveLastGyro_ = true;
        lastTurnRateDegS_ = turnRateDegS;

        if (!tracking)
            trackingSinceT_ = -1;
        else if (trackingSinceT_ < 0)
            trackingSinceT_ = t;
        bool settling = tracking && t - trackingSinceT_ < config.settleS;
        bool held = t < holdUntilT_;
        holdState_ = !tracking ? 1 : settling ? 2 : held ? 3 : 0;

        tracking_ = tracking && !settling && !held;
        if (!tracking_) {
            // Freeze position (no extrapolation) and yaw offset; gyro keeps rotating (3DoF).
            // Whatever ARCore claims happened meanwhile is absorbed when it becomes usable again.
            positionFilter.zeroVelocity();
            absorbNextStep_ = true;
            return;
        }

        arOrientation = normalize(arOrientation);

        // Neck pivot in the ARCore world = camera position minus the rotated camera->neck offset.
        // (Named headAr below: it is the tracked point everything downstream filters.)
        float camFromNeck[3], camOffsetWorld[3], headAr[3];
        for (int i = 0; i < 3; i++)
            camFromNeck[i] = config.cameraInHead[i] + config.eyeFromNeck[i];
        rotate(arOrientation, camFromNeck, camOffsetWorld);
        for (int i = 0; i < 3; i++)
            headAr[i] = arPosition[i] - camOffsetWorld[i];

        // Position jump, or the first usable frame after a hold/settle/loss: absorb it.
        if (haveArHead_) {
            float d[3], step = 0;
            for (int i = 0; i < 3; i++) {
                d[i] = headAr[i] - prevArHead_[i];
                step += d[i] * d[i];
            }
            if (absorbNextStep_ || std::sqrt(step) > config.reanchorStepM) {
                float dOut[3];
                rotate(yawQuat(anchorYaw_), d, dOut);
                for (int i = 0; i < 3; i++)
                    absorbedT_[i] -= dOut[i];
                positionReanchors_++;
                // Move whatever exceeds the bleed budget into the permanent anchor. The output
                // doesn't change: toOutput() adds both.
                float mag = std::sqrt(absorbedT_[0] * absorbedT_[0] +
                                      absorbedT_[1] * absorbedT_[1] +
                                      absorbedT_[2] * absorbedT_[2]);
                if (mag > config.bleedMaxM) {
                    float k = config.bleedMaxM / mag;
                    for (int i = 0; i < 3; i++) {
                        float keep = absorbedT_[i] * k;
                        anchorT_[i] += absorbedT_[i] - keep;
                        absorbedT_[i] = keep;
                    }
                }
            }
        }
        absorbNextStep_ = false;
        for (int i = 0; i < 3; i++)
            prevArHead_[i] = headAr[i];
        haveArHead_ = true;

        // Bleed absorbed jumps back toward ARCore's world.
        if (haveBleedT_) {
            float dt = (float) std::min(0.1, t - lastBleedT_);
            const float *v = positionFilter.velocity();
            float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            float maxStep = (config.bleedBaseMps + config.bleedSpeedFraction * speed) * dt;
            float mag = std::sqrt(absorbedT_[0] * absorbedT_[0] + absorbedT_[1] * absorbedT_[1] +
                                  absorbedT_[2] * absorbedT_[2]);
            if (mag > 1e-6f) {
                float k = std::max(0.f, mag - maxStep) / mag;
                for (float &a : absorbedT_)
                    a *= k;
            }
        }
        lastBleedT_ = t;
        haveBleedT_ = true;

        // Maps the gyro world into the ARCore world.
        AlvrQuat offset = normalize(mul(arOrientation, conj(gyroAtFrame)));
        lastYawMeas_ = twistYaw(offset);
        lastTiltDeg_ = tiltDeg(offset);
        if (!haveYaw_) {
            yaw_ = lastYawMeas_;
            haveYaw_ = true;
        } else if (turnRateDegS <= config.yawUpdateMaxTurnRateDegS) {
            float dev = wrapPi(lastYawMeas_ - yaw_);
            if (std::fabs(dev) * 180.f / (float) M_PI > config.reanchorYawDeg) {
                if (++yawDevFrames_ >= config.reanchorYawFrames) {
                    reanchorYaw(lastYawMeas_, headAr);
                    yawDevFrames_ = 0;
                }
            } else {
                yawDevFrames_ = 0;
                yaw_ = wrapPi(yaw_ + config.yawFilterAlpha * dev);
                yawUpdates_++;
            }
        } else {
            yawSkips_++;
        }

        // Head centre in the output world; everything downstream works there.
        float head[3];
        toOutput(headAr, head);

        // Jitter stats: RMS of frame-to-frame steps while (nearly) still, raw vs filtered.
        float before[3] = {0, 0, 0};
        bool hadFilter = positionFilter.initialized();
        if (hadFilter)
            for (int i = 0; i < 3; i++)
                before[i] = positionFilter.value()[i];
        positionFilter.update(head, t);
        if (hadFilter && haveRaw_) {
            const float *v = positionFilter.velocity();
            float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (speed < 0.05f) {
                float rawStep = 0, outStep = 0;
                for (int i = 0; i < 3; i++) {
                    rawStep += (head[i] - lastRaw_[i]) * (head[i] - lastRaw_[i]);
                    float d = positionFilter.value()[i] - before[i];
                    outStep += d * d;
                }
                const float k = 0.05f;
                rawJitterMs2_ += k * (rawStep - rawJitterMs2_);
                outJitterMs2_ += k * (outStep - outJitterMs2_);
            }
        }
        for (int i = 0; i < 3; i++)
            lastRaw_[i] = head[i];
        haveRaw_ = true;
    }

    // Fused head pose (ARCore world) at targetTimestampNs (CLOCK_BOOTTIME), given the gyro
    // orientation predicted for that time and the one dtS seconds earlier (for angular velocity).
    // Latency breakdown for the status line: how old ARCore frames are when they arrive, and how
    // far ahead ALVR asks us to predict.
    void noteCameraAge(int64_t ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        cameraAgeS_ += 0.1f * (ns * 1e-9f - cameraAgeS_);
    }
    void noteAlvrLookahead(int64_t ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        alvrLookaheadS_ += 0.1f * (ns * 1e-9f - alvrLookaheadS_);
    }

    FusedPose fuse(int64_t targetTimestampNs, AlvrQuat gyro, AlvrQuat gyroEarlier, float dtS) {
        std::lock_guard<std::mutex> lock(mutex_);
        FusedPose out = {};
        AlvrQuat yawQ = haveYaw_ ? yawQuat(anchorYaw_ + yaw_) : AlvrQuat{0, 0, 0, 1};
        out.pose.orientation = normalize(mul(yawQ, gyro));

        if (positionFilter.initialized()) {
            float horizon = 0, g = 0;
            predictPosition(targetTimestampNs, out.pose.orientation, out.pose.position, &horizon, &g);
            lastHorizonS_ = horizon;
            lastPredictionGain_ = g;
            if (config.sendVelocities) {
                const float *v = positionFilter.velocity();
                for (int i = 0; i < 3; i++)
                    out.linearVelocity[i] = v[i] * g;
            }
        }

        if (config.sendVelocities && dtS > 0) {
            float w[3];
            angularVelocity(gyroEarlier, gyro, dtS, w);
            rotate(yawQ, w, out.angularVelocity);   // gyro world -> output world
        }

        // Remember the pose before the play-space reset, then apply it.
        for (int i = 0; i < 3; i++)
            lastPreRecenterPos_[i] = out.pose.position[i];
        lastPreRecenterYaw_ = twistYaw(out.pose.orientation);
        AlvrQuat r = yawQuat(recenterYaw_);
        float local[3];
        for (int i = 0; i < 3; i++)
            local[i] = out.pose.position[i] - recenterPos_[i];
        rotate(r, local, out.pose.position);
        out.pose.orientation = normalize(mul(r, out.pose.orientation));
        float tmp[3];
        for (int i = 0; i < 3; i++)
            tmp[i] = out.linearVelocity[i];
        rotate(r, tmp, out.linearVelocity);
        for (int i = 0; i < 3; i++)
            tmp[i] = out.angularVelocity[i];
        rotate(r, tmp, out.angularVelocity);
        return out;
    }

    // Head (eye-centre) pose in the output world at targetTimestampNs for a gyro orientation,
    // exactly as fuse() would send it, but without touching any stats or the recentre reference.
    // Used by positional timewarp every refresh. Returns false before ARCore's first pose.
    bool poseFor(int64_t targetTimestampNs, AlvrQuat gyro, AlvrPose &out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!positionFilter.initialized())
            return false;
        AlvrQuat yawQ = haveYaw_ ? yawQuat(anchorYaw_ + yaw_) : AlvrQuat{0, 0, 0, 1};
        AlvrQuat orientation = normalize(mul(yawQ, gyro));
        float pre[3];
        predictPosition(targetTimestampNs, orientation, pre, nullptr, nullptr);
        AlvrQuat r = yawQuat(recenterYaw_);
        float local[3];
        for (int i = 0; i < 3; i++)
            local[i] = pre[i] - recenterPos_[i];
        rotate(r, local, out.position);
        out.orientation = normalize(mul(r, orientation));
        return true;
    }

    // Why position is (not) being tracked: 0 tracking, 1 ARCore lost, 2 settling after
    // (re)acquire, 3 held because a hand is close; -1 before ARCore's first pose. Any thread.
    int holdState() {
        std::lock_guard<std::mutex> lock(mutex_);
        return positionFilter.initialized() || holdState_ != 0 ? holdState_ : -1;
    }

    // Upper bound on position extrapolation (seconds). Any thread.
    void setMaxPrediction(float s) {
        std::lock_guard<std::mutex> lock(mutex_);
        config.maxPredictionS = std::clamp(s, 0.f, 0.25f);
    }

    // Reset play space: the current head position (horizontally) becomes the origin and the
    // current heading becomes forward. Height is kept. Uses the last fused pose.
    void recenter() {
        std::lock_guard<std::mutex> lock(mutex_);
        recenterYaw_ = -lastPreRecenterYaw_;
        recenterPos_[0] = lastPreRecenterPos_[0];
        recenterPos_[1] = 0;
        recenterPos_[2] = lastPreRecenterPos_[2];
        recenters_++;
    }

    // Don't trust ARCore position until untilNs (CLOCK_BOOTTIME): a hand close to the camera
    // fills the image and drags ARCore's tracking along with it.
    void holdPosition(int64_t untilNs) {
        std::lock_guard<std::mutex> lock(mutex_);
        holdUntilT_ = std::max(holdUntilT_, untilNs * 1e-9);
    }

    // ARCore world point -> output world (the frame head poses are sent in), e.g. for hands.
    void arToOutput(const float ar[3], float out[3]) {
        std::lock_guard<std::mutex> lock(mutex_);
        float pre[3];
        toOutput(ar, pre);
        for (int i = 0; i < 3; i++)
            pre[i] -= recenterPos_[i];
        rotate(yawQuat(recenterYaw_), pre, out);
    }

    // Output-world orientation for a gyro orientation, without touching any prediction state or
    // stats (used by timewarp several times per frame).
    AlvrQuat orientationFor(AlvrQuat gyro) {
        std::lock_guard<std::mutex> lock(mutex_);
        AlvrQuat yawQ = haveYaw_ ? yawQuat(anchorYaw_ + yaw_) : AlvrQuat{0, 0, 0, 1};
        return normalize(mul(yawQuat(recenterYaw_), mul(yawQ, gyro)));
    }

    void status(char *buf, size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        const float deg = 180.f / (float) M_PI;
        if (!positionFilter.initialized()) {
            snprintf(buf, len, "6DoF: waiting for ARCore (3DoF now)");
            return;
        }
        const float *p = positionFilter.value();
        const float *v = positionFilter.velocity();
        snprintf(buf,
                 len,
                 "6DoF %s  yaw ofs %+.1f (meas %+.1f)  tilt err %.1f%s\n"
                 "neck %+.2f %+.2f %+.2f m  v %.2f m/s  turn %.0f deg/s\n"
                 "pred %.0f ms (cam %.0f + alvr %.0f) x%.1f  jitter raw %.1f / out %.1f mm  yaw upd %u skip %u\n"
                 "re-anchor yaw %u pos %u  offset %.2f m",
                 holdState_ == 0   ? "ON"
                 : holdState_ == 1 ? "LOST - position frozen"
                 : holdState_ == 2 ? "SETTLING - position frozen"
                                   : "HAND CLOSE - position frozen",
                 yaw_ * deg,
                 lastYawMeas_ * deg,
                 lastTiltDeg_,
                 lastTiltDeg_ > 5.f ? " (HIGH)" : "",
                 p[0],
                 p[1],
                 p[2],
                 std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]),
                 lastTurnRateDegS_,
                 lastHorizonS_ * 1000.f,
                 cameraAgeS_ * 1000.f,
                 alvrLookaheadS_ * 1000.f,
                 lastPredictionGain_,
                 std::sqrt(rawJitterMs2_) * 1000.f,
                 std::sqrt(outJitterMs2_) * 1000.f,
                 (unsigned) yawUpdates_,
                 (unsigned) yawSkips_,
                 (unsigned) yawReanchors_,
                 (unsigned) positionReanchors_,
                 std::sqrt(absorbedT_[0] * absorbedT_[0] + absorbedT_[1] * absorbedT_[1] +
                           absorbedT_[2] * absorbedT_[2]));
    }

  private:
    // Predicted eye-centre position (pre-recentre output world) at targetTimestampNs: neck from
    // the filtered ARCore track, extrapolated (speed- and turn-gated), plus the neck->eye offset
    // rotated by `orientation`. Caller holds mutex_ and has checked positionFilter.initialized().
    void predictPosition(int64_t targetTimestampNs,
                         AlvrQuat orientation,
                         float out[3],
                         float *horizonOut,
                         float *gainOut) const {
        const float *p = positionFilter.value();
        const float *v = positionFilter.velocity();
        float horizon = (float) (targetTimestampNs * 1e-9 - positionFilter.time());
        if (horizonOut)
            *horizonOut = horizon;
        // Extrapolate from the last camera frame to the target time, plus the filter's own lag.
        horizon = std::clamp(horizon + positionFilter.lagS(), 0.f, config.maxPredictionS);
        float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        float lo = config.predictionFadeInSpeed[0], hi = config.predictionFadeInSpeed[1];
        float g = std::clamp((speed - lo) / (hi - lo), 0.f, 1.f);
        g = g * g * (3 - 2 * g);   // smoothstep
        float tlo = config.predictionFadeOutTurnDegS[0], thi = config.predictionFadeOutTurnDegS[1];
        float tg = std::clamp((lastTurnRateDegS_ - tlo) / (thi - tlo), 0.f, 1.f);
        g *= 1 - tg * tg * (3 - 2 * tg);
        if (!tracking_)
            g = 0;
        if (gainOut)
            *gainOut = g;
        // Predicted neck + eye offset rotated by the (gyro, already predicted) head rotation.
        float eyeOffset[3];
        rotate(orientation, config.eyeFromNeck, eyeOffset);
        for (int i = 0; i < 3; i++)
            out[i] = p[i] + v[i] * horizon * g + eyeOffset[i];
    }

    // ARCore world -> output world. anchorT_ keeps the view continuous across yaw re-anchors;
    // absorbedT_ holds absorbed position jumps and is bled back to zero over time.
    void toOutput(const float ar[3], float out[3]) const {
        rotate(yawQuat(anchorYaw_), ar, out);
        for (int i = 0; i < 3; i++)
            out[i] += anchorT_[i] + absorbedT_[i];
    }

    // The ARCore world rotated: snap the yaw offset to the new measurement and move the anchor
    // so that neither the output orientation nor the output head position changes.
    void reanchorYaw(float newYaw, const float headAr[3]) {
        // Keep the raw (unfiltered) output position where it is, so the filter sees no step.
        float continuity[3];
        toOutput(headAr, continuity);
        anchorYaw_ = wrapPi(anchorYaw_ + yaw_ - newYaw);
        yaw_ = newYaw;
        float rotated[3];
        rotate(yawQuat(anchorYaw_), headAr, rotated);
        for (int i = 0; i < 3; i++)
            anchorT_[i] = continuity[i] - rotated[i] - absorbedT_[i];
        yawReanchors_++;
    }

    std::mutex mutex_;
    float anchorYaw_ = 0;
    float anchorT_[3] = {0, 0, 0};
    float absorbedT_[3] = {0, 0, 0};
    // Play-space reset, applied last: out = R(recenterYaw_) * (pre - recenterPos_).
    float recenterYaw_ = 0;
    float recenterPos_[3] = {0, 0, 0};
    float lastPreRecenterPos_[3] = {0, 0, 0};
    float lastPreRecenterYaw_ = 0;
    uint32_t recenters_ = 0;
    bool absorbNextStep_ = false;
    double trackingSinceT_ = -1;
    double holdUntilT_ = 0;
    int holdState_ = 0;   // 0 tracking, 1 ARCore lost, 2 settling, 3 hand hold
    double lastBleedT_ = 0;
    bool haveBleedT_ = false;
    float prevArHead_[3] = {0, 0, 0};
    bool haveArHead_ = false;
    int yawDevFrames_ = 0;
    uint32_t yawReanchors_ = 0;
    uint32_t positionReanchors_ = 0;
    bool haveYaw_ = false;
    bool tracking_ = false;
    float yaw_ = 0;
    float lastYawMeas_ = 0;
    float lastTiltDeg_ = 0;
    float lastTurnRateDegS_ = 0;
    float lastHorizonS_ = 0;
    float cameraAgeS_ = 0;
    float alvrLookaheadS_ = 0;
    float lastPredictionGain_ = 0;
    AlvrQuat lastGyro_ = {0, 0, 0, 1};
    double lastGyroT_ = 0;
    bool haveLastGyro_ = false;
    float lastRaw_[3] = {0, 0, 0};
    bool haveRaw_ = false;
    float rawJitterMs2_ = 0;
    float outJitterMs2_ = 0;
    uint32_t yawUpdates_ = 0;
    uint32_t yawSkips_ = 0;
};

}   // namespace fusion

#endif   // PHONEVR_POSE_FUSION_H
