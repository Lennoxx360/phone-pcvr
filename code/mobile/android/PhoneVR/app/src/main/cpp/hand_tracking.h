#ifndef PHONEVR_HAND_TRACKING_H
#define PHONEVR_HAND_TRACKING_H

// MediaPipe hand landmarks -> 3D hands -> ALVR (OpenXR) hand skeletons.
//
// MediaPipe gives, per hand, 21 normalized image landmarks and 21 metric "world" landmarks (the
// hand's shape in metres around its own centre, camera-aligned axes: x right, y down, z away from
// the camera). The hand's position is found by solving for the translation that makes the metric
// shape project onto the image landmarks through the camera intrinsics (linear least squares).
// Points then go camera -> ARCore world (ARCore camera pose) -> output world (pose fusion anchor),
// and are mapped onto the 26-joint XR_EXT_hand_tracking skeleton ALVR expects.

#include "alvr_client_core.h"
#include "pose_fusion.h"
#include <cmath>
#include <cstdint>

namespace hands {

struct Intrinsics {
    float fx, fy, cx, cy, width, height;
};

// MediaPipe landmark indices.
enum {
    MP_WRIST = 0,
    MP_THUMB_CMC = 1,
    MP_THUMB_IP = 3,
    MP_THUMB_TIP = 4,
    MP_INDEX_MCP = 5,
    MP_INDEX_DIP = 7,
    MP_INDEX_TIP = 8,
    MP_MIDDLE_MCP = 9,
    MP_PINKY_MCP = 17,
};

// Solves t (OpenCV camera frame) so that world[i] + t projects onto image[i] (normalized 0..1).
// Returns false if the system is degenerate or the hand would be behind the camera.
inline bool solveTranslation(const float world[21][3],
                             const float image[21][2],
                             const Intrinsics &k,
                             float t[3]) {
    double ata[3][3] = {}, atb[3] = {};
    for (int i = 0; i < 21; i++) {
        double a = (image[i][0] * k.width - k.cx) / k.fx;
        double b = (image[i][1] * k.height - k.cy) / k.fy;
        // (X + tx) - a (Z + tz) = 0   and   (Y + ty) - b (Z + tz) = 0
        double rows[2][4] = {{1, 0, -a, a * world[i][2] - world[i][0]},
                             {0, 1, -b, b * world[i][2] - world[i][1]}};
        for (auto &r : rows)
            for (int p = 0; p < 3; p++) {
                atb[p] += r[p] * r[3];
                for (int q = 0; q < 3; q++)
                    ata[p][q] += r[p] * r[q];
            }
    }
    // 3x3 solve by Cramer's rule.
    auto det3 = [](double m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    double d = det3(ata);
    if (std::fabs(d) < 1e-12)
        return false;
    for (int c = 0; c < 3; c++) {
        double m[3][3];
        for (int r = 0; r < 3; r++)
            for (int q = 0; q < 3; q++)
                m[r][q] = (q == c) ? atb[r] : ata[r][q];
        t[c] = (float) (det3(m) / d);
    }
    return t[2] > 0.05f && t[2] < 2.0f;
}

// OpenCV camera point (x right, y down, z forward) -> ARCore world, given the ARCore physical
// camera pose (OpenGL convention: x right, y up, looking down -z).
inline void cameraToArWorld(const float pCv[3],
                            const float camT[3],
                            AlvrQuat camQ,
                            float out[3]) {
    float pGl[3] = {pCv[0], -pCv[1], -pCv[2]};
    fusion::rotate(camQ, pGl, out);
    for (int i = 0; i < 3; i++)
        out[i] += camT[i];
}

// One detected hand, in the ARCore world.
struct HandObservation {
    bool valid = false;
    bool isLeft = false;      // MediaPipe's label: unreliable from a head-mounted camera
    float leftScore = 0;      // geometric handedness (leftHandScore): > 0 left, < 0 right
    int64_t captureNs = 0;
    float points[21][3] = {};
    float cameraDistanceM = 0;
    float cameraPos[3] = {0, 0, 0};   // ARCore world; the filter smooths depth along this ray
    // Pinch distance straight from this frame (no smoothing lag): the smaller of pinchImageM
    // and pinchShapeM. -1 = none.
    float pinchM = -1;
    float pinchImageM = -1;   // image-anchored joints, depth difference at 0.75 weight
    float pinchShapeM = -1;   // MediaPipe's own 3D hand shape (knows occluded fingers)
    float pinchTipM = -1;     // tips only (no pads), both views: used while making a fist
    float palmFacingDeg = 0;  // 0 = palm or back of the hand faces the camera, 90 = edge-on
};

// Weight of the depth difference in the image pinch distance (see HandObservation).
constexpr float PINCH_DEPTH_WEIGHT = 0.75f;

// Thumb-index pinch distance on 21 MediaPipe joints, robust to how the hand is held:
// the smallest of tip-tip, thumb tip to the index fingertip's last segment (DIP-tip: pads
// touching while the nail tips are a centimetre apart) and index tip to the thumb's last
// segment (IP-tip). zWeight scales the third axis (depth) before measuring. pads = false:
// tip to tip only (in a fist the thumb rests against the curled index finger).
inline float pinchDistance(const float p[21][3], float zWeight = 1.f, bool pads = true) {
    auto at = [&](int i, float o[3]) {
        o[0] = p[i][0];
        o[1] = p[i][1];
        o[2] = p[i][2] * zWeight;
    };
    auto segDist = [](const float q[3], const float a[3], const float b[3]) {
        float ab[3], aq[3];
        for (int k = 0; k < 3; k++) {
            ab[k] = b[k] - a[k];
            aq[k] = q[k] - a[k];
        }
        float len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
        float u = len2 > 1e-12f ? (aq[0] * ab[0] + aq[1] * ab[1] + aq[2] * ab[2]) / len2 : 1.f;
        u = std::fmin(1.f, std::fmax(0.f, u));
        float d2 = 0;
        for (int k = 0; k < 3; k++) {
            float d = aq[k] - u * ab[k];
            d2 += d * d;
        }
        return std::sqrt(d2);
    };
    float tt[3], ti[3], it[3], id[3];
    at(MP_THUMB_TIP, tt);
    at(MP_THUMB_IP, ti);
    at(MP_INDEX_TIP, it);
    at(MP_INDEX_DIP, id);
    if (!pads)
        return segDist(tt, it, it);
    return std::fmin(segDist(tt, id, it), segDist(it, ti, tt));   // both include tip-tip
}

// Places one MediaPipe hand in the ARCore world. Returns false if it can't be solved.
//
// The hand's distance comes from fitting MediaPipe's metric shape to the image (solveTranslation).
// Each joint is then put where it actually is in the image: its 2D landmark back-projected at
// that joint's depth (hand depth + the shape's relative depth). The rigid metric shape alone
// left touching fingertips ~2-3 cm apart (not a single pinch registered in testing);
// the 2D landmarks are far more precise, so fingers that touch in the image touch in 3D.
// imageAnchored = false keeps the old rigid-shape placement.
inline bool placeHand(const float world[21][3],
                      const float image[21][2],
                      const Intrinsics &k,
                      const float camT[3],
                      AlvrQuat camQ,
                      HandObservation &out,
                      bool imageAnchored = true) {
    float t[3];
    if (!solveTranslation(world, image, k, t))
        return false;
    float cv[21][3];
    for (int i = 0; i < 21; i++) {
        float *pCv = cv[i];
        float depth = world[i][2] + t[2];
        if (imageAnchored && depth > 0.03f) {
            pCv[0] = (image[i][0] * k.width - k.cx) / k.fx * depth;
            pCv[1] = (image[i][1] * k.height - k.cy) / k.fy * depth;
            pCv[2] = depth;
        } else {
            pCv[0] = world[i][0] + t[0];
            pCv[1] = world[i][1] + t[1];
            pCv[2] = depth;
        }
        cameraToArWorld(pCv, camT, camQ, out.points[i]);
    }
    // Pinch from two independent views of the hand. The image is precise when the fingertips
    // are visible; when the back of the hand or the other fingers hide them, MediaPipe's 3D
    // shape still knows the fingers are pinched (its 2D tips are then guesses).
    out.pinchImageM = pinchDistance(cv, PINCH_DEPTH_WEIGHT);
    out.pinchShapeM = pinchDistance(world);
    out.pinchM = std::fmin(out.pinchImageM, out.pinchShapeM);
    out.pinchTipM =
        std::fmin(pinchDistance(cv, PINCH_DEPTH_WEIGHT, false), pinchDistance(world, 1.f, false));
    {
        float a[3], b[3], n[3], r[3];
        for (int c = 0; c < 3; c++) {
            a[c] = cv[MP_INDEX_MCP][c] - cv[MP_WRIST][c];
            b[c] = cv[MP_PINKY_MCP][c] - cv[MP_WRIST][c];
            r[c] = cv[MP_WRIST][c];
        }
        n[0] = a[1] * b[2] - a[2] * b[1];
        n[1] = a[2] * b[0] - a[0] * b[2];
        n[2] = a[0] * b[1] - a[1] * b[0];
        float nn = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        float rr = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        if (nn > 1e-9f && rr > 1e-9f) {
            float c = std::fabs(n[0] * r[0] + n[1] * r[1] + n[2] * r[2]) / (nn * rr);
            out.palmFacingDeg = std::acos(std::fmin(1.f, c)) * 57.29578f;
        }
    }
    for (int i = 0; i < 3; i++)
        out.cameraPos[i] = camT[i];
    out.cameraDistanceM = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    out.valid = true;
    return true;
}

// ---- OpenXR skeleton -----------------------------------------------------------------------

inline void sub(const float a[3], const float b[3], float o[3]) {
    for (int i = 0; i < 3; i++)
        o[i] = a[i] - b[i];
}
inline float dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline void cross(const float a[3], const float b[3], float o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline bool normalize3(float v[3]) {
    float n = std::sqrt(dot(v, v));
    if (n < 1e-6f)
        return false;
    for (int i = 0; i < 3; i++)
        v[i] /= n;
    return true;
}

// Quaternion of the rotation whose columns are the basis vectors x, y, z.
inline AlvrQuat quatFromBasis(const float x[3], const float y[3], const float z[3]) {
    float m00 = x[0], m01 = y[0], m02 = z[0];
    float m10 = x[1], m11 = y[1], m12 = z[1];
    float m20 = x[2], m21 = y[2], m22 = z[2];
    float tr = m00 + m11 + m22;
    AlvrQuat q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.f) * 2;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.f + m00 - m11 - m22) * 2;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.f + m11 - m00 - m22) * 2;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.f + m22 - m00 - m11) * 2;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    return fusion::normalize(q);
}

// Joint orientation: -Z along `distalDir`, +Y as close as possible to `dorsal` (out of the back
// of the hand), X = Y x Z (XR_EXT_hand_tracking convention).
inline AlvrQuat jointOrientation(const float distalDir[3], const float dorsal[3]) {
    float z[3] = {-distalDir[0], -distalDir[1], -distalDir[2]};
    normalize3(z);
    float y[3];
    float k = dot(dorsal, z);
    for (int i = 0; i < 3; i++)
        y[i] = dorsal[i] - k * z[i];
    if (!normalize3(y)) {
        float up[3] = {0, 1, 0};
        float kk = dot(up, z);
        for (int i = 0; i < 3; i++)
            y[i] = up[i] - kk * z[i];
        normalize3(y);
    }
    float x[3];
    cross(y, z, x);
    return quatFromBasis(x, y, z);
}

// 21 MediaPipe points (any one world frame) -> 26 OpenXR joints in that frame.
inline void toOpenXrSkeleton(const float mp[21][3], bool isLeft, AlvrPose out[26]) {
    // Back-of-hand normal. For a right hand (index MCP - wrist) x (pinky MCP - wrist) points out
    // of the palm, so flip it; a left hand is the mirror image.
    float a[3], b[3], dorsal[3];
    sub(mp[MP_INDEX_MCP], mp[MP_WRIST], a);
    sub(mp[MP_PINKY_MCP], mp[MP_WRIST], b);
    cross(a, b, dorsal);
    if (!isLeft)
        for (float &v : dorsal)
            v = -v;
    normalize3(dorsal);

    // Joint positions in OpenXR order.
    float pos[26][3];
    auto set = [&](int j, const float p[3]) {
        for (int i = 0; i < 3; i++)
            pos[j][i] = p[i];
    };
    auto lerp = [&](int j, const float p[3], const float q[3], float s) {
        for (int i = 0; i < 3; i++)
            pos[j][i] = p[i] + (q[i] - p[i]) * s;
    };
    lerp(0, mp[MP_WRIST], mp[MP_MIDDLE_MCP], 0.5f);   // palm
    set(1, mp[MP_WRIST]);                              // wrist
    for (int j = 0; j < 4; j++)                        // thumb: CMC, MCP, IP, tip
        set(2 + j, mp[1 + j]);
    for (int f = 0; f < 4; f++) {   // index, middle, ring, little
        int base = 6 + f * 5, mcp = 5 + f * 4;
        lerp(base, mp[MP_WRIST], mp[mcp], 0.2f);   // metacarpal: near the wrist end
        for (int j = 0; j < 4; j++)
            set(base + 1 + j, mp[mcp + j]);   // proximal(MCP), intermediate(PIP), distal(DIP), tip
    }

    // Orientations: each joint's bone points to the next joint along its chain; tips reuse the
    // last bone; palm and wrist point along wrist -> middle MCP.
    auto orient = [&](int j, int from, int to) {
        float d[3];
        sub(pos[to], pos[from], d);
        out[j].orientation = jointOrientation(d, dorsal);
        for (int i = 0; i < 3; i++)
            out[j].position[i] = pos[j][i];
    };
    orient(0, 1, 12);   // palm: wrist -> middle proximal (MCP)
    orient(1, 1, 12);   // wrist
    for (int j = 2; j <= 4; j++)
        orient(j, j, j + 1);
    orient(5, 4, 5);
    for (int f = 0; f < 4; f++) {
        int base = 6 + f * 5;
        for (int j = 0; j < 4; j++)
            orient(base + j, base + j, base + j + 1);
        orient(base + 4, base + 3, base + 4);
    }
}

// ---- Handedness -----------------------------------------------------------------------------
//
// MediaPipe decides left/right from appearance, but a left hand seen from the back looks like a
// right hand seen from the palm, so from a forehead camera its label flips with how the hand is
// held (on device a lone left hand came out "right" most of the time). Handedness is instead
// scored from geometry relative to the body:
//  - which side of the head's centre line the wrist is on (the camera sits a little left, and
//    hands drift left with it, so the line is shifted by centreOffsetM);
//  - which way the hand points: reaching forward, a left hand points inward (to the right) and a
//    right hand to the left.
// Score > 0 means left. Both terms are roughly in -1..1.

struct HandednessConfig {
    float centreOffsetM = -0.05f;   // centre line, head frame (negative = left of the nose)
    float lateralScaleM = 0.10f;    // 10 cm off the line = full confidence from position
    float pointingScale = 0.5f;     // sideways part of the pointing direction for full confidence
};

inline float leftHandScore(const float headPos[3],
                           AlvrQuat headQ,
                           const float wrist[3],
                           const float middleMcp[3],
                           const HandednessConfig &c = HandednessConfig()) {
    float f[3], fwd0[3] = {0, 0, -1};
    fusion::rotate(headQ, fwd0, f);
    f[1] = 0;
    if (!normalize3(f)) {
        f[0] = 0;
        f[2] = -1;
    }
    float right[3] = {-f[2], 0, f[0]};
    float rel[3], dir[3];
    sub(wrist, headPos, rel);
    sub(middleMcp, wrist, dir);
    dir[1] = 0;
    float lateral = dot(rel, right);
    float pointing = normalize3(dir) ? dot(dir, right) : 0.f;   // + = pointing to the right
    return -(lateral - c.centreOffsetM) / c.lateralScaleM + pointing / c.pointingScale;
}

// ---- Pointing ("hand ray") ------------------------------------------------------------------
//
// The emulated controller is aimed along the line from an estimated shoulder through the hand,
// like Quest's system pointer, instead of along the palm's orientation. Positions are far
// steadier than MediaPipe's hand orientation, and you point with your arm, not your palm. The
// shoulder follows the head's heading only (not pitch/roll) so nodding doesn't swing the ray.
// The controller is kept upright (no roll) for stability.

struct AimConfig {
    float shoulderSideM = 0.17f;
    float shoulderDownM = 0.22f;
    float shoulderBackM = 0.08f;
    float minCutoffHz = 1.0f;       // ray smoothing when still
    float beta = 2.0f;              // per rad/s of (smoothed) ray motion: faster when you swing
    float speedCutoffHz = 1.0f;     // smoothing of that speed estimate (noise isn't motion)
};

class AimFilter {
  public:
    AimConfig config;

    void reset() {
        initialized_ = false;
        speed_ = 0;
    }

    // Controller orientation (world from controller, -Z along the ray, +Y up-ish) for a hand at
    // `hand` (knuckles), given the head pose. All in the same y-up world frame.
    AlvrQuat update(const float headPos[3], AlvrQuat headQ, const float hand[3], bool isLeft,
                    double t) {
        float f[3], fwd0[3] = {0, 0, -1};
        fusion::rotate(headQ, fwd0, f);
        f[1] = 0;
        if (!normalize3(f)) {
            f[0] = 0;
            f[2] = -1;
        }
        float right[3] = {-f[2], 0, f[0]};
        float side = isLeft ? -config.shoulderSideM : config.shoulderSideM;
        float shoulder[3];
        for (int i = 0; i < 3; i++)
            shoulder[i] = headPos[i] + right[i] * side - f[i] * config.shoulderBackM;
        shoulder[1] -= config.shoulderDownM;

        float raw[3];
        sub(hand, shoulder, raw);
        if (!normalize3(raw)) {
            for (int i = 0; i < 3; i++)
                raw[i] = f[i];
        }
        if (!initialized_ || t - lastT_ > 0.5) {
            for (int i = 0; i < 3; i++)
                dir_[i] = raw[i];
            initialized_ = true;
        } else {
            float dt = (float) std::fmax(1e-4, t - lastT_);
            float c = std::fmin(1.f, std::fmax(-1.f, dot(dir_, raw)));
            float angSpeed = std::acos(c) / dt;
            float aS = 1.f - std::exp(-dt * 2.f * (float) M_PI * config.speedCutoffHz);
            speed_ += aS * (angSpeed - speed_);
            float cutoff = config.minCutoffHz + config.beta * speed_;
            float a = 1.f - std::exp(-dt * 2.f * (float) M_PI * cutoff);
            for (int i = 0; i < 3; i++)
                dir_[i] += a * (raw[i] - dir_[i]);
            normalize3(dir_);
        }
        lastT_ = t;
        float up[3] = {0, 1, 0};
        if (std::fabs(dot(up, dir_)) > 0.97f) {   // pointing straight up/down: use heading
            for (int i = 0; i < 3; i++)
                up[i] = -f[i] * (dir_[1] > 0 ? 1.f : -1.f);
        }
        return jointOrientation(dir_, up);
    }

    const float *direction() const { return dir_; }

  private:
    bool initialized_ = false;
    float dir_[3] = {0, 0, -1};
    float speed_ = 0;
    double lastT_ = 0;
};

// ---- Gesture measurements -------------------------------------------------------------------
//
// The same distances ALVR's server uses for its gesture -> button emulation
// (alvr/server/src/hand_gestures.rs), measured centre to centre on our 26-joint skeleton, so
// its thresholds can be tuned from real numbers. ALVR subtracts fingertip radii (thumb 7.5 mm,
// index 6.5 mm; palm 5 mm + finger ~6 mm for curls) from these before comparing.
struct GestureDistances {
    float pinchM;   // thumb tip <-> index tip
    float gripM;    // mean of middle/ring/little tip <-> middle of their metacarpal..proximal
};

inline GestureDistances measureGestures(const AlvrPose s[26]) {
    auto d = [](const AlvrPose &a, const float b[3]) {
        float x = a.position[0] - b[0], y = a.position[1] - b[1], z = a.position[2] - b[2];
        return std::sqrt(x * x + y * y + z * z);
    };
    GestureDistances g;
    g.pinchM = d(s[5], s[10].position);
    float sum = 0;
    for (int f = 1; f <= 3; f++) {   // middle, ring, little
        int base = 6 + f * 5;
        float mid[3];
        for (int i = 0; i < 3; i++)
            mid[i] = 0.5f * (s[base].position[i] + s[base + 1].position[i]);
        sum += d(s[base + 4], mid);
    }
    g.gripM = sum / 3;
    return g;
}

// While a pinch is held, pulls the thumb and index tips together (amount 1 = touching) so the
// game shows the fingers touching even when the tracked tips are a centimetre or two apart.
// The thumb IP / index DIP joints follow half way so the fingers bend instead of stretching.
// Runs on MediaPipe points, before toOpenXrSkeleton, so joint orientations stay consistent.
inline void snapPinch(float pts[21][3], float amount) {
    if (amount <= 0)
        return;
    amount = std::fmin(amount, 1.f);
    float *a = pts[MP_THUMB_TIP], *b = pts[MP_INDEX_TIP];
    float shiftA[3], shiftB[3];
    for (int k = 0; k < 3; k++) {
        float mid = 0.5f * (a[k] + b[k]);
        shiftA[k] = (mid - a[k]) * amount;
        shiftB[k] = (mid - b[k]) * amount;
    }
    for (int k = 0; k < 3; k++) {
        a[k] += shiftA[k];
        b[k] += shiftB[k];
        pts[MP_THUMB_IP][k] += 0.5f * shiftA[k];
        pts[MP_INDEX_DIP][k] += 0.5f * shiftB[k];
    }
}

// ---- Temporal filtering ---------------------------------------------------------------------
//
// MediaPipe results arrive ~15-30 times a second, ~100 ms old, with dropouts and occasional
// left/right label flips. HandFilter keeps one track per side:
//  - detections are assigned to a side by continuity (close to where that hand just was) before
//    trusting MediaPipe's label, so a flipping label can't swap hands mid-motion;
//  - single-frame jumps larger than maxJumpM are rejected (monocular depth is the weak axis);
//  - every joint is One-Euro smoothed, and the whole hand is predicted to the display time along
//    the wrist velocity (speed-gated like the head, so a still hand doesn't jitter);
//  - after a miss the hand coasts briefly, then freezes, and is dropped after holdS.

struct HandFilterConfig {
    // ---- Hand position (wrist) ----
    // Depth (along the camera ray) comes from the hand's apparent size and is far noisier than
    // the sideways position, which comes straight from the image. So the wrist is smoothed per
    // axis: sideways like before, along the ray alongFactor times slower, and the depth part of a
    // single step is capped at maxAlongStepM (a 24 cm depth glitch once made a ghost hand).
    float minCutoffHz = 1.0f;
    float beta = 2.0f;
    float derivCutoffHz = 1.5f;
    float alongFactor = 0.35f;
    float maxAlongStepM = 0.05f;
    // ---- Finger shape (joints relative to the wrist) ----
    // Smoothed separately and lightly, so pinches and fists show up quickly.
    float shapeMinCutoffHz = 2.5f;
    float shapeBeta = 4.0f;
    // At ~15 detections/s, 8 mm of noise already reads as ~0.1 m/s, so prediction only fades
    // in above that.
    float predictionFadeInSpeed[2] = {0.12f, 0.35f};   // m/s
    float maxPredictionS = 0.12f;
    float coastS = 0.10f;      // keep extrapolating this long after the last detection
    float holdS = 0.30f;       // then hold still; drop the hand after this long
    // ---- Which detection is which hand ----
    // Distances for matching use sideways distance plus alongWeight x depth distance (depth is
    // the unreliable axis). A detection within assignRadiusM of a hand seen within
    // flickerWindowS continues it whatever its label (at ~16 detections/s gaps > 0.15 s are
    // common, and re-deciding then put one hand on both sides).
    float alongWeight = 0.3f;
    float assignRadiusM = 0.12f;
    float flickerWindowS = 0.30f;
    // One physical hand can't be two: after a frame, a track not updated by it whose wrist is
    // within sameHandM of a hand that was just placed is dropped. Two detections in one frame
    // within duplicateM are one hand reported twice.
    float sameHandM = 0.10f;
    float duplicateM = 0.06f;
    // A hand that comes back (lost < reacquireS ago, within reacquireRadiusM) while the other
    // hand isn't around keeps its side, unless the geometry is beyond +-strongScore the other
    // way. In testing a lone left hand reappeared right of centre and became "right".
    float reacquireS = 1.5f;
    float reacquireRadiusM = 0.45f;
    float strongScore = 2.5f;
    // leftHandScore: within +-tieScore MediaPipe's label decides; a continuing track only
    // switches side when the score is beyond +-switchScore the other way.
    float tieScore = 0.2f;
    float switchScore = 1.5f;
    float maxJumpM = 0.25f;
    // 0 = both hands (automatic sides), 1 = left hand only, 2 = right hand only.
    int handMode = 0;
};

// Wrist filter with different smoothing sideways and along the camera ray (see config).
class RayRootFilter {
  public:
    void reset() { init_ = false; }
    bool initialized() const { return init_; }

    void update(const float z[3], const float cam[3], double t, const HandFilterConfig &c) {
        if (!init_) {
            for (int i = 0; i < 3; i++) {
                x_[i] = raw_[i] = z[i];
                v_[i] = 0;
            }
            t_ = t;
            lag_ = 0;
            init_ = true;
            return;
        }
        float dt = (float) (t - t_);
        if (dt <= 1e-4f)
            return;
        t_ = t;
        float ray[3];
        for (int i = 0; i < 3; i++)
            ray[i] = z[i] - cam[i];
        if (!normalize3(ray)) {
            ray[0] = ray[1] = 0;
            ray[2] = -1;
        }
        // Accepted raw track: the depth part of each step is capped.
        float step[3];
        for (int i = 0; i < 3; i++)
            step[i] = z[i] - raw_[i];
        float along = dot(step, ray);
        float alongC = std::fmax(-c.maxAlongStepM, std::fmin(c.maxAlongStepM, along));
        for (int i = 0; i < 3; i++)
            raw_[i] += step[i] - along * ray[i] + alongC * ray[i];
        // Velocity: sideways in full, depth damped (its noise isn't motion).
        float aD = alpha(c.derivCutoffHz, dt), speed = 0;
        for (int i = 0; i < 3; i++) {
            float rawV = (step[i] - along * ray[i] + c.alongFactor * alongC * ray[i]) / dt;
            v_[i] += aD * (rawV - v_[i]);
            speed += v_[i] * v_[i];
        }
        float cutoff = c.minCutoffHz + c.beta * std::sqrt(speed);
        float a = alpha(cutoff, dt), aAlong = alpha(cutoff * c.alongFactor, dt);
        float d[3];
        for (int i = 0; i < 3; i++)
            d[i] = raw_[i] - x_[i];
        float da = dot(d, ray);
        for (int i = 0; i < 3; i++)
            x_[i] += a * (d[i] - da * ray[i]) + aAlong * da * ray[i];
        lag_ = dt * (1 - a) / a;
    }

    const float *value() const { return x_; }
    const float *velocity() const { return v_; }
    float lagS() const { return lag_; }

  private:
    static float alpha(float cutoffHz, float dt) {
        float tau = 1.f / (2 * (float) M_PI * cutoffHz);
        return 1.f / (1.f + tau / dt);
    }
    bool init_ = false;
    float x_[3] = {0, 0, 0}, raw_[3] = {0, 0, 0}, v_[3] = {0, 0, 0};
    double t_ = 0;
    float lag_ = 0;
};

class HandFilter {
  public:
    HandFilterConfig config;

    // One MediaPipe result (0-2 hands, ARCore world, isLeft already corrected for mirroring).
    void onDetections(int64_t captureNs, int count, const HandObservation *obs) {
        double t = captureNs * 1e-9;
        bool taken[2] = {false, false};
        detectionSide_[0] = detectionSide_[1] = -1;
        if (count == 2 && obs[0].valid && obs[1].valid &&
            dist(obs[0].points[0], obs[1].points[0]) < config.duplicateM) {
            count = 1;   // the same hand reported twice: keep the first
            duplicates_++;
        }
        if (config.handMode != 0 && count > 1)
            count = 1;   // single-hand mode: one hand at most
        const float *placed[2] = {nullptr, nullptr};
        // With two hands, place the more-left one first so it gets first claim on "left".
        int order[2] = {0, 1};
        if (count == 2 && obs[1].valid && (!obs[0].valid || obs[1].leftScore > obs[0].leftScore)) {
            order[0] = 1;
            order[1] = 0;
        }
        for (int kk = 0; kk < count && kk < 2; kk++) {
            const HandObservation &o = obs[order[kk]];
            if (!o.valid)
                continue;
            int side = assignSide(o, t, taken);
            if (side < 0)
                continue;
            taken[side] = true;
            Track &tr = tracks_[side];
            bool continuing = tr.active && t - tr.lastSeenT < 0.15;
            if (continuing && wdist(o, tr.root.value()) > config.maxJumpM) {
                rejected_++;
                continue;
            }
            if (!tr.active || t - tr.lastSeenT > config.holdS) {
                tr.root.reset();
                for (auto &f : tr.shape)
                    f.reset();
            }
            tr.root.update(o.points[0], o.cameraPos, t, config);
            for (int i = 1; i < 21; i++) {
                float off[3];
                sub(o.points[i], o.points[0], off);
                tr.shape[i].minCutoffHz = config.shapeMinCutoffHz;
                tr.shape[i].beta = config.shapeBeta;
                tr.shape[i].derivCutoffHz = config.derivCutoffHz;
                tr.shape[i].update(off, t);
            }
            tr.active = true;
            tr.everSeen = true;
            tr.lastSeenT = t;
            placed[side] = tr.root.value();
            detectionSide_[order[kk]] = side;
            labelVotes_[side][o.isLeft ? 0 : 1]++;
        }
        // Drop a stale track that sits where a hand was just placed on the other side.
        for (int s = 0; s < 2; s++) {
            const float *p = placed[1 - s];
            if (taken[s] || !p || !tracks_[s].active)
                continue;
            if (dist(tracks_[s].root.value(), p) < config.sameHandM) {
                tracks_[s].active = false;
                mergedTracks_++;
            }
        }
    }

    uint32_t duplicates() const { return duplicates_; }
    // Which side detection i of the last onDetections call went to (0 left, 1 right), or -1.
    int detectionSide(int i) const { return i >= 0 && i < 2 ? detectionSide_[i] : -1; }
    // MediaPipe label votes for the detections placed on `side` since the last call: {Left, Right}.
    void takeLabelVotes(int side, int out[2]) {
        out[0] = labelVotes_[side][0];
        out[1] = labelVotes_[side][1];
        labelVotes_[side][0] = labelVotes_[side][1] = 0;
    }
    uint32_t mergedTracks() const { return mergedTracks_; }

    // When `side` was last really seen by the camera (seconds, capture clock), or -1.
    double lastSeenS(int side) const {
        const Track &tr = tracks_[side];
        return tr.active ? tr.lastSeenT : -1;
    }

    // Predicted joints for `side` (0 left, 1 right) at targetNs. False if no recent hand.
    bool predict(int side, int64_t targetNs, float out[21][3]) const {
        const Track &tr = tracks_[side];
        if (!tr.active)
            return false;
        double t = targetNs * 1e-9;
        double sinceSeen = t - tr.lastSeenT;
        if (sinceSeen > config.holdS + 0.15)   // target time runs ~0.1 s ahead of capture
            return false;
        const float *v = tr.root.velocity();
        float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        float lo = config.predictionFadeInSpeed[0], hi = config.predictionFadeInSpeed[1];
        float g = std::fmin(1.f, std::fmax(0.f, (speed - lo) / (hi - lo)));
        g = g * g * (3 - 2 * g);
        // Coast along the velocity for a short while after the last detection, then hold.
        float horizon = (float) std::fmin(sinceSeen, config.coastS + 0.1) + tr.root.lagS();
        horizon = std::fmin(std::fmax(horizon, 0.f), config.maxPredictionS);
        float root[3];
        for (int k = 0; k < 3; k++)
            root[k] = tr.root.value()[k] + v[k] * horizon * g;
        for (int k = 0; k < 3; k++)
            out[0][k] = root[k];
        for (int i = 1; i < 21; i++) {
            const float *off = tr.shape[i].value();
            for (int k = 0; k < 3; k++)
                out[i][k] = root[k] + off[k];
        }
        return true;
    }

    uint32_t rejected() const { return rejected_; }

  private:
    struct Track {
        bool active = false;
        bool everSeen = false;
        double lastSeenT = 0;
        RayRootFilter root;
        fusion::OneEuro3 shape[21];   // joints relative to the wrist (index 0 unused)
    };

    static float dist(const float a[3], const float b[3]) {
        float d0 = a[0] - b[0], d1 = a[1] - b[1], d2 = a[2] - b[2];
        return std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    }

    // Sideways distance + alongWeight x depth distance, relative to the observation's camera ray.
    float wdist(const HandObservation &o, const float p[3]) const {
        float ray[3], d[3];
        sub(o.points[0], o.cameraPos, ray);
        if (!normalize3(ray))
            return dist(o.points[0], p);
        sub(o.points[0], p, d);
        float al = dot(d, ray);
        float perp2 = dot(d, d) - al * al;
        return std::sqrt(std::fmax(0.f, perp2) + config.alongWeight * config.alongWeight * al * al);
    }

    int assignSide(const HandObservation &o, double t, const bool taken[2]) const {
        if (config.handMode == 1)
            return taken[0] ? -1 : 0;
        if (config.handMode == 2)
            return taken[1] ? -1 : 1;
        // Geometric handedness decides; MediaPipe's label only breaks near-ties.
        int scored = o.leftScore > config.tieScore    ? 0
                     : o.leftScore < -config.tieScore ? 1
                                                      : (o.isLeft ? 0 : 1);
        // Continuity: a hand continuing a just-seen track keeps that track's side unless the
        // geometry clearly says otherwise.
        int best = -1;
        float bestD = config.assignRadiusM;
        for (int s = 0; s < 2; s++) {
            const Track &tr = tracks_[s];
            if (taken[s] || !tr.active || t - tr.lastSeenT > config.flickerWindowS)
                continue;
            float d = wdist(o, tr.root.value());
            if (d < bestD) {
                bestD = d;
                best = s;
            }
        }
        if (best >= 0) {
            bool clearlyOther = (best == 0 && o.leftScore < -config.switchScore) ||
                                (best == 1 && o.leftScore > config.switchScore);
            if (!clearlyOther)
                return best;
        }
        // Re-acquire: the only hand seen recently comes back -> same side.
        int recent[2];
        for (int s = 0; s < 2; s++) {
            const Track &tr = tracks_[s];
            recent[s] = tr.everSeen && t - tr.lastSeenT <= config.reacquireS;
        }
        for (int s = 0; s < 2; s++) {
            const Track &tr = tracks_[s];
            if (taken[s] || !recent[s] || recent[1 - s])
                continue;
            if (wdist(o, tr.root.value()) > config.reacquireRadiusM)
                continue;
            bool strongOther = (s == 0 && o.leftScore < -config.strongScore) ||
                               (s == 1 && o.leftScore > config.strongScore);
            if (!strongOther)
                return s;
        }
        if (!taken[scored])
            return scored;
        return taken[1 - scored] ? -1 : 1 - scored;
    }

    Track tracks_[2];
    uint32_t rejected_ = 0;
    uint32_t duplicates_ = 0, mergedTracks_ = 0;
    int labelVotes_[2][2] = {{0, 0}, {0, 0}};
    int detectionSide_[2] = {-1, -1};
};

}   // namespace hands

#endif   // PHONEVR_HAND_TRACKING_H
