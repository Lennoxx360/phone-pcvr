// Headset-fit adjustments applied on top of the Cardboard viewer profile (the QR-code
// DeviceParams protobuf). The profile is decoded with a tiny wire-format reader, the fit
// fields are changed, and it is re-encoded for CardboardLensDistortion_create().
//
// DeviceParams fields used (cardboard_device.proto):
//   3 screen_to_lens_distance (float)  -> image size (magnification)
//   4 inter_lens_distance     (float)  -> lens IPD: horizontal distance between eye centres
//   5 left_eye_field_of_view_angles (packed floats: left, right, bottom, top, degrees)
//   6 tray_to_lens_distance   (float)  -> vertical position of the eye centres
//  11 vertical_alignment      (enum BOTTOM=0, CENTER=1, TOP=2)
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace viewer {

// Cardboard's kDefaultBorderSizeMeters (lens_distortion.cc).
constexpr float kBorderM = 0.003f;

struct Adjust {
    float ipdDeltaM = 0;      // added to inter_lens_distance
    float sizeScale = 1;      // multiplies screen_to_lens_distance (bigger = larger image)
    float fovScale = 1;       // multiplies the four FOV angles
    float verticalM = 0;      // + moves the eye centres away from the tray-side screen edge
    float stereoDeltaM = 0;   // extra eye separation for 3D depth only (not the screen image)
};

struct Info {
    bool ok = false;
    float screenToLens = 0, interLens = 0, trayToLens = 0;
    int alignment = 0;   // BOTTOM
    float fov[4] = {0, 0, 0, 0};
    int fovCount = 0;
};

namespace detail {

inline bool readVarint(const uint8_t *&p, const uint8_t *end, uint64_t &v) {
    v = 0;
    for (int shift = 0; shift < 64 && p < end; shift += 7) {
        uint8_t b = *p++;
        v |= uint64_t(b & 0x7f) << shift;
        if (!(b & 0x80))
            return true;
    }
    return false;
}

inline void writeVarint(std::vector<uint8_t> &out, uint64_t v) {
    while (v >= 0x80) {
        out.push_back(uint8_t(v) | 0x80);
        v >>= 7;
    }
    out.push_back(uint8_t(v));
}

inline float readFloat(const uint8_t *p) {
    float f;
    memcpy(&f, p, 4);   // little-endian on every Android ABI
    return f;
}

inline void writeFloatField(std::vector<uint8_t> &out, int field, float f) {
    writeVarint(out, (uint64_t(field) << 3) | 5);
    uint8_t b[4];
    memcpy(b, &f, 4);
    out.insert(out.end(), b, b + 4);
}

}   // namespace detail

// Decodes the profile into `info`. Returns false on malformed input.
inline bool parse(const uint8_t *data, int size, Info &info) {
    using namespace detail;
    info = Info{};
    const uint8_t *p = data, *end = data + size;
    while (p < end) {
        uint64_t key;
        if (!readVarint(p, end, key))
            return false;
        int field = int(key >> 3), wt = int(key & 7);
        if (wt == 0) {
            uint64_t v;
            if (!readVarint(p, end, v))
                return false;
            if (field == 11)
                info.alignment = int(v);
        } else if (wt == 5) {
            if (end - p < 4)
                return false;
            float f = readFloat(p);
            p += 4;
            if (field == 3)
                info.screenToLens = f;
            else if (field == 4)
                info.interLens = f;
            else if (field == 6)
                info.trayToLens = f;
            else if (field == 5 && info.fovCount < 4)
                info.fov[info.fovCount++] = f;   // unpacked repeated encoding
        } else if (wt == 1) {
            if (end - p < 8)
                return false;
            p += 8;
        } else if (wt == 2) {
            uint64_t len;
            if (!readVarint(p, end, len) || uint64_t(end - p) < len)
                return false;
            if (field == 5) {
                for (uint64_t i = 0; i + 4 <= len && info.fovCount < 4; i += 4)
                    info.fov[info.fovCount++] = readFloat(p + i);
            }
            p += len;
        } else {
            return false;   // groups are not used by DeviceParams
        }
    }
    info.ok = info.screenToLens > 0 && info.interLens > 0 && info.fovCount == 4;
    return info.ok;
}

// Writes the adjusted profile to `out`. `screenHeightM` (short side of the screen, metres) is
// only needed to move a CENTER-aligned profile vertically. Returns false (and leaves `out`
// empty) if the profile can't be decoded; callers should then use the original bytes.
inline bool apply(const uint8_t *data,
                  int size,
                  const Adjust &a,
                  float screenHeightM,
                  std::vector<uint8_t> &out,
                  Info *baseOut = nullptr) {
    using namespace detail;
    out.clear();
    Info base;
    if (!parse(data, size, base))
        return false;
    if (baseOut)
        *baseOut = base;

    // Copy every field we don't rewrite.
    const uint8_t *p = data, *end = data + size;
    while (p < end) {
        const uint8_t *start = p;
        uint64_t key;
        readVarint(p, end, key);
        int field = int(key >> 3), wt = int(key & 7);
        if (wt == 0) {
            uint64_t v;
            readVarint(p, end, v);
        } else if (wt == 5) {
            p += 4;
        } else if (wt == 1) {
            p += 8;
        } else {
            uint64_t len;
            readVarint(p, end, len);
            p += len;
        }
        bool rewritten = field == 3 || field == 4 || field == 5 || field == 6 || field == 11;
        if (!rewritten)
            out.insert(out.end(), start, p);
    }

    float stl = std::clamp(base.screenToLens * a.sizeScale, 0.01f, 0.2f);
    float ipd = std::clamp(base.interLens + a.ipdDeltaM, 0.03f, 0.1f);
    writeFloatField(out, 3, stl);
    writeFloatField(out, 4, ipd);

    // Packed FOV angles.
    writeVarint(out, (5u << 3) | 2);
    writeVarint(out, 16);
    for (float f : base.fov) {
        float v = std::clamp(f * a.fovScale, 5.0f, 85.0f);
        uint8_t b[4];
        memcpy(b, &v, 4);
        out.insert(out.end(), b, b + 4);
    }

    int alignment = base.alignment;
    float tray = base.trayToLens;
    if (a.verticalM != 0) {
        if (alignment == 1 && screenHeightM > 0) {
            // CENTER has no tray distance: express the same centre as BOTTOM, then move it.
            alignment = 0;
            tray = screenHeightM / 2 + kBorderM;
        }
        if (alignment == 0)
            tray += a.verticalM;   // eye centre = tray - border, measured from the bottom
        else if (alignment == 2)
            tray -= a.verticalM;   // measured from the top
        tray = std::max(tray, kBorderM + 0.005f);
    }
    if (tray > 0 || base.trayToLens > 0)
        writeFloatField(out, 6, tray);
    writeVarint(out, 11u << 3);
    writeVarint(out, uint64_t(alignment));
    return true;
}

}   // namespace viewer
