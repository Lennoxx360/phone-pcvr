"""Estimate the phone camera's offset from the head's rotation pivot (neck) from a PVR-Raw log.

Record: stand in one spot, keep your torso still, and slowly nod, shake, tilt and look around
for ~20 s with the app running. Then:
    adb logcat -d -s PVR-Raw:I > raw.log
    python -I calibrate_pivot.py raw.log

Model: camera_position(t) = pivot(t) + R(t) * c, where R is the ARCore display-oriented camera
rotation (head frame: x right, y up, -z forward) and c is the camera position relative to the
pivot in head coordinates. pivot(t) = p0 + v * t absorbs slow torso sway. Linear least squares
in (p0, v, c); pure Python, no numpy.
"""

import math
import re
import sys


def quat_to_mat(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def solve(a, b):
    """Solve the square system a x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("singular system: not enough head rotation in the recording")
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r != col:
                f = m[r][col] / m[col][col]
                for k in range(col, n + 1):
                    m[r][k] -= f * m[col][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def fit(samples, drift=True):
    """samples: list of (t_s, cam[3], R[3][3]). Returns (c, p0, v, rms_m)."""
    t0 = samples[0][0]
    nu = 9 if drift else 6
    ata = [[0.0] * nu for _ in range(nu)]
    atb = [0.0] * nu
    rows = []
    for t, cam, r in samples:
        for i in range(3):
            row = [0.0] * nu
            row[i] = 1.0  # p0
            if drift:
                row[3 + i] = t - t0  # v
            for j in range(3):
                row[nu - 3 + j] = r[i][j]  # c
            rows.append((row, cam[i]))
    for row, y in rows:
        for a in range(nu):
            atb[a] += row[a] * y
            for b in range(nu):
                ata[a][b] += row[a] * row[b]
    x = solve(ata, atb)
    sq = sum((sum(row[k] * x[k] for k in range(nu)) - y) ** 2 for row, y in rows)
    rms = math.sqrt(sq / len(rows))
    p0, v = x[0:3], (x[3:6] if drift else [0, 0, 0])
    return x[nu - 3 :], p0, v, rms


def euler_deg(r):
    yaw = math.degrees(math.atan2(r[0][2], r[2][2]))
    pitch = math.degrees(math.asin(max(-1, min(1, -r[1][2]))))
    roll = math.degrees(math.atan2(r[1][0], r[1][1]))
    return yaw, pitch, roll


def main(path):
    rx = re.compile(r"PVR-Raw\s*:\s*(-?\d+) (\d) " + " ".join([r"(-?[\d.]+)"] * 11))
    samples = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = rx.search(line)
        if not m or m.group(2) != "1":
            continue
        g = [float(v) for v in m.groups()[2:]]
        samples.append((int(m.group(1)) * 1e-9, g[0:3], quat_to_mat(*g[3:7])))
    if len(samples) < 60:
        sys.exit(f"only {len(samples)} tracked samples; record ~20 s with ARCore tracking")
    eul = [euler_deg(s[2]) for s in samples]
    span = [max(e[k] for e in eul) - min(e[k] for e in eul) for k in range(3)]
    print(f"{len(samples)} samples over {samples[-1][0] - samples[0][0]:.1f} s; "
          f"rotation range yaw {span[0]:.0f} pitch {span[1]:.0f} roll {span[2]:.0f} deg")
    if min(span[0], span[1]) < 20:
        print("WARNING: little rotation in yaw or pitch; the fit will be poorly constrained")

    for drift in (False, True):
        c, p0, v, rms = fit(samples, drift)
        print(f"\n{'with' if drift else 'without'} pivot drift: residual RMS {rms * 1000:.1f} mm")
        print(f"  camera relative to pivot (head frame, m): "
              f"x {c[0]:+.3f}  y {c[1]:+.3f}  z {c[2]:+.3f}")
        if drift:
            print(f"  pivot drift {math.sqrt(sum(x * x for x in v)) * 100:.1f} cm/s")

    eye_from_neck = [0.0, 0.075, -0.08]
    cam_in_head = [c[i] - eye_from_neck[i] for i in range(3)]
    print("\nFor pose_fusion.h Config (keeping eyeFromNeck = {0, 0.075, -0.08}):")
    print("    float cameraInHead[3] = {%.3ff, %.3ff, %.3ff};" % tuple(cam_in_head))


if __name__ == "__main__":
    main(sys.argv[1])
