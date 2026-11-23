"""Summarises one stream test: the phone's PVR-Net lines (logcat) and ALVR's session_log.txt.

Usage: python stream_summary.py <dir with logcat.txt and session_log.txt> [skip_seconds]
Skips the first skip_seconds (default 10) of streaming, when SteamVR is still loading.
       python stream_summary.py --trim <dir> <full logcat file>   (used by stream-test.ps1)
"""
import json
import re
import statistics
import sys
from pathlib import Path


def trim(out_dir: Path, raw_logcat: Path):
    """Writes out_dir/logcat.txt with only app/ALVR lines, and strips the per-frame
    GraphStatistics events from out_dir/session_log.txt."""
    keep = re.compile(r" (PVR-[\w-]+|ALVR[\w-]*|ALVRActivity-Java|\[ALVR[^\]]*\]|WifiService|"
                      r"SemWifiOptimizer|AndroidRuntime)\s*:", re.I)
    with open(raw_logcat, encoding="utf-8", errors="ignore") as f:
        lines = [l for l in f if keep.search(l)]
    (out_dir / "logcat.txt").write_text("".join(lines), encoding="utf-8")
    sl = out_dir / "session_log.txt"
    if sl.exists():
        with open(sl, encoding="utf-8", errors="ignore") as f:
            kept = [l for l in f if '"GraphStatistics"' not in l]
        sl.write_text("".join(kept), encoding="utf-8")


if len(sys.argv) > 1 and sys.argv[1] == "--trim":
    trim(Path(sys.argv[2]), Path(sys.argv[3]))
    sys.exit(0)

d = Path(sys.argv[1])
skip = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


def stats(xs):
    if not xs:
        return "n/a"
    xs = sorted(xs)
    p95 = xs[min(len(xs) - 1, int(0.95 * len(xs)))]
    return f"avg {statistics.mean(xs):6.1f}  p95 {p95:6.1f}  max {xs[-1]:6.1f}  (n={len(xs)})"


num = r"([-+]?\d+(?:\.\d+)?)"
pat = {
    "video in /s": rf"video in {num}/s",
    "arrival gap max ms": rf"gap avg {num} max {num} ms",
    "decode ms": rf"dec {num}/s {num} ms",
    "shown /s": rf"shown {num}/s",
    "decoded queue ms": rf"queue {num} ms",
    "late ms": rf"late {num} ms",
    "pose fallback /s": rf"pose-fallback {num}/s",
    "stale drops /s": rf"stale {num}/s",
    "decoder-full drops /s": rf"decoder-full {num}",
    "corrupt drops /s": rf"corrupt {num}",
    "IDR req /s": rf"IDR req {num}",
    "render Hz": rf"render {num} Hz",
    "new frames /s": rf"new {num}/s",
    "re-warped /s": rf"re-warped {num}/s",
    "swap->photons ms": rf"disp {num} ms",
    "tw correction avg deg": rf"correction avg {num} max {num}",
    "render cpu ms": rf"render {num}/{num} ms",
    "lookahead ms": rf"lookahead {num} ms",
}
vals = {k: [] for k in pat}
first_t = None
for line in open(d / "logcat.txt", encoding="utf-8", errors="ignore"):
    if "PVR-Net" not in line:
        continue
    m = re.match(r"\d+-\d+ (\d+):(\d+):(\d+\.\d+)", line)
    if not m:
        continue
    t = int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])
    if first_t is None and "video in" in line:
        first_t = t
    if first_t is None or t - first_t < skip:
        continue
    for k, p in pat.items():
        mm = re.search(p, line)
        if mm:
            idx = 2 if k in ("arrival gap max ms",) else 1
            if k == "decode ms":
                idx = 2
            vals[k].append(float(mm[idx]))

print(f"== phone ({d.name}) ==")
for k, v in vals.items():
    if v:
        print(f"  {k:24s} {stats(v)}")

sl = d / "session_log.txt"
if sl.exists():
    rows = []
    for line in open(sl, encoding="utf-8", errors="ignore"):
        if "StatisticsSummary" not in line:
            continue
        try:
            j = json.loads(line)
        except Exception:
            continue
        rows.append(j["event_type"]["data"])
    rows = rows[int(skip * 2):]  # ~2 summaries per second
    if rows:
        print("== PC (ALVR StatisticsSummary) ==")
        for k in ("network_latency_ms", "total_latency_ms", "encode_latency_ms",
                  "decode_latency_ms", "client_fps", "video_mbits_per_sec"):
            print(f"  {k:24s} {stats([r[k] for r in rows if k in r])}")
        print(f"  packets_lost_total       {rows[-1].get('packets_lost_total')}")
