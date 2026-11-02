#!/usr/bin/env python3
"""Ground-truth check of a robot pose: slide a live scan over a saved map and report where it fits best.

    python3 tools/scan_map_fit.py --map ~/jetbot_maps/livingroom_20261102_0830_start_cleaned.yaml \
        --scan scan.json --guess 1.45 0.15 0          # guess = the pose AMCL/odometry reports (x, y, yaw degrees)

Prints the fit at the guess and the best pose found nearby. "Fit" = share of scan points within one map cell of a wall:
about 0.86 at a true pose on the living-room map, 0.36-0.57 at poses 30 degrees / 0.35 m off. The search is coarse
(2.5 cm, 2 degrees) and can be fooled in long featureless corridors, so treat a best pose that is far from the guess
as "something is wrong", not as an exact measurement. Needs numpy and pillow only. Read-only: it never touches the robot.
"""
import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image


def load_map(yaml_path):
    y = Path(yaml_path).expanduser()
    text = y.read_text()
    res = float(re.search(r"^resolution:\s*([-\d.eE]+)", text, re.M).group(1))
    ox, oy = (float(v) for v in re.search(r"^origin:\s*\[([^\]]+)\]", text, re.M).group(1).split(",")[:2])
    image = re.search(r"^image:\s*(\S+)", text, re.M).group(1)
    img_path = Path(image)
    if not img_path.is_absolute() or not img_path.exists():
        img_path = y.parent / img_path.name
    pgm = np.array(Image.open(img_path))
    occ = np.flipud(pgm == 0)                                   # row 0 = bottom (map y up)
    d = np.full(occ.shape, 12, np.uint8)
    d[occ] = 0
    cur = occ.copy()
    for k in range(1, 13):
        nxt = cur.copy()
        nxt[1:, :] |= cur[:-1, :]
        nxt[:-1, :] |= cur[1:, :]
        nxt[:, 1:] |= cur[:, :-1]
        nxt[:, :-1] |= cur[:, 1:]
        d[nxt & ~cur] = k
        cur = nxt
    return d, res, ox, oy


def fit(d, res, ox, oy, pts, x, y, yaw_deg):
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)
    wx, wy = x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]
    ci, ri = np.floor((wx - ox) / res).astype(int), np.floor((wy - oy) / res).astype(int)
    ok = (ci >= 0) & (ci < d.shape[1]) & (ri >= 0) & (ri < d.shape[0])
    hit = np.zeros(len(pts), bool)
    hit[ok] = d[ri[ok], ci[ok]] <= 1
    return float(hit.mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="map .yaml (its .pgm is found next to it)")
    ap.add_argument("--scan", required=True, help="JSON list of [x, y] robot-frame points (tools/scan_dump.py)")
    ap.add_argument("--guess", nargs=3, type=float, required=True, metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--radius", type=float, default=0.6, help="search +/- this many metres in x and y (default 0.6)")
    ap.add_argument("--yaw-range", type=float, default=40.0, help="search +/- this many degrees (default 40)")
    a = ap.parse_args()
    d, res, ox, oy = load_map(a.map)
    pts = np.array(json.loads(Path(a.scan).read_text()), float)
    gx, gy, gyaw = a.guess
    print("scan points: %d   fit at the guess (%.3f, %.3f, %.0f deg): %.3f" % (len(pts), gx, gy, gyaw, fit(d, res, ox, oy, pts, gx, gy, gyaw)))
    best = max(((fit(d, res, ox, oy, pts, x, y, yw), x, y, yw)
                for x in np.arange(gx - a.radius, gx + a.radius + 1e-9, 0.025)
                for y in np.arange(gy - a.radius, gy + a.radius + 1e-9, 0.025)
                for yw in np.arange(gyaw - a.yaw_range, gyaw + a.yaw_range + 1e-9, 2.0)), key=lambda t: t[0])
    print("best pose nearby: x %.3f  y %.3f  yaw %.0f deg   fit %.3f   (offset from guess: %.2f m, %.0f deg)" % (
        best[1], best[2], best[3], best[0], math.hypot(best[1] - gx, best[2] - gy), best[3] - gyaw))


if __name__ == "__main__":
    main()
