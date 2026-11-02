"""Smoke test: build the real Tk window, feed it a snapshot made from real robot data, run one refresh.

Skipped when Tk or a display is unavailable. Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from jetbot_gui.config import Config
from jetbot_gui.geometry import ScanData
from test_core import snap
from test_fit import FIXTURES, real_map


def regular_scan(points):
    """Bin robot-frame points into a 1-degree regular scan (nearest point per bin)."""
    ranges = np.full(360, np.nan)
    for x, y in points:
        k = int(round(math.degrees(math.atan2(y, x)) + 180.0)) % 360
        r = math.hypot(x, y)
        if not np.isfinite(ranges[k]) or r < ranges[k]:
            ranges[k] = r
    return ScanData("base_footprint", math.radians(-180.0), math.radians(1.0), 0.15, 12.0, ranges)


class UiSmokeTests(unittest.TestCase):
    def test_one_refresh_with_real_data_shows_the_fit_chip(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:                      # no Tk, or no display
            self.skipTest(f"no usable Tk/display: {exc}")
        root.withdraw()
        try:
            from jetbot_gui.app import App
            app = App(root, Config(host="127.0.0.1"))
            with open(os.path.join(FIXTURES, "real_scans.json")) as f:
                scan = json.load(f)["scan_a"]
            x, y, deg = scan["best_pose"]
            pose = (x, y, math.radians(deg))
            s = snap(map=real_map(), map_age=1.0, scan=regular_scan(scan["points"]), scan_age=0.1, scan_hz=7.0,
                     pose_map=pose, pose_map_age=0.1, scan_in_map=pose, scan_in_map_age=0.1,
                     scan_in_base=(0.0, 0.0, 0.0), amcl_pose=pose, amcl_sigma=(0.1, 0.05), amcl_age=0.2)
            app.link.snapshot = lambda: s
            root.update_idletasks()
            app.tick()
            root.update()
            self.assertNotIn("UI error", app.warn_lbl.cget("text"))
            self.assertIn("scan on map walls", app.chips["fit"].cget("text"))
            self.assertIn("SCAN vs MAP", app.pose_txt.cget("text"))
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
