"""Smoke test: build the real Tk window, feed it a snapshot made from real robot data, run one refresh.

Skipped when Tk or a display is unavailable. Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import json
import math
import os
import sys
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from jetbot_gui.config import Config
from jetbot_gui.geometry import ScanData
from test_core import snap
from test_fit import FIXTURES, real_map, room_map, room_scan


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



class Ev:                                                      # minimal stand-in for a Tk mouse event
    def __init__(self, x, y):
        self.x, self.y = x, y


class GoalUiTests(unittest.TestCase):
    def _app(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:
            self.skipTest(f"no usable Tk/display: {exc}")
        root.withdraw()
        from jetbot_gui.app import App
        return root, App(root, Config(host="127.0.0.1"))

    def _snap(self, pose=(0.0, 0.0, 0.0)):
        _, sc = room_scan()
        return snap(map=room_map(), map_age=1.0, scan=sc, scan_age=0.1, scan_hz=7.0, pose_map=pose, pose_map_age=0.1,
                    scan_in_map=pose, scan_in_map_age=0.1, nodes={"/amcl", "/move_base"}, amcl_pose=pose,
                    amcl_sigma=(0.05, 0.05), amcl_age=0.2)

    def test_click_and_drag_set_the_goal_and_the_gate_controls_the_buttons(self):
        root, app = self._app()
        try:
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            s = self._snap()
            app.link.snapshot = lambda: s
            app.goal_mode.set(True)
            root.update_idletasks()
            w, h = app._size()
            app._press(Ev(w // 2 + 100, h // 2))                     # click: heading defaults to robot -> goal
            self.assertIsNotNone(app.pending)
            self.assertAlmostEqual(app.pending[2], 0.0, places=1)
            app._motion(Ev(w // 2 + 100, h // 2 - 100))             # drag upwards: heading becomes ~ +90 deg (map y is up)
            app._release(Ev(0, 0))
            self.assertAlmostEqual(app.pending[2], math.pi / 2, places=1)
            app.tick()
            self.assertIn("not armed", app.block_lbl.cget("text"))
            self.assertTrue(app.send_btn.instate(["disabled"]))
            app.armed.set(True)
            app.tick()
            self.assertEqual(app.block_lbl.cget("text"), "")
            self.assertTrue(app.send_btn.instate(["!disabled"]))
            self.assertEqual(sent, [])                               # nothing is ever sent by ticking alone
        finally:
            root.destroy()

    def test_a_mislocalised_robot_cannot_be_sent_anywhere(self):
        root, app = self._app()
        try:
            wrong = (0.0, 0.0, math.radians(30))
            s = self._snap(pose=wrong)
            app.link.snapshot = lambda: s
            app.pending = (1.0, 1.0, 0.0)
            app.armed.set(True)
            app.tick()
            self.assertIn("does not match", app.block_lbl.cget("text"))
            self.assertTrue(app.send_btn.instate(["disabled"]))
        finally:
            root.destroy()

    def test_cancel_disarms_and_stops_the_route(self):
        root, app = self._app()
        try:
            calls = []
            app.link.cancel_goals = lambda: calls.append("cancel") or True
            app.runner.start([(1.0, 0.0, 0.0)])
            app.armed.set(True)
            app.cancel_all()
            self.assertEqual(calls, ["cancel"])
            self.assertFalse(app.armed.get())
            self.assertFalse(app.runner.running)
        finally:
            root.destroy()



class RelocUiTests(unittest.TestCase):
    """The 'Check pose from scan' / 'Apply to AMCL' flow on a real scan where AMCL was 0.35 m wrong."""

    def _app(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:
            self.skipTest(f"no usable Tk/display: {exc}")
        root.withdraw()
        from jetbot_gui.app import App
        return root, App(root, Config(host="127.0.0.1"))

    def _wait_done(self, root, app, timeout=30.0):
        end = time.time() + timeout
        while time.time() < end and app._reloc and app._reloc["state"] != "done":
            root.update()
            time.sleep(0.05)
        root.update()

    def test_check_then_apply_sends_the_scan_supported_pose(self):
        root, app = self._app()
        try:
            with open(os.path.join(FIXTURES, "real_scans.json")) as f:
                scan = json.load(f)["scan_b"]
            ax, ay, adeg = scan["amcl_pose"]                           # what AMCL wrongly said
            wrong = (ax, ay, math.radians(adeg))
            s = snap(map=real_map(), map_age=1.0, scan=regular_scan(scan["points"]), scan_age=0.1, scan_hz=7.0,
                     pose_map=wrong, pose_map_age=0.1, scan_in_map=wrong, scan_in_map_age=0.1,
                     scan_in_base=(0.0, 0.0, 0.0), nodes={"/amcl", "/move_base"})
            app.link.snapshot = lambda: s
            applied = []
            app.link.set_initial_pose = lambda p: applied.append(p) or True
            app.tick()
            self.assertIn("does not match", app.block_lbl.cget("text") or "does not match")   # gate is blocking at the wrong pose
            app.check_pose()
            self._wait_done(root, app)
            app.tick()
            self.assertIn("scan fits best", app.reloc_txt.cget("text"))
            self.assertTrue(app.apply_btn.instate(["!disabled"]), app.reloc_txt.cget("text"))
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                mb.askyesno.return_value = True
                app.apply_pose()
            self.assertEqual(len(applied), 1)
            x, y, yaw = applied[0]
            tx, ty, tdeg = scan["best_pose"]
            self.assertLess(math.hypot(x - tx, y - ty), 0.12)
            self.assertLess(abs(math.degrees(yaw) - tdeg), 6.0)
        finally:
            root.destroy()

    def test_declining_the_dialog_applies_nothing(self):
        root, app = self._app()
        try:
            with open(os.path.join(FIXTURES, "real_scans.json")) as f:
                scan = json.load(f)["scan_b"]
            ax, ay, adeg = scan["amcl_pose"]
            wrong = (ax, ay, math.radians(adeg))
            s = snap(map=real_map(), map_age=1.0, scan=regular_scan(scan["points"]), scan_age=0.1, scan_hz=7.0,
                     pose_map=wrong, pose_map_age=0.1, scan_in_map=wrong, scan_in_map_age=0.1,
                     scan_in_base=(0.0, 0.0, 0.0), nodes={"/amcl", "/move_base"})
            app.link.snapshot = lambda: s
            applied = []
            app.link.set_initial_pose = lambda p: applied.append(p) or True
            app.check_pose()
            self._wait_done(root, app)
            app.tick()
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                mb.askyesno.return_value = False
                app.apply_pose()
            self.assertEqual(applied, [])
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
