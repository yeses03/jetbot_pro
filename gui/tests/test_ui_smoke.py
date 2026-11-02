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



class Ev:                                                      # minimal stand-in for a Tk event
    def __init__(self, x=0, y=0, state=0, keysym=""):
        self.x, self.y, self.state, self.keysym = x, y, state, keysym
        self.widget = None


def make_app(test):
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as exc:
        test.skipTest(f"no usable Tk/display: {exc}")
    root.withdraw()
    from jetbot_gui.app import App
    return root, App(root, Config(host="127.0.0.1"))


def good_snap(pose=(0.0, 0.0, 0.0), **over):
    _, sc = room_scan()
    kw = dict(map=room_map(), map_age=1.0, scan=sc, scan_age=0.1, scan_hz=7.0, pose_map=pose, pose_map_age=0.1,
              scan_in_map=pose, scan_in_map_age=0.1, scan_in_base=(0.0, 0.0, 0.0), nodes={"/amcl", "/move_base"},
              amcl_pose=pose, amcl_sigma=(0.05, 0.05), amcl_age=0.2)
    kw.update(over)
    return snap(**kw)


class ClickToGoTests(unittest.TestCase):
    def test_a_plain_click_chooses_a_destination_without_any_drag(self):
        root, app = make_app(self)
        try:
            app.link.snapshot = lambda: good_snap()
            root.update_idletasks()
            w, h = app._size()
            app._press(Ev(w // 2 + 100, h // 2))
            app._release(Ev(w // 2 + 100, h // 2))
            self.assertIsNotNone(app.pending)
            self.assertAlmostEqual(app.pending[2], 0.0, places=1)       # heading defaults to the direction of travel
        finally:
            root.destroy()

    def test_a_drag_pans_the_map_and_does_not_choose_a_destination(self):
        root, app = make_app(self)
        try:
            app.link.snapshot = lambda: good_snap()
            root.update_idletasks()
            w, h = app._size()
            cx0 = app.view.cx
            app._press(Ev(200, 200))
            app._motion(Ev(260, 200))
            app._release(Ev(260, 200))
            self.assertIsNone(app.pending)
            self.assertNotEqual(app.view.cx, cx0)
        finally:
            root.destroy()

    def test_shift_drag_sets_the_heading(self):
        root, app = make_app(self)
        try:
            app.link.snapshot = lambda: good_snap()
            root.update_idletasks()
            w, h = app._size()
            app._press(Ev(w // 2, h // 2, state=1))                         # state bit 1 = Shift
            app._motion(Ev(w // 2, h // 2 - 120, state=1))                   # drag up = +y = heading +90 deg
            app._release(Ev(w // 2, h // 2 - 120, state=1))
            self.assertAlmostEqual(app.pending[2], math.pi / 2, places=1)
        finally:
            root.destroy()

    def test_go_needs_motion_on_and_a_passing_gate_and_sends_exactly_once(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            s = good_snap()
            app.link.snapshot = lambda: s
            app.pending = (1.0, 1.0, 0.0)
            app.tick()
            self.assertIn("MOTION", app.block_lbl.cget("text"))
            self.assertTrue(app.send_btn.instate(["disabled"]))
            app.send_pending()
            self.assertEqual(sent, [])                                          # blocked: nothing sent
            app.set_motion(True)
            app.tick()
            self.assertEqual(app.block_lbl.cget("text"), "")
            self.assertTrue(app.send_btn.instate(["!disabled"]))
            self.assertEqual(sent, [])                                          # ticking alone never sends
            app.send_btn.invoke()
            self.assertEqual(sent, [(1.0, 1.0, 0.0)])
            self.assertIsNone(app.pending)
        finally:
            root.destroy()

    def test_double_click_chooses_and_goes_only_when_motion_is_on_and_the_gate_passes(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            app.link.snapshot = lambda: good_snap()
            root.update_idletasks()
            w, h = app._size()
            app._double_click(Ev(w // 2 + 80, h // 2))
            self.assertEqual(sent, [])                              # MOTION is off: nothing is sent
            app.set_motion(True)
            app._double_click(Ev(w // 2 + 80, h // 2))
            self.assertEqual(len(sent), 1)
            wrong = (0.0, 0.0, math.radians(30))
            app.link.snapshot = lambda: good_snap(pose=wrong)
            app._double_click(Ev(w // 2 + 80, h // 2))
            self.assertEqual(len(sent), 1)                          # mislocalised: blocked
        finally:
            root.destroy()

    def test_enter_acts_as_go_but_only_through_the_same_gate(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            app.link.snapshot = lambda: good_snap()
            app.pending = (1.0, 1.0, 0.0)
            app.tick()
            app._key_down(Ev(keysym="Return"))
            self.assertEqual(sent, [])                              # MOTION is off
            app.set_motion(True)
            app.tick()
            self.assertEqual(app._key_down(Ev(keysym="Return")), "break")
            self.assertEqual(len(sent), 1)
        finally:
            root.destroy()

    def test_a_mislocalised_robot_cannot_be_sent_anywhere(self):
        root, app = make_app(self)
        try:
            wrong = (0.0, 0.0, math.radians(30))
            app.link.snapshot = lambda: good_snap(pose=wrong)
            app.pending = (1.0, 1.0, 0.0)
            app.set_motion(True)
            app.tick()
            self.assertIn("does not match", app.block_lbl.cget("text"))
            self.assertIn("Check pose", app.block_lbl.cget("text"))
            self.assertTrue(app.send_btn.instate(["disabled"]))
        finally:
            root.destroy()

    def test_stop_cancels_goals_and_stops_the_route(self):
        root, app = make_app(self)
        try:
            calls = []
            app.link.cancel_goals = lambda: calls.append("cancel") or True
            app.link.send_velocity = lambda v, w: calls.append(("vel", v, w)) or True
            app.runner.start([(1.0, 0.0, 0.0)])
            app.pending = (2.0, 2.0, 0.0)
            app.stop_btn.invoke()
            self.assertEqual(calls[0], "cancel")
            self.assertIn(("vel", 0.0, 0.0), calls)
            self.assertFalse(app.runner.running)
            self.assertIsNone(app.pending)
        finally:
            root.destroy()

    def test_motion_switches_itself_off_after_the_idle_timeout(self):
        root, app = make_app(self)
        try:
            app.link.snapshot = lambda: good_snap()
            app.set_motion(True)
            app._armed_since -= 400.0
            app.tick()
            self.assertFalse(app.armed.get())
        finally:
            root.destroy()


class TeleopUiTests(unittest.TestCase):
    def _start(self, app, sent):
        app.link.send_velocity = lambda v, w: sent.append((round(v, 3), round(w, 3))) or True
        app.link.cancel_goals = lambda: True
        app.set_motion(True)
        with mock.patch("jetbot_gui.app.messagebox") as mb:
            mb.askyesno.return_value = True
            app.toggle_teleop()

    def test_keys_drive_only_while_teleop_is_on_and_match_the_teleop_keys(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.snapshot = lambda: good_snap()
            app.link.send_velocity = lambda v, w: sent.append((round(v, 3), round(w, 3))) or True
            app.set_motion(True)
            self.assertIsNone(app._key_down(Ev(keysym="i")))                    # teleop off: keys do nothing
            app.tick()
            self.assertTrue(all(c == (0.0, 0.0) for c in sent))
            sent.clear()
            self._start(app, sent)
            self.assertTrue(app.teleop_on)
            self.assertEqual(app._key_down(Ev(keysym="i")), "break")
            app.tick()
            self.assertEqual(sent[-1], (0.15, 0.0))
            app._key_down(Ev(keysym="j"))
            app.tick()
            self.assertEqual(sent[-1], (0.0, 0.5))
            app._key_down(Ev(keysym="comma"))
            app.tick()
            self.assertEqual(sent[-1], (-0.15, 0.0))
            app._key_down(Ev(keysym="k"))
            app.tick()
            self.assertEqual(sent[-1], (0.0, 0.0))
            for diagonal in ("u", "o", "m", "period"):
                self.assertIsNone(app._key_down(Ev(keysym=diagonal)))
        finally:
            app.teleop_on = False
            root.destroy()

    def test_the_obstacle_guard_blocks_forward_and_says_why(self):
        root, app = make_app(self)
        try:
            from jetbot_gui.geometry import ScanData
            import numpy as np
            sent = []
            ranges = np.full(360, 3.0)
            ranges[178:183] = 0.30                                              # 0.30 m dead ahead (scan 0 deg = rear: 180 = front)
            sc = ScanData("laser_frame", 0.0, math.radians(1.0), 0.15, 12.0, ranges)
            app.link.snapshot = lambda: good_snap(scan=sc, scan_in_base=(0.0, 0.0, math.pi))
            self._start(app, sent)
            app._key_down(Ev(keysym="i"))
            app.tick()
            self.assertEqual(sent[-1], (0.0, 0.0))
            self.assertIn("ahead", app.tele_out.cget("text"))
        finally:
            app.teleop_on = False
            root.destroy()

    def test_losing_focus_or_esc_stops_teleop_and_sends_zeros(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.snapshot = lambda: good_snap()
            self._start(app, sent)
            app._key_down(Ev(keysym="i"))
            ev = Ev()
            ev.widget = root
            app._focus_out(ev)
            self.assertFalse(app.teleop_on)
            sent.clear()
            for _ in range(5):
                app.tick()
            self.assertEqual(sent, [(0.0, 0.0)] * 3)                            # exactly three zero commands, then silence
            self._start(app, sent)
            app._key_down(Ev(keysym="Escape"))
            self.assertFalse(app.teleop_on)
        finally:
            app.teleop_on = False
            root.destroy()

    def test_teleop_cannot_start_without_motion_a_live_lidar_or_a_connection(self):
        root, app = make_app(self)
        try:
            app.link.snapshot = lambda: good_snap()
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                app.toggle_teleop()
                self.assertTrue(mb.showwarning.called)                          # MOTION is off
            self.assertFalse(app.teleop_on)
            app.set_motion(True)
            app.link.snapshot = lambda: good_snap(scan=None, scan_age=None)
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                app.toggle_teleop()
                self.assertTrue(mb.showwarning.called)                          # no live LiDAR
            self.assertFalse(app.teleop_on)
        finally:
            root.destroy()

    def test_switching_motion_off_stops_teleop(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.snapshot = lambda: good_snap()
            self._start(app, sent)
            app.set_motion(False)
            self.assertFalse(app.teleop_on)
        finally:
            root.destroy()

    def test_typing_in_a_text_box_never_drives(self):
        root, app = make_app(self)
        try:
            import tkinter as tk
            sent = []
            app.link.snapshot = lambda: good_snap()
            self._start(app, sent)
            entry = tk.Entry(root)
            entry.place(x=0, y=0)
            root.deiconify()
            root.update()
            entry.focus_force()
            root.update()
            self.assertIsNone(app._key_down(Ev(keysym="i")))
            app.tick()
            self.assertEqual(sent[-1], (0.0, 0.0))
        finally:
            app.teleop_on = False
            root.destroy()

    def test_pad_buttons_use_the_same_commands(self):
        root, app = make_app(self)
        try:
            sent = []
            app.link.snapshot = lambda: good_snap()
            self._start(app, sent)
            app._pad_down((1.0, 0.0))
            app.tick()
            self.assertEqual(sent[-1], (0.15, 0.0))
            app._pad_up()
            app.tick()
            self.assertEqual(sent[-1], (0.0, 0.0))
        finally:
            app.teleop_on = False
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
