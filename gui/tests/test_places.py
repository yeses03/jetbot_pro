"""Tests for saved named places: storage, validation, map mismatch, and the 'go to place' flow in the window.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import gc
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from jetbot_gui.config import Config
from jetbot_gui.geometry import map_from_message
from jetbot_gui.places import Places, clean_name, map_signature
from test_core import grid_msg, snap
from test_fit import room_map, room_scan

SIG = {"width": 320, "height": 320, "resolution": 0.05, "origin": [-8.0, -8.0]}


class PlacesStorageTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "places.local.json"

    def tearDown(self):
        self.dir.cleanup()

    def test_save_and_load_round_trip(self):
        p = Places(self.path)
        p.put("fridge", (4.5, 1.5, 0.5), SIG)
        p.put("start", (0.0, 0.0, 0.0), SIG)
        p.save()
        q = Places(self.path)
        q.load()
        self.assertEqual(q.names(), ["fridge", "start"])
        self.assertEqual(q.get("fridge"), (4.5, 1.5, 0.5))
        self.assertEqual(q.map_sig, SIG)

    def test_names_are_validated(self):
        for bad in ("", "   ", "../etc", "x" * 40, "a/b", "naughty;name"):
            with self.assertRaises(ValueError, msg=repr(bad)):
                clean_name(bad)
        self.assertEqual(clean_name("  Wardrobe 2  "), "Wardrobe 2")

    def test_non_finite_numbers_are_rejected(self):
        p = Places(self.path)
        for bad in ((float("nan"), 0, 0), (0, float("inf"), 0)):
            with self.assertRaises(ValueError):
                p.put("x", bad, SIG)
        self.assertEqual(p.names(), [])

    def test_places_from_a_different_map_are_flagged_and_cannot_be_mixed(self):
        p = Places(self.path)
        p.put("a", (1, 1, 0), SIG)
        other = dict(SIG, width=640)
        self.assertTrue(p.mismatch(other))
        self.assertFalse(p.mismatch(SIG))
        with self.assertRaises(ValueError):
            p.put("b", (2, 2, 0), other)

    def test_save_is_atomic_and_leaves_no_temp_files(self):
        p = Places(self.path)
        p.put("a", (1, 1, 0), SIG)
        p.save()
        self.assertEqual(sorted(os.listdir(self.dir.name)), ["places.local.json"])
        json.loads(self.path.read_text())

    def test_overwriting_a_name_and_deleting(self):
        p = Places(self.path)
        p.put("a", (1, 1, 0), SIG)
        p.put("a", (2, 2, 0), SIG)
        self.assertEqual(p.get("a"), (2.0, 2.0, 0.0))
        self.assertTrue(p.delete("a"))
        self.assertFalse(p.delete("a"))

    def test_map_signature_is_the_same_for_the_same_grid(self):
        m1 = map_from_message(grid_msg(10, 10, 0.1, (0, 0), 0, {}))
        m2 = map_from_message(grid_msg(10, 10, 0.1, (0, 0), 0, {(1, 1): 100}))
        self.assertEqual(map_signature(m1), map_signature(m2))


class PlacesScopeTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "places.local.json"

    def tearDown(self):
        self.dir.cleanup()

    def test_each_map_has_its_own_places(self):
        p = Places(self.path)
        p.use("kitchen")
        p.put("fridge", (4.5, 1.5, 0.0), SIG)
        p.use("garage")
        self.assertEqual(p.names(), [])                            # a place saved on one map does not exist on another
        p.put("car", (1.0, 1.0, 0.0), SIG)
        p.save()
        q = Places(self.path)
        q.load()
        q.use("kitchen")
        self.assertEqual(q.names(), ["fridge"])
        q.use("garage")
        self.assertEqual(q.names(), ["car"])

    def test_same_shaped_maps_are_still_kept_apart(self):
        p = Places(self.path)
        p.use("a")
        p.put("x", (1, 1, 0), SIG)
        p.use("b")
        p.put("x", (9, 9, 0), SIG)                                 # same signature, different map: independent
        p.use("a")
        self.assertEqual(p.get("x"), (1.0, 1.0, 0.0))

    def test_a_version_1_file_loads_into_the_default_scope(self):
        self.path.write_text(json.dumps({"map": SIG, "places": {"home": [0.5, 0.4, 0.0]}}))
        p = Places(self.path)
        p.load()
        self.assertEqual(p.scope, "default")
        self.assertEqual(p.get("home"), (0.5, 0.4, 0.0))
        p.use("other")
        self.assertEqual(p.names(), [])

    def test_a_corrupt_scope_name_cannot_inject(self):
        p = Places(self.path)
        p.use("../../etc/passwd")                                   # a scope is only a dictionary key, never a path
        p.put("a", (1, 1, 0), SIG)
        p.save()
        self.assertEqual(sorted(os.listdir(self.dir.name)), ["places.local.json"])


class Ev:
    def __init__(self, x, y):
        self.x, self.y = x, y


class PlacesUiTests(unittest.TestCase):
    def _app(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:
            self.skipTest(f"no usable Tk/display: {exc}")
        root.withdraw()
        from jetbot_gui.app import App
        app = App(root, Config(host="127.0.0.1"))
        from test_ui_smoke import stop_background
        stop_background(self, app)
        d = tempfile.TemporaryDirectory()
        app.places = Places(Path(d.name) / "places.local.json")
        return root, app, d

    def _snap(self):
        _, sc = room_scan()
        pose = (0.0, 0.0, 0.0)
        return snap(map=room_map(), map_age=1.0, scan=sc, scan_age=0.1, scan_hz=7.0, pose_map=pose, pose_map_age=0.1,
                    scan_in_map=pose, scan_in_map_age=0.1, nodes={"/amcl", "/move_base"}, amcl_pose=pose,
                    amcl_sigma=(0.05, 0.05), amcl_age=0.2)

    def test_save_a_chosen_destination_then_go_to_it_through_the_normal_gate(self):
        root, app, d = self._app()
        try:
            s = self._snap()
            app.link.snapshot = lambda: s
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            app.pending = (1.2, 0.8, math.pi / 2)
            app.name_var.set("fridge")
            app.save_pending_as_place()
            self.assertEqual(app.places.names(), ["fridge"])
            self.assertEqual(app.place_var.get(), "fridge")
            app.pending = None
            app.tick()
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                app.go_to_place()                                   # MOTION is off: must warn and send nothing
                self.assertEqual(sent, [])
                self.assertTrue(mb.showwarning.called)
                app.set_motion(True)
                app.tick()
                app.go_to_place()                                   # motion on and the gate passes: sends exactly once
            self.assertEqual(sent, [(1.2, 0.8, math.pi / 2)])
            self.assertTrue(app.armed.get())                        # the master switch stays on until you turn it off
        finally:
            root.destroy()
            d.cleanup()

    def test_a_blocked_gate_sends_nothing_even_for_a_saved_place(self):
        root, app, d = self._app()
        try:
            s = self._snap()
            app.link.snapshot = lambda: s
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            app.places.put("a", (1, 1, 0), map_signature(s.map))
            app.place_var.set("a")
            app.set_motion(True)
            app.teleop_on = True                                    # teleop on blocks goals
            app.tick()
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                app.go_to_place()
                self.assertTrue(mb.showwarning.called)
            self.assertEqual(sent, [])
        finally:
            app.teleop_on = False
            root.destroy()
            d.cleanup()

    def test_places_from_another_map_are_not_usable(self):
        root, app, d = self._app()
        try:
            s = self._snap()
            app.link.snapshot = lambda: s
            sent = []
            app.link.send_goal = lambda g: sent.append(g) or True
            app.places.put("a", (1, 1, 0), SIG)                    # saved on a 320x320 map; the robot has the room map
            app.place_var.set("a")
            app.set_motion(True)
            app.tick()
            with mock.patch("jetbot_gui.app.messagebox") as mb:
                app.go_to_place()
                self.assertTrue(mb.showwarning.called)
            self.assertEqual(sent, [])
        finally:
            root.destroy()
            d.cleanup()

    def test_save_robot_pose(self):
        root, app, d = self._app()
        try:
            s = self._snap()
            app.link.snapshot = lambda: s
            app.name_var.set("here")
            app.save_robot_as_place()
            self.assertEqual(app.places.get("here"), (0.0, 0.0, 0.0))
        finally:
            root.destroy()
            d.cleanup()


def tearDownModule():
    # Tk variables from the destroyed windows are cycles; collect them here, on the main thread. Left alone, the collector can
    # run inside a later test's roslibpy reactor thread, where freeing a Tcl object blocks and stalls that test's connection.
    gc.collect()


if __name__ == "__main__":
    unittest.main()
