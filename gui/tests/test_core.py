"""Unit tests for the pure logic. Run:  python -m unittest discover -s tests -v   (from gui/)"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from jetbot_gui.config import Config
from jetbot_gui.geometry import (TfBuffer, View, map_from_message, render_map, scan_from_message,
                                 transform_points)
from jetbot_gui.link import CONNECTED, Snapshot
from jetbot_gui.status import BAD, OK, WARN, assess


def tfmsg(parent, child, x, y, yaw):
    return {"header": {"frame_id": parent}, "child_frame_id": child,
            "transform": {"translation": {"x": x, "y": y, "z": 0.0},
                          "rotation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}}


def grid_msg(w, h, res, origin_xy, yaw, cells):
    data = [-1] * (w * h)
    for (col, row), v in cells.items():
        data[row * w + col] = v
    return {"header": {"frame_id": "map"},
            "info": {"width": w, "height": h, "resolution": res,
                     "origin": {"position": {"x": origin_xy[0], "y": origin_xy[1], "z": 0},
                                "orientation": {"x": 0, "y": 0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}},
            "data": data}


class MapTests(unittest.TestCase):
    def test_cell_lands_at_expected_pixel(self):
        m = map_from_message(grid_msg(10, 10, 0.1, (-0.5, -0.5), 0.0, {(7, 2): 100, (2, 7): 0}))
        img = render_map(m, View(0, 0, 100.0), 100, 100)
        # cell (col 7,row 2) centre = world (0.25,-0.25) -> pixel (75,75); y is up so row 2 is BELOW centre
        self.assertEqual(img.getpixel((75, 75)), (15, 15, 15))
        self.assertEqual(img.getpixel((25, 25)), (250, 250, 250))      # free
        self.assertEqual(img.getpixel((60, 40)), (150, 150, 150))      # unknown

    def test_rotated_origin(self):
        m = map_from_message(grid_msg(10, 10, 0.1, (0, 0), math.pi / 2, {(5, 0): 100}))
        img = render_map(m, View(0, 0, 100.0), 200, 200)
        # grid point (0.55,0.05) rotated 90deg = world (-0.05,0.55) -> pixel (95,45)
        self.assertEqual(img.getpixel((95, 45)), (15, 15, 15))
        x, y = m.world_to_cell(-0.05, 0.55)
        self.assertAlmostEqual(x, 5.5)
        self.assertAlmostEqual(y, 0.5)

    def test_zoom_and_pan_keep_point_fixed(self):
        v = View(1.0, 2.0, 50.0)
        before = v.to_world(120, 80, 400, 300)
        v.zoom_at(120, 80, 2.0, 400, 300)
        after = v.to_world(120, 80, 400, 300)
        self.assertAlmostEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], after[1])

    def test_bad_size_rejected(self):
        msg = grid_msg(4, 4, 0.1, (0, 0), 0, {})
        msg["data"] = msg["data"][:-1]
        with self.assertRaises(ValueError):
            map_from_message(msg)


class ScanTests(unittest.TestCase):
    def test_invalid_readings_dropped_not_invented(self):
        msg = {"header": {"frame_id": "/laser_frame"}, "angle_min": 0.0, "angle_increment": math.pi / 2,
               "range_min": 0.15, "range_max": 12.0, "ranges": [1.0, None, "Infinity", 0.05, 20.0, float("nan")]}
        s = scan_from_message(msg)
        self.assertEqual(s.frame_id, "laser_frame")
        pts = s.points()
        self.assertEqual(len(pts), 1)                           # only the 1.0 m return is valid
        np.testing.assert_allclose(pts[0], [1.0, 0.0], atol=1e-9)

    def test_transform(self):
        pts = np.array([[1.0, 0.0]])
        out = transform_points(pts, (1.0, 2.0, math.pi / 2))
        np.testing.assert_allclose(out[0], [1.0, 3.0], atol=1e-9)


class TfTests(unittest.TestCase):
    def setUp(self):
        self.t = [100.0]
        self.tf = TfBuffer(clock=lambda: self.t[0])

    def test_chain_and_orientation(self):
        self.tf.update([tfmsg("map", "odom", 1, 2, math.pi / 2)], False)
        self.tf.update([tfmsg("odom", "/base_footprint", 1, 0, 0)], False)
        self.tf.update([tfmsg("/base_footprint", "laser_frame", 0, 0, math.pi)], True)
        (x, y, th), age = self.tf.lookup("map", "base_footprint")
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 3.0)
        self.assertAlmostEqual(th, math.pi / 2)
        (lx, ly, lth), _ = self.tf.lookup("map", "laser_frame")
        self.assertAlmostEqual(lth, -math.pi / 2)          # 90 deg + 180 deg, wrapped

    def test_missing_link_is_none_and_staleness_tracks_dynamic_only(self):
        self.tf.update([tfmsg("odom", "base_footprint", 0, 0, 0)], False)
        self.assertIsNone(self.tf.lookup("map", "base_footprint"))     # no map->odom: NOT localised
        self.tf.update([tfmsg("base_footprint", "laser_frame", 0, 0, 0)], True)
        self.t[0] += 5.0
        _, age = self.tf.lookup("odom", "laser_frame")
        self.assertAlmostEqual(age, 5.0)                               # static link contributes no age


def snap(**kw):
    base = dict(now=0.0, conn_state=CONNECTED, conn_detail="", host="h", port=9090, last_rx_age=0.1,
                map=None, map_seq=0, map_age=None, scan=None, scan_age=None, scan_hz=0.0,
                pose_map=None, pose_map_age=None, pose_odom=None, pose_odom_age=None, scan_in_map=None,
                scan_in_map_age=None, scan_in_base=None, odom_topic_pose=None, odom_topic_age=None,
                odom_frame="", amcl_pose=None, amcl_sigma=None, amcl_age=None, stats={}, nodes=None,
                topics=None, discovery_error="", tf_frames=[])
    base.update(kw)
    return Snapshot(**base)


class StatusTests(unittest.TestCase):
    cfg = Config()

    def test_odom_only_is_not_a_map_pose(self):
        a = assess(snap(pose_odom=(1, 2, 0), pose_odom_age=0.1), self.cfg)
        self.assertFalse(a.pose_live_in_map)
        self.assertEqual(a.pose.level, WARN)
        self.assertIn("NOT localised", a.pose.text)

    def test_stale_map_pose_not_drawn(self):
        a = assess(snap(pose_map=(0, 0, 0), pose_map_age=10.0), self.cfg)
        self.assertFalse(a.pose_live_in_map)
        self.assertEqual(a.pose.level, BAD)

    def test_live_pose(self):
        a = assess(snap(pose_map=(0, 0, 0), pose_map_age=0.2), self.cfg)
        self.assertTrue(a.pose_live_in_map)
        self.assertEqual(a.pose.level, OK)

    def test_stale_scan_not_live(self):
        from jetbot_gui.geometry import ScanData
        sc = ScanData("laser_frame", 0, 0.1, 0.1, 10, np.array([1.0]))
        a = assess(snap(scan=sc, scan_age=5.0), self.cfg)
        self.assertFalse(a.scan_live)
        self.assertFalse(a.scan_on_map)
        self.assertEqual(a.scan.level, BAD)

    def test_no_data_while_connected_warns(self):
        a = assess(snap(last_rx_age=30.0), self.cfg)
        self.assertEqual(a.conn.level, WARN)

    def test_two_localisers_conflict(self):
        a = assess(snap(nodes={"/amcl", "/slam_gmapping", "/move_base"}), self.cfg)
        self.assertEqual(a.localizer.level, BAD)


if __name__ == "__main__":
    unittest.main()
