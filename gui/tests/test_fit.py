"""Tests for the pose-vs-scan fit indicator, including real data captured on the robot on 2026-11-02.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import dataclasses
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from PIL import Image

from jetbot_gui.config import Config
from jetbot_gui.geometry import ScanData, fit_score, map_from_message, transform_points
from jetbot_gui.status import BAD, IDLE, OK, WARN, assess
from test_core import snap

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def msg(w, h, res, origin, data):
    return {"header": {"frame_id": "map"},
            "info": {"width": w, "height": h, "resolution": res,
                     "origin": {"position": {"x": origin[0], "y": origin[1], "z": 0.0},
                                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "data": data}


def room_map():
    """A 4 x 3 m room, one-cell walls, 0.05 m cells, origin (-2, -1.5)."""
    w, h = 80, 60
    data = [0] * (w * h)
    for c in range(w):
        data[c] = data[(h - 1) * w + c] = 100
    for r in range(h):
        data[r * w] = data[r * w + w - 1] = 100
    return map_from_message(msg(w, h, 0.05, (-2.0, -1.5), data))


def room_scan():
    """Rays cast from the room centre (0, 0) to the wall cell centres. Returns (points, ScanData)."""
    pts, ranges = [], []
    for deg in range(360):
        a = math.radians(deg)
        dx, dy = math.cos(a), math.sin(a)
        tx = (1.975 if dx > 0 else -1.975) / dx if abs(dx) > 1e-9 else 1e9
        ty = (1.475 if dy > 0 else -1.475) / dy if abs(dy) > 1e-9 else 1e9
        t = min(tx, ty)
        pts.append((t * dx, t * dy))
        ranges.append(t)
    return np.array(pts), ScanData("laser_frame", 0.0, math.radians(1.0), 0.15, 12.0, np.array(ranges))


def real_map():
    img = np.array(Image.open(os.path.join(FIXTURES, "livingroom_start_cleaned.png")))
    occ = np.where(img == 0, 100, np.where(img == 205, -1, 0)).astype(int)
    return map_from_message(msg(320, 320, 0.05, (-8.0, -8.0), np.flipud(occ).flatten().tolist()))


def pose_pts(pts, pose):
    x, y, deg = pose
    return transform_points(np.asarray(pts, float), (x, y, math.radians(deg)))


class SyntheticFitTests(unittest.TestCase):
    def test_true_pose_fits_almost_perfectly(self):
        pts, _ = room_scan()
        self.assertGreater(fit_score(room_map(), pts), 0.95)

    def test_shifted_pose_fits_worse(self):
        pts, _ = room_scan()
        self.assertLess(fit_score(room_map(), pose_pts(pts, (0.3, 0.0, 0.0))), 0.70)

    def test_rotated_pose_fits_much_worse(self):
        pts, _ = room_scan()
        self.assertLess(fit_score(room_map(), pose_pts(pts, (0.0, 0.0, 30.0))), 0.50)

    def test_points_outside_the_map_count_as_misses(self):
        pts, _ = room_scan()
        self.assertEqual(fit_score(room_map(), pose_pts(pts, (50.0, 50.0, 0.0))), 0.0)

    def test_too_few_points_or_no_wall_data_gives_none(self):
        m = room_map()
        self.assertIsNone(fit_score(m, np.zeros((10, 2))))
        self.assertIsNone(fit_score(dataclasses.replace(m, occ=None), room_scan()[0]))

    def test_distance_transform_is_cached_and_capped(self):
        m = room_map()
        d = m.dist_to_wall()
        self.assertIs(d, m.dist_to_wall())
        self.assertEqual(int(d[30, 40]), 12)            # room centre is far from every wall
        self.assertEqual(int(d[0, 40]), 0)              # a wall cell


class RealDataTests(unittest.TestCase):
    """Scans captured on the JetBot, with the pose AMCL reported and the pose the scan-to-map search found."""

    @classmethod
    def setUpClass(cls):
        cls.map = real_map()
        with open(os.path.join(FIXTURES, "real_scans.json")) as f:
            cls.scans = json.load(f)

    def score(self, name, pose):
        return fit_score(self.map, pose_pts(self.scans[name]["points"], pose))

    def test_true_poses_fit_well(self):
        for name in ("scan_a", "scan_b", "scan_c"):
            self.assertGreater(self.score(name, self.scans[name]["best_pose"]), 0.80, name)

    def test_amcl_x_error_of_0_35_m_is_flagged(self):
        # AMCL said x=1.80; the scan only fits at x~1.45
        self.assertLess(self.score("scan_b", self.scans["scan_b"]["amcl_pose"]), Config().fit_warn)

    def test_amcl_heading_error_of_30_deg_is_flagged(self):
        # AMCL said yaw 161 deg; the scan fits at ~194 deg
        self.assertLess(self.score("scan_c", self.scans["scan_c"]["amcl_pose"]), 0.5)

    def test_pose_after_reseeding_is_acceptable(self):
        self.assertGreater(self.score("scan_a", self.scans["scan_a"]["amcl_pose"]), Config().fit_ok)


class StatusFitTests(unittest.TestCase):
    cfg = Config()

    def _assess(self, laser_pose):
        _, sc = room_scan()
        return assess(snap(map=room_map(), map_age=1.0, scan=sc, scan_age=0.1, scan_hz=7.0,
                           pose_map=laser_pose, pose_map_age=0.1, scan_in_map=laser_pose, scan_in_map_age=0.1),
                      self.cfg)

    def test_good_pose_is_ok(self):
        a = self._assess((0.0, 0.0, 0.0))
        self.assertEqual(a.fit.level, OK)
        self.assertGreater(a.fit_value, 0.95)

    def test_wrong_pose_is_bad_and_explained(self):
        a = self._assess((0.0, 0.0, math.radians(30)))
        self.assertEqual(a.fit.level, BAD)
        self.assertTrue(any("does NOT match the map" in w for w in a.warnings))

    def test_no_scan_means_not_applicable(self):
        a = assess(snap(map=room_map(), map_age=1.0), self.cfg)
        self.assertEqual(a.fit.level, IDLE)
        self.assertIsNone(a.fit_value)

    def test_uncertain_amcl_warns(self):
        a = assess(snap(amcl_pose=(0, 0, 0), amcl_sigma=(0.5, 0.1), amcl_age=0.2), self.cfg)
        self.assertTrue(any("AMCL is uncertain" in w for w in a.warnings))
        b = assess(snap(amcl_pose=(0, 0, 0), amcl_sigma=(0.1, 0.1), amcl_age=0.2), self.cfg)
        self.assertFalse(any("AMCL is uncertain" in w for w in b.warnings))


if __name__ == "__main__":
    unittest.main()
