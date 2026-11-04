"""Tests for relocalisation: the scan-to-map pose search (on real scans captured on the robot) and the /initialpose message.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import json
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from jetbot_gui.config import Config
from jetbot_gui.geometry import fit_score, initial_pose_message, pose_search, transform_points
from jetbot_gui.link import RobotLink
from test_fit import FIXTURES, pose_pts, real_map, room_map, room_scan
from test_integration import FakeRosbridge, free_port, wait_for


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class PoseSearchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map = real_map()
        with open(os.path.join(FIXTURES, "real_scans.json")) as f:
            cls.scans = json.load(f)

    def true_pose(self, name):
        x, y, deg = self.scans[name]["best_pose"]
        return x, y, math.radians(deg)

    def test_recovers_the_pose_when_amcl_was_0_35_m_off(self):
        # the real failure: AMCL said x=1.80 while the scan fits at x~1.45
        pts = np.array(self.scans["scan_b"]["points"])
        ax, ay, adeg = self.scans["scan_b"]["amcl_pose"]
        got = pose_search(self.map, pts, centre=(ax, ay, math.radians(adeg)), radius=1.0)
        score, x, y, yaw = got[0]
        tx, ty, tyaw = self.true_pose("scan_b")
        self.assertLess(math.hypot(x - tx, y - ty), 0.08)
        self.assertLess(angle_diff(yaw, tyaw), math.radians(5))
        self.assertGreater(score, 0.80)

    def test_recovers_the_pose_when_amcl_heading_was_30_deg_off(self):
        pts = np.array(self.scans["scan_c"]["points"])
        ax, ay, adeg = self.scans["scan_c"]["amcl_pose"]
        got = pose_search(self.map, pts, centre=(ax, ay, math.radians(adeg)), radius=0.6, yaw_range=math.radians(60))
        score, x, y, yaw = got[0]
        tx, ty, tyaw = self.true_pose("scan_c")
        self.assertLess(math.hypot(x - tx, y - ty), 0.10)
        self.assertLess(angle_diff(yaw, tyaw), math.radians(6))

    def test_whole_room_search_finds_the_pose_without_any_guess(self):
        # the robot was carried by hand: there is no usable guess, search everywhere
        for name in ("scan_a", "scan_c"):
            pts = np.array(self.scans[name]["points"])
            t0 = time.time()
            got = pose_search(self.map, pts, centre=None)
            took = time.time() - t0
            score, x, y, yaw = got[0]
            tx, ty, tyaw = self.true_pose(name)
            self.assertLess(math.hypot(x - tx, y - ty), 0.15, name)
            self.assertLess(angle_diff(yaw, tyaw), math.radians(8), name)
            self.assertLess(took, 20.0, f"{name}: whole-room search took {took:.1f}s")

    def test_a_scan_that_fits_nowhere_scores_low(self):
        rng = np.random.RandomState(1)
        junk = rng.uniform(-3, 3, size=(300, 2))
        got = pose_search(self.map, junk, centre=None)
        self.assertTrue(not got or got[0][0] < 0.5)

    def test_too_few_points_gives_nothing(self):
        self.assertEqual(pose_search(self.map, np.zeros((10, 2)), centre=(0, 0, 0)), [])

    def test_symmetric_room_reports_an_ambiguous_second_candidate(self):
        # a plain rectangle looks the same rotated by 180 degrees, so the search must not claim certainty
        pts, _ = room_scan()
        got = pose_search(room_map(), pts, centre=None)
        self.assertEqual(len(got), 2)
        self.assertGreater(got[1][0], got[0][0] - 0.05)


class InitialPoseTests(unittest.TestCase):
    def test_message_is_a_map_frame_pose_with_a_covariance(self):
        m = initial_pose_message((1.0, 2.0, math.pi / 2))
        self.assertEqual(m["header"]["frame_id"], "map")
        self.assertEqual(m["pose"]["pose"]["position"]["x"], 1.0)
        self.assertAlmostEqual(m["pose"]["pose"]["orientation"]["w"], math.cos(math.pi / 4))
        cov = m["pose"]["covariance"]
        self.assertEqual(len(cov), 36)
        self.assertAlmostEqual(cov[0], 0.05 ** 2)
        self.assertAlmostEqual(cov[35], 0.05 ** 2)

    def test_it_reaches_the_robot_on_the_initialpose_topic_only(self):
        port = free_port()
        cfg = Config(host="127.0.0.1", port=port, discovery_period_s=1.0)
        server = FakeRosbridge(port)
        server.start()
        link = RobotLink(cfg)
        try:
            link.connect("127.0.0.1", port)
            self.assertTrue(wait_for(lambda: link.snapshot().conn_state == "connected"))
            self.assertTrue(link.set_initial_pose((3.0, 1.7, 0.0)))
            self.assertTrue(wait_for(lambda: len(server.client_publishes) >= 1))
            self.assertEqual({m["topic"] for m in server.client_publishes}, {"/initialpose"})
            msg = server.client_publishes[0]["msg"]
            self.assertAlmostEqual(msg["pose"]["pose"]["position"]["x"], 3.0)
        finally:
            link.disconnect()
            server.stop()


if __name__ == "__main__":
    unittest.main()
