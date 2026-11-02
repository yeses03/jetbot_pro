"""Tests for goal sending: message format, the safety gate, the route runner, and the publish whitelist.

Run:  python -m unittest discover -s tests -v   (from gui/; the end-to-end test needs roslibpy + websockets)
"""
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from jetbot_gui.config import Config
from jetbot_gui.geometry import CANCEL_ALL, goal_message, heading_towards
from jetbot_gui.goals import (ABORTED, ACTIVE, SUCCEEDED, RouteRunner, blockers)
from jetbot_gui.link import RobotLink
from jetbot_gui.status import assess
from test_core import snap
from test_fit import room_map, room_scan
from test_integration import PORT, FakeRosbridge, wait_for


def good_snapshot(**over):
    _, sc = room_scan()
    pose = (0.0, 0.0, 0.0)
    kw = dict(map=room_map(), map_age=1.0, scan=sc, scan_age=0.1, scan_hz=7.0, pose_map=pose, pose_map_age=0.1,
              scan_in_map=pose, scan_in_map_age=0.1, nodes={"/amcl", "/move_base"},
              amcl_pose=pose, amcl_sigma=(0.05, 0.05), amcl_age=0.2)
    kw.update(over)
    return snap(**kw)


class MessageTests(unittest.TestCase):
    def test_goal_message_is_a_map_frame_pose_stamped(self):
        m = goal_message((1.5, -2.0, math.pi / 2))
        self.assertEqual(m["header"]["frame_id"], "map")
        self.assertEqual((m["pose"]["position"]["x"], m["pose"]["position"]["y"]), (1.5, -2.0))
        q = m["pose"]["orientation"]
        self.assertAlmostEqual(q["z"], math.sin(math.pi / 4))
        self.assertAlmostEqual(q["w"], math.cos(math.pi / 4))
        self.assertAlmostEqual(q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2 + q["w"] ** 2, 1.0)

    def test_cancel_all_has_empty_id(self):
        self.assertEqual(CANCEL_ALL["id"], "")

    def test_heading_towards(self):
        self.assertAlmostEqual(heading_towards((0, 0), (0, 2)), math.pi / 2)


class GateTests(unittest.TestCase):
    cfg = Config()

    def why(self, armed=True, **over):
        s = good_snapshot(**over)
        return blockers(assess(s, self.cfg), s, self.cfg, armed)

    def test_everything_good_and_armed_is_allowed(self):
        self.assertEqual(self.why(), [])

    def test_not_armed_blocks(self):
        self.assertTrue(any("not armed" in w for w in self.why(armed=False)))

    def test_pose_that_does_not_match_the_scan_blocks(self):
        wrong = (0.0, 0.0, math.radians(30))
        w = self.why(pose_map=wrong, scan_in_map=wrong)
        self.assertTrue(any("does not match" in x for x in w), w)

    def test_no_move_base_blocks(self):
        self.assertTrue(any("move_base" in w for w in self.why(nodes={"/amcl"})))

    def test_uncertain_amcl_blocks(self):
        self.assertTrue(any("AMCL is uncertain" in w for w in self.why(amcl_sigma=(0.6, 0.1))))

    def test_active_goal_blocks_until_it_is_old(self):
        self.assertTrue(any("already active" in w for w in self.why(goal=("g", ACTIVE, ""), goal_age=1.0)))
        self.assertFalse(any("already active" in w for w in self.why(goal=("g", ACTIVE, ""), goal_age=60.0)))
        self.assertFalse(any("already active" in w for w in self.why(goal=("g", SUCCEEDED, ""), goal_age=1.0)))

    def test_disconnected_or_no_pose_blocks(self):
        self.assertTrue(any("not connected" in w for w in self.why(conn_state="disconnected")))
        self.assertTrue(any("no live map-frame pose" in w for w in self.why(pose_map=None, pose_map_age=None)))


class RouteTests(unittest.TestCase):
    L = [(1.0, 0.0, 0.0), (2.0, 1.0, 0.0)]

    def test_two_legs_in_order_and_only_after_success(self):
        r = RouteRunner()
        r.start(self.L)
        t = 0.0
        self.assertEqual(r.step(("old", SUCCEEDED, ""), [], t), self.L[0])              # leg 1 goes out
        self.assertIsNone(r.step(("old", SUCCEEDED, ""), [], t + 1))                   # old goal still newest: waiting
        self.assertIsNone(r.step(("g1", ACTIVE, ""), [], t + 2))                       # leg 1 accepted and driving
        self.assertIsNone(r.step(("g1", ACTIVE, ""), [], t + 3))                       # never sends leg 2 early
        self.assertIsNone(r.step(("g1", SUCCEEDED, "Goal reached."), [], t + 4))       # leg 1 done
        self.assertEqual(r.step(("g1", SUCCEEDED, ""), [], t + 5), self.L[1])          # now leg 2
        r.step(("g2", ACTIVE, ""), [], t + 6)
        r.step(("g2", SUCCEEDED, ""), [], t + 7)
        self.assertEqual(r.state, RouteRunner.DONE)
        self.assertFalse(r.running)

    def test_aborted_leg_stops_the_route_and_never_retries(self):
        r = RouteRunner()
        r.start(self.L)
        r.step(None, [], 0.0)
        r.step(("g1", ACTIVE, ""), [], 1.0)
        r.step(("g1", ABORTED, "Robot is oscillating."), [], 2.0)
        self.assertEqual(r.state, RouteRunner.FAILED)
        self.assertIn("oscillating", r.message)
        self.assertIsNone(r.step(("g1", ABORTED, ""), [], 3.0))

    def test_unaccepted_goal_times_out(self):
        r = RouteRunner(accept_timeout_s=5.0)
        r.start(self.L)
        r.step(("old", SUCCEEDED, ""), [], 0.0)
        r.step(("old", SUCCEEDED, ""), [], 6.0)
        self.assertEqual(r.state, RouteRunner.FAILED)

    def test_a_safety_blocker_stops_the_route_between_legs(self):
        r = RouteRunner()
        r.start(self.L)
        r.step(None, [], 0.0)
        r.step(("g1", ACTIVE, ""), [], 1.0)
        r.step(("g1", SUCCEEDED, ""), [], 2.0)
        self.assertIsNone(r.step(("g1", SUCCEEDED, ""), ["the robot's pose does not match the LiDAR scan"], 3.0))
        self.assertEqual(r.state, RouteRunner.FAILED)
        self.assertIn("leg 2", r.message)

    def test_disarming_stops_the_route(self):
        r = RouteRunner()
        r.start(self.L)
        self.assertIsNone(r.step(None, ["goals are not armed (tick 'Arm goals')"], 0.0))
        self.assertEqual(r.state, RouteRunner.FAILED)

    def test_empty_route_does_nothing(self):
        r = RouteRunner()
        r.start([])
        self.assertFalse(r.running)
        self.assertIsNone(r.step(None, [], 0.0))


class WhitelistTests(unittest.TestCase):
    def test_link_refuses_any_other_topic(self):
        link = RobotLink(Config())
        for topic in ("/cmd_vel", "/cmd_vel_dry", "/move_base_simple/goal_x", "/initialpose_x", "/odom"):
            with self.assertRaises(ValueError):
                link._publish(topic, "geometry_msgs/Twist", {})

    def test_no_velocity_publisher_anywhere_in_the_package(self):
        pkg = os.path.join(os.path.dirname(__file__), "..", "jetbot_gui")
        for name in os.listdir(pkg):
            if name.endswith(".py"):
                with open(os.path.join(pkg, name)) as fh:
                    text = fh.read()
                self.assertNotIn("cmd_vel", text.replace("never publishes velocity", ""), name + " mentions cmd_vel")

    def test_send_before_connect_returns_false(self):
        link = RobotLink(Config())
        self.assertFalse(link.send_goal((1, 1, 0)))
        self.assertFalse(link.cancel_goals())


class EndToEndTests(unittest.TestCase):
    def test_goal_and_cancel_reach_the_robot_and_nothing_else_does(self):
        cfg = Config(host="127.0.0.1", port=PORT, discovery_period_s=1.0)
        server = FakeRosbridge()
        server.start()
        link = RobotLink(cfg)
        try:
            link.connect("127.0.0.1", PORT)
            self.assertTrue(wait_for(lambda: link.snapshot().conn_state == "connected"))
            self.assertTrue(wait_for(lambda: link.snapshot().goal is not None))
            self.assertEqual(link.snapshot().goal[:3], ("g1", SUCCEEDED, "Goal reached."))
            self.assertTrue(link.send_goal((1.0, 2.0, math.pi / 2)))
            self.assertTrue(link.cancel_goals())
            self.assertTrue(wait_for(lambda: len(server.client_publishes) >= 2))
            topics = [m["topic"] for m in server.client_publishes]
            self.assertEqual(sorted(set(topics)), ["/move_base/cancel", "/move_base_simple/goal"])
            goal = next(m for m in server.client_publishes if m["topic"] == "/move_base_simple/goal")["msg"]
            self.assertEqual(goal["header"]["frame_id"], "map")
            self.assertAlmostEqual(goal["pose"]["position"]["y"], 2.0)
            self.assertAlmostEqual(goal["pose"]["orientation"]["w"], math.cos(math.pi / 4))
            cancel = next(m for m in server.client_publishes if m["topic"] == "/move_base/cancel")["msg"]
            self.assertEqual(cancel["id"], "")
        finally:
            link.disconnect()
            server.stop()


if __name__ == "__main__":
    unittest.main()
