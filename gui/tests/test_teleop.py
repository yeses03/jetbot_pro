"""Tests for teleop: key mapping, hold timer, hard speed caps, the obstacle guard, and the capped velocity publisher.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from jetbot_gui.config import Config
from jetbot_gui.link import RobotLink
from jetbot_gui.teleop import (HOLD_S, MAX_ANGULAR, MAX_LINEAR, TeleopState, clamp_command, guard)
from test_integration import PORT, FakeRosbridge, wait_for


class KeyTests(unittest.TestCase):
    def test_keys_match_teleop_twist_keyboard(self):
        t = TeleopState(linear=0.2, angular=0.6)
        for key, want in (("i", (0.2, 0.0)), ("comma", (-0.2, 0.0)), ("j", (0.0, 0.6)), ("l", (0.0, -0.6))):
            self.assertTrue(t.key_down(key, 10.0))
            self.assertEqual(t.command(10.1), want, key)

    def test_k_and_space_stop(self):
        t = TeleopState()
        t.key_down("i", 0.0)
        self.assertTrue(t.key_down("k", 0.1))
        self.assertEqual(t.command(0.1), (0.0, 0.0))
        t.key_down("i", 0.2)
        t.key_down("space", 0.3)
        self.assertEqual(t.command(0.3), (0.0, 0.0))

    def test_diagonal_keys_are_not_mapped(self):
        t = TeleopState()
        for key in ("u", "o", "m", "period", "q", "w", "x", "z"):
            self.assertFalse(t.key_down(key, 0.0), key)
            self.assertEqual(t.command(0.0), (0.0, 0.0))

    def test_command_expires_after_the_hold_time(self):
        t = TeleopState(linear=0.2)
        t.key_down("i", 100.0)
        self.assertEqual(t.command(100.0 + HOLD_S - 0.01), (0.2, 0.0))
        self.assertEqual(t.command(100.0 + HOLD_S + 0.01), (0.0, 0.0))          # a lost key-release cannot keep it moving

    def test_a_repeat_event_extends_the_hold(self):
        t = TeleopState(linear=0.2)
        t.key_down("i", 0.0)
        t.key_down("i", 0.6)
        self.assertEqual(t.command(1.2), (0.2, 0.0))

    def test_release_of_the_active_key_stops_but_not_other_keys(self):
        t = TeleopState(linear=0.2)
        t.key_down("i", 0.0)
        t.key_up("j")
        self.assertEqual(t.command(0.1), (0.2, 0.0))
        t.key_up("i")
        self.assertEqual(t.command(0.1), (0.0, 0.0))

    def test_a_new_key_replaces_the_old_one(self):
        t = TeleopState(linear=0.2, angular=0.5)
        t.key_down("i", 0.0)
        t.key_down("j", 0.1)
        self.assertEqual(t.command(0.2), (0.0, 0.5))

    def test_pad_buttons_use_the_same_timer(self):
        t = TeleopState(linear=0.2)
        t.hold((1.0, 0.0), 5.0)
        self.assertEqual(t.command(5.1), (0.2, 0.0))
        t.release()
        self.assertEqual(t.command(5.1), (0.0, 0.0))


class CapTests(unittest.TestCase):
    def test_speed_settings_cannot_exceed_the_caps(self):
        t = TeleopState()
        t.set_speeds(5.0, 9.0)
        self.assertEqual((t.linear, t.angular), (MAX_LINEAR, MAX_ANGULAR))
        t.set_speeds(-1.0, -1.0)
        self.assertEqual((t.linear, t.angular), (0.0, 0.0))

    def test_clamp_command_is_symmetric(self):
        self.assertEqual(clamp_command(9, -9), (MAX_LINEAR, -MAX_ANGULAR))
        self.assertEqual(clamp_command(-9, 9), (-MAX_LINEAR, MAX_ANGULAR))
        self.assertEqual(clamp_command(0.1, 0.1), (0.1, 0.1))


class GuardTests(unittest.TestCase):
    def pts(self, *xy):
        return np.array(xy, float)

    def test_clear_surroundings_allow_everything(self):
        far = self.pts((2.0, 0.0), (0.0, 1.5), (-2.0, 0.0), (0.0, -0.8))
        for vx, wz in ((0.15, 0.0), (-0.15, 0.0), (0.0, 0.5), (0.0, -0.5)):
            self.assertEqual(guard(vx, wz, far)[:2], (vx, wz))

    def test_obstacle_ahead_blocks_forward_only(self):
        p = self.pts((0.30, 0.05), (1.0, 1.0))
        vx, wz, why = guard(0.15, 0.0, p)
        self.assertEqual((vx, wz), (0.0, 0.0))
        self.assertIn("ahead", why)
        self.assertEqual(guard(-0.15, 0.0, p)[:2], (-0.15, 0.0))

    def test_obstacle_behind_blocks_reverse_and_names_the_charger(self):
        p = self.pts((-0.25, 0.0))
        vx, wz, why = guard(-0.15, 0.0, p)
        self.assertEqual((vx, wz), (0.0, 0.0))
        self.assertIn("charger", why)

    def test_wall_beside_the_corridor_does_not_block_driving_straight(self):
        side = self.pts((0.2, 0.31), (0.2, -0.31), (-0.2, 0.31))          # a wall 0.31 m to each side, as at the start line
        self.assertEqual(guard(0.15, 0.0, side)[:2], (0.15, 0.0))

    def test_no_room_to_turn(self):
        p = self.pts((0.10, 0.18))                                         # 0.21 m from the centre
        self.assertEqual(guard(0.0, 0.5, p)[:2], (0.0, 0.0))
        self.assertIn("turn", guard(0.0, 0.5, p)[2])
        self.assertEqual(guard(0.0, 0.5, self.pts((0.3, 0.3)))[:2], (0.0, 0.5))

    def test_no_scan_means_no_motion(self):
        vx, wz, why = guard(0.15, 0.0, None)
        self.assertEqual((vx, wz), (0.0, 0.0))
        self.assertIn("no live LiDAR", why)
        self.assertEqual(guard(0.0, 0.0, None), (0.0, 0.0, ""))             # a stop command is always allowed

    def test_empty_scan_array_is_clear_not_blind(self):
        self.assertEqual(guard(0.15, 0.0, np.zeros((0, 2)))[:2], (0.15, 0.0))


class PublisherTests(unittest.TestCase):
    def test_send_velocity_is_clamped_inside_the_link_and_goes_to_cmd_vel_only(self):
        cfg = Config(host="127.0.0.1", port=PORT, discovery_period_s=1.0)
        server = FakeRosbridge()
        server.start()
        link = RobotLink(cfg)
        try:
            link.connect("127.0.0.1", PORT)
            self.assertTrue(wait_for(lambda: link.snapshot().conn_state == "connected"))
            self.assertTrue(link.send_velocity(5.0, -9.0))                  # absurd request, as a UI bug might make
            self.assertTrue(link.send_velocity(0.0, 0.0))
            self.assertTrue(wait_for(lambda: len(server.client_publishes) >= 2))
            self.assertEqual({m["topic"] for m in server.client_publishes}, {"/cmd_vel"})
            first = server.client_publishes[0]["msg"]
            self.assertEqual(first["linear"]["x"], MAX_LINEAR)
            self.assertEqual(first["angular"]["z"], -MAX_ANGULAR)
            self.assertEqual(first["linear"]["y"], 0.0)
        finally:
            link.disconnect()
            server.stop()

    def test_send_before_connect_returns_false(self):
        self.assertFalse(RobotLink(Config()).send_velocity(0.1, 0.0))


if __name__ == "__main__":
    unittest.main()
