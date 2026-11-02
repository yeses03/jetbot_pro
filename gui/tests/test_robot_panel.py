"""Tests for the Robot tab (software start/stop, map library, save) and the connect flow (SSH tunnel), driving the real window with a fake manager.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import io
import os
import sys
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from PIL import Image

from jetbot_gui.manager_client import ManagerSnapshot
from jetbot_gui.robot_panel import fmt_age, preview_image
from jetbot_gui.tunnel import TunnelError
from test_ui_smoke import make_app

MAPS = [{"name": "kitchen", "width": 320, "height": 320, "resolution": 0.05, "origin": [-8, -8, 0], "bytes": 1, "modified": 1700000000, "active": False},
        {"name": "garage", "width": 160, "height": 120, "resolution": 0.05, "origin": [-4, -3, 0], "bytes": 1, "modified": 1700000500, "active": True}]


def snap_of(mode="idle", maps=None, busy="", connected=True, trash=None, active="garage", fw=True, error=""):
    st = {"mode": mode, "ros_master": True, "rosbridge": True, "firewall": {"ok": fw, "detail": ""}, "active_map": active,
          "host": {"uptime_s": 7300, "load1": 1.5, "disk_free_gb": 7.2}, "last_exit": ""} if connected else None
    return ManagerSnapshot(connected=connected, status=st, maps=list(MAPS if maps is None else maps), trash=trash or [], active=active,
                           busy=busy, error=error)


def prep(test):
    root, app = make_app(test)
    calls = []
    app.mgr.act = lambda label, method, path, body=None, done=None: calls.append((label, method, path, body, done)) or True
    app.mgr.enabled_override = True
    return root, app, calls


class RobotTabTests(unittest.TestCase):
    def feed(self, app, ms, enabled=True):
        app.mgr.snapshot = lambda: ms
        type(app.mgr).enabled = property(lambda self: enabled)     # patched per test; restored in tearDown
        app.robot.update(ms)

    def tearDown(self):
        from jetbot_gui.manager_client import ManagerLink
        ManagerLink.enabled = property(lambda self: self._port is not None)

    def test_titles_for_each_state(self):
        root, app, _ = prep(self)
        try:
            for ms, enabled, want in ((snap_of(connected=False), False, "NOT CONNECTED"), (snap_of(connected=False, error="x"), True, "UNREACHABLE"),
                                      (snap_of("idle"), True, "STOPPED"), (snap_of("navigation"), True, "NAVIGATION"),
                                      (snap_of("mapping"), True, "MAPPING")):
                self.feed(app, ms, enabled)
                self.assertIn(want, app.robot.mode_lbl.cget("text"))
        finally:
            root.destroy()

    def test_buttons_follow_the_state(self):
        root, app, _ = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            self.assertTrue(app.robot.nav_btn.instate(["!disabled"]))          # a map is selected by default
            self.assertTrue(app.robot.map_btn.instate(["!disabled"]))
            self.assertTrue(app.robot.stop_sw_btn.instate(["disabled"]))       # nothing to stop
            self.assertTrue(app.robot.save_btn.instate(["disabled"]))          # not mapping
            self.feed(app, snap_of("mapping"))
            self.assertTrue(app.robot.stop_sw_btn.instate(["!disabled"]))
            self.assertTrue(app.robot.save_btn.instate(["!disabled"]))
            self.feed(app, snap_of("mapping", busy="start mapping"))
            for b in (app.robot.nav_btn, app.robot.map_btn, app.robot.stop_sw_btn, app.robot.save_btn):
                self.assertTrue(b.instate(["disabled"]))                       # one action at a time
            self.feed(app, snap_of(connected=False), enabled=False)
            self.assertTrue(app.robot.nav_btn.instate(["disabled"]))
            self.feed(app, snap_of("idle", maps=[]))
            self.assertTrue(app.robot.nav_btn.instate(["disabled"]))           # no map to navigate on
        finally:
            root.destroy()

    def test_start_navigation_needs_a_confirmation_and_replaces_whatever_runs(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = False
                app.robot.start_navigation()
                self.assertEqual(calls, [])
                mb.askyesno.return_value = True
                app.robot.start_navigation()
            self.assertEqual(len(calls), 1)
            label, method, path, body, _ = calls[0]
            self.assertEqual((method, path), ("POST", "/start"))
            self.assertEqual(body, {"mode": "navigation", "map": "garage", "replace": True})     # the active map was pre-selected
        finally:
            root.destroy()

    def test_selecting_a_different_map_changes_what_starts(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            app.robot.tree.selection_set("kitchen")
            app.robot._on_select()
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = True
                app.robot.start_navigation()
            self.assertEqual(calls[0][3]["map"], "kitchen")
        finally:
            root.destroy()

    def test_start_mapping_and_stop_need_confirmation(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("navigation"))
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = False
                app.robot.start_mapping()
                app.robot.stop_software()
                self.assertEqual(calls, [])
                mb.askyesno.return_value = True
                app.robot.start_mapping()
                app.robot.stop_software()
            self.assertEqual(calls[0][3], {"mode": "mapping", "replace": True})
            self.assertEqual(calls[1][2], "/stop")
        finally:
            root.destroy()

    def test_save_validates_the_name_and_never_offers_an_existing_one(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("mapping"))
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                for bad in ("", "has space", "../x", "x" * 60, "a;b"):
                    app.robot.name_var.set(bad)
                    app.robot.save_map()
                app.robot.name_var.set("kitchen")                             # already exists
                app.robot.save_map()
                self.assertEqual(mb.showwarning.call_count, 6)
            self.assertEqual(calls, [])
            app.robot.name_var.set("new_room")
            app.robot.save_map()
            self.assertEqual(calls[0][2:4], ("/maps/save", {"name": "new_room"}))
        finally:
            root.destroy()

    def test_after_a_save_it_offers_to_switch_to_navigation(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("mapping"))
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = True
                app.robot._after_save("new_room")
            self.assertEqual(calls[0][3], {"mode": "navigation", "map": "new_room", "replace": True})
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = False
                app.robot._after_save("other")
            self.assertEqual(len(calls), 1)
        finally:
            root.destroy()

    def test_delete_is_confirmed_and_names_the_trash(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            with mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                mb.askyesno.return_value = False
                app.robot.delete_map()
                self.assertEqual(calls, [])
                mb.askyesno.return_value = True
                app.robot.delete_map()
                self.assertIn("trash", mb.askyesno.call_args[0][1].lower())
            self.assertEqual(calls[0][2:4], ("/maps/delete", {"name": "garage"}))
        finally:
            root.destroy()

    def test_restore_uses_the_chosen_trash_entry(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("idle", trash=[{"entry": "old__20260101-101010", "name": "old", "when": "20260101-101010"}]))
            with mock.patch("jetbot_gui.robot_panel.messagebox"):
                app.robot.restore_map()                                       # nothing chosen yet
                self.assertEqual(calls, [])
                app.robot.trash_var.set("old   (20260101-101010)")
                app.robot.restore_map()
            self.assertEqual(calls[0][2:4], ("/maps/restore", {"entry": "old__20260101-101010"}))
        finally:
            root.destroy()

    def test_rename_validates_and_sends(self):
        root, app, calls = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            with mock.patch("jetbot_gui.robot_panel.simpledialog") as sd, mock.patch("jetbot_gui.robot_panel.messagebox") as mb:
                sd.askstring.return_value = "bad name!"
                app.robot.rename_map()
                self.assertEqual(calls, [])
                self.assertTrue(mb.showwarning.called)
                sd.askstring.return_value = "garage2"
                app.robot.rename_map()
            self.assertEqual(calls[0][2:4], ("/maps/rename", {"name": "garage", "new_name": "garage2"}))
        finally:
            root.destroy()

    def test_the_library_marks_the_active_map(self):
        root, app, _ = prep(self)
        try:
            self.feed(app, snap_of("idle"))
            self.assertTrue(app.robot.tree.item("garage", "text").startswith("●"))
            self.assertFalse(app.robot.tree.item("kitchen", "text").startswith("●"))
        finally:
            root.destroy()

    def test_places_follow_the_map_in_use(self):
        root, app, _ = prep(self)
        try:
            from jetbot_gui.places import Places
            app.places = Places(os.devnull)
            app.places.use("garage")
            app.places.put("car", (1, 1, 0), None)
            app.places.use("default")
            app._sync_places(snap_of("navigation", active="garage"))
            self.assertEqual(app.places.scope, "garage")
            self.assertEqual(app.places.names(), ["car"])
            app._sync_places(snap_of("navigation", active="kitchen"))
            self.assertEqual(app.places.names(), [])
            app._sync_places(snap_of(connected=False))
            self.assertEqual(app.places.scope, "default")
        finally:
            root.destroy()


class PreviewTests(unittest.TestCase):
    def pgm(self, w, h, fill=205, box=None):
        im = Image.new("L", (w, h), fill)
        if box:
            im.paste(0, box)
        buf = io.BytesIO()
        im.save(buf, format="PPM")
        return buf.getvalue()

    def test_preview_is_cropped_to_the_known_area_and_fits(self):
        im = preview_image(self.pgm(320, 320, 205, (100, 100, 140, 120)))
        self.assertLessEqual(im.width, 296)
        self.assertLessEqual(im.height, 200)
        self.assertGreater(im.width / im.height, 1.5)                          # the 40 x 20 box, not the 320 x 320 sheet

    def test_garbage_gives_none(self):
        self.assertIsNone(preview_image(b"not an image"))

    def test_uptime_formatting(self):
        self.assertEqual(fmt_age(None), "?")
        self.assertEqual(fmt_age(300), "5m")
        self.assertEqual(fmt_age(7300), "2h 01m")
        self.assertEqual(fmt_age(200000), "2d 7h")


class FakeTunnel:
    def __init__(self, host, user, key, ros_port, mgr_port, *, fail=None, token=True):
        if fail:
            raise TunnelError(fail)
        self.host, self.local_ros, self.local_mgr, self._token, self.closed = host, 5555, 5556, token, False

    def open(self):
        pass

    def token(self):
        if not self._token:
            raise TunnelError("the JetBot manager is not installed on the robot")
        return "t" * 40

    def alive(self):
        return not self.closed

    def close(self):
        self.closed = True


class ConnectFlowTests(unittest.TestCase):
    def wait_tunnel(self, root, app):
        end = time.time() + 5
        while time.time() < end and app._tunnel_job is not None:
            root.update()
            app._poll_tunnel()
            time.sleep(0.05)

    def test_a_remote_host_opens_a_tunnel_then_connects_rosbridge_and_the_manager(self):
        root, app = make_app(self)
        try:
            links = []
            app.link.connect = lambda host, port: links.append(("ros", host, port))
            app.mgr.connect = lambda port, token: links.append(("mgr", port, token))
            with mock.patch("jetbot_gui.app.Tunnel", FakeTunnel):
                app.host_var.set("192.168.68.107")
                app.connect()
                self.wait_tunnel(root, app)
            self.assertEqual(links, [("ros", "127.0.0.1", 5555), ("mgr", 5556, "t" * 40)])
            self.assertEqual(app._conn_note, "")
        finally:
            root.destroy()

    def test_a_tunnel_failure_is_shown_and_nothing_connects(self):
        root, app = make_app(self)
        try:
            links = []
            app.link.connect = lambda *a: links.append(a)
            app.mgr.connect = lambda *a: links.append(a)
            with mock.patch("jetbot_gui.app.Tunnel", lambda *a: FakeTunnel(*a, fail="SSH login to x failed. Load your key with ssh-add")):
                app.host_var.set("192.168.68.107")
                app.connect()
                self.wait_tunnel(root, app)
            self.assertEqual(links, [])
            self.assertIn("ssh-add", app._conn_note)
        finally:
            root.destroy()

    def test_without_the_manager_rosbridge_still_connects_and_says_why(self):
        root, app = make_app(self)
        try:
            links = []
            app.link.connect = lambda host, port: links.append(("ros", port))
            app.mgr.connect = lambda *a: links.append(("mgr",) + a)
            with mock.patch("jetbot_gui.app.Tunnel", lambda *a: FakeTunnel(*a, token=False)):
                app.host_var.set("robot.local")
                app.connect()
                self.wait_tunnel(root, app)
            self.assertEqual(links, [("ros", 5555)])
            self.assertIn("not installed", app._conn_note)
        finally:
            root.destroy()

    def test_loopback_connects_directly_without_ssh(self):
        root, app = make_app(self)
        try:
            links = []
            app.link.connect = lambda host, port: links.append((host, port))
            with mock.patch("jetbot_gui.app.Tunnel", side_effect=AssertionError("must not open a tunnel for loopback")):
                app.host_var.set("127.0.0.1")
                app.connect()
            self.assertEqual(links, [("127.0.0.1", 9090)])
        finally:
            root.destroy()

    def test_a_dropped_tunnel_is_reported_and_disconnect_closes_it(self):
        root, app = make_app(self)
        try:
            app.link.connect = lambda *a: None
            app.mgr.connect = lambda *a: None
            with mock.patch("jetbot_gui.app.Tunnel", FakeTunnel):
                app.host_var.set("192.168.68.107")
                app.connect()
                self.wait_tunnel(root, app)
            t = app.tunnel
            t.closed = True                                                   # the process died
            app._poll_tunnel()
            self.assertIn("dropped", app._conn_note)
            app.tunnel = FakeTunnel("h", "u", "", 1, 2)
            t2 = app.tunnel
            app.disconnect()
            self.assertTrue(t2.closed)
            self.assertIsNone(app.tunnel)
        finally:
            root.destroy()

    def test_a_tunnel_that_finishes_after_a_disconnect_is_not_adopted(self):
        root, app = make_app(self)
        try:
            links = []
            app.link.connect = lambda *a: links.append(a)
            app.mgr.connect = lambda *a: links.append(a)
            with mock.patch("jetbot_gui.app.Tunnel", FakeTunnel):
                app.host_var.set("192.168.68.107")
                app.connect()
                app.disconnect()                                              # the user gave up while it was still opening
                time.sleep(0.5)
                app._poll_tunnel()
            self.assertEqual(links, [])
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
