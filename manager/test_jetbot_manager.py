"""Tests for the JetBot manager. Standard library only. Run:  python3 -m unittest test_jetbot_manager -v   (from manager/)

Processes and ROS commands are faked (a `sleep` stands in for roslaunch, a small script for map_saver); the HTTP layer is real.
"""
import json
import os
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

import jetbot_manager as jm


def write_map(maps_dir: Path, name: str, w=8, h=4, image=None):
    (maps_dir / (name + ".pgm")).write_bytes(b"P5\n# CREATOR: test\n%d %d\n255\n" % (w, h) + bytes(w * h))
    (maps_dir / (name + ".yaml")).write_text("image: %s\nresolution: 0.050000\norigin: [-8.000000, -8.000000, 0.000000]\nnegate: 0\n"
                                             % (image or str(maps_dir / (name + ".pgm"))))


class Fake:
    """Stands in for roslaunch (a long sleep) and for the ROS/systemd shell commands."""

    def __init__(self):
        self.spawned = []
        self.procs = []
        self.run_calls = []
        self.firewall = "active"
        self.saver_ok = True

    def spawn(self, shell_cmd, log_path):
        self.spawned.append(shell_cmd)
        p = subprocess.Popen(["sleep", "600"], preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(p)
        Path(log_path).write_text("line1\nline2\nline3\n")
        return p

    def run(self, shell_cmd, timeout):
        self.run_calls.append(shell_cmd)

        class R:
            returncode, stdout = 0, b""
        r = R()
        if shell_cmd.startswith("systemctl is-active"):
            r.stdout = self.firewall.encode() + b"\n"
        elif "map_saver" in shell_cmd:
            target = Path(shell_cmd.split("-f ")[1].strip().strip("'"))
            if self.saver_ok:
                write_map(target.parent, target.name)
            else:
                r.returncode, r.stdout = 1, b"boom"
        return r

    def cleanup(self):
        for p in self.procs:
            if p.poll() is None:
                p.kill()
            p.wait()


class ManagerTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.fake = Fake()
        self.m = jm.Manager({"maps_dir": str(root / "maps"), "state_dir": str(root / "state"), "log_dir": str(root / "logs"),
                             "stop_timeout_s": 3.0}, spawn=self.fake.spawn, run=self.fake.run)
        self.maps = root / "maps"

    def tearDown(self):
        self.m.shutdown()
        self.fake.cleanup()
        self.tmp.cleanup()

    def err(self, fn, *a, **kw):
        with self.assertRaises(jm.ApiError) as cm:
            fn(*a, **kw)
        return cm.exception


class ValidationTests(ManagerTestCase):
    def test_names(self):
        for bad in ("", " ", "../x", "a/b", "a b", ".hidden", "x" * 49, None, 5, "x;rm -rf /", "a\nb", "é"):
            self.assertEqual(self.err(jm.valid_name, bad).status, 400, repr(bad))
        for good in ("a", "living_room-2", "A1", "x" * 48):
            self.assertEqual(jm.valid_name(good), good)

    def test_numbers(self):
        for bad in ("abc", None, float("nan"), float("inf"), 1e9, -1e9):
            self.assertEqual(self.err(jm.finite, bad, -50, 50, "x").status, 400, repr(bad))
        self.assertEqual(jm.finite("1.5", -50, 50, "x"), 1.5)

    def test_pgm_header_with_comments(self):
        write_map(self.maps, "a", 12, 7)
        self.assertEqual(jm.read_pgm_size(self.maps / "a.pgm"), (12, 7))
        (self.maps / "bad.pgm").write_bytes(b"GIF89a")
        self.assertIsNone(jm.read_pgm_size(self.maps / "bad.pgm"))
        self.assertIsNone(jm.read_pgm_size(self.maps / "nope.pgm"))


class MapTests(ManagerTestCase):
    def test_list_reports_size_origin_active_and_skips_junk(self):
        write_map(self.maps, "room", 320, 240)
        (self.maps / "junk.yaml").write_text("not a map")
        (self.maps / "bad name.yaml").write_text("image: x.pgm\n")
        self.m.activate_map("room")
        got = self.m.list_maps()
        self.assertEqual([g["name"] for g in got], ["room"])
        self.assertEqual((got[0]["width"], got[0]["height"], got[0]["resolution"], got[0]["origin"][:2]), (320, 240, 0.05, [-8.0, -8.0]))
        self.assertTrue(got[0]["active"])

    def test_legacy_absolute_image_path_still_loads(self):
        write_map(self.maps, "old", image="/home/nvidia/maps/old.pgm")           # path does not exist here; falls back to the maps dir
        self.assertEqual(self.m.list_maps()[0]["name"], "old")

    def test_delete_is_a_reversible_move_to_the_trash(self):
        write_map(self.maps, "room")
        self.m.activate_map("room")
        out = self.m.delete_map("room")
        self.assertFalse((self.maps / "room.yaml").exists())
        self.assertTrue((self.maps / ".trash" / out["trashed"] / "room.pgm").exists())
        self.assertIsNone(self.m.active_map)
        self.assertEqual(self.m.list_maps(), [])
        entry = self.m.list_trash()[0]["entry"]
        self.assertEqual(self.m.restore_map(entry), {"restored": "room"})
        self.assertEqual([g["name"] for g in self.m.list_maps()], ["room"])

    def test_restore_never_overwrites(self):
        write_map(self.maps, "room")
        entry = self.m.delete_map("room")["trashed"]
        write_map(self.maps, "room")                                              # a new map took the name meanwhile
        self.assertEqual(self.err(self.m.restore_map, entry).status, 409)
        self.assertEqual(self.err(self.m.restore_map, "../../etc__20200101-000000").status, 400)
        self.assertEqual(self.err(self.m.restore_map, "nothing__20200101-000000").status, 404)

    def test_rename(self):
        write_map(self.maps, "a")
        self.m.activate_map("a")
        self.m.rename_map("a", "b")
        self.assertEqual([g["name"] for g in self.m.list_maps()], ["b"])
        self.assertEqual(self.m.active_map, "b")
        self.assertIn("b.pgm", (self.maps / "b.yaml").read_text())
        write_map(self.maps, "c")
        self.assertEqual(self.err(self.m.rename_map, "b", "c").status, 409)
        self.assertEqual(self.err(self.m.rename_map, "b", "../c").status, 400)

    def test_traversal_in_names_is_rejected_everywhere(self):
        for fn in (self.m.activate_map, self.m.delete_map, self.m.map_pgm_bytes):
            self.assertEqual(self.err(fn, "../secret").status, 400)
        self.assertEqual(self.err(self.m.activate_map, "missing").status, 404)

    def test_map_image_bytes(self):
        write_map(self.maps, "a", 4, 4)
        self.assertTrue(self.m.map_pgm_bytes("a").startswith(b"P5"))


class SaveTests(ManagerTestCase):
    def test_saving_needs_mapping_mode(self):
        self.assertEqual(self.err(self.m.save_map, "x").status, 409)
        self.m.start("navigation", {"map": self._make("a")})
        self.assertEqual(self.err(self.m.save_map, "x").status, 409)

    def _make(self, name):
        write_map(self.maps, name)
        return name

    def test_save_in_mapping_mode_creates_a_relative_path_map(self):
        self.m.start("mapping")
        info = self.m.save_map("newroom")
        self.assertEqual(info["name"], "newroom")
        self.assertIn("image: newroom.pgm", (self.maps / "newroom.yaml").read_text())
        self.assertTrue(any("map_saver" in c for c in self.fake.run_calls))

    def test_never_overwrites_an_existing_map(self):
        self._make("keep")
        before = (self.maps / "keep.pgm").read_bytes()
        self.m.start("mapping")
        self.assertEqual(self.err(self.m.save_map, "keep").status, 409)
        self.assertEqual((self.maps / "keep.pgm").read_bytes(), before)
        (self.maps / "half.pgm").write_bytes(b"x")                                 # even a stray file with the name blocks it
        self.assertEqual(self.err(self.m.save_map, "half").status, 409)

    def test_failed_saver_leaves_nothing_behind(self):
        self.fake.saver_ok = False
        self.m.start("mapping")
        self.assertEqual(self.err(self.m.save_map, "oops").status, 500)
        self.assertFalse((self.maps / "oops.yaml").exists())


class StackTests(ManagerTestCase):
    def test_navigation_command_uses_the_calibrated_values_and_the_chosen_map(self):
        write_map(self.maps, "room")
        self.m.start("navigation", {"map": "room", "initial_pose": {"x": 1.5, "y": -0.25, "a": 0.5}})
        cmd = self.fake.spawned[-1]
        self.assertTrue(cmd.startswith("roslaunch jetbot_pro navigate.launch"))
        for want in ("linear_correction:=1.15", "angular_correction:=0.9", "cmd_vel_topic:=cmd_vel", "initial_x:=1.5", "initial_y:=-0.25",
                     "initial_a:=0.5", "map_file:=%s" % (self.maps / "room.yaml")):
            self.assertIn(want, cmd)
        self.assertEqual(self.m.active_map, "room")
        self.assertEqual(self.m.status()["mode"], "navigation")

    def test_mapping_command(self):
        self.m.start("mapping", {"extent": 10})
        self.assertIn("mapping.launch", self.fake.spawned[-1])
        self.assertIn("extent:=10.0", self.fake.spawned[-1])

    def test_bad_requests_are_refused_before_anything_starts(self):
        write_map(self.maps, "room")
        for mode, params in (("navigation", {}), ("navigation", {"map": "../x"}), ("navigation", {"map": "missing"}),
                             ("navigation", {"map": "room", "initial_pose": {"x": "1;rm -rf /"}}),
                             ("navigation", {"map": "room", "initial_pose": {"x": 1e9}}), ("mapping", {"extent": 1000}),
                             ("explode", {}), (None, {})):
            self.assertIn(self.err(self.m.start, mode, params).status, (400, 404), (mode, params))
        self.assertEqual(self.fake.spawned, [])

    def test_only_one_stack_at_a_time_unless_replace(self):
        write_map(self.maps, "room")
        self.m.start("mapping")
        self.assertEqual(self.err(self.m.start, "navigation", {"map": "room"}).status, 409)
        self.m.start("navigation", {"map": "room"}, replace=True)
        self.assertEqual(len(self.fake.spawned), 2)
        self.assertIsNotNone(self.fake.procs[0].poll())                           # the first one was stopped
        self.assertEqual(self.m.status()["mode"], "navigation")

    def test_stop_ends_the_process_and_goes_idle(self):
        self.m.start("mapping")
        self.assertEqual(self.m.stop(), {"stopped": "mapping"})
        self.assertIsNotNone(self.fake.procs[0].poll())
        self.assertEqual(self.m.status()["mode"], "idle")
        self.assertEqual(self.m.stop(), {"stopped": None})

    def test_a_map_in_use_cannot_be_deleted_or_renamed(self):
        write_map(self.maps, "room")
        self.m.start("navigation", {"map": "room"})
        self.assertEqual(self.err(self.m.delete_map, "room").status, 409)
        self.assertEqual(self.err(self.m.rename_map, "room", "other").status, 409)
        self.m.stop()
        self.m.delete_map("room")

    def test_a_crashed_stack_is_reported(self):
        self.m.start("mapping")
        self.fake.procs[0].kill()
        self.fake.procs[0].wait()
        st = self.m.status()
        self.assertEqual(st["mode"], "idle")
        self.assertIn("exited", st["last_exit"])

    def test_corrections_are_validated_and_used(self):
        self.assertEqual(self.err(self.m.set_config, {"linear_correction": 9}).status, 400)
        self.assertEqual(self.err(self.m.set_config, {"linear_correction": "x"}).status, 400)
        self.m.set_config({"linear_correction": 1.2, "angular_correction": 0.95})
        self.m.start("mapping")
        self.assertIn("linear_correction:=1.2", self.fake.spawned[-1])
        self.assertIn("angular_correction:=0.95", self.fake.spawned[-1])


class BaseTests(ManagerTestCase):
    def test_rosbridge_refuses_to_start_without_the_firewall(self):
        self.fake.firewall = "inactive"
        e = self.err(self.m.start_base)
        self.assertEqual(e.status, 409)
        self.assertIn("firewall", e.message)
        self.assertEqual(self.fake.spawned, [])
        self.assertFalse(self.m.status()["firewall"]["ok"])

    def test_rosbridge_starts_when_the_firewall_unit_is_active_and_not_twice(self):
        self.assertTrue(self.m.start_base()["started"])
        self.assertIn("rosbridge.launch", self.fake.spawned[0])
        self.assertFalse(self.m.start_base()["started"])
        self.assertEqual(len(self.fake.spawned), 1)

    def test_firewall_requirement_can_be_switched_off_explicitly(self):
        self.m.cfg["require_firewall_unit"] = False
        self.fake.firewall = "inactive"
        self.assertTrue(self.m.start_base()["started"])

    def test_stopping_the_base_stops_the_stack_too(self):
        self.m.start_base()
        self.m.start("mapping")
        self.m.stop_base()
        self.assertTrue(all(p.poll() is not None for p in self.fake.procs))

    def test_log_tail(self):
        self.m.start("mapping")
        self.assertEqual(self.m.tail_log("stack", 2), "line2\nline3")
        self.assertEqual(self.err(self.m.tail_log, "base").status, 404)


class HttpTests(ManagerTestCase):
    def setUp(self):
        super().setUp()
        self.srv = jm.serve(self.m, "127.0.0.1", 0, token="s3cret-token")
        self.port = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()
        super().tearDown()

    def call(self, method, path, body=None, token="s3cret-token", raw=False):
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), method=method,
                                     data=json.dumps(body).encode() if body is not None else None)
        if token is not None:
            req.add_header("X-Jetbot-Token", token)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                data = r.read()
                return r.status, (data if raw else json.loads(data.decode()))
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read().decode())

    def test_ping_is_open_everything_else_needs_the_token(self):
        self.assertEqual(self.call("GET", "/ping", token=None)[0], 200)
        for path in ("/status", "/maps", "/config"):
            self.assertEqual(self.call("GET", path, token=None)[0], 401, path)
            self.assertEqual(self.call("GET", path, token="wrong")[0], 401, path)
            self.assertEqual(self.call("GET", path, token="s3cret-token-")[0], 401, path)
        for path in ("/start", "/stop", "/maps/delete", "/maps/save", "/base/start"):
            self.assertEqual(self.call("POST", path, {}, token="wrong")[0], 401, path)
        self.assertEqual(self.fake.spawned, [])                                   # nothing ran without the token

    def test_it_refuses_to_listen_anywhere_but_localhost(self):
        for host in ("0.0.0.0", "", "192.168.1.5"):
            with self.assertRaises(SystemExit):
                jm.serve(self.m, host, 0, token="x")

    def test_full_flow_over_http(self):
        write_map(self.maps, "room", 16, 8)
        self.assertEqual(self.call("GET", "/maps")[1]["maps"][0]["name"], "room")
        self.assertEqual(self.call("POST", "/start", {"mode": "navigation", "map": "room"})[0], 200)
        self.assertEqual(self.call("GET", "/status")[1]["mode"], "navigation")
        self.assertEqual(self.call("POST", "/maps/delete", {"name": "room"})[0], 409)
        self.assertEqual(self.call("POST", "/stop", {})[0], 200)
        self.assertEqual(self.call("POST", "/start", {"mode": "mapping"})[0], 200)
        self.assertEqual(self.call("POST", "/maps/save", {"name": "fresh"})[0], 200)
        self.assertEqual(self.call("POST", "/maps/save", {"name": "fresh"})[0], 409)
        code, img = self.call("GET", "/maps/fresh/image", raw=True)
        self.assertEqual(code, 200)
        self.assertTrue(img.startswith(b"P5"))
        self.assertEqual(self.call("POST", "/maps/delete", {"name": "room"})[0], 200)
        self.assertEqual(len(self.call("GET", "/maps")[1]["trash"]), 1)

    def test_errors_are_clean_json_without_tracebacks(self):
        self.assertEqual(self.call("GET", "/nope")[0], 404)
        code, body = self.call("POST", "/start", {"mode": "navigation", "map": "../../etc/passwd"})
        self.assertEqual(code, 400)
        self.assertNotIn("Traceback", json.dumps(body))
        self.assertEqual(self.call("GET", "/maps/..%2F..%2Fetc%2Fpasswd/image")[0], 400)

    def test_oversized_and_malformed_bodies(self):
        req = urllib.request.Request("http://127.0.0.1:%d/start" % self.port, method="POST", data=b"x" * 20000)
        req.add_header("X-Jetbot-Token", "s3cret-token")
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(cm.exception.code, 400)
        req = urllib.request.Request("http://127.0.0.1:%d/start" % self.port, method="POST", data=b"{not json")
        req.add_header("X-Jetbot-Token", "s3cret-token")
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(cm.exception.code, 400)

    def test_token_file_is_created_private_and_stable(self):
        tok = jm.load_token(self.m.state_dir)
        self.assertEqual(oct(os.stat(str(self.m.state_dir / "token")).st_mode & 0o777), "0o600")
        self.assertEqual(jm.load_token(self.m.state_dir), tok)
        self.assertGreaterEqual(len(tok), 32)


if __name__ == "__main__":
    unittest.main()
