"""Tests for the SSH tunnel and the manager client. The tunnel runs against a fake `ssh` program; the client against the REAL manager
(manager/jetbot_manager.py) running in this process with faked ROS commands.

Run:  python -m unittest discover -s tests -v   (from gui/)
"""
import os
import stat
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "manager"))

import jetbot_manager as jm
from jetbot_gui.manager_client import ManagerLink
from jetbot_gui.tunnel import Tunnel, TunnelError, friendly, is_loopback, validate
from test_jetbot_manager import Fake, write_map


def fake_ssh(dirname: str, kind: str) -> str:
    """A stand-in for ssh. kind: 'ok' (forwards by listening on the -L local ports, token on `cat`), 'denied', 'timeout', 'notoken'."""
    path = os.path.join(dirname, "ssh_" + kind)
    Path(path).write_text('''#!/usr/bin/env python3
import socket, sys, time
kind = "%s"
args = sys.argv[1:]
if "cat ~/.jetbot/token" in args:
    if kind == "notoken":
        sys.stderr.write("cat: /home/nvidia/.jetbot/token: No such file or directory\\n"); sys.exit(1)
    print("a" * 48); sys.exit(0)
if kind == "denied":
    sys.stderr.write("nvidia@host: Permission denied (publickey).\\n"); sys.exit(255)
if kind == "timeout":
    time.sleep(60); sys.exit(0)
socks = []
for i, a in enumerate(args):
    if a == "-L":
        port = int(args[i + 1].split(":")[0])
        s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(("127.0.0.1", port)); s.listen(5); socks.append(s)
time.sleep(60)
''' % kind)
    os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
    return path


class ValidationTests(unittest.TestCase):
    def test_hosts_and_users_that_ssh_could_read_as_options_are_refused(self):
        for host in ("-oProxyCommand=evil", "", "a b", "a;b", "host\nname", "$(id)", "-x", "a" * 300):
            with self.assertRaises(TunnelError, msg=repr(host)):
                validate(host, "nvidia")
        for user in ("", "-x", "Root!", "a b", "x" * 40, "ro;ot"):
            with self.assertRaises(TunnelError, msg=repr(user)):
                validate("192.168.1.5", user)
        with self.assertRaises(TunnelError):
            validate("192.168.1.5", "nvidia", "/no/such/key")
        validate("192.168.68.107", "nvidia")
        validate("robot.local", "nvidia")

    def test_a_key_path_that_looks_like_an_option_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            os.chdir(d)
            Path("-oBad").write_text("x")
            with self.assertRaises(TunnelError):
                validate("h", "nvidia", "-oBad")

    def test_loopback_detection(self):
        for h in ("127.0.0.1", "localhost", "::1", ""):
            self.assertTrue(is_loopback(h))
        self.assertFalse(is_loopback("192.168.68.107"))

    def test_friendly_errors(self):
        self.assertIn("ssh-add", friendly("Permission denied (publickey).", "h"))
        self.assertIn("cannot find", friendly("Could not resolve hostname x: Name or service not known", "x"))
        self.assertIn("powered on", friendly("ssh: connect to host h port 22: Connection timed out", "h"))
        self.assertIn("identity changed", friendly("WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!", "h"))


class CommandTests(unittest.TestCase):
    def test_the_command_is_an_argument_list_with_no_shell_and_only_the_two_forwards(self):
        t = Tunnel("192.168.1.5", "nvidia")
        cmd = t.command(5000, 5001)
        self.assertIsInstance(cmd, list)
        self.assertEqual(cmd[0], "ssh")
        self.assertIn("BatchMode=yes", cmd)
        self.assertIn("ExitOnForwardFailure=yes", cmd)
        self.assertEqual([cmd[i + 1] for i, a in enumerate(cmd) if a == "-L"], ["5000:127.0.0.1:9090", "5001:127.0.0.1:9091"])
        self.assertEqual(cmd[-1], "nvidia@192.168.1.5")
        self.assertNotIn("-R", cmd)                                         # never a reverse tunnel
        self.assertNotIn("-D", cmd)


class TunnelTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_open_succeeds_and_closes(self):
        t = Tunnel("192.168.1.5", "nvidia", ssh=fake_ssh(self.tmp.name, "ok"))
        t.open(timeout=10)
        try:
            self.assertTrue(t.alive())
            self.assertTrue(Tunnel._accepts(t.local_ros))
            self.assertTrue(Tunnel._accepts(t.local_mgr))
            self.assertNotEqual(t.local_ros, t.local_mgr)
        finally:
            t.close()
        self.assertFalse(t.alive())

    def test_login_failure_gives_a_helpful_message(self):
        t = Tunnel("192.168.1.5", "nvidia", ssh=fake_ssh(self.tmp.name, "denied"))
        with self.assertRaises(TunnelError) as cm:
            t.open(timeout=10)
        self.assertIn("ssh-add", str(cm.exception))
        self.assertFalse(t.alive())

    def test_it_times_out_instead_of_hanging(self):
        t = Tunnel("192.168.1.5", "nvidia", ssh=fake_ssh(self.tmp.name, "timeout"))
        start = time.monotonic()
        with self.assertRaises(TunnelError) as cm:
            t.open(timeout=1.5)
        self.assertIn("timed out", str(cm.exception))
        self.assertLess(time.monotonic() - start, 6)
        self.assertFalse(t.alive())

    def test_token_fetch(self):
        self.assertEqual(Tunnel("h", "nvidia", ssh=fake_ssh(self.tmp.name, "ok")).token(), "a" * 48)
        with self.assertRaises(TunnelError) as cm:
            Tunnel("h", "nvidia", ssh=fake_ssh(self.tmp.name, "notoken")).token()
        self.assertIn("not installed", str(cm.exception))


class ManagerClientTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.fake = Fake()
        self.mgr = jm.Manager({"maps_dir": str(root / "maps"), "state_dir": str(root / "state"), "log_dir": str(root / "logs"),
                               "stop_timeout_s": 3.0}, spawn=self.fake.spawn, run=self.fake.run)
        self.maps = root / "maps"
        self.srv = jm.serve(self.mgr, "127.0.0.1", 0, token="t" * 40)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.link = ManagerLink(status_period=0.2, maps_period=0.4)
        self.link.connect(self.srv.server_address[1], "t" * 40)

    def tearDown(self):
        self.link.disconnect()
        self.srv.shutdown()
        self.srv.server_close()
        self.mgr.shutdown()
        self.fake.cleanup()
        self.tmp.cleanup()

    def wait(self, cond, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            if cond(self.link.snapshot()):
                return True
            time.sleep(0.1)
        return False

    def test_it_connects_and_lists_maps_and_status(self):
        write_map(self.maps, "room", 16, 8)
        self.assertTrue(self.wait(lambda s: s.connected and [m["name"] for m in s.maps] == ["room"]))
        s = self.link.snapshot()
        self.assertEqual(s.status["mode"], "idle")
        self.assertEqual((s.maps[0]["width"], s.maps[0]["height"]), (16, 8))

    def test_a_wrong_token_never_connects_and_says_why(self):
        bad = ManagerLink(status_period=0.2)
        bad.connect(self.srv.server_address[1], "wrong")
        try:
            time.sleep(1.0)
            s = bad.snapshot()
            self.assertFalse(s.connected)
            self.assertIn("token", s.error)
        finally:
            bad.disconnect()

    def test_start_save_and_delete_flow_through_the_client(self):
        self.assertTrue(self.wait(lambda s: s.connected))
        done = []
        self.assertTrue(self.link.act("start mapping", "POST", "/start", {"mode": "mapping"}, done=lambda ok, t: done.append(ok)))
        self.assertTrue(self.wait(lambda s: s.status and s.status["mode"] == "mapping" and not s.busy))
        self.assertEqual(done, [True])
        self.link.act("save", "POST", "/maps/save", {"name": "fresh"})
        self.assertTrue(self.wait(lambda s: [m["name"] for m in s.maps] == ["fresh"] and not s.busy))
        self.link.act("save again", "POST", "/maps/save", {"name": "fresh"})
        self.assertTrue(self.wait(lambda s: s.last_ok is False and "already exists" in s.last_action))
        self.link.act("stop", "POST", "/stop", {})
        self.assertTrue(self.wait(lambda s: s.status and s.status["mode"] == "idle" and not s.busy))
        self.link.act("delete", "POST", "/maps/delete", {"name": "fresh"})
        self.assertTrue(self.wait(lambda s: s.maps == [] and len(s.trash) == 1))

    def test_only_one_action_at_a_time(self):
        self.assertTrue(self.wait(lambda s: s.connected))
        gate = threading.Event()
        orig = self.link._http

        def slow(*a, **kw):
            if a[1] == "/stop":
                gate.wait(3)
            return orig(*a, **kw)
        self.link._http = slow
        self.assertTrue(self.link.act("one", "POST", "/stop", {}))
        self.assertFalse(self.link.act("two", "POST", "/stop", {}))            # busy
        gate.set()
        self.assertTrue(self.wait(lambda s: not s.busy))

    def test_images_are_fetched_once_and_cached(self):
        write_map(self.maps, "room", 16, 8)
        self.assertTrue(self.wait(lambda s: s.maps))
        m = self.link.snapshot().maps[0]
        a = self.link.image("room", m["modified"])
        self.assertTrue(a.startswith(b"P5"))
        calls = []
        orig = self.link._http
        self.link._http = lambda *x, **k: calls.append(x) or orig(*x, **k)
        self.assertEqual(self.link.image("room", m["modified"]), a)
        self.assertEqual(calls, [])

    def test_losing_the_manager_is_reported_not_hidden(self):
        self.assertTrue(self.wait(lambda s: s.connected))
        self.srv.shutdown()
        self.srv.server_close()
        self.assertTrue(self.wait(lambda s: not s.connected and "cannot reach" in s.error, timeout=10))


if __name__ == "__main__":
    unittest.main()
