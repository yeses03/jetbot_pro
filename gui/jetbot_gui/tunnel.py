"""SSH tunnel to the robot. Opens local ports that forward to rosbridge and to the JetBot manager, and fetches the manager's token.

Nothing here uses a shell: `ssh` is started with an argument list, and every field is validated first so that a host, user or key path
can never be read by ssh as an option. rosbridge has no authentication and the manager is meant for localhost only, so the tunnel
(your SSH key) is what authenticates you. It never prompts: with BatchMode, a key that is not loaded (`ssh-add`) fails with a clear message.
"""
from __future__ import annotations

import os
import re
import socket
import subprocess
import time
from typing import List, Optional

HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,252}$")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")
TOKEN_RE = re.compile(r"^[A-Za-z0-9]{32,128}$")


class TunnelError(Exception):
    pass


def is_loopback(host: str) -> bool:
    return host.strip().lower() in ("127.0.0.1", "localhost", "::1", "")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def validate(host: str, user: str, key: str = "") -> None:
    if not HOST_RE.match(host or ""):
        raise TunnelError("the robot address must be a hostname or IP (letters, digits, dots, dashes)")
    if not USER_RE.match(user or ""):
        raise TunnelError("the SSH user name looks wrong")
    if key:
        k = os.path.expanduser(key)
        if k.startswith("-") or not os.path.isfile(k):
            raise TunnelError("the SSH key file does not exist: %s" % key)


def friendly(stderr: str, host: str) -> str:
    low = stderr.lower()
    if "permission denied" in low:
        return "SSH login to %s failed. Load your key with ssh-add, or set the key file under More." % host
    if "could not resolve" in low or "name or service not known" in low or "nodename nor servname" in low:
        return "cannot find %s on this network (check the address or the Wi-Fi)." % host
    if "refused" in low:
        return "%s refused the connection (is it powered on and is SSH running?)." % host
    if "timed out" in low or "no route" in low or "unreachable" in low:
        return "%s did not answer: is it powered on and on the same network?" % host
    if "host key verification failed" in low or "remote host identification has changed" in low:
        return "the robot's SSH identity changed. If you did not replace the robot, do not connect; otherwise remove its line from ~/.ssh/known_hosts."
    return "ssh failed: " + (stderr.strip().splitlines()[-1] if stderr.strip() else "unknown error")


class Tunnel:
    def __init__(self, host: str, user: str = "nvidia", key: str = "", ros_port: int = 9090, mgr_port: int = 9091, ssh: str = "ssh",
                 ssh_port: int = 22):
        validate(host, user, key)
        self.host, self.user, self.key = host, user, os.path.expanduser(key) if key else ""
        self.ros_port, self.mgr_port, self.ssh, self.ssh_port = int(ros_port), int(mgr_port), ssh, int(ssh_port)
        self.local_ros: Optional[int] = None
        self.local_mgr: Optional[int] = None
        self.proc: Optional[subprocess.Popen] = None

    def _base(self) -> List[str]:
        args = [self.ssh, "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8",
                "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3", "-p", str(self.ssh_port)]
        if self.key:
            args += ["-i", self.key, "-o", "IdentitiesOnly=yes"]
        return args

    def command(self, local_ros: int, local_mgr: int) -> List[str]:
        return self._base() + ["-o", "ExitOnForwardFailure=yes", "-N",
                               "-L", "%d:127.0.0.1:%d" % (local_ros, self.ros_port),
                               "-L", "%d:127.0.0.1:%d" % (local_mgr, self.mgr_port), "%s@%s" % (self.user, self.host)]

    @staticmethod
    def _accepts(port: int) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.3):
                return True
        except OSError:
            return False

    def open(self, timeout: float = 15.0) -> None:
        self.local_ros, self.local_mgr = free_port(), free_port()
        self.proc = subprocess.Popen(self.command(self.local_ros, self.local_mgr), stdin=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rc = self.proc.poll()
            if rc is not None:
                err = self.proc.stderr.read().decode(errors="replace") if self.proc.stderr else ""
                if self.proc.stderr:
                    self.proc.stderr.close()
                self.proc = None
                raise TunnelError(friendly(err, self.host))
            if self._accepts(self.local_ros) and self._accepts(self.local_mgr):
                return
            time.sleep(0.1)
        self.close()
        raise TunnelError("timed out opening the SSH tunnel to %s" % self.host)

    def token(self, timeout: float = 12.0) -> str:
        """The manager's secret, read over SSH (so the SSH key is what authorises the console)."""
        try:
            r = subprocess.run(self._base() + ["%s@%s" % (self.user, self.host), "cat ~/.jetbot/token"], stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        except subprocess.TimeoutExpired:
            raise TunnelError("timed out reading the manager token")
        if r.returncode != 0:
            raise TunnelError("the JetBot manager is not installed on the robot (no ~/.jetbot/token): " + friendly(r.stderr.decode(errors="replace"), self.host))
        tok = r.stdout.decode().strip()
        if not TOKEN_RE.match(tok):
            raise TunnelError("the manager token on the robot looks wrong")
        return tok

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def close(self) -> None:
        p, self.proc = self.proc, None
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        if p is not None and p.stderr:
            try:
                p.stderr.close()
            except OSError:
                pass
