"""Client for the JetBot manager (manager/jetbot_manager.py on the robot): start/stop the robot software and manage maps.

Everything runs in background threads; the window only reads snapshot(). The manager is reached through the SSH tunnel on localhost and every request
carries the secret token. This module never publishes to ROS and cannot move the robot.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

POLL_STATUS_S = 2.0
POLL_MAPS_S = 6.0


@dataclass
class ManagerSnapshot:
    connected: bool = False
    status: Optional[dict] = None
    maps: List[dict] = field(default_factory=list)
    trash: List[dict] = field(default_factory=list)
    active: Optional[str] = None
    error: str = ""
    busy: str = ""                      # what an action in progress is doing ("" = idle)
    last_action: str = ""
    last_ok: Optional[bool] = None
    status_age: Optional[float] = None


class ManagerError(Exception):
    pass


class ManagerLink:
    def __init__(self, status_period: float = POLL_STATUS_S, maps_period: float = POLL_MAPS_S):
        self._lock = threading.Lock()
        self._port: Optional[int] = None
        self._token = ""
        self._gen = 0
        self._status: Optional[dict] = None
        self._status_rx: Optional[float] = None
        self._maps: List[dict] = []
        self._trash: List[dict] = []
        self._active: Optional[str] = None
        self._error = ""
        self._busy = ""
        self._last = ("", None)
        self._maps_dirty = True
        self._images: Dict[tuple, bytes] = {}
        self._status_period, self._maps_period = status_period, maps_period

    # ------------------------------------------------------------ lifecycle
    def connect(self, port: int, token: str) -> None:
        with self._lock:
            self._gen += 1
            gen = self._gen
            self._port, self._token = int(port), token
            self._status, self._status_rx, self._maps, self._trash, self._active = None, None, [], [], None
            self._error, self._maps_dirty = "", True
        threading.Thread(target=self._poll, args=(gen,), daemon=True, name="manager-poll").start()

    def disconnect(self) -> None:
        with self._lock:
            self._gen += 1
            self._port, self._token = None, ""
            self._status, self._status_rx, self._maps, self._trash, self._active = None, None, [], [], None
            self._error = ""

    def refresh(self) -> None:
        with self._lock:
            self._maps_dirty = True

    @property
    def enabled(self) -> bool:
        return self._port is not None

    # ------------------------------------------------------------ http
    def _http(self, method: str, path: str, body: Optional[dict] = None, timeout: float = 10.0, raw: bool = False):
        with self._lock:
            port, token = self._port, self._token
        if port is None:
            raise ManagerError("not connected to the manager")
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), method=method,
                                     data=json.dumps(body).encode() if body is not None else None)
        req.add_header("X-Jetbot-Token", token)
        if body is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                data = r.read()
                return data if raw else json.loads(data.decode())
        except urllib.error.HTTPError as e:
            try:
                msg = json.loads(e.read().decode()).get("error", "")
            except Exception:
                msg = ""
            finally:
                e.close()
            raise ManagerError(msg or "HTTP %d" % e.code)
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise ManagerError("cannot reach the manager (%s)" % getattr(e, "reason", e))

    def _poll(self, gen: int) -> None:
        last_maps = 0.0
        while True:
            with self._lock:
                if gen != self._gen:
                    return
                dirty = self._maps_dirty
            try:
                st = self._http("GET", "/status", timeout=4.0)
                with self._lock:
                    if gen != self._gen:
                        return
                    self._status, self._status_rx, self._error = st, time.monotonic(), ""
                    self._active = st.get("active_map")
                if dirty or time.monotonic() - last_maps > self._maps_period:
                    mp = self._http("GET", "/maps", timeout=6.0)
                    with self._lock:
                        if gen != self._gen:
                            return
                        self._maps, self._trash, self._active = mp.get("maps", []), mp.get("trash", []), mp.get("active")
                        self._maps_dirty = False
                    last_maps = time.monotonic()
            except ManagerError as e:
                with self._lock:
                    if gen != self._gen:
                        return
                    self._error, self._status = str(e), None
            time.sleep(self._status_period)

    # ------------------------------------------------------------ read side (UI thread)
    def snapshot(self) -> ManagerSnapshot:
        with self._lock:
            now = time.monotonic()
            ok = self._status is not None and self._status_rx is not None and now - self._status_rx < 3 * self._status_period + 2
            return ManagerSnapshot(connected=bool(ok), status=dict(self._status) if self._status else None, maps=list(self._maps),
                                   trash=list(self._trash), active=self._active, error=self._error, busy=self._busy,
                                   last_action=self._last[0], last_ok=self._last[1],
                                   status_age=None if self._status_rx is None else now - self._status_rx)

    # ------------------------------------------------------------ actions (each runs in its own thread)
    def act(self, label: str, method: str, path: str, body: Optional[dict] = None,
            done: Optional[Callable[[bool, str], None]] = None) -> bool:
        with self._lock:
            if self._busy:
                return False
            self._busy = label
        threading.Thread(target=self._run_action, args=(label, method, path, body, done), daemon=True, name="manager-action").start()
        return True

    def _run_action(self, label, method, path, body, done) -> None:
        ok, text = False, ""
        try:
            res = self._http(method, path, body, timeout=120.0)
            ok, text = True, label + ": done" + (" (%s)" % ", ".join("%s=%s" % kv for kv in res.items() if kv[0] != "origin") if isinstance(res, dict) and res else "")
        except ManagerError as e:
            text = "%s failed: %s" % (label, e)
        with self._lock:
            self._busy = ""
            self._last = (text, ok)
            self._maps_dirty = True
        if done:
            try:
                done(ok, text)
            except Exception:
                pass

    def image(self, name: str, modified: int) -> Optional[bytes]:
        """Map preview bytes (cached). Blocking: call from a worker thread."""
        key = (name, modified)
        with self._lock:
            hit = self._images.get(key)
        if hit is not None:
            return hit
        try:
            data = self._http("GET", "/maps/%s/image" % name, timeout=20.0, raw=True)
        except ManagerError:
            return None
        with self._lock:
            self._images[key] = data
            if len(self._images) > 24:
                self._images.pop(next(iter(self._images)))
        return data
