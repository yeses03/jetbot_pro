#!/usr/bin/env python3
"""JetBot manager: a small service on the Jetson that starts and stops the robot software and manages the maps folder.

ROS cannot start itself, so something outside ROS has to. This service:
  * keeps rosbridge (and with it the ROS master) up, but only when the firewall is in place (see manager/README.md);
  * starts ONE "stack" at a time: navigation (chassis + lidar + AMCL + move_base on a chosen map) or mapping (chassis + lidar + GMapping);
  * lists, saves, activates, renames, deletes (to a trash, reversible) and serves maps.

Security: it listens on 127.0.0.1 only (reach it through the SSH tunnel), needs the secret token in ~/.jetbot/token, and offers a FIXED set of
actions. Names are validated, no request value is ever put into a shell, and it can only touch the maps folder. It never publishes to ROS and has no
way to move the robot: motion needs rosbridge (the GUI's MOTION switch) as before.

Standard library only; works on Python 3.6 (the Jetson's) and newer. No third-party packages.
"""

import argparse
import json
import math
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, unquote, urlparse

VERSION = 1
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")
HOME = Path(os.path.expanduser("~"))

DEFAULTS = {
    "port": 9091,
    "maps_dir": str(HOME / "maps"),
    "state_dir": str(HOME / ".jetbot"),
    "log_dir": str(HOME / "logs"),
    "ros_setup": "/opt/ros/melodic/setup.bash",
    "ws_setup": str(HOME / "catkin_ws" / "devel" / "setup.bash"),
    "package": "jetbot_pro",
    "linear_correction": 1.15,       # calibrated 2026-11-02 (docs/bringup_findings_2026-11-02.md)
    "angular_correction": 0.9,
    "map_extent_m": 8.0,
    "require_firewall_unit": True,   # rosbridge has no authentication and listens on every interface: refuse to start it unprotected
    "firewall_unit": "jetbot-firewall.service",
    "autostart_base": True,
    "stop_timeout_s": 25.0,
}


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status, self.message = status, message


def valid_name(name) -> str:
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ApiError(400, "names are 1-48 characters: letters, digits, _ and -, starting with a letter or digit")
    return name


def finite(value, lo, hi, what) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ApiError(400, f"{what} must be a number")
    if not math.isfinite(v) or v < lo or v > hi:
        raise ApiError(400, f"{what} must be between {lo} and {hi}")
    return v


def read_pgm_size(path: Path):
    """(width, height) from a binary PGM header, or None."""
    try:
        with open(str(path), "rb") as fh:
            head = fh.read(256)
    except OSError:
        return None
    tokens, i = [], 0
    while len(tokens) < 4 and i < len(head):
        if head[i:i + 1] == b"#":
            while i < len(head) and head[i:i + 1] != b"\n":
                i += 1
            continue
        if head[i:i + 1].isspace():
            i += 1
            continue
        j = i
        while j < len(head) and not head[j:j + 1].isspace():
            j += 1
        tokens.append(head[i:j])
        i = j
    if len(tokens) < 3 or tokens[0] != b"P5":
        return None
    try:
        return int(tokens[1]), int(tokens[2])
    except ValueError:
        return None


def parse_map_yaml(path: Path) -> dict:
    """resolution / origin / image from a map_server yaml, without needing PyYAML."""
    text = path.read_text()
    out = {}
    m = re.search(r"^resolution:\s*([-\d.eE+]+)", text, re.M)
    if m:
        out["resolution"] = float(m.group(1))
    m = re.search(r"^origin:\s*\[([^\]]+)\]", text, re.M)
    if m:
        out["origin"] = [float(v) for v in m.group(1).split(",")[:3]]
    m = re.search(r"^image:\s*(\S+)", text, re.M)
    if m:
        out["image"] = m.group(1)
    return out


def rewrite_image_line(yaml_path: Path, image_name: str) -> None:
    text = yaml_path.read_text()
    if re.search(r"^image:", text, re.M):
        text = re.sub(r"^image:.*$", "image: " + image_name, text, count=1, flags=re.M)
    else:
        text = "image: " + image_name + "\n" + text
    yaml_path.write_text(text)


class Manager:
    """All behaviour, independent of HTTP so it can be tested directly. `spawn` and `run` are injectable for tests."""

    def __init__(self, cfg=None, spawn=None, run=None, clock=time.time):
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg or {})
        self.maps_dir = Path(self.cfg["maps_dir"]).expanduser()
        self.state_dir = Path(self.cfg["state_dir"]).expanduser()
        self.log_dir = Path(self.cfg["log_dir"]).expanduser()
        for d in (self.maps_dir, self.state_dir, self.log_dir, self.maps_dir / ".trash"):
            d.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self._spawn = spawn or self._default_spawn
        self._run = run or self._default_run
        self.lock = threading.RLock()
        self.stack = None          # dict(name, proc, started, log, args)
        self.base = None
        self.last_exit = ""
        self._load_state()

    # ---------------------------------------------------------------- processes
    def _env_prefix(self) -> str:
        return "source %s && source %s && " % (shlex.quote(self.cfg["ros_setup"]), shlex.quote(self.cfg["ws_setup"]))

    def _default_spawn(self, shell_cmd: str, log_path: Path):
        logfh = open(str(log_path), "ab")
        return subprocess.Popen(["bash", "-c", self._env_prefix() + "exec " + shell_cmd], stdout=logfh, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, preexec_fn=os.setsid)

    def _default_run(self, shell_cmd: str, timeout: float):
        return subprocess.run(["bash", "-c", self._env_prefix() + shell_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              stdin=subprocess.DEVNULL, timeout=timeout)

    def _launch_cmd(self, launch_file: str, args: dict) -> str:
        parts = ["roslaunch", shlex.quote(self.cfg["package"]), shlex.quote(launch_file)]
        parts += ["%s:=%s" % (k, shlex.quote(str(v))) for k, v in args.items()]
        return " ".join(parts)

    @staticmethod
    def _alive(entry) -> bool:
        return bool(entry) and entry["proc"].poll() is None

    def _stop_entry(self, entry) -> None:
        if not entry:
            return
        proc = entry["proc"]
        if proc.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, self.cfg["stop_timeout_s"]), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except (ProcessLookupError, PermissionError, OSError):
                break
            end = self.clock() + wait
            while self.clock() < end:
                if proc.poll() is not None:
                    return
                time.sleep(0.1)

    # ---------------------------------------------------------------- firewall / base
    def firewall(self) -> dict:
        if not self.cfg["require_firewall_unit"]:
            return {"required": False, "ok": True, "detail": "not required by configuration"}
        try:
            r = self._run("systemctl is-active %s" % shlex.quote(self.cfg["firewall_unit"]), 5)
            out = r.stdout.decode(errors="replace").strip()
        except Exception as exc:
            return {"required": True, "ok": False, "detail": "cannot query systemd: %s" % exc}
        return {"required": True, "ok": out == "active", "detail": "%s is %s" % (self.cfg["firewall_unit"], out or "unknown")}

    def start_base(self) -> dict:
        """rosbridge (+ the ROS master). Refused unless the firewall protects its port."""
        with self.lock:
            if self._alive(self.base):
                return {"started": False, "reason": "already running"}
            fw = self.firewall()
            if not fw["ok"]:
                raise ApiError(409, "not starting rosbridge: the firewall is not in place (%s). See manager/README.md." % fw["detail"])
            log = self.log_dir / ("jetbot_base_%d.log" % int(self.clock()))
            proc = self._spawn(self._launch_cmd("rosbridge.launch", {}), log)
            self.base = {"name": "rosbridge", "proc": proc, "started": self.clock(), "log": str(log)}
            return {"started": True}

    def stop_base(self) -> dict:
        with self.lock:
            self._stop_entry(self.stack)       # a stack cannot outlive the master that the base owns
            self._stop_entry(self.base)
            self.base = None
            return {"stopped": True}

    # ---------------------------------------------------------------- stacks
    def start(self, mode: str, params=None, replace: bool = False) -> dict:
        params = params or {}
        with self.lock:
            if self._alive(self.stack):
                if not replace:
                    raise ApiError(409, "a stack is already running (%s); stop it first or pass replace" % self.stack["name"])
                self._stop_entry(self.stack)
            corr = {"linear_correction": self.cfg["linear_correction"], "angular_correction": self.cfg["angular_correction"]}
            if mode == "navigation":
                name = valid_name(params.get("map"))
                info = self._map_info(name)
                if info is None:
                    raise ApiError(404, "no such map: %s" % name)
                pose = params.get("initial_pose") or {}
                args = dict(corr)
                args.update({"cmd_vel_topic": "cmd_vel", "map_file": str(self.maps_dir / (name + ".yaml")),
                             "initial_x": finite(pose.get("x", 0.0), -50, 50, "initial x"),
                             "initial_y": finite(pose.get("y", 0.0), -50, 50, "initial y"),
                             "initial_a": finite(pose.get("a", 0.0), -7, 7, "initial heading")})
                launch, label = "navigate.launch", "navigation"
                self.set_active(name)
            elif mode == "mapping":
                args = dict(corr)
                args["extent"] = finite(params.get("extent", self.cfg["map_extent_m"]), 2, 30, "map extent")
                launch, label = "mapping.launch", "mapping"
            else:
                raise ApiError(400, "mode must be 'navigation' or 'mapping'")
            log = self.log_dir / ("jetbot_%s_%d.log" % (label, int(self.clock())))
            proc = self._spawn(self._launch_cmd(launch, args), log)
            self.stack = {"name": label, "proc": proc, "started": self.clock(), "log": str(log), "args": {k: str(v) for k, v in args.items()}}
            self.last_exit = ""
            return {"started": label}

    def stop(self) -> dict:
        with self.lock:
            was = self.stack["name"] if self._alive(self.stack) else None
            self._stop_entry(self.stack)
            self.stack = None
            return {"stopped": was}

    # ---------------------------------------------------------------- state
    def _load_state(self) -> None:
        self.active_map = None
        try:
            data = json.loads((self.state_dir / "state.json").read_text())
            self.active_map = data.get("active_map")
        except Exception:
            pass

    def set_active(self, name):
        self.active_map = name
        tmp = self.state_dir / "state.json.tmp"
        tmp.write_text(json.dumps({"active_map": name}))
        os.replace(str(tmp), str(self.state_dir / "state.json"))

    def set_config(self, body: dict) -> dict:
        with self.lock:
            if "linear_correction" in body:
                self.cfg["linear_correction"] = finite(body["linear_correction"], 0.5, 2.0, "linear_correction")
            if "angular_correction" in body:
                self.cfg["angular_correction"] = finite(body["angular_correction"], 0.5, 2.0, "angular_correction")
            if "map_extent_m" in body:
                self.cfg["map_extent_m"] = finite(body["map_extent_m"], 2, 30, "map_extent_m")
            return self.get_config()

    def get_config(self) -> dict:
        return {k: self.cfg[k] for k in ("linear_correction", "angular_correction", "map_extent_m")}

    # ---------------------------------------------------------------- status
    def _entry_status(self, entry):
        if not entry:
            return None
        rc = entry["proc"].poll()
        return {"name": entry["name"], "pid": entry["proc"].pid, "alive": rc is None, "returncode": rc,
                "uptime_s": round(self.clock() - entry["started"], 1), "log": os.path.basename(entry["log"]),
                "args": entry.get("args", {})}

    @staticmethod
    def _port_open(port: int) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            return False

    def status(self) -> dict:
        with self.lock:
            st = self._entry_status(self.stack)
            if st and not st["alive"]:
                self.last_exit = "%s exited with code %s" % (st["name"], st["returncode"])
            mode = st["name"] if st and st["alive"] else "idle"
            try:
                load1 = os.getloadavg()[0]
            except OSError:
                load1 = None
            try:
                with open("/proc/uptime") as fh:
                    uptime = float(fh.read().split()[0])
            except Exception:
                uptime = None
            free = shutil.disk_usage(str(self.maps_dir)).free / 1e9
            return {"service": "jetbot-manager", "version": VERSION, "time": self.clock(), "mode": mode, "stack": st,
                    "base": self._entry_status(self.base), "last_exit": self.last_exit,
                    "ros_master": self._port_open(11311), "rosbridge": self._port_open(9090), "firewall": self.firewall(),
                    "active_map": self.active_map, "maps": len(self.list_maps()), "config": self.get_config(),
                    "host": {"name": socket.gethostname(), "uptime_s": uptime, "load1": load1, "disk_free_gb": round(free, 1)}}

    # ---------------------------------------------------------------- maps
    def _map_info(self, name: str):
        y = self.maps_dir / (name + ".yaml")
        if not y.is_file():
            return None
        try:
            meta = parse_map_yaml(y)
        except Exception:
            return None
        pgm = self.maps_dir / (name + ".pgm")
        if not pgm.is_file():
            img = meta.get("image")
            pgm = self.maps_dir / os.path.basename(img) if img else pgm
        size = read_pgm_size(pgm) if pgm.is_file() else None
        if size is None:
            return None
        return {"name": name, "width": size[0], "height": size[1], "resolution": meta.get("resolution"), "origin": meta.get("origin"),
                "bytes": pgm.stat().st_size, "modified": int(max(y.stat().st_mtime, pgm.stat().st_mtime)),
                "active": name == self.active_map, "_pgm": str(pgm)}

    def list_maps(self) -> list:
        out = []
        for y in sorted(self.maps_dir.glob("*.yaml")):
            if not NAME_RE.match(y.stem):
                continue
            info = self._map_info(y.stem)
            if info:
                info.pop("_pgm")
                out.append(info)
        return out

    def map_pgm_bytes(self, name: str) -> bytes:
        info = self._map_info(valid_name(name))
        if info is None:
            raise ApiError(404, "no such map")
        return Path(info["_pgm"]).read_bytes()

    def _map_in_use(self, name: str) -> bool:
        st = self.stack
        return bool(self._alive(st) and st["name"] == "navigation" and st.get("args", {}).get("map_file", "").endswith("/" + name + ".yaml"))

    def save_map(self, name: str) -> dict:
        name = valid_name(name)
        with self.lock:
            if not (self._alive(self.stack) and self.stack["name"] == "mapping"):
                raise ApiError(409, "saving needs mapping mode: start 'mapping' first")
            if (self.maps_dir / (name + ".yaml")).exists() or (self.maps_dir / (name + ".pgm")).exists():
                raise ApiError(409, "a map named '%s' already exists (maps are never overwritten; pick another name)" % name)
            target = self.maps_dir / name
            r = self._run("rosrun map_server map_saver -f %s" % shlex.quote(str(target)), 90)
            if r.returncode != 0 or not (self.maps_dir / (name + ".yaml")).is_file():
                for ext in (".pgm", ".yaml"):
                    try:
                        (self.maps_dir / (name + ext)).unlink()
                    except OSError:
                        pass
                raise ApiError(500, "map_saver failed: %s" % r.stdout.decode(errors="replace")[-300:])
            rewrite_image_line(self.maps_dir / (name + ".yaml"), name + ".pgm")        # relative path: the map survives being moved
            info = self._map_info(name)
            info.pop("_pgm")
            return info

    def activate_map(self, name: str) -> dict:
        name = valid_name(name)
        if self._map_info(name) is None:
            raise ApiError(404, "no such map")
        self.set_active(name)
        return {"active_map": name}

    def delete_map(self, name: str) -> dict:
        name = valid_name(name)
        with self.lock:
            if self._map_info(name) is None:
                raise ApiError(404, "no such map")
            if self._map_in_use(name):
                raise ApiError(409, "that map is in use by the running navigation: stop it first")
            stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(self.clock()))
            dest = self.maps_dir / ".trash" / ("%s__%s" % (name, stamp))
            dest.mkdir(parents=True, exist_ok=True)
            info = self._map_info(name)
            shutil.move(str(self.maps_dir / (name + ".yaml")), str(dest / (name + ".yaml")))
            shutil.move(info["_pgm"], str(dest / (name + ".pgm")))
            rewrite_image_line(dest / (name + ".yaml"), name + ".pgm")
            if self.active_map == name:
                self.set_active(None)
            return {"trashed": dest.name}

    def list_trash(self) -> list:
        out = []
        for d in sorted((self.maps_dir / ".trash").iterdir() if (self.maps_dir / ".trash").exists() else []):
            if d.is_dir() and "__" in d.name:
                out.append({"entry": d.name, "name": d.name.split("__")[0], "when": d.name.split("__")[1]})
        return out

    def restore_map(self, entry: str) -> dict:
        if not isinstance(entry, str) or not re.match(r"^[A-Za-z0-9_-]+__[0-9]{8}-[0-9]{6}$", entry):
            raise ApiError(400, "bad trash entry")
        src = self.maps_dir / ".trash" / entry
        name = entry.split("__")[0]
        if not src.is_dir():
            raise ApiError(404, "no such trash entry")
        with self.lock:
            if (self.maps_dir / (name + ".yaml")).exists() or (self.maps_dir / (name + ".pgm")).exists():
                raise ApiError(409, "a map named '%s' exists now; rename or delete it first" % name)
            shutil.move(str(src / (name + ".yaml")), str(self.maps_dir / (name + ".yaml")))
            shutil.move(str(src / (name + ".pgm")), str(self.maps_dir / (name + ".pgm")))
            try:
                src.rmdir()
            except OSError:
                pass
            return {"restored": name}

    def rename_map(self, name: str, new_name: str) -> dict:
        name, new_name = valid_name(name), valid_name(new_name)
        with self.lock:
            info = self._map_info(name)
            if info is None:
                raise ApiError(404, "no such map")
            if self._map_in_use(name):
                raise ApiError(409, "that map is in use by the running navigation: stop it first")
            if (self.maps_dir / (new_name + ".yaml")).exists() or (self.maps_dir / (new_name + ".pgm")).exists():
                raise ApiError(409, "a map named '%s' already exists" % new_name)
            shutil.move(str(self.maps_dir / (name + ".yaml")), str(self.maps_dir / (new_name + ".yaml")))
            shutil.move(info["_pgm"], str(self.maps_dir / (new_name + ".pgm")))
            rewrite_image_line(self.maps_dir / (new_name + ".yaml"), new_name + ".pgm")
            if self.active_map == name:
                self.set_active(new_name)
            return {"renamed": new_name}

    def tail_log(self, which: str, lines: int = 200) -> str:
        entry = {"stack": self.stack, "base": self.base}.get(which)
        if not entry:
            raise ApiError(404, "no %s log yet" % which)
        lines = max(1, min(int(lines), 1000))
        data = Path(entry["log"]).read_bytes()[-60000:].decode(errors="replace")
        return "\n".join(data.splitlines()[-lines:])

    def shutdown(self) -> None:
        with self.lock:
            self._stop_entry(self.stack)
            self._stop_entry(self.base)


# ====================================================================== HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "JetbotManager/1"
    manager: Manager = None        # set by serve()
    token: str = ""

    def log_message(self, fmt, *args):            # keep the journal quiet; the manager logs what matters
        pass

    def _send(self, status: int, payload, ctype="application/json"):
        body = payload if isinstance(payload, (bytes, bytearray)) else json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _authorised(self) -> bool:
        got = self.headers.get("X-Jetbot-Token", "")
        return bool(self.token) and len(got) == len(self.token) and all(a == b for a, b in zip(got, self.token))

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if n > 8192:
            raise ApiError(400, "request too large")
        if n == 0:
            return {}
        try:
            data = json.loads(self.rfile.read(n).decode())
        except ValueError:
            raise ApiError(400, "body must be JSON")
        if not isinstance(data, dict):
            raise ApiError(400, "body must be a JSON object")
        return data

    def _dispatch(self, method: str):
        u = urlparse(self.path)
        path = u.path.rstrip("/") or "/"
        q = parse_qs(u.query)
        m = self.manager
        try:
            if method == "GET" and path == "/ping":
                return self._send(200, {"ok": True, "service": "jetbot-manager", "version": VERSION})
            if not self._authorised():
                raise ApiError(401, "missing or wrong token")
            if method == "GET":
                if path == "/status":
                    return self._send(200, m.status())
                if path == "/maps":
                    return self._send(200, {"maps": m.list_maps(), "active": m.active_map, "trash": m.list_trash()})
                mm = re.match(r"^/maps/([^/]+)/image$", path)
                if mm:
                    return self._send(200, m.map_pgm_bytes(unquote(mm.group(1))), "application/octet-stream")
                if path == "/logs":
                    return self._send(200, {"text": m.tail_log((q.get("name") or ["stack"])[0], int((q.get("lines") or ["200"])[0]))})
                if path == "/config":
                    return self._send(200, m.get_config())
            elif method == "POST":
                body = self._body()
                if path == "/start":
                    return self._send(200, m.start(body.get("mode"), body, bool(body.get("replace"))))
                if path == "/stop":
                    return self._send(200, m.stop())
                if path == "/base/start":
                    return self._send(200, m.start_base())
                if path == "/base/stop":
                    return self._send(200, m.stop_base())
                if path == "/maps/save":
                    return self._send(200, m.save_map(body.get("name")))
                if path == "/maps/activate":
                    return self._send(200, m.activate_map(body.get("name")))
                if path == "/maps/delete":
                    return self._send(200, m.delete_map(body.get("name")))
                if path == "/maps/restore":
                    return self._send(200, m.restore_map(body.get("entry")))
                if path == "/maps/rename":
                    return self._send(200, m.rename_map(body.get("name"), body.get("new_name")))
                if path == "/config":
                    return self._send(200, m.set_config(body))
            raise ApiError(404, "unknown endpoint")
        except ApiError as e:
            self._send(e.status, {"error": e.message})
        except Exception as exc:                  # never leak a traceback to the client
            sys.stderr.write("manager error on %s %s: %r\n" % (method, path, exc))
            self._send(500, {"error": "internal error"})

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def load_token(state_dir: Path) -> str:
    path = state_dir / "token"
    if not path.exists():
        import secrets
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as fh:
            fh.write(secrets.token_hex(24))
    os.chmod(str(path), 0o600)
    return path.read_text().strip()


def serve(manager: Manager, host: str = "127.0.0.1", port: int = None, token: str = None) -> Server:
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("refusing to listen on %s: this service must stay on localhost (use the SSH tunnel)" % host)
    Handler.manager = manager
    Handler.token = token if token is not None else load_token(manager.state_dir)
    return Server((host, port or manager.cfg["port"]), Handler)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--config", help="JSON file overriding the defaults (default: ~/.jetbot/manager.json if present)")
    p.add_argument("--port", type=int)
    p.add_argument("--no-base", action="store_true", help="do not start rosbridge at launch")
    args = p.parse_args(argv)
    cfg = {}
    path = Path(args.config) if args.config else Path(DEFAULTS["state_dir"]) / "manager.json"
    if path.exists():
        cfg = json.loads(path.read_text())
    if args.port:
        cfg["port"] = args.port
    mgr = Manager(cfg)
    srv = serve(mgr)
    if cfg.get("autostart_base", DEFAULTS["autostart_base"]) and not args.no_base:
        try:
            mgr.start_base()
        except ApiError as e:
            sys.stderr.write("base not started: %s\n" % e.message)
    stopping = threading.Event()

    def bye(*_):
        stopping.set()
        threading.Thread(target=srv.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, bye)
    signal.signal(signal.SIGINT, bye)
    sys.stderr.write("jetbot-manager listening on 127.0.0.1:%d\n" % srv.server_address[1])
    try:
        srv.serve_forever()
    finally:
        mgr.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
