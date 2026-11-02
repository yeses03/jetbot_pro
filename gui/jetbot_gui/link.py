"""Robot communication layer: rosbridge (websocket) via roslibpy, plus the latest-value data store.

Stage 2 is deliberately READ-ONLY: this module subscribes and queries, it never publishes to the robot.

Threading: roslibpy runs its own network thread; every callback here only stores data under a lock.
The Tk thread calls snapshot() to read a consistent copy. Nothing here blocks the UI thread.
Every value carries the *local* time it was received so the UI can refuse to show stale data as live.
"""
from __future__ import annotations

import collections
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

from .config import Config
from .geometry import (MapData, Pose2D, ScanData, TfBuffer, clean_frame, map_from_message,
                       quat_to_yaw, scan_from_message)

# connection states
DISCONNECTED, CONNECTING, CONNECTED = "disconnected", "connecting", "connected"


@dataclass
class TopicStat:
    count: int = 0
    last_rx: Optional[float] = None
    last_error: str = ""


@dataclass
class Snapshot:
    now: float
    conn_state: str
    conn_detail: str
    host: str
    port: int
    last_rx_age: Optional[float]                    # seconds since ANY message arrived
    map: Optional[MapData]
    map_seq: int
    map_age: Optional[float]
    scan: Optional[ScanData]
    scan_age: Optional[float]
    scan_hz: float
    pose_map: Optional[Pose2D]                      # map -> base via TF (None if no chain)
    pose_map_age: Optional[float]
    pose_odom: Optional[Pose2D]                     # odom -> base via TF
    pose_odom_age: Optional[float]
    scan_in_map: Optional[Pose2D]                   # laser frame pose in map frame
    scan_in_map_age: Optional[float]
    scan_in_base: Optional[Pose2D]
    odom_topic_pose: Optional[Pose2D]
    odom_topic_age: Optional[float]
    odom_frame: str
    amcl_pose: Optional[Pose2D]
    amcl_sigma: Optional[tuple]                     # (sigma_xy, sigma_yaw)
    amcl_age: Optional[float]
    stats: Dict[str, TopicStat]
    nodes: Optional[Set[str]]                       # None = rosapi unavailable / not asked yet
    topics: Optional[Dict[str, str]]
    discovery_error: str
    tf_frames: list


class RobotLink:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._gen = 0                               # bumps on every connect/disconnect; stale callbacks self-discard
        self._ros = None
        self._state, self._detail = DISCONNECTED, "not connected"
        self._host, self._port = cfg.host, cfg.port
        self._reset_data()

    # ---------------------------------------------------------------- data store
    def _reset_data(self):
        self._tf = TfBuffer()
        self._map: Optional[MapData] = None
        self._map_rx: Optional[float] = None
        self._map_seq = 0
        self._scan: Optional[ScanData] = None
        self._scan_rx: Optional[float] = None
        self._scan_times = collections.deque(maxlen=30)
        self._odom = None                           # (pose, frame, rx)
        self._amcl = None                           # (pose, sigma, rx)
        self._last_rx_any: Optional[float] = None
        self._stats: Dict[str, TopicStat] = collections.defaultdict(TopicStat)
        self._nodes: Optional[Set[str]] = None
        self._topics: Optional[Dict[str, str]] = None
        self._disc_error = "not asked yet"

    # ---------------------------------------------------------------- lifecycle
    def connect(self, host: str, port: int) -> None:
        """Start (or restart) a connection attempt in the background. Returns immediately."""
        self.disconnect()
        with self._lock:
            self._gen += 1
            gen = self._gen
            self._host, self._port = host, int(port)
            self._reset_data()
            self._state, self._detail = CONNECTING, f"connecting to ws://{host}:{port} ..."
        threading.Thread(target=self._connect_worker, args=(gen, host, int(port)), daemon=True,
                         name="rosbridge-connect").start()

    def disconnect(self) -> None:
        with self._lock:
            self._gen += 1                           # invalidates every in-flight callback and worker
            ros, self._ros = self._ros, None
            self._state, self._detail = DISCONNECTED, "not connected"
        if ros is not None:
            threading.Thread(target=self._close_ros, args=(ros,), daemon=True).start()

    @staticmethod
    def _close_ros(ros):
        try:
            stop = getattr(ros.factory, "stopTrying", None)     # stop auto-reconnect
            if stop:
                stop()
            ros.close(timeout=2)
        except Exception:
            pass

    def _alive(self, gen: int) -> bool:
        return gen == self._gen

    def _set_state(self, gen: int, state: str, detail: str) -> None:
        with self._lock:
            if self._alive(gen):
                self._state, self._detail = state, detail

    def _connect_worker(self, gen: int, host: str, port: int) -> None:
        try:
            import roslibpy
        except ImportError:
            self._set_state(gen, DISCONNECTED, "roslibpy is not installed (pip install -r requirements.txt)")
            return
        try:
            ros = roslibpy.Ros(host, port)
            with self._lock:
                alive = self._alive(gen)
                if alive:
                    self._ros = ros
            if not alive:
                threading.Thread(target=self._close_ros, args=(ros,), daemon=True).start()
                return
            try:
                ros.run(timeout=5)
            except Exception:
                pass                                 # keep retrying in the monitor loop below
            self._subscribe_all(gen, ros)
            self._monitor(gen, ros)
        except Exception as exc:                     # bad hostname etc.
            self._set_state(gen, DISCONNECTED, f"connection failed: {type(exc).__name__}: {exc}")

    def _monitor(self, gen: int, ros) -> None:
        """Track connection state and run periodic rosapi discovery until this generation ends."""
        next_disc = 0.0
        was_connected = False
        while self._alive(gen):
            connected = bool(ros.is_connected)
            if connected:
                if not was_connected:
                    self._set_state(gen, CONNECTED, f"connected to ws://{self._host}:{self._port}")
                if time.monotonic() >= next_disc:
                    next_disc = time.monotonic() + self.cfg.discovery_period_s
                    self._discover(gen, ros)
            else:
                msg = "connection lost, retrying ..." if was_connected else \
                    f"cannot reach ws://{self._host}:{self._port} (is rosbridge running? retrying ...)"
                self._set_state(gen, CONNECTING, msg)
            was_connected = connected
            time.sleep(0.5)

    # ---------------------------------------------------------------- subscriptions
    def _subscribe_all(self, gen: int, ros) -> None:
        import roslibpy
        c = self.cfg
        subs = [
            (c.tf_topic, "tf2_msgs/TFMessage", 0, lambda m: self._on_tf(gen, m, False)),
            (c.tf_static_topic, "tf2_msgs/TFMessage", 0, lambda m: self._on_tf(gen, m, True)),
            (c.scan_topic, "sensor_msgs/LaserScan", c.scan_throttle_ms, lambda m: self._on_scan(gen, m)),
            (c.map_topic, "nav_msgs/OccupancyGrid", c.map_throttle_ms, lambda m: self._on_map(gen, m)),
            (c.odom_topic, "nav_msgs/Odometry", 100, lambda m: self._on_odom(gen, m)),
            (c.amcl_pose_topic, "geometry_msgs/PoseWithCovarianceStamped", 0, lambda m: self._on_amcl(gen, m)),
        ]
        self._topics_objs = []
        for name, typ, throttle, cb in subs:
            # queue_length=1: if we fall behind, rosbridge drops old messages instead of replaying them late.
            is_tf = name in (c.tf_topic, c.tf_static_topic)
            t = roslibpy.Topic(ros, name, typ, throttle_rate=throttle, queue_length=0 if is_tf else 1)
            t.subscribe(cb)
            self._topics_objs.append(t)

    def _note(self, gen: int, topic: str, error: str = "") -> bool:
        with self._lock:
            if not self._alive(gen):
                return False
            now = time.monotonic()
            st = self._stats[topic]
            if error:
                st.last_error = error
                return False
            st.count += 1
            st.last_rx, st.last_error = now, ""
            self._last_rx_any = now
            return True

    def _on_tf(self, gen, msg, static):
        topic = self.cfg.tf_static_topic if static else self.cfg.tf_topic
        try:
            if self._note(gen, topic):
                with self._lock:
                    self._tf.update(msg["transforms"], static)
        except Exception as exc:
            self._note(gen, topic, f"bad message: {exc!r}")

    def _on_scan(self, gen, msg):
        try:
            scan = scan_from_message(msg)
            if self._note(gen, self.cfg.scan_topic):
                with self._lock:
                    self._scan, self._scan_rx = scan, time.monotonic()
                    self._scan_times.append(self._scan_rx)
        except Exception as exc:
            self._note(gen, self.cfg.scan_topic, f"bad message: {exc!r}")

    def _on_map(self, gen, msg):
        try:
            m = map_from_message(msg)
            m.dist_to_wall()                    # precompute here (background thread), never on the UI thread
            if self._note(gen, self.cfg.map_topic):
                with self._lock:
                    self._map, self._map_rx = m, time.monotonic()
                    self._map_seq += 1
        except Exception as exc:
            self._note(gen, self.cfg.map_topic, f"bad map: {exc!r}")

    def _on_odom(self, gen, msg):
        try:
            p, q = msg["pose"]["pose"]["position"], msg["pose"]["pose"]["orientation"]
            pose = (float(p["x"]), float(p["y"]), quat_to_yaw(q["x"], q["y"], q["z"], q["w"]))
            if self._note(gen, self.cfg.odom_topic):
                with self._lock:
                    self._odom = (pose, clean_frame(msg["header"]["frame_id"]), time.monotonic())
        except Exception as exc:
            self._note(gen, self.cfg.odom_topic, f"bad message: {exc!r}")

    def _on_amcl(self, gen, msg):
        try:
            p, q = msg["pose"]["pose"]["position"], msg["pose"]["pose"]["orientation"]
            cov = msg["pose"]["covariance"]
            pose = (float(p["x"]), float(p["y"]), quat_to_yaw(q["x"], q["y"], q["z"], q["w"]))
            sigma = (max(float(cov[0]), 0.0) ** 0.5 if cov[0] is not None else float("nan"),
                     max(float(cov[35]), 0.0) ** 0.5 if cov[35] is not None else float("nan"))
            if self._note(gen, self.cfg.amcl_pose_topic):
                with self._lock:
                    self._amcl = (pose, sigma, time.monotonic())
        except Exception as exc:
            self._note(gen, self.cfg.amcl_pose_topic, f"bad message: {exc!r}")

    # ---------------------------------------------------------------- rosapi discovery
    def _discover(self, gen: int, ros) -> None:
        def got_nodes(res):
            try:
                nodes = set(res["nodes"])
            except Exception as exc:
                return failed(f"unexpected /rosapi/nodes reply: {exc!r}")
            with self._lock:
                if self._alive(gen):
                    self._nodes, self._disc_error = nodes, ""

        def got_topics(res):
            try:
                topics = dict(zip(res["topics"], res["types"]))
            except Exception as exc:
                return failed(f"unexpected /rosapi/topics reply: {exc!r}")
            with self._lock:
                if self._alive(gen):
                    self._topics = topics

        def failed(err):
            with self._lock:
                if self._alive(gen):
                    self._disc_error = f"rosapi unavailable: {err}"
                    self._nodes = self._topics = None
        try:
            ros.get_nodes(got_nodes, failed)
            ros.get_topics(got_topics, failed)
        except Exception as exc:
            failed(exc)

    # ---------------------------------------------------------------- read side (UI thread)
    def snapshot(self) -> Snapshot:
        c = self.cfg
        with self._lock:
            now = time.monotonic()

            def age(t):
                return None if t is None else now - t

            scan_frame = self._scan.frame_id if self._scan else None
            pm = self._tf.lookup(c.map_frame, c.base_frame)
            po = self._tf.lookup(c.odom_frame, c.base_frame)
            sm = self._tf.lookup(c.map_frame, scan_frame) if scan_frame else None
            sb = self._tf.lookup(c.base_frame, scan_frame) if scan_frame else None
            hz = 0.0
            if len(self._scan_times) > 1 and self._scan_times[-1] > self._scan_times[0]:
                hz = (len(self._scan_times) - 1) / (self._scan_times[-1] - self._scan_times[0])
            return Snapshot(
                now=now, conn_state=self._state, conn_detail=self._detail, host=self._host, port=self._port,
                last_rx_age=age(self._last_rx_any),
                map=self._map, map_seq=self._map_seq, map_age=age(self._map_rx),
                scan=self._scan, scan_age=age(self._scan_rx), scan_hz=hz,
                pose_map=pm[0] if pm else None, pose_map_age=pm[1] if pm else None,
                pose_odom=po[0] if po else None, pose_odom_age=po[1] if po else None,
                scan_in_map=sm[0] if sm else None, scan_in_map_age=sm[1] if sm else None,
                scan_in_base=sb[0] if sb else None,
                odom_topic_pose=self._odom[0] if self._odom else None,
                odom_topic_age=age(self._odom[2]) if self._odom else None,
                odom_frame=self._odom[1] if self._odom else "",
                amcl_pose=self._amcl[0] if self._amcl else None,
                amcl_sigma=self._amcl[1] if self._amcl else None,
                amcl_age=age(self._amcl[2]) if self._amcl else None,
                stats={k: TopicStat(v.count, v.last_rx, v.last_error) for k, v in self._stats.items()},
                nodes=set(self._nodes) if self._nodes is not None else None,
                topics=dict(self._topics) if self._topics is not None else None,
                discovery_error=self._disc_error, tf_frames=self._tf.frames())
