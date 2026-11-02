"""GUI settings: defaults, an optional local override file, and command-line overrides.

Precedence (low -> high): built-in DEFAULTS < config.default.json < config.local.json < CLI flags.
config.local.json is git-ignored; put your robot's IP there.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parent.parent
DEFAULT_FILE = GUI_DIR / "config.default.json"
LOCAL_FILE = GUI_DIR / "config.local.json"


@dataclass
class Config:
    host: str = ""
    port: int = 9090
    # Topics. Defaults match the repo's launch files; verify on your robot (see README).
    map_topic: str = "/map"
    scan_topic: str = "/scan"
    odom_topic: str = "/odom"
    amcl_pose_topic: str = "/amcl_pose"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    # Goals: the ONLY topics the GUI ever publishes to (it never publishes velocity commands).
    goal_topic: str = "/move_base_simple/goal"
    cancel_topic: str = "/move_base/cancel"
    goal_status_topic: str = "/move_base/status"
    # Frames (jetbot_pro launch files use base_footprint, odom, map, laser_frame).
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_footprint"
    # A value is "live" only if a message arrived within this many seconds (local receive time).
    scan_stale_s: float = 2.0
    pose_stale_s: float = 2.0
    odom_stale_s: float = 2.0
    # Never fetch the same big topic faster than this (ms, applied by rosbridge).
    scan_throttle_ms: int = 100
    map_throttle_ms: int = 500
    # How often to ask rosapi which nodes/topics exist (seconds).
    discovery_period_s: float = 5.0
    lidar_range_m: float = 6.0
    # Pose-vs-scan fit (see geometry.fit_score): share of scan points on mapped walls.
    fit_tolerance_m: float = 0.05
    fit_ok: float = 0.70
    fit_warn: float = 0.60
    amcl_sigma_xy_warn_m: float = 0.30
    amcl_sigma_yaw_warn_deg: float = 20.0

    def to_dict(self) -> dict:
        return asdict(self)


def _apply(cfg: Config, data: dict, source: str) -> None:
    known = {f.name: f for f in fields(Config)}
    for key, value in data.items():
        if key.startswith("_"):
            continue
        if key not in known:
            raise ValueError(f"{source}: unknown setting '{key}'")
        cfg.__dict__[key] = type(getattr(cfg, key))(value)


def load(cli_overrides: dict | None = None) -> Config:
    cfg = Config()
    for path in (DEFAULT_FILE, LOCAL_FILE):
        if path.exists():
            _apply(cfg, json.loads(path.read_text()), path.name)
    if cli_overrides:
        _apply(cfg, {k: v for k, v in cli_overrides.items() if v is not None}, "command line")
    return cfg


def save_local(cfg: Config) -> None:
    """Persist the current settings as the local override file."""
    LOCAL_FILE.write_text(json.dumps(cfg.to_dict(), indent=2) + "\n")
