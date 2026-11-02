"""Teleop logic. Pure: no Tk, no network, unit-tested.

Mirrors teleop_twist_keyboard as you already use it: i forward, comma backward, j turn left, l turn right, k stop; each key press sets
one pure command (no diagonals) that is held for HOLD_S seconds after the last key event, so a lost key-release or a hung window can
never keep the robot moving for long. Speeds are capped here AND again in the link, so no UI bug can exceed them.

The obstacle guard is a software bumper built on the LiDAR: it blocks motion towards something that is close. It is a convenience, not
a safety system: the LiDAR cannot see anything closer than about 0.15 m, glass, or things above or below its plane. The power switch
is the real stop.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

MAX_LINEAR = 0.25            # m/s, commanded (the robot's real speed is lower)
MAX_ANGULAR = 0.8            # rad/s
HOLD_S = 0.7                 # same as teleop_twist_keyboard _key_timeout used in the project
DEFAULT_LINEAR = 0.15
DEFAULT_ANGULAR = 0.5

# unit commands: (linear, angular)
KEYMAP = {"i": (1.0, 0.0), "comma": (-1.0, 0.0), "j": (0.0, 1.0), "l": (0.0, -1.0),
          "Up": (1.0, 0.0), "Down": (-1.0, 0.0), "Left": (0.0, 1.0), "Right": (0.0, -1.0)}
STOP_KEYS = {"k", "space"}

# obstacle guard geometry, robot frame (x forward, y left), metres from the LiDAR centre (over the wheel axis)
AHEAD_X = 0.42
HALF_WIDTH = 0.20
SPIN_RADIUS = 0.23


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def clamp_command(vx: float, wz: float) -> Tuple[float, float]:
    return clamp(vx, MAX_LINEAR), clamp(wz, MAX_ANGULAR)


class TeleopState:
    def __init__(self, hold_s: float = HOLD_S, linear: float = DEFAULT_LINEAR, angular: float = DEFAULT_ANGULAR):
        self.hold_s = hold_s
        self.linear, self.angular = DEFAULT_LINEAR, DEFAULT_ANGULAR
        self.set_speeds(linear, angular)
        self._unit = (0.0, 0.0)
        self._until = 0.0
        self._key: Optional[str] = None

    def set_speeds(self, linear: float, angular: float) -> None:
        self.linear = max(0.0, min(MAX_LINEAR, float(linear)))
        self.angular = max(0.0, min(MAX_ANGULAR, float(angular)))

    def stop(self) -> None:
        self._unit, self._until, self._key = (0.0, 0.0), 0.0, None

    def key_down(self, keysym: str, now: float) -> bool:
        """Returns True if the key is a teleop key (so the UI can swallow it)."""
        if keysym in STOP_KEYS:
            self.stop()
            return True
        unit = KEYMAP.get(keysym)
        if unit is None:
            return False
        self._unit, self._until, self._key = unit, now + self.hold_s, keysym
        return True

    def key_up(self, keysym: str) -> None:
        if keysym == self._key:
            self.stop()

    def hold(self, unit: Tuple[float, float], now: float, key: str = "pad") -> None:
        """On-screen pad: a pressed button refreshes the same hold timer."""
        self._unit, self._until, self._key = unit, now + self.hold_s, key

    def release(self, key: str = "pad") -> None:
        self.key_up(key)

    def command(self, now: float) -> Tuple[float, float]:
        if now >= self._until or self._unit == (0.0, 0.0):
            return 0.0, 0.0
        return clamp_command(self._unit[0] * self.linear, self._unit[1] * self.angular)


def guard(vx: float, wz: float, pts: Optional[np.ndarray]) -> Tuple[float, float, str]:
    """Zero the part of a command that heads into a nearby obstacle. pts = LiDAR points in the robot frame, (N, 2).

    Returns (vx, wz, reason). With no scan (None) nothing moves: the guard cannot see, so it does not let the robot drive blind.
    """
    if vx == 0.0 and wz == 0.0:
        return 0.0, 0.0, ""
    if pts is None:
        return 0.0, 0.0, "no live LiDAR: movement disabled"
    pts = np.asarray(pts, float).reshape(-1, 2)
    x, y = pts[:, 0], pts[:, 1]
    if vx > 0:
        near = x[(x > 0.0) & (x < AHEAD_X) & (np.abs(y) < HALF_WIDTH)]
        if near.size:
            return 0.0, 0.0, "obstacle %.2f m ahead" % float(near.min())
    elif vx < 0:
        near = -x[(x < 0.0) & (x > -AHEAD_X) & (np.abs(y) < HALF_WIDTH)]
        if near.size:
            return 0.0, 0.0, "obstacle %.2f m behind (charger cable?)" % float(near.min())
    if wz != 0.0 and vx == 0.0:
        r = np.hypot(x, y)
        if r.size and float(r.min()) < SPIN_RADIUS:
            return 0.0, 0.0, "obstacle %.2f m away: no room to turn" % float(r.min())
    return vx, wz, ""
