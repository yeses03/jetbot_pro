"""Goal sending rules. Pure logic: no Tk, no network, unit-tested.

The GUI can only ask move_base for a goal or cancel goals. It never publishes velocity commands, so move_base's
obstacle handling is always in the loop. Cancelling a goal stops the robot but is NOT an emergency stop: the
power switch is the real stop.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .config import Config
from .link import CONNECTED, Snapshot
from .status import BAD, OK, Assessment

Goal = Tuple[float, float, float]              # x, y, yaw in the map frame

# actionlib_msgs/GoalStatus codes
PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, REJECTED = 0, 1, 2, 3, 4, 5
TERMINAL_FAILURES = {PREEMPTED, ABORTED, REJECTED, 8}      # 8 = RECALLED


def blockers(a: Assessment, s: Snapshot, cfg: Config, armed: bool) -> List[str]:
    """Reasons a goal must NOT be sent right now. Empty list = allowed."""
    why: List[str] = []
    if not armed:
        why.append("goals are not armed (tick 'Arm goals')")
    if s.conn_state != CONNECTED:
        why.append("not connected to the robot")
    if not a.pose_live_in_map:
        why.append("no live map-frame pose")
    if a.nav.level != OK:
        why.append("move_base is not running")
    if a.fit.level == BAD:
        why.append("the robot's pose does not match the LiDAR scan (it is probably mislocalised)")
    elif a.fit_value is None:
        why.append("cannot check the pose against the scan yet")
    if s.amcl_pose is not None and s.amcl_sigma is not None and s.amcl_age is not None \
            and s.amcl_age <= cfg.pose_stale_s:
        sxy, syaw = s.amcl_sigma
        if sxy > cfg.amcl_sigma_xy_warn_m or math.degrees(syaw) > cfg.amcl_sigma_yaw_warn_deg:
            why.append(f"AMCL is uncertain (sigma {sxy:.2f} m, {math.degrees(syaw):.0f} deg)")
    if s.goal is not None and s.goal[1] in (PENDING, ACTIVE) and (s.goal_age is None or s.goal_age < 30.0):
        why.append("a goal is already active (cancel it first)")
    return why


class RouteRunner:
    """Sends a list of goals one after another, only moving on after move_base reports success.

    Call step() every UI tick. It returns a goal to send now, or None. Any failure, cancellation or blocked
    safety check stops the route; it never retries on its own.
    """
    IDLE, READY, SENT, RUNNING, DONE, FAILED = "idle", "ready", "sent", "running", "done", "failed"

    def __init__(self, accept_timeout_s: float = 8.0):
        self.legs: List[Goal] = []
        self.idx = 0
        self.state = self.IDLE
        self.message = ""
        self._before_id: Optional[str] = None
        self._sent_at = 0.0
        self.accept_timeout_s = accept_timeout_s

    @property
    def running(self) -> bool:
        return self.state in (self.READY, self.SENT, self.RUNNING)

    def start(self, legs: List[Goal]) -> None:
        self.legs, self.idx = list(legs), 0
        self.state = self.READY if self.legs else self.IDLE
        self.message = f"route started: {len(self.legs)} legs" if self.legs else "route is empty"

    def stop(self, reason: str) -> None:
        if self.running:
            self.state, self.message = self.FAILED, reason

    def step(self, goal_status: Optional[tuple], block: List[str], now: float) -> Optional[Goal]:
        """goal_status = (goal_id, status_code, text) of the newest move_base goal, or None."""
        if not self.running:
            return None
        gid, code = (goal_status[0], goal_status[1]) if goal_status else (None, None)

        if self.state == self.READY:
            # `block` includes "a goal is already active" while a previous goal runs; wait for it, do not fail
            fatal = [b for b in block if "already active" not in b]
            if fatal:
                self.stop("route stopped before leg %d: %s" % (self.idx + 1, fatal[0]))
                return None
            if block:
                self.message = "waiting for the previous goal to finish"
                return None
            self._before_id, self._sent_at = gid, now
            self.state = self.SENT
            self.message = "leg %d of %d sent" % (self.idx + 1, len(self.legs))
            return self.legs[self.idx]

        if self.state == self.SENT:
            if gid is not None and gid != self._before_id:
                self.state = self.RUNNING
            elif now - self._sent_at > self.accept_timeout_s:
                self.stop("move_base did not accept leg %d" % (self.idx + 1))
            return None

        # RUNNING
        if code == SUCCEEDED:
            self.idx += 1
            if self.idx >= len(self.legs):
                self.state, self.message = self.DONE, "route finished"
            else:
                self.state, self.message = self.READY, "leg %d reached" % self.idx
        elif code in TERMINAL_FAILURES:
            text = goal_status[2] if goal_status and len(goal_status) > 2 else ""
            self.stop("leg %d failed: %s" % (self.idx + 1, text or "move_base aborted the goal"))
        else:
            self.message = "driving leg %d of %d" % (self.idx + 1, len(self.legs))
        return None
