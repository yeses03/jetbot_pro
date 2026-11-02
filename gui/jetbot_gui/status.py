"""Turns a Snapshot into honest, human-readable health indicators. Pure logic, unit-tested.

Rule: a value is shown as live only if it arrived recently (local receive time). Odometry-only
poses are labelled as such and are never presented as a map-localised pose.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import math

from .config import Config
from .geometry import fit_score, transform_points
from .link import CONNECTED, CONNECTING, Snapshot

OK, WARN, BAD, IDLE = "ok", "warn", "bad", "idle"

LOCALIZERS = {"amcl": "AMCL (localising on a saved map)",
              "slam_gmapping": "GMapping SLAM (building a map)",
              "hector_mapping": "Hector SLAM (building a map)",
              "karto": "Karto SLAM (building a map)",
              "slam_karto": "Karto SLAM (building a map)",
              "cartographer_node": "Cartographer (building a map)"}


@dataclass
class Item:
    level: str
    text: str


@dataclass
class Assessment:
    conn: Item
    map: Item
    scan: Item
    pose: Item
    odom: Item
    localizer: Item
    nav: Item
    fit: Item                    # does the scan agree with the map at the reported pose?
    fit_value: Optional[float]
    pose_live_in_map: bool       # True only when map->base is fresh: safe to draw the robot on the map
    scan_live: bool
    scan_on_map: bool            # scan fresh AND full TF chain to the map frame is fresh
    warnings: List[str]


def _node_match(nodes, name) -> bool:
    return any(n.rstrip("/").split("/")[-1] == name for n in nodes)


def assess(s: Snapshot, cfg: Config) -> Assessment:
    warn: List[str] = []

    # connection
    if s.conn_state == CONNECTED:
        if s.last_rx_age is None or s.last_rx_age > 3.0:
            conn = Item(WARN, "connected, but no data is arriving"
                        + ("" if s.last_rx_age is None else f" (last {s.last_rx_age:.0f}s ago)"))
            warn.append("Websocket is up but nothing has been received recently: is the robot's ROS master "
                        "alive and publishing?")
        else:
            conn = Item(OK, f"connected to {s.host}:{s.port}")
    elif s.conn_state == CONNECTING:
        conn = Item(WARN, s.conn_detail)
    else:
        conn = Item(BAD, s.conn_detail)

    # map
    if s.map is None:
        err = s.stats.get(cfg.map_topic).last_error if cfg.map_topic in s.stats else ""
        mp = Item(BAD, f"no map on {cfg.map_topic}" + (f" ({err})" if err else ""))
    else:
        m = s.map
        mp = Item(OK, f"{m.width}x{m.height} @ {m.resolution:.3f} m/cell, frame '{m.frame_id}', "
                      f"received {s.map_age:.0f}s ago")
        if m.frame_id != cfg.map_frame:
            mp = Item(WARN, mp.text)
            warn.append(f"Map frame is '{m.frame_id}' but the GUI is configured for '{cfg.map_frame}'.")

    # scan
    scan_live = s.scan is not None and s.scan_age is not None and s.scan_age <= cfg.scan_stale_s
    if s.scan is None:
        err = s.stats.get(cfg.scan_topic).last_error if cfg.scan_topic in s.stats else ""
        sc = Item(BAD, f"no scan on {cfg.scan_topic}" + (f" ({err})" if err else ""))
    elif not scan_live:
        sc = Item(BAD, f"scan STALE: last one {s.scan_age:.1f}s ago")
    else:
        sc = Item(OK, f"{s.scan_hz:.1f} Hz, frame '{s.scan.frame_id}'")

    # pose
    map_pose_fresh = s.pose_map is not None and s.pose_map_age is not None and s.pose_map_age <= cfg.pose_stale_s
    odom_pose_fresh = s.pose_odom is not None and s.pose_odom_age is not None and s.pose_odom_age <= cfg.pose_stale_s
    if map_pose_fresh:
        pose = Item(OK, f"{cfg.map_frame} -> {cfg.base_frame} live")
    elif s.pose_map is not None:
        pose = Item(BAD, f"map pose STALE: TF last updated {s.pose_map_age:.1f}s ago")
    elif odom_pose_fresh:
        pose = Item(WARN, f"ODOMETRY ONLY: no {cfg.map_frame}->{cfg.odom_frame} transform, so the robot is NOT "
                          "localised on the map")
        warn.append("No localisation: nothing is publishing the map->odom transform (start AMCL with a map, or SLAM).")
    elif s.pose_odom is not None:
        pose = Item(BAD, f"odom pose STALE: TF last updated {s.pose_odom_age:.1f}s ago")
    else:
        pose = Item(BAD, f"no TF chain to '{cfg.base_frame}'")

    # /odom topic
    if s.odom_topic_pose is None:
        od = Item(BAD, f"no data on {cfg.odom_topic}")
    elif s.odom_topic_age > cfg.odom_stale_s:
        od = Item(BAD, f"odometry STALE: {s.odom_topic_age:.1f}s")
    else:
        od = Item(OK, f"live (frame '{s.odom_frame}')")

    # scan -> map chain
    scan_on_map = bool(scan_live and map_pose_fresh and s.scan_in_map is not None
                       and s.scan_in_map_age is not None and s.scan_in_map_age <= cfg.pose_stale_s)
    if scan_live and not scan_on_map:
        warn.append(f"Scan is live but the TF chain {cfg.map_frame} -> '{s.scan.frame_id}' is missing or stale, so "
                    "it is not drawn on the map.")

    # pose vs scan: independent check that the reported map pose really explains what the LiDAR sees
    fit_value: Optional[float] = None
    if not scan_on_map or s.map is None:
        fit = Item(IDLE, "n/a (needs a live scan, map and map pose)")
    else:
        fit_value = fit_score(s.map, transform_points(s.scan.points(), s.scan_in_map), cfg.fit_tolerance_m)
        if fit_value is None:
            fit = Item(IDLE, "n/a (no wall data or too few scan points)")
        elif fit_value >= cfg.fit_ok:
            fit = Item(OK, f"{fit_value * 100:.0f}% of scan on map walls")
        elif fit_value >= cfg.fit_warn:
            fit = Item(WARN, f"{fit_value * 100:.0f}% of scan on map walls (marginal)")
            warn.append("The scan only partly matches the map at the reported pose: localisation may be drifting.")
        else:
            fit = Item(BAD, f"only {fit_value * 100:.0f}% of scan on map walls")
            warn.append("The scan does NOT match the map at the reported pose: the pose is probably wrong "
                        "(check the AMCL initial pose and odometry calibration), or the robot is outside the mapped area.")
    if s.amcl_pose is not None and s.amcl_sigma is not None and s.amcl_age is not None and s.amcl_age <= cfg.pose_stale_s:
        sxy, syaw = s.amcl_sigma
        if sxy > cfg.amcl_sigma_xy_warn_m or math.degrees(syaw) > cfg.amcl_sigma_yaw_warn_deg:
            warn.append(f"AMCL is uncertain (sigma {sxy:.2f} m, {math.degrees(syaw):.0f} deg): do not trust the pose "
                        "for fine positioning.")

    # what stack is running (needs rosapi)
    if s.nodes is None:
        loc = Item(IDLE, "unknown (" + (s.discovery_error or "rosapi not queried yet") + ")")
        nav = Item(IDLE, "unknown")
    else:
        found = [label for n, label in LOCALIZERS.items() if _node_match(s.nodes, n)]
        loc = Item(OK if found else WARN, ", ".join(found) if found else "none detected")
        if len(found) > 1:
            loc = Item(BAD, "CONFLICT: " + ", ".join(found))
            warn.append("More than one localiser/SLAM node is running; they will fight over map->odom.")
        nav = Item(OK, "move_base running") if _node_match(s.nodes, "move_base") else Item(IDLE, "move_base not running")
        if _node_match(s.nodes, "amcl") and map_pose_fresh is False and s.amcl_pose is None:
            warn.append("AMCL is running but has not published /amcl_pose yet (it needs an initial pose and/or "
                        "the robot to move).")

    for topic, st in s.stats.items():
        if st.last_error:
            warn.append(f"{topic}: {st.last_error}")

    return Assessment(conn, mp, sc, pose, od, loc, nav, fit, fit_value, map_pose_fresh, scan_live, scan_on_map, warn)
