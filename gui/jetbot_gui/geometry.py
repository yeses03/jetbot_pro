"""Pure geometry / data-conversion helpers (no Tk, no network) so they can be unit-tested.

All poses are planar (x, y, yaw) -- adequate for a differential-drive robot on a 2-D map.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np
from PIL import Image

Pose2D = Tuple[float, float, float]


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """Pose of frame C given a = pose of B in A and b = pose of C in B."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ath + bth))


def invert(a: Pose2D) -> Pose2D:
    ax, ay, ath = a
    c, s = math.cos(ath), math.sin(ath)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), wrap(-ath))


def clean_frame(frame_id: str) -> str:
    """ROS 1 launch files in this repo use '/base_footprint'-style ids; TF itself has no leading slash."""
    return frame_id.lstrip("/")


# --------------------------------------------------------------------------- TF
@dataclass
class _Edge:
    parent: str
    pose: Pose2D          # pose of child in parent
    rx: float             # local monotonic receive time
    static: bool


class TfBuffer:
    """Minimal latest-value TF tree built from /tf and /tf_static messages.

    Staleness is judged on *local receive time*, never on header stamps, because the Jetson's clock
    and this machine's clock are not synchronised.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._edges: Dict[str, _Edge] = {}      # child -> edge (a frame has exactly one parent)

    def update(self, transforms, static: bool) -> None:
        """transforms: iterable of rosbridge TFMessage 'transforms' dicts."""
        now = self._clock()
        for t in transforms:
            tr, q = t["transform"]["translation"], t["transform"]["rotation"]
            child = clean_frame(t["child_frame_id"])
            self._edges[child] = _Edge(
                clean_frame(t["header"]["frame_id"]),
                (float(tr["x"]), float(tr["y"]), quat_to_yaw(q["x"], q["y"], q["z"], q["w"])),
                now, static)

    def _chain_to_root(self, frame: str):
        """[(frame, pose-in-root accumulated)...] walking up; returns (root, pose_in_root, oldest_age)."""
        pose: Pose2D = (0.0, 0.0, 0.0)
        oldest = 0.0
        now = self._clock()
        seen = {frame}
        while frame in self._edges:
            e = self._edges[frame]
            pose = compose(e.pose, pose)
            if not e.static:
                oldest = max(oldest, now - e.rx)
            frame = e.parent
            if frame in seen:
                raise ValueError("cycle in TF tree")
            seen.add(frame)
        return frame, pose, oldest

    def lookup(self, target: str, source: str) -> Optional[Tuple[Pose2D, float]]:
        """Pose of `source` expressed in `target`, and the age (s) of the oldest dynamic link used.

        None if the two frames are not connected by received transforms.
        """
        target, source = clean_frame(target), clean_frame(source)
        try:
            root_t, t_in_root, age_t = self._chain_to_root(target)
            root_s, s_in_root, age_s = self._chain_to_root(source)
        except ValueError:
            return None
        if root_t != root_s:
            return None
        return compose(invert(t_in_root), s_in_root), max(age_t, age_s)

    def frames(self):
        return sorted(set(self._edges) | {e.parent for e in self._edges.values()})


# --------------------------------------------------------------------------- map
@dataclass
class MapData:
    width: int
    height: int
    resolution: float
    origin: Pose2D            # pose of cell (0,0) corner in the map frame
    frame_id: str
    rgb: Image.Image          # image row 0 = top = highest y
    occ: Optional[np.ndarray] = None            # bool (height, width), grid coordinates (row 0 = bottom): occupied cells
    _dist: Optional[np.ndarray] = field(default=None, init=False, repr=False, compare=False)

    def dist_to_wall(self, max_cells: int = 12) -> Optional[np.ndarray]:
        """Distance in cells (4-neighbourhood, capped) from every cell to the nearest occupied cell. Cached.

        Call once from a background thread: it is O(max_cells * cells), fine for 320x320, ~1 s for 2000x2000.
        """
        if self.occ is None:
            return None
        if self._dist is None:
            occ = self.occ
            d = np.full(occ.shape, max_cells, np.uint8)
            d[occ] = 0
            cur = occ.copy()
            for k in range(1, max_cells + 1):
                nxt = cur.copy()
                nxt[1:, :] |= cur[:-1, :]
                nxt[:-1, :] |= cur[1:, :]
                nxt[:, 1:] |= cur[:, :-1]
                nxt[:, :-1] |= cur[:, 1:]
                d[nxt & ~cur] = k
                cur = nxt
            self._dist = d
        return self._dist

    def world_to_cell(self, x: float, y: float) -> Tuple[float, float]:
        """Continuous (col, row) in grid coordinates (row 0 = bottom, y up)."""
        ox, oy, oth = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(oth), math.sin(oth)
        return (c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution


FREE_RGB, UNKNOWN_RGB, OCC_RGB = (250, 250, 250), (150, 150, 150), (15, 15, 15)
OCC_THRESHOLD = 65            # occupancy value (0-100) from which a cell counts as a wall (map_server's 0.65)


def occupancy_to_image(data, width: int, height: int) -> Image.Image:
    """nav_msgs/OccupancyGrid.data -> RGB image. -1 unknown, 0 free, 100 occupied, between = shaded."""
    a = np.asarray(data, dtype=np.int16)
    if a.size != width * height:
        raise ValueError(f"map data has {a.size} cells, expected {width}x{height}")
    a = a.reshape(height, width)
    rgb = np.empty((height, width, 3), np.uint8)
    rgb[:] = UNKNOWN_RGB
    rgb[a == 0] = FREE_RGB
    mid = (a > 0) & (a < 100)
    shade = (250 - a[mid] * 235 // 100).astype(np.uint8)       # probability -> grey ramp
    rgb[mid] = shade[:, None]
    rgb[a >= 100] = OCC_RGB
    return Image.fromarray(np.ascontiguousarray(np.flipud(rgb)))


def map_from_message(msg: dict) -> MapData:
    info = msg["info"]
    o = info["origin"]
    w, h = int(info["width"]), int(info["height"])
    res = float(info["resolution"])
    if w <= 0 or h <= 0 or res <= 0:
        raise ValueError("empty or invalid map metadata")
    q = o["orientation"]
    img = occupancy_to_image(msg["data"], w, h)            # also validates the cell count
    occ = np.asarray(msg["data"], dtype=np.int16).reshape(h, w) >= OCC_THRESHOLD
    return MapData(w, h, res, (float(o["position"]["x"]), float(o["position"]["y"]),
                               quat_to_yaw(q["x"], q["y"], q["z"], q["w"])),
                   clean_frame(msg["header"]["frame_id"]), img, occ)


@dataclass
class View:
    """Map-to-canvas view: world point (cx, cy) at the canvas centre, `scale` pixels per metre."""
    cx: float = 0.0
    cy: float = 0.0
    scale: float = 50.0

    def to_px(self, x: float, y: float, w: int, h: int) -> Tuple[float, float]:
        return w / 2 + (x - self.cx) * self.scale, h / 2 - (y - self.cy) * self.scale

    def to_world(self, u: float, v: float, w: int, h: int) -> Tuple[float, float]:
        return self.cx + (u - w / 2) / self.scale, self.cy - (v - h / 2) / self.scale

    def zoom_at(self, u: float, v: float, factor: float, w: int, h: int) -> None:
        """Zoom keeping the world point under pixel (u, v) fixed."""
        wx, wy = self.to_world(u, v, w, h)
        self.scale = min(2000.0, max(2.0, self.scale * factor))
        self.cx = wx - (u - w / 2) / self.scale
        self.cy = wy + (v - h / 2) / self.scale

    def pan_px(self, du: float, dv: float) -> None:
        self.cx -= du / self.scale
        self.cy += dv / self.scale

    def fit(self, m: MapData, w: int, h: int) -> None:
        """Centre on the map's bounding box and zoom so all of it is visible."""
        ox, oy, oth = m.origin
        c, s = math.cos(oth), math.sin(oth)
        corners = [(ox + c * i * m.resolution * m.width - s * j * m.resolution * m.height,
                    oy + s * i * m.resolution * m.width + c * j * m.resolution * m.height)
                   for i in (0, 1) for j in (0, 1)]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        self.cx, self.cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
        self.scale = max(2.0, 0.95 * min(w / max(span_x, 1e-6), h / max(span_y, 1e-6)))


def render_map(m: MapData, view: View, w: int, h: int, bg=(13, 17, 23)) -> Image.Image:
    """Resample just the visible part of the map onto a w x h canvas (nearest neighbour = crisp cells).

    One PIL affine transform, so cost depends on the canvas size, not the map size or zoom.
    """
    ox, oy, oth = m.origin
    c, s = math.cos(oth), math.sin(oth)
    sc, res = view.scale, m.resolution
    a_ = (view.cx - ox) - w / 2 / sc
    b_ = (view.cy - oy) + h / 2 / sc
    k1, k2 = c * a_ + s * b_, -s * a_ + c * b_
    coeffs = (c / (sc * res), -s / (sc * res), k1 / res,
              s / (sc * res), c / (sc * res), m.height - k2 / res)
    return m.rgb.transform((w, h), Image.AFFINE, coeffs, resample=Image.NEAREST, fillcolor=bg)


# --------------------------------------------------------------------------- scan
@dataclass
class ScanData:
    frame_id: str
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray        # NaN where invalid

    def points(self) -> np.ndarray:
        """Valid returns as an (N, 2) array in the sensor frame. Never invents points."""
        r = self.ranges
        ok = np.isfinite(r) & (r >= max(self.range_min, 0.0)) & (r <= self.range_max)
        ang = self.angle_min + self.angle_increment * np.arange(r.size)[ok]
        rr = r[ok]
        return np.stack([rr * np.cos(ang), rr * np.sin(ang)], axis=1) if rr.size else np.zeros((0, 2))


def scan_from_message(msg: dict) -> ScanData:
    # rosbridge sends inf/NaN as null (or the strings "Infinity"/"NaN") depending on version.
    def num(v):
        try:
            return float("nan") if v is None else float(v)
        except (TypeError, ValueError):
            return float("nan")
    return ScanData(clean_frame(msg["header"]["frame_id"]), float(msg["angle_min"]),
                    float(msg["angle_increment"]), float(msg["range_min"]), float(msg["range_max"]),
                    np.array([num(v) for v in msg["ranges"]], dtype=float))


def transform_points(pts: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Express sensor-frame points in the frame where the sensor sits at `pose`."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    if pts.size == 0:
        return pts
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def fit_score(m: MapData, pts_map: np.ndarray, tol_m: float = 0.05, min_points: int = 50) -> Optional[float]:
    """Fraction of scan points (already in the map frame) that land within `tol_m` of an occupied map cell.

    This is an independent check of a reported pose: at the true pose most of a scan lies on mapped walls
    (0.86-0.87 on the living-room map), at a pose that is 0.35 m or 30 degrees off it drops to 0.36-0.57.
    Returns None when it cannot be computed (no occupancy data, too few points). Points outside the map count as misses.
    """
    d = m.dist_to_wall()
    if d is None or pts_map.shape[0] < min_points:
        return None
    ox, oy, oth = m.origin
    c, s = math.cos(oth), math.sin(oth)
    dx, dy = pts_map[:, 0] - ox, pts_map[:, 1] - oy
    ci = np.floor((c * dx + s * dy) / m.resolution).astype(int)
    ri = np.floor((-s * dx + c * dy) / m.resolution).astype(int)
    inside = (ci >= 0) & (ci < m.width) & (ri >= 0) & (ri < m.height)
    tol_cells = max(1, int(round(tol_m / m.resolution)))
    hit = np.zeros(pts_map.shape[0], dtype=bool)
    hit[inside] = d[ri[inside], ci[inside]] <= tol_cells
    return float(hit.mean())
