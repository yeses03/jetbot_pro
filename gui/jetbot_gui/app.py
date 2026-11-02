"""Tk front end. Reads snapshots from RobotLink; never touches the network itself, so it cannot freeze on it."""
from __future__ import annotations

import math
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

from PIL import ImageTk

from . import config as cfgmod
from .geometry import View, heading_towards, render_map, transform_points
from .goals import RouteRunner, blockers
from .link import CONNECTED, RobotLink
from .status import BAD, IDLE, OK, WARN, assess

BG, PANEL, FG, GREY = "#1e2127", "#2a2f38", "#e6e6e6", "#7d8590"
LEVEL_COLOR = {OK: "#3fb950", WARN: "#e3a008", BAD: "#f85149", IDLE: GREY}
CANVAS_BG = "#0d1117"
REFRESH_MS = 100


class App:
    def __init__(self, root: tk.Tk, cfg: cfgmod.Config):
        self.root, self.cfg = root, cfg
        self.link = RobotLink(cfg)
        self.view = View()
        self.view_fitted_for = None            # map_seq key of the first map we auto-fitted to
        self._photo = None
        self._map_key = None
        self._map_img = None
        self._drag = None
        self._cursor_world = None
        self.show_scan = tk.BooleanVar(value=True)
        self.host_var = tk.StringVar(value=cfg.host)
        self.port_var = tk.StringVar(value=str(cfg.port))
        self.chips = {}
        self.armed = tk.BooleanVar(value=False)       # goals can only be sent while armed; resets after each send
        self.goal_mode = tk.BooleanVar(value=False)   # map clicks set a goal instead of panning
        self.pending = None                           # (x, y, yaw) chosen on the map, not yet sent
        self._goal_drag = None                        # world point where a heading drag started
        self.route = []                               # goals queued for Run route
        self.runner = RouteRunner()
        self._gate = []                               # why a goal is blocked right now (empty = allowed)

        root.title("JetBot Pro dashboard (goals only, no manual driving)")
        root.configure(bg=BG)
        root.minsize(1100, 680)
        self._style()
        self._build()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        if cfg.host:
            self.connect()
        self.root.after(REFRESH_MS, self.tick)

    # ------------------------------------------------------------------ layout
    def _style(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure(".", background=BG, foreground=FG, fieldbackground=PANEL)
        s.configure("TLabelframe", background=BG, foreground=FG)
        s.configure("TLabelframe.Label", background=BG, foreground=FG, font=("TkDefaultFont", 10, "bold"))

    def _build(self):
        r = self.root
        r.columnconfigure(0, weight=1)
        r.rowconfigure(2, weight=1)

        bar = tk.Frame(r, bg=PANEL)
        bar.grid(row=0, column=0, sticky="ew")
        ttk.Label(bar, text="Robot IP / host", background=PANEL).pack(side="left", padx=(8, 2), pady=6)
        ttk.Entry(bar, textvariable=self.host_var, width=18).pack(side="left")
        ttk.Label(bar, text="port", background=PANEL).pack(side="left", padx=(8, 2))
        ttk.Entry(bar, textvariable=self.port_var, width=6).pack(side="left")
        ttk.Button(bar, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(bar, text="Disconnect", command=self.disconnect).pack(side="left")
        ttk.Button(bar, text="Save as default", command=self.save_defaults).pack(side="left", padx=6)
        tk.Label(bar, text="Sends GOALS to move_base only. Cancel stops it; the power switch is the real stop.", bg=PANEL, fg=GREY).pack(side="right", padx=10)

        chips = tk.Frame(r, bg=BG)
        chips.grid(row=1, column=0, sticky="ew", padx=6, pady=(6, 0))
        for key, title in (("conn", "Link"), ("map", "Map"), ("scan", "LiDAR"), ("pose", "Pose"),
                           ("odom", "Odometry"), ("fit", "Pose vs scan"), ("localizer", "Localiser"),
                           ("nav", "Navigation")):
            f = tk.Frame(chips, bg=PANEL)
            f.pack(side="left", padx=3, fill="x", expand=True)
            tk.Label(f, text=title, bg=PANEL, fg=GREY, font=("TkDefaultFont", 8)).pack(anchor="w", padx=6)
            lab = tk.Label(f, text="…", bg=PANEL, fg=GREY, anchor="w", justify="left", wraplength=150,
                           font=("TkDefaultFont", 9, "bold"))
            lab.pack(anchor="w", padx=6, pady=(0, 4), fill="x")
            self.chips[key] = lab

        body = tk.Frame(r, bg=BG)
        body.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        mapbox = ttk.LabelFrame(body, text="Map")
        mapbox.grid(row=0, column=0, sticky="nsew")
        tools = ttk.Frame(mapbox)
        tools.pack(fill="x")
        ttk.Button(tools, text="Fit", width=5, command=self.fit_view).pack(side="left", padx=2, pady=2)
        ttk.Button(tools, text="Centre on robot", command=self.centre_robot).pack(side="left", padx=2)
        ttk.Checkbutton(tools, text="Show LiDAR on map", variable=self.show_scan).pack(side="left", padx=8)
        self.cursor_lbl = ttk.Label(tools, text="wheel = zoom, drag = pan")
        self.cursor_lbl.pack(side="right", padx=6)
        self.map_cv = tk.Canvas(mapbox, bg=CANVAS_BG, highlightthickness=0, width=640, height=480)
        self.map_cv.pack(fill="both", expand=True, padx=4, pady=4)
        cv = self.map_cv
        cv.bind("<ButtonPress-1>", self._press)
        cv.bind("<B1-Motion>", self._motion)
        cv.bind("<ButtonRelease-1>", self._release)
        cv.bind("<Motion>", self._hover)
        cv.bind("<MouseWheel>", lambda e: self._zoom(e, 1.15 if e.delta > 0 else 1 / 1.15))
        cv.bind("<Button-4>", lambda e: self._zoom(e, 1.15))     # X11 wheel
        cv.bind("<Button-5>", lambda e: self._zoom(e, 1 / 1.15))

        side = tk.Frame(body, bg=BG)
        side.grid(row=0, column=1, sticky="ns", padx=(6, 0))

        pb = ttk.LabelFrame(side, text="Robot pose")
        pb.pack(fill="x")
        self.pose_txt = tk.Label(pb, text="", bg=BG, fg=FG, justify="left", anchor="w", font=("Menlo", 10),
                                 width=36)
        self.pose_txt.pack(anchor="w", padx=6, pady=4)

        gb = ttk.LabelFrame(side, text="Goals (moves the robot)")
        gb.pack(fill="x", pady=(6, 0))
        tk.Label(gb, text="Cancel stops the robot, but the POWER SWITCH is the real stop.", bg=BG, fg=GREY,
                 font=("TkDefaultFont", 8), wraplength=270, justify="left").pack(anchor="w", padx=6)
        ttk.Checkbutton(gb, text="Arm goals (allow the robot to move)", variable=self.armed).pack(anchor="w", padx=6)
        ttk.Checkbutton(gb, text="Click map to set a goal (drag = heading)", variable=self.goal_mode).pack(anchor="w", padx=6)
        self.goal_txt = tk.Label(gb, text="no goal chosen", bg=BG, fg=FG, justify="left", anchor="w",
                                 font=("Menlo", 9), wraplength=270)
        self.goal_txt.pack(anchor="w", padx=6, pady=(2, 0))
        row = ttk.Frame(gb)
        row.pack(fill="x", padx=6, pady=2)
        self.send_btn = ttk.Button(row, text="Send goal", command=self.send_pending)
        self.send_btn.pack(side="left")
        ttk.Button(row, text="Add to route", command=self.add_to_route).pack(side="left", padx=4)
        row2 = ttk.Frame(gb)
        row2.pack(fill="x", padx=6, pady=2)
        self.run_btn = ttk.Button(row2, text="Run route", command=self.run_route)
        self.run_btn.pack(side="left")
        ttk.Button(row2, text="Clear route", command=self.clear_route).pack(side="left", padx=4)
        self.cancel_btn = tk.Button(gb, text="CANCEL GOAL (stop)", command=self.cancel_all, bg="#8b1e1e", fg="white")
        self.cancel_btn.pack(fill="x", padx=6, pady=(2, 2))
        self.route_txt = tk.Label(gb, text="", bg=BG, fg=FG, justify="left", anchor="w", font=("Menlo", 9), wraplength=270)
        self.route_txt.pack(anchor="w", padx=6)
        self.block_lbl = tk.Label(gb, text="", bg=BG, fg=LEVEL_COLOR[WARN], justify="left", anchor="w",
                                  font=("TkDefaultFont", 8), wraplength=270)
        self.block_lbl.pack(anchor="w", padx=6, pady=(0, 4))

        lb = ttk.LabelFrame(side, text="LiDAR (robot frame, forward = up)")
        lb.pack(fill="x", pady=(6, 0))
        self.lidar_info = ttk.Label(lb, text="")
        self.lidar_info.pack(anchor="w", padx=6)
        self.lidar_cv = tk.Canvas(lb, bg=CANVAS_BG, highlightthickness=0, width=270, height=270)
        self.lidar_cv.pack(padx=4, pady=4)

        db = ttk.LabelFrame(side, text="Topics")
        db.pack(fill="both", expand=True, pady=(6, 0))
        self.diag_txt = tk.Label(db, text="", bg=BG, fg=FG, justify="left", anchor="nw", font=("Menlo", 9))
        self.diag_txt.pack(anchor="nw", padx=6, pady=4, fill="both", expand=True)

        self.warn_lbl = tk.Label(r, text="", bg=BG, fg=LEVEL_COLOR[WARN], justify="left", anchor="w", wraplength=1050)
        self.warn_lbl.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 6))

    # ------------------------------------------------------------------ actions
    def connect(self):
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get())
        except ValueError:
            self.warn_lbl.configure(text="Port must be a number.")
            return
        if not host:
            self.warn_lbl.configure(text="Enter the robot's IP or hostname.")
            return
        self.cfg.host, self.cfg.port = host, port
        self.view_fitted_for = None
        self.link.connect(host, port)

    def disconnect(self):
        self.link.disconnect()

    def save_defaults(self):
        self.cfg.host, self.cfg.port = self.host_var.get().strip(), int(self.port_var.get() or 9090)
        cfgmod.save_local(self.cfg)
        self.cursor_lbl.configure(text=f"saved to {cfgmod.LOCAL_FILE.name}")

    def on_close(self):
        self.link.disconnect()
        self.root.destroy()

    def _size(self):
        return max(self.map_cv.winfo_width(), 50), max(self.map_cv.winfo_height(), 50)

    def fit_view(self):
        snap = self.link.snapshot()
        if snap.map:
            w, h = self._size()
            self.view.fit(snap.map, w, h)

    def centre_robot(self):
        a = assess(self.link.snapshot(), self.cfg)
        snap = self.link.snapshot()
        if a.pose_live_in_map and snap.pose_map:
            self.view.cx, self.view.cy = snap.pose_map[0], snap.pose_map[1]

    # map mouse handling
    def _press(self, e):
        if self.goal_mode.get():
            w, h = self._size()
            x, y = self.view.to_world(e.x, e.y, w, h)
            self._goal_drag = (x, y)
            self.pending = (x, y, self._default_yaw(x, y))
            return
        self._drag = (e.x, e.y)

    def _default_yaw(self, x, y):
        pose = self.link.snapshot().pose_map
        return heading_towards(pose[:2], (x, y)) if pose else 0.0

    def _release(self, e):
        self._drag = None
        self._goal_drag = None

    def _motion(self, e):
        if self._goal_drag:
            w, h = self._size()
            x, y = self.view.to_world(e.x, e.y, w, h)
            gx, gy = self._goal_drag
            if math.hypot(x - gx, y - gy) > 0.10:             # a real drag sets the heading
                self.pending = (gx, gy, heading_towards((gx, gy), (x, y)))
            self._hover(e)
            return
        if self._drag:
            self.view.pan_px(e.x - self._drag[0], e.y - self._drag[1])
            self._drag = (e.x, e.y)
        self._hover(e)

    def _zoom(self, e, factor):
        w, h = self._size()
        self.view.zoom_at(e.x, e.y, factor, w, h)

    def _hover(self, e):
        w, h = self._size()
        self._cursor_world = self.view.to_world(e.x, e.y, w, h)

    # ------------------------------------------------------------------ periodic refresh
    def tick(self):
        try:
            snap = self.link.snapshot()
            a = assess(snap, self.cfg)
            self._update_chips(a)
            self._update_goals(snap, a)
            self._draw_map(snap, a)
            self._draw_lidar(snap, a)
            self._update_text(snap, a)
        except Exception as exc:            # a drawing bug must not kill the refresh loop
            self.warn_lbl.configure(text=f"UI error: {type(exc).__name__}: {exc}")
        finally:
            self.root.after(REFRESH_MS, self.tick)

    def _update_chips(self, a):
        for key, lab in self.chips.items():
            item = getattr(a, key)
            lab.configure(text=item.text, fg=LEVEL_COLOR[item.level])
        self.warn_lbl.configure(text="\n".join("⚠ " + w for w in a.warnings))

    def _draw_map(self, snap, a):
        cv = self.map_cv
        w, h = self._size()
        cv.delete("overlay")
        m = snap.map
        if m is None:
            cv.delete("map")
            self._map_key = None
            cv.create_text(w / 2, h / 2, fill=GREY, tags="overlay", justify="center",
                           text=f"No map on {self.cfg.map_topic}\n"
                                f"({'waiting for the robot…' if snap.conn_state == CONNECTED else 'not connected'})")
            return
        if self.view_fitted_for is None:
            self.view.fit(m, w, h)
            self.view_fitted_for = True
        v = self.view
        key = (snap.map_seq, round(v.cx, 4), round(v.cy, 4), round(v.scale, 3), w, h)
        if key != self._map_key:
            self._photo = ImageTk.PhotoImage(render_map(m, v, w, h))
            self._map_key = key
            cv.delete("map")
            cv.create_image(0, 0, image=self._photo, anchor="nw", tags="map")
            cv.tag_lower("map")

        # LiDAR on the map: only when the scan AND the whole TF chain are fresh
        if self.show_scan.get() and a.scan_on_map:
            pts = transform_points(snap.scan.points(), snap.scan_in_map)
            for x, y in pts:
                u, vv = v.to_px(x, y, w, h)
                if 0 <= u < w and 0 <= vv < h:
                    cv.create_rectangle(u - 1, vv - 1, u + 1, vv + 1, fill="#f85149", outline="", tags="overlay")
        # robot only when localised in the map frame; odometry-only is NOT drawn as a map position
        if a.pose_live_in_map and snap.pose_map:
            x, y, th = snap.pose_map
            u, vv = v.to_px(x, y, w, h)
            L = 12
            pts = [(u + L * math.cos(th), vv - L * math.sin(th)),
                   (u + 0.7 * L * math.cos(th + 2.5), vv - 0.7 * L * math.sin(th + 2.5)),
                   (u + 0.7 * L * math.cos(th - 2.5), vv - 0.7 * L * math.sin(th - 2.5))]
            cv.create_polygon(*[c for p in pts for c in p], fill="#58a6ff", outline="white", tags="overlay")
        else:
            cv.create_text(8, 8, anchor="nw", fill=LEVEL_COLOR[WARN], tags="overlay",
                           text="Robot not drawn: no live map-frame pose")
        self._draw_goals(cv, v, w, h)
        if self._cursor_world:
            self.cursor_lbl.configure(text=f"x {self._cursor_world[0]:+.2f}  y {self._cursor_world[1]:+.2f} m   "
                                           "(wheel = zoom, drag = pan)")

    # ------------------------------------------------------------------ goals
    def _update_goals(self, snap, a):
        self._gate = blockers(a, snap, self.cfg, self.armed.get())
        was_running = self.runner.running
        goal = self.runner.step(snap.goal, self._gate, time.monotonic())
        if goal is not None:
            self.link.send_goal(goal)
        if was_running and not self.runner.running:
            self.armed.set(False)                              # a finished or failed route disarms
        lines = []
        if self.pending:
            x, y, yaw = self.pending
            pose = snap.pose_map
            d = f", {math.hypot(x - pose[0], y - pose[1]):.1f} m away" if pose else ""
            lines.append(f"goal  x {x:+.2f}  y {y:+.2f}  yaw {math.degrees(yaw):+.0f} deg{d}")
        else:
            lines.append("no goal chosen" + ("  (click the map)" if self.goal_mode.get() else ""))
        if snap.goal is not None and snap.goal_age is not None:
            lines.append(f"move_base: {snap.goal[2] or 'status ' + str(snap.goal[1])} ({snap.goal_age:.0f}s ago)")
        self.goal_txt.configure(text="\n".join(lines))
        route = [f"route: {len(self.route)} legs"] if self.route else []
        if self.runner.message and self.runner.state != RouteRunner.IDLE:
            route.append(self.runner.message)
        self.route_txt.configure(text="\n".join(route), fg=LEVEL_COLOR[BAD if self.runner.state == RouteRunner.FAILED else OK])
        wants = self.pending is not None or bool(self.route)
        self.block_lbl.configure(text=("Blocked: " + "; ".join(self._gate)) if (wants and self._gate) else "")
        connected = snap.conn_state == CONNECTED
        self.send_btn.state(["!disabled"] if (self.pending and not self._gate and not self.runner.running) else ["disabled"])
        self.run_btn.state(["!disabled"] if (self.route and not self._gate and not self.runner.running) else ["disabled"])
        self.cancel_btn.configure(state="normal" if connected else "disabled")

    def send_pending(self):
        if not self.pending or self._gate:
            return
        x, y, yaw = self.pending
        pose = self.link.snapshot().pose_map
        far = f" ({math.hypot(x - pose[0], y - pose[1]):.1f} m away)" if pose else ""
        if not messagebox.askyesno("Send goal", f"Send the robot to x {x:+.2f}, y {y:+.2f}, facing {math.degrees(yaw):+.0f} deg{far}?\n\n"
                                   "THE ROBOT WILL MOVE. Keep clear, and keep a hand near the power switch."):
            return
        if self.link.send_goal(self.pending):
            self.armed.set(False)                              # one goal per arming

    def add_to_route(self):
        if self.pending:
            self.route.append(self.pending)

    def clear_route(self):
        if not self.runner.running:
            self.route = []
            self.runner = RouteRunner()

    def run_route(self):
        if not self.route or self._gate or self.runner.running:
            return
        pose = self.link.snapshot().pose_map
        pts = ([pose[:2]] if pose else []) + [g[:2] for g in self.route]
        length = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(pts, pts[1:]))
        if not messagebox.askyesno("Run route", f"Run {len(self.route)} legs, about {length:.1f} m in a straight-line sum?\n\n"
                                   "THE ROBOT WILL MOVE, one leg after another, and stops at the first failure.\n"
                                   "Keep clear, hand near the power switch, and press CANCEL GOAL to stop."):
            return
        self.runner.start(self.route)

    def cancel_all(self):
        self.runner.stop("cancelled by you")
        self.link.cancel_goals()
        self.armed.set(False)

    def _draw_goals(self, cv, v, w, h):
        legs = self.route
        pts = []
        for i, (x, y, yaw) in enumerate(legs):
            u, vv = v.to_px(x, y, w, h)
            pts.append((u, vv))
            cv.create_oval(u - 9, vv - 9, u + 9, vv + 9, outline="#3fb950", width=2, tags="overlay")
            cv.create_text(u, vv, text=str(i + 1), fill="#3fb950", tags="overlay")
            cv.create_line(u, vv, u + 22 * math.cos(yaw), vv - 22 * math.sin(yaw), fill="#3fb950", width=2, arrow="last", tags="overlay")
        for p, q in zip(pts, pts[1:]):
            cv.create_line(p[0], p[1], q[0], q[1], fill="#3fb950", dash=(4, 3), tags="overlay")
        if self.pending:
            x, y, yaw = self.pending
            u, vv = v.to_px(x, y, w, h)
            cv.create_oval(u - 8, vv - 8, u + 8, vv + 8, outline="#ff9f1a", width=3, tags="overlay")
            cv.create_line(u, vv, u + 26 * math.cos(yaw), vv - 26 * math.sin(yaw), fill="#ff9f1a", width=3, arrow="last", tags="overlay")

    def _draw_lidar(self, snap, a):
        cv = self.lidar_cv
        cv.delete("all")
        W = H = 270
        cx = cy = W / 2
        rng = self.cfg.lidar_range_m
        scale = (W / 2 - 14) / rng
        for m in range(1, int(rng) + 1):
            cv.create_oval(cx - m * scale, cy - m * scale, cx + m * scale, cy + m * scale, outline="#30363d")
            cv.create_text(cx + 3, cy - m * scale, text=f"{m} m", fill="#6e7681", anchor="sw", font=("TkDefaultFont", 7))
        cv.create_polygon(cx, cy - 9, cx - 6, cy + 6, cx + 6, cy + 6, fill="#58a6ff")
        if not a.scan_live:
            cv.create_text(cx, cy + 50, fill=LEVEL_COLOR[BAD], text="NO LIVE SCAN\n" + a.scan.text, justify="center")
            self.lidar_info.configure(text="")
            return
        pts = snap.scan.points()
        frame_note = f"frame '{snap.scan.frame_id}'"
        if snap.scan_in_base is not None:
            pts = transform_points(pts, snap.scan_in_base)
            frame_note = f"in {self.cfg.base_frame}"
        else:
            frame_note = f"SENSOR frame '{snap.scan.frame_id}' (no TF to {self.cfg.base_frame})"
        for x, y in pts:                       # x forward -> up, y left -> left
            if math.hypot(x, y) > rng:
                continue
            u, v = cx - y * scale, cy - x * scale
            d = math.hypot(x, y)
            col = "#f85149" if d < 0.4 else "#e3a008" if d < 1.0 else "#3fb950"
            cv.create_rectangle(u - 1, v - 1, u + 1, v + 1, fill=col, outline=col)
        near = min((math.hypot(x, y) for x, y in pts), default=None)
        self.lidar_info.configure(text=f"{len(pts)}/{len(snap.scan.ranges)} valid beams, {frame_note}"
                                       + (f", nearest {near:.2f} m" if near is not None else ""))

    def _update_text(self, snap, a):
        lines = []
        if a.pose_live_in_map and snap.pose_map:
            x, y, th = snap.pose_map
            lines += ["MAP POSE (live, TF)", f"  x   {x:+8.3f} m", f"  y   {y:+8.3f} m",
                      f"  yaw {math.degrees(th):+8.1f} deg"]
        else:
            lines += ["MAP POSE: unavailable"]
            if a.pose.level == WARN and snap.pose_odom:
                x, y, th = snap.pose_odom
                lines += ["", "ODOMETRY ONLY (drifts, not on map)", f"  x   {x:+8.3f} m",
                          f"  y   {y:+8.3f} m", f"  yaw {math.degrees(th):+8.1f} deg"]
        if a.fit_value is not None:
            lines += ["", f"SCAN vs MAP: {a.fit_value * 100:.0f}% of points on walls"]
        if snap.amcl_pose and snap.amcl_age is not None:
            fresh = snap.amcl_age <= self.cfg.pose_stale_s
            sx, sa = snap.amcl_sigma or (float("nan"), float("nan"))
            lines += ["", f"/amcl_pose {'live' if fresh else 'STALE %.0fs' % snap.amcl_age}",
                      f"  sigma xy {sx:.2f} m  yaw {math.degrees(sa):.0f} deg"]
        self.pose_txt.configure(text="\n".join(lines))

        rows = []
        for name in (self.cfg.map_topic, self.cfg.scan_topic, self.cfg.odom_topic, self.cfg.amcl_pose_topic,
                     self.cfg.tf_topic, self.cfg.tf_static_topic):
            st = snap.stats.get(name)
            if st is None or st.last_rx is None:
                rows.append(f"{name:<14} no data" + (f"  [{st.last_error}]" if st and st.last_error else ""))
            else:
                rows.append(f"{name:<14} {st.count:>6} msgs, {snap.now - st.last_rx:5.1f}s ago")
        if snap.tf_frames:
            rows.append("")
            rows.append("TF frames: " + ", ".join(snap.tf_frames))
        self.diag_txt.configure(text="\n".join(rows), wraplength=290)


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="JetBot Pro read-only dashboard (ROS 1 via rosbridge)")
    p.add_argument("--host", help="robot IP / hostname running rosbridge")
    p.add_argument("--port", type=int)
    for name in ("map_topic", "scan_topic", "odom_topic", "amcl_pose_topic", "map_frame", "odom_frame", "base_frame"):
        p.add_argument("--" + name.replace("_", "-"), dest=name)
    args = p.parse_args(argv)
    cfg = cfgmod.load(vars(args))
    root = tk.Tk()
    App(root, cfg)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
