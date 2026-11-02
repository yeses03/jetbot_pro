"""Tk front end. Reads snapshots from RobotLink; never touches the network itself, so it cannot freeze on it.

Layout: header (connection, MOTION master switch, STOP) / status cards / map + tabbed side panel / status bar.
Interaction: click the map = choose a destination, then GO. Shift+drag sets the heading, drag pans, wheel zooms.
"""
from __future__ import annotations

import math
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from PIL import ImageTk

from . import config as cfgmod
from . import theme as T
from .geometry import View, fit_score, heading_towards, pose_search, render_map, transform_points
from .goals import ACTIVE, PENDING, RouteRunner, blockers
from .link import CONNECTED, RobotLink
from .places import Places, map_signature
from .status import BAD, IDLE, OK, WARN, assess
from .teleop import (DEFAULT_ANGULAR, DEFAULT_LINEAR, MAX_ANGULAR, MAX_LINEAR, TeleopState, guard)

LEVEL_COLOR = {OK: T.OK, WARN: T.WARN, BAD: T.BAD, IDLE: T.MUTED}
REFRESH_MS = 100
MOTION_IDLE_S = 300.0            # the MOTION switch turns itself off after this long without a goal or teleop command
CHIPS = (("conn", "LINK"), ("map", "MAP"), ("scan", "LIDAR"), ("pose", "POSE"), ("odom", "ODOMETRY"),
         ("fit", "POSE vs SCAN"), ("localizer", "LOCALISER"), ("nav", "NAVIGATION"))


class App:
    def __init__(self, root: tk.Tk, cfg: cfgmod.Config):
        self.root, self.cfg = root, cfg
        self.link = RobotLink(cfg)
        self.view = View()
        self.view_fitted_for = None            # becomes True once we auto-fitted to the first map
        self._photo = None
        self._map_key = None
        self._click = None                     # state of the current mouse press on the map
        self._drag = None
        self._cursor_world = None
        self.show_scan = tk.BooleanVar(value=True)
        self.host_var = tk.StringVar(value=cfg.host)
        self.port_var = tk.StringVar(value=str(cfg.port))
        self.chips, self.chip_dots = {}, {}
        # motion master switch: nothing can move the robot while it is off; it also times out by itself
        self.armed = tk.BooleanVar(value=False)
        self._armed_since = 0.0
        self.pending = None                    # (x, y, yaw) chosen on the map, not yet sent
        self._sent = None                      # the goal most recently sent (drawn while it is active)
        self.route = []                        # goals queued for Run route
        self.runner = RouteRunner()
        self._gate = []                        # why a goal is blocked right now (empty = allowed)
        self.places = Places()
        self._places_ok = True
        self.place_var = tk.StringVar()
        self.name_var = tk.StringVar()
        try:
            self.places.load()
        except Exception as exc:
            self.places = Places()
            print(f"could not read {self.places.path.name}: {exc}", file=sys.stderr)
        self.reloc_global = tk.BooleanVar(value=False)
        self._reloc = None
        # teleop
        self.tele = TeleopState()
        self.teleop_on = False
        self._tele_flush = 0                   # zero-velocity messages still to send after teleop stops
        self._tele_note = ""

        root.title("JetBot Pro  -  Navigation Console")
        root.minsize(1280, 760)
        T.apply_theme(root)
        self._build()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind_all("<KeyPress>", self._key_down, add="+")
        root.bind_all("<KeyRelease>", self._key_up, add="+")
        root.bind("<FocusOut>", self._focus_out, add="+")
        if cfg.host:
            self.connect()
        self.root.after(REFRESH_MS, self.tick)

    # ================================================================== layout
    def _build(self):
        r = self.root
        r.columnconfigure(0, weight=1)
        r.rowconfigure(2, weight=1)
        self._build_header(r)
        self._build_chips(r)
        body = tk.Frame(r, bg=T.BG)
        body.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 8))
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)
        self._build_map(body)
        self._build_side(body)
        self._build_status_bar(r)

    def _build_header(self, r):
        bar = tk.Frame(r, bg=T.PANEL, highlightbackground=T.BORDER, highlightthickness=1)
        bar.grid(row=0, column=0, sticky="ew")
        left = tk.Frame(bar, bg=T.PANEL)
        left.pack(side="left", padx=16, pady=10)
        tk.Label(left, text="JetBot Pro", bg=T.PANEL, fg=T.FG, font=T.F_TITLE).pack(side="left")
        tk.Label(left, text="  Navigation Console", bg=T.PANEL, fg=T.MUTED, font=T.F_BASE).pack(side="left", pady=(3, 0))

        conn = tk.Frame(bar, bg=T.PANEL)
        conn.pack(side="left", padx=18)
        self.conn_dot = T.Dot(conn, 10, T.PANEL)
        self.conn_dot.pack(side="left", padx=(0, 8))
        tk.Label(conn, text="Robot", bg=T.PANEL, fg=T.MUTED, font=T.F_SMALL).pack(side="left", padx=(0, 6))
        ttk.Entry(conn, textvariable=self.host_var, width=16).pack(side="left")
        tk.Label(conn, text=":", bg=T.PANEL, fg=T.MUTED).pack(side="left", padx=2)
        ttk.Entry(conn, textvariable=self.port_var, width=6).pack(side="left")
        T.ActionButton(conn, "Connect", self.connect, "primary", pady=5).pack(side="left", padx=(10, 4))
        T.ActionButton(conn, "Disconnect", self.disconnect, "secondary", pady=5).pack(side="left", padx=2)
        T.ActionButton(conn, "Save as default", self.save_defaults, "ghost", font=T.F_SMALL, pady=5).pack(side="left", padx=2)

        right = tk.Frame(bar, bg=T.PANEL)
        right.pack(side="right", padx=16)
        self.stop_btn = T.ActionButton(right, "STOP", self.cancel_all, "danger", font=T.F_BIG, padx=24, pady=8)
        self.stop_btn.pack(side="right", padx=(10, 0))
        self.motion_btn = T.ActionButton(right, "MOTION  OFF", self.toggle_motion, "secondary", font=T.F_BIG, padx=18, pady=8)
        self.motion_btn.pack(side="right")

    def _build_chips(self, r):
        row = tk.Frame(r, bg=T.BG)
        row.grid(row=1, column=0, sticky="ew", padx=12, pady=10)
        for i, (key, title) in enumerate(CHIPS):
            row.columnconfigure(i, weight=1, uniform="chips")
            f = tk.Frame(row, bg=T.CARD, highlightbackground=T.BORDER, highlightthickness=1)
            f.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0))
            top = tk.Frame(f, bg=T.CARD)
            top.pack(fill="x", padx=10, pady=(8, 0))
            dot = T.Dot(top, 8, T.CARD)
            dot.pack(side="left", padx=(0, 6))
            tk.Label(top, text=title, bg=T.CARD, fg=T.MUTED, font=T.F_TINY).pack(side="left")
            lab = tk.Label(f, text="…", bg=T.CARD, fg=T.MUTED, anchor="nw", justify="left", wraplength=116, font=T.F_SMALL)
            lab.pack(fill="x", padx=10, pady=(2, 8))
            self.chips[key], self.chip_dots[key] = lab, dot

    def _build_map(self, body):
        box = tk.Frame(body, bg=T.PANEL, highlightbackground=T.BORDER, highlightthickness=1)
        box.grid(row=0, column=0, sticky="nsew")
        box.rowconfigure(1, weight=1)
        box.columnconfigure(0, weight=1)
        tools = tk.Frame(box, bg=T.PANEL)
        tools.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        T.ActionButton(tools, "Fit map", self.fit_view, "secondary", font=T.F_SMALL, pady=4).pack(side="left")
        T.ActionButton(tools, "Centre on robot", self.centre_robot, "secondary", font=T.F_SMALL, pady=4).pack(side="left", padx=6)
        ttk.Checkbutton(tools, text="Show LiDAR", variable=self.show_scan, style="Panel.TCheckbutton").pack(side="left", padx=10)
        self.cursor_lbl = tk.Label(tools, text="", bg=T.PANEL, fg=T.MUTED, font=T.F_MONO_S)
        self.cursor_lbl.pack(side="right")
        self.map_cv = tk.Canvas(box, bg=T.CANVAS, highlightthickness=0, width=760, height=520, cursor="crosshair")
        self.map_cv.grid(row=1, column=0, sticky="nsew", padx=1, pady=(0, 1))
        cv = self.map_cv
        cv.bind("<ButtonPress-1>", self._press)
        cv.bind("<B1-Motion>", self._motion)
        cv.bind("<ButtonRelease-1>", self._release)
        cv.bind("<Double-Button-1>", self._double_click)
        cv.bind("<Motion>", self._hover)
        cv.bind("<MouseWheel>", lambda e: self._zoom(e, 1.15 if e.delta > 0 else 1 / 1.15))
        cv.bind("<Button-4>", lambda e: self._zoom(e, 1.15))
        cv.bind("<Button-5>", lambda e: self._zoom(e, 1 / 1.15))
        # floating destination bar (shown only while a destination is chosen)
        self.dest_bar = tk.Frame(cv, bg=T.CARD, highlightbackground=T.ACCENT, highlightthickness=2)
        top = tk.Frame(self.dest_bar, bg=T.CARD)
        top.pack(fill="x", padx=14, pady=(10, 4))
        self.goal_txt = tk.Label(top, text="", bg=T.CARD, fg=T.FG, font=T.F_BOLD, anchor="w", justify="left")
        self.goal_txt.pack(side="left")
        btns = tk.Frame(top, bg=T.CARD)
        btns.pack(side="right", padx=(24, 0))
        self.send_btn = T.ActionButton(btns, "GO", self.send_pending, "success", font=T.F_BIG, padx=30, pady=8)
        self.send_btn.pack(side="left")
        T.ActionButton(btns, "Cancel", self.clear_pending, "secondary", pady=8).pack(side="left", padx=(8, 0))
        T.ActionButton(btns, "+ Route", self.add_to_route, "secondary", pady=8).pack(side="left", padx=(8, 0))
        self.block_lbl = tk.Label(self.dest_bar, text="", bg=T.CARD, fg=T.WARN, font=T.F_SMALL, anchor="w", justify="left",
                                  wraplength=700)
        self.block_lbl.pack(fill="x", padx=14, pady=(0, 10))

    def _build_side(self, body):
        side = tk.Frame(body, bg=T.PANEL, highlightbackground=T.BORDER, highlightthickness=1, width=372)
        side.grid(row=0, column=1, sticky="ns", padx=(10, 0))
        side.grid_propagate(False)
        side.rowconfigure(0, weight=1)
        side.columnconfigure(0, weight=1)
        self.tabs = ttk.Notebook(side)
        self.tabs.grid(row=0, column=0, sticky="nsew")
        tabs = {}
        for name in ("Navigate", "Teleop", "Pose", "Diagnostics"):
            sf = T.ScrollFrame(self.tabs)                       # scrolls when the window is too short for the tab
            self.tabs.add(sf, text=name)
            tabs[name] = sf.body
        self._tab_navigate(tabs["Navigate"])
        self._tab_teleop(tabs["Teleop"])
        self._tab_pose(tabs["Pose"])
        self._tab_diag(tabs["Diagnostics"])

    @staticmethod
    def _hint(parent, text, **kw):
        return tk.Label(parent, text=text, bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, anchor="w", justify="left", wraplength=320, **kw)

    def _tab_navigate(self, tab):
        c = T.Card(tab, "How to send the robot somewhere")
        c.pack(fill="x", padx=12, pady=(12, 8))
        self._hint(c.body, "1  Switch MOTION on (top right).\n2  Click the map where you want the robot to go.\n"
                           "3  Press GO, or just double-click the map to choose and go in one step. STOP cancels it.\n\nShift + drag sets which way it ends up facing; plain drag pans the map, "
                           "the wheel zooms.").pack(fill="x")

        c = T.Card(tab, "Route")
        c.pack(fill="x", padx=12, pady=8)
        self.route_txt = tk.Label(c.body, text="empty: click the map, then + Route", bg=T.CARD, fg=T.MUTED, font=T.F_SMALL,
                                  anchor="w", justify="left", wraplength=320)
        self.route_txt.pack(fill="x")
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(8, 0))
        self.run_btn = T.ActionButton(row, "Run route", self.run_route, "primary", pady=6)
        self.run_btn.pack(side="left")
        T.ActionButton(row, "Clear", self.clear_route, "secondary", pady=6).pack(side="left", padx=6)

        c = T.Card(tab, "Saved places")
        c.pack(fill="x", padx=12, pady=8)
        self.place_box = ttk.Combobox(c.body, textvariable=self.place_var, state="readonly", values=self.places.names())
        self.place_box.pack(fill="x")
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(8, 0))
        T.ActionButton(row, "Go to place", self.go_to_place, "primary", pady=6).pack(side="left")
        T.ActionButton(row, "+ Route", self.add_place_to_route, "secondary", pady=6).pack(side="left", padx=6)
        T.ActionButton(row, "Delete", self.delete_place, "ghost", pady=6).pack(side="left")
        tk.Label(c.body, text="SAVE CURRENT AS", bg=T.CARD, fg=T.MUTED, font=T.F_TINY).pack(anchor="w", pady=(12, 2))
        ttk.Entry(c.body, textvariable=self.name_var).pack(fill="x")
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(8, 0))
        T.ActionButton(row, "Chosen destination", self.save_pending_as_place, "secondary", font=T.F_SMALL, pady=5).pack(side="left")
        T.ActionButton(row, "Robot pose", self.save_robot_as_place, "secondary", font=T.F_SMALL, pady=5).pack(side="left", padx=6)

    def _tab_teleop(self, tab):
        c = T.Card(tab, "Remote control")
        c.pack(fill="x", padx=12, pady=(12, 8))
        self.tele_state = tk.Label(c.body, text="TELEOP OFF", bg=T.CARD, fg=T.MUTED, font=T.F_BIG, anchor="w")
        self.tele_state.pack(fill="x")
        self.tele_btn = T.ActionButton(c.body, "Start teleop", self.toggle_teleop, "primary", font=T.F_BIG, pady=9)
        self.tele_btn.pack(fill="x", pady=(8, 6))
        self._hint(c.body, "Needs MOTION on. While teleop is on, goals are blocked; starting it cancels any active goal. "
                           "A key moves the robot for 0.7 s, so keep tapping or hold it; release or lose focus and it stops.").pack(fill="x")

        c = T.Card(tab, "Drive")
        c.pack(fill="x", padx=12, pady=8)
        pad = tk.Frame(c.body, bg=T.CARD)
        pad.pack()
        for col in range(3):
            pad.columnconfigure(col, minsize=84)
        self.pad_btns = []

        def pad_btn(text, row, col, unit):
            b = T.HoldButton(pad, text, on_down=lambda u=unit: self._pad_down(u), on_up=self._pad_up, variant="secondary",
                             font=T.F_BIG, padx=0, pady=14, width=5)
            b.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
            self.pad_btns.append(b)
            return b
        pad_btn("I\nforward", 0, 1, (1.0, 0.0))
        pad_btn("J\nleft", 1, 0, (0.0, 1.0))
        T.ActionButton(pad, "K\nstop", self.tele_stop, "danger", font=T.F_BIG, padx=0, pady=14, width=5).grid(row=1, column=1, padx=3, pady=3, sticky="nsew")
        pad_btn("L\nright", 1, 2, (0.0, -1.0))
        pad_btn(",\nback", 2, 1, (-1.0, 0.0))
        tk.Label(c.body, text="Keyboard:  I  forward    ,  back    J  left    L  right    K or Space  stop    Esc  teleop off",
                 bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, wraplength=320, justify="center").pack(pady=(10, 0))

        c = T.Card(tab, "Speed (hard-capped)")
        c.pack(fill="x", padx=12, pady=8)
        self.lin_var = tk.DoubleVar(value=DEFAULT_LINEAR)
        self.ang_var = tk.DoubleVar(value=DEFAULT_ANGULAR)
        self.lin_lbl = tk.Label(c.body, text="", bg=T.CARD, fg=T.FG, font=T.F_SMALL, anchor="w")
        self.lin_lbl.pack(fill="x")
        ttk.Scale(c.body, from_=0.05, to=MAX_LINEAR, variable=self.lin_var, command=lambda _v: self._speeds()).pack(fill="x", pady=(2, 8))
        self.ang_lbl = tk.Label(c.body, text="", bg=T.CARD, fg=T.FG, font=T.F_SMALL, anchor="w")
        self.ang_lbl.pack(fill="x")
        ttk.Scale(c.body, from_=0.1, to=MAX_ANGULAR, variable=self.ang_var, command=lambda _v: self._speeds()).pack(fill="x", pady=(2, 4))
        self.tele_out = tk.Label(c.body, text="", bg=T.CARD, fg=T.MUTED, font=T.F_MONO_S, anchor="w", justify="left", wraplength=320)
        self.tele_out.pack(fill="x", pady=(8, 0))
        self._speeds()

    def _tab_pose(self, tab):
        c = T.Card(tab, "Robot pose")
        c.pack(fill="x", padx=12, pady=(12, 8))
        self.pose_txt = tk.Label(c.body, text="", bg=T.CARD, fg=T.FG, justify="left", anchor="w", font=T.F_MONO, width=38)
        self.pose_txt.pack(fill="x")
        c = T.Card(tab, "Fix a wrong pose")
        c.pack(fill="x", padx=12, pady=8)
        self._hint(c.body, "After carrying the robot, or when POSE vs SCAN turns red: check where the LiDAR scan fits the saved map best.").pack(fill="x")
        ttk.Checkbutton(c.body, text="Search the whole room (robot was carried)", variable=self.reloc_global).pack(anchor="w", pady=(8, 4))
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(2, 0))
        self.reloc_btn = T.ActionButton(row, "Check pose from scan", self.check_pose, "secondary", pady=6)
        self.reloc_btn.pack(side="left")
        self.apply_btn = T.ActionButton(row, "Apply to AMCL", self.apply_pose, "primary", pady=6)
        self.apply_btn.pack(side="left", padx=6)
        self.apply_btn.state(["disabled"])
        self.reloc_txt = tk.Label(c.body, text="", bg=T.CARD, fg=T.FG, justify="left", anchor="w", font=T.F_MONO_S, wraplength=320)
        self.reloc_txt.pack(fill="x", pady=(10, 0))

    def _tab_diag(self, tab):
        c = T.Card(tab, "LiDAR (robot frame, forward = up)")
        c.pack(fill="x", padx=12, pady=(12, 8))
        self.lidar_info = tk.Label(c.body, text="", bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, anchor="w", wraplength=320, justify="left")
        self.lidar_info.pack(fill="x")
        self.lidar_cv = tk.Canvas(c.body, bg=T.CANVAS, highlightthickness=0, width=296, height=296)
        self.lidar_cv.pack(pady=(6, 0))
        c = T.Card(tab, "Topics")
        c.pack(fill="both", expand=True, padx=12, pady=8)
        self.diag_txt = tk.Label(c.body, text="", bg=T.CARD, fg=T.FG, justify="left", anchor="nw", font=T.F_MONO_S, wraplength=320)
        self.diag_txt.pack(fill="both", expand=True)

    def _build_status_bar(self, r):
        bar = tk.Frame(r, bg=T.PANEL, highlightbackground=T.BORDER, highlightthickness=1)
        bar.grid(row=3, column=0, sticky="ew")
        self.warn_lbl = tk.Label(bar, text="", bg=T.PANEL, fg=T.WARN, justify="left", anchor="w", wraplength=1000, font=T.F_SMALL)
        self.warn_lbl.pack(side="left", fill="x", expand=True, padx=14, pady=7)
        tk.Label(bar, text="Cancel and STOP stop the goal or teleop, but the POWER SWITCH is the real stop.", bg=T.PANEL, fg=T.MUTED,
                 font=T.F_TINY).pack(side="right", padx=14)

    # ================================================================== actions
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
        self._teleop_off("disconnected")
        self.link.disconnect()

    def save_defaults(self):
        self.cfg.host, self.cfg.port = self.host_var.get().strip(), int(self.port_var.get() or 9090)
        cfgmod.save_local(self.cfg)
        self.cursor_lbl.configure(text=f"saved to {cfgmod.LOCAL_FILE.name}")

    def on_close(self):
        self._teleop_off("window closed")
        for _ in range(3):
            self.link.send_velocity(0.0, 0.0)
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

    # ---- motion master switch
    def toggle_motion(self):
        self.set_motion(not self.armed.get())

    def set_motion(self, on: bool):
        self.armed.set(bool(on))
        self._armed_since = time.monotonic()
        if not on:
            self._teleop_off("motion switched off")

    def _touch_motion(self):
        self._armed_since = time.monotonic()

    # ---- map mouse handling: click = choose a destination, shift+drag = heading, drag = pan
    def _world(self, e):
        w, h = self._size()
        return self.view.to_world(e.x, e.y, w, h)

    def _default_yaw(self, x, y):
        pose = self.link.snapshot().pose_map
        return heading_towards(pose[:2], (x, y)) if pose else 0.0

    def _press(self, e):
        shift = bool(getattr(e, "state", 0) & 0x0001)
        wx, wy = self._world(e)
        self._click = {"px": (e.x, e.y), "world": (wx, wy), "moved": False, "shift": shift}
        self._drag = (e.x, e.y)
        self.map_cv.focus_set()
        if shift:                                                  # heading drag: the destination is where you pressed
            self.pending = (wx, wy, self._default_yaw(wx, wy))

    def _motion(self, e):
        c = self._click
        if c is None:
            self._hover(e)
            return
        if c["shift"]:
            x, y = self._world(e)
            gx, gy = c["world"]
            if math.hypot(x - gx, y - gy) > 0.10:
                self.pending = (gx, gy, heading_towards((gx, gy), (x, y)))
            c["moved"] = True
        else:
            if not c["moved"] and math.hypot(e.x - c["px"][0], e.y - c["px"][1]) > 4:
                c["moved"] = True                                  # it is a drag, not a click
            if c["moved"]:
                self.view.pan_px(e.x - self._drag[0], e.y - self._drag[1])
                self._drag = (e.x, e.y)
        self._hover(e)

    def _release(self, e):
        c, self._click, self._drag = self._click, None, None
        if c and not c["moved"] and not c["shift"]:                # a plain click chooses a destination
            wx, wy = c["world"]
            self.pending = (wx, wy, self._default_yaw(wx, wy))

    def _double_click(self, e):
        """Double-click = choose the destination and go, in one gesture. Same MOTION switch and safety gate as the GO button."""
        wx, wy = self._world(e)
        self.pending = (wx, wy, self._default_yaw(wx, wy))
        self._gate = self._gate_now()
        self.send_pending()

    def _gate_now(self):
        snap = self.link.snapshot()
        return blockers(assess(snap, self.cfg), snap, self.cfg, self.armed.get(), self.teleop_on)

    def _zoom(self, e, factor):
        w, h = self._size()
        self.view.zoom_at(e.x, e.y, factor, w, h)

    def _hover(self, e):
        self._cursor_world = self._world(e)

    # ================================================================== periodic refresh
    def tick(self):
        try:
            snap = self.link.snapshot()
            a = assess(snap, self.cfg)
            self._expire_motion()
            self._update_chips(snap, a)
            self._update_goals(snap, a)
            self._update_teleop(snap, a)
            self._update_reloc(snap)
            self._draw_map(snap, a)
            self._draw_lidar(snap, a)
            self._update_text(snap, a)
        except Exception as exc:            # a drawing bug must not kill the refresh loop
            self.warn_lbl.configure(text=f"UI error: {type(exc).__name__}: {exc}")
        finally:
            self.root.after(REFRESH_MS, self.tick)

    def _expire_motion(self):
        if self.armed.get() and time.monotonic() - self._armed_since > MOTION_IDLE_S:
            self.set_motion(False)
            self._tele_note = "MOTION switched itself off after 5 minutes without a command."

    def _update_chips(self, snap, a):
        for key, lab in self.chips.items():
            item = getattr(a, key)
            lab.configure(text=item.text, fg=T.FG if item.level == OK else LEVEL_COLOR[item.level])
            self.chip_dots[key].set(LEVEL_COLOR[item.level])
        self.conn_dot.set(LEVEL_COLOR[a.conn.level])
        on = self.armed.get()
        self.motion_btn.configure(text="MOTION  ON" if on else "MOTION  OFF")
        self.motion_btn.set_variant("success" if on else "secondary")
        tip = self._tele_note
        self.warn_lbl.configure(text="\n".join(["⚠ " + w for w in a.warnings] + ([tip] if tip else [])))

    # ================================================================== goals
    def _update_goals(self, snap, a):
        self._gate = blockers(a, snap, self.cfg, self.armed.get(), self.teleop_on)
        self._places_ok = not self.places.mismatch(map_signature(snap.map) if snap.map else None)
        was_running = self.runner.running
        goal = self.runner.step(snap.goal, self._gate, time.monotonic())
        if goal is not None:
            self.link.send_goal(goal)
            self._touch_motion()
        if was_running and not self.runner.running:
            self._tele_note = f"Route: {self.runner.message}"
        if self.pending:
            x, y, yaw = self.pending
            pose = snap.pose_map
            d = f"   {math.hypot(x - pose[0], y - pose[1]):.1f} m away" if pose else ""
            self.goal_txt.configure(text=f"Destination   x {x:+.2f}   y {y:+.2f}   facing {math.degrees(yaw):+.0f}°{d}")
            self.dest_bar.place(relx=0.5, rely=1.0, anchor="s", y=-16)
        else:
            self.dest_bar.place_forget()
        reasons = [g for g in self._gate]
        hint = "  Try Pose > Check pose from scan." if any("does not match" in g for g in reasons) else ""
        self.block_lbl.configure(text=("Blocked: " + "; ".join(reasons) + hint) if (self.pending and reasons) else "")
        busy = self.runner.running
        self.send_btn.state(["!disabled"] if (self.pending and not reasons and not busy) else ["disabled"])
        self.run_btn.state(["!disabled"] if (self.route and not reasons and not busy) else ["disabled"])
        route = []
        if self.route:
            route.append(f"{len(self.route)} legs queued")
        if self.runner.message and self.runner.state != RouteRunner.IDLE:
            route.append(self.runner.message)
        self.route_txt.configure(text="\n".join(route) if route else "empty: click the map, then + Route",
                                 fg=(T.BAD if self.runner.state == RouteRunner.FAILED else T.FG) if route else T.MUTED)

    def send_pending(self):
        if not self.pending or self._gate:
            return
        if self.link.send_goal(self.pending):
            self._sent = self.pending
            self.pending = None
            self._touch_motion()
            self._tele_note = ""

    def clear_pending(self):
        self.pending = None

    def add_to_route(self):
        if self.pending:
            self.route.append(self.pending)
            self.pending = None

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
                                   "Keep clear, hand near the power switch, and press STOP to cancel."):
            return
        self.runner.start(self.route)

    def cancel_all(self):
        """STOP: cancel goals and routes, stop teleop, send zero velocity. Not an emergency stop."""
        self.runner.stop("stopped by you")
        self.link.cancel_goals()
        if self.teleop_on:
            self._teleop_off("stopped by you")
        else:
            self.link.send_velocity(0.0, 0.0)
        self.pending = None

    # ================================================================== teleop
    def _speeds(self):
        self.tele.set_speeds(self.lin_var.get(), self.ang_var.get())
        self.lin_lbl.configure(text=f"Forward / back   {self.tele.linear:.2f} m/s   (max {MAX_LINEAR:.2f})")
        self.ang_lbl.configure(text=f"Turning   {self.tele.angular:.2f} rad/s   (max {MAX_ANGULAR:.1f})")

    def toggle_teleop(self):
        if self.teleop_on:
            self._teleop_off("stopped")
            return
        snap = self.link.snapshot()
        a = assess(snap, self.cfg)
        why = []
        if not self.armed.get():
            why.append("switch MOTION on first")
        if snap.conn_state != CONNECTED:
            why.append("not connected to the robot")
        if not a.scan_live:
            why.append("no live LiDAR (the obstacle guard needs it)")
        if why:
            messagebox.showwarning("Cannot start teleop", "- " + "\n- ".join(why))
            return
        if not messagebox.askyesno("Start teleop", "The robot will move while you press I, comma, J or L (or the pad).\n\n"
                                   "Stand clear of it, keep a hand near the power switch, and unplug the charger.\n"
                                   "Any active goal is cancelled."):
            return
        self.runner.stop("teleop started")
        self.link.cancel_goals()
        self.pending = None
        self.tele.stop()
        self.teleop_on = True
        self._tele_note = ""
        self._touch_motion()
        self.map_cv.focus_set()

    def _teleop_off(self, reason: str = ""):
        was = self.teleop_on
        self.teleop_on = False
        self.tele.stop()
        if was:
            self._tele_flush = 3                                   # a few zero commands so the robot really stops
            self._tele_note = f"Teleop off: {reason}" if reason else ""

    def tele_stop(self):
        self.tele.stop()

    def _pad_down(self, unit):
        if self.teleop_on:
            self.tele.hold(unit, time.monotonic())
            self._touch_motion()

    def _pad_up(self):
        self.tele.release()

    def _typing(self) -> bool:
        w = self.root.focus_get()
        return isinstance(w, (tk.Entry, ttk.Entry, ttk.Combobox))

    def _key_down(self, e):
        if self._typing():
            return None
        if e.keysym == "Escape" and self.teleop_on:
            self._teleop_off("Esc")
            return "break"
        if e.keysym in ("Return", "KP_Enter") and not self.teleop_on and self.pending:       # Enter = GO
            self.send_pending()
            return "break"
        if e.keysym == "space" and not self.teleop_on:               # Space also acts as a STOP for goals
            self.cancel_all()
            return None
        if self.teleop_on and self.tele.key_down(e.keysym, time.monotonic()):
            self._touch_motion()
            return "break"
        return None

    def _key_up(self, e):
        if self.teleop_on:
            self.tele.key_up(e.keysym)

    def _focus_out(self, e):
        if e.widget is self.root and self.teleop_on:
            self._teleop_off("the window lost focus")

    def _update_teleop(self, snap, a):
        on = self.teleop_on
        self.tele_state.configure(text="TELEOP ON" if on else "TELEOP OFF", fg=T.WARN if on else T.MUTED)
        self.tele_btn.configure(text="Stop teleop" if on else "Start teleop")
        self.tele_btn.set_variant("danger" if on else "primary")
        if not on:
            if self._tele_flush > 0:
                self.link.send_velocity(0.0, 0.0)
                self._tele_flush -= 1
            self.tele_out.configure(text="")
            return
        if snap.conn_state != CONNECTED or not self.armed.get():
            self._teleop_off("not connected" if snap.conn_state != CONNECTED else "motion switched off")
            return
        vx, wz = self.tele.command(time.monotonic())
        pts = transform_points(snap.scan.points(), snap.scan_in_base) if (a.scan_live and snap.scan_in_base is not None) else None
        gx, gz, why = guard(vx, wz, pts)
        self.link.send_velocity(gx, gz)                              # 10 Hz while teleop is on; zeros when no key is held
        self.tele_out.configure(text=f"sending  v {gx:+.2f} m/s   w {gz:+.2f} rad/s" + (f"\nBLOCKED: {why}" if why else ""),
                                fg=T.BAD if why else T.MUTED)

    # ================================================================== saved places
    def _sig(self):
        m = self.link.snapshot().map
        return map_signature(m) if m else None

    def _save_place(self, goal):
        try:
            sig = self._sig()
            if sig is None:
                raise ValueError("no map yet: connect to the robot first")
            name = self.places.put(self.name_var.get(), goal, sig)
            self.places.save()
        except Exception as exc:
            messagebox.showwarning("Cannot save place", str(exc))
            return
        self.place_box.configure(values=self.places.names())
        self.place_var.set(name)

    def save_pending_as_place(self):
        if self.pending is None:
            messagebox.showinfo("Save place", "Click the map to choose a destination first (Shift+drag sets the heading).")
            return
        self._save_place(self.pending)

    def save_robot_as_place(self):
        snap = self.link.snapshot()
        if not assess(snap, self.cfg).pose_live_in_map or snap.pose_map is None:
            messagebox.showinfo("Save robot pose", "The robot has no live map pose right now.")
            return
        self._save_place(snap.pose_map)

    def _selected_place(self):
        name = self.place_var.get()
        if not name or self.places.get(name) is None:
            messagebox.showinfo("Saved places", "Choose a saved place first.")
            return None
        if not self._places_ok:
            messagebox.showwarning("Saved places", "These places were saved on a different map than the one the robot is "
                                   "using, so they are not valid here.")
            return None
        return self.places.get(name)

    def go_to_place(self):
        goal = self._selected_place()
        if goal is None:
            return
        self.pending = goal
        if self._gate:
            messagebox.showwarning("Goal blocked", "Not sending:\n- " + "\n- ".join(self._gate))
            return
        self.send_pending()

    def add_place_to_route(self):
        goal = self._selected_place()
        if goal is not None:
            self.route.append(goal)

    def delete_place(self):
        name = self.place_var.get()
        if name and self.places.get(name) is not None and messagebox.askyesno("Delete place", f"Delete '{name}'?"):
            self.places.delete(name)
            self.places.save()
            self.place_box.configure(values=self.places.names())
            self.place_var.set("")

    # ================================================================== relocalisation
    def check_pose(self):
        """Search for the pose where the LiDAR scan fits the saved map best (background thread: it takes a second or two)."""
        snap = self.link.snapshot()
        if snap.map is None or snap.scan is None or snap.scan_in_base is None:
            messagebox.showinfo("Check pose", "Needs a live map and scan (and the scan's transform to the robot).")
            return
        if self._reloc and self._reloc.get("state") == "running":
            return
        pts = transform_points(snap.scan.points(), snap.scan_in_base)
        reported = snap.pose_map
        centre = None if (self.reloc_global.get() or reported is None) else reported
        reported_fit = fit_score(snap.map, transform_points(snap.scan.points(), snap.scan_in_map)) if snap.scan_in_map else None
        job = {"state": "running", "results": [], "reported": reported, "reported_fit": reported_fit, "whole_room": centre is None}
        self._reloc = job
        m = snap.map

        def work():
            try:
                job["results"] = pose_search(m, pts, centre)
            except Exception as exc:
                job["error"] = repr(exc)
            job["state"] = "done"
        threading.Thread(target=work, daemon=True, name="pose-search").start()

    def _reloc_best(self):
        job = self._reloc
        if not job or job.get("state") != "done" or not job.get("results"):
            return None
        return job["results"][0]

    def _reloc_ambiguous(self):
        res = self._reloc["results"]
        return len(res) > 1 and res[1][0] > res[0][0] - 0.08

    def _update_reloc(self, snap):
        job = self._reloc
        if not job:
            self.reloc_txt.configure(text="")
            self.apply_btn.state(["disabled"])
            return
        if job["state"] == "running":
            self.reloc_txt.configure(text="searching" + (" the whole room" if job["whole_room"] else "") + "...", fg=T.MUTED)
            self.apply_btn.state(["disabled"])
            return
        if job.get("error"):
            self.reloc_txt.configure(text="search failed: " + job["error"], fg=T.BAD)
            self.apply_btn.state(["disabled"])
            return
        best = self._reloc_best()
        if best is None:
            self.reloc_txt.configure(text="no usable fit: too few scan points, or no wall data.", fg=T.BAD)
            self.apply_btn.state(["disabled"])
            return
        score, x, y, yaw = best
        cur = job["reported_fit"]
        lines = [f"scan fits best at x {x:+.2f} y {y:+.2f} yaw {math.degrees(yaw):+.0f}°",
                 f"fit {score * 100:.0f}%" + (f" (at the reported pose: {cur * 100:.0f}%)" if cur is not None else "")]
        ok = score >= self.cfg.fit_ok
        if not ok:
            lines.append("not a good enough fit to trust: the robot may be outside the mapped area, or someone is next to it.")
        if self._reloc_ambiguous():
            r2 = job["results"][1]
            lines.append(f"AMBIGUOUS: x {r2[1]:+.2f} y {r2[2]:+.2f} fits almost as well ({r2[0] * 100:.0f}%). Do not apply.")
            ok = False
        self.reloc_txt.configure(text="\n".join(lines), fg=T.OK if ok else T.WARN)
        self.apply_btn.state(["!disabled"] if ok and snap.conn_state == CONNECTED else ["disabled"])

    def apply_pose(self):
        best = self._reloc_best()
        if best is None or best[0] < self.cfg.fit_ok or self._reloc_ambiguous():
            return
        score, x, y, yaw = best
        if not messagebox.askyesno("Apply pose to AMCL",
                                   f"Tell AMCL the robot is at x {x:+.2f}, y {y:+.2f}, facing {math.degrees(yaw):+.0f}° "
                                   f"(scan fit {score * 100:.0f}%)?\n\nOnly do this if the robot really is there (cyan cross on the map). "
                                   "It does not move the robot, but every goal afterwards is planned from this pose."):
            return
        if self.link.set_initial_pose((x, y, yaw)):
            self._reloc = None

    # ================================================================== drawing
    def _draw_map(self, snap, a):
        cv = self.map_cv
        w, h = self._size()
        cv.delete("overlay")
        m = snap.map
        if m is None:
            cv.delete("map")
            self._map_key = None
            cv.create_text(w / 2, h / 2, fill=T.MUTED, tags="overlay", justify="center", font=T.F_BASE,
                           text=f"No map on {self.cfg.map_topic}\n"
                                f"({'waiting for the robot…' if snap.conn_state == CONNECTED else 'not connected'})")
            return
        if self.view_fitted_for is None:
            self.view.fit(m, w, h)
            self.view_fitted_for = True
        v = self.view
        key = (snap.map_seq, round(v.cx, 4), round(v.cy, 4), round(v.scale, 3), w, h)
        if key != self._map_key:
            self._photo = ImageTk.PhotoImage(render_map(m, v, w, h, bg=(10, 13, 18)))
            self._map_key = key
            cv.delete("map")
            cv.create_image(0, 0, image=self._photo, anchor="nw", tags="map")
            cv.tag_lower("map")
        if self.show_scan.get() and a.scan_on_map:
            pts = transform_points(snap.scan.points(), snap.scan_in_map)
            for x, y in pts:
                u, vv = v.to_px(x, y, w, h)
                if 0 <= u < w and 0 <= vv < h:
                    cv.create_rectangle(u - 1, vv - 1, u + 1, vv + 1, fill="#f85149", outline="", tags="overlay")
        if a.pose_live_in_map and snap.pose_map:
            x, y, th = snap.pose_map
            u, vv = v.to_px(x, y, w, h)
            L = 13
            pts = [(u + L * math.cos(th), vv - L * math.sin(th)),
                   (u + 0.75 * L * math.cos(th + 2.5), vv - 0.75 * L * math.sin(th + 2.5)),
                   (u + 0.75 * L * math.cos(th - 2.5), vv - 0.75 * L * math.sin(th - 2.5))]
            rr = self.cfg.footprint_radius_m * v.scale
            if rr >= 3:
                cv.create_oval(u - rr, vv - rr, u + rr, vv + rr, outline=T.ACCENT, width=1, dash=(3, 3), tags="overlay")
            cv.create_polygon(*[c for p in pts for c in p], fill=T.ACCENT, outline="white", width=1, tags="overlay")
        else:
            cv.create_text(12, 12, anchor="nw", fill=T.WARN, tags="overlay", font=T.F_SMALL, text="Robot not drawn: no live map-frame pose")
        self._draw_goals(cv, v, w, h, snap)
        # scale bar
        px = v.scale
        cv.create_line(16, h - 18, 16 + px, h - 18, fill=T.FG, width=2, tags="overlay")
        cv.create_text(16 + px / 2, h - 24, text="1 m", fill=T.MUTED, font=T.F_TINY, tags="overlay")
        if self._cursor_world:
            self.cursor_lbl.configure(text=f"x {self._cursor_world[0]:+.2f}   y {self._cursor_world[1]:+.2f} m")

    def _draw_goals(self, cv, v, w, h, snap):
        if snap.plan and snap.plan_age is not None and snap.plan_age < 4.0 and snap.goal is not None and snap.goal[1] in (PENDING, ACTIVE):
            line = []
            for x, y in snap.plan:
                u, vv = v.to_px(x, y, w, h)
                line += [u, vv]
            if len(line) >= 4:
                cv.create_line(*line, fill=T.ACCENT_HI, width=2, tags="overlay")
        pts = []
        for i, (x, y, yaw) in enumerate(self.route):
            u, vv = v.to_px(x, y, w, h)
            pts.append((u, vv))
            cv.create_oval(u - 10, vv - 10, u + 10, vv + 10, outline=T.OK, width=2, fill=T.CANVAS, tags="overlay")
            cv.create_text(u, vv, text=str(i + 1), fill=T.OK, font=T.F_SMALL, tags="overlay")
            cv.create_line(u, vv, u + 24 * math.cos(yaw), vv - 24 * math.sin(yaw), fill=T.OK, width=2, arrow="last", tags="overlay")
        for p, q in zip(pts, pts[1:]):
            cv.create_line(p[0], p[1], q[0], q[1], fill=T.OK, dash=(4, 3), tags="overlay")
        best = self._reloc_best() if self._reloc else None
        if best is not None:
            u, vv = v.to_px(best[1], best[2], w, h)
            cv.create_line(u - 10, vv, u + 10, vv, fill="#39d0d8", width=2, tags="overlay")
            cv.create_line(u, vv - 10, u, vv + 10, fill="#39d0d8", width=2, tags="overlay")
            cv.create_line(u, vv, u + 24 * math.cos(best[3]), vv - 24 * math.sin(best[3]), fill="#39d0d8", width=2, arrow="last", tags="overlay")
        if self._places_ok:
            for name in self.places.names():
                x, y, yaw = self.places.get(name)
                u, vv = v.to_px(x, y, w, h)
                cv.create_polygon(u, vv - 8, u + 8, vv, u, vv + 8, u - 8, vv, fill="#a371f7", outline="white", tags="overlay")
                cv.create_text(u + 12, vv, text=name, anchor="w", fill="#d2a8ff", font=T.F_SMALL, tags="overlay")
        g = snap.goal
        if self._sent and g is not None and g[1] in (PENDING, ACTIVE):
            x, y, yaw = self._sent
            u, vv = v.to_px(x, y, w, h)
            cv.create_oval(u - 9, vv - 9, u + 9, vv + 9, outline=T.ACCENT_HI, width=3, tags="overlay")
            cv.create_line(u, vv, u + 26 * math.cos(yaw), vv - 26 * math.sin(yaw), fill=T.ACCENT_HI, width=3, arrow="last", tags="overlay")
        if self.pending:
            x, y, yaw = self.pending
            u, vv = v.to_px(x, y, w, h)
            cv.create_oval(u - 9, vv - 9, u + 9, vv + 9, outline="#ff9f1a", width=3, tags="overlay")
            cv.create_line(u, vv, u + 28 * math.cos(yaw), vv - 28 * math.sin(yaw), fill="#ff9f1a", width=3, arrow="last", tags="overlay")

    def _draw_lidar(self, snap, a):
        cv = self.lidar_cv
        cv.delete("all")
        W = H = 296
        cx = cy = W / 2
        rng = self.cfg.lidar_range_m
        scale = (W / 2 - 14) / rng
        for m in range(1, int(rng) + 1):
            cv.create_oval(cx - m * scale, cy - m * scale, cx + m * scale, cy + m * scale, outline=T.BORDER)
            cv.create_text(cx + 3, cy - m * scale, text=f"{m} m", fill=T.MUTED, anchor="sw", font=T.F_TINY)
        cv.create_polygon(cx, cy - 9, cx - 6, cy + 6, cx + 6, cy + 6, fill=T.ACCENT)
        if not a.scan_live:
            cv.create_text(cx, cy + 50, fill=T.BAD, text="NO LIVE SCAN\n" + a.scan.text, justify="center", font=T.F_SMALL)
            self.lidar_info.configure(text="")
            return
        pts = snap.scan.points()
        if snap.scan_in_base is not None:
            pts = transform_points(pts, snap.scan_in_base)
            frame_note = f"in {self.cfg.base_frame}"
        else:
            frame_note = f"SENSOR frame '{snap.scan.frame_id}' (no TF to {self.cfg.base_frame})"
        for x, y in pts:
            if math.hypot(x, y) > rng:
                continue
            u, v = cx - y * scale, cy - x * scale
            d = math.hypot(x, y)
            col = T.BAD if d < 0.4 else T.WARN if d < 1.0 else T.OK
            cv.create_rectangle(u - 1, v - 1, u + 1, v + 1, fill=col, outline=col)
        near = min((math.hypot(x, y) for x, y in pts), default=None)
        self.lidar_info.configure(text=f"{len(pts)}/{len(snap.scan.ranges)} valid beams, {frame_note}"
                                       + (f", nearest {near:.2f} m" if near is not None else ""))

    def _update_text(self, snap, a):
        lines = []
        if a.pose_live_in_map and snap.pose_map:
            x, y, th = snap.pose_map
            lines += ["MAP POSE (live, TF)", f"  x   {x:+8.3f} m", f"  y   {y:+8.3f} m", f"  yaw {math.degrees(th):+8.1f}°"]
        else:
            lines += ["MAP POSE: unavailable"]
            if a.pose.level == WARN and snap.pose_odom:
                x, y, th = snap.pose_odom
                lines += ["", "ODOMETRY ONLY (drifts, not on map)", f"  x   {x:+8.3f} m", f"  y   {y:+8.3f} m", f"  yaw {math.degrees(th):+8.1f}°"]
        if a.fit_value is not None:
            lines += ["", f"SCAN vs MAP: {a.fit_value * 100:.0f}% of points on walls"]
        if snap.amcl_pose and snap.amcl_age is not None:
            fresh = snap.amcl_age <= self.cfg.pose_stale_s
            sx, sa = snap.amcl_sigma or (float("nan"), float("nan"))
            lines += ["", f"/amcl_pose {'live' if fresh else 'STALE %.0fs' % snap.amcl_age}", f"  sigma xy {sx:.2f} m  yaw {math.degrees(sa):.0f}°"]
        self.pose_txt.configure(text="\n".join(lines))
        rows = []
        for name in (self.cfg.map_topic, self.cfg.scan_topic, self.cfg.odom_topic, self.cfg.amcl_pose_topic,
                     self.cfg.tf_topic, self.cfg.tf_static_topic, self.cfg.goal_status_topic):
            st = snap.stats.get(name)
            if st is None or st.last_rx is None:
                rows.append(f"{name:<19} no data" + (f"  [{st.last_error}]" if st and st.last_error else ""))
            else:
                rows.append(f"{name:<19} {st.count:>6} msgs, {snap.now - st.last_rx:5.1f}s ago")
        if snap.tf_frames:
            rows += ["", "TF frames: " + ", ".join(snap.tf_frames)]
        self.diag_txt.configure(text="\n".join(rows))


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="JetBot Pro navigation console (ROS 1 via rosbridge)")
    p.add_argument("--host", help="robot IP / hostname running rosbridge (use 127.0.0.1 through the SSH tunnel)")
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
