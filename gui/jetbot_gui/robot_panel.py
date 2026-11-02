"""The Robot tab: start the robot software, map a new area, and manage the map library. Talks to the JetBot manager only (never to ROS).

Nothing here can move the robot: starting the software brings up the drivers and the navigation or mapping stack, and motion still needs the
MOTION switch, a goal or the teleop pad.
"""
from __future__ import annotations

import io
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from typing import Optional

import numpy as np
from PIL import Image, ImageTk

from . import theme as T
from .manager_client import ManagerSnapshot

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")
PREVIEW_W, PREVIEW_H = 296, 200


def fmt_age(seconds: Optional[float]) -> str:
    if seconds is None:
        return "?"
    s = int(seconds)
    if s < 3600:
        return "%dm" % (s // 60)
    if s < 86400:
        return "%dh %02dm" % (s // 3600, (s % 3600) // 60)
    return "%dd %dh" % (s // 86400, (s % 86400) // 3600)


def preview_image(pgm: bytes) -> Optional[Image.Image]:
    """A map thumbnail: cropped to the known area, scaled to fit, nearest-neighbour so walls stay crisp."""
    try:
        im = Image.open(io.BytesIO(pgm)).convert("L")
    except Exception:
        return None
    a = np.asarray(im)
    known = np.argwhere(a != 205)
    if known.size:
        (r0, c0), (r1, c1) = known.min(axis=0), known.max(axis=0)
        pad = 6
        im = im.crop((max(c0 - pad, 0), max(r0 - pad, 0), min(c1 + pad + 1, im.width), min(r1 + pad + 1, im.height)))
    scale = min(PREVIEW_W / im.width, PREVIEW_H / im.height)
    scale = max(1, int(scale)) if scale >= 1 else scale
    return im.resize((max(1, int(im.width * scale)), max(1, int(im.height * scale))), Image.NEAREST).convert("RGB")


class RobotPanel:
    def __init__(self, app, parent: tk.Frame):
        self.app, self.mgr = app, app.mgr
        self._maps_key = None
        self._sel_name: Optional[str] = None
        self._preview_for = None
        self._preview_img: Optional[Image.Image] = None
        self._photo = None
        self._build(parent)

    # ================================================================== layout
    def _build(self, tab):
        c = T.Card(tab, "Robot software")
        c.pack(fill="x", padx=12, pady=(12, 8))
        self.mode_lbl = tk.Label(c.body, text="NOT CONNECTED", bg=T.CARD, fg=T.MUTED, font=T.F_BIG, anchor="w")
        self.mode_lbl.pack(fill="x")
        self.detail_lbl = tk.Label(c.body, text="", bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, anchor="w", justify="left", wraplength=320)
        self.detail_lbl.pack(fill="x", pady=(2, 8))
        self.nav_btn = T.ActionButton(c.body, "Start navigation on selected map", self.start_navigation, "primary", pady=8)
        self.nav_btn.pack(fill="x")
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(8, 0))
        self.map_btn = T.ActionButton(row, "Map a new area", self.start_mapping, "secondary", pady=7)
        self.map_btn.pack(side="left", fill="x", expand=True)
        self.stop_sw_btn = T.ActionButton(row, "Stop software", self.stop_software, "danger", pady=7)
        self.stop_sw_btn.pack(side="left", fill="x", expand=True, padx=(8, 0))
        self.msg_lbl = tk.Label(c.body, text="", bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, anchor="w", justify="left", wraplength=320)
        self.msg_lbl.pack(fill="x", pady=(8, 0))

        c = T.Card(tab, "Map library")
        c.pack(fill="x", padx=12, pady=8)
        self.tree = ttk.Treeview(c.body, columns=("size", "saved"), height=5, selectmode="browse")
        self.tree.heading("#0", text="Map")
        self.tree.heading("size", text="Size")
        self.tree.heading("saved", text="Saved")
        self.tree.column("#0", width=150, stretch=True)
        self.tree.column("size", width=60, anchor="e", stretch=False)
        self.tree.column("saved", width=78, anchor="e", stretch=False)
        self.tree.pack(fill="x")
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        self.preview_cv = tk.Canvas(c.body, width=PREVIEW_W, height=PREVIEW_H, bg=T.CANVAS, highlightthickness=0)
        self.preview_cv.pack(pady=(8, 4))
        row = tk.Frame(c.body, bg=T.CARD)
        row.pack(fill="x", pady=(4, 0))
        T.ActionButton(row, "Rename", self.rename_map, "secondary", font=T.F_SMALL, pady=5).pack(side="left")
        T.ActionButton(row, "Delete", self.delete_map, "secondary", font=T.F_SMALL, pady=5).pack(side="left", padx=6)
        T.ActionButton(row, "Refresh", self.mgr_refresh, "ghost", font=T.F_SMALL, pady=5).pack(side="left")
        tk.Label(c.body, text="DELETED MAPS (RECOVERABLE)", bg=T.CARD, fg=T.MUTED, font=T.F_TINY).pack(anchor="w", pady=(10, 2))
        self.trash_var = tk.StringVar()
        self.trash_box = ttk.Combobox(c.body, textvariable=self.trash_var, state="readonly", values=[])
        self.trash_box.pack(fill="x")
        T.ActionButton(c.body, "Restore", self.restore_map, "secondary", font=T.F_SMALL, pady=5).pack(anchor="w", pady=(6, 0))

        c = T.Card(tab, "Save the map being built")
        c.pack(fill="x", padx=12, pady=8)
        self.save_hint = tk.Label(c.body, text="Start mapping, drive around, then name and save the map here.", bg=T.CARD, fg=T.MUTED,
                                  font=T.F_SMALL, anchor="w", justify="left", wraplength=320)
        self.save_hint.pack(fill="x")
        self.name_var = tk.StringVar()
        ttk.Entry(c.body, textvariable=self.name_var).pack(fill="x", pady=(8, 0))
        self.save_btn = T.ActionButton(c.body, "Save map", self.save_map, "success", pady=7)
        self.save_btn.pack(fill="x", pady=(8, 0))

        c = T.Card(tab, "Mapping tips")
        c.pack(fill="x", padx=12, pady=(8, 12))
        tk.Label(c.body, bg=T.CARD, fg=T.MUTED, font=T.F_SMALL, anchor="w", justify="left", wraplength=320, text=(
            "The map's origin is where the robot stands when mapping starts: pick a spot you can find again.\n\n"
            "Drive slowly (Teleop tab, 0.10-0.15 m/s), keep 0.5 m from walls, and finish by returning to the start so the loop closes. "
            "Maps are never overwritten: every save needs a new name.")).pack(fill="x")

    # ================================================================== per-tick update
    def selected_map(self) -> Optional[dict]:
        name = self._sel_name
        for m in self.mgr.snapshot().maps:
            if m["name"] == name:
                return m
        return None

    def update(self, ms: ManagerSnapshot, a=None) -> None:
        st = ms.status or {}
        mode = st.get("mode", "idle") if ms.connected else None
        if not self.mgr.enabled:
            title, color = "MANAGER NOT CONNECTED", T.MUTED
        elif not ms.connected:
            title, color = "MANAGER UNREACHABLE", T.BAD
        elif mode == "navigation":
            title, color = "NAVIGATION RUNNING", T.OK
        elif mode == "mapping":
            title, color = "MAPPING RUNNING", T.WARN
        else:
            title, color = "SOFTWARE STOPPED", T.MUTED
        self.mode_lbl.configure(text=title, fg=color)
        lines = []
        if ms.connected:
            h = st.get("host", {})
            lines.append("Jetson up %s   load %s   %s GB free" % (fmt_age(h.get("uptime_s")), "%.1f" % h["load1"] if h.get("load1") is not None else "?",
                                                                    h.get("disk_free_gb", "?")))
            fw = st.get("firewall", {})
            lines.append("ROS %s   rosbridge %s   firewall %s" % ("up" if st.get("ros_master") else "down", "up" if st.get("rosbridge") else "down",
                                                                   "ok" if fw.get("ok") else "NOT ACTIVE"))
            if st.get("active_map"):
                lines.append("Map in use: %s" % st["active_map"])
            if st.get("last_exit"):
                lines.append("Last stack ended: %s" % st["last_exit"])
            if not fw.get("ok"):
                lines.append("rosbridge will not start without the firewall (see manager/README.md).")
        elif ms.error:
            lines.append(ms.error)
        self.detail_lbl.configure(text="\n".join(lines), fg=T.MUTED)
        msg = ("working: %s ..." % ms.busy) if ms.busy else ms.last_action
        self.msg_lbl.configure(text=msg, fg=T.MUTED if ms.busy or ms.last_ok is not False else T.BAD)

        self._fill_maps(ms)                                   # first: it picks the default selection the buttons depend on
        sel = self.selected_map()
        ready = ms.connected and not ms.busy
        self.nav_btn.state(["!disabled"] if (ready and sel) else ["disabled"])
        self.map_btn.state(["!disabled"] if ready else ["disabled"])
        self.stop_sw_btn.state(["!disabled"] if (ready and mode in ("navigation", "mapping")) else ["disabled"])
        mapping = ms.connected and mode == "mapping"
        self.save_btn.state(["!disabled"] if (mapping and not ms.busy) else ["disabled"])
        self.save_hint.configure(text=("Mapping is running. Name the map and save it (a new name every time)." if mapping
                                       else "Start mapping first; saving only works while a map is being built."))

    def _fill_maps(self, ms: ManagerSnapshot) -> None:
        key = (tuple((m["name"], m["modified"], m["active"]) for m in ms.maps), tuple(t["entry"] for t in ms.trash))
        if key != self._maps_key:
            self._maps_key = key
            self.tree.delete(*self.tree.get_children())
            for m in ms.maps:
                size = "%.0f x %.0f m" % (m["width"] * (m["resolution"] or 0), m["height"] * (m["resolution"] or 0)) if m.get("resolution") else "?"
                saved = time.strftime("%d %b %H:%M", time.localtime(m["modified"]))
                self.tree.insert("", "end", iid=m["name"], text=("● " if m["active"] else "  ") + m["name"], values=(size, saved))
            if self._sel_name and any(m["name"] == self._sel_name for m in ms.maps):
                self.tree.selection_set(self._sel_name)
            elif ms.maps:
                self._sel_name = (ms.active if ms.active and any(m["name"] == ms.active for m in ms.maps) else ms.maps[-1]["name"])
                self.tree.selection_set(self._sel_name)
            else:
                self._sel_name = None
            self.trash_box.configure(values=["%s   (%s)" % (t["name"], t["when"]) for t in ms.trash])
            self._trash_entries = {"%s   (%s)" % (t["name"], t["when"]): t["entry"] for t in ms.trash}
        # preview: fetched in a worker thread, turned into a PhotoImage here (Tk objects belong to the UI thread)
        sel = self.selected_map()
        want = (sel["name"], sel["modified"]) if sel else None
        if want != self._preview_for:
            self._preview_for, self._preview_img = want, None
            self.preview_cv.delete("all")
            if want:
                self.preview_cv.create_text(PREVIEW_W / 2, PREVIEW_H / 2, text="loading preview…", fill=T.MUTED, font=T.F_SMALL)
                threading.Thread(target=self._load_preview, args=(want,), daemon=True, name="map-preview").start()
        if self._preview_img is not None and self._preview_for == want:
            self._photo = ImageTk.PhotoImage(self._preview_img)
            self._preview_img = None
            self.preview_cv.delete("all")
            self.preview_cv.create_image(PREVIEW_W / 2, PREVIEW_H / 2, image=self._photo)

    def _load_preview(self, want) -> None:
        data = self.mgr.image(want[0], want[1])
        img = preview_image(data) if data else None
        if self._preview_for == want:
            self._preview_img = img if img is not None else None

    def _on_select(self, _e=None) -> None:
        sel = self.tree.selection()
        self._sel_name = sel[0] if sel else None

    # ================================================================== actions
    def mgr_refresh(self) -> None:
        self.mgr.refresh()

    def _ui(self, fn) -> None:
        """Run `fn` on the UI thread (action callbacks arrive on worker threads)."""
        self.app.root.after(0, fn)

    def start_navigation(self) -> None:
        m = self.selected_map()
        if not m:
            messagebox.showinfo("Start navigation", "Select a map in the library first.")
            return
        if not messagebox.askyesno("Start navigation", f"Start the navigation software on '{m['name']}'?\n\n"
                                   "This (re)starts the robot software; anything running is stopped first. The robot is assumed to be at the map's start "
                                   "point, facing its start direction. If it is not, press Pose > Check pose from scan once it is up.\n\nNothing moves by itself."):
            return
        self.mgr.act("start navigation", "POST", "/start", {"mode": "navigation", "map": m["name"], "replace": True})

    def start_mapping(self) -> None:
        if not messagebox.askyesno("Map a new area", "Start the mapping software?\n\nThe new map's origin is where the robot stands right now: "
                                   "choose a spot you can find again, and put the robot there, still. Anything running is stopped first.\n\n"
                                   "Then drive with the Teleop tab and save the map here when you are done."):
            return
        self.mgr.act("start mapping", "POST", "/start", {"mode": "mapping", "replace": True})

    def stop_software(self) -> None:
        if messagebox.askyesno("Stop the robot software", "Stop the navigation or mapping software?\n\nMake sure the robot is not moving. "
                               "The robot stays powered; the manager and the link stay up."):
            self.mgr.act("stop software", "POST", "/stop", {})

    def save_map(self) -> None:
        name = self.name_var.get().strip()
        if not NAME_RE.match(name):
            messagebox.showwarning("Save map", "Use 1-48 characters: letters, digits, _ and -, starting with a letter or digit.")
            return
        if any(m["name"] == name for m in self.mgr.snapshot().maps):
            messagebox.showwarning("Save map", f"A map called '{name}' already exists. Maps are never overwritten: choose another name.")
            return

        def done(ok: bool, text: str) -> None:
            if ok:
                self._ui(lambda: self._after_save(name))
        self.mgr.act("save map", "POST", "/maps/save", {"name": name}, done=done)

    def _after_save(self, name: str) -> None:
        self.name_var.set("")
        self._sel_name = name
        if messagebox.askyesno("Map saved", f"'{name}' is saved.\n\nSwitch to navigation on it now? This stops mapping. "
                               "The robot should be at the map's start point (or use Pose > Check pose from scan afterwards)."):
            self.mgr.act("start navigation", "POST", "/start", {"mode": "navigation", "map": name, "replace": True})

    def rename_map(self) -> None:
        m = self.selected_map()
        if not m:
            return
        new = simpledialog.askstring("Rename map", f"New name for '{m['name']}':", parent=self.app.root)
        if new is None:
            return
        new = new.strip()
        if not NAME_RE.match(new):
            messagebox.showwarning("Rename map", "Use 1-48 characters: letters, digits, _ and -, starting with a letter or digit.")
            return
        self._sel_name = new
        self.mgr.act("rename map", "POST", "/maps/rename", {"name": m["name"], "new_name": new})

    def delete_map(self) -> None:
        m = self.selected_map()
        if not m:
            return
        if messagebox.askyesno("Delete map", f"Move '{m['name']}' to the trash?\n\nIt is not erased: you can restore it from 'Deleted maps' below."):
            self._sel_name = None
            self.mgr.act("delete map", "POST", "/maps/delete", {"name": m["name"]})

    def restore_map(self) -> None:
        entry = getattr(self, "_trash_entries", {}).get(self.trash_var.get())
        if not entry:
            messagebox.showinfo("Restore map", "Choose a deleted map first.")
            return
        self.mgr.act("restore map", "POST", "/maps/restore", {"entry": entry})
