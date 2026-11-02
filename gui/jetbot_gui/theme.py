"""Look and feel: one palette, one font scale, and the few custom widgets the dashboard uses.

The native macOS buttons cannot be coloured, so the buttons here are small label-based widgets that look the same everywhere and
support the `state(["disabled"])` / `instate([...])` calls the rest of the code (and the tests) use on ttk buttons.
"""
from __future__ import annotations

import sys
import tkinter as tk
from tkinter import ttk
from typing import Callable, Optional

# ---- palette (dark, low-glare, one accent)
BG = "#0e1217"            # window
PANEL = "#151b23"         # side panels
CARD = "#1b222c"          # cards inside panels
CARD_HI = "#232c38"       # hover / raised
BORDER = "#2a3340"
FG = "#e6edf3"
MUTED = "#8b98a8"
ACCENT = "#388bfd"
ACCENT_HI = "#5aa1ff"
OK = "#3fb950"
OK_DK = "#238636"
WARN = "#d29922"
BAD = "#f85149"
BAD_DK = "#c93c37"
CANVAS = "#0a0d12"

_MAC = sys.platform == "darwin"
FAMILY = "Helvetica Neue" if _MAC else "DejaVu Sans" if sys.platform.startswith("linux") else "Segoe UI"
MONO = "Menlo" if _MAC else "DejaVu Sans Mono" if sys.platform.startswith("linux") else "Consolas"

F_BASE = (FAMILY, 11)
F_SMALL = (FAMILY, 9)
F_TINY = (FAMILY, 8)
F_BOLD = (FAMILY, 11, "bold")
F_TITLE = (FAMILY, 15, "bold")
F_H = (FAMILY, 10, "bold")
F_BIG = (FAMILY, 13, "bold")
F_MONO = (MONO, 10)
F_MONO_S = (MONO, 9)

# button variants: (background, foreground, hover background, disabled background)
VARIANTS = {
    "primary": (ACCENT, "#ffffff", ACCENT_HI, "#27456b"),
    "success": (OK_DK, "#ffffff", OK, "#1d3d28"),
    "danger": (BAD_DK, "#ffffff", BAD, "#4a2523"),
    "secondary": (CARD_HI, FG, "#2e3a49", CARD),
    "ghost": (CARD, MUTED, CARD_HI, CARD),
}


class ActionButton(tk.Label):
    """A flat, coloured button. Works like a ttk button for state()/instate(); `command` runs on click."""

    def __init__(self, master, text: str, command: Optional[Callable[[], None]] = None, variant: str = "secondary",
                 font=F_BASE, padx: int = 14, pady: int = 7, width: Optional[int] = None, **kw):
        self._variant = variant
        self._command = command
        self._disabled = False
        bg, fg, _, _ = VARIANTS[variant]
        super().__init__(master, text=text, bg=bg, fg=fg, font=font, padx=padx, pady=pady, bd=0, cursor="hand2",
                         anchor="center", **({"width": width} if width else {}), **kw)
        self.bind("<Enter>", self._enter)
        self.bind("<Leave>", self._leave)
        self.bind("<ButtonRelease-1>", self._click)

    # -- ttk-compatible state API
    def state(self, spec=None):
        if spec is None:
            return ("disabled",) if self._disabled else ()
        for s in spec:
            if s == "disabled":
                self._disabled = True
            elif s == "!disabled":
                self._disabled = False
        self._paint()
        return None

    def instate(self, spec) -> bool:
        return all((self._disabled if s == "disabled" else not self._disabled) if s in ("disabled", "!disabled") else False
                   for s in spec)

    def set_variant(self, variant: str) -> None:
        self._variant = variant
        self._paint()

    def _paint(self, hover: bool = False) -> None:
        bg, fg, hi, dis = VARIANTS[self._variant]
        if self._disabled:
            self.configure(bg=dis, fg=MUTED, cursor="arrow")
        else:
            self.configure(bg=hi if hover else bg, fg=fg, cursor="hand2")

    def _enter(self, _e):
        if not self._disabled:
            self._paint(hover=True)

    def _leave(self, _e):
        self._paint()

    def _click(self, e):
        if self._disabled or not self._command:
            return
        if 0 <= e.x <= self.winfo_width() and 0 <= e.y <= self.winfo_height():
            self._command()

    def invoke(self) -> None:
        """Same effect as a click (used by keyboard shortcuts and tests)."""
        if not self._disabled and self._command:
            self._command()


class HoldButton(ActionButton):
    """A button that acts while it is held down (the teleop pad): on_down at press, on_up at release or when the pointer leaves."""

    def __init__(self, master, text: str, on_down: Callable[[], None], on_up: Callable[[], None], **kw):
        super().__init__(master, text, command=None, **kw)
        self._on_down, self._on_up, self._held = on_down, on_up, False
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Leave>", self._leave_hold)

    def _press(self, _e):
        if not self._disabled:
            self._held = True
            self._on_down()

    def _release(self, _e):
        if self._held:
            self._held = False
            self._on_up()

    def _leave_hold(self, e):
        self._leave(e)
        self._release(e)


class Card(tk.Frame):
    """A titled rounded-looking panel section."""

    def __init__(self, master, title: str = "", **kw):
        super().__init__(master, bg=CARD, highlightbackground=BORDER, highlightthickness=1, **kw)
        if title:
            tk.Label(self, text=title.upper(), bg=CARD, fg=MUTED, font=F_TINY, anchor="w").pack(fill="x", padx=12, pady=(10, 2))
        self.body = tk.Frame(self, bg=CARD)
        self.body.pack(fill="both", expand=True, padx=12, pady=(2, 12))


class Dot(tk.Canvas):
    """A small status dot."""

    def __init__(self, master, size: int = 10, bg: str = CARD):
        super().__init__(master, width=size, height=size, bg=bg, highlightthickness=0)
        self._size = size
        self._id = self.create_oval(1, 1, size - 1, size - 1, fill=MUTED, outline="")

    def set(self, color: str) -> None:
        self.itemconfigure(self._id, fill=color)


def apply_theme(root: tk.Misc) -> None:
    """ttk styles for the widgets that are not custom (entry, combobox, scale, notebook, checkbutton, scrollbar)."""
    root.configure(bg=BG)
    s = ttk.Style(root)
    s.theme_use("clam")
    s.configure(".", background=PANEL, foreground=FG, fieldbackground=CARD_HI, bordercolor=BORDER, lightcolor=PANEL,
                darkcolor=PANEL, troughcolor=CARD, focuscolor=PANEL, font=F_BASE)
    s.configure("TFrame", background=PANEL)
    s.configure("TLabel", background=PANEL, foreground=FG)
    s.configure("TEntry", fieldbackground=CARD_HI, foreground=FG, insertcolor=FG, bordercolor=BORDER, padding=6)
    s.configure("TCombobox", fieldbackground=CARD_HI, background=CARD_HI, foreground=FG, arrowcolor=FG, bordercolor=BORDER,
                padding=6, selectbackground=CARD_HI, selectforeground=FG)
    s.map("TCombobox", fieldbackground=[("readonly", CARD_HI)], foreground=[("readonly", FG)],
          selectbackground=[("readonly", CARD_HI)], selectforeground=[("readonly", FG)])
    s.configure("TCheckbutton", background=CARD, foreground=FG, indicatorbackground=CARD_HI, indicatorforeground=ACCENT,
                font=F_BASE)
    s.map("TCheckbutton", background=[("active", CARD)], indicatorbackground=[("selected", ACCENT)])
    s.configure("Panel.TCheckbutton", background=PANEL, foreground=FG, indicatorbackground=CARD_HI, indicatorforeground=ACCENT, font=F_SMALL)
    s.map("Panel.TCheckbutton", background=[("active", PANEL)], indicatorbackground=[("selected", ACCENT)])
    s.configure("Horizontal.TScale", background=CARD, troughcolor=CARD_HI, bordercolor=CARD, lightcolor=ACCENT, darkcolor=ACCENT)
    s.configure("TNotebook", background=PANEL, borderwidth=0, tabmargins=(0, 0, 0, 0))
    s.configure("TNotebook.Tab", background=PANEL, foreground=MUTED, padding=(14, 9), borderwidth=0, font=F_H)
    s.map("TNotebook.Tab", background=[("selected", CARD)], foreground=[("selected", FG)])
    s.configure("Vertical.TScrollbar", background=CARD_HI, troughcolor=PANEL, bordercolor=PANEL, arrowcolor=MUTED)
    root.option_add("*TCombobox*Listbox.background", CARD_HI)
    root.option_add("*TCombobox*Listbox.foreground", FG)
    root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
    root.option_add("*TCombobox*Listbox.font", F_BASE)

class ScrollFrame(tk.Frame):
    """A vertically scrollable area. Put widgets in `.body`. The scrollbar only appears when the content does not fit."""

    def __init__(self, master, bg: str = PANEL):
        super().__init__(master, bg=bg)
        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0, bd=0)
        self.bar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.bar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.body = tk.Frame(self.canvas, bg=bg)
        self._win = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self._bar_shown = False
        self.body.bind("<Configure>", self._on_body)
        self.canvas.bind("<Configure>", self._on_canvas)
        for w in (self.canvas, self.body):
            w.bind("<Enter>", self._enter)
            w.bind("<Leave>", self._leave)

    def _on_body(self, _e=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self._toggle_bar()

    def _on_canvas(self, e):
        self.canvas.itemconfigure(self._win, width=e.width)
        self._toggle_bar()

    def _toggle_bar(self):
        need = self.body.winfo_reqheight() > self.canvas.winfo_height() + 1
        if need and not self._bar_shown:
            self.bar.pack(side="right", fill="y")
            self._bar_shown = True
        elif not need and self._bar_shown:
            self.bar.pack_forget()
            self._bar_shown = False
            self.canvas.yview_moveto(0)

    def _enter(self, _e):
        self.canvas.bind_all("<MouseWheel>", self._wheel)

    def _leave(self, _e):
        self.canvas.unbind_all("<MouseWheel>")

    def _wheel(self, e):
        if self._bar_shown:
            self.canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")
