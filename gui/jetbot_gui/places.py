"""Saved named places (map-frame goals such as "start", "fridge"). Pure logic: no Tk, no network.

Stored in places.local.json next to config.local.json (git-ignored: your places are yours). The file remembers which map
the places were saved on, because a place is only meaningful on that map; if the robot is running a different map the GUI
refuses to go to a saved place.
"""
from __future__ import annotations

import json
import math
import os
import re
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .config import GUI_DIR

PLACES_FILE = GUI_DIR / "places.local.json"
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _.-]{0,31}$")
Goal = Tuple[float, float, float]


def clean_name(name: str) -> str:
    name = (name or "").strip()
    if not NAME_RE.match(name):
        raise ValueError("place names are 1-32 characters: letters, digits, space, _ . -")
    return name


def map_signature(m) -> dict:
    """What identifies a map: size, resolution and origin (a different map has a different frame of reference)."""
    return {"width": int(m.width), "height": int(m.height), "resolution": round(float(m.resolution), 4),
            "origin": [round(float(m.origin[0]), 3), round(float(m.origin[1]), 3)]}


class Places:
    def __init__(self, path: Path = PLACES_FILE):
        self.path = Path(path)
        self.items: Dict[str, Goal] = {}
        self.map_sig: Optional[dict] = None

    def names(self) -> List[str]:
        return sorted(self.items, key=str.lower)

    def get(self, name: str) -> Optional[Goal]:
        return self.items.get(name)

    def put(self, name: str, goal: Goal, sig: Optional[dict] = None) -> str:
        name = clean_name(name)
        x, y, yaw = (float(v) for v in goal)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("a place needs finite x, y and yaw")
        if sig is not None:
            if self.map_sig is not None and self.items and sig != self.map_sig:
                raise ValueError("these places were saved on a different map; delete them or use that map")
            self.map_sig = sig
        self.items[name] = (x, y, yaw)
        return name

    def delete(self, name: str) -> bool:
        return self.items.pop(name, None) is not None

    def mismatch(self, sig: Optional[dict]) -> bool:
        """True when the places belong to a different map than the one the robot is using."""
        return bool(self.items) and self.map_sig is not None and sig is not None and sig != self.map_sig

    def load(self) -> None:
        if not self.path.exists():
            return
        data = json.loads(self.path.read_text())
        self.map_sig = data.get("map")
        self.items = {}
        for name, g in data.get("places", {}).items():
            self.items[clean_name(name)] = tuple(float(v) for v in g)       # type: ignore[assignment]

    def save(self) -> None:
        payload = {"map": self.map_sig, "places": {n: list(g) for n, g in self.items.items()}}
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), prefix=".places-", suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(payload, fh, indent=2)
                fh.write("\n")
            os.replace(tmp, self.path)                  # atomic: a crash never leaves a half-written file
        except Exception:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
