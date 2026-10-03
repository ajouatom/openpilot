"""Hardware-backed screen recording (see recorder.py for design notes).

The recorder import is lazy so that headless helpers (muxer, encoder wrapper)
can be used without importing the raylib-based recorder.
"""
from __future__ import annotations

__all__ = ["ScreenRecordHw"]


def __getattr__(name: str):
  if name == "ScreenRecordHw":
    from openpilot.selfdrive.carrot.screen_record.recorder import ScreenRecordHw

    return ScreenRecordHw
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
