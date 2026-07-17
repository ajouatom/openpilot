"""Carrot intro driving-control presets.

This module is the ONLY place the preset values live. The browser sends a
preset name; the server decides what that means.

Why not let the client write them?
  There is no bulk param write API, so the client would have to call
  POST /api/param_set once per param. A failure partway through leaves the
  vehicle control settings half-applied — the browser would have set, say,
  HyundaiCameraSCC but not SpeedFromPCM. These are driving control params;
  a partial state is not acceptable. Applying them here keeps it one request.

Preset table (confirmed 2026-07-17):

                        HyundaiCameraSCC  SpeedFromPCM  EnableCornerRadar  AutoCruiseControl
  radar_long   (ADAS)          1               0               1                  1
  camera_long  (camera)        1               0               0                  1
  stock        (stock SCC)     0               2               0                  0

  DisableDM / EnableRadarTracks are 0 for all three; AutoEngage is 2 for all three.

EnableCornerRadar is the ONLY axis separating radar_long from camera_long.
Changing it makes the two presets identical — do not "tidy" it away.

CanfdHDA2 is NOT part of a preset. It comes from the HDA step and is only
written for Hyundai/Kia/Genesis; other brands keep whatever they had.
"""
from __future__ import annotations

from typing import Dict

# Param -> value. Keys must exist in selfdrive/carrot_settings.json so the
# clamp in services.params can validate them.
PRESETS: Dict[str, Dict[str, int]] = {
  "radar_long": {
    "HyundaiCameraSCC": 1,   # 1: long-con vehicle (interface.py forces OP long)
    "SpeedFromPCM": 0,       # 0: curve/camera decel + long
    "DisableDM": 0,          # driver monitoring stays ON
    "EnableRadarTracks": 0,  # 0: use SCC radar
    "EnableCornerRadar": 1,  # harness is on ADAS/radar, so corner radar is reachable
    "AutoCruiseControl": 1,
    "AutoEngage": 2,         # 2: steering ON + cruise standby
  },
  "camera_long": {
    "HyundaiCameraSCC": 1,
    "SpeedFromPCM": 0,
    "DisableDM": 0,
    "EnableRadarTracks": 0,
    "EnableCornerRadar": 0,  # harness is on the camera only
    "AutoCruiseControl": 1,
    "AutoEngage": 2,
  },
  "stock": {
    "HyundaiCameraSCC": 0,   # 0: no CAMERA_SCC flag -> stock SCC keeps longitudinal
    "SpeedFromPCM": 2,       # 2: curve/camera decel only (button spam)
    "DisableDM": 0,
    "EnableRadarTracks": 0,
    "EnableCornerRadar": 0,
    "AutoCruiseControl": 0,
    "AutoEngage": 2,
  },
}

PRESET_NAMES = tuple(PRESETS.keys())

# Set from the HDA step, not from a preset. Hyundai/Kia/Genesis only.
HDA_PARAM = "CanfdHDA2"


def get_preset(name: str) -> Dict[str, int] | None:
  """Return a copy of the preset, or None when the name is unknown."""
  values = PRESETS.get(str(name or "").strip())
  return dict(values) if values else None
