EXTERNAL_NAVI_SOURCES = frozenset((
  "cam", "section", "bump", "police", "waze", "road", "atc", "atc2", "route",
))
VEHICLE_NAVI_SOURCES = frozenset(("hda", "hda_section", "hda_bump", "hda_curve", "hda_ramp", "school"))

DECELERATION_SOURCE_LABELS = {
  "cam": "cam",
  "section": "section",
  "bump": "bump",
  "police": "police",
  "waze": "waze",
  "road": "road",
  "atc": "turn",
  "atc2": "turn",
  "route": "route",
  "hda": "cam",
  "hda_section": "section",
  "hda_bump": "bump",
  "hda_curve": "curve",
  "hda_ramp": "ramp",
  "school": "school",
  "gas": "gas",
  "vturn": "vturn",
  "model": "model",
  "turn": "turn",
}


def is_vehicle_navigation_source(source: str | None) -> bool:
  normalized = str(source or "").strip().lower()
  return normalized in VEHICLE_NAVI_SOURCES or normalized in ("cam:v", "bump:v", "curve:v", "ramp:v", "school:v")


def deceleration_source_presentation(source: str | None) -> tuple[str, int]:
  """Return the actual deceleration reason and its source color mode.

  Color mode 2 is the normal deceleration orange, mode 3 is vehicle CAN
  navigation lavender, and mode 4 is external-navigation orange.
  """
  normalized = str(source or "").strip().lower()
  if not normalized:
    return "apply", 2
  if is_vehicle_navigation_source(normalized):
    base = normalized.removesuffix(":v")
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), 3
  if normalized in EXTERNAL_NAVI_SOURCES or normalized.endswith(":n"):
    base = normalized.removesuffix(":n")
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), 4
  if normalized.endswith(":v"):
    base = normalized.removesuffix(":v")
    mode = 3 if base in ("cam", "section", "bump", "curve", "ramp", "school") else 2
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), mode
  if normalized.endswith(":c"):
    base = normalized.removesuffix(":c")
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), 2
  return DECELERATION_SOURCE_LABELS.get(normalized, normalized[:8]), 2


def format_vehicle_navi_junction(junction_type: int, side: int, distance_m: int | float) -> str:
  """Return compact HUD/cluster label for stock-navi fork/merge guidance, or empty string."""
  jtype = int(junction_type or 0)
  if jtype == 1:
    kind = "FORK"
  elif jtype == 2:
    kind = "MERGE"
  else:
    return ""
  arrow = "<- " if int(side or 0) == 1 else (" ->" if int(side or 0) == 2 else "")
  dist = max(0, int(round(float(distance_m or 0))))
  dist_text = "NOW" if dist == 0 else f"{dist}m"
  if int(side or 0) == 1:
    return f"{arrow}{kind} {dist_text}"
  if int(side or 0) == 2:
    return f"{kind}{arrow} {dist_text}"
  return f"{kind} {dist_text}"


def navigation_status_presentation(vehicle_available: bool, external_active: bool) -> tuple[str, int] | None:
  """Return the navigation availability badge, independent of speed control."""
  if external_active:
    return "NAVI", 4
  if vehicle_available:
    return "vNAVI", 3
  return None


def external_navigation_connected(legacy_remote: str | None, carrot_navi_connected: bool) -> bool:
  """Return external-navigation connectivity without consulting speed-control state."""
  return bool(str(legacy_remote or "").strip()) or bool(carrot_navi_connected)
