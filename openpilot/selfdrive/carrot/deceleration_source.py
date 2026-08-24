EXTERNAL_NAVI_SOURCES = frozenset((
  "cam", "section", "bump", "police", "waze", "road", "atc", "atc2", "route",
))
VEHICLE_NAVI_SOURCES = frozenset(("hda", "hda_bump", "school"))

DECELERATION_SOURCE_LABELS = {
  "gas": "gas:v",
  "vturn": "turn:c",
  "model": "turn:c",
  "turn": "turn:c",
}


def is_vehicle_navigation_source(source: str | None) -> bool:
  normalized = str(source or "").strip().lower()
  return normalized in VEHICLE_NAVI_SOURCES or normalized in ("cam:v", "bump:v", "school:v")


def deceleration_source_presentation(source: str | None) -> tuple[str, int]:
  """Return the compact HUD label and color mode for a deceleration source.

  Color mode 2 is the normal deceleration orange, mode 3 is vehicle CAN
  navigation blue, and mode 4 is external-navigation green.
  """
  normalized = str(source or "").strip().lower()
  if not normalized:
    return "apply", 2
  if is_vehicle_navigation_source(normalized):
    return "vNAVI", 3
  if normalized in EXTERNAL_NAVI_SOURCES or normalized.endswith(":n"):
    return "NAVI", 4
  if normalized.endswith((":v", ":c")):
    return normalized, 2
  return DECELERATION_SOURCE_LABELS.get(normalized, normalized[:8]), 2
