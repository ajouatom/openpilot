from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NaviGuidanceControl:
  present: bool
  sequence: int
  distance_m: int = 0
  turn_type: int = -1
  main_text: str = ""
  near_direction: str = ""
  far_direction: str = ""


@dataclass(frozen=True)
class NaviSpeedControl:
  present: bool
  sequence: int
  road_limit_kph: int | None = None
  sdi_present: bool = False
  sdi_type: int = -1
  sdi_distance_m: int = 0
  sdi_speed_limit_kph: int = 0
  section_active: bool = False
  section_speed_limit_kph: int = 0
  section_remaining_distance_m: int = 0


@dataclass(frozen=True)
class CarrotNaviControl:
  session_id: str
  speed: NaviSpeedControl
  current: NaviGuidanceControl
  next: NaviGuidanceControl
  off_route: bool = False
  road_category: int | None = None


def _get(obj: Any, name: str, default: Any = None) -> Any:
  if isinstance(obj, dict):
    return obj.get(name, default)
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _int(obj: Any, name: str, default: int = 0) -> int:
  try:
    return int(_get(obj, name, default))
  except (TypeError, ValueError, OverflowError):
    return default


def _float(obj: Any, name: str, default: float = 0.0) -> float:
  try:
    value = float(_get(obj, name, default))
    return value if value == value else default
  except (TypeError, ValueError, OverflowError):
    return default


def _text(obj: Any, name: str) -> str:
  try:
    return str(_get(obj, name, "") or "")
  except Exception:
    return ""


def _meta(item: Any) -> tuple[bool, int]:
  meta = _get(item, "meta")
  return bool(_get(meta, "present", False)), max(0, _int(meta, "sequence"))


def _guidance(item: Any, enabled: bool) -> NaviGuidanceControl:
  present, sequence = _meta(item)
  present = present and enabled
  return NaviGuidanceControl(
    present=present,
    sequence=sequence,
    distance_m=max(0, _int(item, "distanceM")) if present else 0,
    turn_type=_int(item, "turnType", -1) if present else -1,
    main_text=_text(item, "mainText") if present else "",
    near_direction=_text(item, "nearDirection") if present else "",
    far_direction=_text(item, "farDirection") if present else "",
  )


def parse_carrot_navi_control(data: Any) -> CarrotNaviControl | None:
  if _int(data, "schemaVersion") != 1 or not bool(_get(data, "connected", False)):
    return None

  status = _get(data, "navigationStatus")
  status_present, _ = _meta(status)
  off_route = status_present and bool(_get(status, "offRoute", False))

  speed_item = _get(data, "speed")
  speed_present, speed_sequence = _meta(speed_item)
  road_limit_valid = speed_present and bool(_get(speed_item, "roadLimitValid", False))
  road_limit_kph = _int(speed_item, "roadLimitKph") if road_limit_valid else None
  if road_limit_kph is not None and not 0 < road_limit_kph <= 200:
    road_limit_kph = None

  sdi_present = speed_present and bool(_get(speed_item, "sdiPresent", False)) and not off_route
  section_present = speed_present and bool(_get(speed_item, "sectionPresent", False))
  section_active = (
    section_present
    and bool(_get(speed_item, "sectionActive", False))
    and not bool(_get(speed_item, "sectionSuspended", False))
    and not bool(_get(speed_item, "sectionOffRoute", False))
    and not off_route
  )
  section_limit = max(0, _int(speed_item, "sectionSpeedLimitKph")) if section_active else 0
  section_distance = max(0, round(_float(speed_item, "sectionRemainingDistanceM"))) if section_active else 0
  section_active = section_active and section_limit > 0

  speed = NaviSpeedControl(
    present=speed_present,
    sequence=speed_sequence,
    road_limit_kph=road_limit_kph,
    sdi_present=sdi_present,
    sdi_type=_int(speed_item, "sdiType", -1) if sdi_present else -1,
    sdi_distance_m=max(0, _int(speed_item, "sdiDistanceM")) if sdi_present else 0,
    sdi_speed_limit_kph=max(0, _int(speed_item, "sdiSpeedLimitKph")) if sdi_present else 0,
    section_active=section_active,
    section_speed_limit_kph=section_limit if section_active else 0,
    section_remaining_distance_m=section_distance if section_active else 0,
  )

  lane = _get(data, "laneCurrent")
  lane_present, _ = _meta(lane)
  road_category = _int(lane, "roadCategory") if lane_present else None

  return CarrotNaviControl(
    session_id=_text(data, "sessionId"),
    speed=speed,
    current=_guidance(_get(data, "guidanceCurrent"), not off_route),
    next=_guidance(_get(data, "guidanceNext"), not off_route),
    off_route=off_route,
    road_category=road_category,
  )
