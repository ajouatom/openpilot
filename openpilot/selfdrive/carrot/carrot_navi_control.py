from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


MAX_ROUTE_POINTS = 256
MAX_ROAD_LIMIT_KPH = 200
ROAD_LIMIT_STEP_KPH = 10


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
class NaviVehicleControl:
  present: bool
  sequence: int
  latitude: float = 0.0
  longitude: float = 0.0
  heading_deg: float = 0.0
  speed_kph: float = 0.0
  road_name: str = ""


@dataclass(frozen=True)
class NaviSpeedControl:
  present: bool
  sequence: int
  road_limit_kph: int | None = None
  sdi_present: bool = False
  sdi_type: int = -1
  sdi_distance_m: int = 0
  sdi_speed_limit_kph: int = 0
  sdi_section_type: int = -1
  sdi_block_type: int = -1
  sdi_block_speed_kph: int = 0
  sdi_block_distance_m: int = 0
  secondary_sdi_present: bool = False
  secondary_sdi_type: int = -1
  secondary_sdi_distance_m: int = 0
  secondary_sdi_speed_limit_kph: int = 0
  secondary_sdi_section_type: int = -1
  secondary_sdi_block_type: int = -1
  secondary_sdi_block_speed_kph: int = 0
  secondary_sdi_block_distance_m: int = 0
  section_active: bool = False
  section_speed_limit_kph: int = 0
  section_remaining_distance_m: int = 0


@dataclass(frozen=True)
class NaviRouteControl:
  present: bool
  sequence: int
  remaining_distance_m: int = 0
  remaining_time_sec: int = 0
  polyline: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class NaviTrafficControl:
  present: bool
  sequence: int
  visible: bool = False
  distance_m: int = 0
  source: str = ""
  lamp: str = ""
  remain_sec: int = 0


@dataclass(frozen=True)
class CarrotNaviControl:
  session_id: str
  vehicle: NaviVehicleControl
  speed: NaviSpeedControl
  current: NaviGuidanceControl
  next: NaviGuidanceControl
  route: NaviRouteControl
  traffic: NaviTrafficControl
  off_route: bool = False
  guidance_active: bool = False
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
    return value if math.isfinite(value) else default
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


def _valid_road_limit_kph(item: Any) -> int | None:
  if not bool(_get(item, "roadLimitValid", False)):
    return None
  road_limit = _int(item, "roadLimitKph")
  return road_limit if (
    0 < road_limit <= MAX_ROAD_LIMIT_KPH
    and road_limit % ROAD_LIMIT_STEP_KPH == 0
  ) else None


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


def _vehicle(data: Any) -> NaviVehicleControl:
  item = _get(data, "vehicle")
  present, sequence = _meta(item)
  latitude = _float(item, "latitude", math.nan)
  longitude = _float(item, "longitude", math.nan)
  present = present and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0
  return NaviVehicleControl(
    present=present,
    sequence=sequence,
    latitude=latitude if present else 0.0,
    longitude=longitude if present else 0.0,
    heading_deg=_float(item, "headingDeg") % 360.0 if present else 0.0,
    speed_kph=max(0.0, _float(item, "speedKph")) if present else 0.0,
    road_name=_text(item, "roadName") if present else "",
  )


def _route(data: Any) -> NaviRouteControl:
  item = _get(data, "route")
  present, sequence = _meta(item)
  points: list[tuple[float, float]] = []
  if present:
    try:
      values = tuple(_get(item, "polyline", ()) or ())[:MAX_ROUTE_POINTS]
    except Exception:
      values = ()
    for point in values:
      latitude = _float(point, "latitude", math.nan)
      longitude = _float(point, "longitude", math.nan)
      if -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0:
        points.append((latitude, longitude))
  return NaviRouteControl(
    present=present,
    sequence=sequence,
    remaining_distance_m=max(0, _int(item, "remainingDistanceM")) if present else 0,
    remaining_time_sec=max(0, _int(item, "remainingTimeSec")) if present else 0,
    polyline=tuple(points),
  )


def _traffic(data: Any) -> NaviTrafficControl:
  item = _get(data, "trafficSignal")
  present, sequence = _meta(item)
  visible = present and bool(_get(item, "visible", False))
  lamp = ""
  remain_sec = 0
  ui_remain = max(0, _int(item, "uiCounterRemainSec")) if bool(_get(item, "uiCounterValid", False)) else 0
  for name, prefix in (("red", "red"), ("left", "left"), ("green", "green"),
                       ("right", "right"), ("uturn", "uturn")):
    if visible and bool(_get(item, f"{prefix}Valid", False)) and bool(_get(item, f"{prefix}On", False)):
      lamp = name
      remain_sec = max(0, _int(item, f"{prefix}RemainSec")) or ui_remain
      break
  return NaviTrafficControl(
    present=present,
    sequence=sequence,
    visible=visible,
    distance_m=max(0, _int(item, "distanceM")) if present else 0,
    source=_text(item, "source") if present else "",
    lamp=lamp,
    remain_sec=remain_sec,
  )


def parse_carrot_navi_control(data: Any) -> CarrotNaviControl | None:
  if _int(data, "schemaVersion") != 1 or not bool(_get(data, "connected", False)):
    return None

  status = _get(data, "navigationStatus")
  status_present, _ = _meta(status)
  off_route = status_present and bool(_get(status, "offRoute", False))
  guidance_active = status_present and bool(_get(status, "guidanceActive", False))

  speed_item = _get(data, "speed")
  speed_present, speed_sequence = _meta(speed_item)
  road_limit_kph = _valid_road_limit_kph(speed_item) if speed_present else None

  sdi_present = speed_present and bool(_get(speed_item, "sdiPresent", False)) and not off_route
  secondary_sdi_present = (
    speed_present and bool(_get(speed_item, "secondarySdiPresent", False)) and not off_route
  )
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
    sdi_section_type=_int(speed_item, "sdiSectionType", -1) if sdi_present else -1,
    sdi_block_type=_int(speed_item, "sdiBlockType", -1) if sdi_present else -1,
    sdi_block_speed_kph=max(0, _int(speed_item, "sdiBlockSpeedKph")) if sdi_present else 0,
    sdi_block_distance_m=max(0, _int(speed_item, "sdiBlockDistanceM")) if sdi_present else 0,
    secondary_sdi_present=secondary_sdi_present,
    secondary_sdi_type=_int(speed_item, "secondarySdiType", -1) if secondary_sdi_present else -1,
    secondary_sdi_distance_m=max(0, _int(speed_item, "secondarySdiDistanceM")) if secondary_sdi_present else 0,
    secondary_sdi_speed_limit_kph=max(0, _int(speed_item, "secondarySdiSpeedLimitKph")) if secondary_sdi_present else 0,
    secondary_sdi_section_type=_int(speed_item, "secondarySdiSectionType", -1) if secondary_sdi_present else -1,
    secondary_sdi_block_type=_int(speed_item, "secondarySdiBlockType", -1) if secondary_sdi_present else -1,
    secondary_sdi_block_speed_kph=(
      max(0, _int(speed_item, "secondarySdiBlockSpeedKph")) if secondary_sdi_present else 0
    ),
    secondary_sdi_block_distance_m=(
      max(0, _int(speed_item, "secondarySdiBlockDistanceM")) if secondary_sdi_present else 0
    ),
    section_active=section_active,
    section_speed_limit_kph=section_limit if section_active else 0,
    section_remaining_distance_m=section_distance if section_active else 0,
  )

  lane = _get(data, "laneCurrent")
  lane_present, _ = _meta(lane)
  road_category = _int(lane, "roadCategory") if lane_present else None

  return CarrotNaviControl(
    session_id=_text(data, "sessionId"),
    vehicle=_vehicle(data),
    speed=speed,
    current=_guidance(_get(data, "guidanceCurrent"), not off_route),
    next=_guidance(_get(data, "guidanceNext"), not off_route),
    route=_route(data),
    traffic=_traffic(data),
    off_route=off_route,
    guidance_active=guidance_active,
    road_category=road_category,
  )
