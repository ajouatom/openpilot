from __future__ import annotations

"""Compact display-field serialization for the client-rendered Carrot Web UI."""

import math
import struct
from collections.abc import Iterable, Sequence
from typing import Any

FRAME_MAGIC = b"CVS1"
BATCH_MAGIC = b"CVB1"


XYZ_SCHEMA = (
  ("x", "u16_cm_list"),
  ("y", "i16_mm_list"),
  ("z", "i16_mm_list"),
)

MODEL_VELOCITY_SCHEMA = (
  ("x", "i16_cm_list"),
)

MODEL_LEAD_SCHEMA = (
  ("prob", "f32"),
  ("x", "u16_cm_list"),
)

RADAR_LEAD_SCHEMA = (
  ("dRel", "f32"),
  ("yRel", "f32"),
  ("vRel", "f32"),
  ("aRel", "f32"),
  ("vLead", "f32"),
  ("aLead", "f32"),
  ("dPath", "f32"),
  ("vLat", "f32"),
  ("vLeadK", "f32"),
  ("aLeadK", "f32"),
  ("fcw", "bool"),
  ("status", "bool"),
  ("aLeadTau", "f32"),
  ("modelProb", "f32"),
  ("radar", "bool"),
  ("radarTrackId", "i32"),
  ("jLead", "f32"),
  ("score", "f32"),
)

TPMS_SCHEMA = (
  ("fl", "f32"),
  ("fr", "f32"),
  ("rl", "f32"),
  ("rr", "f32"),
)


# This is deliberately a display schema, not a second cereal transport. Every
# field consumed by Carrot Vision is retained, while unused model tensors and
# Cap'n Proto pointer/layout overhead never cross the hotspot link.
SERVICE_SCHEMAS: dict[str, tuple[int, tuple[tuple[Any, ...], ...]]] = {
  "carState": (1, (
    ("vEgo", "f32"),
    ("aEgo", "f32"),
    ("vEgoCluster", "f32"),
    ("vCruiseCluster", "f32"),
    ("steeringAngleDeg", "f32"),
    ("brakeHoldActive", "bool"),
    ("softHoldActive", "i16"),
    ("carrotCruise", "i16"),
    ("gearStep", "i16"),
    ("useLaneLineSpeed", "f32"),
    ("brakeLights", "bool"),
    ("leftBlindspot", "bool"),
    ("rightBlindspot", "bool"),
    ("leftLaneLine", "i16"),
    ("rightLaneLine", "i16"),
    ("gearShifter", ("enum", ("unknown", "park", "drive", "neutral", "reverse", "sport", "low", "brake", "eco", "manumatic"))),
    ("leftBlinker", "bool"),
    ("rightBlinker", "bool"),
    ("fuelGauge", "f32"),
    ("ureaGauge", "f32"),
    ("tpms", ("struct", TPMS_SCHEMA)),
  )),
  "controlsState": (2, (
    ("enabled", "bool", ("deprecated", "enabled")),
    ("vCruiseCluster", "f32", ("deprecated", "vCruiseCluster")),
    ("activeLaneLine", "bool"),
    ("curvature", "f32"),
    ("desiredCurvature", "f32"),
    ("actualLateralAccel", "f32", ("lateralControlState", "torqueState", "actualLateralAccel")),
    ("desiredLateralAccel", "f32", ("lateralControlState", "torqueState", "desiredLateralAccel")),
    ("lateralOutput", "f32", ("lateralControlState", "torqueState", "output")),
  )),
  "deviceState": (3, (
    ("memoryUsagePercent", "i8"),
    ("freeSpacePercent", "f32"),
    ("cpuTempC", "f32_list"),
    ("deviceType", ("enum", ("unknown", "neo", "chffrAndroid", "chffrIos", "tici", "pc", "tizi", "mici"))),
  )),
  "peripheralState": (4, (
    ("voltage", "u32"),
  )),
  "carrotMan": (5, (
    ("activeCarrot", "i32"),
    ("nRoadLimitSpeed", "i32"),
    ("xSpdType", "i32"),
    ("xSpdLimit", "i32"),
    ("xSpdDist", "i32"),
    ("xSpdCountDown", "i32"),
    ("xTurnInfo", "i32"),
    ("xDistToTurn", "i32"),
    ("xTurnCountDown", "i32"),
    ("atcType", "text"),
    ("szPosRoadName", "text"),
    ("szTBTMainText", "text"),
    ("desiredSpeed", "i32"),
    ("xPosLat", "f32"),
    ("xPosLon", "f32"),
    ("xPosAngle", "f32"),
    ("xPosSpeed", "f32"),
    ("trafficState", "i32"),
    ("nGoPosDist", "i32"),
    ("nGoPosTime", "i32"),
    ("szSdiDescr", "text"),
    ("naviPaths", "text"),
    ("desiredSource", "text"),
  )),
  "selfdriveState": (6, (
    ("enabled", "bool"),
    ("personality", ("enum", ("aggressive", "standard", "relaxed", "moreRelaxed"))),
    ("alertStatus", ("enum", ("normal", "userPrompt", "critical"))),
    ("alertSize", ("enum", ("none", "small", "mid", "full"))),
    ("alertType", "text"),
    ("alertText1", "text"),
    ("alertText2", "text"),
  )),
  "gpsLocationExternal": (7, (
    ("latitude", "f64"),
    ("longitude", "f64"),
    ("speed", "f32"),
    ("bearingDeg", "f32"),
    ("bearingAccuracyDeg", "f32"),
    ("speedAccuracy", "f32"),
    ("hasFix", "bool"),
  )),
  "longitudinalPlan": (8, (
    # The page only renders the current sample. Preserve its existing list
    # shape on the wire without traversing/transmitting the full planner arrays.
    ("accels", "f32_first_list"),
    ("speeds", "f32_first_list"),
    ("jerks", "f32_first_list"),
    ("tFollow", "f32"),
    ("desiredDistance", "f32"),
    ("myDrivingMode", "i32"),
    ("xState", "i32"),
    ("trafficState", "i32"),
    ("longitudinalPlanSource", ("enum", ("cruise", "lead0", "lead1", "lead2", "e2e"))),
  )),
  "modelV2": (9, (
    ("frameId", "u32"),
    ("frameIdExtra", "u32"),
    ("position", ("struct", XYZ_SCHEMA)),
    ("velocity", ("struct", MODEL_VELOCITY_SCHEMA)),
    ("laneLines", ("struct_list", XYZ_SCHEMA)),
    ("laneLineProbs", "f32_list"),
    ("roadEdges", ("struct_list", XYZ_SCHEMA)),
    ("roadEdgeStds", "f32_list"),
    ("leadsV3", ("struct_list", MODEL_LEAD_SCHEMA)),
  )),
  "liveCalibration": (10, (
    ("calStatus", ("enum", ("uncalibrated", "calibrated", "invalid", "recalibrating"))),
    ("calCycle", "i32"),
    ("calPerc", "i8"),
    ("validBlocks", "i32"),
    ("rpyCalib", "f32_list"),
    ("height", "f32_list"),
  )),
  "roadCameraState": (11, (
    ("frameId", "u32"),
    ("sensor", ("enum", ("unknown", "ar0231", "ox03c10", "os04c10"))),
  )),
  "lateralPlan": (12, (
    ("useLaneLines", "bool"),
    ("latDebugText", "text"),
    ("position", ("struct", XYZ_SCHEMA)),
    ("distances", "f32_list"),
    ("laneChangeState", ("enum", ("off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing"))),
    ("laneChangeDirection", ("enum", ("none", "left", "right"))),
  )),
  "radarState": (13, (
    ("leadOne", ("struct", RADAR_LEAD_SCHEMA)),
    ("leadTwo", ("struct", RADAR_LEAD_SCHEMA)),
    ("leadRight", ("struct", RADAR_LEAD_SCHEMA)),
    ("leadLeft", ("struct", RADAR_LEAD_SCHEMA)),
    ("leadsLeft", ("struct_list", RADAR_LEAD_SCHEMA)),
    ("leadsCenter", ("struct_list", RADAR_LEAD_SCHEMA)),
    ("leadsRight", ("struct_list", RADAR_LEAD_SCHEMA)),
  )),
  "carControl": (14, (
    ("latActive", "bool"),
    ("longActive", "bool"),
    ("actuators", ("struct", (
      ("steeringAngleDeg", "f32"),
      ("accel", "f32"),
      ("curvature", "f32"),
    ))),
  )),
  "liveDelay": (15, (
    ("lateralDelay", "f32"),
    ("calPerc", "i8"),
  )),
  "liveTorqueParameters": (16, (
    ("liveValid", "bool"),
    ("latAccelFactorFiltered", "f32"),
    ("frictionCoefficientFiltered", "f32"),
    ("calPerc", "i8"),
  )),
  "liveParameters": (17, (
    ("angleOffsetDeg", "f32"),
    ("steerRatio", "f32"),
  )),
}

CARROT_STATE_SERVICES = tuple(SERVICE_SCHEMAS.keys())

# One display cadence shared by live WebSocket delivery and recorded replay.
# Replay therefore presents the same set of samples the live page would retain
# instead of flooding the browser with every message stored in rlog.
COMPACT_SERVICE_INTERVALS = {
  "modelV2": 0.05,
  "carState": 0.03,
  "controlsState": 0.03,
  "longitudinalPlan": 0.05,
  "carControl": 0.03,
  "radarState": 0.05,
  "lateralPlan": 0.05,
  "carrotMan": 0.1,
  "roadCameraState": 0.25,
  "deviceState": 0.5,
  "peripheralState": 0.5,
  "gpsLocationExternal": 0.5,
  "selfdriveState": 0.2,
  "liveCalibration": 0.25,
  "liveParameters": 0.25,
  "liveTorqueParameters": 0.25,
  "liveDelay": 0.25,
}
COMPACT_SERVICE_INTERVAL_DEFAULT = 0.05
COMPACT_BATCH_WINDOW_SECONDS = 0.012


def compact_service_interval(service: str) -> float:
  return COMPACT_SERVICE_INTERVALS.get(service, COMPACT_SERVICE_INTERVAL_DEFAULT)


def _field(value: Any, name: str) -> Any:
  try:
    return getattr(value, name)
  except Exception:
    return None


def _field_path(value: Any, path: Sequence[str]) -> Any:
  for name in path:
    value = _field(value, name)
    if value is None:
      break
  return value


def _number(value: Any, default: float = 0.0) -> float:
  try:
    converted = float(value)
    return converted if math.isfinite(converted) else default
  except Exception:
    return default


def _integer(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return default


def _enum_index(value: Any, names: Sequence[str]) -> int:
  numeric = _integer(value, -1)
  if 0 <= numeric < len(names):
    return numeric
  normalized = str(value or "").split(".")[-1].replace("_", "").lower()
  for index, name in enumerate(names):
    if normalized == name.replace("_", "").lower():
      return index
  return 0


def _text(value: Any) -> bytes:
  raw = str(value or "").encode("utf-8", errors="replace")
  return raw[:0xffff]


def _float_list(value: Any) -> list[float]:
  if value is None or isinstance(value, (str, bytes, bytearray)):
    return []
  try:
    iterable: Iterable[Any] = value
    return [_number(item) for item in iterable][:0xffff]
  except Exception:
    return []


def _quantized_list(value: Any, scale: float, minimum: int, maximum: int) -> list[int]:
  return [max(minimum, min(maximum, round(item * scale))) for item in _float_list(value)]


def _pack_field(out: bytearray, value: Any, spec: Any) -> None:
  if spec == "bool":
    out.extend(struct.pack("<B", 1 if bool(value) else 0))
  elif spec == "i8":
    out.extend(struct.pack("<b", max(-128, min(127, _integer(value)))))
  elif spec == "u8":
    out.extend(struct.pack("<B", max(0, min(255, _integer(value)))))
  elif spec == "i16":
    out.extend(struct.pack("<h", max(-32768, min(32767, _integer(value)))))
  elif spec == "u16":
    out.extend(struct.pack("<H", max(0, min(65535, _integer(value)))))
  elif spec == "i32":
    out.extend(struct.pack("<i", max(-2147483648, min(2147483647, _integer(value)))))
  elif spec == "u32":
    out.extend(struct.pack("<I", max(0, min(0xffffffff, _integer(value)))))
  elif spec == "f32":
    out.extend(struct.pack("<f", _number(value)))
  elif spec == "f64":
    out.extend(struct.pack("<d", _number(value)))
  elif spec == "text":
    encoded = _text(value)
    out.extend(struct.pack("<H", len(encoded)))
    out.extend(encoded)
  elif spec == "f32_list":
    values = _float_list(value)
    out.extend(struct.pack("<H", len(values)))
    if values:
      out.extend(struct.pack(f"<{len(values)}f", *values))
  elif spec == "f32_first_list":
    try:
      values = [_number(value[0])] if value is not None and len(value) > 0 else []
    except Exception:
      values = []
    out.extend(struct.pack("<H", len(values)))
    if values:
      out.extend(struct.pack("<f", values[0]))
  elif spec in ("u16_cm_list", "i16_cm_list", "i16_mm_list"):
    if spec == "u16_cm_list":
      values = _quantized_list(value, 100.0, 0, 0xffff)
      format_char = "H"
    elif spec == "i16_cm_list":
      values = _quantized_list(value, 100.0, -32768, 32767)
      format_char = "h"
    else:
      values = _quantized_list(value, 1000.0, -32768, 32767)
      format_char = "h"
    out.extend(struct.pack("<H", len(values)))
    if values:
      out.extend(struct.pack(f"<{len(values)}{format_char}", *values))
  elif isinstance(spec, tuple) and spec[0] == "enum":
    out.extend(struct.pack("<B", _enum_index(value, spec[1])))
  elif isinstance(spec, tuple) and spec[0] == "struct":
    _pack_schema(out, value, spec[1])
  elif isinstance(spec, tuple) and spec[0] == "struct_list":
    try:
      values = list(value or [])[:255]
    except Exception:
      values = []
    out.extend(struct.pack("<B", len(values)))
    for item in values:
      _pack_schema(out, item, spec[1])
  else:
    raise ValueError(f"unsupported Carrot Vision field spec: {spec!r}")


def _pack_schema(out: bytearray, value: Any, schema: Sequence[tuple[Any, ...]]) -> None:
  for entry in schema:
    name, spec = entry[:2]
    source_path = entry[2] if len(entry) > 2 else (name,)
    _pack_field(out, _field_path(value, source_path), spec)


def encode_carrot_state_frame(service: str, value: Any, sequence: int) -> bytes:
  service_id, schema = SERVICE_SCHEMAS[service]
  out = bytearray(struct.pack("<4sBBH", FRAME_MAGIC, service_id, 0, sequence & 0xffff))
  _pack_schema(out, value, schema)
  return bytes(out)


def encode_carrot_state_batch(frames: Sequence[bytes]) -> bytes:
  selected = tuple(frame for frame in frames if frame)
  if len(selected) > 0xffff:
    raise ValueError("too many compact state frames")
  out = bytearray(struct.pack("<4sH", BATCH_MAGIC, len(selected)))
  for frame in selected:
    out.extend(struct.pack("<I", len(frame)))
    out.extend(frame)
  return bytes(out)
