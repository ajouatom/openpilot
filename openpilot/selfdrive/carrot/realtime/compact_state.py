from __future__ import annotations

"""Compact display-field serialization for the client-rendered Carrot Web UI."""

import math
import struct
from collections.abc import Iterable, Sequence
from typing import Any

FRAME_MAGIC = b"CVS1"
BATCH_MAGIC = b"CVB1"


# 경로 폴리라인 전송 상한. 앵커 tangent 계산에는 근거리 구간이면 충분하고,
# 전 구간을 보내면 2Hz 라도 대역폭이 커진다.
ROUTE_POLYLINE_LIMIT = 64

XYZ_SCHEMA = (
  ("x", "u16_cm_list"),
  ("y", "i16_mm_list"),
  ("z", "i16_mm_list"),
)

MODEL_VELOCITY_SCHEMA = (
  ("x", "i16_cm_list"),
)

# Only sample [0] of each leadsV3 track is drawn, so y/v ride along as
# first-element lists instead of the full 6-entry prediction arrays. Without
# them the browser cannot place a lead laterally (schema.js drops any lead
# whose y is missing) and Drive Insights renders no lead distance at all.
MODEL_LEAD_SCHEMA = (
  ("prob", "f32"),
  ("x", "u16_cm_list"),
  ("y", "f32_first_list"),
  ("v", "f32_first_list"),
)

# liveTracks (Car.RadarData) points. This is the only source that tells the
# browser which radar saw a target, so Drive Insights needs it to separate
# front / SCC / corner returns instead of showing SCC leads alone.
# livePose 의 XYZMeasurement. std 와 valid 로 품질 게이트를 건다.
XYZ_MEASUREMENT_SCHEMA = (
  ("x", "f32"),
  ("y", "f32"),
  ("z", "f32"),
  ("xStd", "f32"),
  ("yStd", "f32"),
  ("zStd", "f32"),
  ("valid", "bool"),
)

# ── carrotNavi (TMap) 압축 서브셋 ──────────────────────────────────
# 전체 CarrotNaviState 를 보내지 않는다. AR 앵커가 "무엇을 어디서 안내할지"
# 정하는 데 필요한 필드만 고른다. route.polyline은 Phase 5의 실제 접선 방향,
# laneCurrent는 권장 차선군 표시를 위해 제한된 서브셋만 보낸다.
NAVI_GUIDANCE_SCHEMA = (
  ("present", "bool", ("meta", "present")),
  ("distanceM", "i32"),
  ("timeSec", "i32"),
  ("turnType", "i32"),
  ("roadName", "text"),
  ("mainText", "text"),
  ("pointValid", "bool"),
  ("latitude", "f64"),
  ("longitude", "f64"),
)

NAVI_VEHICLE_SCHEMA = (
  ("present", "bool", ("meta", "present")),
  ("latitude", "f64"),
  ("longitude", "f64"),
  ("headingDeg", "f32"),
  ("speedKph", "f32"),
  ("roadName", "text"),
)

NAVI_LANE_SCHEMA = (
  ("present", "bool", ("meta", "present")),
  ("count", "i16"),
  ("distanceM", "i32"),
  ("visible", "bool"),
  ("available", "i16_list"),
)

NAVI_SPEED_SCHEMA = (
  ("roadLimitValid", "bool"),
  ("roadLimitKph", "i16"),
  ("sdiPresent", "bool"),
  ("sdiType", "i32"),
  ("sdiDistanceM", "i32"),
  ("sdiSpeedLimitKph", "i16"),
  ("sdiSectionType", "i32"),
  ("sdiBlockType", "i32"),
  ("sdiBlockSpeedKph", "i16"),
  ("sdiBlockDistanceM", "i32"),
  ("secondarySdiPresent", "bool"),
  ("secondarySdiType", "i32"),
  ("secondarySdiDistanceM", "i32"),
  ("secondarySdiSpeedLimitKph", "i16"),
  ("secondarySdiSectionType", "i32"),
  ("secondarySdiBlockType", "i32"),
  ("secondarySdiBlockSpeedKph", "i16"),
  ("secondarySdiBlockDistanceM", "i32"),
  ("sectionPresent", "bool"),
  ("sectionActive", "bool"),
  ("sectionSpeedLimitKph", "i16"),
  ("sectionAverageKph", "f32"),
  ("sectionOverallAverageKph", "f32"),
  ("sectionRemainingDistanceM", "f32"),
  ("sectionRemainingTimeSec", "i32"),
  ("sectionProgress", "f32"),
  ("sectionSuspended", "bool"),
  ("sectionOffRoute", "bool"),
)

NAVI_SIGNAL_SCHEMA = (
  ("visible", "bool"),
  ("distanceM", "i32"),
  ("redValid", "bool"), ("redOn", "bool"), ("redRemainSec", "i16"),
  ("leftValid", "bool"), ("leftOn", "bool"), ("leftRemainSec", "i16"),
  ("greenValid", "bool"), ("greenOn", "bool"), ("greenRemainSec", "i16"),
  ("rightValid", "bool"), ("rightOn", "bool"), ("rightRemainSec", "i16"),
  ("uturnValid", "bool"), ("uturnOn", "bool"), ("uturnRemainSec", "i16"),
  ("uiCounterValid", "bool"), ("uiCounterRemainSec", "i16"),
)

NAVI_CROSSROAD_SCHEMA = (
  ("visible", "bool"),
  ("distanceM", "i32"),
  ("imageCode", "i32"),
)

NAVI_ROUTE_SCHEMA = (
  ("present", "bool", ("meta", "present")),
  ("remainingDistanceM", "i32"),
  ("remainingTimeSec", "i32"),
  ("movedDistanceM", "i32"),
  ("totalDistanceM", "i32"),
  ("polyline", "coord_list"),
)

NAVI_STATUS_SCHEMA = (
  ("guidanceActive", "bool"),
  ("offRoute", "bool"),
  ("routePresent", "bool"),
)

RADAR_POINT_SCHEMA = (
  ("trackId", "u32"),
  ("dRel", "f32"),
  ("yRel", "f32"),
  ("vRel", "f32"),
  ("measured", "bool"),
  ("radarSource", ("enum", ("frontRadar", "scc", "corner235", "corner180", "corner430"))),
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
    # Appended (schema-evolution: keep at the end so shorter recorded frames stay
    # decodable). Cluster parity: green EV telltale (car.capnp evModeActive@85 /
    # evModeValid@86).
    ("evModeValid", "bool"),
    ("evModeActive", "bool"),
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
    # AR 품질 게이트용. 정확도가 나쁘면 월드 앵커를 도로 상대 모드로 강등한다.
    # 고도는 목적지/경사 표현의 근거이고, unixTimestamp 는 stale 판정에 쓴다.
    ("altitude", "f64"),
    ("horizontalAccuracy", "f32"),
    ("verticalAccuracy", "f32"),
    ("unixTimestampMillis", "f64"),
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
    # Appended (kept at the end for schema evolution). Cluster parity: eco cruise
    # override telltale (log.capnp longitudinalPlan.cruiseTarget@44).
    ("cruiseTarget", "f32"),
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
    # AR 품질 게이트용. 차선 신뢰도가 낮거나 프레임이 밀리면 앵커를 숨긴다.
    ("laneLineStds", "f32_list"),
    ("frameAge", "i32"),
    ("frameDropPerc", "f32"),
    ("modelExecutionTime", "f32"),
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
    # AR 정합의 최우선 필드. 이게 없으면 표시 중인 영상 프레임과 model /
    # odometry 를 "시간" 기준으로 묶을 수 없다(frameId 만으로는 부족).
    ("timestampEof", "u64"),
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
    # Second-adjacent lanes and cut-in candidates. Usually empty (1 byte each),
    # so they cost nothing until targets actually appear there.
    ("leadsLeft2", ("struct_list", RADAR_LEAD_SCHEMA)),
    ("leadsRight2", ("struct_list", RADAR_LEAD_SCHEMA)),
    ("leadsCutIn", ("struct_list", RADAR_LEAD_SCHEMA)),
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
  "liveTracks": (18, (
    ("points", ("struct_list", RADAR_POINT_SCHEMA)),
  )),
  # ── AR 앵커 입력 (ar 채널 임대 시에만 구독) ─────────────────────────
  # 앵커를 프레임 사이에 유지하는 데 쓴다. std 가 나쁘거나 frame gap 이
  # 벌어지면 적분을 멈추고 앵커를 숨겨야 하므로 std 도 함께 보낸다.
  "cameraOdometry": (19, (
    ("frameId", "u32"),
    ("timestampEof", "u64"),
    ("trans", "f32_list"),        # m/s, device frame
    ("rot", "f32_list"),          # rad/s
    ("transStd", "f32_list"),
    ("rotStd", "f32_list"),
  )),
  # 자세/유효성. 글로벌 지도 위치가 아니라 device pose 측정값이다.
  # carrotNavi. cereal 은 멀티캐스트라 여기서 따로 구독해도 지도 뷰의
  # /ws/carrot_navi/* 단일 viewer ownership 을 건드리지 않는다.
  "carrotNavi": (21, (
    ("schemaVersion", "u16"),
    ("generation", "u64"),
    ("sessionId", "text"),
    ("publishMonoTimeNanos", "u64"),
    ("connected", "bool"),
    ("vehicle", ("struct", NAVI_VEHICLE_SCHEMA)),
    ("guidanceCurrent", ("struct", NAVI_GUIDANCE_SCHEMA)),
    ("guidanceNext", ("struct", NAVI_GUIDANCE_SCHEMA)),
    ("laneCurrent", ("struct", NAVI_LANE_SCHEMA)),
    ("laneAhead", ("struct_list", NAVI_LANE_SCHEMA)),
    ("speed", ("struct", NAVI_SPEED_SCHEMA)),
    ("trafficSignal", ("struct", NAVI_SIGNAL_SCHEMA)),
    ("crossroad", ("struct", NAVI_CROSSROAD_SCHEMA)),
    ("route", ("struct", NAVI_ROUTE_SCHEMA)),
    ("navigationStatus", ("struct", NAVI_STATUS_SCHEMA)),
  )),
  "livePose": (20, (
    ("orientationNED", ("struct", XYZ_MEASUREMENT_SCHEMA)),
    ("velocityDevice", ("struct", XYZ_MEASUREMENT_SCHEMA)),
    ("accelerationDevice", ("struct", XYZ_MEASUREMENT_SCHEMA)),
    ("angularVelocityDevice", ("struct", XYZ_MEASUREMENT_SCHEMA)),
    ("inputsOK", "bool"),
    ("posenetOK", "bool"),
    ("sensorsOK", "bool"),
    ("timestamp", "u64"),
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
  # A full track list is far larger than a lead struct, and it only feeds the
  # Drive Insights forward view, so it runs at half the radarState cadence.
  "liveTracks": 0.1,
  # AR anchor inputs. Full 20Hz - anchor hold between frames needs every sample.
  # Only subscribed while an "ar" lease is held, so the cost is opt-in.
  "cameraOdometry": 0.05,
  "livePose": 0.05,
  # Navi 는 원래 2Hz 발행이라 그대로 따른다.
  "carrotNavi": 0.5,
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
  elif spec == "u64":
    # 카메라/pose 의 EOF 타임스탬프는 나노초라 u32 로는 못 담는다.
    out.extend(struct.pack("<Q", max(0, min(0xffffffffffffffff, _integer(value)))))
  elif spec == "f32":
    out.extend(struct.pack("<f", _number(value)))
  elif spec == "f64":
    out.extend(struct.pack("<d", _number(value)))
  elif spec == "text":
    encoded = _text(value)
    out.extend(struct.pack("<H", len(encoded)))
    out.extend(encoded)
  elif spec == "coord_list":
    # 경로 폴리라인. f64 쌍을 그대로 보내면 좌표당 16바이트라 2Hz 라도 부담이
    # 크다. 첫 점만 f64 앵커로 두고 나머지는 f32 델타로 보낸다(좌표당 8바이트).
    # 로컬 구간에서 델타는 작아 f32 로도 1e-8도 이하 정밀도가 남는다.
    try:
      items = list(value or [])[:ROUTE_POLYLINE_LIMIT]
    except Exception:
      items = []
    coords = []
    for c in items:
      lat = _number(_field(c, "latitude"), math.nan)
      lon = _number(_field(c, "longitude"), math.nan)
      if math.isfinite(lat) and math.isfinite(lon):
        coords.append((lat, lon))
    out.extend(struct.pack("<B", len(coords)))
    if coords:
      lat0, lon0 = coords[0]
      out.extend(struct.pack("<dd", lat0, lon0))
      for lat, lon in coords[1:]:
        out.extend(struct.pack("<ff", lat - lat0, lon - lon0))
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
  elif spec == "i16_list":
    try:
      values = [max(-32768, min(32767, _integer(item))) for item in list(value or [])[:0xffff]]
    except Exception:
      values = []
    out.extend(struct.pack("<H", len(values)))
    if values:
      out.extend(struct.pack(f"<{len(values)}h", *values))
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
