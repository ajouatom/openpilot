#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
  from openpilot.selfdrive.carrot.carrot_navi import CarrotNaviReceiver


SCHEMA_VERSION = 1
PUBLISH_HEARTBEAT_SECONDS = 0.5
PUBLISH_COALESCE_SECONDS = 0.05
MEDIA_REQUEST_POLL_SECONDS = 0.25
MAX_AHEAD_LANES = 8
MAX_LANE_VALUES = 16
MAX_ROUTE_POINTS = 256
WEB_BOOTSTRAP_KIND = "web_render"
WEB_BOOTSTRAP_IMAGE_KIND = "web_image"


def _is_web_media_stream(kind: str, name: str) -> bool:
  return kind == "image" or (kind == "render" and name == "map_main")


def _dict(value: Any) -> dict[str, Any]:
  return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
  return value if isinstance(value, list) else []


def _finite(value: Any, default: float = 0.0,
            minimum: float | None = None, maximum: float | None = None) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return default
  if not math.isfinite(parsed):
    return default
  if minimum is not None:
    parsed = max(minimum, parsed)
  if maximum is not None:
    parsed = min(maximum, parsed)
  return parsed


def _integer(value: Any, default: int = 0,
             minimum: int | None = None, maximum: int | None = None) -> int:
  parsed = int(_finite(value, float(default)))
  if minimum is not None:
    parsed = max(minimum, parsed)
  if maximum is not None:
    parsed = min(maximum, parsed)
  return parsed


def _text(value: Any, maximum: int) -> str:
  return str(value or "")[:maximum]


def _int16_list(value: Any) -> list[int]:
  return [
    _integer(item, minimum=-32768, maximum=32767)
    for item in _list(value)[:MAX_LANE_VALUES]
  ]


def _record(snapshot: dict[str, Any], name: str) -> dict[str, Any]:
  return _dict(_dict(snapshot.get("items")).get(name))


def _meta(record: dict[str, Any]) -> dict[str, Any]:
  return {
    "present": bool(record.get("present", False)),
    "sequence": _integer(record.get("sequence"), minimum=0),
    "sourceTimestampMillis": _integer(record.get("source_timestamp_ms"), minimum=0),
    "receivedMonoTimeNanos": _integer(record.get("received_mono_ns"), minimum=0),
  }


def _guidance(snapshot: dict[str, Any], name: str) -> dict[str, Any]:
  record = _record(snapshot, name)
  value = _dict(record.get("value")) if record.get("present") else {}
  point = _dict(value.get("point"))
  point_valid = point.get("lat") is not None and point.get("lon") is not None
  return {
    "meta": _meta(record),
    "distanceM": _integer(value.get("distance_m"), minimum=0, maximum=2_000_000),
    "timeSec": _integer(value.get("time_sec"), minimum=0, maximum=604_800),
    "turnType": _integer(value.get("turn_type"), default=-1, minimum=-1, maximum=100_000),
    "roadName": _text(value.get("road_name"), 96),
    "mainText": _text(value.get("main_text"), 160),
    "nearDirection": _text(value.get("near_direction"), 96),
    "midDirection": _text(value.get("mid_direction"), 96),
    "farDirection": _text(value.get("far_direction"), 96),
    "pointValid": point_valid,
    "latitude": _finite(point.get("lat"), minimum=-90.0, maximum=90.0),
    "longitude": _finite(point.get("lon"), minimum=-180.0, maximum=180.0),
  }


def _lane(record: dict[str, Any], value: Any) -> dict[str, Any]:
  lane = _dict(value)
  return {
    "meta": _meta(record),
    "count": _integer(lane.get("count"), minimum=0, maximum=16),
    "distanceM": _integer(lane.get("distance_m"), minimum=0, maximum=2_000_000),
    "visible": bool(lane.get("visible", True)),
    "lanePlay": bool(lane.get("lane_play", False)),
    "currentLane": _integer(lane.get("current_lane"), default=-1, minimum=-1, maximum=16),
    "turnCode": _integer(lane.get("turn_code"), default=-1, minimum=-1, maximum=100_000),
    "turnInfo": _int16_list(lane.get("turn_info")),
    "etcInfo": _int16_list(lane.get("etc_info")),
    "available": _int16_list(lane.get("available")),
    "guideLineColor": _integer(lane.get("guide_line_color"), minimum=-32768, maximum=32767),
    "roadCategory": _integer(lane.get("road_category"), minimum=-32768, maximum=32767),
    "voiceCode": _integer(lane.get("voice_code"), minimum=-32768, maximum=32767),
  }


def _signal_light(value: Any) -> tuple[bool, bool, int]:
  light = _dict(value)
  if not light:
    return False, False, 0
  return True, bool(light.get("on", False)), _integer(light.get("remain_sec"), minimum=0, maximum=999)


def build_carrot_navi_payload(snapshot: dict[str, Any], publish_mono_ns: int | None = None) -> dict[str, Any]:
  vehicle_record = _record(snapshot, "vehicle")
  vehicle = _dict(vehicle_record.get("value")) if vehicle_record.get("present") else {}

  lane_record = _record(snapshot, "lane_current")
  lane_value = lane_record.get("value") if lane_record.get("present") else None
  ahead_record = _record(snapshot, "lane_ahead")
  ahead_values = _list(ahead_record.get("value")) if ahead_record.get("present") else []

  speed_record = _record(snapshot, "speed")
  speed = _dict(speed_record.get("value")) if speed_record.get("present") else {}
  sdi = _dict(speed.get("sdi"))
  secondary_sdi = _dict(speed.get("sdi_secondary"))
  section = _dict(speed.get("section"))
  road_limit_valid = speed.get("road_limit_kph") is not None

  traffic_record = _record(snapshot, "traffic_signal")
  traffic = _dict(traffic_record.get("value")) if traffic_record.get("present") else {}
  lights = _dict(traffic.get("lights"))
  red_valid, red_on, red_remain = _signal_light(lights.get("red"))
  left_valid, left_on, left_remain = _signal_light(lights.get("left"))
  green_valid, green_on, green_remain = _signal_light(lights.get("green"))
  right_valid, right_on, right_remain = _signal_light(lights.get("right"))
  uturn_valid, uturn_on, uturn_remain = _signal_light(lights.get("uturn"))
  ui_counter = _dict(traffic.get("ui_counter"))

  crossroad_record = _record(snapshot, "crossroad")
  crossroad = _dict(crossroad_record.get("value")) if crossroad_record.get("present") else {}

  route_record = _record(snapshot, "route")
  route = _dict(route_record.get("value")) if route_record.get("present") else {}
  polyline = []
  for point_value in _list(route.get("polyline"))[:MAX_ROUTE_POINTS]:
    point = _dict(point_value)
    if point.get("lat") is None or point.get("lon") is None:
      continue
    polyline.append({
      "latitude": _finite(point.get("lat"), minimum=-90.0, maximum=90.0),
      "longitude": _finite(point.get("lon"), minimum=-180.0, maximum=180.0),
    })

  status_record = _record(snapshot, "navigation_status")
  status = _dict(status_record.get("value")) if status_record.get("present") else {}

  return {
    "schemaVersion": SCHEMA_VERSION,
    "generation": _integer(snapshot.get("generation"), minimum=0),
    "sessionId": _text(snapshot.get("session_id"), 32),
    "publishMonoTimeNanos": publish_mono_ns if publish_mono_ns is not None else time.monotonic_ns(),
    "connected": bool(snapshot.get("connected", False)),
    "vehicle": {
      "meta": _meta(vehicle_record),
      "latitude": _finite(vehicle.get("lat"), minimum=-90.0, maximum=90.0),
      "longitude": _finite(vehicle.get("lon"), minimum=-180.0, maximum=180.0),
      "headingDeg": _finite(vehicle.get("heading_deg"), minimum=0.0, maximum=360.0),
      "speedKph": _finite(vehicle.get("speed_kph"), minimum=0.0, maximum=300.0),
      "roadName": _text(vehicle.get("road_name"), 96),
      "virtualGps": bool(vehicle.get("virtual_gps", False)),
    },
    "guidanceCurrent": _guidance(snapshot, "guidance_current"),
    "guidanceNext": _guidance(snapshot, "guidance_next"),
    "laneCurrent": _lane(lane_record, lane_value),
    "laneAhead": [
      _lane(ahead_record, value)
      for value in ahead_values[:MAX_AHEAD_LANES]
    ],
    "speed": {
      "meta": _meta(speed_record),
      "currentKph": _finite(speed.get("current_kph"), minimum=0.0, maximum=300.0),
      "roadLimitValid": road_limit_valid,
      "roadLimitKph": _integer(speed.get("road_limit_kph"), minimum=0, maximum=300),
      "sdiPresent": bool(sdi),
      "sdiType": _integer(sdi.get("type"), default=-1, minimum=-1, maximum=100_000),
      "sdiDistanceM": _integer(sdi.get("distance_m"), minimum=0, maximum=2_000_000),
      "sdiSpeedLimitKph": _integer(sdi.get("speed_limit_kph"), minimum=0, maximum=300),
      "sdiSectionType": _integer(sdi.get("section_type"), default=-1, minimum=-1, maximum=100_000),
      "sdiBlockType": _integer(sdi.get("block_type"), default=-1, minimum=-1, maximum=100_000),
      "sdiBlockSpeedKph": _integer(sdi.get("block_speed_kph"), minimum=0, maximum=300),
      "sdiBlockDistanceM": _integer(sdi.get("block_distance_m"), minimum=0, maximum=2_000_000),
      "secondarySdiPresent": bool(secondary_sdi),
      "secondarySdiType": _integer(secondary_sdi.get("type"), default=-1, minimum=-1, maximum=100_000),
      "secondarySdiDistanceM": _integer(secondary_sdi.get("distance_m"), minimum=0, maximum=2_000_000),
      "secondarySdiSpeedLimitKph": _integer(secondary_sdi.get("speed_limit_kph"), minimum=0, maximum=300),
      "secondarySdiSectionType": _integer(secondary_sdi.get("section_type"), default=-1, minimum=-1, maximum=100_000),
      "secondarySdiBlockType": _integer(secondary_sdi.get("block_type"), default=-1, minimum=-1, maximum=100_000),
      "secondarySdiBlockSpeedKph": _integer(secondary_sdi.get("block_speed_kph"), minimum=0, maximum=300),
      "secondarySdiBlockDistanceM": _integer(secondary_sdi.get("block_distance_m"), minimum=0, maximum=2_000_000),
      "sectionPresent": bool(section),
      "sectionActive": bool(section.get("active", False)),
      "sectionSpeedLimitKph": _integer(section.get("speed_limit_kph"), minimum=0, maximum=300),
      "sectionAverageKph": _finite(section.get("average_kph"), minimum=0.0, maximum=300.0),
      "sectionOverallAverageKph": _finite(section.get("overall_average_kph"), minimum=0.0, maximum=300.0),
      "sectionRemainingDistanceM": _finite(section.get("remaining_distance_m"), minimum=0.0, maximum=2_000_000.0),
      "sectionRemainingTimeSec": _integer(section.get("remaining_time_sec"), minimum=0, maximum=604_800),
      "sectionProgress": _finite(section.get("progress"), minimum=0.0, maximum=1.0),
      "sectionSuspended": bool(section.get("suspended", False)),
      "sectionOffRoute": bool(section.get("off_route", False)),
    },
    "trafficSignal": {
      "meta": _meta(traffic_record),
      "visible": bool(traffic.get("visible", False)),
      "distanceM": _integer(traffic.get("distance_m"), minimum=0, maximum=100_000),
      "source": _text(traffic.get("source"), 32),
      "redValid": red_valid,
      "redOn": red_on,
      "redRemainSec": red_remain,
      "leftValid": left_valid,
      "leftOn": left_on,
      "leftRemainSec": left_remain,
      "greenValid": green_valid,
      "greenOn": green_on,
      "greenRemainSec": green_remain,
      "rightValid": right_valid,
      "rightOn": right_on,
      "rightRemainSec": right_remain,
      "uturnValid": uturn_valid,
      "uturnOn": uturn_on,
      "uturnRemainSec": uturn_remain,
      "uiCounterValid": ui_counter.get("remain_sec") is not None,
      "uiCounterRemainSec": _integer(ui_counter.get("remain_sec"), minimum=0, maximum=999),
    },
    "crossroad": {
      "meta": _meta(crossroad_record),
      "visible": bool(crossroad.get("visible", False)),
      "distanceM": _integer(crossroad.get("distance_m"), minimum=0, maximum=100_000),
      "imageCode": _integer(crossroad.get("image_code"), minimum=0, maximum=2_147_483_647),
      "imageUrl": _text(crossroad.get("image_url"), 512),
    },
    "route": {
      "meta": _meta(route_record),
      "remainingDistanceM": _integer(route.get("remain_distance_m"), minimum=0, maximum=2_000_000),
      "remainingTimeSec": _integer(route.get("remain_time_sec"), minimum=0, maximum=604_800),
      "movedDistanceM": _integer(route.get("moved_distance_m"), minimum=0, maximum=2_000_000),
      "movedTimeSec": _integer(route.get("moved_time_sec"), minimum=0, maximum=604_800),
      "totalDistanceM": _integer(route.get("total_distance_m"), minimum=0, maximum=2_000_000),
      "polyline": polyline,
    },
    "navigationStatus": {
      "meta": _meta(status_record),
      "mode": _text(status.get("mode"), 32),
      "guidanceActive": bool(status.get("guidance_active", False)),
      "offRoute": bool(status.get("off_route", False)),
      "routePresent": bool(status.get("route_present", False)),
    },
  }


def build_carrot_navi_media_payload(record: Any, session_id: str) -> dict[str, Any]:
  return {
    "schemaVersion": SCHEMA_VERSION,
    "sessionId": _text(session_id, 32),
    "kind": _text(record.kind, 16),
    "name": _text(record.name, 64),
    "sequence": _integer(record.sequence, minimum=0),
    "sourceTimestampMillis": _integer(record.source_timestamp_ms, minimum=0),
    "receivedMonoTimeNanos": _integer(record.received_mono_ns, minimum=0),
    "present": bool(record.present),
    "messageType": _integer(record.message_type, minimum=0, maximum=255),
    "formatOrReason": _integer(record.format_or_reason, minimum=0, maximum=255),
    "flags": _integer(record.flags, minimum=0, maximum=65535),
    "width": _integer(record.width, minimum=0, maximum=65535),
    "height": _integer(record.height, minimum=0, maximum=65535),
    "reason": _text(record.reason, 64),
    "payload": record.payload or b"",
  }


class CarrotNaviCerealPublisher:
  def __init__(self, receiver: CarrotNaviReceiver, messaging_module: Any | None = None,
               params: Any | None = None) -> None:
    if messaging_module is None:
      import openpilot.cereal.messaging as messaging_module

    self.receiver = receiver
    self.messaging = messaging_module
    self.pm = messaging_module.PubMaster(["carrotNavi", "carrotNaviMedia"])
    if params is None:
      try:
        from openpilot.common.params import Params
        params = Params()
      except Exception:
        params = None
    self.params = params
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None
    self._media_request = ""
    self._next_media_request_poll = 0.0

  def start(self) -> None:
    if self._thread is not None:
      return
    self._thread = threading.Thread(target=self._run, name="carrot_navi_cereal", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    thread = self._thread
    if thread is not None:
      thread.join(timeout=1.0)
    self._thread = None

  def publish_once(self, snapshot: dict[str, Any] | None = None) -> int:
    if snapshot is None:
      snapshot = self.receiver.cereal_snapshot()
    message = self.messaging.new_message("carrotNavi", valid=True)
    message.carrotNavi = build_carrot_navi_payload(snapshot)
    self.pm.send("carrotNavi", message)
    self.receiver.record_cereal_publish()
    return _integer(snapshot.get("generation"), minimum=0)

  def publish_media(self, record: Any, session_id: str, kind_override: str | None = None) -> None:
    message = self.messaging.new_message("carrotNaviMedia", valid=True)
    payload = build_carrot_navi_media_payload(record, session_id)
    if kind_override is not None:
      payload["kind"] = kind_override
    message.carrotNaviMedia = payload
    self.pm.send("carrotNaviMedia", message)
    self.receiver.record_cereal_publish()

  def _media_web_request(self) -> str:
    now = time.monotonic()
    if now < self._next_media_request_poll:
      return self._media_request
    self._next_media_request_poll = now + MEDIA_REQUEST_POLL_SECONDS
    try:
      value = self.params.get("CarrotNaviWebBootstrapRequest") if self.params is not None else None
      if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
      self._media_request = str(value or "")
    except Exception:
      pass
    return self._media_request

  def _run(self) -> None:
    last_generation = -1
    last_publish = 0.0
    while not self._stop.is_set():
      elapsed = time.monotonic() - last_publish
      self.receiver.wait_for_state_change(max(0.0, PUBLISH_HEARTBEAT_SECONDS - elapsed))
      if self._stop.is_set():
        return

      elapsed = time.monotonic() - last_publish
      if elapsed < PUBLISH_COALESCE_SECONDS and self._stop.wait(PUBLISH_COALESCE_SECONDS - elapsed):
        return

      snapshot = self.receiver.cereal_snapshot()
      media_updates = self.receiver.drain_media_updates()
      try:
        media_request_before = self._media_request
        media_request = self._media_web_request()
        if media_request and media_request != media_request_before:
          bootstrap = [
            record for record in self.receiver.media_bootstrap()
            if _is_web_media_stream(record.kind, record.name)
          ]
          media_update_ids = {(record.kind, record.name, record.sequence) for record in media_updates}
          for record in bootstrap:
            if (record.kind, record.name, record.sequence) in media_update_ids:
              continue
            kind_override = WEB_BOOTSTRAP_IMAGE_KIND if record.kind == "image" else WEB_BOOTSTRAP_KIND
            self.publish_media(record, str(snapshot.get("session_id", "")), kind_override=kind_override)
        for record in media_updates:
          self.publish_media(record, str(snapshot.get("session_id", "")))
      except Exception as exc:
        self.receiver.record_cereal_publish(str(exc))

      generation = _integer(snapshot.get("generation"), minimum=0)
      elapsed = time.monotonic() - last_publish
      if generation == last_generation and elapsed < PUBLISH_HEARTBEAT_SECONDS:
        continue

      try:
        last_generation = self.publish_once(snapshot)
        last_publish = time.monotonic()
      except Exception as exc:
        self.receiver.record_cereal_publish(str(exc))
        last_publish = time.monotonic()
