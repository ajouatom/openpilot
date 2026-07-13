from pathlib import Path
from types import SimpleNamespace
import sys

from openpilot.selfdrive.carrot.carrot_navi import CATALOG
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_navi import fresh_carrot_navi, parse_carrot_navi
from cluster_navi_source import NaviSimulatorSource


def _namespace(value):
  if isinstance(value, dict):
    return SimpleNamespace(**{key: _namespace(item) for key, item in value.items()})
  if isinstance(value, list):
    return [_namespace(item) for item in value]
  return value


def _record(value, sequence: int, received_s: float):
  return {
    "present": True,
    "sequence": sequence,
    "source_timestamp_ms": 1000 + sequence,
    "received_mono_ns": int(received_s * 1_000_000_000),
    "value": value,
  }


def test_parse_and_expire_live_navi_groups_independently():
  snapshot = {
    "generation": 9,
    "session_id": "session",
    "connected": True,
    "items": {
      "guidance_current": _record({
        "distance_m": 320,
        "time_sec": 35,
        "turn_type": 12,
        "main_text": "Turn left",
      }, 1, 100.0),
      "lane_current": _record({
        "count": 4,
        "distance_m": 250,
        "visible": True,
        "current_lane": 2,
        "available": [0, 8, 8, 0],
      }, 2, 100.0),
      "speed": _record({
        "current_kph": 48,
        "road_limit_kph": 50,
        "sdi": {"type": 1, "distance_m": 420, "speed_limit_kph": 50},
      }, 3, 100.0),
      "traffic_signal": _record({
        "visible": True,
        "distance_m": 145,
        "lights": {"red": {"on": True, "remain_sec": 18}},
      }, 4, 100.0),
      "route": _record({
        "remain_distance_m": 12500,
        "remain_time_sec": 1320,
        "polyline": [{"lat": 37.5, "lon": 127.0}],
      }, 5, 100.0),
    },
  }
  payload = build_carrot_navi_payload(snapshot, publish_mono_ns=100_100_000_000)
  state = parse_carrot_navi(_namespace(payload), now=100.1)

  assert state is not None
  assert state.generation == 9
  assert state.current is not None and state.current.main_text == "Turn left"
  assert state.lane_current is not None and state.lane_current.available == (0, 8, 8, 0)
  assert state.speed is not None and state.speed.road_limit_kph == 50
  assert state.traffic_light is not None and state.traffic_light.red_s == 18
  assert state.route is not None and state.route.polyline == ((37.5, 127.0),)

  after_live_expiry, next_expiry = fresh_carrot_navi(state, now=106.1)
  assert after_live_expiry is not None
  assert after_live_expiry.current is None
  assert after_live_expiry.speed is None
  assert after_live_expiry.route is not None
  assert after_live_expiry.traffic_light is not None
  assert next_expiry == 130.0

  after_all_expiry, next_expiry = fresh_carrot_navi(after_live_expiry, now=160.1)
  assert after_all_expiry is None
  assert next_expiry == float("inf")


def test_disconnected_snapshot_clears_live_navi():
  payload = build_carrot_navi_payload({
    "generation": 2,
    "session_id": "session",
    "connected": False,
    "items": {},
  }, publish_mono_ns=1)

  assert parse_carrot_navi(_namespace(payload), now=1.0) is None


def test_parser_accepts_direct_projection_dict():
  payload = build_carrot_navi_payload({
    "generation": 3,
    "session_id": "direct",
    "connected": True,
    "items": {
      "vehicle": _record({
        "lat": 37.5,
        "lon": 127.0,
        "heading_deg": 90,
        "speed_kph": 42,
        "road_name": "Direct road",
      }, 4, 10.0),
    },
  }, publish_mono_ns=10_100_000_000)

  state = parse_carrot_navi(payload, now=10.1)

  assert state is not None
  assert state.session_id == "direct"
  assert state.vehicle is not None
  assert state.vehicle.speed_kph == 42
  assert state.vehicle.road_name == "Direct road"


def test_embedded_navi_source_projects_json_and_png(unused_tcp_port):
  source = NaviSimulatorSource(
    host="127.0.0.1",
    port=unused_tcp_port,
    advertise_ip="127.0.0.1",
    beacon_enabled=False,
  )
  try:
    manifest = source.receiver.negotiate({
      "type": "requirements_query",
      "protocol_version": 2,
      "streams": [
        {"kind": kind, "name": name, "schema_version": 1}
        for kind, name in CATALOG
      ],
    }, "test-app")
    source.receiver.control_connected()
    vehicle = next(
      item for item in manifest["streams"]
      if item["kind"] == "json" and item["name"] == "vehicle"
    )
    source.receiver.record_json(manifest["session_id"], "vehicle", {
      "type": "item_update",
      "protocol_version": 2,
      "session_id": manifest["session_id"],
      "manifest_revision": manifest["revision"],
      "schema_version": 1,
      "kind": "json",
      "name": "vehicle",
      "stream_handle": vehicle["stream_handle"],
      "sequence": 1,
      "source_timestamp_ms": 1000,
      "present": True,
      "value": {"lat": 37.5, "lon": 127.0, "speed_kph": 36, "road_name": "Navi road"},
    }, "127.0.0.1")
    image = next(
      item for item in manifest["streams"]
      if item["kind"] == "image" and item["name"] == "tbt_next"
    )
    source.receiver.record_binary(manifest["session_id"], "image", "tbt_next", {
      "message_type": 1,
      "format_or_reason": 1,
      "flags": 0,
      "stream_handle": image["stream_handle"],
      "manifest_revision": manifest["revision"],
      "sequence": 1,
      "source_timestamp_ms": 1000,
      "width": 32,
      "height": 24,
    }, b"\x89PNG\r\n\x1a\ncontent", "127.0.0.1")

    state = source.update()

    assert state.navi_live is not None
    assert state.speed_kph == 36
    assert state.navi_dashboard is not None
    assert state.navi_dashboard.connected is True
    assert any(frame.key == "image:tbt_next" for frame in state.navi_dashboard.media)
    assert len(state.navi_dashboard.items) == 28
  finally:
    source.close()
