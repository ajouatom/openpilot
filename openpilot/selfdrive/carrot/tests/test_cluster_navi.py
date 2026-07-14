from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

from openpilot.selfdrive.carrot.carrot_navi import CATALOG
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_navi import fresh_carrot_navi, parse_carrot_navi
from cluster_navi_overlay import merge_navi_overlay_state
from cluster_navi_source import NaviSimulatorSource
from cluster_models import NaviDashboardState, NaviMediaFrame, TpmsInfo
from cluster_renderer import ClusterUiRenderer


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


def test_disconnected_dashboard_does_not_reuse_stale_map_frame():
  map_frame = NaviMediaFrame("render:map_main", 1, True)
  connected = NaviDashboardState(True, "tcp://127.0.0.1:7714", media=(map_frame,))
  disconnected = replace(connected, connected=False)

  assert ClusterUiRenderer._navi_map_frame_present(connected) is True
  assert ClusterUiRenderer._navi_map_frame_present(disconnected) is False


def test_disconnected_dashboard_draws_system_panel(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  dashboard = NaviDashboardState(False, "tcp://127.0.0.1:7714")
  state = SimpleNamespace(navi_live=None, navi_dashboard=dashboard)
  drawn = {}

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: SimpleNamespace())

  def capture_system_panel(self, panel_state, **kwargs):
    drawn["state"] = panel_state
    drawn.update(kwargs)

  monkeypatch.setattr(ClusterUiRenderer, "_draw_system_stats_panel", capture_system_panel)

  renderer._draw_navi_live_panel(state)

  assert drawn == {
    "state": state,
    "panel_x": 1124,
    "panel_y": 1,
    "panel_w": 792,
    "panel_h": 478,
    "status_text": "NAVI DISCONNECTED",
  }


def test_navi_panel_shifts_3d_camera_modes_left():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = 0
  dashboard = NaviDashboardState(False, "tcp://127.0.0.1:7714")

  assert renderer._world_view_shift_x(SimpleNamespace(
    camera_view_mode=0,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == 398
  assert renderer._world_view_shift_x(SimpleNamespace(
    camera_view_mode=1,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == 398
  assert renderer._world_view_shift_x(SimpleNamespace(
    camera_view_mode=2,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == 0
  assert renderer._world_view_shift_x(SimpleNamespace(
    camera_view_mode=0,
    navi_live=None,
    navi_dashboard=None,
  )) == 0


def test_road_camera_draws_compact_tpms_only(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  badges = []

  monkeypatch.setattr(ClusterUiRenderer, "_rounded_rect", lambda *args, **kwargs: None)
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_compact_tpms_value",
    lambda self, pressure, center_x, center_y: badges.append((pressure, center_x, center_y)),
  )
  tpms = TpmsInfo(fl=35.0, fr=34.0, rl=33.0, rr=32.0)

  renderer._draw_camera_tpms(SimpleNamespace(camera_view_mode=0, tpms=tpms))
  assert badges == []

  renderer._draw_camera_tpms(SimpleNamespace(camera_view_mode=2, tpms=tpms))
  assert [badge[0] for badge in badges] == [35.0, 34.0, 33.0, 32.0]
  assert badges[0][1] < badges[1][1]
  assert badges[0][2] < badges[2][2]


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

    replay_state = replace(
      state,
      speed_kph=83,
      navi_live=None,
      navi_dashboard=None,
    )
    merged = merge_navi_overlay_state(replay_state, state)

    assert merged.speed_kph == 83
    assert merged.navi_live is state.navi_live
    assert merged.navi_dashboard is state.navi_dashboard
  finally:
    source.close()
