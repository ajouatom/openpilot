from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time

import pytest

from openpilot.selfdrive.carrot.carrot_navi import CATALOG
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_navi import fresh_carrot_navi, parse_carrot_navi
from cluster_navi_overlay import merge_navi_overlay_state
from cluster_navi_source import (
  MAP_FRAME_STALE_TIMEOUT_MS,
  H264DecodeWorker,
  NaviIpcMediaSource,
  NaviSimulatorSource,
  _H264DecodeRequest,
  _H264DecodeResult,
)
from cluster_models import NaviDashboardState, NaviMediaFrame, TpmsInfo
from cluster_renderer import SIDE_GAUGE_OUTLINE, ClusterUiRenderer


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
  stalled = replace(connected, map_stream_stalled=True)

  assert ClusterUiRenderer._navi_map_frame_present(connected) is True
  assert ClusterUiRenderer._navi_map_frame_present(disconnected) is False
  assert ClusterUiRenderer._navi_map_frame_present(stalled) is False


def test_stale_map_frame_is_removed_from_media_cache():
  source = object.__new__(NaviSimulatorSource)
  source._media = {"render:map_main": NaviMediaFrame("render:map_main", 7, True)}
  source._map_frame_age_ms = None
  source._map_stream_stalled = False
  received_at_ms = time.time_ns() // 1_000_000 - MAP_FRAME_STALE_TIMEOUT_MS - 1

  source._update_map_liveness({
    "connected": True,
    "records": {
      "render:map_main": SimpleNamespace(present=True, received_at_ms=received_at_ms),
    },
  })

  assert source._map_stream_stalled is True
  assert source._map_frame_age_ms > MAP_FRAME_STALE_TIMEOUT_MS
  assert "render:map_main" not in source._media


def test_h264_decode_worker_drops_backlog_until_next_keyframe():
  started = threading.Event()
  release = threading.Event()
  reset_calls = []

  class FakeDecoder:
    def reset(self):
      reset_calls.append(True)

    def decode(self, payload):
      if payload.endswith(b"1"):
        started.set()
        assert release.wait(1.0)
      return payload[-1:] * 4, 1, 1

  def request(sequence, *, keyframe=False):
    return _H264DecodeRequest(
      epoch=1,
      key="render:map_main",
      sequence=sequence,
      payload=str(sequence).encode(),
      config_payload=b"config",
      config_sequence=1,
      keyframe=keyframe,
    )

  worker = H264DecodeWorker(decoder_factory=FakeDecoder)
  try:
    worker.submit(request(1, keyframe=True))
    assert started.wait(1.0)
    worker.submit(request(2))
    worker.submit(request(3))
    worker.submit(request(4))
    worker.submit(request(5))
    worker.submit(request(6, keyframe=True))
    release.set()

    latest = None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and (latest is None or latest.frame.sequence != 6):
      results = worker.poll()
      if results:
        latest = results[-1]
      time.sleep(0.005)

    assert latest is not None
    assert latest.frame.sequence == 6
    assert latest.frame.data == b"6666"
    assert worker.dropped_requests == 4
    assert len(reset_calls) == 1
  finally:
    release.set()
    worker.close()


def test_navi_source_rejects_stale_h264_worker_result():
  source = object.__new__(NaviSimulatorSource)
  source._media = {}
  source._media_epoch = 2
  source._h264_requested_sequences = {"render:map_main": 8}
  stale = _H264DecodeResult(
    epoch=2,
    frame=NaviMediaFrame("render:map_main", 7, True, "image/rgba", 1, 1, b"old!"),
  )
  latest = _H264DecodeResult(
    epoch=2,
    frame=NaviMediaFrame("render:map_main", 8, True, "image/rgba", 1, 1, b"new!"),
  )
  results = iter(((stale,), (latest,)))
  source._h264_worker = SimpleNamespace(poll=lambda: next(results))

  source._apply_h264_results()
  assert source._media == {}

  source._apply_h264_results()
  assert source._media["render:map_main"].sequence == 8


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


def test_live_navi_guidance_media_is_scaled_up(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  frames = {
    key: NaviMediaFrame(key, 1, True, "image/rgba", 1, 1, b"rgba")
    for key in ("render:map_main", "image:tbt_current_compact", "image:tbt_next", "image:lane_bottom")
  }
  dashboard = NaviDashboardState(True, "ipc://carrotNaviMedia", media=tuple(frames.values()))
  drawn = {}

  monkeypatch.setattr(
    ClusterUiRenderer,
    "_current_theme",
    lambda self: SimpleNamespace(faint=(0, 0, 0, 0), route_panel_bg=(0, 0, 0, 255)),
  )
  monkeypatch.setattr(ClusterUiRenderer, "_rounded_rect", lambda *args, **kwargs: None)
  monkeypatch.setattr(ClusterUiRenderer, "_draw_navi_crossroad_box", lambda *args, **kwargs: None)
  monkeypatch.setattr(ClusterUiRenderer, "_navi_media_fitted_size", lambda self, frame, rect: (rect.width, rect.height))

  def capture_media(self, frame, rect, **kwargs):
    if frame is not None:
      drawn[frame.key] = rect
    return True

  monkeypatch.setattr(ClusterUiRenderer, "_draw_navi_media", capture_media)
  monkeypatch.setattr("cluster_renderer.rl.draw_rectangle_rec", lambda *args, **kwargs: None)
  monkeypatch.setattr("cluster_renderer.rl.begin_scissor_mode", lambda *args, **kwargs: None)
  monkeypatch.setattr("cluster_renderer.rl.end_scissor_mode", lambda: None)

  renderer._draw_navi_live_panel(SimpleNamespace(navi_live=None, navi_dashboard=dashboard))

  assert drawn["image:tbt_current_compact"].width == pytest.approx(310.0 * 1.2)
  assert drawn["image:tbt_current_compact"].height == pytest.approx(116.0 * 1.2)
  assert drawn["image:tbt_next"].width == pytest.approx(190.0 * 1.2)
  assert drawn["image:tbt_next"].height == pytest.approx(68.0 * 1.2)
  assert drawn["image:lane_bottom"].width == pytest.approx(226.0 * 1.2)
  assert drawn["image:lane_bottom"].height == pytest.approx(67.0 * 1.2)


def test_ipc_media_source_restores_standalone_navigation_images():
  media = SimpleNamespace(
    schemaVersion=1,
    sessionId="ipc-session",
    kind="image",
    name="tbt_next",
    sequence=7,
    present=True,
    reason="",
    messageType=1,
    formatOrReason=1,
    width=32,
    height=24,
    payload=b"\x89PNG\r\n\x1a\ncontent",
  )

  class FakeMessaging:
    def __init__(self):
      self.messages = [SimpleNamespace(carrotNaviMedia=media)]

    def sub_sock(self, service, conflate):
      assert service == "carrotNaviMedia"
      assert conflate is False
      return object()

    def drain_sock(self, _socket):
      messages, self.messages = self.messages, []
      return messages

  source = NaviIpcMediaSource(FakeMessaging())
  dashboard = source.update(SimpleNamespace(session_id="ipc-session"))

  assert dashboard.connected is True
  assert dashboard.endpoint == "ipc://carrotNaviMedia"
  assert dashboard.session_id == "ipc-session"
  assert dashboard.received_count == 1
  assert dashboard.media == (
    NaviMediaFrame("image:tbt_next", 7, True, "image/png", 32, 24, b"\x89PNG\r\n\x1a\ncontent"),
  )


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


def test_navi_panel_uses_same_design_shift_for_turn_signals():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1280
  renderer.screen_mode = 0
  dashboard = NaviDashboardState(False, "tcp://127.0.0.1:7714")

  assert renderer._world_view_shift_design_x(SimpleNamespace(
    camera_view_mode=0,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == pytest.approx(398)
  assert renderer._world_view_shift_design_x(SimpleNamespace(
    camera_view_mode=1,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == pytest.approx(398)
  assert renderer._world_view_shift_design_x(SimpleNamespace(
    camera_view_mode=2,
    navi_live=None,
    navi_dashboard=dashboard,
  )) == 0


def test_dark_theme_uses_visible_side_gauge_outlines(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  state = SimpleNamespace(camera_view_mode=0)

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: SimpleNamespace(is_dark=True))
  assert renderer._side_gauge_outline(state) == SIDE_GAUGE_OUTLINE

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: SimpleNamespace(is_dark=False))
  assert renderer._side_gauge_outline(state) == (5, 9, 12, 235)
  state.camera_view_mode = 2
  assert renderer._side_gauge_outline(state) == SIDE_GAUGE_OUTLINE


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
    map_stream = next(
      item for item in manifest["streams"]
      if item["kind"] == "render" and item["name"] == "map_main"
    )
    assert map_stream["params"]["map_theme"] == "dark"
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

    source.receiver.control_disconnected()
    disconnected = source.update()
    assert disconnected.navi_dashboard is not None
    assert disconnected.navi_dashboard.connected is False
    assert disconnected.navi_dashboard.media == ()
  finally:
    source.close()
