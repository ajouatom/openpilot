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

from cluster_config import (
  DARK_CLUSTER_THEME,
  GREEN,
  LIGHT_CLUSTER_THEME,
  RADAR_TO_CAMERA_M,
  RED,
  VEHICLE_LENGTH_M,
  WHITE,
)
from cluster_navi import fresh_carrot_navi, parse_carrot_navi, resolve_navi_speed_limit
from cluster_navi_overlay import merge_navi_overlay_state
from cluster_navi_source import (
  DecodedH264Frame,
  H264_DECODE_QUEUE_MAX,
  MAP_FRAME_STALE_TIMEOUT_MS,
  H264Decoder,
  H264DecodeWorker,
  NaviIpcMediaSource,
  NaviSimulatorSource,
  _H264DecodeRequest,
  _H264DecodeResult,
)
from cluster_models import NaviDashboardState, NaviMediaFrame, TpmsInfo
from cluster_renderer import (
  CAMERA_BACKGROUND_X,
  CAMERA_BACKGROUND_W,
  CAMERA_OVERLAY_VEHICLE_ROAD_HEIGHT_M,
  DESIGN_HEIGHT,
  DESIGN_WIDTH,
  LANE_TURN_SIGNAL_LEFT_CENTER_X,
  LANE_TURN_SIGNAL_RIGHT_CENTER_X,
  NAV_STATUS_CENTER_X,
  NAV_STATUS_CENTER_Y,
  NAV_STATUS_FONT_SIZE,
  NAVI_LIVE_PANEL_X,
  NAVI_MAP_BACKGROUND,
  SIDE_GAUGE_COLUMN_GAP,
  SIDE_GAUGE_LEFT_CENTER_X,
  SIDE_GAUGE_OUTLINE,
  TPMS_STATUS_CAR_H,
  TPMS_STATUS_CAR_W,
  TPMS_STATUS_CAR_CENTER_Y,
  TPMS_STATUS_CENTER_X,
  TPMS_STATUS_COLUMN_OFFSET,
  TPMS_STATUS_FONT_SIZE,
  TPMS_STATUS_ICON_H,
  TPMS_STATUS_ICON_W,
  TPMS_STATUS_ROW_OFFSET,
  TPMS_STATUS_VALUE_CENTER_Y,
  TPMS_STATUS_WHEEL_H,
  TPMS_STATUS_WHEEL_W,
  ClusterUiRenderer,
)


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


def _decoded_yuv_frame(value: bytes) -> DecodedH264Frame:
  return DecodedH264Frame(
    width=2,
    height=2,
    planes=(value * 4, b"\x80", b"\x80"),
    strides=(2, 1, 1),
  )


def _parse_speed_only_navi(speed_value: dict, sequence: int = 1):
  payload = build_carrot_navi_payload({
    "generation": sequence,
    "session_id": "session",
    "connected": True,
    "items": {
      "speed": _record(speed_value, sequence, 100.0),
    },
  }, publish_mono_ns=100_100_000_000)
  state = parse_carrot_navi(_namespace(payload), now=100.1)
  assert state is not None
  return state


def _parse_cereal_speed_only_navi(road_limit_kph: int):
  payload = {
    "schemaVersion": 1,
    "generation": 1,
    "sessionId": "session",
    "connected": True,
    "speed": {
      "meta": {
        "present": True,
        "sequence": 1,
        "sourceTimestampMillis": 1001,
        "receivedMonoTimeNanos": 100_000_000_000,
      },
      "currentKph": 42,
      "roadLimitValid": True,
      "roadLimitKph": road_limit_kph,
    },
  }
  state = parse_carrot_navi(_namespace(payload), now=100.1)
  assert state is not None
  return state


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
        "sdi_secondary": {"type": 22, "distance_m": 93},
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
  assert state.speed.secondary_sdi_type == 22
  assert state.speed.secondary_sdi_distance_m == 93
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


def test_navi_speed_limit_overrides_legacy_navigation_default_but_not_vehicle_limit():
  navi_50 = _parse_speed_only_navi({"current_kph": 42, "road_limit_kph": 50})

  assert resolve_navi_speed_limit(30, "n", navi_50) == (50, "n")
  assert resolve_navi_speed_limit(None, None, navi_50) == (50, "n")
  assert resolve_navi_speed_limit(80, "v", navi_50) == (80, "v")


def test_navi_speed_limit_unknown_clears_legacy_navigation_default():
  navi_unknown = _parse_speed_only_navi({"current_kph": 42})

  assert navi_unknown.speed is not None
  assert navi_unknown.speed.road_limit_kph is None
  assert resolve_navi_speed_limit(30, "n", navi_unknown) == (None, None)


def test_navi_encoded_speed_limit_clears_legacy_navigation_default():
  navi_encoded = _parse_cereal_speed_only_navi(1020)

  assert navi_encoded.speed is not None
  assert navi_encoded.speed.road_limit_kph is None
  assert resolve_navi_speed_limit(30, "n", navi_encoded) == (None, None)


def test_navi_invalid_speed_limit_clears_legacy_navigation_default():
  navi_invalid = _parse_cereal_speed_only_navi(300)

  assert navi_invalid.speed is not None
  assert navi_invalid.speed.road_limit_kph is None
  assert resolve_navi_speed_limit(30, "n", navi_invalid) == (None, None)


def test_disconnected_snapshot_clears_live_navi():
  payload = build_carrot_navi_payload({
    "generation": 2,
    "session_id": "session",
    "connected": False,
    "items": {},
  }, publish_mono_ns=1)

  assert parse_carrot_navi(_namespace(payload), now=1.0) is None


def test_parse_live_navi_reuses_unchanged_components_within_session():
  route_record = _record({
    "remain_distance_m": 12500,
    "remain_time_sec": 1320,
    "polyline": [
      {"lat": 37.5, "lon": 127.0},
      {"lat": 37.6, "lon": 127.1},
    ],
  }, 5, 100.0)
  first_payload = build_carrot_navi_payload({
    "generation": 9,
    "session_id": "session",
    "connected": True,
    "items": {
      "guidance_current": _record({"distance_m": 320, "main_text": "First"}, 1, 100.0),
      "route": route_record,
    },
  }, publish_mono_ns=100_100_000_000)
  first = parse_carrot_navi(_namespace(first_payload), now=100.1)
  assert first is not None and first.current is not None and first.route is not None

  second_payload = build_carrot_navi_payload({
    "generation": 10,
    "session_id": "session",
    "connected": True,
    "items": {
      "guidance_current": _record({"distance_m": 300, "main_text": "Second"}, 2, 100.1),
      "route": route_record,
    },
  }, publish_mono_ns=100_200_000_000)
  second = parse_carrot_navi(_namespace(second_payload), now=100.2, previous=first)
  fully_reparsed_second = parse_carrot_navi(_namespace(second_payload), now=100.2)

  assert second is not None and second.current is not None and second.route is not None
  assert second == fully_reparsed_second
  assert second.current is not first.current
  assert second.current.main_text == "Second"
  assert second.route is first.route

  next_session_payload = build_carrot_navi_payload({
    "generation": 1,
    "session_id": "next-session",
    "connected": True,
    "items": {"route": route_record},
  }, publish_mono_ns=100_300_000_000)
  next_session = parse_carrot_navi(_namespace(next_session_payload), now=100.3, previous=second)

  assert next_session is not None and next_session.route is not None
  assert next_session.route is not second.route


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


def test_h264_decoder_preserves_yuv420_planes_and_strides():
  class FakePlane:
    def __init__(self, data, line_size):
      self._data = data
      self.line_size = line_size

    def __bytes__(self):
      return self._data

  frame = SimpleNamespace(
    width=4,
    height=2,
    format=SimpleNamespace(name="yuv420p"),
    planes=(
      FakePlane(b"y" * 16, 8),
      FakePlane(b"u" * 4, 4),
      FakePlane(b"v" * 4, 4),
    ),
  )
  decoder = object.__new__(H264Decoder)
  decoder._codec = SimpleNamespace(decode=lambda _packet: [frame])
  decoder._av = SimpleNamespace(Packet=lambda payload: payload, FFmpegError=RuntimeError)

  decoded = decoder.decode(b"access-unit")

  assert decoded == DecodedH264Frame(
    width=4,
    height=2,
    planes=(b"y" * 16, b"u" * 4, b"v" * 4),
    strides=(8, 4, 4),
  )


def test_h264_decoder_prefers_tici_hardware_buffer(monkeypatch):
  hardware_buffer = SimpleNamespace(width=960, height=540, sequence=17)
  calls = []

  class FakeHardwareDecoder:
    def decode(self, payload, sequence):
      calls.append((payload, sequence))
      return hardware_buffer

    def close(self):
      calls.append("close")

  monkeypatch.setitem(
    H264Decoder.decode.__globals__,
    "create_tici_h264_decoder",
    lambda width, height: calls.append((width, height)) or FakeHardwareDecoder(),
  )
  decoder = H264Decoder()

  decoded = decoder.decode(b"access-unit", sequence=17, width=960, height=540)
  decoder.close()

  assert decoded == DecodedH264Frame(width=960, height=540, hardware_buffer=hardware_buffer)
  assert calls == [(960, 540), (b"access-unit", 17), "close"]
  assert decoder._codec is None


def test_h264_decode_worker_drops_backlog_until_next_keyframe():
  started = threading.Event()
  release = threading.Event()
  reset_calls = []

  class FakeDecoder:
    def reset(self):
      reset_calls.append(True)

    def decode(self, payload, **_kwargs):
      if payload.endswith(b"1"):
        started.set()
        assert release.wait(1.0)
      return _decoded_yuv_frame(payload[-1:])

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
    for sequence in range(2, H264_DECODE_QUEUE_MAX + 3):
      worker.submit(request(sequence))
    next_keyframe = H264_DECODE_QUEUE_MAX + 3
    worker.submit(request(next_keyframe, keyframe=True))
    release.set()

    latest = None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and (latest is None or latest.frame.sequence != next_keyframe):
      results = worker.poll()
      if results:
        latest = results[-1]
      time.sleep(0.005)

    assert latest is not None
    assert latest.frame.sequence == next_keyframe
    assert latest.frame.mime == "image/yuv420p"
    assert latest.frame.plane_data == (
      str(next_keyframe).encode()[-1:] * 4,
      b"\x80",
      b"\x80",
    )
    assert worker.dropped_requests == H264_DECODE_QUEUE_MAX + 1
    assert len(reset_calls) == 1
  finally:
    release.set()
    worker.close()


def test_h264_decode_worker_accepts_sixty_hz_poll_burst():
  started = threading.Event()
  release = threading.Event()

  class FakeDecoder:
    def reset(self):
      pass

    def decode(self, payload, **_kwargs):
      if payload.endswith(b"1"):
        started.set()
        assert release.wait(1.0)
      return _decoded_yuv_frame(payload[-1:])

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
    for sequence in range(2, 8):
      worker.submit(request(sequence))
    release.set()

    latest = None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and (latest is None or latest.frame.sequence != 7):
      results = worker.poll()
      if results:
        latest = results[-1]
      time.sleep(0.005)

    assert latest is not None
    assert latest.frame.sequence == 7
    assert worker.dropped_requests == 0
  finally:
    release.set()
    worker.close()


def test_h264_decode_worker_ignores_repeated_identical_codec_config():
  reset_calls = []

  class FakeDecoder:
    def reset(self):
      reset_calls.append(True)

    def decode(self, payload, **_kwargs):
      return _decoded_yuv_frame(payload[-1:])

  def request(sequence, config_sequence, config_payload):
    return _H264DecodeRequest(
      epoch=1,
      key="render:map_main",
      sequence=sequence,
      payload=str(sequence).encode(),
      config_payload=config_payload,
      config_sequence=config_sequence,
      keyframe=True,
      width=960,
      height=540,
    )

  def wait_for(worker, sequence):
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
      results = worker.poll()
      if results and results[-1].frame.sequence == sequence:
        return
      time.sleep(0.005)
    raise AssertionError(f"decoder result {sequence} did not arrive")

  worker = H264DecodeWorker(decoder_factory=FakeDecoder)
  try:
    worker.submit(request(1, 10, b"same-config"))
    wait_for(worker, 1)
    worker.submit(request(2, 11, b"same-config"))
    wait_for(worker, 2)
    assert reset_calls == []

    worker.submit(request(3, 12, b"changed-config"))
    wait_for(worker, 3)
    assert reset_calls == [True]
  finally:
    worker.close()


def test_navi_source_rejects_stale_h264_worker_result():
  source = object.__new__(NaviSimulatorSource)
  source._media = {
    "render:map_main": NaviMediaFrame("render:map_main", 8, False, reason="source_absent"),
  }
  source._media_epoch = 2
  stale = _H264DecodeResult(
    epoch=2,
    frame=NaviMediaFrame("render:map_main", 7, True, "image/rgba", 1, 1, b"old!"),
  )
  latest = _H264DecodeResult(
    epoch=2,
    frame=NaviMediaFrame("render:map_main", 9, True, "image/rgba", 1, 1, b"new!"),
  )
  results = iter(((stale,), (latest,)))
  source._h264_worker = SimpleNamespace(poll=lambda: next(results))

  source._apply_h264_results()
  assert source._media["render:map_main"].sequence == 8
  assert source._media["render:map_main"].present is False

  source._apply_h264_results()
  assert source._media["render:map_main"].sequence == 9


def test_navi_source_accepts_decoded_frame_behind_latest_request():
  source = object.__new__(NaviIpcMediaSource)
  source._media = {
    "render:map_main": NaviMediaFrame("render:map_main", 10, True, "image/rgba", 1, 1, b"old!"),
  }
  source._media_epoch = 2
  source._h264_requested_sequences = {"render:map_main": 15}
  source._map_frame_at_s = 0.0
  result = _H264DecodeResult(
    epoch=2,
    frame=NaviMediaFrame("render:map_main", 12, True, "image/rgba", 1, 1, b"new!"),
  )
  source._h264_worker = SimpleNamespace(poll=lambda: (result,))

  source._apply_h264_results(123.0)

  assert source._media["render:map_main"].sequence == 12
  assert source._map_frame_at_s == 123.0


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


def test_dark_canvas_matches_navigation_backing_without_flattening_panels():
  assert (*DARK_CLUSTER_THEME.bg, 255) == NAVI_MAP_BACKGROUND == (0, 0, 0, 255)
  assert DARK_CLUSTER_THEME.panel_bg != DARK_CLUSTER_THEME.bg
  assert DARK_CLUSTER_THEME.route_panel_bg != DARK_CLUSTER_THEME.bg
  assert DARK_CLUSTER_THEME.route_video_bg != DARK_CLUSTER_THEME.bg


def test_live_navi_guidance_media_is_scaled_up(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  frames = {
    key: NaviMediaFrame(key, 1, True, "image/rgba", 1, 1, b"rgba")
    for key in (
      "render:map_main",
      "image:tbt_current_compact",
      "image:tbt_next",
      "image:lane_top",
      "image:lane_bottom",
      "image:traffic_signal",
      "image:center_tbt_icon",
      "image:center_tbt_text",
    )
  }
  dashboard = NaviDashboardState(True, "ipc://carrotNaviMedia", media=tuple(frames.values()))
  drawn = {}
  stroked_text = []

  monkeypatch.setattr(
    ClusterUiRenderer,
    "_current_theme",
    lambda self: SimpleNamespace(faint=(0, 0, 0, 0), route_panel_bg=(0, 0, 0, 255)),
  )
  monkeypatch.setattr(ClusterUiRenderer, "_rounded_rect", lambda *args, **kwargs: None)
  monkeypatch.setattr(ClusterUiRenderer, "_draw_navi_crossroad_box", lambda *args, **kwargs: None)
  monkeypatch.setattr(ClusterUiRenderer, "_navi_media_fitted_size", lambda self, frame, rect: (rect.width, rect.height))
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_navi_traffic_light_panel",
    lambda *args, **kwargs: pytest.fail("structured traffic signal fallback must not render"),
  )

  def capture_media(self, frame, rect, **kwargs):
    if frame is not None:
      drawn[frame.key] = rect
    return True

  monkeypatch.setattr(ClusterUiRenderer, "_draw_navi_media", capture_media)
  monkeypatch.setattr(ClusterUiRenderer, "_ellipsize_text", lambda _self, text, *_args: text)
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_text_with_stroke",
    lambda _self, *args, **kwargs: stroked_text.append((args, kwargs)),
  )
  monkeypatch.setattr("cluster_renderer.rl.draw_rectangle_rec", lambda *args, **kwargs: None)
  monkeypatch.setattr("cluster_renderer.rl.begin_scissor_mode", lambda *args, **kwargs: None)
  monkeypatch.setattr("cluster_renderer.rl.end_scissor_mode", lambda: None)

  navi = SimpleNamespace(traffic_light=object(), route=None, vehicle=SimpleNamespace(road_name="Test road"))
  renderer._draw_navi_live_panel(SimpleNamespace(navi_live=navi, navi_dashboard=dashboard))

  assert drawn["image:tbt_current_compact"].width == pytest.approx(310.0 * 1.2)
  assert drawn["image:tbt_current_compact"].height == pytest.approx(116.0 * 1.2)
  assert drawn["image:tbt_next"].width == pytest.approx(190.0 * 1.2)
  assert drawn["image:tbt_next"].height == pytest.approx(68.0 * 1.2)
  assert drawn["image:lane_bottom"].width == pytest.approx(226.0 * 1.2)
  assert drawn["image:lane_bottom"].height == pytest.approx(67.0 * 1.2)
  assert drawn["image:traffic_signal"].width == pytest.approx(230.0)
  assert drawn["image:traffic_signal"].height == pytest.approx(98.0)
  center_icon = drawn["image:center_tbt_icon"]
  center_text = drawn["image:center_tbt_text"]
  assert center_text.x + center_text.width * 0.5 == pytest.approx(center_icon.x + center_icon.width * 0.5)
  assert center_text.y == pytest.approx(center_icon.y + center_icon.height + 2.0)
  assert center_text.width == pytest.approx(220.0)
  assert center_text.height == pytest.approx(78.0)
  assert "image:lane_top" not in drawn
  assert len(stroked_text) == 1
  assert stroked_text[0][0][0] == "Test road"
  assert stroked_text[0][1] == {"anchor": "right", "cache": True}


def test_navi_dashboard_hides_top_lane_and_draws_received_signal(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  frames = {
    key: NaviMediaFrame(key, 1, True, "image/rgba", 1, 1, b"rgba")
    for key in ("image:lane_top", "image:lane_bottom", "image:traffic_signal")
  }
  drawn = {}

  def capture_media(self, frame, rect, **kwargs):
    if frame is not None:
      drawn[frame.key] = rect
    return True

  monkeypatch.setattr(ClusterUiRenderer, "_draw_navi_media", capture_media)

  renderer._draw_navi_map_media(frames)

  assert "image:lane_top" not in drawn
  assert drawn["image:lane_bottom"].height == pytest.approx(174.0)
  assert drawn["image:traffic_signal"].width == pytest.approx(230.0)
  assert drawn["image:traffic_signal"].height == pytest.approx(98.0)


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

  unchanged = source.update(SimpleNamespace(session_id="ipc-session"))

  assert unchanged.media is dashboard.media
  assert unchanged.items is dashboard.items

  source._clear_media()
  assert source._projected_media() == ()


def test_navi_or_trip_report_panel_shifts_3d_camera_modes_left():
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
  )) == 398


def test_turn_signals_center_on_the_active_world_or_road_camera_content():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1280
  renderer.screen_mode = 0
  dashboard = NaviDashboardState(False, "tcp://127.0.0.1:7714")

  assert renderer._turn_signal_center_x_offset(SimpleNamespace(
    camera_view_mode=0,
    navi_live=None,
    navi_dashboard=dashboard,
  ), "left") == pytest.approx(-398)
  assert renderer._turn_signal_center_x_offset(SimpleNamespace(
    camera_view_mode=1,
    navi_live=None,
    navi_dashboard=dashboard,
  ), "right") == pytest.approx(-398)

  road_camera_state = SimpleNamespace(
    camera_view_mode=2,
    navi_live=None,
    navi_dashboard=dashboard,
  )
  left_offset = renderer._turn_signal_center_x_offset(road_camera_state, "left")
  right_offset = renderer._turn_signal_center_x_offset(road_camera_state, "right")
  assert LANE_TURN_SIGNAL_LEFT_CENTER_X + left_offset == pytest.approx(
    CAMERA_BACKGROUND_X + LANE_TURN_SIGNAL_LEFT_CENTER_X * CAMERA_BACKGROUND_W / DESIGN_WIDTH
  )
  assert LANE_TURN_SIGNAL_RIGHT_CENTER_X + right_offset == pytest.approx(
    CAMERA_BACKGROUND_X + LANE_TURN_SIGNAL_RIGHT_CENTER_X * CAMERA_BACKGROUND_W / DESIGN_WIDTH
  )
  assert renderer._turn_signal_center_x_offset(SimpleNamespace(
    camera_view_mode=0,
    navi_live=None,
    navi_dashboard=None,
  ), "left") == pytest.approx(-398)


def test_road_camera_ends_exactly_where_right_navigation_panel_begins():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = DESIGN_WIDTH
  renderer.height = DESIGN_HEIGHT

  camera_rect = renderer._camera_overlay_content_rect()

  assert camera_rect.x == CAMERA_BACKGROUND_X
  assert camera_rect.x + camera_rect.width == NAVI_LIVE_PANEL_X
  assert CAMERA_BACKGROUND_W == NAVI_LIVE_PANEL_X


def test_road_camera_cover_crop_retains_more_of_lower_frame():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = DESIGN_WIDTH
  renderer.height = DESIGN_HEIGHT
  renderer.camera_overlay_pitch_offset_deg = 0.0
  projection = renderer._camera_overlay_projection(SimpleNamespace(
    route_overlay=None,
    camera_calibration_euler=(0.0, 0.0, 0.0),
    camera_device_type="tici",
    camera_sensor="ar0231",
    road_transform_trans=(0.0, 0.0, 1.418),
  ))

  assert projection is not None
  top_crop = projection.dest.y - projection.video_dest.y
  bottom_crop = (
    projection.video_dest.y + projection.video_dest.height
    - projection.dest.y - projection.dest.height
  )
  assert top_crop > bottom_crop
  assert top_crop / (top_crop + bottom_crop) == pytest.approx(0.75)


def test_road_camera_vehicle_frame_uses_vehicle_road_anchor(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  projected = []
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_camera_overlay_screen_xy",
    lambda self, point, projection, scene_shift_x_m=0.0: (projected.append(point), None)[1],
  )
  renderer._draw_camera_overlay_vehicle_frame(
    SimpleNamespace(center=SimpleNamespace(x=0.0, y=10.0)),
    None,
    0.0,
    0,
  )

  assert CAMERA_OVERLAY_VEHICLE_ROAD_HEIGHT_M == pytest.approx(0.025)
  assert projected[0].y == pytest.approx(10.0 + RADAR_TO_CAMERA_M + VEHICLE_LENGTH_M)
  assert projected[0].z == pytest.approx(0.025)


def test_dark_theme_uses_visible_side_gauge_outlines(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  state = SimpleNamespace(camera_view_mode=0)

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: SimpleNamespace(is_dark=True))
  assert renderer._side_gauge_outline(state) == SIDE_GAUGE_OUTLINE

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: SimpleNamespace(is_dark=False))
  assert renderer._side_gauge_outline(state) == (5, 9, 12, 235)
  state.camera_view_mode = 2
  assert renderer._side_gauge_outline(state) == SIDE_GAUGE_OUTLINE


def test_fuel_and_def_level_gauges_remain_enabled(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._current_theme = lambda: SimpleNamespace(
    is_dark=True,
    gauge_midline=(98, 112, 128),
    muted=(150, 160, 172),
  )
  renderer._rounded_rect = lambda *args, **kwargs: None
  renderer._draw_text_with_stroke = lambda *args, **kwargs: None
  level_gauges = []
  renderer._draw_level_gauge = lambda *args, **kwargs: level_gauges.append(args)
  monkeypatch.setattr("cluster_renderer.rl.draw_line_ex", lambda *args, **kwargs: None)
  state = SimpleNamespace(
    camera_view_mode=0,
    accel_mps2=0.0,
    fuel_gauge=0.64,
    energy_gauge_label="fuel",
    steering_output=None,
    steering_output_normalized=None,
    steering_output_kind=None,
    urea_gauge=0.72,
  )

  renderer._draw_accel_block(state)
  renderer._draw_steering_output_block(state)

  assert [(gauge[0], gauge[5]) for gauge in level_gauges] == [
    (SIDE_GAUGE_LEFT_CENTER_X, "fuel"),
    (SIDE_GAUGE_LEFT_CENTER_X + SIDE_GAUGE_COLUMN_GAP, "DEF"),
  ]


@pytest.mark.parametrize("camera_view_mode", (0, 2))
def test_default_and_road_camera_share_fixed_tpms_status(monkeypatch, camera_view_mode):
  renderer = object.__new__(ClusterUiRenderer)
  badges = []
  rects = []

  monkeypatch.setattr(ClusterUiRenderer, "_current_theme", lambda self: DARK_CLUSTER_THEME)
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_rounded_rect",
    lambda self, *args, **kwargs: rects.append(args),
  )
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_compact_tpms_value",
    lambda self, pressure, center_x, center_y: badges.append((pressure, center_x, center_y)),
  )
  monkeypatch.setattr("cluster_renderer.rl.draw_line_ex", lambda *args, **kwargs: None)
  tpms = TpmsInfo(fl=35.0, fr=34.0, rl=33.0, rr=32.0)

  renderer._draw_tpms_status(SimpleNamespace(camera_view_mode=camera_view_mode, tpms=tpms))

  assert badges == [
    (
      35.0,
      TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
    ),
    (
      34.0,
      TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
    ),
    (
      33.0,
      TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
    ),
    (
      32.0,
      TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
    ),
  ]
  car_rect = next(rect for rect in rects if rect[2:4] == (TPMS_STATUS_CAR_W, TPMS_STATUS_CAR_H))
  assert car_rect[0] + car_rect[2] * 0.5 == pytest.approx(TPMS_STATUS_CENTER_X)
  assert car_rect[1] + car_rect[3] * 0.5 == pytest.approx(TPMS_STATUS_CAR_CENTER_Y)
  wheel_rects = [rect for rect in rects if rect[2:4] == (TPMS_STATUS_WHEEL_W, TPMS_STATUS_WHEEL_H)]
  assert len(wheel_rects) == 4
  assert [(rect[0] + rect[2] * 0.5, rect[1] + rect[3] * 0.5) for rect in wheel_rects] == [
    (
      TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
    ),
    (
      TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
    ),
    (
      TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
    ),
    (
      TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
      TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
    ),
  ]


def test_tpms_status_draws_loaded_toy_car_texture_once(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._tpms_car_texture = SimpleNamespace(width=416, height=312)
  texture_draws = []
  badges = []
  monkeypatch.setattr(
    "cluster_renderer.rl.draw_texture_pro",
    lambda *args: texture_draws.append(args),
  )
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_tpms_car_fallback",
    lambda self: pytest.fail("fallback TPMS car drawn"),
  )
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_compact_tpms_value",
    lambda self, pressure, center_x, center_y: badges.append((pressure, center_x, center_y)),
  )

  renderer._draw_tpms_status(SimpleNamespace(tpms=TpmsInfo(fl=35.0, fr=34.0, rl=33.0, rr=32.0)))

  assert len(texture_draws) == 1
  source = texture_draws[0][1]
  destination = texture_draws[0][2]
  assert (source.width, source.height) == (416.0, 312.0)
  assert (destination.width, destination.height) == (TPMS_STATUS_ICON_W, TPMS_STATUS_ICON_H)
  assert destination.x + destination.width * 0.5 == pytest.approx(TPMS_STATUS_CENTER_X)
  assert destination.y + destination.height * 0.5 == pytest.approx(TPMS_STATUS_CAR_CENTER_Y)
  assert len(badges) == 4


def test_tpms_status_hides_when_all_pressures_are_missing(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  draws = []
  monkeypatch.setattr(ClusterUiRenderer, "_rounded_rect", lambda self, *args, **kwargs: draws.append(args))
  monkeypatch.setattr(
    ClusterUiRenderer,
    "_draw_compact_tpms_value",
    lambda self, *args, **kwargs: draws.append(args),
  )

  renderer._draw_tpms_status(SimpleNamespace(tpms=TpmsInfo()))

  assert draws == []


@pytest.mark.parametrize(
  ("theme", "pressure", "expected_text", "expected_color"),
  (
    (DARK_CLUSTER_THEME, 31.0, "31", WHITE),
    (LIGHT_CLUSTER_THEME, 35.0, "35", WHITE),
    (DARK_CLUSTER_THEME, 30.9, "31", RED),
    (LIGHT_CLUSTER_THEME, None, "--", (170, 180, 188, 255)),
  ),
)
def test_compact_tpms_value_uses_theme_and_low_pressure_color(
  theme,
  pressure,
  expected_text,
  expected_color,
):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._current_theme = lambda: theme
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))

  renderer._draw_compact_tpms_value(pressure, 100.0, 200.0)

  assert draws == [(
    (
      expected_text,
      100.0,
      200.0,
      TPMS_STATUS_FONT_SIZE,
      expected_color,
      (0, 0, 0, 255),
      2,
    ),
    {"anchor": "center"},
  )]


@pytest.mark.parametrize(
  ("external_nav_active", "navi_dashboard"),
  (
    (True, None),
    (False, SimpleNamespace(connected=True)),
  ),
)
def test_nav_status_is_centered_below_wifi_and_keeps_clock(
  external_nav_active,
  navi_dashboard,
):
  renderer = object.__new__(ClusterUiRenderer)
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))
  state = SimpleNamespace(
    center_clock_text="12:34:56",
    external_nav_active=external_nav_active,
    navi_dashboard=navi_dashboard,
  )

  renderer._draw_center_clock(state)

  assert draws[0][0][0] == "12:34:56"
  assert draws[1] == ((
    "NAV",
    NAV_STATUS_CENTER_X,
    NAV_STATUS_CENTER_Y,
    NAV_STATUS_FONT_SIZE,
    GREEN,
    (10, 13, 16),
    2,
  ), {"anchor": "center", "cache": True})


def test_nav_status_hides_without_external_navigation():
  renderer = object.__new__(ClusterUiRenderer)
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))

  renderer._draw_center_clock(SimpleNamespace(
    center_clock_text=None,
    external_nav_active=False,
    navi_dashboard=None,
  ))

  assert draws == []


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
      "catalog_revision": 1,
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
      "sent_at_ms": 1001,
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
