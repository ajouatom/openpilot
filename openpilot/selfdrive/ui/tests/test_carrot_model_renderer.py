from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.onroad import model_renderer


class FakeSubMaster(dict):
  def __init__(self, **messages):
    super().__init__(messages)
    self.valid = dict.fromkeys(messages, True)


def blind_spot_messages(
  *,
  left_blindspot: bool = False,
  right_blindspot: bool = False,
  left_status: bool = False,
  right_status: bool = False,
  left_d_rel: float = 100.0,
  right_d_rel: float = 100.0,
  v_ego: float = 10.0,
  lane_change_state=None,
  lane_change_direction: str = "none",
):
  if lane_change_state is None:
    lane_change_state = model_renderer.LaneChangeState.off
  car_state = SimpleNamespace(
    leftBlindspot=left_blindspot,
    rightBlindspot=right_blindspot,
    vEgo=v_ego,
  )
  radar_state = SimpleNamespace(
    leadLeft=SimpleNamespace(status=left_status, dRel=left_d_rel),
    leadRight=SimpleNamespace(status=right_status, dRel=right_d_rel),
  )
  meta = SimpleNamespace(
    laneChangeState=lane_change_state,
    laneChangeDirection=lane_change_direction,
  )
  return car_state, radar_state, meta


@pytest.mark.parametrize(
  "messages, expected",
  [
    (blind_spot_messages(), (False, False, False, False)),
    (blind_spot_messages(left_blindspot=True), (True, False, False, False)),
    (blind_spot_messages(right_blindspot=True), (False, True, False, False)),
    (
      blind_spot_messages(
        left_status=True,
        left_d_rel=29.9,
        lane_change_state=model_renderer.LaneChangeState.preLaneChange,
        lane_change_direction="left",
      ),
      (False, False, True, False),
    ),
    (
      blind_spot_messages(
        right_status=True,
        right_d_rel=30.0,
        lane_change_state=model_renderer.LaneChangeState.preLaneChange,
        lane_change_direction="right",
      ),
      (False, False, False, False),
    ),
    (
      blind_spot_messages(
        left_blindspot=True,
        left_status=True,
        left_d_rel=5.0,
        lane_change_state=model_renderer.LaneChangeState.preLaneChange,
        lane_change_direction="left",
      ),
      (True, False, False, False),
    ),
  ],
)
def test_blind_spot_draw_state(messages, expected):
  assert model_renderer.ModelRenderer._blind_spot_draw_state_carrot(*messages) == expected


def test_blind_spot_inactive_skips_barrier_update():
  renderer = object.__new__(model_renderer.ModelRenderer)
  car_state, radar_state, meta = blind_spot_messages()
  sm = FakeSubMaster(modelV2=SimpleNamespace(meta=meta), carState=car_state, radarState=radar_state)
  updates = []
  renderer._update_blind_spot_barriers_carrot = lambda *args, **kwargs: updates.append((args, kwargs))

  state_mask, state_valid = renderer._draw_blind_spot_carrot(sm)

  assert updates == []
  assert state_mask == 0
  assert state_valid


def test_blind_spot_invalid_state_is_distinct_from_inactive():
  renderer = object.__new__(model_renderer.ModelRenderer)
  car_state, radar_state, meta = blind_spot_messages()
  sm = FakeSubMaster(modelV2=SimpleNamespace(meta=meta), carState=car_state, radarState=radar_state)
  sm.valid["radarState"] = False

  assert renderer._draw_blind_spot_carrot(sm) == (0, False)


@pytest.mark.parametrize("mask", range(16))
def test_blind_spot_state_mask_uses_stable_bits(mask):
  state = tuple(bool(mask & (1 << bit)) for bit in range(4))
  assert model_renderer.ModelRenderer._blind_spot_state_mask_carrot(*state) == mask


def test_blind_spot_updates_only_active_side():
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_lane_barrier_vertices = [np.ones((4, 2), dtype=np.float32), np.ones((4, 2), dtype=np.float32)]
  car_state, radar_state, meta = blind_spot_messages(left_blindspot=True)
  sm = FakeSubMaster(modelV2=SimpleNamespace(meta=meta), carState=car_state, radarState=radar_state)
  updates = []
  draws = []
  renderer._update_blind_spot_barriers_carrot = lambda _, **kwargs: updates.append(kwargs)
  renderer._draw_blind_spot_segments_carrot = lambda points, color: draws.append((points, color))

  state_mask, state_valid = renderer._draw_blind_spot_carrot(sm)

  assert updates == [{"update_left": True, "update_right": False}]
  assert len(draws) == 1
  assert draws[0][0] is renderer._carrot_lane_barrier_vertices[0]
  assert state_mask == model_renderer.BLIND_SPOT_LEFT_MASK
  assert state_valid


class FakeParams:
  def __init__(self):
    self.values = {
      "ShowLaneInfo": 1,
      "CarrotTireTrajectory": False,
      "ShowRadarInfo": 0,
      "ShowPathMode": 9,
      "ShowPathColor": 20,
      "ShowPathModeLane": 14,
      "ShowPathColorLane": 20,
      "ShowPathColorCruiseOff": 19,
    }
    self.calls = []

  def get_int(self, key):
    self.calls.append(key)
    return int(self.values[key])

  def get_bool(self, key):
    self.calls.append(key)
    return bool(self.values[key])


def test_carrot_params_refresh_is_throttled(monkeypatch):
  params = FakeParams()
  monkeypatch.setattr(model_renderer.ui_state, "params", params)
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_params_next_refresh_time = 0.0

  renderer._refresh_carrot_params(0.0)
  assert len(params.calls) == 8
  assert renderer._carrot_show_path_mode_normal == 9

  params.values["ShowPathMode"] = 13
  renderer._refresh_carrot_params(0.999)
  assert len(params.calls) == 8
  assert renderer._carrot_show_path_mode_normal == 9

  renderer._refresh_carrot_params(1.0)
  assert len(params.calls) == 16
  assert renderer._carrot_show_path_mode_normal == 13


def test_carrot_params_failed_refresh_retries(monkeypatch):
  params = FakeParams()
  original_get_int = params.get_int
  failed = False

  def fail_once(key):
    nonlocal failed
    if key == "ShowPathColor" and not failed:
      failed = True
      raise OSError("temporary params read failure")
    return original_get_int(key)

  params.get_int = fail_once
  monkeypatch.setattr(model_renderer.ui_state, "params", params)
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_params_next_refresh_time = 0.0

  with pytest.raises(OSError):
    renderer._refresh_carrot_params(0.0)
  assert renderer._carrot_params_next_refresh_time == 0.0

  renderer._refresh_carrot_params(0.1)
  assert renderer._carrot_params_next_refresh_time == pytest.approx(1.1)


def test_lane_draw_reuses_cached_raw_points():
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_show_lane_info = 1
  renderer._lane_line_probs = np.ones(4, dtype=np.float32)
  renderer._road_edge_stds = np.zeros(2, dtype=np.float32)
  renderer._lane_lines = [
    model_renderer.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(4)
  ]
  renderer._road_edges = [
    model_renderer.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(2)
  ]
  renderer._get_path_length_idx = lambda *_: 1
  projected_inputs = []

  def project(line, *_args, **_kwargs):
    projected_inputs.append(line)
    return np.empty((0, 2), dtype=np.float32)

  renderer._map_line_to_polygon = project
  car_state = SimpleNamespace(leftLaneLine=0, rightLaneLine=0)

  class LaneSubMaster(FakeSubMaster):
    def __getitem__(self, key):
      if key == "modelV2":
        raise AssertionError("lane draw must reuse cached model arrays")
      return super().__getitem__(key)

  sm = LaneSubMaster(modelV2=object(), carState=car_state)
  renderer._draw_lane_lines_carrot(sm)

  expected = [
    renderer._lane_lines[0].raw_points,
    renderer._lane_lines[1].raw_points,
    renderer._lane_lines[2].raw_points,
    renderer._lane_lines[3].raw_points,
  ]
  assert len(projected_inputs) == len(expected)
  assert all(actual is cached for actual, cached in zip(projected_inputs, expected, strict=True))


def test_lane_draw_builds_cached_optional_geometry():
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_show_lane_info = 2
  renderer._lane_line_probs = np.ones(4, dtype=np.float32)
  renderer._road_edge_stds = np.zeros(2, dtype=np.float32)
  renderer._lane_lines = [
    model_renderer.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(4)
  ]
  renderer._road_edges = [
    model_renderer.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(2)
  ]
  renderer._get_path_length_idx = lambda *_: 1
  projected_inputs = []

  def project(line, *_args, **_kwargs):
    projected_inputs.append(line)
    return np.empty((0, 2), dtype=np.float32)

  renderer._map_line_to_polygon = project
  car_state = SimpleNamespace(leftLaneLine=24, rightLaneLine=0)

  class LaneSubMaster(FakeSubMaster):
    def __getitem__(self, key):
      if key == "modelV2":
        raise AssertionError("lane draw must reuse cached model arrays")
      return super().__getitem__(key)

  sm = LaneSubMaster(modelV2=object(), carState=car_state)
  renderer._draw_lane_lines_carrot(sm)

  expected = [
    renderer._lane_lines[0].raw_points,
    renderer._lane_lines[1].raw_points,
    renderer._lane_lines[1].raw_points,
    renderer._lane_lines[2].raw_points,
    renderer._lane_lines[3].raw_points,
    renderer._road_edges[0].raw_points,
    renderer._road_edges[1].raw_points,
  ]
  assert len(projected_inputs) == len(expected)
  assert all(actual is cached for actual, cached in zip(projected_inputs, expected, strict=True))


def test_model_overlay_timings(monkeypatch):
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._render_timings = model_renderer.ModelRenderTimings()
  calls = []
  renderer._draw_path_carrot = lambda _: calls.append("path")
  renderer._draw_lane_center_indicator_carrot = lambda _: calls.append("laneCenter")
  renderer._draw_lane_lines_carrot = lambda _: calls.append("laneLines")
  def draw_blind_spot(_):
    calls.append("blindSpot")
    return model_renderer.BLIND_SPOT_LEFT_MASK, True

  renderer._draw_blind_spot_carrot = draw_blind_spot
  renderer._draw_radar_info_carrot = lambda _: calls.append("radar")
  timestamps = iter((1_000_000_000, 1_010_000_000, 1_030_000_000, 1_034_000_000, 1_035_000_000))
  monkeypatch.setattr(model_renderer.time, "monotonic_ns", lambda: next(timestamps))

  renderer._draw_carrot_overlays(object())

  assert calls == ["path", "laneCenter", "laneLines", "blindSpot", "radar"]
  assert renderer.render_timings.path_time_millis == pytest.approx(10.0)
  assert renderer.render_timings.lane_time_millis == pytest.approx(20.0)
  assert renderer.render_timings.blind_spot_time_millis == pytest.approx(4.0)
  assert renderer.render_timings.radar_time_millis == pytest.approx(1.0)
  assert renderer.render_timings.valid
  assert renderer.render_timings.blind_spot_state_mask == model_renderer.BLIND_SPOT_LEFT_MASK
  assert renderer.render_timings.blind_spot_state_valid


def test_render_early_return_resets_timings(monkeypatch):
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._render_timings = model_renderer.ModelRenderTimings(1.0, 2.0, 3.0, 4.0, True, 15, True)
  sm = SimpleNamespace(recv_frame={"liveCalibration": 0, "modelV2": 0})
  monkeypatch.setattr(model_renderer.ui_state, "sm", sm, raising=False)
  monkeypatch.setattr(model_renderer.ui_state, "started_frame", 1, raising=False)

  renderer._render(object())

  assert renderer.render_timings == model_renderer.ModelRenderTimings()
