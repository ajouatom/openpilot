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

  renderer._draw_blind_spot_carrot(sm)

  assert updates == []


def test_blind_spot_invalid_input_skips_barrier_update():
  renderer = object.__new__(model_renderer.ModelRenderer)
  car_state, radar_state, meta = blind_spot_messages(left_blindspot=True)
  sm = FakeSubMaster(modelV2=SimpleNamespace(meta=meta), carState=car_state, radarState=radar_state)
  sm.valid["radarState"] = False
  updates = []
  renderer._update_blind_spot_barriers_carrot = lambda *args, **kwargs: updates.append((args, kwargs))

  renderer._draw_blind_spot_carrot(sm)

  assert updates == []


def test_blind_spot_updates_only_active_side():
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_lane_barrier_vertices = [np.ones((4, 2), dtype=np.float32), np.ones((4, 2), dtype=np.float32)]
  car_state, radar_state, meta = blind_spot_messages(left_blindspot=True)
  sm = FakeSubMaster(modelV2=SimpleNamespace(meta=meta), carState=car_state, radarState=radar_state)
  updates = []
  draws = []
  renderer._update_blind_spot_barriers_carrot = lambda _, **kwargs: updates.append(kwargs)
  renderer._draw_blind_spot_segments_carrot = lambda points, color: draws.append((points, color))

  renderer._draw_blind_spot_carrot(sm)

  assert updates == [{"update_left": True, "update_right": False}]
  assert len(draws) == 1
  assert draws[0][0] is renderer._carrot_lane_barrier_vertices[0]


class FakeParams:
  def __init__(self):
    self.values = {
      "ShowLaneInfo": 1,
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


def test_carrot_params_refresh_is_throttled(monkeypatch):
  params = FakeParams()
  monkeypatch.setattr(model_renderer.ui_state, "params", params)
  renderer = object.__new__(model_renderer.ModelRenderer)
  renderer._carrot_params_next_refresh_time = 0.0

  renderer._refresh_carrot_params(0.0)
  assert len(params.calls) == 7
  assert renderer._carrot_show_path_mode_normal == 9

  params.values["ShowPathMode"] = 13
  renderer._refresh_carrot_params(0.999)
  assert len(params.calls) == 7
  assert renderer._carrot_show_path_mode_normal == 9

  renderer._refresh_carrot_params(1.0)
  assert len(params.calls) == 14
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
  car_state = SimpleNamespace(leftLaneLine=1, rightLaneLine=1)

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
  car_state = SimpleNamespace(leftLaneLine=24, rightLaneLine=1)

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


def test_model_overlay_draw_order():
  renderer = object.__new__(model_renderer.ModelRenderer)
  calls = []
  renderer._draw_path_carrot = lambda _: calls.append("path")
  renderer._draw_lane_lines_carrot = lambda _: calls.append("laneLines")
  renderer._draw_blind_spot_carrot = lambda _: calls.append("blindSpot")
  renderer._draw_radar_info_carrot = lambda _: calls.append("radar")

  renderer._draw_carrot_overlays(object())

  assert calls == ["path", "laneLines", "blindSpot", "radar"]


def test_render_stale_data_skips_overlays(monkeypatch):
  renderer = object.__new__(model_renderer.ModelRenderer)
  calls = []
  renderer._draw_carrot_overlays = lambda _: calls.append("overlays")
  sm = SimpleNamespace(recv_frame={"liveCalibration": 0, "modelV2": 0})
  monkeypatch.setattr(model_renderer.ui_state, "sm", sm, raising=False)
  monkeypatch.setattr(model_renderer.ui_state, "started_frame", 1, raising=False)

  renderer._render(object())

  assert calls == []
