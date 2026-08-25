import ast
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
  CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
  CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
  CLUSTER_RADAR_INFO_NONE,
  EGO_FORWARD_M,
  GREEN,
  LIGHT_CLUSTER_THEME,
  VEHICLE_LENGTH_M,
)
from cluster_models import ClusterAlert, ClusterUiState, DetectedVehicle, LaneMarking, ModelPathPoint, RadarPoint, RouteOverlay
import cluster_renderer
from cluster_renderer import ClusterUiRenderer
import cluster_scene
from cluster_scene import (
  MeshStrip,
  RadarPointMarker,
  SCENE_STATE_FIELDS,
  Vec3,
  VehicleBox,
  build_cluster_scene,
  cluster_scene_state_key,
  detected_vehicle_scene_forward_m,
  render_scene_forward_m,
)


def test_road_camera_radar_point_uses_simple_source_colored_square(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  squares = []
  color = (44, 211, 112)
  point = RadarPointMarker(
    center=Vec3(1.0, 25.0, 0.2),
    radius_m=0.2,
    color=color,
    label="R1",
    longitudinal_m=25.0,
    lateral_m=1.0,
  )

  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", lambda *_args: (100.0, 80.0))
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rec", lambda rect, color: squares.append((rect, color)))
  monkeypatch.setattr(
    cluster_renderer.rl,
    "draw_rectangle_rounded_lines_ex",
    lambda *_args: pytest.fail("rounded raw radar marker drawn"),
  )

  renderer._draw_camera_overlay_radar_point(point, object(), 0.0, CLUSTER_RADAR_INFO_NONE)

  assert len(squares) == 1
  rect, square_color = squares[0]
  assert rect.width == rect.height
  assert (square_color.r, square_color.g, square_color.b) == color


def test_cluster_alert_is_centered_in_swapped_camera_panel(monkeypatch) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = cluster_renderer.DESIGN_WIDTH
  renderer.height = cluster_renderer.DESIGN_HEIGHT
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  renderer.screen_mode = cluster_renderer.CLUSTER_SCREEN_MODE_DEFAULT
  labels = []

  monkeypatch.setattr(cluster_renderer.rl, "rl_push_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_scalef", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_pop_matrix", lambda: None)
  monkeypatch.setattr(renderer, "_rounded_rect", lambda *_args: pytest.fail("alert background drawn"))
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rec", lambda *_args: pytest.fail("alert background drawn"))
  monkeypatch.setattr(renderer, "_fit_alert_text", lambda text, size, *_args: (text, size))
  monkeypatch.setattr(renderer, "_draw_text_with_stroke", lambda *args, **kwargs: labels.append((args, kwargs)))

  renderer._draw_alert_overlay(ClusterAlert("Steering Unavailable", "Take Control", size=2, status=1))

  camera_center_x = cluster_renderer.NAVI_LIVE_PANEL_W + cluster_renderer.CAMERA_BACKGROUND_W * 0.5
  assert {args[1] for args, _kwargs in labels} == {camera_center_x}
  assert [args[0] for args, _kwargs in labels] == ["Steering Unavailable", "Take Control"]
  assert all(args[5] == cluster_renderer.CLUSTER_ALERT_TEXT_STROKE for args, _kwargs in labels)


def test_synthetic_cluster_alert_uses_cluster_language() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer.language = "ko"

  title, detail = renderer._cluster_alert_text(ClusterAlert(
    "TAKE CONTROL IMMEDIATELY",
    "System Unresponsive",
    size=3,
    status=2,
    alert_type="clusterSelfdriveTimeout",
  ))

  assert title == "즉시 운전대를 잡으세요"
  assert detail == "시스템 응답 없음"


def test_road_camera_radar_point_keeps_optional_text(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.is_metric = True
  labels = []
  point = RadarPointMarker(
    center=Vec3(1.0, 25.0, 0.2),
    radius_m=0.2,
    color=(44, 211, 112),
    label="R1",
    longitudinal_m=25.0,
    lateral_m=1.0,
    absolute_speed_kph=50.0,
  )

  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", lambda *_args: (100.0, 80.0))
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rec", lambda *_args: None)
  monkeypatch.setattr(renderer, "_draw_world_label_text", lambda *args, **_kwargs: labels.append(args[0]))

  renderer._draw_camera_overlay_radar_point(point, object(), 0.0, CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE)

  assert labels == ["25 m 50 km/h"]


def test_road_camera_detected_vehicle_uses_transparent_colored_rounded_frame(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  outlines = []
  projected = iter((
    (100.0, 100.0),
    (80.0, 100.0),
    (120.0, 100.0),
    (80.0, 50.0),
    (120.0, 50.0),
  ))
  vehicle = VehicleBox(
    center=Vec3(0.0, 25.0, 0.0),
    right_x=1.0,
    right_y=0.0,
    forward_x=0.0,
    forward_y=1.0,
    width_m=2.0,
    length_m=4.5,
    height_m=1.5,
    body_color=(255, 255, 255),
    side_color=(255, 255, 255),
    rear_color=(255, 255, 255),
    top_highlight=(255, 255, 255),
    outline_color=(255, 255, 255),
    source="cornerRadar",
    longitudinal_m=25.0,
  )

  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", lambda *_args: next(projected))
  monkeypatch.setattr(
    cluster_renderer.rl,
    "draw_rectangle_rounded_lines_ex",
    lambda rect, roundness, segments, width, outline: outlines.append((rect, roundness, segments, width, outline)),
  )
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rec", lambda *_args: pytest.fail("filled marker drawn"))
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rounded", lambda *_args: pytest.fail("filled marker drawn"))

  projection = SimpleNamespace(dest=cluster_renderer.rl.Rectangle(0.0, 0.0, 300.0, 200.0))
  renderer._draw_camera_overlay_vehicle_frame(vehicle, projection, 0.0, CLUSTER_RADAR_INFO_NONE)

  assert len(outlines) == 1
  marker = outlines[0][0]
  assert marker.height > marker.width
  assert outlines[0][1] > 0.0
  assert outlines[0][2] == cluster_renderer.CAMERA_OVERLAY_FRAME_ROUND_SEGMENTS
  _expected_fill, _expected_side, expected_ring = cluster_renderer.camera_overlay_vehicle_coin_colors(
    vehicle,
    False,
    False,
  )
  assert (outlines[0][4].r, outlines[0][4].g, outlines[0][4].b) == expected_ring


def test_road_camera_vehicle_frame_rejects_incomplete_or_edge_clipped_projection(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  outlines = []
  projection = SimpleNamespace(dest=cluster_renderer.rl.Rectangle(0.0, 0.0, 300.0, 200.0))
  vehicle = VehicleBox(
    center=Vec3(0.0, 25.0, 0.0),
    right_x=0.0,
    right_y=1.0,
    forward_x=-1.0,
    forward_y=0.0,
    width_m=2.0,
    length_m=4.5,
    height_m=1.5,
    body_color=(255, 255, 255),
    side_color=(255, 255, 255),
    rear_color=(255, 255, 255),
    top_highlight=(255, 255, 255),
    outline_color=(255, 255, 255),
    source="cornerRadar",
    longitudinal_m=25.0,
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "draw_rectangle_rounded_lines_ex",
    lambda *args: outlines.append(args),
  )

  incomplete = iter((
    (150.0, 100.0),
    (130.0, 100.0),
    None,
    (130.0, 50.0),
    (170.0, 50.0),
  ))
  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", lambda *_args: next(incomplete))
  renderer._draw_camera_overlay_vehicle_frame(vehicle, projection, 0.0, CLUSTER_RADAR_INFO_NONE)

  edge_clipped = iter((
    (5.0, 100.0),
    (-15.0, 100.0),
    (25.0, 100.0),
    (-15.0, 50.0),
    (25.0, 50.0),
  ))
  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", lambda *_args: next(edge_clipped))
  renderer._draw_camera_overlay_vehicle_frame(vehicle, projection, 0.0, CLUSTER_RADAR_INFO_NONE)

  assert outlines == []


def test_road_camera_vehicle_frame_ignores_radar_yaw_for_screen_box_width(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  projected_road_points = []
  vehicle = VehicleBox(
    center=Vec3(2.0, 25.0, 0.0),
    right_x=0.0,
    right_y=1.0,
    forward_x=-1.0,
    forward_y=0.0,
    width_m=2.0,
    length_m=4.5,
    height_m=1.5,
    body_color=(255, 255, 255),
    side_color=(255, 255, 255),
    rear_color=(255, 255, 255),
    top_highlight=(255, 255, 255),
    outline_color=(255, 255, 255),
    source="cornerRadar",
    longitudinal_m=25.0,
  )

  def project(_self, point, _projection, _scene_shift_x_m=0.0):
    projected_road_points.append(point)
    return 150.0 + point.x * 10.0, 100.0 - point.z * 20.0

  monkeypatch.setattr(ClusterUiRenderer, "_camera_overlay_screen_xy", project)
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rounded_lines_ex", lambda *_args: None)
  projection = SimpleNamespace(dest=cluster_renderer.rl.Rectangle(0.0, 0.0, 300.0, 200.0))
  renderer._draw_camera_overlay_vehicle_frame(vehicle, projection, 0.0, CLUSTER_RADAR_INFO_NONE)

  left_base, right_base = projected_road_points[1:3]
  assert left_base.y == pytest.approx(right_base.y)
  assert left_base.x < vehicle.center.x < right_base.x


def test_road_camera_strip_projects_each_endpoint_once_and_reuses_buffers(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._camera_overlay_strip_points = None
  renderer._camera_overlay_strip_point_capacity = 0
  renderer._camera_overlay_strip_pair_visibility = bytearray()
  projected = []
  batches = []
  strip = MeshStrip(
    left=(Vec3(-1.0, 10.0), Vec3(-2.0, 20.0), Vec3(-3.0, 30.0)),
    right=(Vec3(1.0, 10.0), Vec3(2.0, 20.0), Vec3(3.0, 30.0)),
    color=(10, 20, 30, 40),
    x_offset_m=0.25,
  )

  def project(point, _projection, scene_shift_x_m=0.0):
    projected.append((point, scene_shift_x_m))
    return point.x * 10.0, point.y

  def draw_strip(points, point_count, _color):
    batches.append(tuple((points[index].x, points[index].y) for index in range(point_count)))

  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", project)
  renderer._raw_draw_triangle_strip_2d = draw_strip

  renderer._draw_camera_overlay_strip(strip, object(), 0.5)
  point_buffer = renderer._camera_overlay_strip_points
  visibility_buffer = renderer._camera_overlay_strip_pair_visibility
  renderer._draw_camera_overlay_strip(strip, object(), 0.5)

  expected_batch = (
    (-10.0, 10.0), (10.0, 10.0),
    (-20.0, 20.0), (20.0, 20.0),
    (-30.0, 30.0), (30.0, 30.0),
  )
  assert batches == [expected_batch, expected_batch]
  assert len(projected) == 12
  assert all(offset == pytest.approx(0.75) for _point, offset in projected)
  assert renderer._camera_overlay_strip_points is point_buffer
  assert renderer._camera_overlay_strip_pair_visibility is visibility_buffer


def test_road_camera_strip_batches_only_contiguous_visible_pairs(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._camera_overlay_strip_points = None
  renderer._camera_overlay_strip_point_capacity = 0
  renderer._camera_overlay_strip_pair_visibility = bytearray()
  batches = []
  strip = MeshStrip(
    left=tuple(Vec3(-1.0, float(index)) for index in range(6)),
    right=tuple(Vec3(1.0, float(index)) for index in range(6)),
    color=(255, 255, 255, 255),
  )

  def project(point, _projection, _scene_shift_x_m=0.0):
    if point.y == 2.0:
      return None
    return point.x, point.y

  def draw_strip(points, point_count, _color):
    batches.append(tuple((points[index].x, points[index].y) for index in range(point_count)))

  monkeypatch.setattr(renderer, "_camera_overlay_screen_xy", project)
  renderer._raw_draw_triangle_strip_2d = draw_strip
  renderer._draw_camera_overlay_strip(strip, object(), 0.0)

  assert batches == [
    ((-1.0, 0.0), (1.0, 0.0), (-1.0, 1.0), (1.0, 1.0)),
    ((-1.0, 3.0), (1.0, 3.0), (-1.0, 4.0), (1.0, 4.0), (-1.0, 5.0), (1.0, 5.0)),
  ]


def test_road_camera_strip_fallback_preserves_original_triangle_order(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._camera_overlay_strip_points = None
  renderer._camera_overlay_strip_point_capacity = 0
  renderer._camera_overlay_strip_pair_visibility = bytearray()
  renderer._raw_draw_triangle_strip_2d = None
  triangles = []
  strip = MeshStrip(
    left=(Vec3(-1.0, 10.0), Vec3(-2.0, 20.0), Vec3(-3.0, 30.0)),
    right=(Vec3(1.0, 10.0), Vec3(2.0, 20.0), Vec3(3.0, 30.0)),
    color=(10, 20, 30, 40),
  )

  monkeypatch.setattr(
    renderer,
    "_camera_overlay_screen_xy",
    lambda point, _projection, _scene_shift_x_m=0.0: (point.x, point.y),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "draw_triangle",
    lambda p0, p1, p2, _color: triangles.append(((p0.x, p0.y), (p1.x, p1.y), (p2.x, p2.y))),
  )

  renderer._draw_camera_overlay_strip(strip, object(), 0.0)

  assert triangles == [
    ((-1.0, 10.0), (1.0, 10.0), (2.0, 20.0)),
    ((-1.0, 10.0), (2.0, 20.0), (-2.0, 20.0)),
    ((-2.0, 20.0), (2.0, 20.0), (3.0, 30.0)),
    ((-2.0, 20.0), (3.0, 30.0), (-3.0, 30.0)),
  ]


def test_road_camera_world_reuses_one_projection_for_background_and_overlay(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.profile_enabled = False
  state = SimpleNamespace(camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA, onroad=True)
  theme = SimpleNamespace(bg=(1, 2, 3, 255))
  scene = object()
  projection = object()
  projection_calls = []
  background_calls = []
  overlay_calls = []

  monkeypatch.setattr(renderer, "_current_theme", lambda: theme)
  monkeypatch.setattr(renderer, "_highlight_lane_lit", lambda *_args: False)
  monkeypatch.setattr(renderer, "_scene_for_state", lambda *_args: scene)
  monkeypatch.setattr(
    renderer,
    "_camera_overlay_projection",
    lambda value: projection_calls.append(value) or projection,
  )
  monkeypatch.setattr(renderer, "_draw_camera_background", lambda *args: background_calls.append(args))
  monkeypatch.setattr(renderer, "_draw_camera_projected_overlay", lambda *args: overlay_calls.append(args))
  monkeypatch.setattr(cluster_renderer.rl, "clear_background", lambda _color: None)

  renderer._render_world(state, (False, False))

  assert projection_calls == [state]
  assert background_calls == [(state, projection)]
  assert overlay_calls == [(scene, state, projection)]


def _cluster_state(**changes) -> ClusterUiState:
  state = ClusterUiState(
    speed_kph=60.0,
    accel_mps2=0.0,
    steering=0.0,
    speed_limit_kph=None,
    speed_limit_source=None,
    cruise_kph=None,
    cruise_display_state="off",
    gear_text=None,
    cruise_gap=None,
    lfa_active=None,
    left_signal=False,
    right_signal=False,
    left_blindspot=False,
    right_blindspot=False,
    lane_change=None,
    lane_change_phase="off",
    lane_change_progress=0.0,
    highlight_lane=None,
    highlight_lane_offset=None,
    ego_lane_offset=0.0,
    road_view_lane_position=0.0,
    camera_lane_center_offset_m=None,
    lane_width_m=3.6,
    steering_angle_deg=0.0,
    surround_yaw_deg=0.0,
    surround_pitch_deg=0.0,
    surround_view_active=False,
    lanes=(LaneMarking(-1.8), LaneMarking(1.8)),
  )
  return replace(state, **changes)


def _max_scene_forward_m(strips) -> float:
  return max(
    point.y
    for strip in strips
    for side in (strip.left, strip.right)
    for point in side
  )


def test_scene_cache_key_covers_every_cluster_scene_state_access() -> None:
  tree = ast.parse(Path(cluster_scene.__file__).read_text(encoding="utf-8"))
  accessed_fields = {
    node.attr
    for node in ast.walk(tree)
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "state"
  }

  assert accessed_fields == set(SCENE_STATE_FIELDS)


def test_renderer_reuses_scene_when_only_hud_state_changes(monkeypatch) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._scene_cache_key = None
  renderer._scene_cache = None
  renderer.profile_enabled = False
  scenes = []

  def build_scene(*_args, **_kwargs):
    scene = SimpleNamespace(sequence=len(scenes))
    scenes.append(scene)
    return scene

  monkeypatch.setattr(cluster_renderer, "build_cluster_scene", build_scene)
  state = _cluster_state()
  first = renderer._scene_for_state(state, True, LIGHT_CLUSTER_THEME)
  hud_only = renderer._scene_for_state(
    replace(state, center_clock_text="12:34", ev_mode_valid=True, ev_mode_active=True, driving_mode=2),
    True,
    LIGHT_CLUSTER_THEME,
  )
  changed_world = renderer._scene_for_state(replace(state, steering=0.1), True, LIGHT_CLUSTER_THEME)

  assert cluster_scene_state_key(state) == cluster_scene_state_key(
    replace(state, center_clock_text="12:34", ev_mode_valid=True, ev_mode_active=True, driving_mode=2)
  )
  assert first is hud_only
  assert changed_world is not first
  assert len(scenes) == 2


@pytest.mark.parametrize(
  ("valid", "active", "expected_draws"),
  ((False, False, 0), (False, True, 0), (True, False, 0), (True, True, 1)),
)
def test_ev_mode_indicator_draws_green_only_when_valid_and_active(valid, active, expected_draws) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))

  renderer._draw_ev_mode_indicator(_cluster_state(ev_mode_valid=valid, ev_mode_active=active))

  assert len(draws) == expected_draws
  if draws:
    args, kwargs = draws[0]
    assert args[0] == "EV"
    assert args[1:4] == (
      cluster_renderer.SPEED_EV_CENTER_X,
      cluster_renderer.SPEED_EV_CENTER_Y,
      cluster_renderer.SPEED_EV_FONT_SIZE,
    )
    assert args[4] == GREEN
    assert kwargs == {"anchor": "center", "cache": True}


def test_ev_mode_indicator_fits_between_three_digit_speeds() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._current_theme = lambda: LIGHT_CLUSTER_THEME
  renderer._speed_bg_texture = None
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))

  state = _cluster_state(
    speed_kph=260.0,
    display_speed_kph=260.0,
    cruise_kph=260,
    cruise_display_state="engaged",
    ev_mode_valid=True,
    ev_mode_active=True,
  )
  renderer._draw_speed_block(state)

  speed = next(args for args, _ in draws if args[0] == "260" and args[1] == cluster_renderer.SPEED_VALUE_CENTER_X)
  cruise = next(args for args, _ in draws if args[0] == "260" and args[1] == cluster_renderer.CRUISE_SET_SPEED_CENTER_X)
  ev = next(args for args, _ in draws if args[0] == "EV")

  # Width ratios measured from the bundled cluster font. Include each 2D stroke
  # and preserve visible separation at the real three-digit domain maximum.
  speed_width = speed[3] * (138.89345 / 77.056)
  cruise_width = cruise[3] * (83.78001 / 46.4)
  ev_width = ev[3] * (30.4 / 24.0)
  speed_right = speed[1] + speed_width * 0.5 + speed[6]
  ev_left = ev[1] - ev_width * 0.5 - ev[6]
  ev_right = ev[1] + ev_width * 0.5 + ev[6]
  cruise_left = cruise[1] - cruise_width * 0.5 - cruise[6]

  assert ev[1:4] == (181.0, cluster_renderer.SPEED_VALUE_CENTER_Y, 28.0)
  assert ev_left - speed_right >= 3.0
  assert cruise_left - ev_right >= 3.0


@pytest.mark.parametrize(("active", "expected_draws"), ((False, 0), (True, 1)))
def test_egpu_indicator_draws_only_while_active(monkeypatch, active, expected_draws) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  texts = []
  renderer._draw_text = lambda *args, **kwargs: texts.append((args, kwargs))
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rounded", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "draw_rectangle_rounded_lines_ex", lambda *_args: None)

  renderer._draw_egpu_status(_cluster_state(egpu_active=active))

  assert len(texts) == expected_draws
  if texts:
    assert texts[0][0][0] == "eGPU"
    assert texts[0][0][4] == GREEN
    assert texts[0][1] == {"anchor": "center"}


@pytest.mark.parametrize(
  ("mode", "label", "color"),
  (
    (1, "연비", (0, 255, 0, 200)),
    (2, "안전", (255, 165, 0, 200)),
    (3, "일반", (255, 255, 255, 200)),
    (4, "고속", (255, 0, 0, 200)),
  ),
)
def test_driving_mode_indicator_matches_c3x_style(mode, label, color) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  boxes = []
  texts = []
  renderer._rounded_rect = lambda *args, **kwargs: boxes.append((args, kwargs))
  renderer._draw_text_with_stroke = lambda *args, **kwargs: texts.append((args, kwargs))

  renderer._draw_driving_mode_indicator(_cluster_state(driving_mode=mode))

  assert boxes == []
  assert texts == [((
    label,
    cluster_renderer.SPEED_DRIVING_MODE_CENTER_X,
    cluster_renderer.SPEED_DRIVING_MODE_CENTER_Y,
    cluster_renderer.SPEED_DRIVING_MODE_FONT_SIZE,
    color,
    (5, 9, 12),
    2,
  ), {"anchor": "center", "cache": True})]


@pytest.mark.parametrize("mode", (None, 0, 5))
def test_driving_mode_indicator_hides_unknown_values(mode) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  draws = []
  renderer._rounded_rect = lambda *args, **kwargs: draws.append((args, kwargs))
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append((args, kwargs))

  renderer._draw_driving_mode_indicator(_cluster_state(driving_mode=mode))

  assert draws == []


@pytest.mark.parametrize("traffic_state", (1, 2))
def test_traffic_states_share_the_slot_beside_driving_mode(monkeypatch, traffic_state) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._traffic_red_texture = SimpleNamespace(width=56, height=56)
  renderer._traffic_green_texture = SimpleNamespace(width=56, height=56)
  draws = []
  monkeypatch.setattr(cluster_renderer.rl, "draw_texture_pro", lambda *args: draws.append(args))

  renderer._draw_model_traffic_state(traffic_state)

  assert len(draws) == 1
  destination = draws[0][2]
  assert destination.x == pytest.approx(
    cluster_renderer.SPEED_MODEL_TRAFFIC_CENTER_X - cluster_renderer.SPEED_MODEL_TRAFFIC_ICON_SIZE * 0.5
  )
  assert destination.y == pytest.approx(
    cluster_renderer.SPEED_MODEL_TRAFFIC_CENTER_Y - cluster_renderer.SPEED_MODEL_TRAFFIC_ICON_SIZE * 0.5
  )
  assert destination.width == pytest.approx(cluster_renderer.SPEED_MODEL_TRAFFIC_ICON_SIZE)
  assert destination.height == pytest.approx(cluster_renderer.SPEED_MODEL_TRAFFIC_ICON_SIZE)
  traffic_right = destination.x + destination.width
  assert cluster_renderer.SPEED_DRIVING_MODE_CENTER_X - traffic_right == pytest.approx(
    cluster_renderer.SPEED_DRIVING_MODE_BASE_CENTER_OFFSET_X
  )


def test_top_status_icons_keep_outer_and_inter_icon_margins() -> None:
  lfa_lane_left = (
    cluster_renderer.LFA_STATUS_CENTER_X
    - cluster_renderer.LFA_STATUS_ICON_SIZE * cluster_renderer.LFA_LANE_ICON_WIDTH_SCALE * 0.5
  )
  lfa_lane_right = (
    cluster_renderer.LFA_STATUS_CENTER_X
    + cluster_renderer.LFA_STATUS_ICON_SIZE * cluster_renderer.LFA_LANE_ICON_WIDTH_SCALE * 0.5
  )
  wifi_left = cluster_renderer.WIFI_STATUS_CENTER_X - cluster_renderer.WIFI_STATUS_ICON_SIZE * 0.5
  wifi_right = cluster_renderer.WIFI_STATUS_CENTER_X + cluster_renderer.WIFI_STATUS_ICON_SIZE * 0.5

  assert lfa_lane_left >= 12.0
  assert wifi_left - lfa_lane_right >= 6.0
  assert cluster_renderer.TOP_CLOCK_CENTER_X - wifi_right >= 100.0


def test_speed_gear_badge_has_transparent_interior() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  boxes = []
  renderer._current_theme = lambda: LIGHT_CLUSTER_THEME
  renderer._rounded_rect = lambda *args, **kwargs: boxes.append((args, kwargs))
  renderer._draw_text_with_stroke = lambda *_args, **_kwargs: None

  renderer._draw_speed_gear_badge(_cluster_state(gear_text="D", camera_view_mode=2))

  assert len(boxes) == 1
  assert boxes[0][0][5] == (0, 0, 0, 0)


def test_speed_block_draws_driving_mode_before_traffic_state() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._current_theme = lambda: LIGHT_CLUSTER_THEME
  renderer._speed_bg_texture = None
  renderer._draw_text_with_stroke = lambda *_args, **_kwargs: None
  renderer._draw_cruise_gap_badge = lambda *_args, **_kwargs: None
  renderer._draw_speed_gear_badge = lambda *_args, **_kwargs: None
  renderer._draw_ev_mode_indicator = lambda *_args, **_kwargs: None
  renderer._draw_tpms_status = lambda *_args, **_kwargs: None
  order = []
  renderer._draw_driving_mode_indicator = lambda *_args, **_kwargs: order.append("driving_mode")
  renderer._draw_model_traffic_state = lambda *_args, **_kwargs: order.append("traffic")

  renderer._draw_speed_block(_cluster_state(driving_mode=2, traffic_state=2))

  assert order == ["driving_mode", "traffic"]


def test_full_navi_does_not_draw_speed_mode_indicators() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._current_theme = lambda: SimpleNamespace(text=(1, 2, 3), muted=(4, 5, 6))
  renderer._draw_navi_media = lambda *_args, **_kwargs: False
  renderer._draw_text = lambda *_args, **_kwargs: None
  draws = []
  renderer._draw_ev_mode_indicator = lambda *args, **kwargs: draws.append((args, kwargs))
  renderer._draw_driving_mode_indicator = lambda *args, **kwargs: draws.append((args, kwargs))

  state = _cluster_state(ev_mode_valid=True, ev_mode_active=True, driving_mode=2)
  renderer._draw_navi_left_band(state, None, {})

  assert draws == []


def test_longitudinal_render_distance_is_halved_without_changing_lateral_data() -> None:
  vehicle = DetectedVehicle(
    "L1",
    longitudinal_m=40.0,
    lateral_m=2.25,
    source="radarState",
    primary=True,
  )

  state = _cluster_state(detected_vehicles=(vehicle,))
  assert vehicle.lateral_m == 2.25
  assert render_scene_forward_m(vehicle.longitudinal_m, state) == pytest.approx(EGO_FORWARD_M + 20.0)
  assert render_scene_forward_m(-20.0, state) == pytest.approx(EGO_FORWARD_M - 10.0)
  assert detected_vehicle_scene_forward_m(vehicle, state) == pytest.approx(
    EGO_FORWARD_M + 20.0 + VEHICLE_LENGTH_M * 0.5
  )
  scene = build_cluster_scene(state)
  detected_box = next(box for box in scene.vehicles if box.label == "L1")
  assert detected_box.center.x == pytest.approx(2.25)
  assert detected_box.center.y == pytest.approx(EGO_FORWARD_M + 20.0 + VEHICLE_LENGTH_M * 0.5)
  assert detected_box.longitudinal_m == 40.0


@pytest.mark.parametrize("camera_view_mode", (
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
))
def test_camera_background_modes_keep_longitudinal_render_distance_one_to_one(camera_view_mode) -> None:
  vehicle = DetectedVehicle(
    "L1",
    longitudinal_m=40.0,
    lateral_m=2.25,
    source="radarState",
    primary=True,
  )
  state = _cluster_state(
    camera_view_mode=camera_view_mode,
    detected_vehicles=(vehicle,),
  )

  assert render_scene_forward_m(vehicle.longitudinal_m, state) == pytest.approx(EGO_FORWARD_M + 40.0)
  assert render_scene_forward_m(-20.0, state) == pytest.approx(EGO_FORWARD_M - 20.0)
  scene = build_cluster_scene(state)
  detected_box = next(box for box in scene.vehicles if box.label == "L1")
  assert detected_box.center.x == pytest.approx(2.25)
  assert detected_box.center.y == pytest.approx(EGO_FORWARD_M + 40.0 + VEHICLE_LENGTH_M * 0.5)
  assert detected_box.longitudinal_m == 40.0


def test_wide_camera_projection_uses_ecam_calibration_and_speed_zoom() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = cluster_renderer.DESIGN_WIDTH
  renderer.height = cluster_renderer.DESIGN_HEIGHT
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  renderer.camera_overlay_pitch_offset_deg = 0.0
  state = _cluster_state(
    speed_kph=0.0,
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
    camera_device_type="tici",
    camera_sensor="ar0231",
    camera_calibration_euler=(0.0, 0.0, 0.0),
    wide_camera_from_device_euler=(0.0, 0.02, 0.0),
  )

  renderer._camera_overlay_wide = False
  narrow = renderer._camera_overlay_projection(state)
  renderer._camera_overlay_wide = True
  wide_low_speed = renderer._camera_overlay_projection(state)
  wide_high_speed = renderer._camera_overlay_projection(replace(state, speed_kph=54.0))

  assert narrow is not None and wide_low_speed is not None and wide_high_speed is not None
  assert not narrow.wide_camera
  assert wide_low_speed.wide_camera
  assert narrow.focal_length == pytest.approx(2648.0)
  assert wide_low_speed.focal_length == pytest.approx(567.0)
  assert wide_low_speed.view_from_road != narrow.view_from_road
  assert wide_high_speed.zoom == pytest.approx(wide_low_speed.zoom * 1.55)


def test_route_overlay_declares_the_camera_projection_stream() -> None:
  renderer = object.__new__(ClusterUiRenderer)
  state = _cluster_state(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
    route_overlay=RouteOverlay(video_rgba=b"frame", camera_stream="wide"),
  )

  assert renderer._select_camera_overlay_stream(state)


def test_model_road_geometry_matches_vehicle_longitudinal_scale() -> None:
  raw_forward_points = (0.0, 20.0, 40.0)
  left_lane = tuple(ModelPathPoint(forward_m, -1.8) for forward_m in raw_forward_points)
  right_lane = tuple(ModelPathPoint(forward_m, 1.8) for forward_m in raw_forward_points)
  path = tuple(ModelPathPoint(forward_m, 0.0) for forward_m in raw_forward_points)
  left_edge = tuple(ModelPathPoint(forward_m, -3.6) for forward_m in raw_forward_points)
  vehicle = DetectedVehicle("V", 40.0, 0.0, source="modelV2", probability=0.9)
  for camera_view_mode, expected_relative_forward_m in (
    (0, 20.0),
    (CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA, 40.0),
  ):
    state = _cluster_state(
      camera_view_mode=camera_view_mode,
      lanes=(
        LaneMarking(-0.5, model_points=left_lane),
        LaneMarking(0.5, model_points=right_lane),
      ),
      highlight_lane="left",
      highlight_lane_offset=0.0,
      model_path=path,
      left_road_edge_points=left_edge,
      left_road_edge_confidence=1.0,
      detected_vehicles=(vehicle,),
    )

    scene = build_cluster_scene(state)
    expected_forward_m = EGO_FORWARD_M + expected_relative_forward_m
    vehicle_box = next(box for box in scene.vehicles if box.label == "V")

    assert vehicle_box.center.y == pytest.approx(expected_forward_m)
    assert _max_scene_forward_m(scene.lane_markings) == pytest.approx(expected_forward_m)
    assert _max_scene_forward_m(scene.highlight_lanes) == pytest.approx(expected_forward_m)
    assert _max_scene_forward_m(scene.road_edges) == pytest.approx(expected_forward_m)
    unblocked_scene = build_cluster_scene(replace(state, detected_vehicles=()))
    assert _max_scene_forward_m(unblocked_scene.planned_path) == pytest.approx(expected_forward_m)


def test_corner_radar_does_not_duplicate_points_covered_by_vehicle_boxes() -> None:
  points = (
    RadarPoint("C1", 12.0, -2.0, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C2", 18.0, 0.5, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C3", 26.0, 2.0, "cornerRadar", relative_speed_mps=-1.0),
  )
  state = _cluster_state(
    detected_vehicles=(
      DetectedVehicle("V", 18.0, 0.5, source="modelV2", probability=0.9),
    ),
    radar_points=points,
  )

  scene = build_cluster_scene(state)

  assert {vehicle.label for vehicle in scene.vehicles if vehicle.source == "cornerRadar"} == {"C1", "C2", "C3"}
  assert scene.radar_points == ()


@pytest.mark.parametrize("lead_label", ("L1", "L2"))
def test_road_camera_hides_own_lane_corner_radar_when_front_lead_is_detected(lead_label) -> None:
  points = (
    RadarPoint("CENTER_EXPLICIT", 12.0, 0.2, "cornerRadar", in_my_lane=1),
    RadarPoint("CENTER_PATH", 30.0, 1.2, "cornerRadar"),
    RadarPoint("ADJACENT", 20.0, 2.4, "cornerRadar", in_my_lane=0),
  )
  state = _cluster_state(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    model_path=(ModelPathPoint(0.0, 0.0), ModelPathPoint(40.0, 1.2)),
    detected_vehicles=(DetectedVehicle(lead_label, 45.0, 0.0, source="radarState", primary=True),),
    radar_points=points,
  )

  scene = build_cluster_scene(state)
  labels = {vehicle.label for vehicle in scene.vehicles}

  assert lead_label in labels
  assert "ADJACENT" in labels
  assert "CENTER_EXPLICIT" not in labels
  assert "CENTER_PATH" not in labels
  assert {marker.label for marker in scene.radar_points}.isdisjoint({"CENTER_EXPLICIT", "CENTER_PATH"})


def test_non_camera_view_keeps_own_lane_corner_radar_with_front_lead() -> None:
  point = RadarPoint("CENTER", 12.0, 0.2, "cornerRadar", in_my_lane=1)
  state = _cluster_state(
    detected_vehicles=(DetectedVehicle("L1", 45.0, 0.0, source="radarState", primary=True),),
    radar_points=(point,),
  )

  scene = build_cluster_scene(state)

  assert any(vehicle.label == "CENTER" for vehicle in scene.vehicles)


def test_road_camera_keeps_own_lane_corner_radar_without_front_lead() -> None:
  point = RadarPoint("CENTER", 12.0, 0.2, "cornerRadar", in_my_lane=1)
  state = _cluster_state(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    detected_vehicles=(DetectedVehicle("M1", 45.0, 0.0, source="modelV2.leadsV3", primary=True),),
    radar_points=(point,),
  )

  scene = build_cluster_scene(state)

  assert any(vehicle.label == "CENTER" for vehicle in scene.vehicles)


def test_many_corner_radar_points_are_bounded_and_not_drawn_twice() -> None:
  points = tuple(
    RadarPoint(
      f"C{index}",
      5.0 + (index % 50) * 2.4,
      ((index % 13) - 6) * 0.55,
      "cornerRadar",
      relative_speed_mps=-1.5 + (index % 5) * 0.2,
      lateral_speed_mps=((index % 7) - 3) * 0.35,
      valid=1,
      valid_count=40,
      in_my_lane=1 if index % 4 == 0 else 0,
      motion_consistent=True,
    )
    for index in range(128)
  )

  scene = build_cluster_scene(_cluster_state(radar_points=points))
  radar_vehicles = tuple(vehicle for vehicle in scene.vehicles if vehicle.source == "cornerRadar")
  vehicle_labels = {vehicle.label for vehicle in radar_vehicles}
  marker_labels = {marker.label for marker in scene.radar_points}

  assert len(radar_vehicles) <= cluster_scene.RADAR_VEHICLE_DISPLAY_LIMIT
  assert len(scene.radar_points) <= cluster_scene.RADAR_MARKER_DISPLAY_LIMIT
  assert vehicle_labels.isdisjoint(marker_labels)


def test_corner_radar_detail_mode_still_obeys_display_budget() -> None:
  points = tuple(
    RadarPoint(f"D{index}", 4.0 + index, (index % 9 - 4) * 0.7, "cornerRadar")
    for index in range(80)
  )

  displayed = cluster_scene.corner_radar_points_for_cluster_display(
    points,
    _cluster_state(radar_points=points, radar_display_mode=1),
    3.6,
  )

  assert len(displayed) == cluster_scene.CORNER_RADAR_DISPLAY_POINT_LIMIT
  assert len({point.label for point in displayed}) == len(displayed)
