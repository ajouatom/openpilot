"""실제 표시 helper·Cap'n Proto 왕복과 장치 표시 AST/GFX sink 회귀.

합성 화면 입력과 가상 카메라다. 그리기 출력만 메모리에 두고 판단·투영
본문은 실행한다. 전체 UI·GPU·장치·센서·제어·CAN 검증은 아니다.
"""

import ast
from functools import partial
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.ui.radar_display import (
  DISPLAY_MAX_AGE_S, DISPLAY_MAX_POINTS, DISPLAY_MAX_LATERAL_M, DISPLAY_MIN_LATERAL_M,
  select_radar_display_points,
)


class DisplaySM(dict):
  def __init__(self, event, *, now=100.0):
    super().__init__(liveTracks=event.liveTracks)
    self.seen = {"liveTracks": True}
    self.alive = {"liveTracks": True}
    self.valid = {"liveTracks": True}
    self.recv_frame = {"liveTracks": 10}
    self.recv_time = {"liveTracks": now - .05}
    self.logMonoTime = {"liveTracks": event.logMonoTime}


def event_for_display(*, supported=True, points=None, now=100.):
  event = log.Event.new_message()
  event.logMonoTime = int((now - .05) * 1e9)
  event.init("liveTracks")
  event.liveTracks.hasDisplayData = supported
  event.liveTracks.points = [{"trackId": 7, "dRel": 40., "yRel": 0., "vLead": 20., "measured": True}]
  event.liveTracks.displayPoints = ([{"trackId": 1001, "dRel": 20., "yRel": 3.5, "vLead": 20., "yvRel": -.5,
                                     "vRel": -5., "measured": True, "radarSource": "frontRadar"}]
                                   if points is None else points)
  return event


def select(sm, level=3, **kwargs):
  return select_radar_display_points(sm, level, now=100., **kwargs)


@pytest.mark.parametrize("level", [0, 1, 2])
def test_levels_zero_one_two_keep_original_control_groups(level):
  assert select(DisplaySM(event_for_display()), level) is None


def test_legacy_provider_without_display_support_keeps_original_groups():
  sm = DisplaySM(event_for_display(supported=False))
  sm.valid["liveTracks"] = False
  assert select(sm) is None


def test_missing_service_keeps_legacy_compatibility():
  assert select({}) is None


def test_full_display_set_replaces_control_set_and_survives_wire_roundtrip():
  event = event_for_display()
  before = event.to_dict()
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    selected = select(DisplaySM(decoded))
    assert len(selected) == 1
    assert (selected[0].dRel, selected[0].yRel, selected[0].vLeadK, selected[0].vLat) == pytest.approx((20., 3.5, 20., -.5))
    assert selected[0].radar
  assert event.to_dict() == before
  assert event.liveTracks.points[0].trackId == 7


def test_supported_empty_display_is_empty_without_legacy_fallback():
  sm = DisplaySM(event_for_display(points=[]))
  assert len(sm["liveTracks"].points) == 1
  assert select(sm) == ()


@pytest.mark.parametrize("field", ["seen", "alive", "valid"])
def test_bad_stream_health_clears_supported_display(field):
  sm = DisplaySM(event_for_display())
  getattr(sm, field)["liveTracks"] = False
  assert select(sm) == ()


@pytest.mark.parametrize("field", ["canError", "radarUnavailableTemporary"])
def test_radar_error_clears_supported_display(field):
  sm = DisplaySM(event_for_display())
  setattr(sm["liveTracks"].errors, field, True)
  assert select(sm) == ()


@pytest.mark.parametrize("kind", [
  "receive_stale", "receive_future", "receive_missing", "publication_stale", "publication_future", "publication_zero", "publication_missing",
])
def test_receive_and_publication_clock_must_both_be_fresh(kind):
  sm = DisplaySM(event_for_display())
  if kind == "receive_stale":
    sm.recv_time["liveTracks"] = 100. - DISPLAY_MAX_AGE_S - .001
  elif kind == "receive_future":
    sm.recv_time["liveTracks"] = 100.001
  elif kind == "receive_missing":
    del sm.recv_time["liveTracks"]
  elif kind == "publication_missing":
    del sm.logMonoTime["liveTracks"]
  else:
    sm.logMonoTime["liveTracks"] = {"publication_stale": int((100. - DISPLAY_MAX_AGE_S - .001) * 1e9),
                                    "publication_future": 100_001_000_000, "publication_zero": 0}[kind]
  assert select(sm) == ()


def test_previous_onroad_frame_cannot_supply_current_display():
  assert select(DisplaySM(event_for_display()), started_frame=11) == ()


def test_closed_age_boundary_accepts_point_two_seconds():
  sm = DisplaySM(event_for_display())
  sm.recv_time["liveTracks"] = 99.8
  sm.logMonoTime["liveTracks"] = 99_800_000_000
  assert len(select(sm)) == 1


@pytest.mark.parametrize("now", [math.nan, math.inf])
def test_invalid_host_clock_clears_supported_display(now):
  assert select_radar_display_points(DisplaySM(event_for_display()), 3, now=now) == ()


@pytest.mark.parametrize("field", ["dRel", "yRel", "vLead", "yvRel"])
@pytest.mark.parametrize("value", [math.nan, math.inf])
def test_nonfinite_geometry_or_motion_does_not_create_display_object(field, value):
  sm = DisplaySM(event_for_display())
  setattr(sm["liveTracks"].displayPoints[0], field, value)
  assert select(sm) == ()


@pytest.mark.parametrize("field,value", [("dRel", 0), ("dRel", -1), ("dRel", 246), ("yRel", 31), ("yRel", -31),
                                         ("trackId", 2**31), ("measured", False), ("radarSource", "corner235")])
def test_unsupported_payload_is_not_displayed(field, value):
  sm = DisplaySM(event_for_display())
  setattr(sm["liveTracks"].displayPoints[0], field, value)
  assert select(sm) == ()


@pytest.mark.parametrize("lateral", [DISPLAY_MIN_LATERAL_M, DISPLAY_MAX_LATERAL_M])
def test_upstream_flip_extremes_are_accepted_without_double_flip(lateral):
  event = event_for_display()
  event.liveTracks.displayPoints[0].yRel = lateral
  selected = select(DisplaySM(event))
  assert len(selected) == 1
  assert selected[0].yRel == pytest.approx(lateral)


def test_excessive_point_count_and_duplicate_identity_clear_display():
  event = event_for_display()
  first = event.liveTracks.displayPoints[0].to_dict()
  event.liveTracks.displayPoints = [dict(first, trackId=1000 + index) for index in range(DISPLAY_MAX_POINTS + 1)]
  assert select(DisplaySM(event)) == ()
  event.liveTracks.displayPoints = [first, first]
  assert select(DisplaySM(event)) == ()


def draw_standard_renderer(sm, level, *, now, positions=None):
  root = Path(__file__).resolve().parents[4]
  path = root / "openpilot/selfdrive/ui/onroad/model_renderer.py"
  cls = next(node for node in ast.parse(path.read_text(encoding="utf-8")).body if isinstance(node, ast.ClassDef) and node.name == "ModelRenderer")
  names = ("_draw_radar_info_carrot", "_get_path_length_idx", "_map_to_screen")
  nodes = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in names]
  output = []
  rl = SimpleNamespace(Color=lambda *args: args, draw_circle=lambda *args: None)
  namespace = {"np": np, "math": math, "rl": rl, "ui_state": SimpleNamespace(is_metric=True, started_frame=0),
               "select_radar_display_points": partial(select_radar_display_points, now=now),
               "draw_text_ui_style": lambda text, *args, **kwargs: output.append(("text", text))}
  # 그리기 함수와 카메라 투영은 원형, GPU 출력만 메모리 sink다.
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), namespace)
  renderer = type("RendererProbe", (), {name: namespace[name] for name in names})()
  renderer._carrot_show_radar_info = level
  renderer._carrot_radar_lat_factor = .5
  renderer._car_space_transform = np.array([[640., 800., 0.], [360., 0., -800.], [1., 0., 0.]])
  renderer._clip_region = SimpleNamespace(x=0., y=0., width=1280., height=720.)
  def text_box(x, y, text, *args):
    output.append(("badge", text))
    if positions is not None:
      positions.append((x, y))

  renderer._draw_text_box_carrot = text_box
  renderer._draw_line_segment_carrot = lambda *args: None
  renderer._draw_radar_info_carrot(sm)
  return output


@pytest.mark.parametrize("level", [1, 2, 3])
def test_standard_renderer_uses_display_only_at_level_three_and_clears_loss(level):
  now = 100.
  event = event_for_display(now=now)
  sm = DisplaySM(event, now=now)
  sm["radarState"] = SimpleNamespace(leadsLeft=[], leadsRight=[], leadsCenter=[
    SimpleNamespace(dRel=30., yRel=0., vLeadK=10., vLat=0., radar=True, modelProb=.03)])
  lane = SimpleNamespace(x=[0., 20., 40.], z=[0., 0., 0.])
  sm["modelV2"] = SimpleNamespace(laneLines=[lane, lane, lane, lane])
  sm.valid.update(radarState=True, modelV2=True)
  before = event.to_dict()
  output = draw_standard_renderer(sm, level, now=now)
  badges = [text for kind, text in output if kind == "badge"]
  assert badges == (["72"] if level == 3 else ["36"])
  assert event.to_dict() == before
  sm.valid["liveTracks"] = False
  output = draw_standard_renderer(sm, level, now=now)
  assert [text for kind, text in output if kind == "badge"] == ([] if level == 3 else ["36"])


def test_standard_renderer_stationary_display_uses_existing_star():
  now = 100.
  event = event_for_display(now=now)
  event.liveTracks.displayPoints[0].vLead = 0.
  event.liveTracks.displayPoints[0].yvRel = 0.
  sm = DisplaySM(event, now=now)
  sm["radarState"] = SimpleNamespace(leadsLeft=[], leadsRight=[], leadsCenter=[])
  lane = SimpleNamespace(x=[0., 20., 40.], z=[0., 0., 0.])
  sm["modelV2"] = SimpleNamespace(laneLines=[lane, lane, lane, lane])
  sm.valid.update(radarState=True, modelV2=True)
  assert draw_standard_renderer(sm, 3, now=now) == [("text", "*")]


def test_standard_renderer_projects_left_and_right_without_double_flip():
  # 가상 카메라의 원형 투영 계산을 실행한다. native GPU·실물 보정 시험은 아니다.
  event = event_for_display()
  first = event.liveTracks.displayPoints[0].to_dict()
  event.liveTracks.displayPoints = [first, dict(first, trackId=1002, yRel=-3.5)]
  sm = DisplaySM(event)
  sm["radarState"] = SimpleNamespace(leadsLeft=[], leadsRight=[], leadsCenter=[])
  lane = SimpleNamespace(x=[0., 20., 40.], z=[0., 0., 0.])
  sm["modelV2"] = SimpleNamespace(laneLines=[lane, lane, lane, lane])
  sm.valid.update(radarState=True, modelV2=True)
  positions = []
  output = draw_standard_renderer(sm, 3, now=100., positions=positions)
  assert [text for kind, text in output if kind == "badge"] == ["72", "72"]
  assert [x for x, _ in positions] == [500, 780]
