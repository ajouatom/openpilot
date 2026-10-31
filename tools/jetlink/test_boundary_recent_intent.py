from types import SimpleNamespace as N

import pytest

from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import adjacent_lane
from opendbc.car.hyundai.lane_line_time import lane_line_time
from test_blinker_boundary import scene, prepare_scene


def maneuver(direction=1, mode='release', gate=None):
  cs, md = scene(direction)
  detector, requests = BoundaryIndicator(), []
  for frame in range(90):
    now = 10 + frame * .05
    progress = max(0, frame - 30) * .04
    distance = max(.3, 1.8 - progress)
    if mode == 'retreat' and frame >= 44:
      distance = min(1.8, 1.28 + (frame - 44) * .05)
    if mode == 'stalled' and frame >= 44:
      distance = 1.28 if frame < 60 else max(.3, 1.28 - (frame - 60) * .04)
    if mode == 'expired':
      distance = max(.3, 1.8 - max(0, frame - 30) * .022)
    md.frameId = frame
    md.laneLines[1].y = [-distance if direction == 1 else -(3.6 - distance)] * 9
    md.laneLines[2].y = [3.6 - distance if direction == 1 else distance] * 9
    prepare_scene(cs, md, now)
    cs.steeringPressed = frame < (40 if mode == 'expired' else 44) and mode != 'no_torque'
    cs.steeringTorque = (300 if direction == 1 else -300) if cs.steeringPressed else 0
    if mode == 'countersteer' and frame >= 44:
      cs.steeringPressed = True
      cs.steeringTorque = -300 if direction == 1 else 300
    if gate is not None:
      gate(cs, md, now, frame)
    detector.update(now, cs, md, enabled=True, valid=True, busy=False)
    if detector.request_direction:
      requests.append((now, detector.request_direction, detector.request_time, cs.steeringPressed))
  return requests


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('mode', ['release', 'countersteer'])
def test_continued_approach_remembers_driver_and_emits_once(direction, mode):
  requests = maneuver(direction, mode)
  assert requests and {r[1] for r in requests} == {direction}
  assert len({r[2] for r in requests}) == 1
  assert requests[0][0] > 12.2
  if mode == 'release':
    assert not any(r[3] for r in requests)


@pytest.mark.parametrize('mode', ['no_torque', 'retreat', 'stalled', 'expired'])
def test_no_intent_replay_after_retreat_stop_or_expiry(mode):
  assert not maneuver(mode=mode)


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('code', [-1, 0, 1, 2, 12, 13, 16, 18, 20, 21, 24, 25, 30])
def test_unknown_centreline_edge_and_barrier_never_indicate(direction, code):
  def gate(cs, md, now, frame):
    setattr(cs, 'leftLaneLine' if direction == 1 else 'rightLaneLine', code)
  assert not maneuver(direction, gate=gate)


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('failure', ['no_outer', 'shoulder', 'edge', 'narrow_ahead', 'edge_ahead', 'stale', 'future', 'unknown', 'nan'])
def test_adjacent_lane_positive_evidence_required(direction, failure):
  def gate(cs, md, now, frame):
    inner, outer, edge = (1, 0, 0) if direction == 1 else (2, 3, 1)
    sign = -1 if direction == 1 else 1
    if failure == 'no_outer':
      md.laneLineProbs[outer] = .1
    if failure in ('shoulder', 'edge'):
      md.roadEdges[edge].y = [v + sign * 1.5 for v in md.laneLines[inner].y]
    if failure == 'narrow_ahead':
      md.laneLines[outer].y[8] = md.laneLines[inner].y[8] + sign * 2
    if failure == 'edge_ahead':
      md.roadEdges[edge].y[8] = md.laneLines[inner].y[8] + sign * 2
    if failure == 'stale':
      cs.laneLineInfoMonoTime = int((now - .3) * 1e9)
    if failure == 'future':
      cs.laneLineInfoMonoTime = int((now + .1) * 1e9)
    if failure == 'unknown':
      cs.laneLineInfoMonoTime = 0
    if failure == 'nan':
      md.roadEdgeStds[edge] = float('nan')
  assert not maneuver(direction, gate=gate)


def test_centreline_appearing_after_torque_cancels_remembered_intent():
  assert not maneuver(gate=lambda cs, md, now, frame: setattr(cs, 'leftLaneLine', 21 if frame >= 44 else 10))


def test_only_selected_side_is_restricted():
  assert maneuver(gate=lambda cs, md, now, frame: setattr(cs, 'rightLaneLine', 21))


def test_old_schema_unknown_colour_and_truncated_geometry_fail_closed():
  cs, md = scene()
  del cs.laneLineInfoMonoTime
  assert not adjacent_lane(cs, md, 1, 10.)
  prepare_scene(cs, md, 10.)
  md.laneLines[0].y = [-5.4]
  assert not adjacent_lane(cs, md, 1, 10.)


def test_timestamp_belongs_to_exact_selected_parser_and_requires_all_fields():
  selected = {'LEFT_LANE_TYPE':0}
  fields = ('LEFT_LANE_TYPE', 'RIGHT_LANE_TYPE', 'LEFT_LANE_COLOR', 'RIGHT_LANE_COLOR')
  old = N(vl={'CAM_0x2a4':selected}, ts_nanos={'CAM_0x2a4':dict.fromkeys(fields, 9)})
  other = N(vl={'CAM_0x2a4':dict(selected)}, ts_nanos={'CAM_0x2a4':dict.fromkeys(fields, 100)})
  assert lane_line_time((other, None, old), selected) == 9
  del old.ts_nanos['CAM_0x2a4']['LEFT_LANE_COLOR']
  assert lane_line_time((old, other), selected) == 0
  assert lane_line_time((old, other), None) == 0


def test_new_timestamp_roundtrips_real_schema():
  from openpilot.cereal import log
  event = log.Event.new_message()
  event.init('carState').laneLineInfoMonoTime = 123456789
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert decoded.carState.laneLineInfoMonoTime == 123456789
