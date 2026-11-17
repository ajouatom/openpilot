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
    # Release early enough to expire before even the predictive boundary gate.
    cs.steeringPressed = frame < (34 if mode == 'expired' else 44) and mode != 'no_torque'
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


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('missing_frames', [1, 2, 4])
def test_width_dropout_preserves_only_a_continuous_established_episode(direction, missing_frames):
  cs, md = scene(direction)
  detector, requests = BoundaryIndicator(), []
  for frame in range(60):
    now = 10 + frame * .05
    distance = 1.3 - max(0, frame - 32) * .04
    md.frameId = frame
    md.laneLines[1].y = [-distance if direction == 1 else -(2.6-distance)] * 9
    md.laneLines[2].y = [2.6-distance if direction == 1 else distance] * 9
    if 32 <= frame < 32 + missing_frames:
      md.laneLines[3-direction].y = [v + (.12 if direction == 2 else -.12) for v in md.laneLines[3-direction].y]
    if frame >= 32:
      md.laneLineProbs[direction] = .01
    prepare_scene(cs, md, now)
    detector.update(now, cs, md, enabled=True, valid=True, busy=False)
    if detector.request_direction:
      requests.append(detector.request_time)
  assert bool(requests) == (missing_frames <= 2)
  assert len(set(requests)) <= 1


def test_intermittent_invalid_width_does_not_establish_a_new_episode():
  cs, md = scene()
  detector = BoundaryIndicator()
  for frame in range(60):
    now = 10 + frame * .05
    distance = max(.2, 1.8 - max(0, frame-30) * .05)
    md.frameId = frame
    cs.steeringPressed = frame >= 30
    width = 2.48 if frame >= 30 and frame % 2 else 2.6
    md.laneLines[1].y = [-distance] * 9
    md.laneLines[2].y = [width-distance] * 9
    prepare_scene(cs, md, now)
    detector.update(now, cs, md, enabled=True, valid=True, busy=False)
    assert detector.episode is None and not detector.request_direction


@pytest.mark.parametrize('mode', ['no_torque', 'retreat', 'stalled', 'expired'])
def test_no_intent_replay_after_retreat_stop_or_expiry(mode):
  assert not maneuver(mode=mode)


@pytest.mark.parametrize('direction', [1, 2])
def test_single_frame_boundary_retreat_preserves_recent_approach(direction):
  def jitter(cs, md, now, frame):
    if frame == 46:
      shift = -.22 if direction == 1 else .22
      for lane in md.laneLines[1:3]:
        lane.y = [y + shift for y in lane.y]
  requests = maneuver(direction, gate=jitter)
  assert requests and {r[1] for r in requests} == {direction}
  assert len({r[2] for r in requests}) == 1


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('consistent', [True, False])
def test_paired_boundary_progress_requires_consistent_width(direction, consistent):
  def reappearing(cs, md, now, frame):
    if frame >= 44:
      shift = .5 if direction == 1 else -.5
      lines = md.laneLines[1:3] if consistent else [md.laneLines[direction]]
      for lane in lines:
        lane.y = [y + shift for y in lane.y]
  requests = maneuver(direction, gate=reappearing)
  assert bool(requests) == consistent


@pytest.mark.parametrize('code', [-1, 0, 1, 2, 12, 13, 16, 18, 20, 21, 24, 25, 30])
def test_left_unknown_centreline_edge_and_barrier_never_indicate(code):
  def gate(cs, md, now, frame):
    cs.leftLaneLine = code
  assert not maneuver(1, gate=gate)


@pytest.mark.parametrize('failure', ['edge', 'edge_ahead', 'nan', 'truncated'])
def test_right_requires_finite_road_edge_clearance(failure):
  def gate(cs, md, now, frame):
    if failure == 'edge':
      md.roadEdges[1].y = [v + 1.5 for v in md.laneLines[2].y]
    if failure == 'edge_ahead':
      md.roadEdges[1].y[8] = md.laneLines[2].y[8] + 2
    if failure == 'nan':
      md.roadEdges[1].y[4] = float('nan')
    if failure == 'truncated':
      md.roadEdges[1].y = [5.4]
  assert not maneuver(2, gate=gate)


@pytest.mark.parametrize('age', [.3, -.1, 10])
def test_left_unknown_or_stale_marking_remains_blocked(age):
  assert not maneuver(1, gate=lambda cs, md, now, frame: setattr(cs, 'laneLineInfoMonoTime', int((now-age)*1e9) if age != 10 else 0))


@pytest.mark.parametrize('direction', [1, 2])
def test_missing_outer_line_and_uncertain_edge_std_do_not_block(direction):
  def gate(cs, md, now, frame):
    md.laneLineProbs[0] = md.laneLineProbs[3] = 0
    md.laneLines[0].y = md.laneLines[3].y = []
    md.roadEdgeStds = [float('nan'), float('nan')]
  assert maneuver(direction, gate=gate)


def test_left_does_not_use_road_edge_geometry():
  assert maneuver(1, gate=lambda cs, md, now, frame: setattr(md, 'roadEdges', []))


def test_right_does_not_use_oem_marking_or_timestamp():
  def gate(cs, md, now, frame):
    cs.leftLaneLine = cs.rightLaneLine = 21
    cs.laneLineInfoMonoTime = 0
  assert maneuver(2, gate=gate)


@pytest.mark.parametrize('clearance,expected', [(2.499, False), (2.5, True), (3., True)])
def test_right_clearance_is_measured_beyond_target_boundary(clearance, expected):
  cs, md = scene(2)
  md.roadEdges[1].y = [y+clearance for y in md.laneLines[2].y]
  assert adjacent_lane(cs, md, 2, 10.) == expected


def test_centreline_appearing_after_torque_cancels_remembered_intent():
  assert not maneuver(gate=lambda cs, md, now, frame: setattr(cs, 'leftLaneLine', 21 if frame >= 44 else 10))


def test_only_selected_side_is_restricted():
  assert maneuver(gate=lambda cs, md, now, frame: setattr(cs, 'rightLaneLine', 21))


def test_old_schema_unknown_left_colour_fails_closed():
  cs, md = scene()
  del cs.laneLineInfoMonoTime
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


@pytest.mark.parametrize('direction', [1, 2])
def test_opposite_boundary_single_frame_jump_preserves_driver_approach(direction):
  def jitter(cs, md, now, frame):
    if frame == 44:
      lane = md.laneLines[3-direction]
      lane.y = [v + (.55 if direction == 1 else -.55) for v in lane.y]
  requests = maneuver(direction, gate=jitter)
  assert requests and len({r[2] for r in requests}) == 1
  assert requests[0][0] < 12.6
