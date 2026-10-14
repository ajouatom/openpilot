"""Stopped-corner corroboration must not admit lane aliases or stale evidence."""
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot, vision_lead_from_model
from openpilot.selfdrive.carrot.radar_motion.stationary_lane import StoppingLaneCornerTracker
from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import _controller_model
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import model_with_lead


def sample(t, side=1., curve=False):
  distance = 75. - 15. * t
  center = lambda x: side * .001 * x * x if curve else 0.
  model = model_with_lead(distance, -center(distance), 0., probability=.9)
  xs = (0., 10., 20., 30., 40., distance - 13., distance - 10.)
  model.position = NS(x=xs, y=tuple(center(x) + (side * 3. if i == 6 else 0.) for i, x in enumerate(xs)))
  lane_x = (0., 10., 20., 30., 40., 50., 60., 70., 80., 90., 100.)
  model.laneLines = tuple(NS(x=lane_x, y=tuple(center(x) + shift for x in lane_x)) for shift in (-5.25, -1.75, 1.75, 5.25))
  model.laneLineProbs = (.9, .98, .98, .9)
  model.laneLineStds = (.1, .1, .1, .1)
  model.velocity = NS(x=(15., 0.))
  model.meta = NS(laneChangeState='off')
  point = RadarPointSnapshot(track_id=1193, source='corner235', d_rel=distance,
                             y_rel=-center(distance), v_rel=-15., a_rel=0.,
                             yv_rel=30. * side * .001 * distance if curve else 0.,
                             v_lead=0., a_lead=0., j_lead=0., measured=True, radar_track_state=0)
  return model, point


def update(tracker, t, model, points):
  return tracker.update(t, 15., model, tuple(points), vision_lead_from_model(model))


@pytest.mark.parametrize('side', (-1., 1.))
@pytest.mark.parametrize('curve', (False, True))
def test_continuous_measured_car_needs_visual_confirmation_and_both_lanes(side, curve):
  tracker = StoppingLaneCornerTracker()
  for i in range(12):
    t = i * .05
    model, point = sample(t, side, curve)
    out = update(tracker, t, model, [point])
    if i < 7:
      assert out is None
    elif i >= 8:
      assert out is not None
      assert out[0] is point
      assert abs(out[1]) < .1


@pytest.mark.parametrize('change', (
  'no_vision', 'low_vision', 'moving_vision', 'other_visual_car', 'visual_range',
  'unmeasured', 'moving_radar', 'range_jump', 'lateral_jump', 'speed_jump', 'new_ids',
  'adjacent', 'weak_left', 'weak_right', 'uncertain_lane', 'crossed_lanes', 'wide_lane',
  'short_lanes', 'nan_lane', 'nonmonotonic_lane', 'path_disagreement', 'nonstopping',
  'pre_lane_change', 'lane_change', 'unknown_state', 'no_lanes', 'invalid_time', 'slow_ego',
))
def test_unqualified_or_inconsistent_evidence_never_promotes(change):
  tracker = StoppingLaneCornerTracker()
  for i in range(15):
    t = i * .05
    model, point = sample(t)
    if change == 'no_vision': model.leadsV3 = ()
    if change == 'low_vision': model.leadsV3[0].prob = .39
    if change == 'moving_vision': model.leadsV3[0].v = (12.,)
    if change == 'other_visual_car': model.leadsV3[0].y = (2.,)
    if change == 'visual_range': model.leadsV3[0].x = (point.d_rel + 12.,)
    if change == 'unmeasured': point = replace(point, measured=False)
    if change == 'moving_radar': point = replace(point, v_lead=4.)
    if change == 'range_jump': point = replace(point, d_rel=point.d_rel + 2. * (i % 2))
    if change == 'lateral_jump': point = replace(point, y_rel=.5 * (-1)**i)
    if change == 'speed_jump': point = replace(point, v_lead=1. * (-1)**i)
    if change == 'new_ids': point = replace(point, track_id=i)
    if change == 'adjacent':
      point = replace(point, y_rel=3.5)
      model.leadsV3[0].y = (-3.5,)
    if change == 'weak_left': model.laneLineProbs = (.9, .89, .98, .9)
    if change == 'weak_right': model.laneLineProbs = (.9, .98, .89, .9)
    if change == 'uncertain_lane': model.laneLineStds = (.1, .3, .1, .1)
    if change == 'crossed_lanes': model.laneLines = (model.laneLines[0], model.laneLines[2], model.laneLines[1], model.laneLines[3])
    if change == 'wide_lane': model.laneLines[2].y = (4.,) * 11
    if change == 'short_lanes':
      for line in model.laneLines: line.x, line.y = line.x[:6], line.y[:6]
    if change == 'nan_lane': model.laneLines[1].y = (float('nan'),) * 11
    if change == 'nonmonotonic_lane': model.laneLines[1].x = (0.,) * 11
    if change == 'path_disagreement': model.position.y = (3.,) * len(model.position.x)
    if change == 'nonstopping': model.velocity.x = (15., 15.)
    if change == 'pre_lane_change': model.meta.laneChangeState = 'preLaneChange'
    if change == 'lane_change': model.meta.laneChangeState = 'laneChangeStarting'
    if change == 'unknown_state': model.meta = NS()
    if change == 'no_lanes': model.laneLines = ()
    if change == 'invalid_time': t = float('nan')
    result = tracker.update(t, 3. if change == 'slow_ego' else 15., model, (point,), vision_lead_from_model(model))
    assert result is None


@pytest.mark.parametrize('change', ('missing', 'gap', 'repeat', 'weak_lane', 'lost_vision', 'ambiguous', 'changed_id'))
def test_active_fallback_withdraws_on_current_evidence_loss(change):
  tracker = StoppingLaneCornerTracker()
  for i in range(12):
    model, point = sample(i * .05)
    out = update(tracker, i * .05, model, [point])
  assert out is not None
  t = .8 if change == 'gap' else .55 if change == 'repeat' else .6
  model, point = sample(t)
  points = [point]
  if change == 'missing': points = []
  if change == 'weak_lane': model.laneLineProbs = (.9, .2, .98, .9)
  if change == 'lost_vision': model.leadsV3[0].prob = .1
  if change == 'ambiguous': points += [replace(point, track_id=1200, d_rel=point.d_rel + .3)]
  if change == 'changed_id': points = [replace(point, track_id=1200)]
  assert update(tracker, t, model, points) is None


def test_weak_vision_can_only_briefly_continue_an_already_confirmed_car():
  tracker = StoppingLaneCornerTracker()
  for i in range(12):
    model, point = sample(i * .05)
    assert update(tracker, i * .05, model, [point]) is not None or i < 8
  for t in (.6, .65, .7, .75):
    model, point = sample(t)
    model.leadsV3[0].prob = .38
    out = update(tracker, t, model, [point])
    assert (out is not None) == (t <= .7)


def test_second_matching_return_beyond_output_range_is_still_ambiguous():
  tracker = StoppingLaneCornerTracker()
  for i in range(11):
    t = -.5 + i*.05
    model, point = sample(t)
    out = update(tracker, t, model, [point])
  assert out is not None
  model, point = sample(.05)
  other = replace(point, track_id=1200, d_rel=81.)
  assert update(tracker, .05, model, [point, other]) is None


@pytest.mark.parametrize('mode', (-2, -1, 0, 1, 2, 3))
@pytest.mark.parametrize('corner', (False, True))
def test_controller_keeps_primary_policy_and_requires_corner_input(mode, corner):
  controller = DPathRadarController(enable_radar_tracks=mode, prefer_corner_radar=corner, corner_radar_measurement_delay_s=0.)
  for i in range(12):
    t = i * .05
    model, point = sample(t)
    out = controller.update(t, 15., [point] if corner else [], model)
  assert out.lead_one is None or not out.lead_one['radar']
  assert (out.lead_two is not None) == (mode > 0 and corner)
  if out.lead_two is not None:
    assert out.lead_two['radarTrackId'] == 1193
    assert out.lead_two['dRel'] == pytest.approx(point.d_rel)
    assert out.lead_two['vLead'] == 0.


def test_replay_model_preserves_stopping_context_and_invalid_geometry_fails_closed():
  model, _ = sample(0.)
  frame = NS(path=tuple(zip(model.position.x, model.position.y)), model_leads=(), v_ego=14.,
             model_velocity=model.velocity.x, model_lane_change_state='preLaneChange',
             stopping_lane_geometry_valid=True, lane_lines=tuple(tuple(zip(l.x, l.y)) for l in model.laneLines),
             lane_probs=model.laneLineProbs, lane_stds=model.laneLineStds)
  replay = _controller_model(frame)
  assert replay.velocity.x == (15., 0.)
  assert replay.meta.laneChangeState == 'preLaneChange'
  assert replay.laneLines[1].y == model.laneLines[1].y
  frame.stopping_lane_geometry_valid = False
  assert _controller_model(frame).laneLines == ()
