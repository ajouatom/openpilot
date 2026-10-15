"""A born-in-lane lead must not depend on CUT-IN or accept adjacent aliases."""
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.radar_motion.in_lane import InLaneCornerTracker
from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot, vision_lead_from_model
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import model_with_lead


def sample(t, curvature=0.):
  d = 70. - 10. * t - t*t
  def center(x):
    return curvature * x * x
  m = model_with_lead(d, -center(d), 10. - 2.*t, probability=.9)
  xs = tuple(float(x) for x in range(0, 131, 10))
  m.position = NS(x=xs, y=tuple(center(x) for x in xs))
  m.laneLines = tuple(NS(x=xs, y=tuple(center(x) + shift for x in xs)) for shift in (-5.25, -1.75, 1.75, 5.25))
  m.laneLineProbs = (.9, .98, .98, .9)
  m.laneLineStds = (.1, .1, .1, .1)
  m.meta = NS(laneChangeState='off')
  m.velocity = NS(x=(20., 10.))
  q = RadarPointSnapshot(track_id=101, source='corner235', d_rel=d,
                        y_rel=-center(d), v_rel=-10. - 2.*t, a_rel=-2.,
                        yv_rel=2. * (10. + 2.*t) * curvature * d, v_lead=10. - 2.*t, a_lead=-2.,
                        j_lead=0., measured=True, radar_track_state=0)
  return m, q


def update(c, t, m, points, **kw):
  return c.update(t, 20., m, tuple(points), vision_lead_from_model(m), **kw)


@pytest.mark.parametrize('curve', (-.001, 0., .001))
def test_already_in_lane_moving_lead_needs_measured_lane_and_visual_history(curve):
  c = InLaneCornerTracker()
  for i in range(12):
    t = i * .05
    m, q = sample(t, curve)
    out = update(c, t, m, [q])
    if i < 7:
      assert out is None
    elif i >= 8:
      assert out is not None and out[0] is q


@pytest.mark.parametrize('bad', (
  'vision_missing', 'vision_weak', 'range_disagreement', 'speed_disagreement',
  'visual_adjacent', 'radar_adjacent', 'uncertain_vision', 'invalid_vision',
  'weak_left', 'weak_right', 'uncertain_lane', 'no_lanes', 'lane_change',
  'short_lanes', 'path_disagreement', 'unmeasured', 'range_jump', 'side_jump',
  'speed_jump', 'changing_ids', 'ambiguous', 'nonclosing', 'invalid_point',
  'cruising_model', 'nonbraking_radar', 'isolated_braking', 'range_rate_disagreement', 'false_acceleration',
))
def test_unsupported_inputs_never_promote(bad):
  c = InLaneCornerTracker()
  for i in range(20):
    t = i * .05
    m, q = sample(t)
    points = [q]
    if bad == 'vision_missing':
      m.leadsV3 = ()
    if bad == 'vision_weak':
      m.leadsV3[0].prob = .49
    if bad == 'range_disagreement':
      m.leadsV3[0].x = (q.d_rel + 20.,)
    if bad == 'speed_disagreement':
      m.leadsV3[0].v = (20.,)
    if bad == 'visual_adjacent':
      m.leadsV3[0].y = (3.5,)
    if bad == 'radar_adjacent':
      points = [replace(q, y_rel=3.5)]
    if bad == 'uncertain_vision':
      m.leadsV3[0].yStd = (3.,)
    if bad == 'invalid_vision':
      m.leadsV3[0].x = (float('nan'),)
    if bad == 'weak_left':
      m.laneLineProbs = (.9, .2, .98, .9)
    if bad == 'weak_right':
      m.laneLineProbs = (.9, .98, .2, .9)
    if bad == 'uncertain_lane':
      m.laneLineStds = (.1, .5, .1, .1)
    if bad == 'no_lanes':
      m.laneLines = ()
    if bad == 'lane_change':
      m.meta.laneChangeState = 'laneChangeStarting'
    if bad == 'short_lanes':
      for l in m.laneLines:
        l.x, l.y = l.x[:4], l.y[:4]
    if bad == 'path_disagreement':
      m.position.y = (3.5,) * len(m.position.x)
    if bad == 'unmeasured':
      points = [replace(q, measured=False)]
    if bad == 'range_jump':
      points = [replace(q, d_rel=q.d_rel + 2. * (i % 2))]
    if bad == 'side_jump':
      points = [replace(q, y_rel=.6 * (-1)**i)]
    if bad == 'speed_jump':
      points = [replace(q, v_lead=10. + 2. * (-1)**i)]
    if bad == 'changing_ids':
      points = [replace(q, track_id=i)]
    if bad == 'ambiguous':
      points += [replace(q, track_id=102, d_rel=q.d_rel+1.)]
    if bad == 'nonclosing':
      points = [replace(q, v_rel=0.)]
    if bad == 'range_rate_disagreement':
      points = [replace(q, d_rel=q.d_rel-5.*t)]
      m.leadsV3[0].x = (points[0].d_rel+1.52,)
    if bad == 'false_acceleration':
      points = [replace(q, v_lead=10., v_rel=-10.)]
    if bad == 'cruising_model':
      m.velocity.x = (20., 20.)
    if bad == 'nonbraking_radar':
      points = [replace(q, a_lead=0.)]
    if bad == 'isolated_braking':
      points = [replace(q, a_lead=0. if i % 3 == 0 else -2.)]
    if bad == 'invalid_point':
      points = [replace(q, y_rel=float('nan'))]
    assert update(c, t, m, points) is None


@pytest.mark.parametrize('loss', ('vision', 'lane', 'point', 'id', 'gap', 'repeated_time', 'ambiguous'))
def test_current_evidence_loss_withdraws_even_after_confirmation(loss):
  c = InLaneCornerTracker()
  for i in range(12):
    m, q = sample(i*.05)
    out = update(c, i*.05, m, [q])
  assert out is not None
  t = .9 if loss == 'gap' else .55 if loss == 'repeated_time' else .6
  m, q = sample(t)
  points = [q]
  if loss == 'vision':
    m.leadsV3 = ()
  if loss == 'lane':
    m.laneLineProbs = (.9, .2, .98, .9)
  if loss == 'point':
    points = []
  if loss == 'id':
    points = [replace(q, track_id=102)]
  if loss == 'ambiguous':
    points += [replace(q, track_id=102)]
  assert update(c, t, m, points) is None


def test_repeated_radar_measurement_cannot_confirm_on_new_model_frames():
  c = InLaneCornerTracker()
  for i in range(20):
    m, q = sample(0.)
    assert update(c, i*.05, m, [q], measurement_time_s=0.) is None


def test_moving_into_lane_requires_new_lane_dwell_not_just_old_radar_history():
  c = InLaneCornerTracker()
  for i in range(20):
    t = i*.05
    m, q = sample(t)
    if i < 10:
      m.laneLineProbs = (.9, .2, .98, .9)
    out = update(c, t, m, [q])
    if i < 17:
      assert out is None
    if i >= 18:
      assert out is not None


def test_second_matching_return_beyond_output_cap_still_vetoes():
  c = InLaneCornerTracker()
  for i in range(12):
    m, q = sample(i*.05)
    q = replace(q, d_rel=q.d_rel+10.)
    m.leadsV3[0].x = (q.d_rel+1.5,)
    assert update(c, i*.05, m, [q, replace(q, track_id=102, d_rel=q.d_rel+8.)]) is None


@pytest.mark.parametrize('mode', (-2, -1, 0, 1, 2, 3))
@pytest.mark.parametrize('front', (False, True))
def test_controller_preserves_primary_and_only_adds_auxiliary_in_radar_modes(mode, front):
  c = DPathRadarController(prefer_corner_radar=True, enable_radar_tracks=mode,
                          corner_radar_measurement_delay_s=0.)
  for i in range(15):
    t = i*.05
    m, q = sample(t)
    points = [q] + ([replace(q, source='frontRadar', track_id=41, radar_track_state=2)] if front else [])
    out = c.update(t, 20., points, m)
  if front and mode > 0:
    assert out.lead_one is not None and out.lead_one['radarTrackId'] == 41
    assert out.lead_two is None
  elif mode > 0:
    assert out.lead_one is not None and not out.lead_one['radar']
    assert out.lead_two is not None and out.lead_two['radarTrackId'] == 101
    assert out.lead_two['dRel'] == pytest.approx(q.d_rel)
    assert out.lead_two['vLead'] == pytest.approx(q.v_lead)
    assert out.lead_two['aLeadK'] < -.5
    assert out.leads_cutin == ()
  else:
    assert out.lead_two is None


def test_missing_path_resets_controller_history():
  c = DPathRadarController(prefer_corner_radar=True, corner_radar_measurement_delay_s=0.)
  for i in range(12):
    m, q = sample(i*.05)
    out = c.update(i*.05, 20., [q], m)
  assert out.lead_two is not None
  m.position = NS(x=(), y=())
  assert c.update(.6, 20., [q], m).lead_two is None
  m, q = sample(.65)
  assert c.update(.65, 20., [q], m).lead_two is None
