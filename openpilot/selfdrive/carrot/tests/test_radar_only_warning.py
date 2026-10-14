from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib.radar_only_warning import RadarOnlyWarning
from openpilot.selfdrive.carrot.tests.test_stopping_lane_corner import sample


def scene(t, speed=15., distance=75.):
  model, corner = sample(t)
  model.position.x = (0., 10., 20., 30., 40., 80., 100.)
  model.position.y = (0.,) * 7
  for line in model.laneLines:
    line.x = (*line.x, 150.)
    line.y = (*line.y, line.y[-1])
  model.leadsV3[0].prob = .1
  corner = replace(corner, d_rel=distance - speed*t, v_rel=-speed)
  front = replace(corner, track_id=50, source='frontRadar', radar_track_state=3)
  return model, (front, corner)


def test_warning_is_one_shot_after_independent_confirmation():
  detector = RadarOnlyWarning()
  alerts = []
  for i in range(60):
    t = i*.05
    model, points = scene(t)
    if detector.update(t, 15., model, points, enabled=True): alerts.append(t)
  assert alerts == [.5]


@pytest.mark.parametrize('speed,distance', ((10., 48.), (60./3.6, 80.), (100./3.6, 120.)))
def test_threshold_tracks_closing_time_with_120m_cap(speed, distance):
  d = RadarOnlyWarning()
  fired = []
  for i in range(20):
    t = i*.05
    model, points = scene(t, speed, distance + speed*.5)
    if d.update(t, speed, model, points, enabled=True): fired.append((t, points[0].d_rel))
  assert len(fired) == 1
  assert fired[0][1] <= distance


@pytest.mark.parametrize('change', ('front_missing', 'corner_missing', 'tentative', 'moving', 'unmeasured',
  'adjacent', 'range_disagree', 'lateral_disagree', 'speed_disagree', 'weak_lane', 'lane_change', 'nan',
  'vision', 'missing_vision', 'selected', 'disabled', 'new_id', 'range_jump', 'too_far', 'slow', 'gap'))
def test_rejects_uncorroborated_or_unsupported_alerts(change):
  d = RadarOnlyWarning()
  for i in range(20):
    t = i*(.2 if change == 'gap' else .05)
    model, points = scene(min(t, .95))
    f, c = points
    if change == 'front_missing': points = (c,)
    if change == 'corner_missing': points = (f,)
    if change == 'tentative': points = (replace(f, radar_track_state=1), c)
    if change == 'moving': points = (replace(f, v_lead=5.), replace(c, v_lead=5.))
    if change == 'unmeasured': points = (f, replace(c, measured=False))
    if change == 'adjacent': points = (replace(f, y_rel=3.5), replace(c, y_rel=3.5))
    if change == 'range_disagree': points = (f, replace(c, d_rel=c.d_rel+3.1))
    if change == 'lateral_disagree': points = (replace(f, y_rel=-.5), replace(c, y_rel=.5))
    if change == 'speed_disagree': points = (replace(f, v_lead=-1.5), replace(c, v_lead=1.5))
    if change == 'weak_lane': model.laneLineProbs = (1., .8, 1., 1.)
    if change == 'lane_change': model.meta.laneChangeState = 'preLaneChange'
    if change == 'nan': points = (replace(f, y_rel=float('nan')), c)
    if change == 'vision': model.leadsV3[0].prob = .4
    if change == 'missing_vision': model.leadsV3 = ()
    if change == 'new_id': points = (replace(f, track_id=i), c)
    if change == 'range_jump': points = tuple(replace(p, d_rel=p.d_rel+2*(i%2)) for p in points)
    if change == 'too_far': points = tuple(replace(p, d_rel=121.) for p in points)
    assert not d.update(t, 5. if change == 'slow' else 15., model, points,
                        enabled=change != 'disabled', selected_lead=change == 'selected')


def test_id_changes_and_short_dropouts_do_not_repeat_alarm():
  d = RadarOnlyWarning()
  fired = []
  for i in range(800):
    t = i*.05
    model, points = scene(t % 2.)
    points = tuple(replace(p, track_id=p.track_id+int(t//2)) for p in points)
    if i%40 == 0: points = ()
    if d.update(t, 15., model, points, enabled=True): fired.append(t)
  assert len(fired) == 1
  for i in range(100):
    model, _ = scene(0.)
    assert not d.update(40+i*.05, 15., model, (), enabled=True)
  for i in range(20):
    model, points = scene(i*.05)
    if d.update(45+i*.05, 15., model, points, enabled=True): fired.append(45+i*.05)
  assert len(fired) == 2


def live(t):
  model, points = scene(t)
  model.timestampEof = int((10+t)*1e9)
  model.meta.hardBrakePredicted = False
  raw = [NS(trackId=p.track_id, radarSource=p.source, dRel=p.d_rel, yRel=p.y_rel, vRel=p.v_rel,
            yvRel=0., aRel=0., aLead=0., jLead=0., measured=True, trackState=p.radar_track_state) for p in points]
  class SM(dict): pass
  sm = SM(modelV2=model, liveTracks=NS(points=raw), radarState=NS(leadOne=NS(status=False), leadTwo=NS(status=False)),
          carControl=NS(longActive=True), longitudinalPlan=NS(fcw=False))
  sm.valid = dict.fromkeys(sm, True)
  sm.updated = dict.fromkeys(sm, True)
  sm.logMonoTime = dict.fromkeys(sm, int((10+t)*1e9))
  return NS(vEgo=15., canValid=True, gasPressed=False, brakePressed=False), sm


@pytest.mark.parametrize('change', ('none', 'stale', 'invalid', 'pedal', 'inactive', 'fcw', 'reused_radar', 'reused_model', 'nan'))
def test_live_adapter_requires_fresh_independent_samples_and_active_long(change):
  d = RadarOnlyWarning()
  alerts = []
  for i in range(20):
    t = i*.05
    cs, sm = live(t)
    if change == 'stale': sm.logMonoTime['liveTracks'] -= 300_000_000
    if change == 'invalid': sm.valid['radarState'] = False
    if change == 'pedal': cs.brakePressed = True
    if change == 'inactive': sm['carControl'].longActive = False
    if change == 'fcw': sm['longitudinalPlan'].fcw = True
    if change == 'reused_radar': sm.logMonoTime['liveTracks'] = 10_000_000_000
    if change == 'reused_model': sm.updated['modelV2'] = False
    if change == 'nan': sm['liveTracks'].points[0].yRel = float('nan')
    if d.update_live(10+t+.01, cs, sm, True): alerts.append(t)
  assert len(alerts) == (1 if change == 'none' else 0)
