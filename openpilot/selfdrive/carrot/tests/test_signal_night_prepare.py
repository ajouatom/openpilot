from copy import deepcopy

import numpy as np
import pytest

from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, night_lamp_support, road_corridor
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import context, geometry, observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO  # noqa: TID251 -- repository-level analysis tools
from tools.signal_analysis.test_signal_assist_planner import inputs  # noqa: TID251


def night_obs(t, ids=(1, 2), source='night_lamp_core'):
  o = observation(t)
  o['tracks'] = []
  for i, tid in enumerate(ids):
    tr = deepcopy(observation(t)['tracks'][0])
    # Inferred housings overlap; actual compact lamps do not.
    x, d = 650+10*i, 3
    tr.update(id=tid, box=[x-d/2, 300-d, x+3.5*d, 300+d])
    tr['evidence'].update(source=source, core_diameter=d, match_score=.95)
    o['tracks'].append(tr)
  return o


def ctx():
  return dict(context(), night_geometry=lambda box: (80., 190.))


def primed():
  gate = SignalStopPrepare()
  for i in range(8):
    d = gate.update(i*.1+.12, night_obs(i*.1), **ctx())
  assert d.active and d.distance_range is None
  assert d.evidence_kind == 'night_lamp_group' and d.support_interval == (80., 190.)
  return gate


def test_two_distinct_lamps_need_time_and_have_no_stop_distance():
  gate = SignalStopPrepare()
  sounds = []
  for i in range(12):
    t = i*.1
    d = gate.update(t+.12, night_obs(t), **ctx())
    assert d.active == (i >= 6)
    assert d.distance_range is None and d.support_interval == (80., 190.)
    sounds.append(d.entered)
  assert sum(sounds) == 1


@pytest.mark.parametrize('ids', [(1,), (1, 1), (1, 2, 3, 4, 5)])
def test_one_duplicate_or_excess_lamps_cannot_establish(ids):
  gate = SignalStopPrepare()
  for i in range(12):
    assert not gate.update(i*.1+.12, night_obs(i*.1, ids), **ctx()).active


@pytest.mark.parametrize('change', ['overlap', 'row', 'scale', 'quality', 'green_score', 'diameter', 'missing_geometry'])
def test_bad_lamp_group_cannot_enter(change):
  gate = SignalStopPrepare()
  for i in range(12):
    o, c = night_obs(i*.1), ctx()
    t = o['tracks'][1]
    if change == 'overlap':
      t['box'] = o['tracks'][0]['box']
    if change == 'row':
      t['box'][1] += 30
      t['box'][3] += 30
    if change == 'scale':
      t['evidence']['core_diameter'] = 10
    if change == 'quality':
      t['evidence']['quality'] = .49
    if change == 'green_score':
      t['evidence']['green_score'] = .09
    if change == 'diameter':
      t['evidence']['core_diameter'] = 30
    if change == 'missing_geometry':
      c['night_geometry'] = lambda b: None
    assert not gate.update(i*.1+.12, o, **c).active


def test_alternating_direct_and_current_pixel_template_keeps_identity():
  gate = SignalStopPrepare()
  for i in range(12):
    o = night_obs(i*.1)
    o['tracks'][i % 2]['evidence']['source'] = 'causal_template'
    d = gate.update(i*.1+.12, o, **ctx())
  assert d.active


def test_templates_cannot_start_or_refresh_the_direct_anchor():
  gate = SignalStopPrepare()
  for i in range(12):
    assert not gate.update(i*.1+.12, night_obs(i*.1, source='causal_template'), **ctx()).active
  gate = primed()
  for t in (.8, .9, 1., 1.1):
    assert gate.update(t+.12, night_obs(t, source='causal_template'), **ctx()).active
  assert not gate.update(1.21+.12, night_obs(1.21, source='causal_template'), **ctx()).active


@pytest.mark.parametrize('source', ['night_lamp_core', 'causal_template'])
def test_invalid_new_box_cannot_borrow_prior_valid_core(source):
  gate = primed()
  o = night_obs(.8, source=source)
  for tr in o['tracks']:
    tr['box'][1] -= 20
    tr['box'][3] += 20
  assert not gate.update(.92, o, **ctx()).active


def test_malformed_night_box_fails_closed():
  gate = primed()
  o = night_obs(.8)
  o['tracks'][0]['box'] = []
  assert not gate.update(.92, o, **ctx()).active
  assert not gate.histories


@pytest.mark.parametrize('change', ['id', 'session', 'match', 'jump', 'scale', 'raw', 'state'])
def test_template_cannot_borrow_or_corrupt_anchor(change):
  gate = primed()
  o = night_obs(.8, source='causal_template')
  if change == 'session':
    o['session'] = 'restart'
  for tr in o['tracks']:
    if change == 'id':
      tr['id'] += 10
    if change == 'match':
      tr['evidence']['match_score'] = .89
    if change == 'jump':
      tr['box'][0] += 30
      tr['box'][2] += 30
    if change == 'scale':
      tr['box'][2] += 10
    if change == 'raw':
      tr['evidence']['raw'] = 'unknown'
    if change == 'state':
      tr['state'] = 'unknown'
  assert not gate.update(.92, o, **ctx()).active


def test_green_immediately_cancels_and_discards_red_confirmation():
  gate = primed()
  o = night_obs(.8)
  green = deepcopy(o['tracks'][0])
  green.update(id=3, state='unknown')
  green['evidence'].update(raw='green', source='causal_template')
  o['tracks'].append(green)
  assert gate.update(.92, o, **ctx()).reason == 'night_green_conflict'
  assert not gate.histories and not gate.night_anchors
  assert not gate.update(1.02, night_obs(.9), **ctx()).active


def test_brief_unknown_is_inactive_but_can_resume_same_pair():
  gate = primed()
  empty = night_obs(.8, ())
  assert gate.update(.92, empty, **ctx()).reason == 'night_waiting'
  d = gate.update(1.02, night_obs(.9), **ctx())
  assert d.active and not d.entered
  # A new pair cannot inherit a disappeared pair's confirmation.
  assert not gate.update(1.12, night_obs(1., (3, 4)), **ctx()).active


def test_long_unknown_and_capture_outage_discard_confirmation():
  gate = primed()
  for t in (.8, .9, 1., 1.1):
    assert not gate.update(t+.12, night_obs(t, ()), **ctx()).active
  assert not gate.update(1.32, night_obs(1.2), **ctx()).active
  assert not primed().update(1.12, night_obs(1.), **ctx()).active


def test_350ms_is_capture_deadline_and_duplicates_never_add_votes():
  gate = primed()
  stamp = gate.last_capture
  assert gate.update(1.04, night_obs(stamp), **ctx()).active
  assert not gate.update(1.051, night_obs(stamp), **ctx()).active
  gate = SignalStopPrepare()
  for t in (.12, .2, .3, .34):
    assert not gate.update(t, night_obs(0.), **ctx()).active
  assert all(count == 1 for since, count in gate.histories.values())


@pytest.mark.parametrize('key,value', [('enabled', False), ('valid', False), ('drive', False), ('speed', 82/3.6),
  ('lead', True), ('gas', True), ('brake', True), ('turning', True), ('model_stopping', True), ('entry_allowed', False)])
def test_context_remains_authoritative(key, value):
  c = ctx()
  c[key] = value
  gate = primed()
  assert not gate.update(.92, night_obs(.8), **c).active
  assert not gate.histories and not gate.night_anchors


def test_direction_support_uses_height_and_uncertainty_not_inferred_width():
  g = geometry()
  # The full housing range would be extremely far for a 12px inferred box.
  b = night_obs(0.)['tracks'][0]['box']
  support = night_lamp_support(b, **g)
  assert support is not None
  # Same ray, different inferred box width gives identical direction support.
  x, y = b[0]+.125*(b[2]-b[0]), (b[1]+b[3])/2
  assert night_lamp_support([x-5, y-10, x+35, y+10], **g) == support
  assert night_lamp_support([650, 400, 662, 406], **g) is None  # below road horizon
  assert night_lamp_support([1000, 300, 1012, 306], **g) is None  # lateral
  g['lane_probs'] = [.1]*4
  g['edge_stds'] = [.8, .8]
  assert night_lamp_support(b, **g) is not None
  args = {k:g[k] for k in ('edges', 'edge_stds', 'lane_lines', 'lane_probs', 'lane_stds')}
  assert road_corridor(np.array([80.]), **args) is None  # legacy contract unchanged
  corridor = road_corridor(np.array([80.]), **args, uncertainty_inset=True)
  assert corridor[0][0] == pytest.approx(-2.1) and corridor[1][0] == pytest.approx(2.1)
  g['edge_stds'] = [1.1, 1.1]
  assert night_lamp_support(b, **g) is None


@pytest.mark.parametrize('kind', ['nan', 'reversed', 'short', 'conflict'])
def test_no_extrapolation_or_invalid_geometry(kind):
  g = geometry()
  if kind == 'nan':
    g['rpy'][1] = float('nan')
  if kind == 'reversed':
    for c in (*g['edges'], *g['lane_lines']):
      c.x = c.x[::-1]
  if kind == 'short':
    for c in (*g['edges'], *g['lane_lines']):
      c.x = c.x/20
  if kind == 'conflict':
    g['lane_lines'][1].y = np.full(33, 8.)
    g['lane_lines'][2].y = np.full(33, 11.6)
  assert night_lamp_support(night_obs(0.)['tracks'][0]['box'], **g) is None


@pytest.mark.parametrize('actuate', [False, True])
def test_actual_planner_night_handover_and_observation_only(actuate):
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p = cls(stop_prepare=SignalStopPrepare())
  sm = inputs(speed=15., distance=160.)
  for i in range(20):
    t = i*.05
    p.update(sm, 80., 'acc', prepare_observation=night_obs(t), prepare_context={
      'now': t+.12, 'enabled': True, 'valid': True, 'drive': True, 'turning': False,
      'geometry': lambda b: None, 'night_geometry': lambda b: (80., 190.), 'actuate': actuate})
  assert p.prepare_decision.active and p.prepare_control_active == actuate
  assert (p.xState.name == 'e2eStopPrepare') == actuate
  assert p.stop_dist == 1000. and p.prepare_decision.distance_range is None
  sm['modelV2'].position.x = [25*i/32 for i in range(33)]
  sm['modelV2'].velocity.x = [15.]+[0.]*32
  for i in range(20):
    t = 1.+i*.05
    p.update(sm, 80., 'acc', prepare_observation=night_obs(t), prepare_context={
      'now': t+.12, 'enabled': True, 'valid': True, 'drive': True, 'turning': False,
      'geometry': lambda b: None, 'night_geometry': lambda b: (80., 190.), 'actuate': actuate})
  assert p.xState.name == 'e2eStop' and not p.prepare_decision.active
