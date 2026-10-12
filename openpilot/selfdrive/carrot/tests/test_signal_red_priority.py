from dataclasses import asdict

import numpy as np
import pytest

from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, housing_alignment, housing_range
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation, context, geometry


def scene(t, reds=(1,), greens=(4,)):
  o = observation(t)
  o['tracks'] = []
  for tid in (*reds, *greens):
    track = observation(t, 'green' if tid in greens else 'red')['tracks'][0]
    track['id'] = tid
    track['box'] = [480+60*tid, 260, 520+60*tid, 270]
    if tid in greens:
      track['evidence'].update(red_score=0., green_score=.8)
    o['tracks'].append(track)
  return o


def aligned(bounds):
  c = context()
  c['alignment'] = lambda box, distance: bounds.get(round((box[0]-480)/60))
  return c


def run(p, c, reds=(1,), greens=(4,), start=0, stop=8):
  decisions = []
  for i in range(start, stop):
    t = i*.1
    decisions.append(p.update(t, scene(t, reds, greens), **c))
  return decisions


@pytest.mark.parametrize('green_bounds', [(1.8, 3.), (-3., -1.8)])
def test_central_red_overrides_separated_green_only_after_confirmation(green_bounds):
  p = SignalStopPrepare()
  ds = run(p, aligned({1: (-.5, .5), 4: green_bounds}))
  assert not any(d.active for d in ds[:6])
  assert all(d.active for d in ds[6:])
  assert sum(d.entered for d in ds) == 1
  d = ds[-1]
  assert d.priority == 'central_red' and d.priority_ids == (1,) and d.side_green_ids == (4,)
  assert asdict(d)['side_green_ids'] == (4,)


def test_two_near_center_reds_support_each_other_without_majority_vote():
  bounds = {1: (-1.2, -.8), 2: (.8, 1.2), 4: (1.8, 2.5)}
  assert run(SignalStopPrepare(), aligned(bounds))[-1].reason == 'mixed_colors'
  ds = run(SignalStopPrepare(), aligned(bounds), reds=(1, 2))
  assert ds[-1].active and ds[-1].priority == 'central_red_pair'
  # Even a red majority cannot defeat an equally central green.
  bounds[4] = (-.4, .4)
  assert not run(SignalStopPrepare(), aligned(bounds), reds=(1, 2, 3))[-1].active


@pytest.mark.parametrize('green', [None, (-.5, .5), (.6, 1.7), (-1.7, -.6),
                                  (float('nan'), 3.), (2., float('inf')), (3., 2.)])
def test_ambiguous_green_never_loses_to_central_red(green):
  ds = run(SignalStopPrepare(), aligned({1: (-.5, .5), 4: green}))
  assert all(not d.active and d.reason == 'mixed_colors' for d in ds)


@pytest.mark.parametrize('red', [None, (-2., -.8), (.76, .9), (float('nan'), .5)])
def test_offcenter_or_uncertain_single_red_has_no_priority(red):
  assert not run(SignalStopPrepare(), aligned({1: red, 4: (2., 3.)}))[-1].active


def test_all_greens_must_be_clearly_separated():
  c = aligned({1: (-.5, .5), 3: (-.3, .3), 4: (2., 3.)})
  assert not run(SignalStopPrepare(), c, greens=(3, 4))[-1].active


def test_same_housing_or_duplicate_identity_color_conflict_cannot_be_ranked():
  c = context()
  c['alignment'] = lambda box, distance: (-.5, .5) if box[0] < 560 else (2., 3.)
  o = scene(0.)
  o['tracks'][1]['box'] = [559, 260, 599, 270]
  assert SignalStopPrepare().update(0., o, **c).reason == 'mixed_colors'
  o = scene(0.)
  o['tracks'][1]['id'] = 1
  assert SignalStopPrepare().update(0., o, **c).reason == 'mixed_colors'


def test_priority_uses_stable_identities_and_current_geometry():
  p = SignalStopPrepare()
  c = aligned({1: (-.5, .5), 4: (2., 3.), 5: (2., 3.)})
  assert run(p, c)[-1].active
  # A newly appearing competing green must re-establish separation over time.
  assert not run(p, c, greens=(5,), start=8, stop=9)[-1].active
  assert run(p, c, greens=(5,), start=9, stop=15)[-1].active
  c = aligned({1: (-.5, .5), 5: (.6, 1.4)})
  assert run(p, c, greens=(5,), start=15, stop=16)[-1].reason == 'mixed_colors'
  # Restored centrality cannot inherit the invalidated confirmation.
  c = aligned({1: (-.5, .5), 5: (2., 3.)})
  assert not run(p, c, greens=(5,), start=16, stop=17)[-1].active


def test_pair_loses_priority_when_one_support_disappears():
  p = SignalStopPrepare()
  c = aligned({1: (-1.2, -.8), 2: (.8, 1.2), 4: (2., 3.)})
  assert run(p, c, reds=(1, 2))[-1].active
  assert not run(p, c, reds=(1,), start=8, stop=9)[-1].active


@pytest.mark.parametrize('field,value', [('lead', True), ('turning', True), ('gas', True), ('brake', True),
                                        ('model_stopping', True), ('valid', False), ('speed', 82/3.6)])
def test_priority_preserves_existing_context_gates(field, value):
  p = SignalStopPrepare()
  c = aligned({1: (-.5, .5), 4: (2., 3.)})
  assert run(p, c)[-1].active
  c[field] = value
  assert not run(p, c, start=8, stop=9)[-1].active


def test_repeated_frames_and_new_red_cannot_inherit_priority():
  p = SignalStopPrepare()
  c = aligned({1: (-.5, .5), 2: (-.5, .5), 4: (2., 3.)})
  for t in (0., .05, .1, .15):
    assert not p.update(t, scene(0.), **c).active
  p = SignalStopPrepare()
  assert run(p, c)[-1].active
  assert not run(p, c, reds=(2,), start=8, stop=9)[-1].active
  o = scene(.9, reds=(2,))
  o['session'] = 'new'
  assert not p.update(.9, o, **c).active


def test_alignment_uses_lane_center_not_image_center():
  g = geometry()
  g['edge_stds'] = [2., 2.]
  for curve in g['lane_lines']:
    curve.y += 3.
  centered_on_lane = housing_alignment([737, 260, 777, 270], (40., 42.), **g)
  screen_center = housing_alignment([652, 260, 692, 270], (40., 42.), **g)
  assert max(abs(v) for v in centered_on_lane) < .75
  assert max(abs(v) for v in screen_center) > 1.25


def test_alignment_includes_full_housing_distance_and_interior_curve_knots():
  g = geometry()
  g['edge_stds'] = [2., 2.]
  box = [692, 260, 732, 270]
  short = housing_alignment(box, (30., 35.), **g)
  long = housing_alignment(box, (30., 90.), **g)
  assert max(abs(v) for v in short) < 1.25 < max(abs(v) for v in long)
  for curve in g['lane_lines']:
    curve.x = np.array([0., 40., 40.1, 40.2, 200.])
    curve.y = np.full(5, curve.y[0])
    curve.y[2] += 4.
    curve.z = np.zeros(5)
  # A narrow lane kink between uniform samples must still defeat centrality.
  result = housing_alignment([652, 260, 692, 270], (30., 70.), **g)
  assert max(abs(v) for v in result) > 2.


def test_alignment_rejects_edges_only_bad_calibration_and_extrapolation():
  g = geometry()
  box = [652, 260, 692, 270]
  assert housing_alignment(box, (30., 70.), **g) is not None
  g['lane_probs'] = [.1]*4
  assert housing_alignment(box, (30., 70.), **g) is None
  g = geometry()
  g['rpy'] = [0., float('nan'), 0.]
  assert housing_alignment(box, (30., 70.), **g) is None
  g = geometry()
  for curve in (*g['edges'], *g['lane_lines']):
    curve.x = np.linspace(0., 60., 33)
  assert housing_alignment(box, (30., 70.), **g) is None


def test_actual_projected_red_and_side_green_reach_prepare():
  g = geometry()
  g['edges'][0].y = np.full(33, -8.)
  g['edges'][1].y = np.full(33, 8.)
  c = context()
  c['geometry'] = lambda b: housing_range(b, **g)
  c['alignment'] = lambda b, d: housing_alignment(b, d, **g)
  p = SignalStopPrepare()
  for i in range(8):
    t = i*.1
    o = scene(t)
    o['tracks'][0]['box'] = [652, 260, 692, 270]
    o['tracks'][1]['box'] = [812, 260, 852, 270]
    d = p.update(t, o, **c)
  assert d.active and d.priority == 'central_red'


@pytest.mark.parametrize('actuate', [False, True])
def test_planner_receives_alignment_and_model_stop_still_wins(actuate):
  from tools.signal_analysis.replay_signal_assist import planner_class, REPO  # noqa: TID251 -- root analysis tools
  from tools.signal_analysis.test_signal_assist_planner import inputs  # noqa: TID251 -- root analysis tools
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p = cls(stop_prepare=SignalStopPrepare())
  sm = inputs(speed=15., distance=160.)
  c = aligned({1: (-.5, .5), 4: (2., 3.)})
  c['actuate'] = actuate
  for i in range(20):
    t = i*.05
    p.update(sm, 80., 'acc', prepare_observation=scene(t), prepare_context=dict(c, now=t))
  assert p.xState.name == ('e2eStopPrepare' if actuate else 'e2eCruise')
  assert p.prepare_decision.active and p.prepare_decision.priority == 'central_red'
  sm['modelV2'].position.x = [25*i/32 for i in range(33)]
  sm['modelV2'].velocity.x = [15.]+[0.]*32
  for i in range(20, 40):
    t = i*.05
    p.update(sm, 80., 'acc', prepare_observation=scene(t), prepare_context=dict(c, now=t))
  assert p.xState.name == 'e2eStop' and not p.prepare_decision.active
