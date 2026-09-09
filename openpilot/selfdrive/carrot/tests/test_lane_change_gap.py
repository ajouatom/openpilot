from dataclasses import replace
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.carrot.radar_motion.lane_change_gap import (
  GapLead, LaneChangeGapPlan, LaneChangeGapTracker,
)
from openpilot.selfdrive.carrot.t_follow import ramp_t_follow


def lead(distance=35.0, speed=15.0, lateral=0.0, track=1, accel=0.0):
  return GapLead(track, distance, lateral, speed - 15.0, speed, accel)


def departure(direction=1, *, stopped_ego=False, blindspot=False, target_distance=120.0,
              path_reentry=False, yaw_only=False, target_switch=False):
  tracker = LaneChangeGapTracker()
  heading, ego_y = 0.0, 0.0
  results = []
  for i in range(51):
    t = 1.0 + i * .05
    yaw = 0.04 if not stopped_ego else 0.0
    if i:
      ego_y += 15.0 * math.sin(heading + direction * yaw * .025) * .05
      heading += direction * yaw * .05
    # Raw radar azimuth rotates with ego. Keep the original lead in its lane
    # in the fixed frame; yaw_only deliberately has no real ego translation.
    physical_y = 0.0 if yaw_only else ego_y
    radar_y = (physical_y + 35.0 * math.sin(heading)) / math.cos(heading)
    times = np.linspace(0, 4, 81)
    # A smooth lane-change path, represented in the CURRENT ego frame.
    world_y = direction * (3.6 - (3.6 - direction * physical_y) * np.exp(-times))
    if path_reentry:
      world_y[times > 2.0] = 0.0
    xs = 15.0 * times
    ys = (world_y - physical_y - xs * math.sin(heading)) / math.cos(heading)
    results.append(tracker.update(
      now=t, direction=direction, v_ego=15.0, yaw_rate=direction * yaw,
      path_t=tuple(times), path_x=tuple(xs), path_y=tuple(ys),
      primary=lead(lateral=radar_y),
      secondary=lead(target_distance, lateral=-direction * 3.6, track=2 + int(target_switch and i % 2 == 0)),
      blindspot=blindspot,
    ))
  return tracker, results


@pytest.mark.parametrize('direction', [-1, 1])
def test_measured_departure_requires_confirmation_and_preserves_target(direction):
  _, plans = departure(direction)
  assert plans[0].confidence == 0.0
  assert plans[-1].confidence > 0.0, plans[-1].reason
  assert plans[-1].targets[0].radarTrackId == 2
  assert .35 <= plans[-1].clearance_s <= 2.5


@pytest.mark.parametrize('kwargs', [{'stopped_ego': True}, {'blindspot': True}, {'path_reentry': True}, {'target_switch': True}, {'yaw_only': True}])
def test_intent_blindspot_reentry_and_unstable_targets_never_grant_credit(kwargs):
  _, plans = departure(**kwargs)
  assert all(p.confidence == 0.0 for p in plans)
  assert plans[-1].targets  # selected leadTwo remains an acceleration-credit guard only


def plan(**kwargs):
  return replace(LaneChangeGapPlan(True, 1, (lead(120., track=2),), 1., 1., 'test'), **kwargs)


def credit(p=None, primary=None, **kwargs):
  args = {'v_ego': 15., 'max_accel': 1., 't_follow': 1.3, 'stop_distance': 6., 'ratio': .9}
  args.update(kwargs)
  times = np.linspace(0, 10, 101)
  return times, (p or plan()).credit(primary or lead(), times, **args)


def test_credit_is_only_after_clearance_and_never_changes_common_tf():
  times, values = credit()
  assert np.all(values[times <= 1.35] == 0.0)
  assert values.max() == pytest.approx(1.95)


@pytest.mark.parametrize('ratio', [.2, .5, .8, .9, .95, 1., 0., -1., math.nan, math.inf])
def test_old_aggressive_values_are_bounded_and_invalid_values_disable(ratio):
  _, values = credit(ratio=ratio)
  assert values.max() <= min(4., .25 * 15., .2 * 1.3 * 15.)
  if not 0 < ratio < 1:
    assert not np.any(values)
  if 0 < ratio <= .8:
    np.testing.assert_array_equal(values, credit(ratio=.8)[1])


@pytest.mark.parametrize('primary', [lead(10.), lead(speed=5.), lead(accel=-2.), lead(track=9)])
def test_close_braking_closing_and_replaced_primary_block_credit(primary):
  assert not np.any(credit(primary=primary)[1])


@pytest.mark.parametrize('target', [lead(20., track=2), lead(45., speed=0., track=2), lead(50., speed=10., track=2)])
def test_destination_gap_is_checked_through_entire_accelerating_transition(target):
  assert not np.any(credit(p=plan(targets=(target,)))[1])


def test_every_destination_target_keeps_its_own_limit():
  assert not np.any(credit(p=plan(targets=(lead(120., track=2), lead(25., track=3))))[1])


def test_missing_destination_or_small_base_tf_cannot_grant_credit():
  assert not np.any(credit(p=plan(targets=()))[1])
  assert not np.any(credit(t_follow=.6)[1])


@pytest.mark.parametrize('direction,valid', [(0, True), (1, False)])
def test_cancel_or_invalid_input_revokes_immediately(direction, valid):
  tracker, _ = departure()
  p = tracker.update(now=2.6, direction=direction, valid=valid, v_ego=15., yaw_rate=.04,
                     path_t=(), path_x=(), path_y=(), primary=lead(), secondary=lead(120., track=2))
  assert p.confidence == 0.0


def test_missing_pose_retains_selected_lead_metadata_without_relief():
  p = LaneChangeGapTracker().update(now=1., direction=1, v_ego=15., yaw_rate=math.nan,
                                  path_t=(), path_x=(), path_y=(), primary=lead(), secondary=lead(25., track=2))
  assert p.targets and p.confidence == 0.0


def test_repeated_or_stale_frames_cannot_accumulate_evidence():
  tracker, _ = departure()
  p = tracker.update(now=10., direction=1, v_ego=15., yaw_rate=.04,
                     path_t=(), path_x=(), path_y=(), primary=lead(), secondary=lead(120., track=2))
  assert p.confidence == 0.0


@pytest.mark.parametrize('changing,ratio', [(True, .2), (True, .8), (True, 1.), (False, 1.)])
def test_production_tf_update_cycle_has_no_repeated_reduction(changing, ratio):
  # Load the real methods without importing Params/hardware/UI native modules.
  path = Path(__file__).resolve().parents[1] / 'carrot_functions.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarrotPlanner')
  methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('get_T_FOLLOW',)]
  namespace = {'LEAD_ACCEL_DEADBAND': .1, 'LEAD_ACCEL_CONFIGURED_TF_MIN': 4, 'np': np, 'DT_MDL': .05,
               'ramp_t_follow': ramp_t_follow, 'log': NS(LongitudinalPersonality=NS(standard=1))}
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
  planner_type = type('ActualTFMethods', (), {n.name: namespace[n.name] for n in methods})
  p = planner_type()
  p._get_base_t_follow = lambda *a, **kw: 1.3
  p._apply_speed_t_follow_scale = lambda tf, v: tf
  p._apply_decel_hold_and_boost_t_follow = lambda tf, a: tf
  p._clip_t_follow = lambda tf: tf
  p.myTFollowFactor = 1.
  p._tf_decel_extra = 0.
  p.t_follow_last = 1.3
  p.lane_change_active = changing
  p.dynamicTFollowLC, p.leadAccelResponse, p.jerk_factor = ratio, 0, .7
  p.desireState, p.desireStateCount = 1., 1
  for _ in range(100):
    tf = p.get_T_FOLLOW()
    assert tf == pytest.approx(1.3)
    assert p.t_follow_last == pytest.approx(1.3)


@pytest.mark.parametrize('replace_primary', [True, False])
def test_entry_pair_change_revokes_credit_for_rest_of_maneuver(replace_primary):
  tracker, plans = departure()
  assert plans[-1].confidence > 0.0
  assert plans[-1].entry_ids == (1, 2)
  args = {'direction': 1, 'v_ego': 15., 'yaw_rate': .04, 'path_t': (), 'path_x': (), 'path_y': ()}
  changed = tracker.update(now=3.55, primary=lead(track=9 if replace_primary else 1),
                           secondary=lead(120., track=2) if replace_primary else None, **args)
  assert changed.reason == 'selected-leads-changed'
  assert changed.confidence == 0.0
  assert changed.entry_ids == (1, 2)
  restored = tracker.update(now=3.60, primary=lead(), secondary=lead(120., track=2), **args)
  assert restored.reason == 'selected-leads-changed'
  assert restored.confidence == 0.0


def test_production_adapter_reads_only_selected_pair():
  path = Path(__file__).resolve().parents[1] / 'carrot_functions.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_update_lane_change_gap')
  namespace = {'LaneChangeGapPlan': LaneChangeGapPlan}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  planner = NS(lane_change_active=True, _lane_change_tracker=LaneChangeGapTracker(), _lane_change_model_ns=0)
  empty = NS(status=False)
  # No side candidate attributes exist: any read would fail this test.
  sm = {'carState': NS(leftBlinker=False, rightBlinker=True, rightBlindspot=False, vEgo=26.),
        'modelV2': NS(position=NS(t=(), x=(), y=())),
        'radarState': NS(leadOne=empty, leadTwo=empty),
        'livePose': NS(inputsOK=True, sensorsOK=True, angularVelocityDevice=NS(valid=True, z=.01))}
  class SM(dict):
    valid = alive = dict.fromkeys(sm, True)
    logMonoTime = dict.fromkeys(sm, 1_000_000_000)
  namespace['_update_lane_change_gap'](planner, SM(sm))
  assert planner.lane_change_gap.targets == ()
  assert planner.lane_change_gap.entry_ids == (-1, -1)
