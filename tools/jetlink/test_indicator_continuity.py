"""Recorded failure shapes and cancellation counterexamples for manual lamps."""
import math

import pytest

from test_indicator_reconstruction import scene
from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import ModelIndicator, Movement
from openpilot.selfdrive.controls.lib.desire_lib.manual_turn_indicator import ManualTurnMovement


def lines(md, left, right):
  md.laneLines[1].y = [left] * 13
  md.laneLines[2].y = [right] * 13


def test_recorded_left_retreat_then_pair_rebase_keeps_original_request():
  cs, md = scene()
  cs.steeringAngleDeg = 2.
  lines(md, -.160, 2.801)
  movement = Movement(10., 1, 'manual', cs, md)
  for i, pair in enumerate(((-.192, 2.82), (-.191, 2.773), (-.143, 2.812),
                            (-.186, 2.566), (-.361, 2.457), (-1.089, 1.648), (-2.475, .407)), 1):
    lines(md, *pair)
    assert movement.update(10.+i*.05, cs, md)
  assert movement.crossed
  lines(md, -1.8, 1.8)
  # Settle using ordinary small steps, not an arbitrary new line jump.
  movement.last = (10.35, -1.8, 1.8)
  for i in range(8):
    assert movement.update(10.4+i*.05, cs, md)
  assert not movement.update(10.85, cs, md)
  assert movement.reason == 'completed'


def recovery():
  cs, md = scene()
  cs.steeringAngleDeg = -3.
  cs.steeringPressed, cs.steeringTorque = True, -400.
  md.laneLineProbs = [0., .02, .04, 0.]
  lines(md, -2.186, .962)
  movement = Movement(10., 2, 'manual', cs, md)
  lines(md, -2.432, 1.293)
  assert movement.update(10.05, cs, md)
  return cs, md, movement


def test_uncertain_right_boundary_with_continuing_far_line_finishes_on_stable_geometry():
  cs, md, movement = recovery()
  for i in range(1, 12):
    lines(md, -2.5-min(i*.05, .5), 1.9)
    assert movement.update(10.05+i*.05, cs, md)
  assert movement.geometry_recovery is not None
  md.laneLineProbs = [.9] * 4
  lines(md, -1.8, 1.8)
  for i in range(1, 9):
    assert movement.update(10.6+i*.05, cs, md)
  assert not movement.update(11.1, cs, md)
  assert movement.reason == 'completed'


@pytest.mark.parametrize('fault', ['opposite_driver', 'nan', 'gap', 'too_wide', 'persistent_uncertainty'])
def test_boundary_recovery_is_bounded_and_never_masks_driver_or_input_cancellation(fault):
  cs, md, movement = recovery()
  if fault == 'opposite_driver':
    cs.steeringPressed, cs.steeringTorque, cs.steeringAngleDeg = True, 400., 1.
  elif fault == 'nan':
    md.laneLines[1].y[0] = math.nan
  elif fault == 'too_wide':
    lines(md, -4., 3.)
  if fault == 'persistent_uncertainty':
    lines(md, -2.7, 1.8)
    for i in range(1, 61):
      if not movement.update(10.05+i*.05, cs, md):
        break
    assert movement.reason == 'geometry' and i == 60
  else:
    assert not movement.update(10.3 if fault == 'gap' else 10.1, cs, md)


def test_coherent_persistent_return_is_not_kept_by_geometry_recovery():
  cs, md = scene()
  lines(md, -3.4, .2)
  movement = Movement(10., 2, 'manual', cs, md)
  lines(md, -3.1, .5)
  assert movement.update(10.05, cs, md)
  assert movement.geometry_recovery is None
  assert movement.update(10.15, cs, md)
  assert not movement.update(10.21, cs, md) and movement.reason == 'returned'


@pytest.mark.parametrize('source,started', [('model_path', 10.), ('model_desire', 10.), ('manual', 9.)])
def test_ordinary_model_or_late_return_keeps_existing_cancellation(source, started):
  cs, md = scene()
  lines(md, -3.4, .2)
  movement = Movement(started, 2, source, cs, md)
  movement.last = (10., -3.4, .2)
  lines(md, -3.1, .5)
  assert not movement.update(10.05, cs, md)
  assert movement.reason == 'returned'


def test_uncertain_boundary_without_sufficient_far_progress_cancels():
  cs, md = scene()
  cs.steeringAngleDeg = -3.
  cs.steeringPressed, cs.steeringTorque = True, -400.
  md.laneLineProbs = [0.] * 4
  lines(md, -2.2, 1.)
  movement = Movement(10., 2, 'manual', cs, md)
  lines(md, -2.31, 1.3)
  for i in range(1, 6):
    assert movement.update(10.+i*.05, cs, md)
  assert not movement.update(10.31, cs, md)
  assert movement.reason == 'geometry'


def successive(fault=None):
  cs, md = scene()
  cs.vEgo = 5.
  md.laneLineProbs = [0.] * 4
  detector, isolation = ModelIndicator(), BoundaryIndicator()
  detector.active, detector.request_time = ManualTurnMovement(10., 1), 10.
  cs.steeringPressed, cs.steeringTorque, cs.steeringAngleDeg = True, -400., -1.
  isolation.own(10., 1)
  detector.update(10.05, cs, md, valid=True, lateral_active=False, desire_enabled=False,
                  path_enabled=False, busy=False, isolation=isolation, manual_turn_enabled=True, yaw_rate=.2)
  assert detector.active is None and detector.reason == 'driver_opposite'
  starts=[]
  for i in range(1, 75):
    now=10.05+i*.05
    md.frameId=i
    cs.leftBlinker=now<10.5
    cs.steeringAngleDeg=-min(300., i*6.)
    if fault == 'late_growth':
      cs.steeringAngleDeg = -(min(160., i*8.) + max(0, min(i-40, 20))*.5 + max(0, i-60)*6.)
    if fault == 'low_speed':
      cs.vEgo = 7/3.6
    cs.blinkerLever=2 if fault=='lever' else 0
    isolation.update(now, cs, md, enabled=False, valid=fault!='invalid', busy=fault=='busy')
    detector.update(now, cs, md, valid=fault!='invalid', lateral_active=False, desire_enabled=False,
      path_enabled=False, busy=fault=='busy', isolation=isolation, manual_turn_enabled=fault!='disabled',
      yaw_rate=None if fault=='pose' else .2)
    if detector.active:
      starts.append((now, detector.active.direction, detector.active.source))
  return starts


def test_successive_opposite_turn_keeps_new_growth_without_bypassing_dark_or_rearm():
  starts=successive()
  assert starts and 12. <= starts[0][0] <=12.11
  assert {s[1:] for s in starts} == {(2, 'manual_turn')}


@pytest.mark.parametrize('fault', ['lever', 'invalid', 'busy', 'disabled', 'pose'])
def test_successive_turn_does_not_survive_explicit_cancellation_or_missing_inputs(fault):
  assert not successive(fault)


def test_low_speed_uses_fresh_ordinary_detection_without_rearm_memory():
  starts = successive('low_speed')
  assert starts and starts[0][0] >= 12.3


def test_retained_intent_expiry_never_erases_fresh_ordinary_turn_detection():
  starts = successive('late_growth')
  assert starts and 13.1 <= starts[0][0] <= 13.5


def short_lane_follow(fault=None):
  cs, md = scene()
  cs.vEgo = 5.
  cs.steeringPressed, cs.steeringTorque, cs.steeringAngleDeg = True, -400., -20.
  md.laneLineProbs = [0.] * 4
  lines(md, -2.6, 1.)
  detector, isolation = ModelIndicator(), BoundaryIndicator()
  detector.active, detector.request_time = Movement(10., 2, 'manual', cs, md), 10.
  detector.active.last = (10.45, -2.6, 1.)
  isolation.own(10.45, 2)
  lines(md, -2.3, 1.3)
  detector.update(10.5, cs, md, valid=True, lateral_active=False, desire_enabled=False,
    path_enabled=False, busy=False, isolation=isolation, manual_turn_enabled=True, yaw_rate=.2)
  assert detector.active is None and detector.follow_lane_turn
  result=[]
  for i in range(1, 41):
    now=10.5+i*.05
    md.frameId=i
    cs.steeringAngleDeg=-20.-i*3.
    cs.blinkerLever=2 if fault=='lever' else 0
    cs.leftBlinker=fault=='opposite_lamp'
    if fault=='opposite_torque':cs.steeringTorque=400.
    isolation.update(now, cs, md, enabled=False, valid=fault!='invalid', busy=fault=='busy')
    detector.update(now, cs, md, valid=fault!='invalid', lateral_active=False, desire_enabled=False,
      path_enabled=False, busy=fault=='busy', isolation=isolation, manual_turn_enabled=fault!='disabled',
      yaw_rate=None if fault=='pose' else .2)
    if detector.active:result.append((now,detector.active.direction,detector.active.source,detector.request_time))
  return result


def test_short_boundary_request_does_not_consume_next_independent_same_side_turn():
  starts=short_lane_follow()
  assert starts and 10.5 < starts[0][0] < 12.45
  assert starts[0][1:3] == (2,'manual_turn')
  assert starts[0][3] != 10.  # New request; never resurrect the ended lease.


@pytest.mark.parametrize('fault',['lever','invalid','busy','disabled','pose','opposite_lamp','opposite_torque'])
def test_short_boundary_turn_recovery_respects_driver_and_opposite_lamps(fault):
  assert not short_lane_follow(fault)


@pytest.mark.parametrize('fault', [None,'same_side','lit','invalid','lever'])
def test_short_attempt_rearms_only_fresh_opposite_approach_after_dark_second(fault):
  from test_boundary_recent_intent import maneuver
  from unittest.mock import patch
  import test_boundary_recent_intent as checks
  detector=BoundaryIndicator()
  detector.initialized=True
  detector.own(11.,1)
  detector.short_attempt_end=(11.,1)
  def gate(cs, md, now, frame):
    # Install the prior ended attempt immediately before this new approach.
    if frame==29:
      detector.own(now,1 if fault!='same_side' else 2)
      detector.short_attempt_end=(now,1 if fault!='same_side' else 2)
    distance=max(.3,1.8-max(0,frame-50)*.04)
    md.laneLines[1].y=[-(3.6-distance)]*9
    md.laneLines[2].y=[distance]*9
    cs.steeringPressed,cs.steeringTorque=True,-300.
    if fault=='lit':cs.leftBlinker=True
    if fault=='lever':cs.blinkerLever=2
    if fault=='invalid':cs.canValid=False
  with patch.object(checks,'BoundaryIndicator',return_value=detector):
    requests=maneuver(2,gate=gate)
  early=[r for r in requests if r[0]<14.45]
  assert bool(early)==(fault is None)
  assert detector.isolate_until>=23.
