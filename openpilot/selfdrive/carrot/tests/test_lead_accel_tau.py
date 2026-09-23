from dataclasses import replace
import math

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import RadarLeadDynamics
from openpilot.selfdrive.carrot.radar_motion.lead_dynamics import LeadAccelTau
from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot


def legacy(tau, a, j):
  return 1.5 if abs(a) < .5 and abs(j) < .5 else tau * .9


@pytest.mark.parametrize('a,j', [(-.4, -4.), (-3., -.8), (-3., 0.), (1., -4.), (1., 4.), (0., 0.), (-.5, -3.), (-2., -1.5)])
def test_ordinary_motion_keeps_existing_policy(a, j):
  state = LeadAccelTau()
  expected = 1.5
  for i in range(60):
    expected = legacy(expected, a, j)
    assert state.update(a, j, i * .05) == expected


def test_oscillating_jerk_and_single_negative_spike_do_not_accelerate_attack():
  state = LeadAccelTau()
  expected = 1.5
  for i, j in enumerate([-.8, .9, -1.25, .2, -4., .3, -4., 2., -4., 0.]):
    expected = legacy(expected, -1., j)
    assert state.update(-1., j, i * .05) == expected


def test_confirmed_braking_retains_more_future_deceleration_without_changing_a_lead():
  state = LeadAccelTau()
  assert state.update(-2., -4., 0.) == pytest.approx(1.35)
  tau = state.update(-2., -4., .05)
  assert tau == pytest.approx(.675)
  for i in range(2, 6):
    tau = state.update(-2., -4., i * .05)
  assert tau < .05  # 250 ms after corroborated onset, versus old 0.797.
  assert -2. * math.exp(-tau * 2.**2 / 2.) < -1.8


def test_attack_strength_is_continuous_and_bounded():
  previous = 2.
  for j in [-1.5, -1.500001, -1.8, -2.2, -2.8, -3., -30.]:
    state = LeadAccelTau()
    state.update(-2., j, 0.)
    tau = state.update(-2., j, .05)
    assert .675 <= tau <= 1.215 + 1e-12
    assert tau <= previous
    previous = tau


def test_constant_braking_keeps_low_tau_and_quiet_release_restores_default():
  state = LeadAccelTau()
  state.update(-3., -4., 0.)
  state.update(-3., -4., .05)
  last = state.tau
  for i in range(2, 20):
    # Jerk becomes zero during steady braking or positive as braking eases.
    tau = state.update(-2., 0. if i < 10 else 1., i * .05)
    assert tau == last * .9
    last = tau
  assert state.update(-.1, .1, 1.) == 1.5


@pytest.mark.parametrize('initial_tau', [1.5, .3])
def test_braking_correction_does_not_boost_subsequent_positive_acceleration(initial_tau):
  state = LeadAccelTau(initial_tau)
  ordinary = initial_tau
  for i in range(10):
    ordinary = legacy(ordinary, -2., -4.)
    state.update(-2., -4., i * .05)
  assert state.tau < ordinary
  # Even without a quiet sample between braking and acceleration, the braking
  # correction must not change the positive-acceleration prediction.
  for i in range(10, 20):
    ordinary = legacy(ordinary, 1., 2.)
    assert state.update(1., 2., i * .05) == ordinary


@pytest.mark.parametrize('second_time', [0., -.05, .151, 2.])
def test_duplicate_out_of_order_or_gapped_sample_cannot_confirm_attack(second_time):
  state = LeadAccelTau()
  state.update(-2., -4., 0.)
  assert state.update(-2., -4., second_time) == pytest.approx(1.215)


def test_unmeasured_sample_and_explicit_dropout_clear_braking_evidence():
  state = LeadAccelTau()
  state.update(-2., -4., 0.)
  state.update(-2., -4., .05, measured=False)
  assert state.update(-2., -4., .10) == pytest.approx(1.5 * .9**3)
  state.clear_evidence()
  assert state.update(-2., -4., .15) == pytest.approx(1.5 * .9**4)


def test_out_of_order_measurement_cannot_seed_next_confirmation():
  state = LeadAccelTau()
  state.update(-2., -4., .10)
  state.update(-2., -4., .05)
  assert state.update(-2., -4., .15) == pytest.approx(1.5 * .9**3)


@pytest.mark.parametrize('a,j,t', [(math.nan, -4., .05), (-2., math.inf, .05), (-2., -4., math.nan)])
def test_nonfinite_inputs_reset_evidence_and_never_poison_tau(a, j, t):
  state = LeadAccelTau()
  state.update(-2., -4., 0.)
  assert state.update(a, j, t) == 1.5
  assert state.update(-2., -4., .1) == pytest.approx(1.35)


def point(track_id=49, source='frontRadar'):
  return RadarPointSnapshot(track_id, source, 30., 0., -2., 0., 0., 18., -2., -4., True)


def test_controller_history_follows_physical_source_not_selected_corner_slot():
  dynamics = RadarLeadDynamics()
  front = point()
  corner = replace(point(1010, 'corner235'), kinematics_source='frontRadar', kinematics_track_id=49)
  dynamics.update((front,), 0.)
  dynamics.update((front,), .05)
  assert dynamics.a_lead_tau(corner) == pytest.approx(.675)
  # Reusing the numeric ID under a different sensor never confirms braking.
  other = point(49, 'scc')
  dynamics.update((front, other), .10)
  assert dynamics.a_lead_tau(other) == pytest.approx(1.35)
  dynamics.update((), .15)
  dynamics.update((front,), .20)
  assert dynamics.a_lead_tau(front) == pytest.approx(1.35)
  dynamics.reset()
  assert dynamics.a_lead_tau(front) == 1.5
