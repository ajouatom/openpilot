from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_safe_follow import SafeFollowState
from openpilot.selfdrive.controls.tests.test_longitudinal_gap_recovery import load_mpc_update, run_update


TIMES = np.array([0., .25, .5, 1., 2.])


def apply(state, *, cycles=1, level=5, driving_mode=2, enabled=True, track_id=49,
          gap_margin=.5, v_rel=0., a_lead=.2, a_ego=1.5, dt=.05, limits=None):
  if limits is None:
    limits = np.full(len(TIMES), 2.)
  for _ in range(cycles):
    actual = state.acceleration_limits(limits, TIMES, level=level, driving_mode=driving_mode,
                                      enabled=enabled, track_id=track_id, gap_margin=gap_margin, v_rel=v_rel,
                                      a_lead=a_lead, a_ego=a_ego, dt=dt)
  return actual


@pytest.mark.parametrize('level', [4, 5])
def test_prompt_launch_unchanged_at_first_frame_and_after_mode_settles(level):
  state = SafeFollowState()
  np.testing.assert_array_equal(apply(state, level=level, a_ego=0., a_lead=1.5), np.full(5, 2.))
  np.testing.assert_array_equal(apply(state, level=level, a_ego=.5, a_lead=1.5, cycles=20), np.full(5, 2.))


def test_taper_follows_launch_without_holding_large_acceleration():
  state = SafeFollowState()
  apply(state, cycles=20, a_ego=0., a_lead=1.5)
  actual = apply(state)
  np.testing.assert_allclose(actual, [1.5, 1.3, 1.1, .7, .45])
  # If the lead opens the gap again, immediately allow the original response.
  np.testing.assert_array_equal(apply(state, gap_margin=3., v_rel=1.), np.full(5, 2.))


def test_safe_to_normal_mode_transition_releases_progressively():
  state = SafeFollowState()
  before = apply(state, cycles=20)
  first = apply(state, driving_mode=3)
  assert np.all(first >= before) and np.any(first < 2.)
  np.testing.assert_array_equal(apply(state, driving_mode=3, cycles=20), np.full(5, 2.))


@pytest.mark.parametrize('driving_mode', [1, 3, 4])
@pytest.mark.parametrize('level', range(6))
def test_other_modes_and_levels_preserve_limits(driving_mode, level):
  np.testing.assert_array_equal(apply(SafeFollowState(), cycles=20, level=level, driving_mode=driving_mode), np.full(5, 2.))


@pytest.mark.parametrize('level', range(4))
def test_safe_lower_levels_unchanged(level):
  np.testing.assert_array_equal(apply(SafeFollowState(), cycles=20, level=level), np.full(5, 2.))


@pytest.mark.parametrize('kwargs', [{'enabled':False}, {'track_id':-1}, {'gap_margin':float('nan')}, {'dt':0.}])
def test_invalidation_drops_stale_tail(kwargs):
  state = SafeFollowState()
  apply(state, cycles=20)
  np.testing.assert_array_equal(apply(state, **kwargs), np.full(5, 2.))
  assert state.blend == 0.


def test_existing_stronger_limit_and_braking_always_available():
  original = np.array([1.5, 1., 0., -.5, -1.])
  actual = apply(SafeFollowState(), cycles=20, a_lead=-3., limits=original)
  assert np.all(actual <= original)
  np.testing.assert_array_equal(actual[2:], original[2:])
  assert actual[0] == 1.5


@pytest.fixture
def mpc_class():
  return load_mpc_update(Path(__file__).resolve().parents[1]/'lib/longitudinal_mpc_lib/long_mpc.py')


@pytest.mark.parametrize('level', [4, 5])
def test_mpc_preserves_prompt_launch_boost_and_trajectory_inputs(mpc_class, level):
  normal = run_update(mpc_class, level, driving_mode=3, lead_accel=1.5, ego_accel=0., lead_speed=22.)
  safe = run_update(mpc_class, level, driving_mode=2, lead_accel=1.5, ego_accel=0., lead_speed=22.)
  np.testing.assert_array_equal(normal.params, safe.params)
  np.testing.assert_array_equal(normal.yref, safe.yref)
  for key in normal.solver.weights:
    np.testing.assert_array_equal(normal.solver.weights[key], safe.solver.weights[key])


@pytest.mark.parametrize('level', [4, 5])
def test_mpc_taper_only_reduces_positive_acceleration_ceiling(mpc_class, level):
  args = {'level':level, 'distance':21., 'speed':20., 'lead_speed':20., 'lead_accel':.2, 'ego_accel':1.5}
  normal = run_update(mpc_class, driving_mode=3, **args)
  safe = run_update(mpc_class, driving_mode=2, **args)
  np.testing.assert_array_equal(normal.params[:,[0,2,3,4,5,6,7]], safe.params[:,[0,2,3,4,5,6,7]])
  assert np.all(safe.params[:,1] <= normal.params[:,1])
  assert np.any(safe.params[:,1] < normal.params[:,1])
  assert np.all(safe.params[:,1] >= 0.)
  assert safe.params[0,1] >= safe.x0[2]
  assert normal.source == safe.source
  np.testing.assert_array_equal(normal.yref, safe.yref)


def test_reset_clears_mode_transition(mpc_class):
  mpc = run_update(mpc_class, 5, driving_mode=2)
  assert mpc.safe_follow_state.blend == 1.
  mpc.reset()
  assert mpc.safe_follow_state.blend == 0.
