import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_braking_gap import BrakingGapState, braking_gap_reference


def update(state, **kwargs):
  args = {'enabled': True, 'track_id': 43, 'dt': .05, 'distance': 20., 'ego_speed': 15., 'ego_accel': -.5,
              'lead_speed': 14.5, 'lead_accel': -.5, 'relative_speed': -.5, 'base_tf': .6, 'stop_distance': 5., 'desired_distance': 17.}
  args.update(kwargs)
  return state.update(**args)


def test_distance_memory_survives_falling_speed_without_accumulating_each_frame():
  state = BrakingGapState()
  for _ in range(20):
    update(state)
  captured = state.floor
  for i in range(150):
    update(state, ego_speed=max(2., 15. - i * .08), lead_speed=max(1.5, 14.5 - i * .08), desired_distance=10.)
  assert 10. < state.floor < captured <= 20.
  assert 0. < state.margin < 10.


@pytest.mark.parametrize('change', [{'enabled': False}, {'track_id': 44}, {'distance': 100.}, {'lead_accel': float('nan')}])
def test_lost_replaced_or_jumped_track_cannot_inherit_distance_memory(change):
  state = BrakingGapState()
  for _ in range(20):
    update(state)
  assert state.margin > 0.
  assert update(state, **change) == 0.
  assert state.floor is None


def test_opening_and_constant_speed_do_not_create_braking_memory():
  for change in [{'relative_speed': .5}, {'ego_accel': 0.}, {'lead_accel': 0.}]:
    state = BrakingGapState()
    for _ in range(40):
      assert update(state, **change) == 0.


def test_prediction_does_not_mutate_live_state_and_matches_repeated_advances():
  state = BrakingGapState()
  for _ in range(30):
    update(state)
  before = vars(state).copy()
  t = np.arange(21) * .05
  actual = state.margins(t, np.full(21, 10.), np.full(21, 9.5), np.full(21, 10.), .6, 5.)
  assert vars(state) == before
  expected = [state.margin]
  for _ in range(20):
    state._advance(.05, 11., 10., 10., 9.5)
    expected.append(state.margin)
  np.testing.assert_allclose(actual, expected)


def relief(state, **changes):
  args = {'dt': .05, 'enabled': True, 'base_tf': 1.1, 'ego_speeds': np.full(13, 20.), 'lead_speeds': np.full(13, 19.),
              'lead_accel': -1., 'physical_gaps': np.full(13, 40.), 'danger_margins': np.full(13, 10.),
              'times': np.linspace(0, 10, 13), 'stop_distance': 5.}
  args.update(changes)
  return state.comfort_relief(**args)


def test_comfort_allowance_is_bounded_and_danger_withdraws_it_immediately():
  state = BrakingGapState()
  state.age, state.distance, state.relative_speed = 2., 40., -1.
  for _ in range(20):
    result = relief(state)
  np.testing.assert_allclose(result, 8.)
  assert not relief(state, danger_margins=np.full(13, -1.)).any()
  assert state.relief == 0.


@pytest.mark.parametrize('change', [{'base_tf': .45}, {'enabled': False}, {'lead_speeds': np.zeros(13)},
                                    {'physical_gaps': np.full(13, 10.)}, {'lead_accel': 0.}])
def test_short_gap_stopped_lead_and_invalid_eligibility_never_get_relief(change):
  state = BrakingGapState()
  state.age, state.distance, state.relative_speed = 2., 40., -1.
  assert not relief(state, **change).any()


def test_slack_does_not_move_physical_obstacles_or_relax_another_obstacle():
  obstacles = np.array([[50., 30., 100.], [20., 30., 19.]])
  original = obstacles.copy()
  result = braking_gap_reference(obstacles, np.array([[-5., 0.], [-5., -5.]]), np.array([20., 20.]))
  np.testing.assert_array_equal(obstacles, original)
  np.testing.assert_array_equal(result, 0.)


def test_stopped_pair_retains_distance_until_departure():
  state = BrakingGapState()
  for _ in range(20):
    update(state)
  captured = state.floor
  for _ in range(600):
    update(state, distance=20., ego_speed=0., ego_accel=0., lead_speed=0., lead_accel=0.,
           relative_speed=0., desired_distance=5.)
  assert state.floor == captured
  held = state.margin
  for _ in range(400):
    update(state, distance=20., ego_speed=0., ego_accel=0., lead_speed=1., lead_accel=.5,
           relative_speed=1., desired_distance=5.)
  assert state.margin < held * .02


@pytest.mark.parametrize('field', ['danger_margins', 'physical_gaps', 'ego_speeds'])
def test_nonfinite_prediction_withdraws_comfort_relief(field):
  state = BrakingGapState()
  state.age, state.distance, state.relative_speed = 2., 40., -1.
  relief(state)
  assert state.relief > 0.
  assert not relief(state, **{field: np.full(13, np.nan)}).any()
