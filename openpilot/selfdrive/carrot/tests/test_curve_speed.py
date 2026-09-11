import math
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot.curve_speed import (
  APPROACH_DECEL, CurveSpeed, NO_LIMIT_KPH, VisionCurveSpeed, curve_speed,
)
from openpilot.selfdrive.modeld.constants import ModelConstants


TIMES = np.asarray(ModelConstants.T_IDXS)


def model(speed=15.0, curvature=0.01, start=0.0):
  distance = TIMES * speed
  curves = np.where(distance >= start, curvature, 0.0)
  return SimpleNamespace(
    position=SimpleNamespace(x=distance, y=np.zeros_like(TIMES), z=np.zeros_like(TIMES)),
    velocity=SimpleNamespace(x=np.full_like(TIMES, speed)),
    orientationRate=SimpleNamespace(z=curves * speed),
  )


@pytest.mark.parametrize('direction', [-1, 1])
def test_same_curve_has_same_target_at_different_model_and_vehicle_speeds(direction):
  # Radius 100 m at 1.9 m/s2 gives sqrt(190) m/s, independently of model speed.
  expected = math.sqrt(190.0) * 3.6
  for prediction in (25 / 3.6, 50 / 3.6, 75 / 3.6):
    for ego in (0.0, 25 / 3.6, 50 / 3.6, 75 / 3.6):
      result = curve_speed(model(prediction, direction * .01), ego)
      assert result.curve_kph == pytest.approx(expected)
      assert result.approach_kph == pytest.approx(expected)
      assert result.direction == direction


def test_remaining_distance_relaxes_approach_but_not_curve_target():
  results = [curve_speed(model(20., .025, start), 60 / 3.6) for start in (0., 40., 80.)]
  assert results[0].approach_kph < results[1].approach_kph < results[2].approach_kph
  assert len({r.curve_kph for r in results}) == 1
  for result in results:
    # Even without the extra response allowance, the approach is within a
    # constant-deceleration envelope that reaches the curve speed in time.
    required = ((result.approach_kph / 3.6)**2 - (result.curve_kph / 3.6)**2) / (2 * max(result.distance, .001))
    assert required <= APPROACH_DECEL


def test_response_allowance_tightens_when_currently_accelerating():
  path = model(20., .025, 80.)
  coast = curve_speed(path, 60 / 3.6)
  accelerating = curve_speed(path, 60 / 3.6, a_ego=1.5)
  assert accelerating.approach_kph < coast.approach_kph
  assert accelerating.curve_kph == coast.curve_kph


def test_sensitivity_preserves_direction_and_inverse_square_root_strength():
  path = model()
  normal = curve_speed(path, 15.)
  assert curve_speed(path, 15., .5).curve_kph == pytest.approx(normal.curve_kph * math.sqrt(2))
  assert curve_speed(path, 15., 2.).curve_kph == pytest.approx(normal.curve_kph / math.sqrt(2))


def test_floor_and_cluster_conversion_are_applied_before_distance_envelope():
  path = model(curvature=.01)
  nominal = curve_speed(path, 15.)
  scaled = curve_speed(path, 15., speed_ratio=.9)
  assert scaled.approach_kph * .9 == pytest.approx(nominal.approach_kph)
  assert curve_speed(model(curvature=.2), 15., lower_limit_kph=35., speed_ratio=.9).curve_kph == 35.


@pytest.mark.parametrize('index', [0, 1, 8, 17, 23])
def test_isolated_yaw_spike_does_not_create_a_curve(index):
  path = model(curvature=0.)
  path.orientationRate.z[index] = 3.
  assert curve_speed(path, 15.).approach_kph == NO_LIMIT_KPH


def test_sustained_curve_and_both_s_curve_directions_survive_filter():
  path = model(curvature=.012)
  path.orientationRate.z[TIMES > 1.] = -.03 * 15.
  result = curve_speed(path, 15., lower_limit_kph=20.)
  assert result.direction == -1
  assert result.curve_kph == pytest.approx(math.sqrt(1.9 / .03) * 3.6)


def test_distant_tail_and_unreachable_curve_are_not_used():
  path = model(curvature=0.)
  path.orientationRate.z[TIMES > 6.] = math.nan
  assert curve_speed(path, 15.).approach_kph == NO_LIMIT_KPH
  assert curve_speed(model(30., .1, start=100.), 3.).approach_kph == NO_LIMIT_KPH


def test_slow_model_is_not_a_speed_target_and_invalid_input_is_not_zero():
  assert curve_speed(model(speed=.5, curvature=.5), 15.) is None
  broken = model()
  broken.position.x = []
  assert curve_speed(broken, 15.) is None
  assert curve_speed(model(), math.nan) is None
  assert curve_speed(model(), 15., sensitivity=math.nan) is None
  assert curve_speed(model(), 15., sensitivity=0.) is None


def test_invalid_middle_node_does_not_join_unrelated_curvatures():
  path = model(curvature=0.)
  path.velocity.x[10] = math.nan
  path.orientationRate.z[9:12] = 2.
  assert curve_speed(path, 15.).approach_kph == NO_LIMIT_KPH


def test_straight_model_and_missing_initial_model_do_not_limit_cruise():
  state = VisionCurveSpeed()
  assert state.update(None, 0.) == NO_LIMIT_KPH
  assert state.update(curve_speed(model(curvature=0.), 15.), .05) == NO_LIMIT_KPH


def test_tighter_curve_is_immediate_but_brief_release_does_not_jump():
  state = VisionCurveSpeed()
  assert state.update(CurveSpeed(40., 35., 20., -1.), 0.) == -40.
  for t in (.05, .10, .15, .2):
    assert state.update(CurveSpeed(), t) == -40.
  assert state.update(CurveSpeed(40., 35., 20., -1.), .25) == -40.
  assert state.update(CurveSpeed(30., 30., 0., 1.), .3) == 30.


def test_sustained_release_and_model_loss_recover_at_bounded_rate():
  state = VisionCurveSpeed()
  state.update(CurveSpeed(40.), 0.)
  outputs = [state.update(None, float(t)) for t in np.arange(.05, 1.05, .05)]
  assert outputs[0] == 40.
  assert 40. < outputs[-1] < 46.
  assert max(np.diff(outputs)) <= .360001
  # A scheduler pause is not permission to jump straight back to 250 km/h.
  before = state.speed
  assert state.update(None, 100.) <= before + 1.440001


def test_recorded_geometry_distinguishes_mild_bend_entry_and_exit():
  samples = json.loads((Path(__file__).parent / 'fixtures/curve_speed_models.json').read_text())
  results = {}
  for name, sample in samples.items():
    path = SimpleNamespace(**{key: SimpleNamespace(**value) for key, value in sample.pop('model').items()})
    results[name] = curve_speed(path, **sample)
  assert results['mild_bend'].approach_kph > 55.
  assert results['curve_approach'].approach_kph > 50.
  assert 25. < results['curve_entry'].curve_kph < 50.
  assert results['curve_entry'].distance < 20.
  assert results['curve_exit'].approach_kph > 50.
