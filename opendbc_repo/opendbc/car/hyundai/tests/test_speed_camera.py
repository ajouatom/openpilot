from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carstate import CarState


def _car_state(distance_factor=6):
  state = CarState.__new__(CarState)
  state.totalDistance = 0.0
  state.speedLimitDistance = 0.0
  state.vehicleSpeedCameraDistanceFactor = distance_factor
  return state


@pytest.mark.parametrize("gas_pressed", (False, True))
def test_vehicle_speed_camera_distance_is_independent_of_accelerator(gas_pressed):
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=gas_pressed)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(300.0)


def test_vehicle_speed_camera_distance_uses_configured_factor():
  state = _car_state(distance_factor=10)
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(500.0)


@pytest.mark.parametrize(("speed_limit", "camera", "expected"), (
  (0.0, True, 0.0),
  (50.0, False, 0.0),
))
def test_vehicle_speed_camera_distance_requires_valid_camera_speed(speed_limit, camera, expected):
  state = _car_state()
  state.speedLimitDistance = 300.0
  ret = SimpleNamespace(vEgo=10.0, speedLimit=speed_limit, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=camera)

  assert ret.speedLimitDistance == pytest.approx(expected)
