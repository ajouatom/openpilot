from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carstate import CarState, VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES


class FakeParams:
  def __init__(self, value=60):
    self.value = value
    self.read_count = 0

  def get_int(self, key):
    assert key == "VehicleSpeedCameraDistanceTime"
    self.read_count += 1
    return self.value


def _car_state(distance_time_tenths=60):
  state = CarState.__new__(CarState)
  state.op_params = FakeParams(distance_time_tenths)
  state.totalDistance = 0.0
  state.speedLimitDistance = 0.0
  state.vehicleSpeedCameraDistanceTime = CarState._vehicle_speed_camera_distance_time(distance_time_tenths)
  state.vehicleSpeedCameraParamsCounter = 0
  return state


@pytest.mark.parametrize("gas_pressed", (False, True))
def test_vehicle_speed_camera_distance_is_independent_of_accelerator(gas_pressed):
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=gas_pressed)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(300.0)


def test_vehicle_speed_camera_distance_uses_tenths_of_a_second():
  state = _car_state(distance_time_tenths=62)
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(310.0)


@pytest.mark.parametrize(("raw_value", "expected"), (
  (0, 1.0),
  (10, 1.0),
  (62, 6.2),
  (200, 20.0),
  (999, 20.0),
))
def test_vehicle_speed_camera_distance_time_is_clamped_and_scaled(raw_value, expected):
  assert CarState._vehicle_speed_camera_distance_time(raw_value) == pytest.approx(expected)


def test_vehicle_speed_camera_distance_time_reloads_every_second():
  state = _car_state()
  state.op_params.value = 62
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert state.op_params.read_count == 1
  assert state.vehicleSpeedCameraDistanceTime == pytest.approx(6.2)
  assert ret.speedLimitDistance == pytest.approx(310.0)


def test_vehicle_speed_camera_params_are_not_read_every_frame():
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  for _ in range(VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1):
    state.update_speed_limit(ret, speed_limit_cam=True)

  assert state.op_params.read_count == 0


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
