import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params


@pytest.mark.parametrize("camera_scc", (0, 1))
@pytest.mark.parametrize(("candidate", "lfa_alt_bus", "expected_angle"), (
  (CAR.HYUNDAI_TUCSON_4TH_GEN, 2, False),
  (CAR.HYUNDAI_TUCSON_4TH_GEN, None, False),
  (CAR.KIA_SPORTAGE_5TH_GEN, 2, True),
  (CAR.KIA_SPORTAGE_5TH_GEN, 0, False),
  (CAR.KIA_SPORTAGE_5TH_GEN, None, False),
  (CAR.HYUNDAI_IONIQ_9, None, True),
))
def test_lfa_alt_steering_mode(candidate, lfa_alt_bus, expected_angle, camera_scc):
  Params().put_int("HyundaiCameraSCC", camera_scc)
  Params().put_int("CanfdHDA2", 0)
  fingerprint = gen_empty_fingerprint()
  if lfa_alt_bus is not None:
    fingerprint[lfa_alt_bus][0xCB] = 24

  CP = CarInterface.get_params(candidate, fingerprint, [], False, False, False)

  assert bool(CP.flags & HyundaiFlags.ANGLE_CONTROL) == expected_angle
  expected_type = structs.CarParams.SteerControlType.angle if expected_angle else structs.CarParams.SteerControlType.torque
  assert CP.steerControlType == expected_type
  if not expected_angle:
    assert CP.lateralTuning.which() == "torque"
