from types import SimpleNamespace

from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.values import CAR


def angle_max(car, kph):
  cc = SimpleNamespace(CP=SimpleNamespace(carFingerprint=car),
                       params=SimpleNamespace(ANGLE_LIMITS=SimpleNamespace(STEER_ANGLE_MAX=175)))
  return CarController.steer_angle_max(cc, kph / 3.6)


def test_ioniq_5_pe_turns_past_175_only_at_parking_speeds():
  assert angle_max(CAR.HYUNDAI_IONIQ_5_PE, 5) == 450
  assert angle_max(CAR.HYUNDAI_IONIQ_5_PE, 25) == 312.5   # tapers 20 -> 30 km/h
  assert angle_max(CAR.HYUNDAI_IONIQ_5_PE, 40) == 175
  assert angle_max(CAR.KIA_EV9, 5) == 175
