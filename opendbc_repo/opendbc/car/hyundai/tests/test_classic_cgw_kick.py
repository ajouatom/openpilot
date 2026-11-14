from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiSafetyFlags


def car_params(candidate, cgw1, kick_mode=1):
  class Settings:
    def get_bool(self, key):
      return key == "CanfdBlinkerKick"

    def get_int(self, key):
      return kick_mode if key == "CanfdBlinkerKickMode" else 0

  fingerprint = gen_empty_fingerprint()
  if cgw1:
    fingerprint[0][0x541] = 8
  with patch("opendbc.car.hyundai.interface.Params", Settings):
    return CarInterface.get_params(candidate, fingerprint, [], False, False, False)


@pytest.mark.parametrize("candidate", [CAR.HYUNDAI_PALISADE, CAR.KIA_K5_2021, CAR.HYUNDAI_GENESIS])
def test_classic_cars_with_a_stock_cgw1_get_the_cgw_kick(candidate):
  # Palisade only until 11-14: any classic model whose bus 0 carries a stock CGW1 (0x541) may forge it
  param = car_params(candidate, cgw1=True).safetyConfigs[-1].safetyParam
  assert param & HyundaiSafetyFlags.PALISADE_CGW_TEST
  assert not param & HyundaiSafetyFlags.CANFD_SPAS_DIRECT  # mode 1: panda piggyback
  assert car_params(candidate, cgw1=True, kick_mode=2).safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_SPAS_DIRECT
  assert not car_params(candidate, cgw1=False).safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.PALISADE_CGW_TEST


def test_pedal_interceptor_keeps_its_exact_safety_param():
  # panda's pedal branch accepts one exact safetyParam (safety_hyundai.h): an extra CGW1 flag would block every TX
  from opendbc.car.hyundai.tests.test_pedal_interceptor import car_params as pedal_params, pedal_fingerprint
  fingerprint = pedal_fingerprint()
  fingerprint[0][0x541] = 8
  param = pedal_params(CAR.KIA_RAY_EV, True, fingerprint).safetyConfigs[-1].safetyParam
  assert param == pedal_params(CAR.KIA_RAY_EV, True, pedal_fingerprint()).safetyConfigs[-1].safetyParam
  assert not param & HyundaiSafetyFlags.PALISADE_CGW_TEST
