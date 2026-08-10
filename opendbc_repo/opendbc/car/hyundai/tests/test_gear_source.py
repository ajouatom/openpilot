from openpilot.common.params import Params

from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiExtFlags


def test_canfd_gear_shifter_is_registered_and_selected():
  Params().put_int("HyundaiCameraSCC", 0)
  Params().put_int("CanfdHDA2", 0)
  fingerprint = gen_empty_fingerprint()
  ecan = CanBus(None, fingerprint).ECAN
  fingerprint[ecan][0x130] = 16
  Params().put("FingerPrints", str(fingerprint))

  CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  carstate = CarState(CP)
  parser = carstate.get_can_parsers_canfd(CP)[Bus.pt]

  assert CP.extFlags & HyundaiExtFlags.CANFD_GEARS_130
  assert carstate.gear_msg_canfd == "GEAR_SHIFTER"
  assert carstate.use_accelerator is False
  assert parser.dbc.name_to_msg["GEAR_SHIFTER"].address in parser.addresses


def test_canfd_ev_without_gear_shifter_keeps_accelerator_source():
  Params().put_int("HyundaiCameraSCC", 0)
  Params().put_int("CanfdHDA2", 0)
  fingerprint = gen_empty_fingerprint()
  Params().put("FingerPrints", str(fingerprint))

  CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  carstate = CarState(CP)

  assert not CP.extFlags & HyundaiExtFlags.CANFD_GEARS_130
  assert carstate.gear_msg_canfd == "ACCELERATOR"
  assert carstate.use_accelerator is True
