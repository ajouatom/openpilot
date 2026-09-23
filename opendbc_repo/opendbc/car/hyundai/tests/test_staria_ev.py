import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, STD_CARGO_KG, gen_empty_fingerprint
from opendbc.car import interfaces as interfaces_module
from opendbc.car.hyundai import carstate, hyundaicanfd, interface
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai.values import CAR, CANFD_CAR, EV_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.selected_car import get_selected_car_platform
from opendbc.car.structs import CarParams


@pytest.fixture
def staria_params(monkeypatch):
  # Observed bus layout: EV/vehicle signals on 0, radar on 1, camera SCC/LFA on 2.
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = {0x35: 32, 0x130: 16, 0x175: 24, 0x1AA: 16, 0x1BA: 24}
  fingerprint[1] = {0x100: 24, 0x110: 32, 0x3A5: 24}
  fingerprint[2] = {0xCB: 24, 0x12A: 16, 0x1A0: 32}
  values = {"HyundaiCameraSCC": 1, "CanfdHDA2": 0, "FingerPrints": repr(fingerprint),
            "VehicleSpeedCameraDistanceTime": 60}

  class Params:
    def get_int(self, key):
      return int(values.get(key, 0))

    def get_bool(self, key):
      return bool(values.get(key, False))

    def get(self, key):
      return values.get(key, "")

    def put_bool(self, key, value):
      pass

  for module in (interface, hyundaicanfd, carstate, interfaces_module):
    monkeypatch.setattr(module, "Params", Params)

  def make(candidate=CAR.HYUNDAI_STARIA_EV):
    return interface.CarInterface.get_params(candidate, fingerprint, [], False, False, False)

  return make, fingerprint, values


def test_staria_ev_manual_selection_and_specs(staria_params):
  make, _, _ = staria_params
  assert get_selected_car_platform("Hyundai Staria EV 2026") == CAR.HYUNDAI_STARIA_EV
  assert get_selected_car_platform("Hyundai Staria 2023") == CAR.HYUNDAI_STARIA_4TH_GEN
  assert CAR.HYUNDAI_STARIA_EV in EV_CAR & CANFD_CAR
  assert CAR.HYUNDAI_STARIA_4TH_GEN not in EV_CAR
  assert not FW_VERSIONS.get(CAR.HYUNDAI_STARIA_EV)  # No invented/copied automatic fingerprint.
  cp = make()
  assert cp.mass == pytest.approx(2590 + STD_CARGO_KG)
  assert cp.wheelbase == pytest.approx(3.275)
  assert cp.steerRatio == pytest.approx(CAR.HYUNDAI_STARIA_4TH_GEN.config.specs.steerRatio)


@pytest.mark.parametrize("candidate", (CAR.HYUNDAI_STARIA_EV, CAR.HYUNDAI_IONIQ_5, CAR.KIA_EV6, CAR.KIA_PV5))
def test_staria_uses_existing_canfd_ev_signals_and_safety(staria_params, candidate):
  make, _, _ = staria_params
  cp = make(candidate)
  state = carstate.CarState(cp)
  assert state.accelerator_msg_canfd == state.gear_msg_canfd == "ACCELERATOR"
  assert state.use_accelerator
  assert hyundaicanfd.CanBus(cp).ECAN == 0
  assert cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.EV_GAS
  assert not cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.HYBRID_GAS
  assert cp.flags & HyundaiFlags.ANGLE_CONTROL
  assert cp.steerControlType == CarParams.SteerControlType.angle


@pytest.mark.parametrize(("gear", "expected"), ((0, "park"), (5, "drive"), (6, "neutral"), (7, "reverse")))
@pytest.mark.parametrize("pedal", (0, 128, 255))
def test_ev_pedal_and_gear_decode(staria_params, gear, expected, pedal):
  make, _, _ = staria_params
  cp = make()
  state = carstate.CarState(cp)
  parsers = state.get_can_parsers(cp)
  parser = parsers[Bus.pt]
  # Match the existing runtime's EV accelerator registration policy.
  parser._add_message("ACCELERATOR", ignore_counter=True)
  packer = CANPacker("hyundai_canfd_generated")
  frame = packer.make_can_msg("ACCELERATOR", 0, {"ACCELERATOR_PEDAL": pedal, "GEAR": gear})
  parser.update([1_000_000_000, [frame]])
  state.accelerator = parser.vl["ACCELERATOR"]
  result = state.update_canfd(parsers)
  assert result.gas == pytest.approx(pedal / 255)
  assert result.gasPressed == (pedal > 0)
  assert str(result.gearShifter) == expected


def test_original_staria_and_torque_fallback_unchanged(staria_params):
  make, fingerprint, _ = staria_params
  original = make(CAR.HYUNDAI_STARIA_4TH_GEN)
  assert original.mass == pytest.approx(2205 + STD_CARGO_KG)
  assert not original.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.EV_GAS
  original_state = carstate.CarState(original)
  assert original_state.accelerator_msg_canfd == "ACCELERATOR_BRAKE_ALT"
  assert original_state.gear_msg_canfd == "GEAR_SHIFTER"
  fingerprint[2].pop(0xCB)
  ev = make()
  assert ev.steerControlType == CarParams.SteerControlType.torque
  assert interfaces_module.get_torque_params()[CAR.HYUNDAI_STARIA_EV] == \
         interfaces_module.get_torque_params()[CAR.HYUNDAI_STARIA_4TH_GEN]
