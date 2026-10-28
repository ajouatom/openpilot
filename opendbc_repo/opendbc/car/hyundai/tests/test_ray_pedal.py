from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.carcontroller import get_pedal_interceptor_target
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiSafetyFlags
from opendbc.car.structs import CarControl
from openpilot.selfdrive.controls.lib.longcontrol import long_control_state_trans
from types import SimpleNamespace


def ray_fingerprint(sensor_length=6):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x201] = sensor_length
  fingerprint[0][0x391] = 8
  fingerprint[2][0x485] = 8
  return fingerprint


def test_ray_pedal_requires_the_vehicle_markers_and_sets_only_ray_params():
  CP = CarInterface.get_params(CAR.KIA_RAY_EV, ray_fingerprint(), [], False, False, False)
  assert CP.enableGasInterceptorDEPRECATED
  assert CP.openpilotLongitudinalControl
  assert not CP.pcmCruise
  assert CP.autoResumeSng is False
  assert CP.minEnableSpeed == -1.0
  assert CP.safetyConfigs[-1].safetyParam == 0x9405

  fingerprint = ray_fingerprint(sensor_length=8)
  CP = CarInterface.get_params(CAR.KIA_RAY_EV, fingerprint, [], False, False, False)
  assert not CP.enableGasInterceptorDEPRECATED
  assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAN_REFRESH_MSGS)


def test_ray_pedal_parser_is_added_only_for_enabled_pedal():
  CP = CarInterface.get_params(CAR.KIA_RAY_EV, ray_fingerprint(), [], False, False, False)
  parsers = CarState(CP).get_can_parsers(CP)
  assert Bus.party in parsers
  assert parsers[Bus.party].dbc_name == "hyundai_kia_ray_pedal"

  CP = CarInterface.get_params(CAR.KIA_RAY_EV, ray_fingerprint(sensor_length=8), [], False, False, False)
  assert Bus.party not in CarState(CP).get_can_parsers(CP)


def test_ray_pedal_sensor_frames_validate_with_route_crc():
  parser = CANParser("hyundai_kia_ray_pedal", [("GAS_SENSOR", 50)], 0)
  samples = [bytes.fromhex(s) for s in (
    "01f403d55de8", "01f603d55ef1", "01f403d55f51", "01f603d3503f",
    "01f903d551ab", "01f903d552a4", "01f703d55370",
  )]
  for idx, dat in enumerate(samples):
    parser.update([(1_000_000_000 + idx * 20_000_000, [(0x201, dat, 0)])])
    assert parser.can_valid
    assert parser.vl["GAS_SENSOR"]["STATE"] == 5


def test_ray_pedal_does_not_hold_long_control_stopped_on_cruise_standstill():
  CP = CarInterface.get_params(CAR.KIA_RAY_EV, ray_fingerprint(), [], False, False, False)
  state = CarControl.Actuators.LongControlState
  radar_state = SimpleNamespace()
  result = long_control_state_trans(CP, True, state.off, 0.0, False, False, True, 0.0, -0.5, radar_state)
  assert result == state.pid

  CP.enableGasInterceptorDEPRECATED = False
  result = long_control_state_trans(CP, True, state.off, 0.0, False, False, True, 0.0, -0.5, radar_state)
  assert result == state.stopping


def test_pedal_interceptor_dispatch_does_not_apply_ray_tune_to_other_cars():
  ray_target = get_pedal_interceptor_target(CAR.KIA_RAY_EV, 0.0, 10.0, 20.0)
  other_target = get_pedal_interceptor_target(CAR.HYUNDAI_ELANTRA, 0.0, 10.0, 20.0)
  assert ray_target > 0.0
  assert other_target == 0.0
