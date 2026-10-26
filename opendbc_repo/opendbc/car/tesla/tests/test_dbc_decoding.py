import pytest

from opendbc.can import CANDefine, CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.tests.test_auto_turn_signals import N, params, stalk_frame


HEALTHY = {
  "DI_speed": {"DI_vehicleSpeed": 20, "DI_uiSpeed": 20, "DI_uiSpeedUnits": 1},
  "DI_state": {"DI_digitalSpeed": 20, "DI_speedUnits": 1, "DI_cruiseState": 2},
  "DI_systemStatus": {"DI_accelPedalPos": 0, "DI_gear": 4},
  "ESP_wheelSpeeds": dict.fromkeys(("ESP_wheelSpeedFrL", "ESP_wheelSpeedFrR", "ESP_wheelSpeedReL", "ESP_wheelSpeedReR"), 20),
}


@pytest.fixture
def vehicle(monkeypatch):
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N)
  ci = CarInterface(params(enabled=False))
  packer = CANPacker("tesla_model3_party")
  ci.update([])
  ci.update([(N, [packer.make_can_msg(message, 0, fields) for message, fields in HEALTHY.items()])])
  return ci, packer


@pytest.mark.parametrize("units", [0, 1])
@pytest.mark.parametrize("speed", [0, 5, 90])
@pytest.mark.parametrize("reserved_bit32", [0, 1])
def test_display_units_use_bit33_without_speed_based_guessing(vehicle, units, speed, reserved_bit32):
  ci, packer = vehicle
  values = {"DI_vehicleSpeed": speed, "DI_uiSpeed": speed, "DI_uiSpeedUnits": units}
  address, payload, source = packer.make_can_msg("DI_speed", 0, values)
  data = bytearray(payload)
  data[4] = (data[4] & ~1) | reserved_bit32
  data[0] = (0x57 + 2 + sum(data[1:])) & 255
  result = ci.update([(N, [(address, bytes(data), source)])])
  assert (data[4] >> 1) & 1 == units
  assert result.vEgoCluster == pytest.approx(speed * (1 / 3.6 if units else .44704), abs=.1)
  assert not result.vehicleSensorsInvalid


@pytest.mark.parametrize("message,signal,value,field", [
  ("DI_speed", "DI_vehicleSpeed", 287.6, "vEgoRaw"),
  ("DI_speed", "DI_uiSpeed", 255, "vEgoCluster"),
  ("ESP_wheelSpeeds", "ESP_wheelSpeedFrL", 327.64, "wheelSpeeds.fl"),
  ("ESP_wheelSpeeds", "ESP_wheelSpeedFrR", 327.64, "wheelSpeeds.fr"),
  ("ESP_wheelSpeeds", "ESP_wheelSpeedReL", 327.64, "wheelSpeeds.rl"),
  ("ESP_wheelSpeeds", "ESP_wheelSpeedReR", 327.64, "wheelSpeeds.rr"),
  ("DI_systemStatus", "DI_accelPedalPos", 102, "gas"),
])
def test_reserved_measurements_hold_last_value_and_report_invalid_until_recovery(vehicle, monkeypatch, message, signal, value, field):
  ci, packer = vehicle
  errors = []
  monkeypatch.setattr("opendbc.car.tesla.carstate.carlog.error", errors.append)

  def read(state):
    result = state
    for name in field.split("."):
      result = getattr(result, name)
    return result

  previous = read(ci.CS.out)
  fields = HEALTHY[message] | {signal: value}
  result = ci.update([(N, [packer.make_can_msg(message, 0, fields)])])
  assert result.vehicleSensorsInvalid
  assert read(result) == pytest.approx(previous)
  if message == "DI_systemStatus":
    assert result.gasPressed
  for _ in range(5):
    assert ci.update([]).vehicleSensorsInvalid
  assert len(errors) == 1 and f"{message}.{signal}" in errors[0]
  result = ci.update([(N, [packer.make_can_msg(message, 0, HEALTHY[message])])])
  assert not result.vehicleSensorsInvalid


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), 9999])
@pytest.mark.parametrize("message,signal", [
  ("DI_speed", "DI_vehicleSpeed"), ("DI_speed", "DI_uiSpeed"),
  ("ESP_wheelSpeeds", "ESP_wheelSpeedFrL"), ("DI_state", "DI_digitalSpeed"),
])
def test_nonfinite_or_out_of_range_measurements_fail_closed(vehicle, value, message, signal):
  ci, _ = vehicle
  ci.can_parsers[Bus.party].vl[message][signal] = value
  result = ci.update([])
  assert result.vehicleSensorsInvalid


@pytest.mark.parametrize("truncated", [False, True])
def test_invalid_vehicle_speed_never_advances_speed_filter(vehicle, monkeypatch, truncated):
  ci, packer = vehicle
  calls = []
  monkeypatch.setattr(ci.CS, "update_speed_kf", lambda speed: calls.append(speed))
  frame = packer.make_can_msg("DI_speed", 0, HEALTHY["DI_speed"] | {"DI_vehicleSpeed": 287.6})
  if truncated:
    data = bytearray(frame[1][:5])
    data[0] = (0x57 + 2 + sum(data[1:])) & 255
    frame = (frame[0], bytes(data), frame[2])
  result = ci.update([(N, [frame])])
  assert result.vehicleSensorsInvalid
  assert not calls
  assert result.vEgoRaw == pytest.approx(20 / 3.6)


@pytest.mark.parametrize("units", [0, 1])
def test_fused_speed_limit_has_fixed_kph_units(vehicle, units):
  ci, packer = vehicle
  frames = [packer.make_can_msg("DI_state", 0, HEALTHY["DI_state"] | {"DI_speedUnits": units}),
            packer.make_can_msg("DAS_status", 2, {"DAS_fusedSpeedLimit": 60})]
  result = ci.update([(N, frames)])
  assert result.speedLimit == 60
  assert ci.CS.tesla_speed_limit_target == pytest.approx(60 / 3.6)
  assert ci.CS.tesla_speed_units == ("KPH" if units else "MPH")


@pytest.mark.parametrize("pressed,position,expected", [(1, .4, True), (0, .4, False), (0, 1.2, True)])
def test_digital_pedal_press_cannot_be_hidden_by_analog_noise_filter(vehicle, pressed, position, expected):
  ci, packer = vehicle
  frames = [packer.make_can_msg("DI_speed", 0, HEALTHY["DI_speed"] | {"DI_accelPedalPressed": pressed}),
            packer.make_can_msg("DI_systemStatus", 0, HEALTHY["DI_systemStatus"] | {"DI_accelPedalPos": position})]
  result = ci.update([(N, frames)])
  assert result.gasPressed == expected
  assert not result.vehicleSensorsInvalid


@pytest.mark.parametrize("age", [-1, 500_000_001])
def test_future_or_stale_digital_pedal_flag_is_not_replayed(vehicle, age):
  ci, packer = vehicle
  frame = packer.make_can_msg("DI_speed", 0, HEALTHY["DI_speed"] | {"DI_accelPedalPressed": 1})
  assert not ci.update([(N - age, [frame])]).gasPressed


@pytest.mark.parametrize("initial_position,expected_after_release", [(.4, False), (1.2, True)])
def test_digital_pedal_release_preserves_only_analog_hysteresis(vehicle, initial_position, expected_after_release):
  ci, packer = vehicle
  frames = [packer.make_can_msg("DI_speed", 0, HEALTHY["DI_speed"] | {"DI_accelPedalPressed": 1}),
            packer.make_can_msg("DI_systemStatus", 0, HEALTHY["DI_systemStatus"] | {"DI_accelPedalPos": initial_position})]
  assert ci.update([(N, frames)]).gasPressed
  frames = [packer.make_can_msg("DI_speed", 0, HEALTHY["DI_speed"]),
            packer.make_can_msg("DI_systemStatus", 0, HEALTHY["DI_systemStatus"] | {"DI_accelPedalPos": .8})]
  assert ci.update([(N, frames)]).gasPressed == expected_after_release
  released = packer.make_can_msg("DI_systemStatus", 0, HEALTHY["DI_systemStatus"])
  assert not ci.update([(N, [released])]).gasPressed


@pytest.mark.parametrize("position", range(10))
def test_vehicle_dbc_exposes_full_extended_stalk_code(position):
  parser = CANParser("tesla_model3_vehicle", [("SCCM_leftStalk", 50)], 1)
  parser.update([(N, [stalk_frame(position)])])
  assert parser.vl["SCCM_leftStalk"]["SCCM_turnIndicatorStalkStatus"] == position
  assert parser.dbc.name_to_msg["SCCM_leftStalk"].size == 4
  assert CANDefine("tesla_model3_vehicle").dv["SCCM_leftStalk"]["SCCM_turnIndicatorStalkStatus"][9] == "SNA"


def test_separate_hw4_tpms_parser_uses_vehicle_bus_without_false_bus_timeout(vehicle):
  ci, _ = vehicle
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = packer.make_can_msg("VCSEC_TPMSDisplay", 1, {"VCSEC_TPMSDisplayPressureFL": 2.6})
  result = ci.update([(N, [frame])])
  assert result.tpms.fl == pytest.approx(37.7)
  assert ci.can_parsers[Bus.adas].dbc_name == "tesla_model3_vehicle"
  assert ci.can_parsers[Bus.body].dbc_name == "tesla_modely_hw4_perception"
  assert not ci.can_parsers[Bus.body].bus_timeout
  ci.update([(N + 600_000_000, [])])
  assert not ci.can_parsers[Bus.body].bus_timeout
  assert ci.can_parsers[Bus.body].can_valid


@pytest.mark.parametrize("bus", [0, 2, 0x81, 0xC1])
def test_tpms_ignores_unrelated_sources_and_transmit_echoes(vehicle, bus):
  ci, _ = vehicle
  frame = CANPacker("tesla_modely_hw4_perception").make_can_msg("VCSEC_TPMSDisplay", bus, {"VCSEC_TPMSDisplayPressureFL": 2.6})
  assert ci.update([(N, [frame])]).tpms.fl == 0


@pytest.mark.parametrize("invalid", ["stale", "future", "short", "long", "sna"])
def test_tpms_stale_or_invalid_data_is_not_displayed(vehicle, monkeypatch, invalid):
  ci, _ = vehicle
  frame = CANPacker("tesla_modely_hw4_perception").make_can_msg("VCSEC_TPMSDisplay", 1, {"VCSEC_TPMSDisplayPressureFL": 2.6})
  assert ci.update([(N, [frame])]).tpms.fl == pytest.approx(37.7)
  if invalid == "stale":
    monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N + 10_000_000_001)
    packets = []
  elif invalid == "future":
    packets = [(N + 1, [frame])]
  else:
    data = frame[1][:6] if invalid == "short" else frame[1] + b"\0" if invalid == "long" else b"\xff" + frame[1][1:]
    packets = [(N, [(frame[0], data, frame[2])])]
  assert ci.update(packets).tpms.fl == 0
