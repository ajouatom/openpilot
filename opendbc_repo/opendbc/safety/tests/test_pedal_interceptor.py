import pytest

from opendbc.can import CANPacker
from opendbc.car.pedal_interceptor import PEDAL_INTERCEPTOR_DBC, create_pedal_interceptor_command
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum


PEDAL_INTERCEPTOR_SAFETY_PARAM = 0x6005
REPLACE_CRUISE_SAFETY_PARAM = 0xE005


@pytest.fixture
def safety():
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(CarParams.SafetyModel.hyundai, PEDAL_INTERCEPTOR_SAFETY_PARAM) == 0
  lib.init_tests()
  return lib


@pytest.fixture
def replacement_safety(safety):
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundai, REPLACE_CRUISE_SAFETY_PARAM) == 0
  safety.init_tests()
  return safety


def packet(message):
  address, data, bus = message
  return libsafety_py.make_CANPacket(address, bus, data)


def command(amount=0.1, counter=0):
  return create_pedal_interceptor_command(CANPacker(PEDAL_INTERCEPTOR_DBC), amount, counter)


def sensor(state=0, track1=264, track2=497, counter=1):
  return CANPacker(PEDAL_INTERCEPTOR_DBC).make_can_msg("GAS_SENSOR", 0, {
    "INTERCEPTOR_GAS": track1 * 0.672 - 177.408, "INTERCEPTOR_GAS2": track2 * 0.332 - 165.004,
    "STATE": state, "COUNTER_PEDAL": counter,
  })


def ready(safety):
  assert safety.safety_rx_hook(packet(sensor()))
  assert safety.safety_rx_hook(packet((0x592, bytes(8), 0)))
  safety.set_controls_allowed(True)


@pytest.mark.parametrize("amount", [0.00101, 0.08, 0.20, 0.55, 0.65, 0.70])
def test_expanded_commands_remain_inside_ray_profile(safety, amount):
  ready(safety)
  assert safety.safety_tx_hook(packet(command(amount)))


def expanded_raw_command(track1, track2):
  return CANPacker(PEDAL_INTERCEPTOR_DBC).make_can_msg("GAS_COMMAND", 0, {
    "GAS_COMMAND": track1 * 0.336 - 88.704, "GAS_COMMAND2": track2 * 0.166 - 82.502,
    "ENABLE": 1, "COUNTER_PEDAL": 0,
  })


@pytest.mark.parametrize("track1,track2,allowed", [(264, 497, True), (795, 1572, True),
                                                  (263, 497, False), (264, 496, False),
                                                  (796, 1572, False), (795, 1573, False),
                                                  (600, 1210, False)])
def test_ray_expanded_limits_and_original_track_correlation(safety, track1, track2, allowed):
  ready(safety)
  assert safety.safety_tx_hook(packet(expanded_raw_command(track1, track2))) == allowed


def test_above_target_cap_is_still_rejected(safety):
  ready(safety)
  assert not safety.safety_tx_hook(packet(command(0.701)))


@pytest.mark.parametrize("bits", [0x04, 0x08, 0x0C])
def test_oem_cruise_ready_or_engaged_blocks_pedal_without_canceling_oem(safety, bits):
  ready(safety)
  assert safety.safety_tx_hook(packet(command()))
  data = bytes([0, 0, 0, 0, bits, 0, 0, 0])
  assert safety.safety_rx_hook(packet((0x592, data, 0)))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))
  assert safety.safety_rx_hook(packet((0x592, bytes(8), 0)))
  assert safety.safety_tx_hook(packet(command()))


def test_missing_and_stale_cruise_state_blocks_pedal(safety):
  assert safety.safety_rx_hook(packet(sensor()))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  # A frame on another bus must not establish permission.
  assert safety.safety_rx_hook(packet((0x592, bytes(8), 2)))
  assert not safety.safety_tx_hook(packet(command()))
  ready(safety)
  safety.set_timer(1000001)
  assert safety.safety_rx_hook(packet(sensor(counter=2)))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


def test_missing_sensor_and_disengagement_allow_only_disabled_heartbeat(safety):
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))
  ready(safety)
  assert safety.safety_tx_hook(packet(command()))
  safety.set_controls_allowed(False)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


@pytest.mark.parametrize("state", range(1, 6))
def test_fault_states_block_positive_command(safety, state):
  ready(safety)
  assert safety.safety_rx_hook(packet(sensor(state=state, counter=2)))
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


def test_physical_override_and_native_commanded_gas_are_distinct(safety):
  ready(safety)
  native_gas = (0x371, bytes.fromhex("004e008000ae0700"), 0)
  assert safety.safety_rx_hook(packet(native_gas))
  assert not safety.get_gas_pressed_prev()
  assert safety.safety_tx_hook(packet(command()))
  assert safety.safety_rx_hook(packet(sensor(track1=310, track2=593, counter=2)))
  assert safety.get_gas_pressed_prev()
  assert not safety.safety_tx_hook(packet(command()))


@pytest.mark.parametrize("moving", [False, True])
def test_hardware_brake_blocks_positive_even_if_controls_reenabled(safety, moving):
  ready(safety)
  packer = CANPacker("hyundai_kia_generic")
  assert safety.safety_rx_hook(packet(checksum(packer.make_can_msg("WHL_SPD11", 0, {
    "WHL_SPD_FL": 10 if moving else 0, "WHL_SPD_RR": 10 if moving else 0,
  }))))
  assert safety.get_vehicle_moving() == moving
  assert safety.safety_rx_hook(packet(checksum(packer.make_can_msg("TCS13", 0, {"DriverOverride": 2}))))
  assert safety.get_brake_pressed_prev()
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


def test_rx_crc_and_stale_state_cannot_be_bypassed_by_reenable(safety):
  ready(safety)
  address, data, bus = sensor(counter=2)
  bad = data[:-1] + bytes([data[-1] ^ 1])
  assert not safety.safety_rx_hook(packet((address, bad, bus)))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_rx_hook(packet(sensor(counter=3)))
  safety.set_controls_allowed(True)
  assert safety.safety_tx_hook(packet(command()))
  safety.set_timer(200001)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


def test_rejected_scc_tx_does_not_authorize_pedal(safety):
  ready(safety)
  safety.set_controls_allowed(False)
  # SCC12 ACC mode plus a nonzero acceleration exercises both legacy side effects.
  scc = CANPacker("hyundai_kia_generic").make_can_msg("SCC12", 0, {"ACCMode": 1, "aReqRaw": 1.0, "aReqValue": 1.0})
  assert not safety.safety_tx_hook(packet(scc))
  assert not safety.get_controls_allowed()
  assert not safety.safety_tx_hook(packet(command()))


@pytest.mark.parametrize("bus,length", [(1, 6), (2, 6), (0, 5), (0, 8)])
def test_command_bus_and_length_isolation(safety, bus, length):
  ready(safety)
  address, data, _ = command()
  assert not safety.safety_tx_hook(packet((address, data[:length].ljust(length, b"\0"), bus)))


def test_relay_fault_cannot_be_overridden(safety):
  ready(safety)
  safety.set_relay_malfunction(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert not safety.safety_tx_hook(packet(command(0.0)))


@pytest.mark.parametrize("param", [0, 1, 4, 5, 8192, 8192 | 4, 8192 | 16384 | 4, 0x600D,
                                    0x8000, 0xA005, 0xE004, 0xE00D])
def test_pedal_is_not_implicitly_added_to_existing_or_incomplete_profiles(param):
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
  safety.init_tests()
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))


def test_malformed_checksum_and_disabled_nonzero_tracks(safety):
  ready(safety)
  address, data, bus = command()
  assert not safety.safety_tx_hook(packet((address, data[:-1] + bytes([data[-1] ^ 1]), bus)))
  disabled_nonzero = CANPacker(PEDAL_INTERCEPTOR_DBC).make_can_msg("GAS_COMMAND", 0, {
    "ENABLE": 0, "GAS_COMMAND": 0, "GAS_COMMAND2": 0, "COUNTER_PEDAL": 1,
  })
  assert disabled_nonzero[1][:4] != bytes(4)
  assert not safety.safety_tx_hook(packet(disabled_nonzero))


def test_stalled_sensor_counter_blocks_positive_until_a_valid_frame(safety):
  ready(safety)
  for _ in range(5):
    safety.safety_rx_hook(packet(sensor(counter=1)))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))
  assert safety.safety_rx_hook(packet(sensor(counter=2)))
  safety.set_controls_allowed(True)
  assert safety.safety_tx_hook(packet(command()))


@pytest.mark.parametrize("model", [CarParams.SafetyModel.hyundai, CarParams.SafetyModel.hyundaiLegacy,
                                    CarParams.SafetyModel.hyundaiCanfd])
@pytest.mark.parametrize("param", [0, 1, 4, 5, 8, 12, 16, 24, 189])
def test_switching_from_pedal_to_ordinary_profiles_removes_pedal_permission(safety, model, param):
  ready(safety)
  assert safety.safety_tx_hook(packet(command()))
  assert safety.set_safety_hooks(model, param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert not safety.safety_tx_hook(packet(command(0.0)))


@pytest.mark.parametrize("button", [1, 2])  # RES, SET
def test_replacement_requires_driver_and_waits_for_oem_release(replacement_safety, button):
  safety = replacement_safety
  cancel = packet((0x4F1, bytes([4, 0, 0, 0]), 0))
  positive = packet(command(0.70))
  heartbeat = packet(command(0.0))
  label_ready = packet((0x592, bytes([0, 0, 0, 0, 4, 0, 0, 0]), 0))
  label_engaged = packet((0x592, bytes([0, 0, 0, 0, 8, 0, 0, 0]), 0))
  assert safety.safety_rx_hook(packet(sensor()))
  assert safety.safety_rx_hook(label_ready)
  assert not safety.safety_tx_hook(positive)
  assert not safety.safety_tx_hook(cancel)
  assert safety.safety_rx_hook(packet((0x4F1, bytes([button, 0, 0, 0]), 0)))
  assert safety.safety_rx_hook(label_engaged)
  assert safety.safety_rx_hook(packet((0x4F1, bytes([0, 0, 0, 16]), 0)))
  assert safety.get_controls_allowed()
  assert safety.safety_tx_hook(cancel)
  assert safety.get_controls_allowed()  # Transmitting CANCEL must not change driver authorization.
  assert not safety.safety_tx_hook(positive)
  assert safety.safety_tx_hook(heartbeat)
  assert safety.safety_rx_hook(label_ready)
  assert safety.safety_tx_hook(positive)
  assert not safety.safety_tx_hook(cancel)
  assert not safety.safety_tx_hook(packet(command(0.701)))

  assert safety.safety_rx_hook(packet((0x4F1, bytes([8, 0, 0, 32]), 0)))  # MAIN press before LABEL11 changes
  assert not safety.get_controls_allowed()
  assert not safety.safety_tx_hook(positive)
  safety.set_controls_allowed(True)
  assert safety.safety_rx_hook(packet((0x4F1, bytes([4, 0, 0, 48]), 0)))  # Driver CANCEL
  assert not safety.get_controls_allowed()
  assert not safety.safety_tx_hook(positive)
  safety.set_controls_allowed(True)
  assert safety.safety_rx_hook(packet((0x592, bytes(8), 0)))  # MAIN off
  assert not safety.get_controls_allowed()
  assert not safety.safety_tx_hook(positive)
  assert safety.safety_rx_hook(label_ready)
  assert not safety.safety_tx_hook(positive)


def test_replacement_cancel_is_only_for_fresh_oem_engagement(replacement_safety):
  safety = replacement_safety
  label_engaged = bytes([0, 0, 0, 0, 0xC, 0, 0, 0])
  cancel = packet((0x4F1, bytes([4, 0, 0, 0]), 0))
  assert not safety.safety_tx_hook(cancel)
  assert safety.safety_rx_hook(packet((0x592, label_engaged, 2)))
  assert not safety.safety_tx_hook(cancel)
  assert safety.safety_rx_hook(packet((0x592, label_engaged, 0)))
  assert safety.safety_tx_hook(cancel)
  assert not safety.get_controls_allowed()
  for bad in [0, 1, 2, 3, 4 | 8, 4 | 16]:
    assert not safety.safety_tx_hook(packet((0x4F1, bytes([bad, 0, 0, 0]), 0)))
  assert not safety.safety_tx_hook(packet((0x4F1, bytes([4, 0, 0, 0]), 2)))
  assert not safety.safety_tx_hook(packet((0x4F1, bytes([4, 0, 0, 0, 0, 0]), 0)))
  safety.set_timer(1_000_001)
  assert safety.safety_rx_hook(packet(sensor()))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(cancel)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_tx_hook(packet(command(0.0)))


def test_replacement_does_not_arm_from_alternate_or_malformed_buttons(replacement_safety):
  safety = replacement_safety
  assert safety.safety_rx_hook(packet(sensor()))
  assert safety.safety_rx_hook(packet((0x592, bytes([0, 0, 0, 0, 4, 0, 0, 0]), 0)))
  for message in ((0x3EF, bytes(7) + b"\x20", 0), (0x3EF, bytes(8), 0),
                  (0x4F1, b"\x02" + bytes(5), 0), (0x4F1, bytes(6), 0)):
    assert safety.safety_rx_hook(packet(message))
  assert not safety.get_controls_allowed()
  assert not safety.safety_tx_hook(packet(command()))


def test_replacement_profile_reset_restores_mode1_oem_priority(replacement_safety):
  safety = replacement_safety
  assert safety.safety_rx_hook(packet((0x592, bytes([0, 0, 0, 0, 8, 0, 0, 0]), 0)))
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundai, PEDAL_INTERCEPTOR_SAFETY_PARAM) == 0
  safety.init_tests()
  assert safety.safety_rx_hook(packet(sensor()))
  assert safety.safety_rx_hook(packet((0x592, bytes([0, 0, 0, 0, 4, 0, 0, 0]), 0)))
  safety.set_controls_allowed(True)
  assert not safety.safety_tx_hook(packet(command()))
  assert safety.safety_rx_hook(packet((0x592, bytes(8), 0)))
  assert safety.safety_tx_hook(packet(command()))
