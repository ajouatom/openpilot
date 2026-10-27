from unittest.mock import patch

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, TeslaFlags, TeslaSafetyFlags

N = 10_000_000_000


def observe(cs, frames, timestamp=N):
  for _, data, source in frames:
    cs.observe_turn_stalk_frame(data, source, timestamp)


def stalk_frame(position, bus=1, size=4, counter=0):
  data = bytearray((0, counter, position, 0))
  crc = 0
  for value in (0, position, 0, 0):
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x2F) & 255 if crc & 128 else (crc << 1) & 255
  magic = (0x9B, 0xE8, 0x2A, 0xD3, 0xD3, 0x83, 0x4C, 0x5E, 0x3F, 0x5E, 0xE2, 0x28, 0x3A, 0x13, 0xAF, 0xCE)
  data[0] = crc ^ magic[counter]
  return (0x249, bytes(data[:size]), bus)


def params(candidate=CAR.TESLA_MODEL_Y, enabled=True, vehicle_bus=True, longitudinal=False):
  fingerprint = gen_empty_fingerprint()
  if vehicle_bus:
    fingerprint[1][0x3DF] = 8
  with patch("opendbc.car.tesla.interface.Params") as store:
    store.return_value.get_bool.return_value = enabled
    if candidate == CAR.TESLA_MODEL_X:
      # Model X has no generic torque entry; test its Tesla capability gate directly.
      ret = structs.CarParams()
      ret.carFingerprint = candidate
      return CarInterface._get_params(ret, candidate, fingerprint, [], longitudinal, False, False)
    return CarInterface.get_params(candidate, fingerprint, [], longitudinal, False, False)


@pytest.mark.parametrize("candidate", [CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_X])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("vehicle_bus", [False, True])
@pytest.mark.parametrize("longitudinal", [False, True])
def test_configuration_is_independent_and_only_enables_supported_platforms(candidate, enabled, vehicle_bus, longitudinal):
  cp = params(candidate, enabled, vehicle_bus, longitudinal)
  expected = candidate != CAR.TESLA_MODEL_X and enabled and vehicle_bus
  assert bool(cp.flags & TeslaFlags.AUTO_TURN_SIGNALS) == expected
  assert bool(cp.safetyConfigs[0].safetyParam & TeslaSafetyFlags.AUTO_TURN_SIGNALS) == expected
  assert bool(cp.flags & TeslaFlags.HAS_VEHICLE_BUS) == vehicle_bus
  assert bool(cp.flags & TeslaFlags.AUTO_SPEED_LIMIT) == (vehicle_bus and longitudinal)
  assert bool(cp.safetyConfigs[0].safetyParam & TeslaSafetyFlags.LONG_CONTROL) == longitudinal
  assert cp.dashcamOnly == (candidate == CAR.TESLA_MODEL_X)


@pytest.mark.parametrize("bus", [0, 2])
def test_extension_marker_on_another_bus_does_not_enable_vehicle_outputs(bus):
  fingerprint = gen_empty_fingerprint()
  fingerprint[bus][0x3DF] = 8
  with patch("opendbc.car.tesla.interface.Params") as store:
    store.return_value.get_bool.return_value = True
    cp = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], True, False, False)
  assert not cp.flags & (TeslaFlags.HAS_VEHICLE_BUS | TeslaFlags.AUTO_TURN_SIGNALS | TeslaFlags.AUTO_SPEED_LIMIT)
  assert not cp.safetyConfigs[0].safetyParam & TeslaSafetyFlags.AUTO_TURN_SIGNALS


@pytest.mark.parametrize("bus", [0, 1])
def test_real_stalk_frames_count_presses_without_counting_half_to_full_twice(bus):
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    for raw, direction, counts in ((6, 1, (1, 0)), (8, 1, (1, 0)), (0, 0, (1, 0)),
                                   (6, 1, (2, 0)), (2, 2, (2, 1)), (4, 2, (2, 1))):
      frame = stalk_frame(raw, bus)
      observe(cs, [frame])
      ret = cs.update(parsers)
      assert ret.turnSignalDriverValid
      assert ret.turnSignalDriverDirection == direction
      assert (ret.leftBlinkerStalkCount, ret.rightBlinkerStalkCount) == counts
    observe(cs, [stalk_frame(9, bus)])
    assert not cs.update(parsers).turnSignalDriverValid
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N + 1_000_000_001):
    assert not cs.update(parsers).turnSignalDriverValid


def test_optional_stalk_does_not_change_disabled_parser_or_fake_lamp_feedback():
  cp = params(enabled=False)
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  assert 0x249 not in parsers[Bus.party].addresses
  assert 0x249 not in parsers[Bus.adas].addresses
  ret = cs.update(parsers)
  assert not ret.turnSignalDriverValid
  assert ret.autoTurnSignalDirection == 0
  parsers[Bus.party].vl["UI_warning"]["leftBlinkerBlinking"] = 2
  assert cs.update(parsers).leftBlinker


@pytest.mark.parametrize("bus", [0, 1])
def test_press_and_release_in_one_batch_is_not_lost(bus):
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  frames = [stalk_frame(position, bus) for position in (6, 8, 0)]
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    observe(cs, frames)
    ret = cs.update(parsers)
  assert ret.turnSignalDriverValid and ret.turnSignalDriverDirection == 0
  assert ret.leftBlinkerStalkCount == 1 and ret.rightBlinkerStalkCount == 0


@pytest.mark.parametrize("party_position", [0, 1, 9])
def test_conflicting_fresh_sources_do_not_authorize_automatic_output(party_position):
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  for bus, position in ((0, party_position), (1, 6)):
    frame = stalk_frame(position, bus)
    observe(cs, [frame])
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    assert not cs.update(parsers).turnSignalDriverValid


def test_vehicle_stalk_does_not_fall_back_to_idle_party_when_stale():
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  observe(cs, [stalk_frame(0, 1)])
  later = N + 1_000_000_001
  observe(cs, [stalk_frame(0, 0)], later)
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=later):
    assert not cs.update(parsers).turnSignalDriverValid


def test_mirrored_stalk_and_invalid_gap_do_not_double_count_a_press():
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    for party, vehicle, valid in ((6, 6, True), (8, 6, True), (0, 6, False), (6, 6, True)):
      for bus, position in enumerate((party, vehicle)):
        frame = stalk_frame(position, bus)
        observe(cs, [frame])
      ret = cs.update(parsers)
      assert ret.turnSignalDriverValid == valid
      assert ret.leftBlinkerStalkCount == 1


def test_party_fast_press_is_not_ignored_when_vehicle_bus_is_idle():
  cp = params()
  cs = CarState(cp)
  parsers = CarState.get_can_parsers(cp)
  observe(cs, [stalk_frame(0, 1)])
  frames = [stalk_frame(p, 0) for p in (6, 0)]
  observe(cs, frames)
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    assert not cs.update(parsers).turnSignalDriverValid


@pytest.mark.parametrize("bus", [0, 1])
@pytest.mark.parametrize("position", range(16))
def test_every_stalk_code_uses_the_xpilot_dbc_layout(bus, position):
  ci = CarInterface(params())
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = ci.update([(N, [stalk_frame(position, bus)])])
  valid = position < 9
  expected = 1 if 5 <= position <= 8 else 2 if 1 <= position <= 4 else 0
  assert ret.turnSignalDriverValid == valid
  assert ret.turnSignalDriverDirection == (expected if valid else 0)


@pytest.mark.parametrize("bus", [0, 1])
@pytest.mark.parametrize("position", range(8))
def test_three_byte_stalk_protocol_is_rejected(bus, position):
  ci = CarInterface(params())
  frame = (0x249, bytes((0, 0, position)), bus)
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = ci.update([(N, [frame])])
  assert not ret.turnSignalDriverValid
  assert ret.turnSignalDriverDirection == 0
  assert ret.leftBlinkerStalkCount == ret.rightBlinkerStalkCount == 0


@pytest.mark.parametrize("counter", range(16))
@pytest.mark.parametrize("position", [0, 2, 4, 6, 8, 9])
def test_new_layout_crc_and_sna_are_checked(counter, position):
  from opendbc.car.tesla.carstate import decode_tesla_stalk

  _, data, _ = stalk_frame(position, counter=counter)
  expected = 1 if 5 <= position <= 8 else 2 if 1 <= position <= 4 else 0
  assert decode_tesla_stalk(data) == (expected if position < 9 else None)
  for index in (0, 2, 3):
    damaged = bytearray(data)
    damaged[index] ^= 0x80
    assert decode_tesla_stalk(bytes(damaged)) is None


@pytest.mark.parametrize("size", [0, 1, 2, 3, 5, 8])
def test_unknown_dlc_cannot_fall_back_to_party_idle(size):
  cs = CarState(params())
  parsers = CarState.get_can_parsers(cs.CP)
  observe(cs, [stalk_frame(0, 0), (0x249, bytes(size), 1)])
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    assert not cs.update(parsers).turnSignalDriverValid


@pytest.mark.parametrize("left", [False, True])
def test_different_press_depths_compare_normalized_direction(left):
  cs = CarState(params())
  parsers = CarState.get_can_parsers(cs.CP)
  observe(cs, [stalk_frame(8 if left else 4, 0), stalk_frame(6 if left else 2, 1)])
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = cs.update(parsers)
  assert ret.turnSignalDriverValid
  assert ret.turnSignalDriverDirection == (1 if left else 2)


def test_opposite_fast_presses_from_two_sources_are_ambiguous_even_after_release():
  cs = CarState(params())
  parsers = CarState.get_can_parsers(cs.CP)
  observe(cs, [stalk_frame(6, 0), stalk_frame(0, 0), stalk_frame(2, 1), stalk_frame(0, 1)])
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = cs.update(parsers)
  assert not ret.turnSignalDriverValid
  assert ret.leftBlinkerStalkCount == ret.rightBlinkerStalkCount == 0


@pytest.mark.parametrize("bus", [0, 1])
@pytest.mark.parametrize("positions,direction", [([1, 2, 3, 4, 0], 2), ([5, 6, 7, 8, 0], 1)])
def test_new_layout_half_full_and_release_are_one_press(bus, positions, direction):
  ci = CarInterface(params())
  frames = [stalk_frame(position, bus, counter=i) for i, position in enumerate(positions)]
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = ci.update([(N, frames)])
  assert ret.turnSignalDriverValid and ret.turnSignalDriverDirection == 0
  assert (ret.leftBlinkerStalkCount, ret.rightBlinkerStalkCount) == ((1, 0) if direction == 1 else (0, 1))


@pytest.mark.parametrize("source", [2, 0x80, 0x81, 0xC1])
def test_transmitted_or_unrelated_stalk_frames_never_become_driver_input(source):
  ci = CarInterface(params())
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = ci.update([(N, [stalk_frame(6, source)])])
  assert not ret.turnSignalDriverValid
  assert ret.leftBlinkerStalkCount == ret.rightBlinkerStalkCount == 0


@pytest.mark.parametrize("bus", [0, 1])
@pytest.mark.parametrize("age", [1_000_000_001, -1])
def test_old_or_future_press_in_receive_batch_is_not_revived_by_fresh_idle(bus, age):
  ci = CarInterface(params())
  packets = [(N - age, [stalk_frame(6, bus)]), (N, [stalk_frame(0, bus)])]
  with patch("opendbc.car.tesla.carstate.time.clock_gettime_ns", return_value=N):
    ret = ci.update(packets)
  assert ret.turnSignalDriverValid and ret.turnSignalDriverDirection == 0
  assert ret.leftBlinkerStalkCount == ret.rightBlinkerStalkCount == 0
