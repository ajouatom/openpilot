import pytest

from panda.tests.libpanda.libpanda_py import libpanda, make_CANPacket


def tesla_checksum(addr, data, checksum_byte):
  # (addr_lo + addr_hi + sum of all bytes except the checksum byte) mod 256,
  # matching the party bus checksum in tesla_model3_party.dbc.
  checksum = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i, b in enumerate(data):
    if i != checksum_byte:
      checksum += b
  return checksum & 0xFF


def send(address, data, bus=0):
  libpanda.ignition_can_hook(make_CANPacket(address, bus, data))


def power(state, counters=(0, 1), bus=0):
  for counter in counters:
    data = bytearray(8)
    data[0] = state << 5
    data[6] = counter << 4
    data[7] = tesla_checksum(0x221, data, 7)
    send(0x221, data, bus)


def gear(value, counters=(0, 1), bus=0):
  for counter in counters:
    data = bytearray(8)
    data[1] = counter
    data[2] = value << 5
    data[0] = tesla_checksum(0x118, data, 0)
    send(0x118, data, bus)


def cabin(*, latched=False, door_open=False):
  for counter in (0, 1):
    data = bytearray(7)
    data[1] = counter | (int(latched) << 5)
    data[3] = int(door_open) << 4
    data[0] = tesla_checksum(0x311, data, 0)
    send(0x311, data)


@pytest.fixture(autouse=True)
def reset_ignition():
  # Valid frame pairs reset each rolling-counter history and cabin state.
  power(3)
  cabin()
  gear(1)
  power(0)
  libpanda.ignition_can = False
  libpanda.ignition_can_cnt = 0
  libpanda.wake_on_can = False
  libpanda.wake_on_can_cnt = 0
  libpanda.tesla_power_on_can = False
  libpanda.tesla_power_on_can_cnt = 0


@pytest.mark.parametrize("state", [1, 2, 3])
def test_tesla_can_wake_without_going_onroad(state):
  power(state)
  assert libpanda.wake_on_can
  assert not libpanda.ignition_can


@pytest.mark.parametrize("value", [2, 3, 4])
def test_tesla_driving_gears_go_onroad(value):
  power(3)
  gear(value)
  assert libpanda.ignition_can


@pytest.mark.parametrize("latched,door_open", [(False, False), (True, True), (False, True)])
def test_tesla_park_and_exit_go_offroad_but_stay_awake(latched, door_open):
  power(3)
  gear(4)
  cabin(latched=latched, door_open=door_open)
  gear(1)
  assert not libpanda.ignition_can
  assert libpanda.wake_on_can


def test_park_with_latched_belt_and_closed_doors_preserves_prior_ignition():
  power(3)
  cabin(latched=True)
  gear(1)
  assert not libpanda.ignition_can
  gear(4)
  gear(1)
  assert libpanda.ignition_can


@pytest.mark.parametrize("prior_ignition", [False, True])
@pytest.mark.parametrize("value", [1, 4])
def test_shared_118_address_does_not_change_other_brands_ignition(prior_ignition, value):
  # Subaru Steering_Torque_2 uses the same address, length, and counter bits.
  # Its torque bits must not be mistaken for Tesla gear without power evidence.
  libpanda.ignition_can = prior_ignition
  libpanda.ignition_can_cnt = 2
  gear(value)
  assert libpanda.ignition_can == prior_ignition
  assert libpanda.ignition_can_cnt == 2


def test_stale_tesla_power_evidence_cannot_authorize_gear_updates():
  power(3)
  libpanda.tesla_power_on_can_cnt = 3
  gear(4)
  assert not libpanda.ignition_can


def test_wake_and_ignition_expire_when_can_stops():
  power(3)
  gear(4)
  for _ in range(3):
    libpanda.ignition_can_tick()
  assert libpanda.wake_on_can
  assert libpanda.ignition_can
  libpanda.ignition_can_tick()
  assert not libpanda.wake_on_can
  assert not libpanda.ignition_can
  gear(4)
  assert not libpanda.ignition_can
  power(3)
  gear(4)
  assert libpanda.ignition_can


def test_accessory_traffic_keeps_wake_alive_without_refreshing_ignition():
  power(3)
  gear(4)
  for _ in range(4):
    power(2)
    libpanda.ignition_can_tick()
  assert libpanda.wake_on_can
  assert not libpanda.ignition_can


@pytest.mark.parametrize("invalid_gear", [0, 5, 6, 7])
def test_invalid_gear_does_not_keep_ignition_alive(invalid_gear):
  power(3)
  gear(4)
  for _ in range(4):
    power(3)
    gear(invalid_gear)
    libpanda.ignition_can_tick()
  assert not libpanda.ignition_can


def test_single_valid_frame_does_not_wake():
  # The rolling counter requires two consecutive valid frames.
  power(3, counters=(14,))
  assert not libpanda.wake_on_can
  power(3, counters=(15,))
  assert libpanda.wake_on_can


def test_bad_checksum_does_not_wake_and_breaks_sequence():
  # A corrupt frame must neither wake on its own nor refresh the counter.
  power(3, counters=(14,))
  assert not libpanda.wake_on_can
  data = bytearray(8)
  data[0] = 3 << 5
  data[6] = 15 << 4
  data[7] = tesla_checksum(0x221, data, 7) ^ 0x01
  send(0x221, data)
  assert not libpanda.wake_on_can
  # The corrupt frame reset the counter sequence, so one more valid frame
  # (counter 0, which would follow 15) still cannot wake.
  power(3, counters=(0,))
  assert not libpanda.wake_on_can
  power(3, counters=(1,))
  assert libpanda.wake_on_can


@pytest.mark.parametrize("corrupt_byte", list(range(8)))
def test_any_corrupt_byte_does_not_wake(corrupt_byte):
  power(3, counters=(14,))
  assert not libpanda.wake_on_can
  data = bytearray(8)
  data[0] = 3 << 5
  data[6] = 15 << 4
  data[7] = tesla_checksum(0x221, data, 7)
  data[corrupt_byte] ^= 0x01
  if corrupt_byte == 7:
    data[7] = tesla_checksum(0x221, data, 7) ^ 0x01
  send(0x221, data)
  assert not libpanda.wake_on_can


def test_extended_frame_does_not_wake():
  power(3, counters=(14,))
  assert not libpanda.wake_on_can
  data = bytearray(8)
  data[0] = 3 << 5
  data[6] = 15 << 4
  data[7] = tesla_checksum(0x221, data, 7)
  packet = make_CANPacket(0x221, 0, data)
  packet[0].extended = 1
  libpanda.ignition_can_hook(packet)
  assert not libpanda.wake_on_can
  # Extended frames also break the sequence.
  power(3, counters=(0,))
  assert not libpanda.wake_on_can
  power(3, counters=(1,))
  assert libpanda.wake_on_can


def test_garbage_221_traffic_from_other_ecu_does_not_wake():
  # Rivian 0x221 carries random data: consecutive counters with bad checksums
  # must never be interpreted as a Tesla power state.
  for counter in range(4):
    data = bytearray(8)
    for i in range(8):
      data[i] = (0xA5 * (counter + 1) + i) & 0xFF
    data[6] = (data[6] & 0x0F) | (counter << 4)
    send(0x221, data)
  assert not libpanda.wake_on_can
  assert not libpanda.ignition_can


def test_bad_checksum_gear_does_not_set_ignition():
  power(3)
  data = bytearray(8)
  data[1] = 0
  data[2] = 4 << 5  # D gear
  data[0] = tesla_checksum(0x118, data, 0) ^ 0x01
  send(0x118, data)
  assert not libpanda.ignition_can
  data[1] = 1
  data[0] = tesla_checksum(0x118, data, 0) ^ 0x01
  send(0x118, data)
  assert not libpanda.ignition_can
  gear(4)
  assert libpanda.ignition_can


def test_bad_checksum_cabin_does_not_update_state():
  power(3)
  gear(4)
  assert libpanda.ignition_can
  # Corrupt cabin frames must not latch the belt or report a closed door,
  # so P must still drop ignition.
  data = bytearray(7)
  data[1] = 0 | (1 << 5)  # buckleStatus latched, counter 0
  data[3] = 0             # anyDoorOpen clear
  data[0] = tesla_checksum(0x311, data, 0) ^ 0x01
  send(0x311, data)
  data[1] = 1 | (1 << 5)
  data[0] = tesla_checksum(0x311, data, 0) ^ 0x01
  send(0x311, data)
  gear(1)
  assert not libpanda.ignition_can


@pytest.mark.parametrize("address,length", [(0x221, 7), (0x118, 7), (0x311, 8)])
def test_incorrect_frame_lengths_do_not_change_state(address, length):
  for counter in (0, 1):
    data = bytearray(length)
    data[0] = 3 << 5
    data[1] = counter
    data[2] = 4 << 5
    data[6] = counter << 4
    send(address, data)
  assert not libpanda.wake_on_can
  assert not libpanda.ignition_can


@pytest.mark.parametrize("bus", [1, 2])
def test_wake_and_ignition_only_use_bus_zero(bus):
  power(3, bus=bus)
  assert not libpanda.wake_on_can
  power(3)
  gear(4, bus=bus)
  assert not libpanda.ignition_can


def test_counter_discontinuities_are_ignored_and_wrap_is_accepted():
  power(3, counters=(3, 5))
  assert not libpanda.wake_on_can
  power(3, counters=(15, 0))
  assert libpanda.wake_on_can
  gear(4, counters=(3, 5))
  assert not libpanda.ignition_can
  gear(4, counters=(15, 0))
  assert libpanda.ignition_can


def test_other_brand_ignition_source_still_works():
  send(0x1F1, bytes([2]) + bytes(7))
  assert libpanda.ignition_can
  send(0x1F1, bytes(8))
  assert not libpanda.ignition_can
