import pytest

from panda.tests.libpanda.libpanda_py import libpanda, make_CANPacket


def send(address, data, bus=0):
  libpanda.ignition_can_hook(make_CANPacket(address, bus, data))


def power(state, counters=(0, 1), bus=0):
  for counter in counters:
    data = bytearray(8)
    data[0] = state << 5
    data[6] = counter << 4
    send(0x221, data, bus)


def gear(value, counters=(0, 1), bus=0):
  for counter in counters:
    data = bytearray(8)
    data[1] = counter
    data[2] = value << 5
    send(0x118, data, bus)


def cabin(*, latched=False, door_open=False):
  for counter in (0, 1):
    data = bytearray(7)
    data[1] = counter | (int(latched) << 5)
    data[3] = int(door_open) << 4
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


def test_stale_tesla_wake_cannot_authorize_gear_updates():
  power(3)
  libpanda.wake_on_can_cnt = 3
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
