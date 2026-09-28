import math

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.steering_touch import HyundaiSteeringTouch, TOUCH_ADDR, TOUCH_MSG, touch_checksum


def frame(counter, status=1, touch1=19, touch2=14):
  payload = bytes([counter << 4, status, 1, touch1, touch2, 1, 0])
  return bytes([touch_checksum(payload)]) + payload


def setup(bus=0, supported=True):
  return CANParser('hyundai_canfd_generated', [(TOUCH_MSG, math.nan)], bus), HyundaiSteeringTouch(supported)


def receive(cp, monitor, timestamp, data, bus=None):
  cp.update([timestamp, [(TOUCH_ADDR, data, cp.bus if bus is None else bus)]])
  return monitor.update(cp)


@pytest.mark.parametrize('raw', ['7e6000010c0e0100', '444001011e100100', '9ea0020133160100', '0b300301371b0100'])
def test_independent_recorded_receive_checksum_vectors(raw):
  data = bytes.fromhex(raw)
  assert touch_checksum(data[1:]) == data[0]


@pytest.mark.parametrize('status', [0, 1, 2, 3, 4])
def test_lowest_contact_status_and_counter_rollover(status):
  cp, monitor = setup()
  assert not receive(cp, monitor, 1_000_000_000, frame(14, status))['valid']
  state = receive(cp, monitor, 1_100_000_000, frame(0, status))
  assert state['valid'] and state['touched'] == (status > 0)
  cs = structs.CarState(steeringTouch=state)
  assert cs.steeringTouch.rawStatus == status
  assert not cs.steeringPressed


@pytest.mark.parametrize('bus', [0, 4])
def test_only_original_ecan_not_forwarding_or_send_receipts(bus):
  cp, monitor = setup(bus)
  for source in (bus + 2, bus + 128, bus + 130):
    state = receive(cp, monitor, 1_000_000_000, frame(0), source)
    assert not state['available'] and not state['touched']
  receive(cp, monitor, 1_100_000_000, frame(0))
  assert receive(cp, monitor, 1_200_000_000, frame(1))['touched']


@pytest.mark.parametrize('invalid', [b'', frame(2)[:-1], frame(2) + b'\x00',
                                   bytes([0]) + frame(2)[1:], frame(2, 5), frame(15)])
def test_malformed_or_unknown_frame_revokes_contact_immediately(invalid):
  cp, monitor = setup()
  receive(cp, monitor, 1_000_000_000, frame(0))
  assert receive(cp, monitor, 1_100_000_000, frame(1))['touched']
  state = receive(cp, monitor, 1_200_000_000, invalid)
  assert not state['valid'] and not state['touched']


def test_frozen_counter_stale_data_empty_bus_and_recovery():
  cp, monitor = setup()
  receive(cp, monitor, 1_000_000_000, frame(0))
  assert receive(cp, monitor, 1_100_000_000, frame(1))['touched']
  assert not receive(cp, monitor, 1_200_000_000, frame(1))['valid']
  assert receive(cp, monitor, 1_300_000_000, frame(2))['valid']
  cp.update([1_550_000_001, []])
  assert not monitor.update(cp)['valid']
  assert not receive(cp, monitor, 1_600_000_000, frame(3))['valid']
  assert receive(cp, monitor, 1_700_000_000, frame(4))['valid']
  # Releasing requires no extra delay once the stream is healthy.
  assert not receive(cp, monitor, 1_800_000_000, frame(5, 0))['touched']


def test_missing_unknown_platform_and_read_only_contract():
  cp = CANParser('hyundai_canfd_generated', [], 0)
  monitor = HyundaiSteeringTouch(True)
  addresses = cp.addresses.copy()
  assert not monitor.update(cp)['available']
  assert cp.addresses == addresses
  cp, monitor = setup()
  receive(cp, monitor, 1_000_000_000, frame(0))
  state = receive(cp, monitor, 1_100_000_000, frame(1))
  assert state['touched']
  snapshot = dict(cp.vl[TOUCH_MSG]), cp.dat[TOUCH_ADDR]
  assert HyundaiSteeringTouch(False).update(cp) == {}
  monitor.update(cp)
  assert snapshot == (dict(cp.vl[TOUCH_MSG]), cp.dat[TOUCH_ADDR])


def test_unexpected_profile_bytes_and_counter_jump_are_not_evidence():
  cp, monitor = setup()
  receive(cp, monitor, 1_000_000_000, frame(0))
  assert not receive(cp, monitor, 1_100_000_000, frame(3))['valid']
  data = bytearray(frame(4))
  data[3] = 2
  data[0] = touch_checksum(data[1:])
  assert not receive(cp, monitor, 1_200_000_000, bytes(data))['valid']
