from dataclasses import replace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.carrot.radar.can_batch import (
  MAX_CAN_PACKETS, MAX_INPUT_AGE_NS, MAX_STATE_PACKETS, RadarCanBatches, RadarEgoSample,
)


def sample(first=10, last=20, count=2, receive=30):
  return RadarEgoSample(first, last, count, receive, 12.5, -1.2)


@pytest.mark.parametrize('state_first', [False, True])
def test_independent_socket_order_preserves_exact_batch_and_ego(state_first):
  buffer = RadarCanBatches()
  packets = [(10, [(1, b'a', 0)]), (20, [(2, b'b', 1)])]
  if state_first:
    buffer.add_state(sample())
    assert buffer.take(30) is None
    buffer.add_can([(1, [])] + packets + [(40, [])])
  else:
    buffer.add_can([(1, [])] + packets + [(40, [])])
    assert buffer.take(30) is None
    buffer.add_state(sample())
  assert buffer.take(31) == (sample(), packets, None)
  assert list(buffer.can) == [(40, [])]
  assert buffer.take(31) is None


def test_partial_can_arrival_waits_for_last_packet():
  buffer = RadarCanBatches()
  buffer.add_state(sample())
  buffer.add_can([(10, [])])
  assert buffer.take(31) is None
  buffer.add_can([(20, [])])
  assert buffer.take(32)[2] is None


def test_missing_middle_packet_invalidates_even_when_endpoints_match():
  buffer = RadarCanBatches()
  buffer.add_state(sample(count=3))
  buffer.add_can([(10, []), (20, [])])
  assert buffer.take(31)[2] == 'missingCanPacket'


def test_empty_can_timeout_still_advances_one_radar_interface_tick():
  buffer = RadarCanBatches()
  state = sample(0, 0, 0)
  buffer.add_state(state)
  assert buffer.take(31) == (state, [], None)


def test_expired_missing_batch_does_not_block_newer_input():
  buffer = RadarCanBatches()
  buffer.add_state(sample())
  later = sample(40, 40, 1, MAX_INPUT_AGE_NS + 32)
  buffer.add_state(later)
  buffer.add_can([(40, [])])
  now = MAX_INPUT_AGE_NS + 32
  assert buffer.take(now)[2] == 'staleEgoState'
  assert buffer.take(now) == (later, [(40, [])], None)


def test_missing_metadata_cannot_silently_use_latest_ego_state():
  state = RadarEgoSample.from_car_state(car.CarState.new_message())
  buffer = RadarCanBatches()
  buffer.add_state(state)
  assert buffer.take(31)[2] == 'missingBatchMetadata'


def test_buffers_are_bounded_and_report_lost_input():
  buffer = RadarCanBatches()
  for _ in range(MAX_STATE_PACKETS + 1):
    buffer.add_state(sample())
  assert len(buffer.states) <= MAX_STATE_PACKETS
  assert buffer.take(31)[2] == 'stateOverflow'
  buffer.add_can([(i, []) for i in range(1, MAX_CAN_PACKETS + 10)])
  assert len(buffer.can) == MAX_CAN_PACKETS
  buffer.add_state(sample(1, 20, 20))
  assert buffer.take(31)[2] == 'missingCanPacket'


def test_consecutive_updates_keep_ego_history_cadence_through_batched_can():
  buffer = RadarCanBatches()
  states = [sample(10, 20, 2), sample(40, 40, 1, 50), sample(0, 0, 0, 60)]
  states = [replace(state, v_ego=10.0 + i) for i, state in enumerate(states)]
  buffer.add_can([(10, []), (20, []), (40, []), (80, [])])
  for state in states:
    buffer.add_state(state)
  delivered = [buffer.take(61) for _ in states]
  assert [item[0] for item in delivered] == states
  assert [len(item[1]) for item in delivered] == [2, 1, 0]
  assert all(item[2] is None for item in delivered)
  assert list(buffer.can) == [(80, [])]
