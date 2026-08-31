import json
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.xiaoge_lane import (
  XIAOGE_LANE_TIMEOUT_NS,
  apply_xiaoge_lane_result,
  merge_xiaoge_lane_type,
  parse_xiaoge_lane_payload,
  parse_xiaoge_udp_payload,
)


def lane_payload(**overrides) -> bytes:
  data = {
    "type": "lane",
    "version": 1,
    "leftLine": 0,
    "rightLine": 1,
    "lineValid": True,
    "receivedMonoTimeNanos": 1_000_000_000,
  }
  data.update(overrides)
  return json.dumps(data).encode()


def test_lane_payload_validation():
  assert parse_xiaoge_lane_payload(lane_payload()) == (0, 1, True, 1_000_000_000)

  for overrides in (
    {"version": 2},
    {"leftLine": True},
    {"rightLine": 2},
    {"lineValid": 1},
    {"receivedMonoTimeNanos": -1},
  ):
    with pytest.raises(ValueError):
      parse_xiaoge_lane_payload(lane_payload(**overrides))


def test_udp_lane_payload_requires_integer_lane_types():
  assert parse_xiaoge_udp_payload(b'{"resp":"lane","left_lane":0,"right_lane":1}') == (0, 1)

  for payload in (
    b'{"resp":"other","left_lane":0,"right_lane":1}',
    b'{"resp":"lane","left_lane":false,"right_lane":1}',
    b'{"resp":"lane","left_lane":0.5,"right_lane":1}',
  ):
    with pytest.raises(ValueError):
      parse_xiaoge_udp_payload(payload)


def test_lane_result_preserves_vehicle_color_and_unknown_side():
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14)

  assert apply_xiaoge_lane_result(state, (1, -1, True, 1_000), 2_000)
  assert state.leftLaneLine == 21
  assert state.rightLaneLine == 14
  assert merge_xiaoge_lane_type(-1, 0) == 0


@pytest.mark.parametrize(
  "result, now_nanos",
  [
    (None, 1_000),
    ((0, 1, False, 1_000), 2_000),
    ((0, 1, True, 0), 2_000),
    ((0, 1, True, 2_001), 2_000),
    ((0, 1, True, 1_000), 1_000 + XIAOGE_LANE_TIMEOUT_NS + 1),
    ((-1, -1, True, 1_000), 2_000),
  ],
)
def test_invalid_or_stale_lane_result_does_not_modify_state(result, now_nanos):
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14)

  assert not apply_xiaoge_lane_result(state, result, now_nanos)
  assert (state.leftLaneLine, state.rightLaneLine) == (24, 14)
