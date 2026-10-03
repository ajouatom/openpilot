from opendbc.car import structs
from opendbc.car.hyundai.carstate import CarState


def _stalk_state():
  cs = CarState.__new__(CarState)
  cs.blinker_stalks = None
  cs.left_stalk_prev = cs.right_stalk_prev = False
  cs.left_blinker_stalk_count = cs.right_blinker_stalk_count = 0
  return cs


def _counts(cs, left, right):
  cs.blinker_stalks = {"LEFT_BLINKER": left, "RIGHT_BLINKER": right}
  ret = structs.CarState()
  cs._update_blinker_stalks(ret)
  return ret.leftBlinkerStalkCount, ret.rightBlinkerStalkCount


def test_stalk_counter_counts_presses_not_hold_time():
  cs = _stalk_state()
  assert _counts(cs, 0, 0) == (0, 0)
  assert _counts(cs, 1, 0) == (1, 0)                                # press
  assert _counts(cs, 1, 0) == (1, 0)                                # still held: same press
  assert _counts(cs, 0, 0) == (1, 0)
  assert _counts(cs, 1, 0) == (2, 0)                                # second press
  assert _counts(cs, 0, 1) == (2, 1)


def test_stalk_counter_wraps_and_survives_a_missing_message():
  cs = _stalk_state()
  cs.left_blinker_stalk_count = 255
  assert _counts(cs, 1, 0) == (0, 0)                                # UInt8 wrap
  cs.blinker_stalks = None                                          # BLINKER_STALKS not on this car
  ret = structs.CarState()
  cs._update_blinker_stalks(ret)
  assert (ret.leftBlinkerStalkCount, ret.rightBlinkerStalkCount) == (0, 0)
