import json
from typing import Protocol


XIAOGE_LANE_TIMEOUT_NS = 3_000_000_000
XIAOGE_LANE_TYPES = (-1, 0, 1)

XiaogeLaneResult = tuple[int, int, bool, int]


class LaneState(Protocol):
  leftLaneLine: int
  rightLaneLine: int


def _lane_type(value: object, field: str) -> int:
  if isinstance(value, bool) or not isinstance(value, int) or value not in XIAOGE_LANE_TYPES:
    raise ValueError(f"{field} must be -1, 0, or 1")
  return value


def parse_xiaoge_lane_payload(payload: bytes) -> XiaogeLaneResult:
  data = json.loads(payload)
  if not isinstance(data, dict) or data.get("type") != "lane" or data.get("version") != 1:
    raise ValueError("expected a version 1 lane object")

  left_lane = _lane_type(data.get("leftLine"), "leftLine")
  right_lane = _lane_type(data.get("rightLine"), "rightLine")
  line_valid = data.get("lineValid")
  received_nanos = data.get("receivedMonoTimeNanos")
  if not isinstance(line_valid, bool):
    raise ValueError("lineValid must be a boolean")
  if isinstance(received_nanos, bool) or not isinstance(received_nanos, int) or received_nanos < 0:
    raise ValueError("receivedMonoTimeNanos must be a non-negative integer")
  return left_lane, right_lane, line_valid, received_nanos


def parse_xiaoge_udp_payload(payload: bytes) -> tuple[int, int]:
  data = json.loads(payload)
  if not isinstance(data, dict) or data.get("resp") != "lane":
    raise ValueError("expected a lane response object")
  return _lane_type(data.get("left_lane"), "left_lane"), _lane_type(data.get("right_lane"), "right_lane")


def merge_xiaoge_lane_type(current: int, detected: int) -> int:
  """Replace only the lane-marking type while preserving a vehicle-provided color code."""
  if detected < 0:
    return current
  color = (current // 10) * 10 if current >= 10 else 0
  return color + detected


def apply_xiaoge_lane_result(CS: LaneState, lane_result: XiaogeLaneResult | None, now_nanos: int) -> bool:
  if lane_result is None:
    return False

  left_lane, right_lane, line_valid, received_nanos = lane_result
  age_nanos = now_nanos - received_nanos
  if not line_valid or received_nanos == 0 or age_nanos < 0 or age_nanos > XIAOGE_LANE_TIMEOUT_NS:
    return False

  applied = False
  if left_lane >= 0:
    CS.leftLaneLine = merge_xiaoge_lane_type(CS.leftLaneLine, left_lane)
    applied = True
  if right_lane >= 0:
    CS.rightLaneLine = merge_xiaoge_lane_type(CS.rightLaneLine, right_lane)
    applied = True
  return applied
