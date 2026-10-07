"""Xiaoge vision payload validation and car-state integration helpers."""

from dataclasses import dataclass
import json
from typing import Protocol


XIAOGE_LANE_TIMEOUT_NS = 4_000_000_000
XIAOGE_BLINDSPOT_TIMEOUT_NS = 1_500_000_000
XIAOGE_LANE_TYPES = (-1, 0, 1)


@dataclass(frozen=True)
class XiaogeVisionResult:
  left_lane: int
  right_lane: int
  lane_valid: bool
  lane_received_nanos: int
  left_blindspot: bool
  right_blindspot: bool
  blindspot_valid: bool
  blindspot_received_nanos: int
  left_blindspot_received_nanos: int = 0
  right_blindspot_received_nanos: int = 0


class VisionState(Protocol):
  leftLaneLine: int
  rightLaneLine: int
  leftBlindspot: bool
  rightBlindspot: bool


def _lane_type(value: object, field: str) -> int:
  if isinstance(value, bool) or not isinstance(value, int) or value not in XIAOGE_LANE_TYPES:
    raise ValueError(f"{field} must be -1, 0, or 1")
  return value


def _bool(value: object, field: str) -> bool:
  if not isinstance(value, bool):
    raise ValueError(f"{field} must be a boolean")
  return value


def _received_nanos(value: object, field: str) -> int:
  if isinstance(value, bool) or not isinstance(value, int) or value < 0:
    raise ValueError(f"{field} must be a non-negative integer")
  return value


def parse_xiaoge_vision_payload(payload: bytes) -> XiaogeVisionResult:
  data = json.loads(payload)
  if not isinstance(data, dict) or data.get("type") != "xiaogeVision" or data.get("version") != 1:
    raise ValueError("expected a version 1 xiaogeVision object")
  lane = data.get("lane")
  blindspot = data.get("blindspot")
  if not isinstance(lane, dict) or not isinstance(blindspot, dict):
    raise ValueError("lane and blindspot must be objects")
  blindspot_received_nanos = _received_nanos(blindspot.get("receivedMonoTimeNanos"), "blindspot.receivedMonoTimeNanos")
  by_side = blindspot.get("receivedMonoTimeNanosBySide", {})
  if not isinstance(by_side, dict):
    raise ValueError("blindspot.receivedMonoTimeNanosBySide must be an object")
  return XiaogeVisionResult(
    _lane_type(lane.get("leftLine"), "lane.leftLine"),
    _lane_type(lane.get("rightLine"), "lane.rightLine"),
    _bool(lane.get("valid"), "lane.valid"),
    _received_nanos(lane.get("receivedMonoTimeNanos"), "lane.receivedMonoTimeNanos"),
    _bool(blindspot.get("left"), "blindspot.left"),
    _bool(blindspot.get("right"), "blindspot.right"),
    _bool(blindspot.get("valid"), "blindspot.valid"),
    blindspot_received_nanos,
    _received_nanos(by_side.get("left", blindspot_received_nanos), "blindspot.receivedMonoTimeNanosBySide.left"),
    _received_nanos(by_side.get("right", blindspot_received_nanos), "blindspot.receivedMonoTimeNanosBySide.right"),
  )


def merge_xiaoge_lane_type(current: int, detected: int) -> int:
  """Replace only the lane-marking type while preserving a vehicle-provided color code."""
  if detected < 0:
    return current
  color = (current // 10) * 10 if current >= 10 else 0
  return color + detected


def _is_fresh(received_nanos: int, now_nanos: int, timeout_nanos: int) -> bool:
  age_nanos = now_nanos - received_nanos
  return received_nanos != 0 and 0 <= age_nanos <= timeout_nanos


def xiaoge_blindspot_vision(result: XiaogeVisionResult | None, now_nanos: int) -> tuple[bool, bool]:
  if result is None or not result.blindspot_valid or \
     not _is_fresh(result.blindspot_received_nanos, now_nanos, XIAOGE_BLINDSPOT_TIMEOUT_NS):
    return False, False
  left_received = result.left_blindspot_received_nanos or result.blindspot_received_nanos
  right_received = result.right_blindspot_received_nanos or result.blindspot_received_nanos
  return (
    result.left_blindspot and _is_fresh(left_received, now_nanos, XIAOGE_BLINDSPOT_TIMEOUT_NS),
    result.right_blindspot and _is_fresh(right_received, now_nanos, XIAOGE_BLINDSPOT_TIMEOUT_NS),
  )


def apply_xiaoge_vision_result(CS: VisionState, result: XiaogeVisionResult | None, now_nanos: int) -> bool:
  if result is None:
    return False

  applied = False
  if result.lane_valid and _is_fresh(result.lane_received_nanos, now_nanos, XIAOGE_LANE_TIMEOUT_NS):
    if result.left_lane >= 0:
      CS.leftLaneLine = merge_xiaoge_lane_type(CS.leftLaneLine, result.left_lane)
      applied = True
    if result.right_lane >= 0:
      CS.rightLaneLine = merge_xiaoge_lane_type(CS.rightLaneLine, result.right_lane)
      applied = True

  left_vision, right_vision = xiaoge_blindspot_vision(result, now_nanos)
  if left_vision or right_vision:
    CS.leftBlindspot = bool(CS.leftBlindspot) or left_vision
    CS.rightBlindspot = bool(CS.rightBlindspot) or right_vision
    applied = True
  return applied
