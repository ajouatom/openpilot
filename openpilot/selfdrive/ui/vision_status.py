"""Display-only Xiaoge state; unknown or unevaluated sides are never shown clear."""

from dataclasses import dataclass
import json
import math

from openpilot.selfdrive.carrot.xiaoge.xiaoge_vision import (
  XIAOGE_BLINDSPOT_TIMEOUT_NS, XIAOGE_LANE_TIMEOUT_NS, XiaogeVisionResult, parse_xiaoge_vision_payload,
)


@dataclass(frozen=True)
class VisionDisplayPacket:
  result: XiaogeVisionResult
  blindspot_side: str = ""
  latency_ms: float | None = None


@dataclass(frozen=True)
class VisionDisplayState:
  state: str = "waiting"
  left_lane: int = -1
  right_lane: int = -1
  clear_side: str = ""
  latency_ms: float | None = None


@dataclass(frozen=True)
class BlindspotSourcePacket:
  left_oem: bool
  left_vision: bool
  right_oem: bool
  right_vision: bool
  received_nanos: int


def parse_blindspot_source_packet(payload: bytes) -> BlindspotSourcePacket:
  data = json.loads(payload)
  if not isinstance(data, dict) or data.get("type") != "xiaogeBlindspotSources" or data.get("version") != 1:
    raise ValueError("expected a version 1 xiaogeBlindspotSources object")
  left, right = data.get("left"), data.get("right")
  if not isinstance(left, dict) or not isinstance(right, dict):
    raise ValueError("left and right must be objects")
  received_nanos = data.get("receivedMonoTimeNanos")
  if isinstance(received_nanos, bool) or not isinstance(received_nanos, int) or received_nanos < 0:
    raise ValueError("receivedMonoTimeNanos must be a non-negative integer")
  values = [left.get("oem"), left.get("vision"), right.get("oem"), right.get("vision")]
  if any(not isinstance(value, bool) for value in values):
    raise ValueError("blindspot source states must be booleans")
  return BlindspotSourcePacket(*values, received_nanos)


def blindspot_source_packet(sm, now_nanos: int) -> BlindspotSourcePacket | None:
  try:
    if not sm.valid["customReservedRawData1"]:
      return None
    packet = parse_blindspot_source_packet(bytes(sm["customReservedRawData1"]))
  except (KeyError, UnicodeDecodeError, ValueError, TypeError):
    return None
  age_nanos = now_nanos - packet.received_nanos
  return packet if 0 <= age_nanos <= XIAOGE_BLINDSPOT_TIMEOUT_NS else None


def blindspot_sources(car_state, side: str, packet: BlindspotSourcePacket | None = None) -> tuple[bool, bool]:
  if bool(getattr(car_state, "blindspotSplitSourcesValid", False)):
    oem_state = getattr(car_state, f"{side}BlindspotOem", None)
    vision_state = getattr(car_state, f"{side}BlindspotOnnx", None)
    if isinstance(oem_state, bool) and isinstance(vision_state, bool):
      return oem_state, vision_state
  if packet is not None:
    return bool(getattr(packet, f"{side}_oem")), bool(getattr(packet, f"{side}_vision"))
  merged = bool(getattr(car_state, f"{side}Blindspot"))
  return merged, False


def parse_vision_display_packet(payload: bytes) -> VisionDisplayPacket:
  result = parse_xiaoge_vision_payload(payload)
  data = json.loads(payload)
  side = data["blindspot"].get("side", "")
  side = side if side in ("left", "right") else ""
  latency = data["lane"].get("latencyMs")
  if isinstance(latency, bool) or not isinstance(latency, (int, float)) or not math.isfinite(latency) or latency < 0:
    latency = None
  return VisionDisplayPacket(result, side, latency)


def vision_display_state(packet: VisionDisplayPacket | None, now_nanos: int) -> VisionDisplayState:
  if packet is None:
    return VisionDisplayState()
  result = packet.result
  lane_age = now_nanos - result.lane_received_nanos
  lane_fresh = result.lane_valid and result.lane_received_nanos > 0 and 0 <= lane_age <= XIAOGE_LANE_TIMEOUT_NS
  side_received = {
    "left": result.left_blindspot_received_nanos or result.blindspot_received_nanos,
    "right": result.right_blindspot_received_nanos or result.blindspot_received_nanos,
  }.get(packet.blindspot_side, 0)
  blindspot_age = now_nanos - side_received
  blindspot_fresh = (result.blindspot_valid and side_received > 0 and
                     0 <= blindspot_age <= XIAOGE_BLINDSPOT_TIMEOUT_NS)
  side_detected = result.left_blindspot if packet.blindspot_side == "left" else result.right_blindspot
  return VisionDisplayState(
    state="running" if lane_fresh else "stale",
    left_lane=result.left_lane if lane_fresh else -1,
    right_lane=result.right_lane if lane_fresh else -1,
    clear_side=packet.blindspot_side if blindspot_fresh and not side_detected else "",
    latency_ms=packet.latency_ms if lane_fresh else None,
  )
