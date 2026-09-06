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
  blindspot_age = now_nanos - result.blindspot_received_nanos
  blindspot_fresh = (result.blindspot_valid and result.blindspot_received_nanos > 0 and
                    0 <= blindspot_age <= XIAOGE_BLINDSPOT_TIMEOUT_NS)
  side_detected = result.left_blindspot if packet.blindspot_side == "left" else result.right_blindspot
  return VisionDisplayState(
    state="running" if lane_fresh else "stale",
    left_lane=result.left_lane if lane_fresh else -1,
    right_lane=result.right_lane if lane_fresh else -1,
    clear_side=packet.blindspot_side if blindspot_fresh and not side_detected else "",
    latency_ms=packet.latency_ms if lane_fresh else None,
  )
