import json
from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.selfdrive.ui.vision_status import (
  blindspot_source_packet, blindspot_sources, parse_blindspot_source_packet,
  parse_vision_display_packet, vision_display_state,
)

NOW = 10_000_000_000


def packet(*, lane_time=NOW, blindspot_time=NOW, side="left", valid=True, detected=False, latency=485.0, side_time=None):
  blindspot = {"left": detected, "right": False, "valid": valid, "receivedMonoTimeNanos": blindspot_time, "side": side}
  if side_time is not None:
    blindspot["receivedMonoTimeNanosBySide"] = {"left": side_time, "right": blindspot_time}
  return parse_vision_display_packet(json.dumps({
    "type": "xiaogeVision", "version": 1,
    "lane": {"leftLine": 0, "rightLine": 1, "valid": True, "receivedMonoTimeNanos": lane_time, "latencyMs": latency},
    "blindspot": blindspot,
  }).encode())


def test_lane_status_and_only_evaluated_blindspot_side():
  state = vision_display_state(packet(), NOW)
  assert (state.state, state.left_lane, state.right_lane, state.clear_side, state.latency_ms) == ("running", 0, 1, "left", 485.0)


@pytest.mark.parametrize("kwargs", [{"side": ""}, {"valid": False}, {"detected": True}, {"blindspot_time": NOW - 1_500_000_001}, {"blindspot_time": NOW + 1}])
def test_unevaluated_detected_or_stale_side_is_never_clear(kwargs):
  assert vision_display_state(packet(**kwargs), NOW).clear_side == ""


@pytest.mark.parametrize("lane_time", [0, NOW + 1, NOW - 4_000_000_001])
def test_stale_or_future_lanes_are_unknown(lane_time):
  state = vision_display_state(packet(lane_time=lane_time), NOW)
  assert (state.state, state.left_lane, state.right_lane, state.latency_ms) == ("stale", -1, -1, None)


def test_waiting_without_any_packet():
  assert vision_display_state(None, NOW).state == "waiting"


@pytest.mark.parametrize("latency", [None, -1, True, "485", float("nan"), float("inf")])
def test_optional_latency_is_not_trusted(latency):
  assert vision_display_state(packet(latency=latency), NOW).latency_ms is None


def test_legacy_payload_cannot_claim_both_sides_were_checked():
  state = vision_display_state(packet(side="", latency=None), NOW)
  assert state.state == "running"
  assert state.clear_side == ""


def test_display_uses_selected_side_timestamp():
  state = vision_display_state(packet(side_time=NOW - 1_500_000_001), NOW)
  assert state.clear_side == ""


def test_blindspot_sources_use_fresh_raw_packet_without_carstate_schema_fields():
  payload = json.dumps({
    "type": "xiaogeBlindspotSources",
    "version": 1,
    "receivedMonoTimeNanos": NOW,
    "left": {"oem": False, "vision": True},
    "right": {"oem": True, "vision": True},
  }).encode()
  parsed = parse_blindspot_source_packet(payload)
  car_state = SimpleNamespace(leftBlindspot=True, rightBlindspot=True)

  assert blindspot_sources(car_state, "left", parsed) == (False, True)
  assert blindspot_sources(car_state, "right", parsed) == (True, True)


def test_blindspot_source_packet_rejects_stale_and_invalid_payloads():
  class SM(dict):
    valid = {"customReservedRawData1": True}

  stale = json.dumps({
    "type": "xiaogeBlindspotSources",
    "version": 1,
    "receivedMonoTimeNanos": NOW - 1_500_000_001,
    "left": {"oem": False, "vision": True},
    "right": {"oem": False, "vision": False},
  }).encode()
  assert blindspot_source_packet(SM(customReservedRawData1=stale), NOW) is None
  assert blindspot_source_packet(SM(customReservedRawData1=b"{}"), NOW) is None


def test_blindspot_sources_prefer_carstate_split_fields():
  car_state = SimpleNamespace(
    leftBlindspot=True, rightBlindspot=True,
    blindspotSplitSourcesValid=True,
    leftBlindspotOem=False, rightBlindspotOem=True,
    leftBlindspotOnnx=True, rightBlindspotOnnx=False,
  )

  assert blindspot_sources(car_state, "left", None) == (False, True)
  assert blindspot_sources(car_state, "right", None) == (True, False)


def test_blindspot_sources_ignore_default_split_fields_without_marker():
  payload = json.dumps({
    "type": "xiaogeBlindspotSources",
    "version": 1,
    "receivedMonoTimeNanos": NOW,
    "left": {"oem": False, "vision": True},
    "right": {"oem": True, "vision": True},
  }).encode()
  parsed = parse_blindspot_source_packet(payload)

  state_msg = car.CarState.new_message()
  state_msg.leftBlindspot = True
  state_msg.rightBlindspot = True
  with car.CarState.from_bytes(state_msg.to_bytes()) as decoded:
    assert blindspot_sources(decoded, "left", parsed) == (False, True)
    assert blindspot_sources(decoded, "right", parsed) == (True, True)
