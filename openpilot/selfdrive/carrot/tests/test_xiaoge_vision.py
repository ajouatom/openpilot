import json
import threading
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.xiaoge import v_asm_server
from openpilot.selfdrive.carrot.xiaoge.v_asm_server import VASMService
from openpilot.selfdrive.carrot.xiaoge.xiaoge_vision import (
  XIAOGE_LANE_TIMEOUT_NS,
  XiaogeVisionResult,
  apply_xiaoge_vision_result,
  merge_xiaoge_lane_type,
  parse_xiaoge_vision_payload,
)


def vision_payload(**overrides) -> bytes:
  data = {
    "type": "xiaogeVision",
    "version": 1,
    "lane": {"leftLine": 0, "rightLine": 1, "valid": True, "receivedMonoTimeNanos": 1_000_000_000},
    "blindspot": {"left": True, "right": False, "valid": True, "receivedMonoTimeNanos": 1_000_000_000},
  }
  data.update(overrides)
  return json.dumps(data).encode()


def test_vision_payload_validation():
  result = parse_xiaoge_vision_payload(vision_payload())
  assert result.left_lane == 0
  assert result.left_blindspot

  for payload in (
    vision_payload(version=2),
    vision_payload(lane={"leftLine": False, "rightLine": 1, "valid": True, "receivedMonoTimeNanos": 1}),
    vision_payload(blindspot={"left": True, "right": 0, "valid": True, "receivedMonoTimeNanos": 1}),
  ):
    with pytest.raises(ValueError):
      parse_xiaoge_vision_payload(payload)


def test_vision_result_merges_lane_type_and_oem_blindspot():
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14, leftBlindspot=False, rightBlindspot=True)
  result = parse_xiaoge_vision_payload(vision_payload())

  assert apply_xiaoge_vision_result(state, result, 1_000_000_001)
  assert (state.leftLaneLine, state.rightLaneLine) == (20, 11)
  assert state.leftBlindspot
  assert state.rightBlindspot
  assert merge_xiaoge_lane_type(-1, 0) == 0


def test_stale_vision_result_does_not_modify_state():
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14, leftBlindspot=False, rightBlindspot=False)
  result = XiaogeVisionResult(0, 1, True, 1, True, True, True, 1)

  assert not apply_xiaoge_vision_result(state, result, XIAOGE_LANE_TIMEOUT_NS + 2)
  assert (state.leftLaneLine, state.rightLaneLine, state.leftBlindspot, state.rightBlindspot) == (24, 14, False, False)


def test_vasm_gate_requires_speed_direction_and_target_lane_width():
  service = VASMService.__new__(VASMService)
  service.lock = threading.Lock()
  service.vasm_gate = {}
  car_state = SimpleNamespace(vEgo=20.0)
  model_v2 = SimpleNamespace(meta=SimpleNamespace(
    laneChangeDirection=log.LaneChangeDirection.left,
    laneWidthLeft=3.2,
    laneWidthRight=2.8,
  ))

  class FakeSubMaster:
    valid = {"carState": True, "modelV2": True}

    @staticmethod
    def update(_timeout):
      pass

    def __getitem__(self, key):
      return {"carState": car_state, "modelV2": model_v2}[key]

  service.sm = FakeSubMaster()

  assert service._update_vasm_gate() == (True, "left")

  model_v2.meta.laneChangeDirection = log.LaneChangeDirection.right
  assert service._update_vasm_gate() == (False, "right")
  assert service.vasm_gate["reason"] == "target lane width below 3.0 m"

  car_state.vEgo = 5.0
  assert service._update_vasm_gate() == (False, "")


def test_vision_service_publishes_the_composite_payload(monkeypatch):
  service = VASMService.__new__(VASMService)
  service.lock = threading.Lock()
  service.publish_lock = threading.Lock()
  service.lane_result = {
    "leftLine": 0,
    "rightLine": 1,
    "valid": True,
    "updatedMonoTimeNanos": 100,
  }
  service.vasm_result = {"left": True, "right": False, "updatedMonoTimeNanos": 200}
  sent = []

  class FakePublisher:
    @staticmethod
    def new_message(_service, size, valid):
      assert valid
      return SimpleNamespace(customReservedRawData0=b"\0" * size)

    def send(self, service_name, message):
      sent.append((service_name, message.customReservedRawData0))

  monkeypatch.setattr(v_asm_server.messaging, "new_message", FakePublisher.new_message)
  service.pm = FakePublisher()

  service.publish_vision_result()

  assert len(sent) == 1
  assert sent[0][0] == "customReservedRawData0"
  result = parse_xiaoge_vision_payload(sent[0][1])
  assert (result.left_lane, result.right_lane, result.left_blindspot, result.right_blindspot) == (0, 1, True, False)
