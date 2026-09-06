import json
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from io import BytesIO
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from PIL import Image

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.carrot.xiaoge import v_asm_server
from openpilot.selfdrive.carrot.xiaoge.v_asm_server import VASMService
from openpilot.selfdrive.carrot.xiaoge.xiaoge_vision import (
  XIAOGE_BLINDSPOT_TIMEOUT_NS,
  XIAOGE_LANE_TIMEOUT_NS,
  XiaogeVisionResult,
  apply_xiaoge_vision_result,
  merge_xiaoge_lane_type,
  parse_xiaoge_vision_payload,
)


@pytest.fixture
def message_transport(monkeypatch):
  """Replace native sockets while retaining SubMaster, PubMaster and cereal."""
  sockets = {}
  sent = []

  class Socket:
    data = None

    def receive(self, non_blocking=False):
      data, self.data = self.data, None
      return data

  class Poller:
    def poll(self, _timeout):
      return [sock for sock in sockets.values() if sock.data is not None]

  def sub_sock(service, **_kwargs):
    sockets[service] = Socket()
    return sockets[service]

  def publish(service, valid=True, **data):
    message = messaging.new_message(service, valid=valid)
    setattr(message, service, data)
    sockets[service].data = message.to_bytes()

  monkeypatch.setattr(messaging, "Poller", Poller)
  monkeypatch.setattr(messaging, "sub_sock", sub_sock)
  monkeypatch.setattr(messaging, "pub_sock", lambda service: SimpleNamespace(send=lambda data: sent.append((service, data))))
  return SimpleNamespace(publish=publish, sent=sent)


@pytest.fixture
def gate_service(message_transport):
  service = VASMService.__new__(VASMService)
  service.lock = threading.Lock()
  service.vasm_gate = {}
  service.sm = messaging.SubMaster(["carState", "modelV2"])
  return service


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


def test_vasm_gate_requires_speed_direction_and_target_lane_width(monkeypatch, gate_service, message_transport):
  service = gate_service
  car_state = {"vEgo": 20.0}
  model_meta = {"laneChangeDirection": "left", "laneWidthLeft": 3.2, "laneWidthRight": 2.8}
  now = 10.0
  monkeypatch.setattr(v_asm_server.time, "monotonic", lambda: now)

  def update_gate():
    nonlocal now
    now += 0.05
    message_transport.publish("carState", **car_state)
    message_transport.publish("modelV2", meta=model_meta)
    return service._update_vasm_gate()

  assert update_gate() == (True, "left")

  model_meta["laneChangeDirection"] = "right"
  assert update_gate() == (False, "right")
  assert service.vasm_gate["reason"] == "target lane width below 3.0 m"

  car_state["vEgo"] = 5.0
  assert update_gate() == (False, "")

  car_state["vEgo"] = 20.0
  model_meta["laneChangeDirection"] = "none"
  assert update_gate() == (False, "")

  model_meta["laneChangeDirection"] = "right"
  model_meta["laneWidthRight"] = 3.0
  # Test either side of the limits without depending on Float32 rounding at equality.
  for speed, expected in ((29.9, False), (30.1, True), (119.9, True), (120.1, False)):
    car_state["vEgo"] = speed / 3.6
    assert update_gate()[0] is expected


@pytest.mark.parametrize("service_name", ["carState", "modelV2"])
@pytest.mark.parametrize("failure", ["missing", "invalid", "stale"])
def test_vasm_gate_rejects_unavailable_messages(monkeypatch, gate_service, message_transport, service_name, failure):
  service = gate_service
  monkeypatch.setattr(v_asm_server.time, "monotonic", lambda: 10.0)
  data = {"carState": {"vEgo": 20.0}, "modelV2": {"meta": {"laneChangeDirection": "left", "laneWidthLeft": 3.2}}}
  for name, fields in data.items():
    if failure != "missing" or name != service_name:
      message_transport.publish(name, valid=not (failure == "invalid" and name == service_name), **fields)
  if failure == "stale":
    assert service._update_vasm_gate() == (True, "left")
    monkeypatch.setattr(v_asm_server.time, "monotonic", lambda: 11.0)
    for name, fields in data.items():
      if name != service_name:
        message_transport.publish(name, **fields)

  assert service._update_vasm_gate() == (False, "")
  assert service.vasm_gate["reason"] == "carState or modelV2 is unavailable"


@pytest.mark.parametrize("age", [-1, 0, XIAOGE_BLINDSPOT_TIMEOUT_NS, XIAOGE_BLINDSPOT_TIMEOUT_NS + 1, XIAOGE_LANE_TIMEOUT_NS + 1])
def test_lane_and_blindspot_expire_independently(age):
  received = 1_000_000_000
  result = XiaogeVisionResult(0, 1, True, received, True, True, True, received)
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14, leftBlindspot=False, rightBlindspot=True)

  apply_xiaoge_vision_result(state, result, received + age)

  lanes_fresh = 0 <= age <= XIAOGE_LANE_TIMEOUT_NS
  assert (state.leftLaneLine, state.rightLaneLine) == ((20, 11) if lanes_fresh else (24, 14))
  assert state.leftBlindspot is (0 <= age <= XIAOGE_BLINDSPOT_TIMEOUT_NS)
  assert state.rightBlindspot  # OEM state survives even an expired visual result.


@pytest.mark.parametrize("overrides", [
  {"lane_valid": False, "blindspot_valid": False},
  {"lane_received_nanos": 0, "blindspot_received_nanos": 0},
  {"left_lane": -1, "right_lane": -1, "left_blindspot": False, "right_blindspot": False},
])
def test_invalid_and_unknown_results_preserve_vehicle_state(overrides):
  state = SimpleNamespace(leftLaneLine=24, rightLaneLine=14, leftBlindspot=True, rightBlindspot=False)
  result = replace(XiaogeVisionResult(0, 1, True, 1, True, True, True, 1), **overrides)

  assert not apply_xiaoge_vision_result(state, result, 2)
  assert (state.leftLaneLine, state.rightLaneLine, state.leftBlindspot, state.rightBlindspot) == (24, 14, True, False)


def test_default_polygons_can_be_saved_without_editing():
  assert v_asm_server.normalize_config(v_asm_server.DEFAULT_POLYGONS) == v_asm_server.DEFAULT_POLYGONS


def test_lane_snapshot_center_crops_stride_padded_frame():
  width, height, stride = 8, 4, 10
  rows = np.full((height + height // 2 + 2, stride), 255, dtype=np.uint8)
  rows[:height, :width] = np.arange(width, dtype=np.uint8) * 30

  jpeg = VASMService._lane_jpeg_from_nv12(rows.tobytes(), width, height, stride)

  with Image.open(BytesIO(jpeg)) as image:
    assert image.size == (416, 416)
    assert image.mode == "L"
    pixels = np.asarray(image)
  assert 55 <= float(pixels[:, 0].mean()) <= 70
  assert 145 <= float(pixels[:, -1].mean()) <= 160


def test_lane_snapshot_rejects_short_y_plane():
  with pytest.raises(ValueError, match="short NV12 Y plane"):
    VASMService._lane_jpeg_from_nv12(bytes(12), 4, 4, 4)


def test_vision_service_publishes_the_composite_payload(message_transport):
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
  service.pm = messaging.PubMaster(["customReservedRawData0"])

  service.publish_vision_result()

  sent = message_transport.sent
  assert len(sent) == 1
  assert sent[0][0] == "customReservedRawData0"
  event = messaging.log_from_bytes(sent[0][1])
  assert event.valid
  result = parse_xiaoge_vision_payload(event.customReservedRawData0)
  assert (result.left_lane, result.right_lane, result.left_blindspot, result.right_blindspot) == (0, 1, True, False)


@pytest.fixture
def vision_service(message_transport):
  service = VASMService(v_asm_server.DEFAULT_MODEL_PATH)
  assert service.inference.valid, service.inference.error
  assert service.lane_inference.valid, service.lane_inference.error
  message_transport.publish("carState", vEgo=20.0)
  message_transport.publish("modelV2", meta={"laneChangeDirection": "left", "laneWidthLeft": 3.2})
  service.sent = message_transport.sent
  return service


@pytest.mark.parametrize("stream", ["road", "wide"])
def test_snapshot_waits_for_camera_busy_longer_than_one_second(vision_service, stream):
  service = vision_service
  jpeg = b"fresh snapshot"
  with ThreadPoolExecutor(max_workers=1) as executor:
    pending = executor.submit(service.snapshot, stream)
    deadline = time.monotonic() + 1.0
    while service.snapshot_requests[stream] == 0 and time.monotonic() < deadline:
      time.sleep(0.01)
    assert service.snapshot_requests[stream] == 1
    # A camera worker can spend more than the old one-second timeout in inference.
    time.sleep(1.1)
    with service.snapshot_condition:
      if stream == "road":
        service.last_road_jpeg = jpeg
      else:
        service.last_jpeg = jpeg
      service.snapshot_responses[stream] = service.snapshot_requests[stream]
      service.snapshot_condition.notify_all()
    assert pending.result(timeout=1.0) == jpeg


@pytest.mark.parametrize("stream", ["road", "wide"])
def test_snapshot_timeout_does_not_report_old_image_as_refreshed(monkeypatch, vision_service, stream):
  monkeypatch.setattr(v_asm_server, "SNAPSHOT_TIMEOUT_SECONDS", 0.01)
  vision_service.last_road_jpeg = vision_service.last_jpeg = b"old snapshot"
  assert vision_service.snapshot(stream) is None


@pytest.fixture
def padded_camera_frame():
  width, height, stride = 1344, 760, 1408
  uv_offset = stride * 768
  raw = np.full(2_428_928, 255, dtype=np.uint8)
  raw[:stride * height].reshape(height, stride)[:, :width] = 80
  uv = raw[uv_offset:uv_offset + stride * (height // 2)].reshape(height // 2, stride)
  uv[:, :width:2] = 90
  uv[:, 1:width:2] = 240
  return SimpleNamespace(data=memoryview(raw), width=width, height=height, stride=stride, uv_offset=uv_offset)


@pytest.mark.parametrize("stream", ["road", "wide"])
@pytest.mark.parametrize("initial_disconnect", [False, True])
def test_camera_loop_infers_and_snapshots_padded_allocation(monkeypatch, vision_service, padded_camera_frame, stream, initial_disconnect):
  service = vision_service
  frame = padded_camera_frame
  connect_attempts = 0

  class FakeClient:
    delivered = False

    def __init__(self, *_args):
      self.connected = False

    def is_connected(self):
      return self.connected

    def connect(self, _blocking):
      nonlocal connect_attempts
      connect_attempts += 1
      self.connected = connect_attempts > int(initial_disconnect)
      return self.connected

    def recv(self, timeout_ms):
      assert timeout_ms == 1000
      if self.delivered:
        service.running = False
        return None
      self.delivered = True
      return frame

  visionipc = ModuleType("msgq.visionipc")
  visionipc.VisionIpcClient = FakeClient
  visionipc.VisionStreamType = SimpleNamespace(VISION_STREAM_ROAD=0, VISION_STREAM_WIDE_ROAD=1)
  monkeypatch.setitem(sys.modules, "msgq.visionipc", visionipc)

  def retry_sleep(_seconds):
    # Permit the initial offroad connection failure, but not an inference error.
    assert initial_disconnect and connect_attempts == 1
    error = service.lane_camera_error if stream == "road" else service.camera_error
    assert "camera is unavailable" in error

  monkeypatch.setattr(v_asm_server.time, "sleep", retry_sleep)
  service.snapshot_requests[stream] = 1

  if stream == "road":
    service.run_road_camera()
    assert service.lane_inference_count == 1
    assert service.lane_result["valid"], service.lane_result["error"]
    assert service.status()["lane"]["resultFresh"]
    assert service.lane_camera_error == ""
    jpeg = service.last_road_jpeg
  else:
    service.run_camera()
    assert service.inference_count == 1
    assert service.status()["vehicleSide"]["left"]["valid"]
    assert not service.status()["vehicleSide"]["right"]["valid"]
    assert service.camera_error == ""
    jpeg = service.last_jpeg
  assert len(service.sent) == 1
  event = messaging.log_from_bytes(service.sent[0][1])
  assert event.valid
  parse_xiaoge_vision_payload(event.customReservedRawData0)
  with Image.open(BytesIO(jpeg)) as image:
    assert image.size == ((416, 416) if stream == "road" else (frame.width, frame.height))
    pixels = np.asarray(image)
    if stream == "road":
      assert float(pixels.mean()) == pytest.approx(80, abs=2)
    else:
      # The UV plane encodes red; interpreting padded Y rows as UV changes it.
      assert float(pixels[:, :, 0].mean()) > 240
      assert float(pixels[:, :, 1:].mean()) < 30


def test_initial_status_does_not_report_clear_blindspots(vision_service):
  status = vision_service.status()
  assert not status["camera"]["available"]
  assert not status["lane"]["resultFresh"]
  assert not status["vehicleSide"]["left"]["valid"]
  assert not status["vehicleSide"]["right"]["valid"]


@pytest.mark.parametrize("failure", ["camera_error", "camera_stale", "inference_stale", "gate_closed", "side_changed"])
def test_status_invalidates_results_after_camera_or_inference_stops(monkeypatch, vision_service, failure):
  service = vision_service
  monkeypatch.setattr(v_asm_server.time, "monotonic", lambda: 10.0)
  monkeypatch.setattr(v_asm_server.time, "monotonic_ns", lambda: 10_000_000_000)
  service.last_frame_at = service.last_road_frame_at = 10.0
  service.camera_error = service.lane_camera_error = ""
  service.vasm_gate = {"active": True, "side": "left"}
  service.vasm_result = {"left": False, "right": False, "side": "left", "updatedMonoTimeNanos": 10_000_000_000}
  service.lane_result.update(valid=True, leftLine=1, rightLine=0, updatedMonoTimeNanos=10_000_000_000)
  assert service.status()["vehicleSide"]["left"]["valid"]
  assert service.status()["lane"]["resultFresh"]

  if failure == "camera_error":
    service.camera_error = service.lane_camera_error = "invalid camera frame"
  elif failure == "camera_stale":
    service.last_frame_at = service.last_road_frame_at = 7.0
  elif failure == "inference_stale":
    service.vasm_result["updatedMonoTimeNanos"] = 8_000_000_000
    service.lane_result["updatedMonoTimeNanos"] = 5_000_000_000
  elif failure == "gate_closed":
    service.vasm_gate["active"] = False
  else:
    service.vasm_gate["side"] = "right"

  status = service.status()
  assert not status["vehicleSide"]["left"]["valid"]
  assert not status["vehicleSide"]["right"]["valid"]
  if failure in ("camera_error", "camera_stale", "inference_stale"):
    assert not status["lane"]["resultFresh"]
  if failure in ("camera_error", "camera_stale"):
    assert not status["camera"]["available"]
    assert not status["lane"]["cameraAvailable"]
