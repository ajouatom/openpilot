#!/usr/bin/env python3
"""Xiaoge vision service providing lane and gated blindspot results to car."""

import argparse
import json
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from PIL import Image

if __package__ in (None, ""):
  sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.selfdrive.carrot.xiaoge.lane_inference import DEFAULT_LANE_MODEL_PATH, LaneInference
from openpilot.selfdrive.carrot.xiaoge.v_asm_inference import DEFAULT_MODEL_PATH, VASMInference

try:
  import cv2
  cv2.setNumThreads(1)
except ModuleNotFoundError as error:
  if error.name == "cv2":
    raise SystemExit(
      "V-ASM requires OpenCV. Activate the openpilot virtual environment, then run " +
      "`python -m pip install --no-cache-dir -r openpilot/selfdrive/carrot/xiaoge/requirements.txt`."
    ) from None
  raise


HOST = "0.0.0.0"
PORT = 8082
CONFIG_PATH = Path(__file__).resolve().parent / "v_asm_config.json"
MIN_THRESHOLD = 0.25
MAX_THRESHOLD = 1.0
MIN_SMOOTHING_SECONDS = 0.1
MAX_SMOOTHING_SECONDS = 0.5
MIN_BASE_INTERVAL_SECONDS = 0.05
MAX_BASE_INTERVAL_SECONDS = 1.0
BASE_INTERVAL_SECONDS = 0.25
FOLLOWUP_INTERVAL_SECONDS = 0.15
FOLLOWUP_WINDOW_SECONDS = 1.5
VASM_MIN_SPEED_MPS = 30.0 / 3.6
VASM_MAX_SPEED_MPS = 120.0 / 3.6
VASM_MIN_LANE_WIDTH_METERS = 3.0


DEFAULT_POLYGONS = {
  "width": 1928,
  "height": 1208,
  "poly_left": [
    [0, 550],
    [550, 480],
    [650, 950],
    [0, 1200],
  ],
  "poly_right": [
    [1378, 480],
    [1928, 550],
    [1928, 1200],
    [1278, 950],
  ],
}


def normalize_config(config: object) -> dict:
  if not isinstance(config, dict):
    raise ValueError("configuration must be a JSON object")
  try:
    width, height = int(config.get("width", 0)), int(config.get("height", 0))
  except (TypeError, ValueError) as error:
    raise ValueError("camera dimensions must be integers") from error
  if not 1 <= width <= 8192 or not 1 <= height <= 8192:
    raise ValueError("camera dimensions must be between 1 and 8192")

  normalized = {"width": width, "height": height}
  for side in ("left", "right"):
    polygon = config.get(f"poly_{side}", [])
    if not isinstance(polygon, list) or len(polygon) > 64:
      raise ValueError(f"poly_{side} must contain at most 64 points")
    if polygon and len(polygon) < 3:
      raise ValueError(f"poly_{side} requires at least 3 points")
    points = []
    for point in polygon:
      if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise ValueError(f"poly_{side} points must be [x, y]")
      try:
        x, y = round(float(point[0])), round(float(point[1]))
      except (TypeError, ValueError) as error:
        raise ValueError(f"poly_{side} coordinates must be numbers") from error
      if not 0 <= x < width or not 0 <= y < height:
        raise ValueError(f"poly_{side} point is outside the image")
      points.append([x, y])
    normalized[f"poly_{side}"] = points
  if not normalized["poly_left"] and not normalized["poly_right"]:
    raise ValueError("annotate at least one side")
  return normalized


class VASMService:
  def __init__(self, model_path: Path):
    self.lock = threading.Lock()
    self.snapshot_condition = threading.Condition(self.lock)
    self.snapshot_requests = {"wide": 0, "road": 0}
    self.snapshot_responses = {"wide": 0, "road": 0}
    self.vasm_inference_lock = threading.Lock()
    self.publish_lock = threading.Lock()
    self.inference = VASMInference(model_path)
    self.inference.load()
    self.config = self._read_config()
    if self.config:
      self.inference.load_config(self.config)
    self.threshold = 0.45
    self.smoothing_seconds = 0.2
    self.base_interval_seconds = BASE_INTERVAL_SECONDS
    self.running = True
    self.last_jpeg: bytes | None = None
    self.last_frame_at = 0.0
    self.last_inference_at = 0.0
    self.last_inference_ms = 0.0
    self.inference_count = 0
    self.inference_fps = 0.0
    self._fps_window_start = time.monotonic()
    self._fps_window_count = 0
    self.last_side_at = {"left": 0.0, "right": 0.0}
    self.next_side = "left"
    self.followup_until = 0.0
    self.camera_error = "waiting for wide road camera"
    self.sm = messaging.SubMaster(["carState", "modelV2"])
    self.pm = messaging.PubMaster(["customReservedRawData0"])
    self.vasm_gate = {
      "active": False,
      "side": "",
      "reason": "waiting for carState and modelV2",
      "laneWidth": 0.0,
    }
    self.vasm_result = {"left": False, "right": False, "updatedMonoTimeNanos": 0}

    # Lane inference engine setup
    self.lane_inference = LaneInference(DEFAULT_LANE_MODEL_PATH)
    self.lane_inference.load()
    self.lane_threshold = 0.25
    self.lane_interval_seconds = 0.40
    self.lane_camera_error = "waiting for road camera"
    self.last_road_jpeg: bytes | None = None
    self.last_road_frame_at = 0.0
    self.last_lane_inference_at = 0.0
    self.last_lane_inference_ms = 0.0
    self.lane_inference_count = 0
    self.lane_inference_fps = 0.0
    self._lane_fps_window_start = time.monotonic()
    self._lane_fps_window_count = 0
    self.lane_result = {
      "leftLine": -1,
      "rightLine": -1,
      "leftConf": 0.0,
      "rightConf": 0.0,
      "candidatesCount": 0,
      "valid": False,
      "error": "",
      "updatedMonoTimeNanos": 0,
    }

  def _read_config(self) -> dict:
    try:
      return normalize_config(json.loads(CONFIG_PATH.read_text()))
    except (OSError, ValueError, json.JSONDecodeError):
      return DEFAULT_POLYGONS

  def _write_config(self, config: dict) -> None:
    temporary_path = CONFIG_PATH.with_suffix(".tmp")
    temporary_path.write_text(json.dumps(config, separators=(",", ":")) + "\n")
    temporary_path.replace(CONFIG_PATH)

  def save_config(self, config: object) -> dict:
    normalized = normalize_config(config)
    self._write_config(normalized)
    with self.vasm_inference_lock:
      self.inference.load_config(normalized)
    with self.lock:
      self.config = normalized
    return normalized

  def clear_config(self) -> None:
    try:
      CONFIG_PATH.unlink()
    except FileNotFoundError:
      pass
    with self.vasm_inference_lock:
      self.inference.load_config(DEFAULT_POLYGONS)
    with self.lock:
      self.config = DEFAULT_POLYGONS

  def set_settings(self, settings: object) -> dict:
    if not isinstance(settings, dict):
      raise ValueError("settings must be a JSON object")
    try:
      threshold = float(settings.get("threshold", self.threshold))
      smoothing_seconds = float(settings.get("smoothingSeconds", self.smoothing_seconds))
      base_interval_seconds = float(settings.get("baseIntervalSeconds", self.base_interval_seconds))
      lane_threshold = float(settings.get("laneThreshold", self.lane_threshold))
      lane_interval_seconds = float(settings.get("laneIntervalSeconds", self.lane_interval_seconds))
    except (TypeError, ValueError) as error:
      raise ValueError("settings must be numeric") from error
    if not MIN_THRESHOLD <= threshold <= MAX_THRESHOLD:
      raise ValueError(f"threshold must be {MIN_THRESHOLD:.2f} to {MAX_THRESHOLD:.2f}")
    if not MIN_SMOOTHING_SECONDS <= smoothing_seconds <= MAX_SMOOTHING_SECONDS:
      raise ValueError(f"smoothingSeconds must be {MIN_SMOOTHING_SECONDS:.1f} to {MAX_SMOOTHING_SECONDS:.1f}")
    if not MIN_BASE_INTERVAL_SECONDS <= base_interval_seconds <= MAX_BASE_INTERVAL_SECONDS:
      raise ValueError(f"baseIntervalSeconds must be {MIN_BASE_INTERVAL_SECONDS:.2f} to {MAX_BASE_INTERVAL_SECONDS:.2f}")
    if not 0.05 <= lane_threshold <= 1.0:
      raise ValueError("laneThreshold must be 0.05 to 1.0")
    if not 0.05 <= lane_interval_seconds <= 2.0:
      raise ValueError("laneIntervalSeconds must be 0.05 to 2.0")

    with self.lock:
      self.threshold = threshold
      self.smoothing_seconds = smoothing_seconds
      self.base_interval_seconds = base_interval_seconds
      self.lane_threshold = lane_threshold
      self.lane_interval_seconds = lane_interval_seconds
    return self.status()

  def status(self) -> dict:
    with self.lock:
      return {
        "standalone": False,
        "integrated": True,
        "model": {
          "path": str(self.inference.model_path),
          "loaded": self.inference.valid,
          "error": self.inference.error,
        },
        "configured": bool(self.config),
        "configuredSides": list(self.inference.configured_sides),
        "gate": self.vasm_gate,
        "threshold": self.threshold,
        "smoothingSeconds": self.smoothing_seconds,
        "baseIntervalSeconds": self.base_interval_seconds,
        "camera": {
          "available": bool(self.last_frame_at),
          "error": self.camera_error,
          "lastFrameAgeSeconds": time.monotonic() - self.last_frame_at if self.last_frame_at else None,
        },
        "imageSide": {
          side: {"active": self.inference.active[side], "confidence": self.inference.confidence[side]}
          for side in ("left", "right")
        },
        "vehicleSide": {
          side: {"active": self.inference.active[side], "confidence": self.inference.confidence[side]}
          for side in ("left", "right")
        },
        "inference": {
          "latencyMs": round(self.last_inference_ms, 1),
          "fps": self.inference_fps,
          "count": self.inference_count,
          "lastAgeSeconds": round(time.monotonic() - self.last_inference_at, 2) if self.last_inference_at else None,
        },
        "lastInferenceAgeSeconds": time.monotonic() - self.last_inference_at if self.last_inference_at else None,
        "lane": {
          "enabled": True,
          "loaded": self.lane_inference.valid,
          "error": self.lane_inference.error,
          "threshold": self.lane_threshold,
          "intervalSeconds": self.lane_interval_seconds,
          "cameraAvailable": bool(self.last_road_frame_at),
          "cameraError": self.lane_camera_error,
          "result": self.lane_result,
          "inference": {
            "latencyMs": round(self.last_lane_inference_ms, 1),
            "fps": self.lane_inference_fps,
            "count": self.lane_inference_count,
            "lastAgeSeconds": round(time.monotonic() - self.last_lane_inference_at, 2) if self.last_lane_inference_at else None,
          },
        },
      }

  def snapshot(self, stream_type: str) -> bytes | None:
    with self.snapshot_condition:
      self.snapshot_requests[stream_type] += 1
      requested = self.snapshot_requests[stream_type]
      self.snapshot_condition.wait_for(
        lambda: self.snapshot_responses[stream_type] >= requested or not self.running,
        timeout=1.0,
      )
      return self.last_road_jpeg if stream_type == "road" else self.last_jpeg

  def _update_vasm_gate(self) -> tuple[bool, str]:
    self.sm.update(0)
    if not self.sm.valid["carState"] or not self.sm.valid["modelV2"]:
      gate = {"active": False, "side": "", "reason": "carState or modelV2 is unavailable", "laneWidth": 0.0}
    else:
      speed = float(self.sm["carState"].vEgo)
      direction = self.sm["modelV2"].meta.laneChangeDirection
      if speed < VASM_MIN_SPEED_MPS or speed > VASM_MAX_SPEED_MPS:
        gate = {"active": False, "side": "", "reason": "speed outside 30-120 km/h", "laneWidth": 0.0}
      elif direction == log.LaneChangeDirection.left:
        width = float(self.sm["modelV2"].meta.laneWidthLeft)
        gate = {"active": width >= VASM_MIN_LANE_WIDTH_METERS, "side": "left", "reason": "", "laneWidth": width}
      elif direction == log.LaneChangeDirection.right:
        width = float(self.sm["modelV2"].meta.laneWidthRight)
        gate = {"active": width >= VASM_MIN_LANE_WIDTH_METERS, "side": "right", "reason": "", "laneWidth": width}
      else:
        gate = {"active": False, "side": "", "reason": "no lane-change direction", "laneWidth": 0.0}
      if gate["side"] and not gate["active"]:
        gate["reason"] = "target lane width below 3.0 m"
    with self.lock:
      self.vasm_gate = gate
    return bool(gate["active"]), str(gate["side"])

  def publish_vision_result(self) -> None:
    with self.lock:
      lane = dict(self.lane_result)
      vasm = dict(self.vasm_result)
    now_nanos = time.monotonic_ns()
    payload = json.dumps({
      "type": "xiaogeVision",
      "version": 1,
      "lane": {
        "leftLine": lane["leftLine"],
        "rightLine": lane["rightLine"],
        "valid": lane["valid"],
        "receivedMonoTimeNanos": lane["updatedMonoTimeNanos"] or now_nanos,
      },
      "blindspot": {
        "left": vasm["left"],
        "right": vasm["right"],
        "valid": True,
        "receivedMonoTimeNanos": vasm["updatedMonoTimeNanos"] or now_nanos,
      },
    }, separators=(",", ":")).encode()
    with self.publish_lock:
      msg = messaging.new_message("customReservedRawData0", size=len(payload), valid=True)
      msg.customReservedRawData0 = payload
      self.pm.send("customReservedRawData0", msg)

  @staticmethod
  def _jpeg_from_nv12(data: bytes, width: int, height: int, stride: int) -> bytes:
    frame = np.frombuffer(data, dtype=np.uint8).reshape((-1, stride))
    nv12 = frame[:height + height // 2, :width]
    rgb = cv2.cvtColor(nv12, cv2.COLOR_YUV2RGB_NV12)
    output = BytesIO()
    Image.fromarray(rgb).save(output, "JPEG", quality=85)
    return output.getvalue()

  @staticmethod
  def _lane_jpeg_from_nv12(data: bytes, width: int, height: int, stride: int) -> bytes:
    """Build the grayscale road-camera input expected by the lane model."""
    frame = np.frombuffer(data, dtype=np.uint8).reshape((-1, stride))
    y_plane = frame[:height, :width]
    crop_size = min(width, height)
    start_x = (width - crop_size) // 2
    start_y = (height - crop_size) // 2
    gray = np.ascontiguousarray(y_plane[start_y:start_y + crop_size, start_x:start_x + crop_size])
    gray = cv2.resize(gray, (416, 416), interpolation=cv2.INTER_LINEAR)
    output = BytesIO()
    Image.fromarray(gray).save(output, "JPEG", quality=50)
    return output.getvalue()

  def run_camera(self) -> None:
    from msgq.visionipc import VisionIpcClient, VisionStreamType

    client = None
    while self.running:
      try:
        if client is None or not client.is_connected():
          client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, True)
          if not client.connect(False):
            with self.lock:
              self.camera_error = "wide road camera is unavailable; start openpilot/camerad first"
            time.sleep(1.0)
            continue

        buffer = client.recv(timeout_ms=1000)
        if buffer is None:
          continue
        now = time.monotonic()
        frame = np.frombuffer(buffer.data, dtype=np.uint8).reshape((-1, client.stride))[:, :client.width]
        gate_active, side = self._update_vasm_gate()
        publish_clear = False
        with self.lock:
          self.last_frame_at = now
          self.camera_error = ""
          if self.snapshot_responses["wide"] < self.snapshot_requests["wide"]:
            self.last_jpeg = self._jpeg_from_nv12(buffer.data, client.width, client.height, client.stride)
            self.snapshot_responses["wide"] = self.snapshot_requests["wide"]
            self.snapshot_condition.notify_all()
          if not gate_active:
            publish_clear = self.vasm_result["left"] or self.vasm_result["right"]
            self.vasm_result = {"left": False, "right": False, "updatedMonoTimeNanos": time.monotonic_ns()}
          else:
            interval = FOLLOWUP_INTERVAL_SECONDS if now < self.followup_until else self.base_interval_seconds
            if now - self.last_inference_at < interval:
              continue
        if not gate_active:
          if publish_clear:
            self.publish_vision_result()
          continue
        with self.vasm_inference_lock:
          configured_sides = self.inference.configured_sides
          if side not in configured_sides or not self.inference.valid:
            continue
          previous = self.last_side_at[side]
          t0 = time.monotonic()
          self.inference.update(frame, client.width, client.height, side, self.threshold, self.smoothing_seconds, now - previous if previous else interval)
          t1 = time.monotonic()
          active = self.inference.active[side]
        with self.lock:
          self.last_inference_ms = (t1 - t0) * 1000.0
          self.inference_count += 1
          self._fps_window_count += 1
          if t1 - self._fps_window_start >= 1.0:
            self.inference_fps = round(self._fps_window_count / (t1 - self._fps_window_start), 1)
            self._fps_window_start = t1
            self._fps_window_count = 0
          self.last_inference_at = self.last_side_at[side] = now
          if active:
            self.followup_until = now + FOLLOWUP_WINDOW_SECONDS
          self.vasm_result = {
            "left": self.inference.active["left"] if side == "left" else False,
            "right": self.inference.active["right"] if side == "right" else False,
            "updatedMonoTimeNanos": time.monotonic_ns(),
          }
        self.publish_vision_result()
      except (OSError, ValueError, cv2.error) as error:
        with self.lock:
          self.camera_error = str(error)
        client = None
        time.sleep(1.0)

  def run_road_camera(self) -> None:
    from msgq.visionipc import VisionIpcClient, VisionStreamType

    client = None
    while self.running:
      try:
        if client is None or not client.is_connected():
          client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
          if not client.connect(False):
            with self.lock:
              self.lane_camera_error = "road camera is unavailable; start openpilot/camerad first"
            time.sleep(1.0)
            continue

        buffer = client.recv(timeout_ms=1000)
        if buffer is None:
          continue
        now = time.monotonic()
        frame = np.frombuffer(buffer.data, dtype=np.uint8).reshape((-1, client.stride))[:, :client.width]
        with self.lock:
          self.last_road_frame_at = now
          self.lane_camera_error = ""
          if self.snapshot_responses["road"] < self.snapshot_requests["road"]:
            self.last_road_jpeg = self._lane_jpeg_from_nv12(buffer.data, client.width, client.height, client.stride)
            self.snapshot_responses["road"] = self.snapshot_requests["road"]
            self.snapshot_condition.notify_all()
          if not self.lane_inference.valid:
            continue
          if now - self.last_lane_inference_at < self.lane_interval_seconds:
            continue

          lane_threshold = self.lane_threshold
        t0 = time.monotonic()
        res = self.lane_inference.infer(
          frame, client.width, client.height, conf_thresh=lane_threshold
        )
        t1 = time.monotonic()
        with self.lock:
          self.last_lane_inference_ms = (t1 - t0) * 1000.0
          self.lane_inference_count += 1
          self._lane_fps_window_count += 1
          if t1 - self._lane_fps_window_start >= 1.0:
            self.lane_inference_fps = round(self._lane_fps_window_count / (t1 - self._lane_fps_window_start), 1)
            self._lane_fps_window_start = t1
            self._lane_fps_window_count = 0

          self.last_lane_inference_at = now
          res["updatedMonoTimeNanos"] = time.monotonic_ns()
          self.lane_result = res
        self.publish_vision_result()
      except (OSError, ValueError, cv2.error) as error:
        with self.lock:
          self.lane_camera_error = str(error)
        client = None
        time.sleep(1.0)


class Handler(BaseHTTPRequestHandler):
  service: VASMService
  index_path = Path(__file__).resolve().parent / "v_asm_web.html"

  def handle_one_request(self) -> None:
    try:
      super().handle_one_request()
    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
      return

  def _json(self, status: HTTPStatus, payload: object) -> None:
    body = json.dumps(payload).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self) -> None:
    path = urlparse(self.path).path
    if path == "/":
      body = self.index_path.read_bytes()
      self.send_response(HTTPStatus.OK)
      self.send_header("Content-Type", "text/html; charset=utf-8")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)
    elif path == "/api/status":
      self._json(HTTPStatus.OK, self.service.status())
    elif path == "/api/config":
      self._json(HTTPStatus.OK, self.service.config)
    elif path == "/api/snapshot":
      parsed_url = urlparse(self.path)
      stream_type = "wide"
      if "stream=road" in parsed_url.query:
        stream_type = "road"
      jpeg = self.service.snapshot(stream_type)
      if jpeg is None:
        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": f"no {stream_type} camera frame available"})
        return
      self.send_response(HTTPStatus.OK)
      self.send_header("Content-Type", "image/jpeg")
      self.send_header("Content-Length", str(len(jpeg)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(jpeg)
    else:
      self.send_error(HTTPStatus.NOT_FOUND)

  def do_POST(self) -> None:
    path = urlparse(self.path).path
    try:
      length = int(self.headers.get("Content-Length", "0"))
      if not 0 < length <= 65536:
        raise ValueError("request body must be 1 to 65536 bytes")
      payload = json.loads(self.rfile.read(length))
      if path == "/api/config":
        self._json(HTTPStatus.OK, self.service.save_config(payload))
      elif path == "/api/settings":
        self._json(HTTPStatus.OK, self.service.set_settings(payload))
      else:
        self.send_error(HTTPStatus.NOT_FOUND)
    except (ValueError, json.JSONDecodeError) as error:
      self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})

  def do_DELETE(self) -> None:
    if urlparse(self.path).path != "/api/config":
      self.send_error(HTTPStatus.NOT_FOUND)
      return
    self.service.clear_config()
    self._json(HTTPStatus.OK, {"success": True})

  def log_message(self, _format: str, *_args) -> None:
    return


def main() -> None:
  parser = argparse.ArgumentParser(description="Xiaoge vision server")
  parser.add_argument("--host", default=HOST)
  parser.add_argument("--port", type=int, default=PORT)
  parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
  args = parser.parse_args()
  if not 1 <= args.port <= 65535:
    parser.error("port must be 1 through 65535")

  service, server = create_server(args.host, args.port, args.model)
  print(f"Xiaoge vision server: http://{args.host}:{args.port}")
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    pass
  finally:
    server.server_close()
    service.running = False


def create_server(host: str = HOST, port: int = PORT, model_path: Path = DEFAULT_MODEL_PATH) -> tuple[VASMService, ThreadingHTTPServer]:
  service = VASMService(model_path)
  Handler.service = service
  server = ThreadingHTTPServer((host, port), Handler)
  threading.Thread(target=service.run_camera, daemon=True).start()
  threading.Thread(target=service.run_road_camera, daemon=True).start()
  return service, server


if __name__ == "__main__":
  main()
