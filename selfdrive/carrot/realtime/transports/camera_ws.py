from __future__ import annotations

import asyncio
import json
import struct
import time
from typing import Any

from aiohttp import web


def _safe_int(value: Any) -> int | None:
  if value is None:
    return None
  try:
    return int(value)
  except Exception:
    return None


def _is_h264_keyframe(payload: bytes) -> bool:
  n = len(payload)
  i = 0
  while i + 5 < n:
    if payload[i] != 0 or payload[i + 1] != 0:
      i += 1
      continue
    if payload[i + 2] == 1:
      nal_start = i + 3
    elif payload[i + 2] == 0 and payload[i + 3] == 1:
      nal_start = i + 4
    else:
      i += 1
      continue
    if nal_start < n:
      nal_type = payload[nal_start] & 0x1F
      if nal_type == 5:
        return True
    i = nal_start
  return False


def _extract_h264_codec(payload: bytes) -> str | None:
  n = len(payload)
  i = 0
  while i + 6 < n:
    if payload[i] != 0 or payload[i + 1] != 0:
      i += 1
      continue
    if payload[i + 2] == 1:
      nal_start = i + 3
    elif payload[i + 2] == 0 and payload[i + 3] == 1:
      nal_start = i + 4
    else:
      i += 1
      continue
    if nal_start + 3 < n:
      nal_type = payload[nal_start] & 0x1F
      if nal_type == 7:
        profile = payload[nal_start + 1]
        constraints = payload[nal_start + 2]
        level = payload[nal_start + 3]
        return f"avc1.{profile:02X}{constraints:02X}{level:02X}"
    i = nal_start
  return None


class CameraWsHub:
  CAMERA_QUEUE_MAXSIZE = 8
  IDLE_STOP_SEC = 5.0
  CAMERA_SERVICE_CANDIDATES: dict[str, list[str]] = {
    "road": [
      "livestreamRoadEncodeData",
      "roadEncodeData",
    ],
  }
  CAMERA_STATE_SERVICES: dict[str, str] = {
    "road": "roadCameraState",
  }

  def __init__(self, messaging: Any) -> None:
    self.messaging = messaging
    self.clients: dict[str, set[web.WebSocketResponse]] = {
      camera: set() for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._producer_tasks: dict[str, asyncio.Task] = {}
    self._sender_tasks: dict[str, asyncio.Task] = {}
    self._sockets: dict[str, dict[str, Any]] = {
      camera: {} for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._queues: dict[str, asyncio.Queue[bytes]] = {
      camera: asyncio.Queue(maxsize=self.CAMERA_QUEUE_MAXSIZE)
      for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._sm_frame_id: dict[str, int] = {}
    self._last_codec: dict[str, str] = {
      camera: "" for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._last_frame_id: dict[str, int] = {
      camera: -1 for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._last_frame_at_mono: dict[str, float] = {
      camera: 0.0 for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._selected_service: dict[str, str] = {
      camera: "" for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._frame_count: dict[str, int] = {
      camera: 0 for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._queue_drop_count: dict[str, int] = {
      camera: 0 for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._send_drop_count: dict[str, int] = {
      camera: 0 for camera in self.CAMERA_SERVICE_CANDIDATES
    }
    self._ws_send_failures: dict[web.WebSocketResponse, int] = {}
    self._lock = asyncio.Lock()

  def client_count(self, camera: str | None = None) -> int:
    if camera is None:
      return sum(len(clients) for clients in self.clients.values())
    return len(self.clients.get(camera, set()))

  def has_clients(self, camera: str | None = None) -> bool:
    return self.client_count(camera) > 0

  async def ws_camera(self, request: web.Request) -> web.WebSocketResponse:
    camera = request.match_info.get("camera", "").strip()
    if camera not in self.CAMERA_SERVICE_CANDIDATES:
      raise web.HTTPNotFound(text=f"unknown camera: {camera}")

    ws = web.WebSocketResponse(heartbeat=20, max_msg_size=2 * 1024 * 1024, compress=False)
    await ws.prepare(request)
    try:
      await ws.send_str(json.dumps({
        "type": "hello",
        "camera": camera,
        "mode": "direct-encode-relay",
      }, separators=(",", ":")))
      self.clients[camera].add(ws)
      await self.ensure_camera_task(camera)
      async for _ in ws:
        pass
    finally:
      self.clients[camera].discard(ws)
      self._ws_send_failures.pop(ws, None)
      try:
        await ws.close()
      except Exception:
        pass
    return ws

  async def ensure_camera_task(self, camera: str) -> None:
    async with self._lock:
      producer = self._producer_tasks.get(camera)
      if producer is None or producer.done():
        self._producer_tasks[camera] = asyncio.create_task(self._camera_producer_loop(camera))
      sender = self._sender_tasks.get(camera)
      if sender is None or sender.done():
        self._sender_tasks[camera] = asyncio.create_task(self._camera_sender_loop(camera))

  async def stop_all(self) -> None:
    async with self._lock:
      tasks = list(self._producer_tasks.values()) + list(self._sender_tasks.values())
      self._producer_tasks = {}
      self._sender_tasks = {}
    for task in tasks:
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    for camera_clients in self.clients.values():
      for ws in tuple(camera_clients):
        try:
          await ws.close()
        except Exception:
          pass
      camera_clients.clear()
    self._ws_send_failures.clear()

  def status(self) -> dict[str, Any]:
    cameras: dict[str, Any] = {}
    for camera in self.CAMERA_SERVICE_CANDIDATES:
      last_frame_at = self._last_frame_at_mono.get(camera, 0.0)
      cameras[camera] = {
        "clients": len(self.clients.get(camera, set())),
        "frames": self._frame_count.get(camera, 0),
        "queue": self._queues[camera].qsize(),
        "queueDrops": self._queue_drop_count.get(camera, 0),
        "sendDrops": self._send_drop_count.get(camera, 0),
        "lastFrameId": self._last_frame_id.get(camera, -1),
        "lastFrameAgeMs": max(0, int((time.monotonic() - last_frame_at) * 1000.0)) if last_frame_at > 0 else None,
        "service": self._selected_service.get(camera, ""),
        "codec": self._last_codec.get(camera, ""),
      }
    return {
      "mode": "direct-encode-relay",
      "cameras": cameras,
    }

  async def _camera_producer_loop(self, camera: str) -> None:
    queue = self._queues[camera]
    idle_started_at = 0.0
    while True:
      try:
        if not self.clients.get(camera):
          if idle_started_at <= 0.0:
            idle_started_at = time.monotonic()
          elif (time.monotonic() - idle_started_at) >= self.IDLE_STOP_SEC:
            break
          await asyncio.sleep(0.03)
          continue
        idle_started_at = 0.0

        await self._refresh_ready_frame_id(camera)
        ready_frame_id = self._sm_frame_id.get(camera)
        if ready_frame_id is None or ready_frame_id <= 0:
          await asyncio.sleep(0.03)
          continue

        msg = None
        source_service = ""
        for service in self.CAMERA_SERVICE_CANDIDATES[camera]:
          sock = await self._get_camera_socket(camera, service)
          if sock is None:
            continue
          try:
            candidate = self.messaging.recv_one_or_none(sock)
          except Exception:
            continue
          if candidate is not None:
            msg = candidate
            source_service = service
            break

        if msg is None:
          await asyncio.sleep(0.002)
          continue

        which = ""
        try:
          which = msg.which()
        except Exception:
          await asyncio.sleep(0.001)
          continue
        if not which:
          await asyncio.sleep(0.001)
          continue

        frame = getattr(msg, which, None)
        if frame is None:
          await asyncio.sleep(0.001)
          continue

        frame_sample = self._build_frame_sample(frame, service=source_service, which=which)
        frame_id = _safe_int(frame_sample.get("frameId"))
        if frame_id is None or frame_id < 0:
          frame_id = ready_frame_id
          frame_sample["frameId"] = frame_id

        packet = self._pack_frame(camera, frame, frame_sample)
        if queue.full():
          try:
            queue.get_nowait()
            self._queue_drop_count[camera] += 1
          except Exception:
            pass
        try:
          queue.put_nowait(packet)
        except Exception:
          await asyncio.sleep(0.001)
          continue

        self._frame_count[camera] += 1
        self._last_frame_at_mono[camera] = time.monotonic()
        if frame_id is not None and frame_id >= 0:
          self._last_frame_id[camera] = frame_id
        if source_service:
          self._selected_service[camera] = source_service
      except asyncio.CancelledError:
        break
      except Exception:
        await asyncio.sleep(0.01)
    async with self._lock:
      current = self._producer_tasks.get(camera)
      if current is asyncio.current_task():
        self._producer_tasks.pop(camera, None)
    self._sockets[camera] = {}
    self._sm_frame_id.pop(camera, None)

  async def _camera_sender_loop(self, camera: str) -> None:
    queue = self._queues[camera]
    idle_started_at = 0.0
    while True:
      try:
        if not self.clients.get(camera):
          if idle_started_at <= 0.0:
            idle_started_at = time.monotonic()
          elif (time.monotonic() - idle_started_at) >= self.IDLE_STOP_SEC:
            break
          while queue.qsize() > 1:
            try:
              queue.get_nowait()
              self._queue_drop_count[camera] += 1
            except Exception:
              break
          await asyncio.sleep(0.03)
          continue
        idle_started_at = 0.0

        try:
          packet = await asyncio.wait_for(queue.get(), timeout=0.25)
        except asyncio.TimeoutError:
          continue

        while queue.qsize() > 1:
          try:
            packet = queue.get_nowait()
            self._queue_drop_count[camera] += 1
          except Exception:
            break

        stale: list[web.WebSocketResponse] = []
        clients = list(self.clients.get(camera, set()))
        results = await asyncio.gather(
          *[asyncio.wait_for(ws.send_bytes(packet), timeout=0.35) for ws in clients],
          return_exceptions=True,
        )
        for ws, result in zip(clients, results):
          if not isinstance(result, Exception):
            self._ws_send_failures.pop(ws, None)
            continue
          fail_count = self._ws_send_failures.get(ws, 0) + 1
          self._ws_send_failures[ws] = fail_count
          if fail_count >= 4:
            stale.append(ws)
            self._send_drop_count[camera] += 1

        for ws in stale:
          self.clients[camera].discard(ws)
          self._ws_send_failures.pop(ws, None)
          try:
            await ws.close(code=1011, message=b"camera_send_timeout")
          except Exception:
            pass
      except asyncio.CancelledError:
        break
      except Exception:
        await asyncio.sleep(0.01)
    async with self._lock:
      current = self._sender_tasks.get(camera)
      if current is asyncio.current_task():
        self._sender_tasks.pop(camera, None)

  async def _get_camera_socket(self, camera: str, service: str) -> Any:
    camera_sockets = self._sockets.get(camera)
    if camera_sockets is None:
      camera_sockets = {}
      self._sockets[camera] = camera_sockets
    existing = camera_sockets.get(service)
    if existing is not None:
      return existing
    try:
      sock = self.messaging.sub_sock(service, conflate=True)
      camera_sockets[service] = sock
      return sock
    except Exception:
      return None

  async def _refresh_ready_frame_id(self, camera: str) -> None:
    state_service = self.CAMERA_STATE_SERVICES.get(camera)
    if not state_service:
      return
    sock = await self._get_camera_socket(camera, state_service)
    if sock is None:
      return
    try:
      msg = self.messaging.recv_one_or_none(sock)
    except Exception:
      return
    if msg is None:
      return
    try:
      which = msg.which()
    except Exception:
      return
    state = getattr(msg, which, None)
    frame_id = _safe_int(getattr(state, "frameId", None)) if state is not None else None
    if frame_id is not None and frame_id > 0:
      self._sm_frame_id[camera] = frame_id

  def _build_frame_sample(self, frame: Any, *, service: str = "", which: str = "") -> dict[str, Any]:
    frame_id = _safe_int(getattr(frame, "frameId", None))
    sof = _safe_int(getattr(frame, "timestampSof", None))
    eof = _safe_int(getattr(frame, "timestampEof", None))
    width = _safe_int(getattr(frame, "width", None))
    height = _safe_int(getattr(frame, "height", None))
    flags = None
    encode_id = None
    segment_id = None
    frame_type = None
    try:
      idx = getattr(frame, "idx", None)
      if idx is not None:
        flags = _safe_int(getattr(idx, "flags", None))
        encode_id = _safe_int(getattr(idx, "encodeId", None))
        segment_id = _safe_int(getattr(idx, "segmentId", None))
        frame_type = str(getattr(idx, "type", ""))
        if frame_id is None:
          frame_id = _safe_int(getattr(idx, "frameId", None))
    except Exception:
      pass
    return {
      "service": service,
      "which": which,
      "frameId": frame_id,
      "timestampSof": sof,
      "timestampEof": eof,
      "width": width,
      "height": height,
      "flags": flags,
      "encodeId": encode_id,
      "segmentId": segment_id,
      "frameType": frame_type,
    }

  def _pack_frame(self, camera: str, frame: Any, frame_sample: dict[str, Any]) -> bytes:
    header_raw = bytes(getattr(frame, "header", b"") or b"")
    data_raw = bytes(getattr(frame, "data", b"") or b"")
    payload_len = len(header_raw) + len(data_raw)

    codec = self._last_codec[camera]
    if not codec:
      parsed_codec = _extract_h264_codec(header_raw + data_raw if payload_len < 4096 else header_raw[:256])
      if parsed_codec:
        self._last_codec[camera] = parsed_codec
        codec = parsed_codec

    flags = _safe_int(frame_sample.get("flags"))
    is_key = bool(flags is not None and flags & 0x8)
    if not is_key and payload_len > 0:
      is_key = _is_h264_keyframe(header_raw) if header_raw else _is_h264_keyframe(data_raw[:64])
    meta = {
      "camera": camera,
      "codec": codec or "avc1.640028",
      "frameId": _safe_int(frame_sample.get("frameId")),
      "width": _safe_int(frame_sample.get("width")),
      "height": _safe_int(frame_sample.get("height")),
      "flags": flags,
      "encodeId": _safe_int(frame_sample.get("encodeId")),
      "segmentId": _safe_int(frame_sample.get("segmentId")),
      "frameType": frame_sample.get("frameType"),
      "timestampSof": _safe_int(frame_sample.get("timestampSof")),
      "timestampEof": _safe_int(frame_sample.get("timestampEof")),
      "keyFrame": is_key,
      "size": payload_len,
      "ts": time.time(),
    }
    meta_bytes = json.dumps(meta, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    buf = bytearray(4 + len(meta_bytes) + payload_len)
    struct.pack_into(">I", buf, 0, len(meta_bytes))
    buf[4:4 + len(meta_bytes)] = meta_bytes
    off = 4 + len(meta_bytes)
    buf[off:off + len(header_raw)] = header_raw
    off += len(header_raw)
    buf[off:off + len(data_raw)] = data_raw
    return bytes(buf)
