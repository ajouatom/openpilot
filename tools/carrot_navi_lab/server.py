from __future__ import annotations

import argparse
import asyncio
from collections import deque
from dataclasses import dataclass
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import time
from typing import Any

from aiohttp import ClientSession, ClientWebSocketResponse, WSMsgType, web
import av


LAB_DIR = Path(__file__).resolve().parent
ROOT = LAB_DIR.parents[1]
PROTOCOL_PATH = ROOT / "openpilot/selfdrive/carrot/server/features/carrot_navi/protocol.py"
WORKER_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_worker.js"
OVERLAY_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_overlay.js"
OVERLAY_PRIMITIVES_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_overlay_primitives.js"
OVERLAY_STORE_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_overlay_store.js"
MSE_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_mse.js"
COMPOSITOR_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_compositor.js"
SPLIT_PATH = ROOT / "openpilot/selfdrive/carrot/web/js/realtime/carrot_navi_split.js"
SPLIT_CSS_PATH = ROOT / "openpilot/selfdrive/carrot/web/css/pages/carrot_navi.css"
MEDIA_HEADER = struct.Struct(">4sBI")
H264_KEYFRAME_FLAG = 1
CLIENT_QUEUE_SIZE = 48
BRIDGE_KEY: web.AppKey["ClusterMediaPreviewBridge"] = web.AppKey("carrot_navi_lab_bridge", object)


def _load_protocol_module():
  spec = importlib.util.spec_from_file_location("carrot_navi_lab_production_protocol", PROTOCOL_PATH)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load production protocol: {PROTOCOL_PATH}")
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  return module


encode_media_frame = _load_protocol_module().encode_media_frame


def parse_media_frame(wire: bytes) -> tuple[dict[str, Any], bytes]:
  if len(wire) < MEDIA_HEADER.size:
    raise ValueError("short Carrot Navi media frame")
  magic, version, header_size = MEDIA_HEADER.unpack_from(wire)
  if magic != b"CNWB" or version != 1:
    raise ValueError("invalid Carrot Navi media frame")
  payload_offset = MEDIA_HEADER.size + header_size
  if header_size <= 0 or payload_offset > len(wire):
    raise ValueError("invalid Carrot Navi media header")
  metadata = json.loads(wire[MEDIA_HEADER.size:payload_offset])
  if not isinstance(metadata, dict):
    raise ValueError("Carrot Navi media metadata is not an object")
  return metadata, wire[payload_offset:]


def _box_payload(box: bytes) -> bytes:
  if len(box) < 8:
    raise ValueError("short MP4 box")
  size = struct.unpack_from(">I", box)[0]
  header_size = 8
  if size == 1:
    if len(box) < 16:
      raise ValueError("short extended MP4 box")
    size = struct.unpack_from(">Q", box, 8)[0]
    header_size = 16
  elif size == 0:
    size = len(box)
  if size < header_size or size > len(box):
    raise ValueError("invalid MP4 box size")
  return box[header_size:size]


def _top_level_boxes(payload: bytes) -> tuple[tuple[bytes, bytes], ...]:
  boxes: list[tuple[bytes, bytes]] = []
  offset = 0
  while offset < len(payload):
    if offset + 8 > len(payload):
      raise ValueError("incomplete MP4 box")
    size = struct.unpack_from(">I", payload, offset)[0]
    header_size = 8
    if size == 1:
      if offset + 16 > len(payload):
        raise ValueError("incomplete extended MP4 box")
      size = struct.unpack_from(">Q", payload, offset + 8)[0]
      header_size = 16
    elif size == 0:
      size = len(payload) - offset
    if size < header_size or offset + size > len(payload):
      raise ValueError("invalid MP4 box boundary")
    box = payload[offset:offset + size]
    boxes.append((box[4:8], box))
    offset += size
  return tuple(boxes)


def avcc_configuration(initialization: bytes) -> tuple[bytes, str, int]:
  marker = initialization.find(b"avcC")
  if marker < 4:
    raise ValueError("fMP4 initialization has no avcC box")
  size = struct.unpack_from(">I", initialization, marker - 4)[0]
  box_start = marker - 4
  if size < 13 or box_start + size > len(initialization):
    raise ValueError("invalid avcC box")
  data = initialization[marker + 4:box_start + size]
  if len(data) < 7 or data[0] != 1:
    raise ValueError("unsupported AVCDecoderConfigurationRecord")
  codec = f"avc1.{data[1]:02X}{data[2]:02X}{data[3]:02X}"
  length_size = (data[4] & 0x03) + 1
  offset = 5
  units: list[bytes] = []
  sps_count = data[offset] & 0x1F
  offset += 1
  for _ in range(sps_count):
    if offset + 2 > len(data):
      raise ValueError("truncated avcC SPS length")
    unit_size = struct.unpack_from(">H", data, offset)[0]
    offset += 2
    if offset + unit_size > len(data):
      raise ValueError("truncated avcC SPS")
    units.append(b"\x00\x00\x00\x01" + data[offset:offset + unit_size])
    offset += unit_size
  if offset >= len(data):
    raise ValueError("avcC has no PPS count")
  pps_count = data[offset]
  offset += 1
  for _ in range(pps_count):
    if offset + 2 > len(data):
      raise ValueError("truncated avcC PPS length")
    unit_size = struct.unpack_from(">H", data, offset)[0]
    offset += 2
    if offset + unit_size > len(data):
      raise ValueError("truncated avcC PPS")
    units.append(b"\x00\x00\x00\x01" + data[offset:offset + unit_size])
    offset += unit_size
  if not units:
    raise ValueError("avcC contains no parameter sets")
  return b"".join(units), codec, length_size


def fmp4_sample_to_annex_b(fragment: bytes, length_size: int) -> bytes:
  mdat = next((_box_payload(box) for kind, box in _top_level_boxes(fragment) if kind == b"mdat"), b"")
  if not mdat:
    raise ValueError("fMP4 fragment has no media data")
  offset = 0
  units: list[bytes] = []
  while offset < len(mdat):
    if offset + length_size > len(mdat):
      raise ValueError("truncated AVC sample length")
    unit_size = int.from_bytes(mdat[offset:offset + length_size], "big")
    offset += length_size
    if unit_size <= 0 or offset + unit_size > len(mdat):
      raise ValueError("invalid AVC sample unit")
    units.append(b"\x00\x00\x00\x01" + mdat[offset:offset + unit_size])
    offset += unit_size
  return b"".join(units)


@dataclass(eq=False)
class PreviewClient:
  ws: web.WebSocketResponse
  queue: asyncio.Queue[bytes | str]
  sender: asyncio.Task[None] | None = None


class H264JpegDecoder:
  def __init__(self) -> None:
    self.codec: av.CodecContext | None = None
    self.config = b""

  def configure(self, config: bytes) -> None:
    self.close()
    self.config = bytes(config)

  def decode(self, payload: bytes, keyframe: bool) -> bytes | None:
    if not payload or not self.config:
      return None
    if self.codec is None:
      self.codec = av.CodecContext.create("h264", "r")
      self.codec.open()
    data = (self.config if keyframe else b"") + payload
    frames: list[av.VideoFrame] = []
    try:
      for packet in self.codec.parse(data):
        frames.extend(self.codec.decode(packet))
    except av.FFmpegError:
      if not keyframe:
        return None
      self.codec = av.CodecContext.create("h264", "r")
      self.codec.open()
      for packet in self.codec.parse(data):
        frames.extend(self.codec.decode(packet))
    if not frames:
      return None
    output = io.BytesIO()
    frames[-1].to_image().save(output, format="JPEG", quality=78, subsampling=2)
    return output.getvalue()

  def close(self) -> None:
    self.codec = None
    self.config = b""


class ClusterMediaPreviewBridge:
  def __init__(self, upstream: str) -> None:
    self.upstream = upstream.rstrip("/")
    self.clients: set[PreviewClient] = set()
    self.upstream_task: asyncio.Task[None] | None = None
    self.state_task: asyncio.Task[None] | None = None
    self.upstream_ws: ClientWebSocketResponse | None = None
    self.upstream_connected = False
    self.state_connected = False
    self.state_messages = 0
    self.last_state_at = 0.0
    self.last_state_wire: str | None = None
    self.latest_images: dict[str, bytes] = {}
    self.media_records: dict[str, dict[str, Any]] = {}
    self.initialization_wire: bytes | None = None
    self.fmp4_gop: deque[bytes] = deque(maxlen=90)
    self.raw_gop: deque[bytes] = deque(maxlen=90)
    self.latest_jpeg_wire: bytes | None = None
    self.config = b""
    self.codec = "avc1.42E01E"
    self.length_size = 4
    self.width = 960
    self.height = 540
    self.session_id = ""
    self.frames = 0
    self.keyframes = 0
    self.last_sequence = 0
    self.last_frame_at = 0.0
    self.error = ""
    self.jpeg_decoder = H264JpegDecoder()

  def status(self) -> dict[str, Any]:
    age_ms = int((time.monotonic() - self.last_frame_at) * 1000) if self.last_frame_at else None
    return {
      "ok": True,
      "source": f"{self.upstream}/ws/carrot_navi/media",
      "receiver": {
        "connected": self.upstream_connected,
        "peer": self.upstream,
        "appVersion": "cluster-live",
        "sessionId": self.session_id or None,
        "receivedCount": self.frames,
        "error": self.error or None,
      },
      "state": {
        "connected": self.state_connected,
        "messages": self.state_messages,
        "ageMs": int((time.monotonic() - self.last_state_at) * 1000) if self.last_state_at else None,
      },
      "map": {
        "configPresent": bool(self.config),
        "codec": self.codec,
        "width": self.width,
        "height": self.height,
        "frames": self.frames,
        "keyframes": self.keyframes,
        "lastSequence": self.last_sequence,
        "frameAgeMs": age_ms,
      },
      "previewClients": len(self.clients),
      "streams": self.media_records,
      "error": self.error,
    }

  async def start(self) -> None:
    if self.upstream_task is None or self.upstream_task.done():
      self.upstream_task = asyncio.create_task(self._upstream_loop())
    if self.state_task is None or self.state_task.done():
      self.state_task = asyncio.create_task(self._state_loop())

  async def stop(self) -> None:
    tasks = (self.upstream_task, self.state_task)
    self.upstream_task = None
    self.state_task = None
    for task in tasks:
      if task is None:
        continue
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
    for client in tuple(self.clients):
      await self.unregister(client)
    self.jpeg_decoder.close()

  async def register(self, ws: web.WebSocketResponse) -> PreviewClient:
    client = PreviewClient(ws=ws, queue=asyncio.Queue(maxsize=CLIENT_QUEUE_SIZE))
    client.sender = asyncio.create_task(self._sender(client))
    self.clients.add(client)
    client.queue.put_nowait(self._hello_wire())
    if self.last_state_wire is not None and not client.queue.full():
      client.queue.put_nowait(self.last_state_wire)
    for wire in self._bootstrap():
      if client.queue.full():
        break
      client.queue.put_nowait(wire)
    return client

  async def unregister(self, client: PreviewClient) -> None:
    self.clients.discard(client)
    sender = client.sender
    client.sender = None
    if sender is not None and sender is not asyncio.current_task():
      sender.cancel()
      try:
        await sender
      except asyncio.CancelledError:
        pass
    if not client.ws.closed:
      await client.ws.close()

  async def _sender(self, client: PreviewClient) -> None:
    try:
      while not client.ws.closed:
        wire = await client.queue.get()
        if isinstance(wire, bytes):
          await client.ws.send_bytes(wire)
        else:
          await client.ws.send_str(wire)
    except (asyncio.CancelledError, ConnectionError, RuntimeError):
      pass

  def _hello_wire(self) -> str:
    return json.dumps({
      "type": "hello",
      "source": "cluster-live-carrot-navi-media",
      "codec": self.codec,
      "width": self.width,
      "height": self.height,
      "status": self.status(),
    }, ensure_ascii=False, separators=(",", ":"))

  def _broadcast(self, wire: bytes | str) -> None:
    for client in tuple(self.clients):
      if client.queue.full():
        asyncio.create_task(client.ws.close(code=1013, message=b"preview_client_slow"))
      else:
        client.queue.put_nowait(wire)

  def _bootstrap(self) -> tuple[bytes, ...]:
    wires: list[bytes] = []
    if self.initialization_wire is not None:
      wires.append(self.initialization_wire)
      wires.extend(self.fmp4_gop)
    wires.extend(self.raw_gop)
    wires.extend(self.latest_images.values())
    if self.latest_jpeg_wire is not None:
      wires.append(self.latest_jpeg_wire)
    return tuple(wires)

  def _reset_media(self) -> None:
    self.initialization_wire = None
    self.fmp4_gop.clear()
    self.raw_gop.clear()
    self.latest_jpeg_wire = None
    self.config = b""
    self.jpeg_decoder.close()

  def _derived_wire(self, metadata: dict[str, Any], *, kind: str, message_type: int, payload: bytes) -> bytes:
    derived = {
      **metadata,
      "type": "carrotNaviLab",
      "kind": kind,
      "messageType": message_type,
      "payloadKind": "annex-b" if kind == "raw_h264" else "server-decoded-jpeg",
    }
    if kind == "raw_h264":
      derived["codec"] = self.codec
    elif kind == "jpeg":
      derived["mime"] = "image/jpeg"
      derived["flags"] = 0
    return encode_media_frame(derived, payload)

  def _handle_upstream_wire(self, wire: bytes) -> None:
    metadata, payload = parse_media_frame(wire)
    kind = str(metadata.get("kind", ""))
    name = str(metadata.get("name", ""))
    if kind == "image" and name:
      self.media_records[f"image:{name}"] = {
        "present": bool(metadata.get("present", False)),
        "sequence": int(metadata.get("sequence", 0)),
        "sourceTimestampMillis": int(metadata.get("sourceTimestampMillis", 0)),
        "messageType": int(metadata.get("messageType", 0)),
        "wireBytes": len(wire),
        "updatedAtMs": int(time.time() * 1000),
      }
      if metadata.get("present", False) and int(metadata.get("messageType", 0)) == 1 and payload:
        self.latest_images[name] = wire
      else:
        self.latest_images.pop(name, None)
      self._broadcast(wire)
      return
    if metadata.get("kind") != "fmp4" or metadata.get("name") != "map_main":
      return
    self.media_records["render:map_main"] = {
      "present": bool(metadata.get("present", False)),
      "sequence": int(metadata.get("sequence", 0)),
      "sourceTimestampMillis": int(metadata.get("sourceTimestampMillis", 0)),
      "messageType": int(metadata.get("messageType", 0)),
      "wireBytes": len(wire),
      "updatedAtMs": int(time.time() * 1000),
    }
    self.session_id = str(metadata.get("sessionId", ""))
    message_type = int(metadata.get("messageType", 0))
    if not metadata.get("present", False) or message_type == 4:
      self._reset_media()
      self._broadcast(wire)
      return
    if message_type == 2:
      self._reset_media()
      self.config, self.codec, self.length_size = avcc_configuration(payload)
      self.width = max(1, int(metadata.get("width", 960)))
      self.height = max(1, int(metadata.get("height", 540)))
      self.jpeg_decoder.configure(self.config)
      self.initialization_wire = wire
      self._broadcast(self._hello_wire())
      self._broadcast(wire)
      return
    if message_type != 3 or not payload or not self.config:
      return
    keyframe = bool(int(metadata.get("flags", 0)) & H264_KEYFRAME_FLAG)
    self.frames += 1
    self.keyframes += int(keyframe)
    self.last_sequence = int(metadata.get("sequence", 0))
    self.last_frame_at = time.monotonic()
    if keyframe:
      self.fmp4_gop.clear()
    self.fmp4_gop.append(wire)
    self._broadcast(wire)

    annex_b = fmp4_sample_to_annex_b(payload, self.length_size)
    raw_payload = (self.config if keyframe else b"") + annex_b
    raw_wire = self._derived_wire(metadata, kind="raw_h264", message_type=3, payload=raw_payload)
    if keyframe:
      self.raw_gop.clear()
    self.raw_gop.append(raw_wire)
    self._broadcast(raw_wire)

    jpeg = self.jpeg_decoder.decode(annex_b, keyframe)
    if jpeg is not None:
      self.latest_jpeg_wire = self._derived_wire(metadata, kind="jpeg", message_type=1, payload=jpeg)
      self._broadcast(self.latest_jpeg_wire)
    self.error = ""

  async def _consume_upstream(self, session: ClientSession) -> None:
    url = f"{self.upstream}/ws/carrot_navi/media"
    async with session.ws_connect(url, heartbeat=15, compress=0, max_msg_size=8 * 1024 * 1024) as ws:
      self.upstream_ws = ws
      self.upstream_connected = True
      self._broadcast(self._hello_wire())
      async for message in ws:
        if message.type == WSMsgType.BINARY:
          self._handle_upstream_wire(bytes(message.data))
        elif message.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
          break

  async def _consume_state(self, session: ClientSession) -> None:
    url = f"{self.upstream}/ws/carrot_navi/state"
    async with session.ws_connect(url, heartbeat=15, compress=0, max_msg_size=2 * 1024 * 1024) as ws:
      self.state_connected = True
      self._broadcast(self._hello_wire())
      async for message in ws:
        if message.type == WSMsgType.TEXT:
          try:
            envelope = json.loads(message.data)
          except (TypeError, json.JSONDecodeError):
            continue
          state = envelope.get("state") if isinstance(envelope, dict) and envelope.get("type") == "carrotNaviState" else envelope
          if not isinstance(state, dict):
            continue
          self.state_messages += 1
          self.last_state_at = time.monotonic()
          self.last_state_wire = json.dumps({"type": "state", "state": state}, ensure_ascii=False, separators=(",", ":"))
          self._broadcast(self.last_state_wire)
        elif message.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
          break

  async def _upstream_loop(self) -> None:
    async with ClientSession() as session:
      while True:
        try:
          await self._consume_upstream(session)
        except asyncio.CancelledError:
          raise
        except Exception as exc:
          self.error = f"{type(exc).__name__}: {exc}"[:256]
        finally:
          self.upstream_ws = None
          self.upstream_connected = False
          self._broadcast(self._hello_wire())
        await asyncio.sleep(0.8)

  async def _state_loop(self) -> None:
    async with ClientSession() as session:
      while True:
        try:
          await self._consume_state(session)
        except asyncio.CancelledError:
          raise
        except Exception as exc:
          self.error = f"{type(exc).__name__}: {exc}"[:256]
        finally:
          self.state_connected = False
          self._broadcast(self._hello_wire())
        await asyncio.sleep(0.8)


async def bridge_context(app: web.Application):
  bridge = app[BRIDGE_KEY]
  await bridge.start()
  try:
    yield
  finally:
    await bridge.stop()


async def index(_request: web.Request) -> web.FileResponse:
  return web.FileResponse(LAB_DIR / "index.html")


async def worker(_request: web.Request) -> web.FileResponse:
  return web.FileResponse(WORKER_PATH)


async def streams_page(_request: web.Request) -> web.FileResponse:
  return web.FileResponse(LAB_DIR / "streams.html")


async def split_page(_request: web.Request) -> web.FileResponse:
  return web.FileResponse(LAB_DIR / "split.html")


def script_file(path: Path):
  async def handler(_request: web.Request) -> web.FileResponse:
    return web.FileResponse(path)
  return handler


async def status(request: web.Request) -> web.Response:
  return web.json_response(request.app[BRIDGE_KEY].status())


async def stream(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=15, compress=False, max_msg_size=8 * 1024 * 1024)
  await ws.prepare(request)
  bridge = request.app[BRIDGE_KEY]
  client = await bridge.register(ws)
  try:
    async for _message in ws:
      pass
  finally:
    await bridge.unregister(client)
  return ws


def create_app(upstream: str) -> web.Application:
  app = web.Application()
  app[BRIDGE_KEY] = ClusterMediaPreviewBridge(upstream)
  app.cleanup_ctx.append(bridge_context)
  app.router.add_get("/", index)
  app.router.add_get("/lab", index)
  app.router.add_get("/lab/", index)
  app.router.add_get("/lab/carrot_navi_worker.js", worker)
  app.router.add_get("/lab/streams", streams_page)
  app.router.add_get("/lab/streams/", streams_page)
  app.router.add_get("/lab/split", split_page)
  app.router.add_get("/lab/split/", split_page)
  app.router.add_get("/lab/carrot_navi_overlay_primitives.js", script_file(OVERLAY_PRIMITIVES_PATH))
  app.router.add_get("/lab/carrot_navi_overlay.js", script_file(OVERLAY_PATH))
  app.router.add_get("/lab/carrot_navi_overlay_store.js", script_file(OVERLAY_STORE_PATH))
  app.router.add_get("/lab/carrot_navi_mse.js", script_file(MSE_PATH))
  app.router.add_get("/lab/carrot_navi_compositor.js", script_file(COMPOSITOR_PATH))
  app.router.add_get("/lab/carrot_navi_split.js", script_file(SPLIT_PATH))
  app.router.add_get("/lab/carrot_navi.css", script_file(SPLIT_CSS_PATH))
  app.router.add_get("/lab/api/status", status)
  app.router.add_get("/lab/ws", stream)
  return app


def main() -> None:
  parser = argparse.ArgumentParser(description="Live cluster Carrot Navi browser pipeline preview")
  parser.add_argument("--host", default="0.0.0.0")
  parser.add_argument("--port", default=8765, type=int)
  parser.add_argument("--upstream", default="http://172.30.1.24:7000")
  args = parser.parse_args()
  print(f"Live source: {args.upstream}/ws/carrot_navi/media", flush=True)
  print(f"Preview: http://127.0.0.1:{args.port}/lab", flush=True)
  web.run_app(create_app(args.upstream), host=args.host, port=args.port, access_log=None)


if __name__ == "__main__":
  main()
