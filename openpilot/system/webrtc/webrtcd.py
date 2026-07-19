#!/usr/bin/env python3

import argparse
import asyncio
import json
import os
import struct
import uuid
import logging
from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

# aiortc and its dependencies have lots of internal warnings :(
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

import capnp
from aiohttp import web
if TYPE_CHECKING:
  from aiortc.rtcdatachannel import RTCDataChannel

from openpilot.common.swaglog import cloudlog
from openpilot.system.webrtc.schema import generate_field
from openpilot.system.webrtc.carrot_state import CarrotStateChannelProxy
from openpilot.cereal import messaging, log


CARROT_VISION_ACTIVE_PARAM = "CarrotVisionActive"
CARROT_VISION_BUSY_CODE = "carrot_vision_busy"
_carrot_vision_params: Any | None = None
_carrot_vision_active: bool | None = None
_carrot_vision_mode = False


def _set_carrot_vision_active(active: bool) -> None:
  global _carrot_vision_active
  if not _carrot_vision_mode or _carrot_vision_params is None:
    return
  active = bool(active)
  if _carrot_vision_active == active:
    return
  _carrot_vision_active = active
  try:
    _carrot_vision_params.put_bool_nonblocking(CARROT_VISION_ACTIVE_PARAM, active)
  except Exception:
    cloudlog.exception("[webrtcd] failed updating %s=%s", CARROT_VISION_ACTIVE_PARAM, active)


def _sync_carrot_vision_active(stream_dict: dict[str, Any]) -> None:
  if not _carrot_vision_mode:
    return
  active = any(getattr(session, "uses_road_camera", False) for session in stream_dict.values())
  _set_carrot_vision_active(active)


def webrtcd_log(level: str, msg: str, *args):
  logger = logging.getLogger("webrtcd")
  getattr(logger, level)(msg, *args)
  getattr(cloudlog, level)("[webrtcd] " + msg, *args)


class CerealOutgoingMessageProxy:
  def __init__(self, sm: messaging.SubMaster):
    self.sm = sm
    self.channels: list[RTCDataChannel] = []

  def add_channel(self, channel: 'RTCDataChannel'):
    self.channels.append(channel)

  def to_json(self, msg_content: Any):
    if isinstance(msg_content, capnp._DynamicStructReader):
      msg_dict = msg_content.to_dict()
    elif isinstance(msg_content, capnp._DynamicListReader):
      msg_dict = [self.to_json(msg) for msg in msg_content]
    elif isinstance(msg_content, bytes):
      msg_dict = msg_content.decode()
    else:
      msg_dict = msg_content

    return msg_dict

  async def update(self):
    # this is blocking in async context...
    self.sm.update(0)

    for service, updated in self.sm.updated.items():
      if not updated:
        continue

      msg_dict = self.to_json(self.sm[service])
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {"type": service, "logMonoTime": mono_time, "valid": valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()

      alive_channels = []
      for channel in self.channels:
        try:
          if isinstance(channel, web.WebSocketResponse):
            await channel.send_bytes(encoded_msg)
          else:
            channel.send(encoded_msg)
          alive_channels.append(channel)
        except Exception:
          continue

      self.channels = alive_channels


class CerealIncomingMessageProxy:
  def __init__(self, pm: messaging.PubMaster):
    self.pm = pm

  def send(self, message: bytes):
    msg_json = json.loads(message)
    msg_type, msg_data = msg_json["type"], msg_json["data"]
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(msg_type, size=size)
    setattr(msg, msg_type, msg_data)
    self.pm.send(msg_type, msg)


class CerealProxyRunner:
  def __init__(self, proxy: Any):
    self.proxy = proxy
    self.is_running = False
    self.task = None
    self.logger = logging.getLogger("webrtcd")

  def start(self):
    assert self.task is None
    self.task = asyncio.create_task(self.run())

  def stop(self):
    if self.task is None or self.task.done():
      return
    self.task.cancel()
    self.task = None

  async def run(self):
    from aiortc.exceptions import InvalidStateError

    while True:
      try:
        await self.proxy.update()
      except InvalidStateError:
        self.logger.warning("Cereal outgoing proxy invalid state (connection closed)")
        break
      except Exception:
        self.logger.exception("Cereal outgoing proxy failure")
      await asyncio.sleep(0.01)


class DynamicPubMaster(messaging.PubMaster):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.lock = asyncio.Lock()

  async def add_services_if_needed(self, services):
    async with self.lock:
      for service in services:
        if service not in self.sock:
          self.sock[service] = messaging.pub_sock(service)


class StreamSession:
  shared_pub_master = DynamicPubMaster([])
  FRAME_SYNC_PACKET = struct.Struct(">4sII")
  FRAME_SYNC_MAGIC = b"CVF1"
  FRAME_SYNC_MAX_BUFFERED_BYTES = 4096

  def __init__(self, sdp: str, cameras: list[str], incoming_services: list[str], outgoing_services: list[str],
               debug_mode: bool = False, client_key: str = "", carrot_state: bool = False):
    from aiortc.mediastreams import VideoStreamTrack
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc import WebRTCAnswerBuilder
    from teleoprtc.info import parse_info_from_offer

    self.logger = logging.getLogger("webrtcd")
    config = parse_info_from_offer(sdp)
    builder = WebRTCAnswerBuilder(sdp)

    self._video_tracks = []
    self.carrot_state = _carrot_vision_mode and bool(carrot_state)
    self.uses_road_camera = "road" in cameras
    assert len(cameras) == config.n_expected_camera_tracks, "Incoming stream has misconfigured number of video tracks"
    for cam in cameras:
      try:
        track = LiveStreamVideoStreamTrack(
          cam,
          use_source_frame_timestamps=_carrot_vision_mode,
        ) if not debug_mode else VideoStreamTrack()
        builder.add_video_stream(cam, track)
        if hasattr(track, 'close_sock'):
          self._video_tracks.append(track)
        self.logger.info("added camera track: %s", cam)
      except Exception:
        self.logger.exception("failed to create camera track: %s", cam)
        raise

    self.identifier = str(uuid.uuid4())
    self.client_key = client_key
    self.stream = builder.stream()
    setattr(self.stream, "session_identifier", self.identifier)

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = incoming_services
    self.outgoing_bridge: CerealOutgoingMessageProxy | None = None
    self.outgoing_bridge_runner: CerealProxyRunner | None = None
    self.carrot_state_bridge: CarrotStateChannelProxy | None = None
    self.carrot_state_bridge_runner: CerealProxyRunner | None = None
    if len(incoming_services) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(outgoing_services) > 0:
      self.outgoing_bridge = CerealOutgoingMessageProxy(messaging.SubMaster(outgoing_services))
      self.outgoing_bridge_runner = CerealProxyRunner(self.outgoing_bridge)
    if carrot_state and not _carrot_vision_mode:
      self.carrot_state_bridge = CarrotStateChannelProxy()
      self.carrot_state_bridge_runner = CerealProxyRunner(self.carrot_state_bridge)

    self.run_task: asyncio.Task | None = None
    self.logger.info("New stream session (%s), cameras %s, incoming services %s, outgoing services %s",
                      self.identifier, cameras, incoming_services, outgoing_services)
    self.logger.info("request cameras=%s", cameras)


  def start(self):
    webrtcd_log("info", "Stream session (%s) start requested", self.identifier)
    self.run_task = asyncio.create_task(self.run())

  async def stop(self):
    if self.run_task is None:
      return

    webrtcd_log("info", "Stream session (%s) stop requested (done=%s)", self.identifier, self.run_task.done())

    if not self.run_task.done():
      self.run_task.cancel()
      try:
        await self.run_task
      except asyncio.CancelledError:
        pass

    self.run_task = None
    await self.post_run_cleanup()
    webrtcd_log("info", "Stream session (%s) stop cleanup complete", self.identifier)

    try:
      if hasattr(self, "stream_dict"):
        self.stream_dict.pop(self.identifier, None)
        _sync_carrot_vision_active(self.stream_dict)
    except Exception:
      self.logger.exception("Failed removing session from streams in stop()")
    

  async def get_answer(self):
    webrtcd_log("info", "Stream session (%s) building answer", self.identifier)
    answer = await self.stream.start()
    self._bind_frame_sync_senders()
    webrtcd_log("info", "Stream session (%s) answer ready", self.identifier)
    return answer

  def _bind_frame_sync_senders(self) -> None:
    if not self.carrot_state:
      return
    senders = self.stream.peer_connection.getSenders()
    for track in self._video_tracks:
      if not getattr(track, "_use_source_frame_timestamps", False):
        continue
      sender = next((candidate for candidate in senders if candidate.track is track), None)
      if sender is not None and hasattr(track, "bind_frame_sync"):
        track.bind_frame_sync(sender, self._send_frame_sync)

  def _send_frame_sync(self, frame_id: int, rtp_timestamp: int) -> None:
    if not self.stream.has_messaging_channel():
      return
    channel = self.stream.get_messaging_channel()
    if channel.readyState != "open" or channel.bufferedAmount > self.FRAME_SYNC_MAX_BUFFERED_BYTES:
      return
    channel.send(self.FRAME_SYNC_PACKET.pack(
      self.FRAME_SYNC_MAGIC,
      int(frame_id) & 0xFFFFFFFF,
      int(rtp_timestamp) & 0xFFFFFFFF,
    ))

  async def message_handler(self, message: bytes):
    assert self.incoming_bridge is not None
    try:
      self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run(self):
    try:
      webrtcd_log("info", "Stream session (%s) waiting for peer connection", self.identifier)
      await self.stream.wait_for_connection()
      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
          self.stream.set_message_handler(self.message_handler)
        if self.outgoing_bridge_runner is not None:
          channel = self.stream.get_messaging_channel()
          self.outgoing_bridge_runner.proxy.add_channel(channel)
          self.outgoing_bridge_runner.start()
        if self.carrot_state_bridge_runner is not None:
          channel = self.stream.get_messaging_channel()
          self.carrot_state_bridge_runner.proxy.add_channel(channel)
          self.carrot_state_bridge_runner.start()
      webrtcd_log("info", "Stream session (%s) connected", self.identifier)

      webrtcd_log("info", "Stream session (%s) waiting for disconnection", self.identifier)
      await self.stream.wait_for_disconnection()
      await self.post_run_cleanup()

      webrtcd_log("info", "Stream session (%s) ended", self.identifier)
    except Exception:
      self.logger.exception("Stream session failure")
    finally:
      await self.post_run_cleanup()
      try:
        if hasattr(self, "stream_dict"):
          self.stream_dict.pop(self.identifier, None)
          _sync_carrot_vision_active(self.stream_dict)
      except Exception:
        self.logger.exception("Failed removing session from streams")
      
  async def post_run_cleanup(self):
    await self.stream.stop()
    if self.outgoing_bridge is not None:
      self.outgoing_bridge_runner.stop()
    if self.carrot_state_bridge_runner is not None:
      self.carrot_state_bridge_runner.stop()
    # Close ZMQ sockets held by video tracks
    for track in getattr(self, '_video_tracks', []):
      try:
        track.close_sock()
      except Exception:
        self.logger.exception("Failed to close video track socket")


@dataclass
class StreamRequestBody:
  sdp: str
  cameras: list[str]
  bridge_services_in: list[str] = field(default_factory=list)
  bridge_services_out: list[str] = field(default_factory=list)
  client_id: str = ""
  takeover: bool = False
  carrot_state: bool = False


def _stream_client_key(client_id: str, remote: str | None) -> str:
  normalized = str(client_id or "").strip()[:128]
  return f"client:{normalized}" if normalized else f"remote:{remote or 'unknown'}"


def _stream_sessions_for_client(stream_dict: dict[str, StreamSession], client_key: str) -> list[StreamSession]:
  return [session for session in stream_dict.values()
          if getattr(session, "client_key", "") == client_key]


def _carrot_vision_road_sessions(
  stream_dict: dict[str, StreamSession],
  client_key: str,
) -> tuple[list[StreamSession], list[StreamSession]]:
  road_sessions = [session for session in stream_dict.values()
                   if getattr(session, "uses_road_camera", False)]
  owned = [session for session in road_sessions
           if getattr(session, "client_key", "") == client_key]
  foreign = [session for session in road_sessions if session not in owned]
  return owned, foreign


async def get_stream(request: 'web.Request'):
  stream_dict, debug_mode = request.app['streams'], request.app['debug']

  raw_body = await request.json()
  body = StreamRequestBody(
    sdp=raw_body["sdp"],
    cameras=raw_body["cameras"],
    bridge_services_in=raw_body.get("bridge_services_in", []),
    bridge_services_out=raw_body.get("bridge_services_out", []),
    client_id=str(raw_body.get("client_id", "")),
    takeover=bool(raw_body.get("takeover", False)),
    carrot_state=bool(raw_body.get("carrot_state", False)),
  )
  async with request.app['stream_lock']:
    client_key = _stream_client_key(body.client_id, request.remote)
    road_requested = _carrot_vision_mode and "road" in body.cameras
    if road_requested:
      old_sessions, foreign_sessions = _carrot_vision_road_sessions(stream_dict, client_key)
    elif _carrot_vision_mode:
      old_sessions, foreign_sessions = [], []
    else:
      old_sessions = _stream_sessions_for_client(stream_dict, client_key)
      foreign_sessions = []
    webrtcd_log("info", "get_stream request from %s cameras=%s takeover=%s old_sessions=%s", request.remote,
                body.cameras, body.takeover, [getattr(old, "identifier", "?") for old in old_sessions])

    # One road-video peer keeps RTP/packetization bounded. Other devices can use
    # the HUD/state relay concurrently, but a second Vision viewer must wait until
    # the current owner stops instead of multiplying real-time work on-device.
    if road_requested and foreign_sessions and not body.takeover:
      webrtcd_log("warning", "get_stream rejected busy owner=%s requester=%s",
                  [getattr(session, "identifier", "?") for session in foreign_sessions], client_key)
      raise web.HTTPConflict(
        text=json.dumps({
          "ok": False,
          "code": CARROT_VISION_BUSY_CODE,
          "error": "Carrot Vision is already active on another client",
        }),
        content_type="application/json",
      )

    session = StreamSession(body.sdp, body.cameras, body.bridge_services_in, body.bridge_services_out,
                            debug_mode, client_key=client_key, carrot_state=body.carrot_state)
    session.stream_dict = stream_dict

    replaced_sessions = old_sessions + (foreign_sessions if body.takeover else [])
    for old in replaced_sessions:
      try:
        await old.stop()
        stream_dict.pop(getattr(old, "identifier", ""), None)
      except Exception:
        webrtcd_log("exception", "Failed stopping replaced session %s", getattr(old, "identifier", "?"))
        await session.post_run_cleanup()
        raise web.HTTPInternalServerError(text="Failed replacing active Carrot Vision session") from None

    stream_dict[session.identifier] = session
    _sync_carrot_vision_active(stream_dict)

    try:
      answer = await session.get_answer()
    except Exception:
      stream_dict.pop(session.identifier, None)
      _sync_carrot_vision_active(stream_dict)
      await session.post_run_cleanup()
      raise
    session.start()

    webrtcd_log("info", "get_stream created new session %s active_sessions=%d", session.identifier, len(stream_dict))

    return web.json_response({"sdp": answer.sdp, "type": answer.type}, headers={'Access-Control-Allow-Origin': '*'})


async def get_schema(request: 'web.Request'):
  services = request.query["services"].split(",")
  services = [s for s in services if s]
  assert all(s in log.Event.schema.fields and not s.endswith("DEPRECATED") for s in services), "Invalid service name"
  schema_dict = {s: generate_field(log.Event.schema.fields[s]) for s in services}
  return web.json_response(schema_dict)

async def post_notify(request: 'web.Request'):
  try:
    payload = await request.json()
  except Exception as e:
    raise web.HTTPBadRequest(text="Invalid JSON") from e

  for session in list(request.app.get('streams', {}).values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return web.Response(status=200, text="OK")

async def on_shutdown(app: 'web.Application'):
  sessions = list(app['streams'].values())
  for session in sessions:
    await session.stop()
  app['streams'].clear()
  _set_carrot_vision_active(False)

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
    return response

async def handle_cors_preflight(request):
    if request.method == 'OPTIONS':
        headers = {
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Methods': 'GET, POST, PUT, DELETE, OPTIONS',
            'Access-Control-Allow-Headers': 'Content-Type, Authorization',
            'Access-Control-Max-Age': '86400',
        }
        return web.Response(status=200, headers=headers)
    return await request.app['handler'](request)

def webrtcd_thread(host: str, port: int, debug: bool):
  _set_carrot_vision_active(False)
  logging.basicConfig(level=logging.CRITICAL, handlers=[logging.StreamHandler()])
  logging_level = logging.DEBUG if debug else logging.INFO
  logging.getLogger("WebRTCStream").setLevel(logging_level)
  logging.getLogger("webrtcd").setLevel(logging_level)

  app = web.Application(middlewares=[cors_middleware])

  app['streams'] = dict()
  app['stream_lock'] = asyncio.Lock()
  app['debug'] = debug
  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/stream", get_stream)
  app.router.add_post("/notify", post_notify)
  app.router.add_get("/schema", get_schema)
  app.router.add_route('OPTIONS', '/{tail:.*}', handle_cors_preflight)
  
  web.run_app(app, host=host, port=port)


def main(carrot_vision_mode: bool = False):
  global _carrot_vision_mode, _carrot_vision_params
  _carrot_vision_mode = bool(carrot_vision_mode)
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  parser.add_argument("--debug", action="store_true", help="Enable debug mode")
  args = parser.parse_args()

  if _carrot_vision_mode:
    from openpilot.common.params import Params
    _carrot_vision_params = Params()

    # Carrot Vision is a background web feature. Keep its Python RTP work on
    # the same efficiency-core pool used by carrot_server, away from the
    # UI/model cores. Override remains available for device diagnostics.
    try:
      from openpilot.common.realtime import set_core_affinity
      cores = [int(core) for core in os.getenv("CARROT_VISION_WEBRTC_CORES", "0,1,2,3").split(",") if core.strip()]
      if cores:
        set_core_affinity(cores)
    except Exception:
      cloudlog.exception("[webrtcd] failed setting Carrot Vision core affinity")

  webrtcd_thread(args.host, args.port, args.debug)


if __name__=="__main__":
  main()
