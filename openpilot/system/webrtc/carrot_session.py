import asyncio
import json
import re
import time
from dataclasses import dataclass, field
from typing import Any

from aiohttp import web

from openpilot.system.webrtc import webrtcd


_STREAM_IDENTIFIER_PATTERN = re.compile(r"[^a-zA-Z0-9._:-]+")
CARROT_STREAM_NEGOTIATION_TIMEOUT_SEC = 12.0
CARROT_STREAM_DISCONNECTED_GRACE_SEC = 3.0
CARROT_STREAM_PRUNE_INTERVAL_SEC = 1.0


def normalize_stream_identifier(value: Any) -> str:
  return _STREAM_IDENTIFIER_PATTERN.sub("-", str(value or "").strip()).strip("-")[:128]


@dataclass(frozen=True)
class CarrotStreamRequestBody:
  sdp: str
  cameras: list[str]
  bridge_services_in: list[str] = field(default_factory=list)
  bridge_services_out: list[str] = field(default_factory=list)
  client_id: str = ""
  device_id: str = ""
  tab_id: str = ""
  attempt_id: str = ""
  takeover: bool = False
  carrot_state: bool = False


class CarrotStreamSession(webrtcd.StreamSession):
  """Carrot Vision session with idempotent, registry-aware cleanup."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._carrot_cleanup_lock = asyncio.Lock()
    self._carrot_cleanup_complete = False
    self._carrot_created_monotonic = time.monotonic()
    self._carrot_connected_monotonic = None
    self._carrot_disconnected_monotonic = None
    peer_connection = getattr(self.stream, "peer_connection", None)
    if callable(getattr(peer_connection, "on", None)):
      peer_connection.on("connectionstatechange", self._record_carrot_connection_state)

  def _record_carrot_connection_state(self):
    current_time = time.monotonic()
    connection_state = getattr(getattr(self.stream, "peer_connection", None), "connectionState", None)
    if connection_state == "connected":
      self._carrot_connected_monotonic = current_time
      self._carrot_disconnected_monotonic = None
    elif connection_state == "disconnected":
      if self._carrot_disconnected_monotonic is None:
        self._carrot_disconnected_monotonic = current_time
    elif connection_state not in {"failed", "closed"}:
      self._carrot_disconnected_monotonic = None

  async def stop(self):
    run_task = self.run_task
    webrtcd.webrtcd_log("info", "Carrot stream session (%s) stop requested (started=%s done=%s)",
                        self.identifier, run_task is not None, run_task.done() if run_task is not None else False)

    if run_task is not None and run_task is not asyncio.current_task():
      if not run_task.done():
        run_task.cancel()
      try:
        await run_task
      except asyncio.CancelledError:
        pass
      except Exception:
        self.logger.exception("Carrot stream session task failed while stopping")
    self.run_task = None
    await self.post_run_cleanup()

  async def post_run_cleanup(self):
    async with self._carrot_cleanup_lock:
      if self._carrot_cleanup_complete:
        return

      cleanup_cancelled = False
      try:
        await self.stream.stop()
      except asyncio.CancelledError:
        cleanup_cancelled = True
      except Exception:
        self.logger.exception("Failed stopping Carrot Vision WebRTC stream")

      if self.outgoing_bridge_runner is not None:
        try:
          self.outgoing_bridge_runner.stop()
        except Exception:
          self.logger.exception("Failed stopping Carrot Vision outgoing bridge")

      if self.carrot_state_bridge_runner is not None:
        try:
          self.carrot_state_bridge_runner.stop()
        except Exception:
          self.logger.exception("Failed stopping Carrot Vision state bridge")

      for track in getattr(self, "_video_tracks", []):
        try:
          track.close_sock()
        except Exception:
          self.logger.exception("Failed closing Carrot Vision video track socket")

      try:
        stream_dict = getattr(self, "stream_dict", None)
        if stream_dict is not None:
          remove_stream_session(stream_dict, self.identifier)
      except Exception:
        self.logger.exception("Failed removing Carrot Vision session from registry")
      finally:
        self._carrot_cleanup_complete = True

      if cleanup_cancelled:
        raise asyncio.CancelledError


def remove_stream_session(stream_dict: dict[str, Any], identifier: str) -> bool:
  removed = stream_dict.pop(identifier, None) is not None
  if removed:
    webrtcd._sync_carrot_vision_active(stream_dict)
  return removed


def stream_session_has_ended(session: webrtcd.StreamSession) -> bool:
  if getattr(session, "_carrot_cleanup_complete", False) is True:
    return True

  run_task = getattr(session, "run_task", None)
  if run_task is not None:
    try:
      if run_task.done() is True:
        return True
    except Exception:
      pass

  stream = getattr(session, "stream", None)
  peer_connection = getattr(stream, "peer_connection", None)
  return getattr(peer_connection, "connectionState", None) in {"closed", "failed"}


def stream_session_reclaim_reason(
  session: webrtcd.StreamSession,
  now_monotonic: float | None = None,
) -> str | None:
  if stream_session_has_ended(session):
    return "ended"

  current_time = time.monotonic() if now_monotonic is None else float(now_monotonic)
  stream = getattr(session, "stream", None)
  peer_connection = getattr(stream, "peer_connection", None)
  connection_state = getattr(peer_connection, "connectionState", None)

  if connection_state == "connected":
    session._carrot_connected_monotonic = current_time
    session._carrot_disconnected_monotonic = None
    return None

  if connection_state == "disconnected":
    disconnected_at = getattr(session, "_carrot_disconnected_monotonic", None)
    if disconnected_at is None:
      session._carrot_disconnected_monotonic = current_time
      return None
    if current_time - float(disconnected_at) >= CARROT_STREAM_DISCONNECTED_GRACE_SEC:
      return "disconnected-grace-expired"
    return None

  session._carrot_disconnected_monotonic = None
  if connection_state in {"new", "connecting"}:
    created_at = getattr(session, "_carrot_created_monotonic", None)
    if created_at is not None and current_time - float(created_at) >= CARROT_STREAM_NEGOTIATION_TIMEOUT_SEC:
      return "negotiation-timeout"
  return None


async def prune_ended_stream_sessions(stream_dict: dict[str, webrtcd.StreamSession]) -> None:
  reclaimable_sessions = [
    (session, reason)
    for session in list(stream_dict.values())
    if (reason := stream_session_reclaim_reason(session)) is not None
  ]
  for session, reason in reclaimable_sessions:
    webrtcd.webrtcd_log(
      "warning",
      "Reclaiming stale Carrot Vision session %s reason=%s",
      getattr(session, "identifier", "?"),
      reason,
    )
    try:
      await session.stop()
    except Exception:
      webrtcd.webrtcd_log("exception", "Failed stopping stale Carrot Vision session %s",
                          getattr(session, "identifier", "?"))
    finally:
      remove_stream_session(stream_dict, getattr(session, "identifier", ""))


async def stream_session_cleanup_context(app: web.Application):
  async def cleanup_loop():
    while True:
      await asyncio.sleep(CARROT_STREAM_PRUNE_INTERVAL_SEC)
      try:
        async with app["stream_lock"]:
          await prune_ended_stream_sessions(app["streams"])
      except asyncio.CancelledError:
        raise
      except Exception:
        webrtcd.webrtcd_log("exception", "Carrot Vision stale session cleanup failed")

  cleanup_task = asyncio.create_task(cleanup_loop(), name="carrot-vision-session-cleanup")
  app["stream_cleanup_task"] = cleanup_task
  try:
    yield
  finally:
    cleanup_task.cancel()
    try:
      await cleanup_task
    except asyncio.CancelledError:
      pass
    app.pop("stream_cleanup_task", None)


async def get_stream(request: web.Request):
  stream_dict, debug_mode = request.app["streams"], request.app["debug"]

  raw_body = await request.json()
  body = CarrotStreamRequestBody(
    sdp=raw_body["sdp"],
    cameras=raw_body["cameras"],
    bridge_services_in=raw_body.get("bridge_services_in", []),
    bridge_services_out=raw_body.get("bridge_services_out", []),
    client_id=normalize_stream_identifier(raw_body.get("client_id", "")),
    device_id=normalize_stream_identifier(raw_body.get("device_id", "")),
    tab_id=normalize_stream_identifier(raw_body.get("tab_id", "")),
    attempt_id=normalize_stream_identifier(raw_body.get("attempt_id", "")),
    takeover=bool(raw_body.get("takeover", False)),
    carrot_state=bool(raw_body.get("carrot_state", False)),
  )

  async with request.app["stream_lock"]:
    await prune_ended_stream_sessions(stream_dict)
    device_id = body.device_id or body.client_id
    client_key = webrtcd._stream_client_key(device_id, request.remote)
    road_requested = "road" in body.cameras
    if road_requested:
      old_sessions, foreign_sessions = webrtcd._carrot_vision_road_sessions(stream_dict, client_key)
    else:
      old_sessions, foreign_sessions = [], []

    stream_request_log = "Carrot Vision stream request from %s cameras=%s device=%s tab=%s attempt=%s takeover=%s old_sessions=%s"
    webrtcd.webrtcd_log("info", stream_request_log,
                        request.remote, body.cameras, device_id or "legacy-remote",
                        body.tab_id or "-", body.attempt_id or "-", body.takeover,
                        [getattr(old, "identifier", "?") for old in old_sessions])

    if road_requested and foreign_sessions and not body.takeover:
      webrtcd.webrtcd_log("warning", "Carrot Vision stream rejected busy owner=%s requester=%s",
                          [getattr(session, "identifier", "?") for session in foreign_sessions], client_key)
      raise web.HTTPConflict(
        text=json.dumps({
          "ok": False,
          "code": webrtcd.CARROT_VISION_BUSY_CODE,
          "error": "Carrot Vision is already active on another client",
        }),
        content_type="application/json",
      )

    session = CarrotStreamSession(
      body.sdp,
      body.cameras,
      body.bridge_services_in,
      body.bridge_services_out,
      debug_mode,
      client_key=client_key,
      carrot_state=body.carrot_state,
    )
    session.stream_dict = stream_dict
    session.device_id = device_id
    session.tab_id = body.tab_id
    session.attempt_id = body.attempt_id

    replaced_sessions = old_sessions + (foreign_sessions if body.takeover else [])
    for old in replaced_sessions:
      try:
        await old.stop()
      except Exception:
        webrtcd.webrtcd_log("exception", "Failed stopping replaced Carrot Vision session %s",
                            getattr(old, "identifier", "?"))
        await session.stop()
        raise web.HTTPInternalServerError(text="Failed replacing active Carrot Vision session") from None
      finally:
        remove_stream_session(stream_dict, getattr(old, "identifier", ""))

    stream_dict[session.identifier] = session
    webrtcd._sync_carrot_vision_active(stream_dict)

    try:
      answer = await session.get_answer()
    except Exception:
      await session.stop()
      raise
    session.start()

    webrtcd.webrtcd_log("info", "Created Carrot Vision session %s active_sessions=%d",
                        session.identifier, len(stream_dict))
    return web.json_response({"sdp": answer.sdp, "type": answer.type},
                             headers={"Access-Control-Allow-Origin": "*"})


async def on_shutdown(app: web.Application):
  sessions = list(app["streams"].values())
  for session in sessions:
    try:
      await session.stop()
    except Exception:
      webrtcd.webrtcd_log("exception", "Failed stopping Carrot Vision session %s during shutdown",
                          getattr(session, "identifier", "?"))
    finally:
      remove_stream_session(app["streams"], getattr(session, "identifier", ""))
  app["streams"].clear()
  webrtcd._set_carrot_vision_active(False)
