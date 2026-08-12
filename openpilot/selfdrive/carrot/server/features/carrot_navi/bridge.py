from __future__ import annotations

import asyncio
import json
import time
from typing import Any

from aiohttp import web

from openpilot.cereal import log, messaging

from ...services.params import HAS_PARAMS, Params
from .client_hub import CarrotNaviClientHub
from .media_pipeline import CarrotNaviMediaPipeline
from .protocol import MEDIA_WIRE_VERSION, encode_media_frame


STATE_STALE_SECONDS = 3.0
CLIENT_DIAGNOSTIC_TTL_SECONDS = 15.0
CLUSTER_GATE_CHECK_SECONDS = 0.1
WEB_BOOTSTRAP_KIND = "web_render"
WEB_BOOTSTRAP_IMAGE_KIND = "web_image"


def _is_web_media_stream(kind: str, name: str) -> bool:
  return kind in ("image", WEB_BOOTSTRAP_IMAGE_KIND) or (kind in ("render", WEB_BOOTSTRAP_KIND) and name == "map_main")


class CarrotNaviWebBridge:
  """Demand-driven Cereal reader coordinating client and media pipeline modules."""

  POLL_SLEEP = 0.004
  IDLE_CLOSE_SECONDS = 2.0
  SEND_TIMEOUT = 1.0
  MEDIA_QUEUE_SIZE = 12

  def __init__(self, messaging_module: Any = messaging) -> None:
    self.messaging = messaging_module
    self._media = CarrotNaviMediaPipeline()
    self._clients = CarrotNaviClientHub(
      media_queue_size=self.MEDIA_QUEUE_SIZE,
      send_timeout=self.SEND_TIMEOUT,
      on_map_demand_lost=lambda: self._media.reset_fmp4(clear_config=False),
    )
    self._state_socket: Any | None = None
    self._media_socket: Any | None = None
    self._poll_task: asyncio.Task | None = None
    self._lock = asyncio.Lock()
    self._last_state_wire: str | None = None
    self._last_state: dict[str, Any] | None = None
    self._last_state_at = 0.0
    self._state_count = 0
    self._media_count = 0
    self._client_diagnostics: dict[str, tuple[float, dict[str, Any]]] = {}
    self._hud_map_profile_clients = 0
    self._error = ""
    self._stream_allowed = False
    self._next_stream_allowed_check = 0.0
    try:
      self._params: Any | None = Params() if HAS_PARAMS and Params is not None else None
      if self._params is None:
        raise RuntimeError("Params unavailable")
    except Exception:
      self._params = None
    try:
      if self._params is not None:
        self._params.put_int_nonblocking("CarrotNaviHudMapProfile", 0)
    except Exception:
      pass

  def stream_allowed(self, force: bool = False) -> bool:
    now = time.monotonic()
    if not force and now < self._next_stream_allowed_check:
      return self._stream_allowed
    self._next_stream_allowed_check = now + CLUSTER_GATE_CHECK_SECONDS
    try:
      self._stream_allowed = bool(
        self._params is not None
        and self._params.get_int("ClusterHud") != 1
      )
    except Exception:
      self._stream_allowed = False
    return self._stream_allowed

  def status(self) -> dict[str, Any]:
    now = time.monotonic()
    stream_allowed = self.stream_allowed(force=True)
    for peer, (received_at, _diagnostic) in tuple(self._client_diagnostics.items()):
      if now - received_at > CLIENT_DIAGNOSTIC_TTL_SECONDS:
        self._client_diagnostics.pop(peer, None)
    state_age_ms = int(max(0.0, now - self._last_state_at) * 1000) if self._last_state_at else None
    state = self._last_state or {}
    navigation = state.get("navigationStatus") if isinstance(state, dict) else {}
    if not isinstance(navigation, dict):
      navigation = {}
    media_status = self._media.status(now)
    return {
      "stateClients": self._clients.state_count,
      "mediaClients": self._clients.media_count,
      "streamAllowed": stream_allowed,
      "readerActive": self._poll_task is not None and not self._poll_task.done(),
      "connected": bool(state.get("connected", False)),
      "guidanceActive": bool(navigation.get("guidanceActive", False)),
      "routePresent": bool(navigation.get("routePresent", False)),
      "stateFresh": state_age_ms is not None and state_age_ms <= int(STATE_STALE_SECONDS * 1000),
      "mapFresh": media_status["mapFresh"],
      "stateAgeMs": state_age_ms,
      "mapAgeMs": media_status["mapAgeMs"],
      "stateMessages": self._state_count,
      "mediaMessages": self._media_count,
      "mapStream": media_status["mapStream"],
      "hudMapProfile": self._hud_map_profile_clients > 0,
      "clientDiagnostics": [
        {
          **diagnostic,
          "peer": peer,
          "ageMs": int(max(0.0, now - received_at) * 1000),
        }
        for peer, (received_at, diagnostic) in sorted(self._client_diagnostics.items())
      ],
      "error": self._error,
    }

  def record_client_diagnostic(self, peer: str, diagnostic: dict[str, Any]) -> None:
    self._client_diagnostics[str(peer or "-")[:128]] = (time.monotonic(), diagnostic)

  def set_hud_map_profile(self, active: bool) -> None:
    if active:
      self._hud_map_profile_clients += 1
    else:
      self._hud_map_profile_clients = max(0, self._hud_map_profile_clients - 1)
    try:
      if self._params is not None:
        self._params.put_int_nonblocking(
          "CarrotNaviHudMapProfile",
          1 if self._hud_map_profile_clients > 0 else 0,
        )
    except Exception:
      pass

  async def register_state(self, ws: web.WebSocketResponse, client_id: str, *, takeover: bool = False) -> bool:
    initial_wire = None
    if self._last_state_wire is not None and time.monotonic() - self._last_state_at <= STATE_STALE_SECONDS:
      initial_wire = self._last_state_wire
    registered = await self._clients.register_state(ws, client_id, takeover=takeover, initial_wire=initial_wire)
    if not registered:
      return False
    await self._ensure_poll_task()
    return True

  async def register_media(
    self,
    ws: web.WebSocketResponse,
    client_id: str,
    *,
    include_map: bool = True,
  ) -> bool:
    bootstrap = self._media.bootstrap(include_map=include_map)
    registered = await self._clients.register_media(
      ws,
      client_id,
      include_map=include_map,
      bootstrap=bootstrap,
    )
    if not registered:
      return False
    await self._ensure_poll_task()
    self._ensure_sockets()
    if not bootstrap or (include_map and not self._media.initialization_present):
      self._request_media_bootstrap()
    return True

  async def unregister(self, ws: web.WebSocketResponse) -> None:
    await self._clients.unregister(ws)

  async def stop(self) -> None:
    async with self._lock:
      task = self._poll_task
      self._poll_task = None
    if task is not None:
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    await self._clients.stop()
    self._hud_map_profile_clients = 0
    try:
      if self._params is not None:
        self._params.put_int_nonblocking("CarrotNaviHudMapProfile", 0)
    except Exception:
      pass
    self._clear_runtime_cache()
    self._close_sockets()

  async def _ensure_poll_task(self) -> None:
    async with self._lock:
      if self._poll_task is None or self._poll_task.done():
        self._poll_task = asyncio.create_task(self._poll_loop())

  def _request_media_bootstrap(self) -> None:
    try:
      if self._params is not None:
        token = f"{time.time_ns():x}"
        self._params.put_nonblocking("CarrotNaviWebBootstrapRequest", token)
    except Exception:
      pass

  def _clear_runtime_cache(self) -> None:
    self._last_state_wire = None
    self._last_state = None
    self._last_state_at = 0.0
    self._media.clear()

  @staticmethod
  def _state_payload(payload: bytes) -> tuple[dict[str, Any], str]:
    with log.Event.from_bytes(payload, traversal_limit_in_words=2**64 - 1) as event:
      data = event.carrotNavi.to_dict()
    wire = json.dumps({"type": "carrotNaviState", "version": 1, "state": data}, ensure_ascii=False, separators=(",", ":"))
    return data, wire

  @staticmethod
  def _media_payload(payload: bytes) -> tuple[dict[str, Any], bytes, bytes] | None:
    with log.Event.from_bytes(payload, traversal_limit_in_words=2**64 - 1) as event:
      data = event.carrotNaviMedia
      source_kind = str(data.kind)
      name = str(data.name)
      if not _is_web_media_stream(source_kind, name):
        return None
      if source_kind == WEB_BOOTSTRAP_KIND:
        kind = "render"
      elif source_kind == WEB_BOOTSTRAP_IMAGE_KIND:
        kind = "image"
      else:
        kind = source_kind
      raw_payload = bytes(data.payload)
      metadata = {
        "type": "carrotNaviMedia",
        "version": MEDIA_WIRE_VERSION,
        "sessionId": str(data.sessionId),
        "kind": kind,
        "name": name,
        "sequence": int(data.sequence),
        "sourceTimestampMillis": int(data.sourceTimestampMillis),
        "receivedMonoTimeNanos": int(data.receivedMonoTimeNanos),
        "present": bool(data.present),
        "messageType": int(data.messageType),
        "formatOrReason": int(data.formatOrReason),
        "flags": int(data.flags),
        "width": int(data.width),
        "height": int(data.height),
        "reason": str(data.reason),
        "bootstrap": source_kind in (WEB_BOOTSTRAP_KIND, WEB_BOOTSTRAP_IMAGE_KIND),
      }
    return metadata, encode_media_frame(metadata, raw_payload), raw_payload

  def _close_sockets(self) -> None:
    for name in ("_state_socket", "_media_socket"):
      sock = getattr(self, name)
      if sock is not None:
        try:
          sock.close()
        except Exception:
          pass
        setattr(self, name, None)

  def _close_inactive_sockets(self) -> None:
    if not self._clients.has_clients and self._state_socket is not None:
      try:
        self._state_socket.close()
      except Exception:
        pass
      self._state_socket = None
    if not self._clients.has_media_clients and self._media_socket is not None:
      try:
        self._media_socket.close()
      except Exception:
        pass
      self._media_socket = None

  def _ensure_sockets(self) -> None:
    if self._clients.has_clients and self._state_socket is None:
      self._state_socket = self.messaging.sub_sock("carrotNavi", conflate=True)
    if self._clients.has_media_clients and self._media_socket is None:
      self._media_socket = self.messaging.sub_sock("carrotNaviMedia", conflate=False)

  def _poll_state(self) -> None:
    if self._state_socket is None:
      return
    payload = self._state_socket.receive(non_blocking=True)
    if payload is None:
      return
    state, wire = self._state_payload(payload)
    self._last_state = state
    self._last_state_wire = wire
    self._last_state_at = time.monotonic()
    self._state_count += 1
    self._error = ""
    self._clients.queue_state(wire)

  def _poll_media(self) -> None:
    if self._media_socket is None:
      return
    for _ in range(64):
      payload = self._media_socket.receive(non_blocking=True)
      if payload is None:
        break
      media = self._media_payload(payload)
      if media is None:
        continue
      metadata, wire, raw_payload = media
      self._media_count += 1
      self._error = ""
      self._media.remember(metadata, wire)
      for output_metadata, output_wire in self._media.outputs(
        metadata,
        wire,
        raw_payload,
        map_requested=self._clients.wants_map,
      ):
        self._clients.queue_media(output_metadata, output_wire)

  async def _poll_loop(self) -> None:
    idle_since = 0.0
    try:
      while True:
        if not self.stream_allowed():
          self._error = "Cluster HUD active"
          await self._clients.stop()
          self._clear_runtime_cache()
          break
        if not self._clients.has_clients:
          if idle_since == 0.0:
            idle_since = time.monotonic()
          elif time.monotonic() - idle_since >= self.IDLE_CLOSE_SECONDS:
            break
          await asyncio.sleep(0.03)
          continue
        idle_since = 0.0
        try:
          self._close_inactive_sockets()
          self._ensure_sockets()
          self._poll_state()
          self._poll_media()
        except Exception as exc:
          self._error = str(exc)[:256]
          await asyncio.sleep(0.1)
          continue
        await asyncio.sleep(self.POLL_SLEEP)
    finally:
      self._close_sockets()
      async with self._lock:
        if self._poll_task is asyncio.current_task():
          self._poll_task = None
