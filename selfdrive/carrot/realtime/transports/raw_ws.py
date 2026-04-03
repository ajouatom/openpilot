from __future__ import annotations

import asyncio
from typing import Any

from aiohttp import web

from ..raw_services import is_supported_raw_service
from ..raw_protocol import encode_raw_multiplex_frame


class RawWsHub:
  SEND_TIMEOUT = 0.35
  IDLE_SLEEP = 0.03
  ACTIVE_POLL_SLEEP = 0.008
  IDLE_STOP_SEC = 5.0
  FAILURE_THRESHOLD = 3

  def __init__(self, messaging: Any) -> None:
    self.messaging = messaging
    self._clients: dict[str, set[web.WebSocketResponse]] = {}
    self._tasks: dict[str, asyncio.Task] = {}
    self._sockets: dict[str, Any] = {}
    self._send_failures: dict[str, dict[web.WebSocketResponse, int]] = {}
    self._last_send_time: dict[str, float] = {}
    self._ws_modes: dict[web.WebSocketResponse, str] = {}
    self._ws_services: dict[web.WebSocketResponse, set[str]] = {}
    self._ws_send_locks: dict[web.WebSocketResponse, asyncio.Lock] = {}
    self._lock = asyncio.Lock()

  def is_allowed_service(self, service: str) -> bool:
    return is_supported_raw_service(service)

  # Phase 2-2: per-service throttle intervals (seconds)
  # 0 = no throttle (send every message)
  _THROTTLE_MAP = {
    "modelV2": 0,             # camera-synced, don't throttle
    "roadCameraState": 0.25,  # metadata/debug only on web HUD
    "deviceState": 0.5,       # slow-changing HUD stats
    "peripheralState": 0.5,
    "gpsLocationExternal": 0.5,
    "selfdriveState": 0.2,
    "liveCalibration": 0.25,  # slow-changing
    "liveParameters": 0.25,
    "liveTorqueParameters": 0.25,
    "liveDelay": 0.25,
  }
  _THROTTLE_DEFAULT = 0.05  # 20Hz for everything else

  def _throttle_interval(self, service: str) -> float:
    return self._THROTTLE_MAP.get(service, self._THROTTLE_DEFAULT)

  def client_count(self, service: str | None = None) -> int:
    if service is None:
      return sum(len(clients) for clients in self._clients.values())
    return len(self._clients.get(service, set()))

  async def register(self, service: str, ws: web.WebSocketResponse) -> None:
    self._clients.setdefault(service, set()).add(ws)
    self._send_failures.setdefault(service, {})
    self._ws_modes.setdefault(ws, "single")
    self._ws_services.setdefault(ws, set()).add(service)
    self._ws_send_locks.setdefault(ws, asyncio.Lock())
    await self.ensure_service_task(service)

  async def register_many(self, services: list[str], ws: web.WebSocketResponse) -> None:
    unique_services = [service for service in dict.fromkeys(services) if service]
    self._ws_modes[ws] = "multiplex"
    self._ws_services[ws] = set(unique_services)
    self._ws_send_locks.setdefault(ws, asyncio.Lock())
    for service in unique_services:
      self._clients.setdefault(service, set()).add(ws)
      self._send_failures.setdefault(service, {})
      await self.ensure_service_task(service)

  async def unregister_client(self, ws: web.WebSocketResponse, *, close_code: int | None = None, close_message: bytes | None = None) -> None:
    services = self._ws_services.pop(ws, set())
    for service in services:
      self._clients.get(service, set()).discard(ws)
      self._send_failures.get(service, {}).pop(ws, None)
    self._ws_modes.pop(ws, None)
    self._ws_send_locks.pop(ws, None)
    try:
      if close_code is not None:
        await ws.close(code=close_code, message=close_message or b"")
      else:
        await ws.close()
    except Exception:
      pass

  async def ensure_service_task(self, service: str) -> None:
    async with self._lock:
      task = self._tasks.get(service)
      if task is None or task.done():
        self._tasks[service] = asyncio.create_task(self._service_loop(service))

  async def stop_all(self) -> None:
    async with self._lock:
      tasks = list(self._tasks.values())
      self._tasks = {}
    for task in tasks:
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    all_clients: set[web.WebSocketResponse] = set(self._ws_services.keys())
    for clients in self._clients.values():
      all_clients.update(clients)
    for ws in tuple(all_clients):
      try:
        await ws.close()
      except Exception:
        pass
    for clients in self._clients.values():
      clients.clear()
    self._ws_modes.clear()
    self._ws_services.clear()
    self._ws_send_locks.clear()
    self._send_failures.clear()
    self._sockets.clear()

  async def _send_payload(self, service: str, ws: web.WebSocketResponse, payload: bytes) -> None:
    lock = self._ws_send_locks.setdefault(ws, asyncio.Lock())
    mode = self._ws_modes.get(ws, "single")
    wire_payload = encode_raw_multiplex_frame(service=service, payload=payload) if mode == "multiplex" else payload
    async with lock:
      await ws.send_bytes(wire_payload)

  async def _service_loop(self, service: str) -> None:
    idle_started_at = 0.0
    try:
      while True:
        clients = self._clients.get(service, set())
        if not clients:
          if idle_started_at <= 0.0:
            idle_started_at = asyncio.get_running_loop().time()
          elif (asyncio.get_running_loop().time() - idle_started_at) >= self.IDLE_STOP_SEC:
            break
          await asyncio.sleep(self.IDLE_SLEEP)
          continue

        idle_started_at = 0.0
        sock = self._sockets.get(service)
        if sock is None:
          try:
            sock = self.messaging.sub_sock(service, conflate=True)
            self._sockets[service] = sock
          except Exception:
            await asyncio.sleep(self.IDLE_SLEEP)
            continue

        try:
          payload = sock.receive(non_blocking=True)
        except Exception:
          await asyncio.sleep(self.ACTIVE_POLL_SLEEP)
          continue
        if payload is None:
          await asyncio.sleep(self.ACTIVE_POLL_SLEEP)
          continue

        # Phase 2-2: per-service throttle — skip if too soon since last send
        now = asyncio.get_running_loop().time()
        interval = self._throttle_interval(service)
        last_send = self._last_send_time.get(service, 0.0)
        if interval > 0 and (now - last_send) < interval:
          await asyncio.sleep(self.ACTIVE_POLL_SLEEP)
          continue
        self._last_send_time[service] = now

        stale: list[web.WebSocketResponse] = []
        client_list = list(clients)
        results = await asyncio.gather(
          *[asyncio.wait_for(self._send_payload(service, ws, payload), timeout=self.SEND_TIMEOUT) for ws in client_list],
          return_exceptions=True,
        )
        failures = self._send_failures.setdefault(service, {})
        for ws, result in zip(client_list, results):
          if not isinstance(result, Exception):
            failures.pop(ws, None)
            continue
          fail_count = failures.get(ws, 0) + 1
          failures[ws] = fail_count
          if fail_count >= self.FAILURE_THRESHOLD:
            stale.append(ws)

        for ws in stale:
          await self.unregister_client(ws, close_code=1011, close_message=b"raw_send_timeout")
    except asyncio.CancelledError:
      raise
    finally:
      async with self._lock:
        current = self._tasks.get(service)
        if current is asyncio.current_task():
          self._tasks.pop(service, None)
      self._sockets.pop(service, None)
      self._send_failures.pop(service, None)
