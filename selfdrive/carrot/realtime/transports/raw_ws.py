from __future__ import annotations

import asyncio
from typing import Any

from aiohttp import web

from ..raw_services import is_supported_raw_service


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
    self._lock = asyncio.Lock()

  def is_allowed_service(self, service: str) -> bool:
    return is_supported_raw_service(service)

  def client_count(self, service: str | None = None) -> int:
    if service is None:
      return sum(len(clients) for clients in self._clients.values())
    return len(self._clients.get(service, set()))

  async def register(self, service: str, ws: web.WebSocketResponse) -> None:
    self._clients.setdefault(service, set()).add(ws)
    self._send_failures.setdefault(service, {})
    await self.ensure_service_task(service)

  async def unregister(self, service: str, ws: web.WebSocketResponse) -> None:
    self._clients.get(service, set()).discard(ws)
    self._send_failures.get(service, {}).pop(ws, None)
    try:
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
    for service, clients in self._clients.items():
      for ws in tuple(clients):
        try:
          await ws.close()
        except Exception:
          pass
      clients.clear()
      self._send_failures.get(service, {}).clear()
    self._sockets.clear()

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

        stale: list[web.WebSocketResponse] = []
        client_list = list(clients)
        results = await asyncio.gather(
          *[asyncio.wait_for(ws.send_bytes(payload), timeout=self.SEND_TIMEOUT) for ws in client_list],
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
          self._clients.get(service, set()).discard(ws)
          failures.pop(ws, None)
          try:
            await ws.close(code=1011, message=b"raw_send_timeout")
          except Exception:
            pass
    except asyncio.CancelledError:
      raise
    finally:
      async with self._lock:
        current = self._tasks.get(service)
        if current is asyncio.current_task():
          self._tasks.pop(service, None)
      self._sockets.pop(service, None)
      self._send_failures.pop(service, None)
