from __future__ import annotations

import asyncio
from collections import OrderedDict
from typing import Any

from aiohttp import web

from openpilot.cereal import log
from openpilot.selfdrive.carrot.realtime.compact_state import (
  CARROT_STATE_SERVICES,
  COMPACT_BATCH_WINDOW_SECONDS,
  compact_service_interval,
  encode_carrot_state_batch,
  encode_carrot_state_frame,
)

try:
  from openpilot.selfdrive.carrot.realtime.compact_state_pyx import encode_frame as encode_compact_frame_native
except ImportError:
  encode_compact_frame_native = None

from ..raw_services import is_supported_raw_service
from ..raw_protocol import encode_raw_multiplex_frame


class RawWsHub:
  """One latest-only cereal relay shared by every Carrot Web client.

  The server only selects/serializes display fields. Projection, lane/path
  construction, and drawing remain browser work. Legacy raw capnp endpoints
  remain diagnostic-only; all modes share these sockets and this poll loop so
  viewers never multiply device-side cereal readers.
  """

  SEND_TIMEOUT = 0.75
  COMPACT_BATCH_WINDOW = COMPACT_BATCH_WINDOW_SECONDS
  MIN_POLL_SLEEP = 0.002
  EMPTY_READ_RETRY = 0.01
  IDLE_SLEEP = 0.03
  IDLE_STOP_SEC = 5.0

  def __init__(self, messaging: Any) -> None:
    self.messaging = messaging
    self._clients: dict[str, set[web.WebSocketResponse]] = {}
    self._sockets: dict[str, Any] = {}
    self._next_read_time: dict[str, float] = {}
    self._sequence: dict[str, int] = {}
    self._ws_modes: dict[web.WebSocketResponse, str] = {}
    self._ws_services: dict[web.WebSocketResponse, set[str]] = {}
    self._ws_pending: dict[web.WebSocketResponse, OrderedDict[str, bytes]] = {}
    self._ws_sender_events: dict[web.WebSocketResponse, asyncio.Event] = {}
    self._ws_sender_tasks: dict[web.WebSocketResponse, asyncio.Task] = {}
    self._poll_task: asyncio.Task | None = None
    self._lock = asyncio.Lock()

  def is_allowed_service(self, service: str) -> bool:
    return is_supported_raw_service(service)

  def is_compact_service(self, service: str) -> bool:
    return service in CARROT_STATE_SERVICES

  def _throttle_interval(self, service: str) -> float:
    return compact_service_interval(service)

  def _next_read_delay(self, interval: float, *, payload_received: bool) -> float:
    """Keep a missed poll from consuming a complete display interval.

    Cereal publishers and this shared poll loop are not phase locked. Advancing
    a 20 Hz service by another 50 ms when receive() returns None can therefore
    discard every other model/lateralPlan frame. A short bounded retry catches
    the next publication without restoring the old continuous 4 ms polling.
    """
    cadence = max(self.MIN_POLL_SLEEP, float(interval))
    return cadence if payload_received else min(cadence, self.EMPTY_READ_RETRY)

  def client_count(self, service: str | None = None) -> int:
    if service is None:
      return len(self._ws_services)
    return len(self._clients.get(service, set()))

  async def register(self, service: str, ws: web.WebSocketResponse) -> None:
    await self._register_many([service], ws, mode="single")

  async def register_many(self, services: list[str], ws: web.WebSocketResponse) -> None:
    await self._register_many(services, ws, mode="multiplex")

  async def register_compact_many(self, services: list[str], ws: web.WebSocketResponse) -> None:
    await self._register_many(services, ws, mode="compact")

  async def _register_many(self, services: list[str], ws: web.WebSocketResponse, *, mode: str) -> None:
    unique_services = [service for service in dict.fromkeys(services) if service]
    self._ws_modes[ws] = mode
    self._ws_services[ws] = set(unique_services)
    self._ws_pending.setdefault(ws, OrderedDict())
    self._ws_sender_events.setdefault(ws, asyncio.Event())
    for service in unique_services:
      self._clients.setdefault(service, set()).add(ws)
    await self._ensure_poll_task()

  async def unregister_client(self, ws: web.WebSocketResponse, *, close_code: int | None = None,
                              close_message: bytes | None = None) -> None:
    services = self._ws_services.pop(ws, set())
    for service in services:
      self._clients.get(service, set()).discard(ws)
    self._ws_modes.pop(ws, None)
    self._ws_pending.pop(ws, None)
    self._ws_sender_events.pop(ws, None)
    sender_task = self._ws_sender_tasks.pop(ws, None)
    if sender_task is not None and sender_task is not asyncio.current_task():
      sender_task.cancel()
      try:
        await sender_task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    try:
      if close_code is not None:
        await ws.close(code=close_code, message=close_message or b"")
      else:
        await ws.close()
    except Exception:
      pass

  async def _ensure_poll_task(self) -> None:
    async with self._lock:
      if self._poll_task is None or self._poll_task.done():
        self._poll_task = asyncio.create_task(self._poll_loop())

  async def stop_all(self) -> None:
    async with self._lock:
      poll_task = self._poll_task
      self._poll_task = None
    if poll_task is not None:
      poll_task.cancel()
      try:
        await poll_task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass

    all_clients = set(self._ws_services)
    for clients in self._clients.values():
      all_clients.update(clients)
    for ws in tuple(all_clients):
      await self.unregister_client(ws)
    for clients in self._clients.values():
      clients.clear()
    self._close_sockets()
    self._ws_modes.clear()
    self._ws_services.clear()
    self._ws_pending.clear()
    self._ws_sender_events.clear()
    self._ws_sender_tasks.clear()
    self._next_read_time.clear()
    self._sequence.clear()

  def _close_sockets(self, services: set[str] | None = None) -> None:
    selected = set(self._sockets) if services is None else services
    for service in selected:
      sock = self._sockets.pop(service, None)
      if sock is not None:
        try:
          sock.close()
        except Exception:
          pass
      self._next_read_time.pop(service, None)

  def _queue_wire_payload(self, service: str, ws: web.WebSocketResponse, wire_payload: bytes) -> None:
    if ws not in self._ws_services:
      return
    pending = self._ws_pending.setdefault(ws, OrderedDict())
    pending[service] = wire_payload
    pending.move_to_end(service)
    self._ws_sender_events.setdefault(ws, asyncio.Event()).set()
    task = self._ws_sender_tasks.get(ws)
    if task is None or task.done():
      self._ws_sender_tasks[ws] = asyncio.create_task(self._sender_loop(ws))

  async def _sender_loop(self, ws: web.WebSocketResponse) -> None:
    try:
      while ws in self._ws_services and not ws.closed:
        pending = self._ws_pending.get(ws)
        event = self._ws_sender_events.get(ws)
        if pending is None or event is None:
          break
        if not pending:
          event.clear()
          if pending:
            continue
          await event.wait()
          continue

        if self._ws_modes.get(ws) == "compact":
          # Coalesce the services that changed during one display tick. Each
          # service remains latest-only, but one websocket write wakes the
          # browser once instead of once per cereal service.
          await asyncio.sleep(self.COMPACT_BATCH_WINDOW)
          if not pending:
            continue
          wire_payload = encode_carrot_state_batch(tuple(pending.values()))
          pending.clear()
        else:
          _, wire_payload = pending.popitem(last=False)
        try:
          await asyncio.wait_for(ws.send_bytes(wire_payload), timeout=self.SEND_TIMEOUT)
        except Exception:
          await self.unregister_client(ws, close_code=1011, close_message=b"state_send_timeout")
          break
    except asyncio.CancelledError:
      raise
    finally:
      if self._ws_sender_tasks.get(ws) is asyncio.current_task():
        self._ws_sender_tasks.pop(ws, None)

  def _compact_frame(self, service: str, payload: bytes) -> bytes | None:
    sequence = (self._sequence.get(service, 0) + 1) & 0xffff
    try:
      if encode_compact_frame_native is not None:
        try:
          frame = encode_compact_frame_native(service, payload, sequence)
        except Exception:
          with log.Event.from_bytes(payload, traversal_limit_in_words=2**64 - 1) as event:
            frame = encode_carrot_state_frame(service, getattr(event, service), sequence)
      else:
        with log.Event.from_bytes(payload, traversal_limit_in_words=2**64 - 1) as event:
          frame = encode_carrot_state_frame(service, getattr(event, service), sequence)
    except Exception:
      return None
    self._sequence[service] = sequence
    return frame

  def _broadcast_payload(self, service: str, payload: bytes, clients: set[web.WebSocketResponse]) -> None:
    single_clients: list[web.WebSocketResponse] = []
    multiplex_clients: list[web.WebSocketResponse] = []
    compact_clients: list[web.WebSocketResponse] = []
    for ws in list(clients):
      mode = self._ws_modes.get(ws)
      if mode == "single":
        single_clients.append(ws)
      elif mode == "multiplex":
        multiplex_clients.append(ws)
      elif mode == "compact":
        compact_clients.append(ws)

    for ws in single_clients:
      self._queue_wire_payload(service, ws, payload)

    if multiplex_clients:
      multiplex_payload = encode_raw_multiplex_frame(service=service, payload=payload)
      for ws in multiplex_clients:
        self._queue_wire_payload(service, ws, multiplex_payload)

    if compact_clients:
      compact_payload = self._compact_frame(service, payload)
      if compact_payload is not None:
        for ws in compact_clients:
          self._queue_wire_payload(service, ws, compact_payload)

  async def _poll_loop(self) -> None:
    idle_started_at = 0.0
    try:
      while True:
        active_services = {service for service, clients in self._clients.items() if clients}
        if not active_services:
          if idle_started_at <= 0.0:
            idle_started_at = asyncio.get_running_loop().time()
          elif asyncio.get_running_loop().time() - idle_started_at >= self.IDLE_STOP_SEC:
            break
          await asyncio.sleep(self.IDLE_SLEEP)
          continue

        idle_started_at = 0.0
        inactive_services = set(self._sockets) - active_services
        if inactive_services:
          self._close_sockets(inactive_services)

        now = asyncio.get_running_loop().time()
        for service in active_services:
          # Sockets are conflated, so reading faster than the fixed display
          # cadence only copies samples that are immediately discarded. Read
          # at that cadence and let msgq retain the newest sample in between.
          if now < self._next_read_time.get(service, 0.0):
            continue
          interval = self._throttle_interval(service)

          sock = self._sockets.get(service)
          if sock is None:
            try:
              sock = self.messaging.sub_sock(service, conflate=True)
              self._sockets[service] = sock
            except Exception:
              self._next_read_time[service] = now + self.IDLE_SLEEP
              continue

          try:
            payload = sock.receive(non_blocking=True)
          except Exception:
            self._next_read_time[service] = now + self.EMPTY_READ_RETRY
            continue
          if payload is None:
            self._next_read_time[service] = now + self._next_read_delay(interval, payload_received=False)
            continue
          self._next_read_time[service] = now + self._next_read_delay(interval, payload_received=True)
          self._broadcast_payload(service, payload, self._clients.get(service, set()))

        next_due = min(
          (self._next_read_time.get(service, now + self.IDLE_SLEEP) for service in active_services),
          default=now + self.IDLE_SLEEP,
        )
        sleep_for = max(self.MIN_POLL_SLEEP, min(self.IDLE_SLEEP, next_due - asyncio.get_running_loop().time()))
        await asyncio.sleep(sleep_for)
    except asyncio.CancelledError:
      raise
    finally:
      self._close_sockets()
      async with self._lock:
        if self._poll_task is asyncio.current_task():
          self._poll_task = None
