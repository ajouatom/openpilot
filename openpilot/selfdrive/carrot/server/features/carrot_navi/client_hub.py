from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Callable

from aiohttp import web

from .protocol import (
  SESSION_ACCEPTED_CODE,
  SESSION_BUSY_CLOSE_CODE,
  SESSION_BUSY_CODE,
  SESSION_REPLACED_CLOSE_CODE,
  SESSION_REPLACED_CODE,
  encode_session_status,
)


@dataclass
class _Client:
  ws: web.WebSocketResponse
  queue: asyncio.Queue[bytes | str]
  client_id: str
  include_map: bool = True
  sender: asyncio.Task | None = None


class CarrotNaviClientHub:
  """Owns websocket clients, backpressure queues, and sender tasks."""

  def __init__(
    self,
    *,
    media_queue_size: int = 12,
    send_timeout: float = 1.0,
    on_map_demand_lost: Callable[[], Any] | None = None,
  ) -> None:
    self._media_queue_size = max(1, int(media_queue_size))
    self._send_timeout = max(0.1, float(send_timeout))
    self._on_map_demand_lost = on_map_demand_lost
    self._state_clients: dict[web.WebSocketResponse, _Client] = {}
    self._media_clients: dict[web.WebSocketResponse, _Client] = {}
    self._owner_client_id = ""

  @property
  def state_count(self) -> int:
    return len(self._state_clients)

  @property
  def media_count(self) -> int:
    return len(self._media_clients)

  @property
  def has_clients(self) -> bool:
    return bool(self._state_clients or self._media_clients)

  @property
  def has_media_clients(self) -> bool:
    return bool(self._media_clients)

  @property
  def wants_map(self) -> bool:
    return any(client.include_map for client in self._media_clients.values())

  def _detach_foreign_clients(self, client_id: str) -> list[_Client]:
    detached: list[_Client] = []
    for clients in (self._state_clients, self._media_clients):
      for ws, client in tuple(clients.items()):
        if client.client_id == client_id:
          continue
        clients.pop(ws, None)
        detached.append(client)
    return detached

  async def _close_client(self, client: _Client, *, code: int | None = None, reason: str = "") -> None:
    if client.sender is not None and client.sender is not asyncio.current_task():
      client.sender.cancel()
      try:
        await client.sender
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    if not client.ws.closed:
      try:
        if code is None:
          await client.ws.close()
        else:
          await client.ws.close(code=code, message=reason.encode("utf-8"))
      except Exception:
        pass

  async def _claim(self, ws: web.WebSocketResponse, client_id: str, takeover: bool) -> tuple[bool, list[_Client]]:
    if self._owner_client_id and self._owner_client_id != client_id and not takeover:
      try:
        await ws.send_str(encode_session_status("busy", SESSION_BUSY_CODE))
      except Exception:
        pass
      try:
        await ws.close(code=SESSION_BUSY_CLOSE_CODE, message=SESSION_BUSY_CODE.encode("utf-8"))
      except Exception:
        pass
      return False, []

    replaced: list[_Client] = []
    if self._owner_client_id and self._owner_client_id != client_id:
      replaced = self._detach_foreign_clients(client_id)
    self._owner_client_id = client_id
    return True, replaced

  async def _accept(self, ws: web.WebSocketResponse) -> bool:
    try:
      await ws.send_str(encode_session_status("accepted", SESSION_ACCEPTED_CODE))
      return True
    except Exception:
      return False

  async def _close_replaced(self, clients: list[_Client]) -> None:
    for client in clients:
      await self._close_client(
        client,
        code=SESSION_REPLACED_CLOSE_CODE,
        reason=SESSION_REPLACED_CODE,
      )
    if clients and not self.wants_map and self._on_map_demand_lost is not None:
      self._on_map_demand_lost()

  async def register_state(
    self,
    ws: web.WebSocketResponse,
    client_id: str,
    *,
    takeover: bool = False,
    initial_wire: str | None = None,
  ) -> bool:
    allowed, replaced = await self._claim(ws, client_id, takeover)
    if not allowed:
      return False
    client = _Client(ws, asyncio.Queue(maxsize=2), client_id=client_id, include_map=False)
    if initial_wire is not None:
      client.queue.put_nowait(initial_wire)
    self._state_clients[ws] = client
    if not await self._accept(ws):
      await self.unregister(ws)
      await self._close_replaced(replaced)
      return False
    client.sender = asyncio.create_task(self._sender(client))
    await self._close_replaced(replaced)
    return True

  async def register_media(
    self,
    ws: web.WebSocketResponse,
    client_id: str,
    *,
    include_map: bool,
    bootstrap: list[bytes],
  ) -> bool:
    allowed, replaced = await self._claim(ws, client_id, False)
    if not allowed:
      return False
    client = _Client(ws, asyncio.Queue(maxsize=self._media_queue_size), client_id=client_id, include_map=include_map)
    for wire in bootstrap:
      if client.queue.full():
        break
      client.queue.put_nowait(wire)
    self._media_clients[ws] = client
    if not await self._accept(ws):
      await self.unregister(ws)
      await self._close_replaced(replaced)
      return False
    client.sender = asyncio.create_task(self._sender(client))
    await self._close_replaced(replaced)
    return True

  async def unregister(self, ws: web.WebSocketResponse) -> None:
    client = self._state_clients.pop(ws, None) or self._media_clients.pop(ws, None)
    if client is not None:
      await self._close_client(client)
    if client is not None and client.include_map and not self.wants_map:
      if self._on_map_demand_lost is not None:
        self._on_map_demand_lost()
    if self._owner_client_id and not any(
      item.client_id == self._owner_client_id
      for item in (*self._state_clients.values(), *self._media_clients.values())
    ):
      self._owner_client_id = ""

  async def stop(self) -> None:
    for ws in tuple(set(self._state_clients) | set(self._media_clients)):
      await self.unregister(ws)

  async def _sender(self, client: _Client) -> None:
    try:
      while not client.ws.closed:
        wire = await client.queue.get()
        operation = client.ws.send_bytes(wire) if isinstance(wire, bytes) else client.ws.send_str(wire)
        await asyncio.wait_for(operation, timeout=self._send_timeout)
    except asyncio.CancelledError:
      raise
    except Exception:
      pass
    finally:
      if client.ws in self._state_clients or client.ws in self._media_clients:
        asyncio.create_task(self.unregister(client.ws))

  def queue_state(self, wire: str) -> None:
    for client in tuple(self._state_clients.values()):
      while client.queue.full():
        try:
          client.queue.get_nowait()
        except asyncio.QueueEmpty:
          break
      client.queue.put_nowait(wire)

  def queue_media(self, metadata: dict[str, Any], wire: bytes) -> None:
    is_map = metadata.get("kind") in ("render", "fmp4") and metadata.get("name") == "map_main"
    for client in tuple(self._media_clients.values()):
      if is_map and not client.include_map:
        continue
      if client.queue.full():
        asyncio.create_task(client.ws.close(code=1013, message=b"carrot_navi_client_slow"))
        continue
      client.queue.put_nowait(wire)
