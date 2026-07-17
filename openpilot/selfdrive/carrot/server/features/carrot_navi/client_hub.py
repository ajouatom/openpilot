from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Callable

from aiohttp import web


@dataclass
class _Client:
  ws: web.WebSocketResponse
  queue: asyncio.Queue[bytes | str]
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

  async def register_state(self, ws: web.WebSocketResponse, initial_wire: str | None = None) -> None:
    client = _Client(ws, asyncio.Queue(maxsize=2), include_map=False)
    client.sender = asyncio.create_task(self._sender(client))
    self._state_clients[ws] = client
    if initial_wire is not None:
      client.queue.put_nowait(initial_wire)

  async def register_media(
    self,
    ws: web.WebSocketResponse,
    *,
    include_map: bool,
    bootstrap: list[bytes],
  ) -> None:
    client = _Client(ws, asyncio.Queue(maxsize=self._media_queue_size), include_map=include_map)
    client.sender = asyncio.create_task(self._sender(client))
    self._media_clients[ws] = client
    for wire in bootstrap:
      if client.queue.full():
        break
      client.queue.put_nowait(wire)

  async def unregister(self, ws: web.WebSocketResponse) -> None:
    client = self._state_clients.pop(ws, None) or self._media_clients.pop(ws, None)
    if client is not None and client.sender is not None and client.sender is not asyncio.current_task():
      client.sender.cancel()
      try:
        await client.sender
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    if client is not None and client.include_map and not self.wants_map:
      if self._on_map_demand_lost is not None:
        self._on_map_demand_lost()
    if not ws.closed:
      try:
        await ws.close()
      except Exception:
        pass

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
