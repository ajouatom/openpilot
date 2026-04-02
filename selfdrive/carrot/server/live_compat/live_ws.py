from __future__ import annotations

import asyncio
from dataclasses import dataclass, field

from aiohttp import web


@dataclass(eq=False)
class _LiveClient:
  ws: web.WebSocketResponse
  queue_size: int = 1
  queue: asyncio.Queue[bytes] = field(init=False)
  task: asyncio.Task | None = field(default=None, init=False)

  def __post_init__(self) -> None:
    self.queue = asyncio.Queue(maxsize=self.queue_size)

  def offer(self, payload: bytes) -> None:
    if self.queue.full():
      try:
        self.queue.get_nowait()
      except asyncio.QueueEmpty:
        pass
    try:
      self.queue.put_nowait(payload)
    except asyncio.QueueFull:
      pass


class LiveWsHub:
  def __init__(self) -> None:
    self._clients: set[_LiveClient] = set()

  def client_count(self) -> int:
    return len(self._clients)

  def has_clients(self) -> bool:
    return bool(self._clients)

  async def register(self, ws: web.WebSocketResponse, hello_payload: bytes | None = None) -> _LiveClient:
    client = _LiveClient(ws=ws)
    client.task = asyncio.create_task(self._writer(client))
    self._clients.add(client)
    if hello_payload is not None:
      client.offer(hello_payload)
    return client

  async def unregister(self, client: _LiveClient) -> None:
    self._clients.discard(client)
    if client.task is not None:
      client.task.cancel()
      try:
        await client.task
      except asyncio.CancelledError:
        pass
      except Exception:
        pass
    try:
      await client.ws.close()
    except Exception:
      pass

  async def close(self) -> None:
    for client in tuple(self._clients):
      await self.unregister(client)

  def publish(self, payload: bytes) -> None:
    for client in tuple(self._clients):
      client.offer(payload)

  async def _writer(self, client: _LiveClient) -> None:
    while True:
      try:
        payload = await client.queue.get()
        await client.ws.send_bytes(payload)
      except asyncio.CancelledError:
        raise
      except Exception:
        try:
          await client.ws.close(code=1011, message=b"live_send_failed")
        except Exception:
          pass
        break
