from __future__ import annotations

import asyncio
import threading

from openpilot.selfdrive.carrot.server.services.youtube_live import YouTubeLiveService
from openpilot.selfdrive.carrot.server.services.youtube_live_transport import LibrtmpClient


class ConnectedLibrary:
  @staticmethod
  def RTMP_IsConnected(_handle) -> int:
    return 1


def test_connection_probe_never_waits_for_an_active_write():
  client = object.__new__(LibrtmpClient)
  client._lock = threading.Lock()
  client._handle = 1
  client._library = ConnectedLibrary()
  client._bytes_written = 123

  assert client._lock.acquire(blocking=False)
  try:
    assert client.bytes_written == 123
    assert client.try_is_connected() is None
  finally:
    client._lock.release()

  assert client.try_is_connected() is True


def test_transport_failures_finish_in_backoff_after_cleanup():
  async def scenario() -> None:
    service = object.__new__(YouTubeLiveService)
    service._transport_connected = True
    service._last_error = ""
    calls: list[str] = []

    async def stop_stream() -> None:
      calls.append("stop")

    service._stop_stream = stop_stream
    service._schedule_backoff = lambda reason="": calls.append(f"schedule:{reason}")
    service._set_state = lambda state: calls.append(f"state:{state}")
    service._log = lambda message, level="info": calls.append(f"log:{level}:{message}")

    await service._enter_backoff("connection closed", reason="RTMPS connection closed")

    assert service._transport_connected is False
    assert service._last_error == "connection closed"
    assert calls == [
      "log:warn:connection closed",
      "stop",
      "schedule:RTMPS connection closed",
      "state:backoff",
    ]

  asyncio.run(scenario())
