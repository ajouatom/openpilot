from __future__ import annotations

import asyncio
import logging
from typing import Any

from openpilot.cereal import messaging
from openpilot.selfdrive.carrot.realtime.compact_state import CARROT_STATE_SERVICES, encode_carrot_state_frame


MAX_CHANNEL_BUFFER_BYTES = 16 * 1024
THROTTLE_INTERVALS = {
  "carState": 1 / 30,
  "controlsState": 1 / 30,
  "carControl": 1 / 30,
  "roadCameraState": 0.25,
  "deviceState": 0.5,
  "peripheralState": 0.5,
  "gpsLocationExternal": 0.5,
  "selfdriveState": 0.2,
  "liveCalibration": 0.25,
  "liveParameters": 0.25,
  "liveTorqueParameters": 0.25,
  "liveDelay": 0.25,
}


class CarrotStateChannelProxy:
  """Compatibility bridge for non-Carrot WebRTC callers using carrot_state."""

  def __init__(self) -> None:
    self.sm = messaging.SubMaster(list(CARROT_STATE_SERVICES))
    self.channel: Any = None
    self.logger = logging.getLogger("webrtcd")
    self._last_send_time: dict[str, float] = {}
    self._sequence: dict[str, int] = {}

  def add_channel(self, channel: Any) -> None:
    self.channel = channel

  async def update(self) -> None:
    self.sm.update(0)
    channel = self.channel
    if channel is None or getattr(channel, "readyState", "") != "open":
      return

    now = asyncio.get_running_loop().time()
    for service in CARROT_STATE_SERVICES:
      if not self.sm.updated.get(service, False):
        continue
      interval = THROTTLE_INTERVALS.get(service, 0.0)
      if interval > 0 and now - self._last_send_time.get(service, 0.0) < interval:
        continue
      self._last_send_time[service] = now

      if int(getattr(channel, "bufferedAmount", 0) or 0) >= MAX_CHANNEL_BUFFER_BYTES:
        continue

      sequence = (self._sequence.get(service, 0) + 1) & 0xffff
      self._sequence[service] = sequence
      try:
        channel.send(encode_carrot_state_frame(service, self.sm[service], sequence))
      except Exception:
        self.logger.exception("Carrot Vision compatibility state send failed for %s", service)
