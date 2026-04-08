import asyncio
import time

import av
from teleoprtc.tracks import TiciVideoStreamTrack

from cereal import messaging
from openpilot.common.realtime import DT_MDL, DT_DMON
from openpilot.common.swaglog import cloudlog


class LiveStreamVideoStreamTrack(TiciVideoStreamTrack):
  RECOVERY_LOG_THRESHOLD_SECONDS = 1.0

  camera_to_sock_mapping = {
    "driver": "livestreamDriverEncodeData",
    "wideRoad": "livestreamWideRoadEncodeData",
    "road": "livestreamRoadEncodeData",
  }

  def __init__(self, camera_type: str):
    dt = DT_DMON if camera_type == "driver" else DT_MDL
    super().__init__(camera_type, dt)

    self._source_name = self.camera_to_sock_mapping[camera_type]
    self._sock = messaging.sub_sock(self._source_name, conflate=True)
    self._pts = 0
    self._sock_closed = False
    self._timeout_started_at = None

  async def recv(self):
    waited = 0
    while True:
      try:
        msg = messaging.recv_one_or_none(self._sock)
        if msg is None:
          await asyncio.sleep(0.005)
          waited += 0.005
          if self._timeout_started_at is None:
            self._timeout_started_at = time.monotonic()
          if waited > 1.0:
            elapsed = time.monotonic() - self._timeout_started_at if self._timeout_started_at is not None else waited
            self._logger.warning("%s frame recv timed out (elapsed=%.2fs pts=%s sock_closed=%s)",
                                 self._source_name, elapsed, self._pts, self._sock_closed)
            cloudlog.warning("[webrtcd-video] %s frame recv timed out (elapsed=%.2fs pts=%s sock_closed=%s)",
                             self._source_name, elapsed, self._pts, self._sock_closed)
            waited = 0
          continue

        if self._timeout_started_at is not None:
          elapsed = time.monotonic() - self._timeout_started_at
          if elapsed >= self.RECOVERY_LOG_THRESHOLD_SECONDS:
            self._logger.info("%s frame recv recovered after %.2fs (pts=%s)", self._source_name, elapsed, self._pts)
            cloudlog.info("[webrtcd-video] %s frame recv recovered after %.2fs (pts=%s)", self._source_name, elapsed, self._pts)
          self._timeout_started_at = None
        waited = 0
        evta = getattr(msg, msg.which())

        packet = av.Packet(evta.header + evta.data)
        packet.time_base = self._time_base
        packet.pts = self._pts

        self.log_debug("track sending frame %s", self._pts)
        self._pts += self._dt * self._clock_rate

        return packet
      except asyncio.CancelledError:
        raise
      except Exception:
        self._logger.exception("failed to build outgoing video packet (%s pts=%s)", self._source_name, self._pts)
        cloudlog.exception("[webrtcd-video] failed to build outgoing video packet (%s pts=%s)", self._source_name, self._pts)
        await asyncio.sleep(0.01)

  def close_sock(self):
    if not self._sock_closed and self._sock is not None:
      try:
        self._sock.close()
      except Exception:
        pass
      self._sock_closed = True

  def codec_preference(self) -> str | None:
    return "H264"
