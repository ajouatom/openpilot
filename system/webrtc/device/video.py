import asyncio

import av
from teleoprtc.tracks import TiciVideoStreamTrack

from cereal import messaging
from openpilot.common.realtime import DT_MDL, DT_DMON


class LiveStreamVideoStreamTrack(TiciVideoStreamTrack):
  camera_to_sock_mapping = {
    "driver": "livestreamDriverEncodeData",
    "wideRoad": "livestreamWideRoadEncodeData",
    "road": "livestreamRoadEncodeData",
  }

  def __init__(self, camera_type: str):
    dt = DT_DMON if camera_type == "driver" else DT_MDL
    super().__init__(camera_type, dt)

    self._sock = messaging.sub_sock(self.camera_to_sock_mapping[camera_type], conflate=True)
    self._pts = 0

  async def recv(self):
    waited = 0
    while True:
      try:
        msg = messaging.recv_one_or_none(self._sock)
        if msg is None:
          await asyncio.sleep(0.005)
          waited += 0.005
          if waited > 1.0:
            self._logger.warning("%s frame recv timed out", self.camera_to_sock_mapping.get(self._id.split(':', 1)[0], "video"))
            waited = 0
          continue

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
        self._logger.exception("failed to build outgoing video packet")
        await asyncio.sleep(0.01)

  def codec_preference(self) -> str | None:
    return "H264"
