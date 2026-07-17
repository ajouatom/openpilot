import asyncio
import time

import av
from teleoprtc.tracks import TiciVideoStreamTrack

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL, DT_DMON
from openpilot.common.swaglog import cloudlog


class LiveStreamVideoStreamTrack(TiciVideoStreamTrack):
  RECOVERY_LOG_THRESHOLD_SECONDS = 1.0

  camera_to_sock_mapping = {
    "driver": "livestreamDriverEncodeData",
    "wideRoad": "livestreamWideRoadEncodeData",
    "road": "livestreamRoadEncodeData",
  }

  def __init__(self, camera_type: str, *, use_source_frame_timestamps: bool = False):
    dt = DT_DMON if camera_type == "driver" else DT_MDL
    super().__init__(camera_type, dt)

    self._source_name = self.camera_to_sock_mapping[camera_type]
    self._sock = messaging.sub_sock(self._source_name, conflate=True)
    self._pts = 0
    self._sock_closed = False
    self._timeout_started_at = None
    self._use_source_frame_timestamps = camera_type == "road" and bool(use_source_frame_timestamps)
    self._frame_sync_sender = None
    self._frame_sync_callback = None
    self._last_source_frame_id = None
    self._last_sync_frame_id = None

  def bind_frame_sync(self, sender, callback) -> None:
    """Publish the RTP timestamp for a source frame after aiortc sent it."""
    self._frame_sync_sender = sender
    self._frame_sync_callback = callback

  def _publish_completed_frame_sync(self) -> None:
    frame_id = self._last_source_frame_id
    if frame_id is None or frame_id == self._last_sync_frame_id:
      return

    sender = self._frame_sync_sender
    callback = self._frame_sync_callback
    if sender is None or callback is None:
      return

    # aiortc updates this after every encoded frame is fully sent. recv() for
    # the next frame is therefore the first public-track point where the RTP
    # timestamp and source frame ID are guaranteed to describe the same frame.
    rtp_timestamp = getattr(sender, "_RTCRtpSender__rtp_timestamp", None)
    if rtp_timestamp is None:
      return

    self._last_sync_frame_id = frame_id
    try:
      callback(int(frame_id), int(rtp_timestamp) & 0xFFFFFFFF)
    except Exception:
      self._logger.exception("failed to publish frame sync (%s frame=%s)", self._source_name, frame_id)

  async def recv(self):
    self._publish_completed_frame_sync()
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
        frame_id = int(evta.idx.frameId)
        packet_pts = int(frame_id * self._dt * self._clock_rate) if self._use_source_frame_timestamps else self._pts
        packet.pts = packet_pts

        self.log_debug("track sending frame %s source frame %s", packet_pts, frame_id)
        self._pts = packet_pts + self._dt * self._clock_rate
        self._last_source_frame_id = frame_id

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
    self._frame_sync_sender = None
    self._frame_sync_callback = None

  def codec_preference(self) -> str | None:
    return "H264"
