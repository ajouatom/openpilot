from __future__ import annotations

import threading
from fractions import Fraction
from typing import Any, BinaryIO

from .youtube_h264 import avc_decoder_configuration, normalize_access_unit


H264_CODEC = "h264"
AAC_CODEC = "aac"
AUDIO_RATE = 44_100
AUDIO_BITRATE = 128_000
AUDIO_SAMPLES = 1_024


def _cfr_timestamp_ms(packet_index: int, fps: int) -> int:
  return max(0, int(packet_index)) * 1_000 // max(1, int(fps))


def pyav_capabilities() -> dict[str, Any]:
  try:
    import av
  except Exception as exc:
    return {
      "available": False,
      "version": "",
      "flv": False,
      "h264": False,
      "aac": False,
      "error": str(exc),
    }
  codecs = getattr(av, "codecs_available", set())
  return {
    "available": True,
    "version": str(getattr(av, "__version__", "")),
    "flv": True,
    "h264": H264_CODEC in codecs,
    "aac": AAC_CODEC in codecs,
    "error": "",
  }


class H264FlvMuxer:
  def __init__(
    self,
    output: BinaryIO,
    *,
    codec_header: bytes,
    fps: int = 20,
    width: int = 526,
    height: int = 330,
  ) -> None:
    if not codec_header:
      raise ValueError("H.264 codec header is required")

    import av

    self._av = av
    self._output = output
    self._fps = max(1, int(fps))
    self._video_time_base = Fraction(1, self._fps)
    self._audio_time_base = Fraction(1, AUDIO_RATE)
    self._video_config = avc_decoder_configuration(codec_header)
    self._audio_codec = av.CodecContext.create(AAC_CODEC, "w")
    self._audio_codec.sample_rate = AUDIO_RATE
    self._audio_codec.layout = "stereo"
    self._audio_codec.format = "fltp"
    self._audio_codec.bit_rate = AUDIO_BITRATE
    self._audio_codec.time_base = self._audio_time_base
    self._audio_codec.open()
    self._packet_index = 0
    self._audio_pts = 0
    self._last_video_ms = 0
    self._closed = False
    self._lock = threading.RLock()
    self._output.write(b"FLV\x01\x05\x00\x00\x00\x09\x00\x00\x00\x00")
    self._write_tag(9, 0, b"\x17\x00\x00\x00\x00" + self._video_config)
    audio_config = bytes(self._audio_codec.extradata or b"\x12\x10")
    self._write_tag(8, 0, b"\xAF\x00" + audio_config)

  def mux(self, payload: bytes, *, keyframe: bool = False, timestamp_ms: int | None = None) -> None:
    with self._lock:
      if self._closed:
        raise RuntimeError("FLV muxer is closed")
      if not payload:
        return
      access_unit = normalize_access_unit(payload)
      if self._packet_index == 0 and not access_unit.is_idr:
        raise ValueError("first H.264 access unit is not an IDR frame")
      if self._packet_index > 0 and keyframe and not access_unit.is_idr:
        raise ValueError("H.264 frame marked as keyframe has no IDR NAL")

      if timestamp_ms is None:
        video_ms = _cfr_timestamp_ms(self._packet_index, self._fps)
      else:
        video_ms = max(0, int(timestamp_ms))
      # Every video access unit needs a distinct, increasing FLV timestamp.
      if self._packet_index > 0 and video_ms <= self._last_video_ms:
        video_ms = self._last_video_ms + 1
      self._last_video_ms = video_ms

      # keep the silent audio track filled up to the current video time so a
      # dropped-frame gap stays A/V aligned
      self._mux_silence_until(int(video_ms * AUDIO_RATE / 1000))

      frame_header = b"\x17" if access_unit.is_idr else b"\x27"
      self._write_tag(9, video_ms, frame_header + b"\x01\x00\x00\x00" + access_unit.avcc)
      self._packet_index += 1

  def close(self) -> None:
    with self._lock:
      if self._closed:
        return
      self._closed = True
      try:
        self._mux_silence_until(int(self._last_video_ms * AUDIO_RATE / 1000))
        for packet in self._audio_codec.encode(None):
          self._write_audio_packet(packet)
      finally:
        self._output.flush()

  def _mux_silence_until(self, target_pts: int) -> None:
    while self._audio_pts <= target_pts:
      frame = self._av.AudioFrame(format="fltp", layout="stereo", samples=AUDIO_SAMPLES)
      frame.sample_rate = AUDIO_RATE
      frame.pts = self._audio_pts
      frame.time_base = self._audio_time_base
      for plane in frame.planes:
        plane.update(bytes(plane.buffer_size))
      for packet in self._audio_codec.encode(frame):
        self._write_audio_packet(packet)
      self._audio_pts += AUDIO_SAMPLES

  def _write_audio_packet(self, packet: Any) -> None:
    packet_pts = packet.pts if packet.pts is not None else self._audio_pts
    time_base = packet.time_base or self._audio_time_base
    timestamp_ms = max(0, int(packet_pts * time_base * 1000))
    self._write_tag(8, timestamp_ms, b"\xAF\x01" + bytes(packet))

  def _write_tag(self, tag_type: int, timestamp_ms: int, payload: bytes) -> None:
    timestamp = max(0, int(timestamp_ms)) & 0xFFFFFFFF
    header = (
      bytes((tag_type,))
      + len(payload).to_bytes(3, "big")
      + (timestamp & 0xFFFFFF).to_bytes(3, "big")
      + bytes(((timestamp >> 24) & 0xFF,))
      + b"\x00\x00\x00"
    )
    self._output.write(header + payload + (len(payload) + 11).to_bytes(4, "big"))
