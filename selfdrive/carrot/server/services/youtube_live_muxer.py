from __future__ import annotations

import threading
from fractions import Fraction
from typing import Any, BinaryIO


FLV_FORMAT = "flv"
H264_CODEC = "h264"
AAC_CODEC = "aac"
AUDIO_RATE = 44_100
AUDIO_BITRATE = 128_000
AUDIO_SAMPLES = 1_024


def _annexb_nalus(payload: bytes) -> list[bytes]:
  starts: list[tuple[int, int]] = []
  index = 0
  size = len(payload)
  while index + 3 <= size:
    if index + 4 <= size and payload[index:index + 4] == b"\x00\x00\x00\x01":
      starts.append((index, 4))
      index += 4
    elif payload[index:index + 3] == b"\x00\x00\x01":
      starts.append((index, 3))
      index += 3
    else:
      index += 1

  nalus = []
  for position, (offset, start_size) in enumerate(starts):
    start = offset + start_size
    end = starts[position + 1][0] if position + 1 < len(starts) else size
    while end > start and payload[end - 1] == 0:
      end -= 1
    if end > start:
      nalus.append(payload[start:end])
  return nalus


def _avc_decoder_configuration(codec_header: bytes) -> bytes:
  if codec_header[:1] == b"\x01" and not codec_header.startswith((b"\x00\x00\x01", b"\x00\x00\x00\x01")):
    return bytes(codec_header)
  nalus = _annexb_nalus(codec_header)
  sps_units = [nalu for nalu in nalus if nalu and nalu[0] & 0x1F == 7]
  pps_units = [nalu for nalu in nalus if nalu and nalu[0] & 0x1F == 8]
  if not sps_units or not pps_units or len(sps_units[0]) < 4:
    raise ValueError("H.264 codec header has no SPS/PPS")
  if len(sps_units) > 31 or len(pps_units) > 255:
    raise ValueError("H.264 codec header has too many parameter sets")

  first_sps = sps_units[0]
  result = bytearray((1, first_sps[1], first_sps[2], first_sps[3], 0xFF, 0xE0 | len(sps_units)))
  for sps in sps_units:
    result.extend(len(sps).to_bytes(2, "big"))
    result.extend(sps)
  result.append(len(pps_units))
  for pps in pps_units:
    result.extend(len(pps).to_bytes(2, "big"))
    result.extend(pps)
  return bytes(result)


def _annexb_to_avcc(payload: bytes) -> bytes:
  nalus = _annexb_nalus(payload)
  if not nalus:
    return bytes(payload)
  return b"".join(len(nalu).to_bytes(4, "big") + nalu for nalu in nalus)


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
  formats = getattr(av, "formats_available", set())
  codecs = getattr(av, "codecs_available", set())
  return {
    "available": True,
    "version": str(getattr(av, "__version__", "")),
    "flv": FLV_FORMAT in formats,
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
    self._fps = max(1, int(fps))
    self._video_time_base = Fraction(1, self._fps)
    self._audio_time_base = Fraction(1, AUDIO_RATE)
    self._container = av.open(
      output,
      mode="w",
      format=FLV_FORMAT,
      options={"flvflags": "no_duration_filesize", "flush_packets": "1"},
    )
    self._video_stream = self._container.add_stream(H264_CODEC, rate=self._fps)
    self._video_stream.width = max(1, int(width))
    self._video_stream.height = max(1, int(height))
    self._video_stream.time_base = self._video_time_base
    self._video_stream.codec_context.extradata = _avc_decoder_configuration(codec_header)

    self._audio_stream = self._container.add_stream(AAC_CODEC, rate=AUDIO_RATE)
    self._audio_stream.bit_rate = AUDIO_BITRATE
    self._audio_stream.layout = "stereo"
    self._container.start_encoding()
    self._packet_index = 0
    self._audio_pts = 0
    self._closed = False
    self._lock = threading.RLock()

  def mux(self, payload: bytes, *, keyframe: bool = False) -> None:
    with self._lock:
      if self._closed:
        raise RuntimeError("FLV muxer is closed")
      if not payload:
        return

      target_audio_pts = int(self._packet_index * AUDIO_RATE / self._fps)
      self._mux_silence_until(target_audio_pts)

      packet = self._av.Packet(_annexb_to_avcc(payload))
      packet.stream = self._video_stream
      packet.pts = self._packet_index
      packet.dts = self._packet_index
      packet.duration = 1
      packet.time_base = self._video_time_base
      if keyframe:
        try:
          packet.is_keyframe = True
        except Exception:
          pass
      self._container.mux(packet)
      self._packet_index += 1

  def close(self) -> None:
    with self._lock:
      if self._closed:
        return
      self._closed = True
      try:
        target_audio_pts = int(self._packet_index * AUDIO_RATE / self._fps)
        self._mux_silence_until(target_audio_pts)
        for packet in self._audio_stream.encode(None):
          self._container.mux(packet)
      finally:
        self._container.close()

  def _mux_silence_until(self, target_pts: int) -> None:
    while self._audio_pts <= target_pts:
      frame = self._av.AudioFrame(format="fltp", layout="stereo", samples=AUDIO_SAMPLES)
      frame.sample_rate = AUDIO_RATE
      frame.pts = self._audio_pts
      frame.time_base = self._audio_time_base
      for plane in frame.planes:
        plane.update(bytes(plane.buffer_size))
      for packet in self._audio_stream.encode(frame):
        self._container.mux(packet)
      self._audio_pts += AUDIO_SAMPLES
