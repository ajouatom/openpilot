from __future__ import annotations

import threading
from fractions import Fraction
from typing import Any, BinaryIO


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
    self._video_config = _avc_decoder_configuration(codec_header)
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

      if timestamp_ms is None:
        video_ms = int(self._packet_index * 1000 / self._fps)
      else:
        video_ms = max(0, int(timestamp_ms))
      # FLV/RTMP timestamps must be non-decreasing
      if video_ms < self._last_video_ms:
        video_ms = self._last_video_ms
      self._last_video_ms = video_ms

      # keep the silent audio track filled up to the current video time so a
      # dropped-frame gap stays A/V aligned
      self._mux_silence_until(int(video_ms * AUDIO_RATE / 1000))

      frame_header = b"\x17" if keyframe else b"\x27"
      self._write_tag(9, video_ms, frame_header + b"\x01\x00\x00\x00" + _annexb_to_avcc(payload))
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
