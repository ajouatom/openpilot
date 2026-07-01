from __future__ import annotations

import io
from fractions import Fraction
from typing import Any


MPEGTS_FORMAT = "mpegts"
H264_CODEC = "h264"


class _StreamingBuffer(io.RawIOBase):
  def __init__(self) -> None:
    super().__init__()
    self._chunks = bytearray()
    self._position = 0

  def writable(self) -> bool:
    return True

  def seekable(self) -> bool:
    return False

  def write(self, data: bytes | bytearray | memoryview) -> int:
    chunk = bytes(data)
    self._chunks.extend(chunk)
    self._position += len(chunk)
    return len(chunk)

  def tell(self) -> int:
    return self._position

  def drain(self) -> bytes:
    if not self._chunks:
      return b""
    chunk = bytes(self._chunks)
    self._chunks.clear()
    return chunk


def pyav_capabilities() -> dict[str, Any]:
  try:
    import av
  except Exception as exc:
    return {
      "available": False,
      "version": "",
      "mpegts": False,
      "h264": False,
      "error": str(exc),
    }
  formats = getattr(av, "formats_available", set())
  codecs = getattr(av, "codecs_available", set())
  return {
    "available": True,
    "version": str(getattr(av, "__version__", "")),
    "mpegts": MPEGTS_FORMAT in formats,
    "h264": H264_CODEC in codecs,
    "error": "",
  }


class H264MpegTsMuxer:
  def __init__(self, *, fps: int = 20, width: int = 526, height: int = 330) -> None:
    import av

    self._av = av
    self._fps = max(1, int(fps))
    self._time_base = Fraction(1, self._fps)
    self._buffer = _StreamingBuffer()
    self._container = av.open(
      self._buffer,
      mode="w",
      format=MPEGTS_FORMAT,
      options={"flush_packets": "1", "mpegts_flags": "+resend_headers"},
    )
    self._stream = self._container.add_stream(H264_CODEC, rate=self._fps)
    self._stream.width = max(1, int(width))
    self._stream.height = max(1, int(height))
    self._stream.time_base = self._time_base
    self._packet_index = 0
    self._closed = False

  def mux(self, payload: bytes, *, keyframe: bool = False) -> bytes:
    if self._closed:
      raise RuntimeError("MPEG-TS muxer is closed")
    if not payload:
      return b""
    packet = self._av.Packet(payload)
    packet.stream = self._stream
    packet.pts = self._packet_index
    packet.dts = self._packet_index
    packet.duration = 1
    packet.time_base = self._time_base
    if keyframe:
      try:
        packet.is_keyframe = True
      except Exception:
        pass
    self._container.mux(packet)
    self._packet_index += 1
    return self._buffer.drain()

  def close(self) -> bytes:
    if self._closed:
      return self._buffer.drain()
    self._closed = True
    try:
      self._container.close()
    except Exception:
      pass
    return self._buffer.drain()
