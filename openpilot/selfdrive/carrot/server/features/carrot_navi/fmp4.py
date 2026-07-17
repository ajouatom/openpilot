from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from fractions import Fraction
import importlib.util
import io
import os
import struct
from typing import Any


TIMESCALE = 90_000
DEFAULT_FRAME_DURATION = 18_000
MIN_FRAME_DURATION = 900
MAX_FRAME_DURATION = 22_500


@dataclass(frozen=True)
class Fmp4Initialization:
  payload: bytes
  mime: str
  width: int
  height: int


@dataclass(frozen=True)
class Fmp4Segment:
  payload: bytes
  sequence: int
  source_timestamp_ms: int
  duration_ms: int
  keyframe: bool


@dataclass(frozen=True)
class Fmp4Output:
  initialization: Fmp4Initialization | None = None
  segments: tuple[Fmp4Segment, ...] = ()


@dataclass(frozen=True)
class _PendingSample:
  sequence: int
  source_timestamp_ms: int
  duration_ms: int
  keyframe: bool


class _IncrementalOutput:
  """Seekable append sink whose already-emitted prefix can be released."""

  def __init__(self) -> None:
    self._base = 0
    self._position = 0
    self._data = bytearray()

  def writable(self) -> bool:
    return True

  def seekable(self) -> bool:
    return True

  def tell(self) -> int:
    return self._position

  def write(self, value: bytes | bytearray | memoryview) -> int:
    payload = bytes(value)
    if self._position < self._base:
      raise OSError("fragmented MP4 muxer attempted to rewrite emitted data")
    offset = self._position - self._base
    end = offset + len(payload)
    if end > len(self._data):
      self._data.extend(b"\x00" * (end - len(self._data)))
    self._data[offset:end] = payload
    self._position += len(payload)
    return len(payload)

  def seek(self, offset: int, whence: int = os.SEEK_SET) -> int:
    if whence == os.SEEK_SET:
      position = offset
    elif whence == os.SEEK_CUR:
      position = self._position + offset
    elif whence == os.SEEK_END:
      position = self._base + len(self._data) + offset
    else:
      raise ValueError(f"unsupported seek mode: {whence}")
    if position < self._base:
      raise OSError("fragmented MP4 muxer attempted to seek into emitted data")
    self._position = position
    return position

  def flush(self) -> None:
    pass

  def drain(self) -> bytes:
    end = self._base + len(self._data)
    if self._position != end:
      raise OSError("fragmented MP4 muxer left a non-append write position")
    payload = bytes(self._data)
    self._base = end
    self._data.clear()
    return payload


def _annex_b_units(payload: bytes) -> tuple[bytes, ...]:
  positions: list[tuple[int, int]] = []
  index = 0
  while index + 3 <= len(payload):
    if payload[index : index + 4] == b"\x00\x00\x00\x01":
      positions.append((index, 4))
      index += 4
    elif payload[index : index + 3] == b"\x00\x00\x01":
      positions.append((index, 3))
      index += 3
    else:
      index += 1
  return tuple(
    payload[start + prefix : (positions[item + 1][0] if item + 1 < len(positions) else len(payload))]
    for item, (start, prefix) in enumerate(positions)
    if start + prefix < (positions[item + 1][0] if item + 1 < len(positions) else len(payload))
  )


def _codec_from_config(config_payload: bytes) -> str:
  sps = next((unit for unit in _annex_b_units(config_payload) if unit and unit[0] & 0x1F == 7), b"")
  if len(sps) < 4:
    return "avc1.42E01E"
  return f"avc1.{sps[1]:02X}{sps[2]:02X}{sps[3]:02X}"


def _boxes(payload: bytes) -> tuple[tuple[str, bytes], ...]:
  boxes: list[tuple[str, bytes]] = []
  offset = 0
  while offset < len(payload):
    if offset + 8 > len(payload):
      raise ValueError("incomplete fragmented MP4 box header")
    size = struct.unpack_from(">I", payload, offset)[0]
    header_size = 8
    if size == 1:
      if offset + 16 > len(payload):
        raise ValueError("incomplete extended fragmented MP4 box header")
      size = struct.unpack_from(">Q", payload, offset + 8)[0]
      header_size = 16
    elif size == 0:
      size = len(payload) - offset
    if size < header_size or offset + size > len(payload):
      raise ValueError("invalid fragmented MP4 box size")
    box = payload[offset : offset + size]
    boxes.append((box[4:8].decode("ascii", errors="replace"), box))
    offset += size
  return tuple(boxes)


class CarrotNaviFmp4Muxer:
  """Demand-driven, zero-transcode Annex-B H.264 to fragmented MP4 muxer."""

  def __init__(self) -> None:
    self._available = importlib.util.find_spec("av") is not None
    self._config = b""
    self._width = 0
    self._height = 0
    self._session_id = ""
    self._container: Any | None = None
    self._stream: Any | None = None
    self._sink: _IncrementalOutput | None = None
    self._pending: deque[_PendingSample] = deque()
    self._first_source_timestamp_ms: int | None = None
    self._last_source_timestamp_ms: int | None = None
    self._last_pts = -1
    self._initialization_sent = False
    self._fragment = bytearray()

  @property
  def available(self) -> bool:
    return self._available

  @property
  def active(self) -> bool:
    return self._container is not None and self._stream is not None

  def configure(self, config_payload: bytes, width: int, height: int, session_id: str) -> bool:
    config = bytes(config_payload)
    safe_width = max(1, min(65_535, int(width or 960)))
    safe_height = max(1, min(65_535, int(height or 540)))
    normalized_session = str(session_id or "")
    changed = config != self._config or safe_width != self._width or safe_height != self._height or normalized_session != self._session_id
    if not changed:
      return False
    self.close()
    self._config = config
    self._width = safe_width
    self._height = safe_height
    self._session_id = normalized_session
    return True

  def _start(self, keyframe_payload: bytes) -> None:
    if not self._available:
      raise RuntimeError("PyAV is unavailable")
    if not self._config:
      raise RuntimeError("H.264 configuration is unavailable")

    import av

    probe = av.open(io.BytesIO(self._config + keyframe_payload), mode="r", format="h264")
    try:
      template = probe.streams.video[0]
      sink = _IncrementalOutput()
      container = av.open(
        sink,
        mode="w",
        format="mp4",
        options={
          "movflags": "frag_every_frame+empty_moov+default_base_moof+omit_tfhd_offset",
          "flush_packets": "1",
        },
      )
      try:
        stream = container.add_stream_from_template(template)
        stream.time_base = Fraction(1, TIMESCALE)
      except Exception:
        container.close()
        raise
    finally:
      probe.close()

    self._sink = sink
    self._container = container
    self._stream = stream
    self._pending.clear()
    self._first_source_timestamp_ms = None
    self._last_source_timestamp_ms = None
    self._last_pts = -1
    self._initialization_sent = False
    self._fragment.clear()

  def _timing(self, source_timestamp_ms: int) -> tuple[int, int, int]:
    source_timestamp = max(0, int(source_timestamp_ms))
    if self._first_source_timestamp_ms is None:
      self._first_source_timestamp_ms = source_timestamp
      pts = 0
    elif source_timestamp > self._first_source_timestamp_ms:
      pts = int((source_timestamp - self._first_source_timestamp_ms) * TIMESCALE / 1000)
    else:
      pts = self._last_pts + DEFAULT_FRAME_DURATION
    if pts <= self._last_pts:
      pts = self._last_pts + 1

    duration = DEFAULT_FRAME_DURATION
    if self._last_source_timestamp_ms is not None and source_timestamp > self._last_source_timestamp_ms:
      duration = int((source_timestamp - self._last_source_timestamp_ms) * TIMESCALE / 1000)
      duration = max(MIN_FRAME_DURATION, min(MAX_FRAME_DURATION, duration))
    self._last_source_timestamp_ms = source_timestamp
    self._last_pts = pts
    return pts, duration, max(1, round(duration * 1000 / TIMESCALE))

  def _consume(self, payload: bytes) -> Fmp4Output:
    initialization_parts: list[bytes] = []
    segments: list[Fmp4Segment] = []
    for box_type, box in _boxes(payload):
      if box_type in ("ftyp", "moov"):
        initialization_parts.append(box)
        continue
      if box_type == "moof":
        if self._fragment:
          raise ValueError("fragmented MP4 media fragment is missing mdat")
        self._fragment.extend(box)
        continue
      if box_type == "mdat":
        if not self._fragment or not self._pending:
          raise ValueError("fragmented MP4 media data has no pending sample")
        self._fragment.extend(box)
        sample = self._pending.popleft()
        segments.append(
          Fmp4Segment(
            payload=bytes(self._fragment),
            sequence=sample.sequence,
            source_timestamp_ms=sample.source_timestamp_ms,
            duration_ms=sample.duration_ms,
            keyframe=sample.keyframe,
          )
        )
        self._fragment.clear()
        continue
      if box_type != "mfra" and self._fragment:
        self._fragment.extend(box)

    initialization = None
    if initialization_parts and not self._initialization_sent:
      initialization = Fmp4Initialization(
        payload=b"".join(initialization_parts),
        mime=f'video/mp4; codecs="{_codec_from_config(self._config)}"',
        width=self._width,
        height=self._height,
      )
      self._initialization_sent = True
    return Fmp4Output(initialization=initialization, segments=tuple(segments))

  def push(self, frame_payload: bytes, *, sequence: int, source_timestamp_ms: int, keyframe: bool) -> Fmp4Output:
    payload = bytes(frame_payload)
    if not payload:
      return Fmp4Output()
    if not self.active:
      if not keyframe:
        return Fmp4Output()
      self._start(payload)

    import av

    pts, duration, duration_ms = self._timing(source_timestamp_ms)
    packet = av.Packet(payload)
    packet.stream = self._stream
    packet.pts = pts
    packet.dts = pts
    packet.duration = duration
    packet.time_base = Fraction(1, TIMESCALE)
    packet.is_keyframe = bool(keyframe)
    self._pending.append(
      _PendingSample(
        sequence=max(0, int(sequence)),
        source_timestamp_ms=max(0, int(source_timestamp_ms)),
        duration_ms=duration_ms,
        keyframe=bool(keyframe),
      )
    )
    try:
      self._container.mux(packet)
      return self._consume(self._sink.drain())
    except Exception:
      self.close()
      raise

  def clear(self) -> None:
    self.close()
    self._config = b""
    self._width = 0
    self._height = 0
    self._session_id = ""

  def close(self) -> None:
    container = self._container
    self._container = None
    self._stream = None
    self._sink = None
    if container is not None:
      try:
        container.close()
      except Exception:
        pass
    self._pending.clear()
    self._first_source_timestamp_ms = None
    self._last_source_timestamp_ms = None
    self._last_pts = -1
    self._initialization_sent = False
    self._fragment.clear()
