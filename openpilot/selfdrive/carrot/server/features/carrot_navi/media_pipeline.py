from __future__ import annotations

from collections import deque
import time
from typing import Any

from .fmp4 import CarrotNaviFmp4Muxer, Fmp4Initialization, Fmp4Segment
from .protocol import encode_media_frame


MEDIA_STALE_SECONDS = 3.0
H264_KEYFRAME_FLAG = 1


class CarrotNaviMediaPipeline:
  """Caches web media and remuxes the map H.264 stream into bootstrap-safe fMP4."""

  def __init__(self) -> None:
    self._latest_images: dict[str, bytes] = {}
    self._muxer = CarrotNaviFmp4Muxer()
    self._initialization: bytes | None = None
    self._gop: deque[bytes] = deque(maxlen=90)
    self._active = False
    self._failed = False
    self._retry_at = 0.0
    self._error = "" if self._muxer.available else "PyAV is unavailable"
    self._session_id = ""
    self._last_map_at = 0.0
    self._config_count = 0
    self._frame_count = 0
    self._keyframe_count = 0
    self._last_sequence = 0
    self._last_source_timestamp_ms = 0
    self._source_delta_ms: int | None = None
    self._last_wire_bytes = 0
    self._keyframe_at = 0.0

  @property
  def initialization_present(self) -> bool:
    return self._initialization is not None

  @property
  def last_map_at(self) -> float:
    return self._last_map_at

  def status(self, now: float | None = None) -> dict[str, Any]:
    current = time.monotonic() if now is None else now
    map_age_ms = int(max(0.0, current - self._last_map_at) * 1000) if self._last_map_at else None
    keyframe_age_ms = int(max(0.0, current - self._keyframe_at) * 1000) if self._keyframe_at else None
    return {
      "mapFresh": map_age_ms is not None and map_age_ms <= int(MEDIA_STALE_SECONDS * 1000),
      "mapAgeMs": map_age_ms,
      "mapStream": {
        "configPresent": self._initialization is not None,
        "gopFrames": len(self._gop),
        "configMessages": self._config_count,
        "frames": self._frame_count,
        "keyframes": self._keyframe_count,
        "lastSequence": self._last_sequence,
        "lastSourceTimestampMs": self._last_source_timestamp_ms,
        "sourceDeltaMs": self._source_delta_ms,
        "lastWireBytes": self._last_wire_bytes,
        "keyframeAgeMs": keyframe_age_ms,
        "webPipeline": {
          "mode": "server-fmp4" if self._active else "server-fmp4-waiting",
          "available": self._muxer.available,
          "active": self._active,
          "initializationPresent": self._initialization is not None,
          "gopSegments": len(self._gop),
          "error": self._error,
        },
      },
    }

  def remember(self, metadata: dict[str, Any], wire: bytes) -> None:
    session_id = str(metadata.get("sessionId", ""))
    if session_id != self._session_id:
      self._session_id = session_id
      self._latest_images.clear()
      self.reset_fmp4(clear_config=True)
      self._last_map_at = 0.0
      self._config_count = 0
      self._frame_count = 0
      self._keyframe_count = 0
      self._last_sequence = 0
      self._last_source_timestamp_ms = 0
      self._source_delta_ms = None
      self._last_wire_bytes = 0
      self._keyframe_at = 0.0
    key = f"{metadata['kind']}:{metadata['name']}"
    message_type = int(metadata["messageType"])
    present = bool(metadata["present"])
    if not present or message_type == 4:
      self._latest_images.pop(key, None)
      if key == "render:map_main":
        self._last_map_at = 0.0
      return
    if message_type == 1:
      self._latest_images[key] = wire
      return
    if message_type == 2:
      if key == "render:map_main":
        self._config_count += 1
      return
    if message_type != 3:
      return
    keyframe = bool(int(metadata["flags"]) & H264_KEYFRAME_FLAG)
    if key == "render:map_main":
      source_timestamp_ms = int(metadata.get("sourceTimestampMillis", 0))
      if self._last_source_timestamp_ms and source_timestamp_ms:
        self._source_delta_ms = source_timestamp_ms - self._last_source_timestamp_ms
      self._last_source_timestamp_ms = source_timestamp_ms
      self._last_sequence = int(metadata.get("sequence", 0))
      self._last_wire_bytes = len(wire)
      self._frame_count += 1
      if keyframe:
        self._keyframe_count += 1
        self._keyframe_at = time.monotonic()
      self._last_map_at = time.monotonic()

  def reset_fmp4(self, clear_config: bool) -> None:
    if clear_config:
      self._muxer.clear()
    else:
      self._muxer.close()
    self._initialization = None
    self._gop.clear()
    self._active = False
    self._failed = False
    self._retry_at = 0.0
    if clear_config:
      self._error = "" if self._muxer.available else "PyAV is unavailable"

  def clear(self) -> None:
    self._latest_images.clear()
    self.reset_fmp4(clear_config=True)
    self._session_id = ""
    self._last_map_at = 0.0
    self._config_count = 0
    self._frame_count = 0
    self._keyframe_count = 0
    self._last_sequence = 0
    self._last_source_timestamp_ms = 0
    self._source_delta_ms = None
    self._last_wire_bytes = 0
    self._keyframe_at = 0.0

  @staticmethod
  def _initialization_wire(
    metadata: dict[str, Any],
    initialization: Fmp4Initialization,
  ) -> tuple[dict[str, Any], bytes]:
    output_metadata = {
      **metadata,
      "type": "carrotNaviFmp4",
      "kind": "fmp4",
      "messageType": 2,
      "flags": 0,
      "width": initialization.width,
      "height": initialization.height,
      "mime": initialization.mime,
      "frameCount": 0,
      "keyframeCount": 0,
      "durationMs": 0,
    }
    return output_metadata, encode_media_frame(output_metadata, initialization.payload)

  @staticmethod
  def _segment_wire(metadata: dict[str, Any], segment: Fmp4Segment) -> tuple[dict[str, Any], bytes]:
    output_metadata = {
      **metadata,
      "type": "carrotNaviFmp4",
      "kind": "fmp4",
      "sequence": segment.sequence,
      "sourceTimestampMillis": segment.source_timestamp_ms,
      "messageType": 3,
      "flags": H264_KEYFRAME_FLAG if segment.keyframe else 0,
      "frameCount": 1,
      "keyframeCount": 1 if segment.keyframe else 0,
      "durationMs": segment.duration_ms,
    }
    return output_metadata, encode_media_frame(output_metadata, segment.payload)

  @staticmethod
  def _clear_wire(metadata: dict[str, Any], reason: str) -> tuple[dict[str, Any], bytes]:
    output_metadata = {
      **metadata,
      "type": "carrotNaviFmp4",
      "kind": "fmp4",
      "present": False,
      "messageType": 4,
      "flags": 0,
      "reason": str(reason or "cleared")[:64],
      "frameCount": 0,
      "keyframeCount": 0,
      "durationMs": 0,
    }
    return output_metadata, encode_media_frame(output_metadata, b"")

  def outputs(
    self,
    metadata: dict[str, Any],
    raw_wire: bytes,
    raw_payload: bytes,
    *,
    map_requested: bool,
  ) -> tuple[tuple[dict[str, Any], bytes], ...]:
    is_map = metadata.get("kind") == "render" and metadata.get("name") == "map_main"
    if not is_map:
      return ((metadata, raw_wire),)
    if not map_requested:
      return ()
    if not self._muxer.available:
      self._error = "PyAV is unavailable"
      return ()

    message_type = int(metadata.get("messageType", 0))
    present = bool(metadata.get("present", False))
    if not present or message_type == 4:
      reason = str(metadata.get("reason", "cleared"))
      self.reset_fmp4(clear_config=True)
      return (self._clear_wire(metadata, reason),)
    if message_type == 2:
      changed = self._muxer.configure(
        raw_payload,
        int(metadata.get("width", 0)),
        int(metadata.get("height", 0)),
        str(metadata.get("sessionId", "")),
      )
      if changed:
        self._initialization = None
        self._gop.clear()
        self._active = False
        self._failed = False
        self._retry_at = 0.0
        self._error = ""
      return ()
    if message_type != 3 or not raw_payload:
      return ()

    keyframe = bool(int(metadata.get("flags", 0)) & H264_KEYFRAME_FLAG)
    if self._failed:
      if not keyframe or time.monotonic() < self._retry_at:
        return ()
      self._failed = False

    try:
      output = self._muxer.push(
        raw_payload,
        sequence=int(metadata.get("sequence", 0)),
        source_timestamp_ms=int(metadata.get("sourceTimestampMillis", 0)),
        keyframe=keyframe,
      )
    except Exception as exc:
      self._failed = True
      self._retry_at = time.monotonic() + 5.0
      self._error = f"{type(exc).__name__}: {exc}"[:256]
      self._initialization = None
      self._gop.clear()
      self._active = False
      return (self._clear_wire(metadata, "server_remux_error"),)

    wires: list[tuple[dict[str, Any], bytes]] = []
    if output.initialization is not None:
      init_metadata, init_wire = self._initialization_wire(metadata, output.initialization)
      self._initialization = init_wire
      self._gop.clear()
      self._active = True
      self._error = ""
      wires.append((init_metadata, init_wire))
    for segment in output.segments:
      segment_metadata, segment_wire = self._segment_wire(metadata, segment)
      if segment.keyframe:
        self._gop.clear()
      self._gop.append(segment_wire)
      wires.append((segment_metadata, segment_wire))
    return tuple(wires)

  def bootstrap(self, include_map: bool = True) -> list[bytes]:
    if not self._last_map_at or time.monotonic() - self._last_map_at > MEDIA_STALE_SECONDS:
      return []
    bootstrap: list[bytes] = []
    if include_map and self._active and self._initialization is not None:
      bootstrap.append(self._initialization)
      bootstrap.extend(self._gop)
    bootstrap.extend(self._latest_images[key] for key in sorted(self._latest_images))
    return bootstrap
