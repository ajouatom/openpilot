from __future__ import annotations

import asyncio
import json
import os
import re
import socket
import ssl
import time
from collections import deque
from pathlib import Path
from typing import Any

from ..config import CARROT_YOUTUBE_LIVE_SECRET_PATH, CARROT_YOUTUBE_LIVE_STATE_PATH
from .youtube_live_captions import Cea608TimestampInjector
from .youtube_live_muxer import H264FlvMuxer, pyav_capabilities
from .youtube_live_transport import LibrtmpClient, RtmpSink, librtmp_capabilities


YOUTUBE_LIVE_PARAM = "CarrotYouTubeLive"
YOUTUBE_QUALITY_PARAM = "CarrotYouTubeQuality"
YOUTUBE_TIMESTAMP_PARAM = "CarrotYouTubeTimestamp"
# The high-quality source has a dedicated encoder so Carrot Vision's shared
# livestream resolution and bitrate remain unchanged.
SOURCE_BY_QUALITY = {
  0: "qRoadEncodeData",
  1: "livestreamRoadEncodeData",
  2: "youtubeRoadEncodeData",
  3: "livestreamWideRoadEncodeData",
}
QUALITY_LABELS = {0: "low", 1: "medium", 2: "high", 3: "wide"}
YOUTUBE_RTMPS_BASE = "rtmps://a.rtmps.youtube.com:443/live2"
YOUTUBE_RTMPS_HOST = "a.rtmps.youtube.com"
YOUTUBE_RTMPS_PORT = 443
NO_FRAME_STOP_SECONDS = 8.0
START_BACKOFF_SECONDS = 5.0
STATUS_WRITE_MIN_INTERVAL = 1.0
BACKOFF_BASE_SECONDS = 3.0
BACKOFF_MAX_SECONDS = 60.0
STREAM_STABLE_SECONDS = 10.0
MIN_RECONNECT_INTERVAL_SECONDS = 3.0
EVENT_LOG_MAX = 50
PROC_CACHE_SECONDS = 5.0

_PROCESS_MATCHES = {
  "carrot_cluster": "selfdrive.carrot.cluster_autorun",
  "webrtcd": "system.webrtc.webrtcd",
  "stream_encoderd": "encoderd\x00--stream",
  "youtube_encoderd": "encoderd\x00--youtube",
}


def _now() -> float:
  return time.time()


def _mono() -> float:
  # Monotonic clock for durations/intervals — immune to wall-clock jumps
  # (comma syncs system time from GPS/NTP after boot, which can step the clock).
  return time.monotonic()


def _read_json(path: Path) -> dict[str, Any]:
  try:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return raw if isinstance(raw, dict) else {}
  except Exception:
    return {}


def _write_json_atomic(path: Path, payload: dict[str, Any], *, mode: int | None = None) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temp_path = path.with_suffix(path.suffix + ".tmp")
  temp_path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
  if mode is not None:
    try:
      os.chmod(temp_path, mode)
    except OSError:
      pass
  temp_path.replace(path)
  if mode is not None:
    try:
      os.chmod(path, mode)
    except OSError:
      pass


def _mask_stream_key(stream_key: str) -> str:
  key = stream_key.strip()
  if not key:
    return ""
  if len(key) <= 8:
    return "*" * len(key)
  return f"{key[:4]}...{key[-4:]}"


def _extract_stream_key(value: str) -> str:
  raw = str(value or "").strip()
  if not raw:
    return ""
  if raw.startswith("rtmp://") or raw.startswith("rtmps://"):
    return raw.rstrip("/").rsplit("/", 1)[-1].strip()
  return raw


def _validate_stream_key_format(stream_key: str) -> tuple[bool, str]:
  key = stream_key.strip()
  if not key:
    return False, "stream key is required"
  if len(key) < 8:
    return False, "stream key is too short"
  if len(key) > 256:
    return False, "stream key is too long"
  if re.search(r"\s", key):
    return False, "stream key must not contain spaces"
  if not re.fullmatch(r"[A-Za-z0-9._/-]+", key):
    return False, "stream key contains unsupported characters"
  return True, "format looks valid"


def _check_rtmps_reachable(timeout: float = 2.5) -> tuple[bool, str]:
  try:
    context = ssl.create_default_context()
    with socket.create_connection((YOUTUBE_RTMPS_HOST, YOUTUBE_RTMPS_PORT), timeout=timeout) as sock:
      with context.wrap_socket(sock, server_hostname=YOUTUBE_RTMPS_HOST):
        return True, "YouTube RTMPS ingest is reachable"
  except Exception as exc:
    return False, f"YouTube RTMPS ingest unreachable: {exc}"


class YouTubeLiveService:
  def __init__(
    self,
    *,
    state_path: str = CARROT_YOUTUBE_LIVE_STATE_PATH,
    secret_path: str = CARROT_YOUTUBE_LIVE_SECRET_PATH,
  ) -> None:
    self.state_path = Path(state_path)
    self.secret_path = Path(secret_path)
    self._task: asyncio.Task | None = None
    self._stop_event = asyncio.Event()
    self._transport: LibrtmpClient | None = None
    self._transport_connected = False
    self._muxer: H264FlvMuxer | None = None
    self._caption_injector = Cea608TimestampInjector()
    self._messaging: Any | None = None
    self._socket: Any | None = None
    self._socket_source = ""
    self._active_source = ""
    self._params: Any | None = None
    self._last_status_write = 0.0
    self._started_at = 0.0
    self._started_mono = 0.0
    self._last_frame_at = 0.0
    self._last_frame_mono = 0.0
    self._last_frame_id: int | None = None
    self._frame_width = 526
    self._frame_height = 330
    self._frame_fps = 20
    self._bytes_sent = 0
    self._session_started_bytes = 0
    self._restart_count = 0
    self._consecutive_failures = 0
    self._next_retry_mono = 0.0
    self._last_error = ""
    self._state = "disabled"
    self._events: deque[dict[str, Any]] = deque(maxlen=EVENT_LOG_MAX)
    self._last_start_mono = 0.0
    self._proc_cache: dict[str, Any] | None = None
    self._proc_cache_mono = 0.0
    self._muxer_capabilities = pyav_capabilities()
    self._transport_capabilities = librtmp_capabilities()
    persisted = _read_json(self.state_path)
    try:
      self._bytes_sent = int(persisted.get("bytes_sent") or 0)
      self._restart_count = int(persisted.get("restart_count") or 0)
    except Exception:
      pass

  async def start(self) -> None:
    if self._task is not None and not self._task.done():
      return
    self._stop_event.clear()
    self._task = asyncio.create_task(self._run(), name="carrot-youtube-live")

  async def stop(self) -> None:
    self._stop_event.set()
    task = self._task
    if task is not None:
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
    await self._stop_stream()
    self._task = None
    self._write_status(force=True)

  def status(self) -> dict[str, Any]:
    stream_key = self.get_stream_key()
    running = self._transport_connected
    now = _now()
    mono = _mono()
    uptime = int(mono - self._started_mono) if self._started_mono and running else 0
    elapsed = max(1.0, mono - self._started_mono) if self._started_mono and running else 1.0
    session_bytes = max(0, self._bytes_sent - self._session_started_bytes) if running else 0
    last_frame_age_ms = int((mono - self._last_frame_mono) * 1000) if self._last_frame_mono else None
    retry_in_sec = max(0, int(self._next_retry_mono - mono)) if self._next_retry_mono else 0
    resource_status = self._resource_status()
    warnings = self._warnings(resource_status)
    return {
      "state": self._state,
      "enabled": self._param_enabled(),
      "configured": bool(stream_key),
      "masked_key": _mask_stream_key(stream_key),
      "source": self._current_source(),
      "muxer": "pyav-flv-aac",
      "muxer_available": bool(
        self._muxer_capabilities.get("available")
        and self._muxer_capabilities.get("flv")
        and self._muxer_capabilities.get("h264")
        and self._muxer_capabilities.get("aac")
      ),
      "transport": "librtmp-rtmps",
      "transport_available": bool(self._transport_capabilities.get("available")),
      "quality": self._quality_label(),
      "requested_quality": self._param_int(YOUTUBE_QUALITY_PARAM, 0),
      "timestamp_caption_enabled": self._param_bool(YOUTUBE_TIMESTAMP_PARAM),
      "timestamp_caption_mode": "cea608-sei",
      "timestamp_caption_packets": self._caption_injector.packets_injected,
      "phase": 1,
      "running": running,
      "pid": None,
      "started_at": self._started_at if running else 0,
      "uptime_sec": uptime,
      "bytes_sent": self._bytes_sent,
      "total_mb": round(max(0, self._bytes_sent) / (1024 * 1024), 2),
      "session_bytes": session_bytes,
      "session_mb": round(session_bytes / (1024 * 1024), 2),
      "estimated_kbps": int((session_bytes * 8 / 1000) / elapsed) if running else 0,
      "restart_count": self._restart_count,
      "consecutive_failures": self._consecutive_failures,
      "next_retry_at": (now + retry_in_sec) if retry_in_sec else 0.0,
      "retry_in_sec": retry_in_sec,
      "last_error": self._last_error,
      "last_frame_at": self._last_frame_at,
      "last_frame_age_ms": last_frame_age_ms,
      "last_frame_id": self._last_frame_id,
      "frame_width": self._frame_width,
      "frame_height": self._frame_height,
      "frame_fps": self._frame_fps,
      "log_tail": [f"{event['level']}: {event['message']}" for event in list(self._events)[-12:]],
      "resource_status": resource_status,
      "warnings": warnings,
    }

  def test_config(self) -> dict[str, Any]:
    stream_key = self.get_stream_key()
    rtmps_ok, rtmps_message = _check_rtmps_reachable()
    muxer_ok = bool(
      self._muxer_capabilities.get("available")
      and self._muxer_capabilities.get("flv")
      and self._muxer_capabilities.get("h264")
      and self._muxer_capabilities.get("aac")
    )
    transport_ok = bool(self._transport_capabilities.get("available"))
    ok = bool(stream_key and muxer_ok and transport_ok and rtmps_ok)
    status = self.status()
    return {
      "ok": ok,
      "configured": bool(stream_key),
      "transport_available": transport_ok,
      "transport": dict(self._transport_capabilities),
      "rtmps_reachable": rtmps_ok,
      "rtmps_message": rtmps_message,
      "muxer_available": muxer_ok,
      "muxer": dict(self._muxer_capabilities),
      "source": self._current_source(),
      "quality": self._quality_label(),
      "resource_status": status["resource_status"],
      "warnings": status["warnings"],
      "log_tail": [f"{event['level']}: {event['message']}" for event in list(self._events)[-12:]],
      "message": "ready" if ok else "stream key, RTMPS network, librtmp, or PyAV FLV/AAC is unavailable",
    }

  def validate_stream_key(self, value: str | None = None) -> dict[str, Any]:
    stream_key = _extract_stream_key(value) if value is not None else self.get_stream_key()
    format_ok, format_message = _validate_stream_key_format(stream_key)
    rtmps_ok, rtmps_message = _check_rtmps_reachable()
    transport_ok = bool(self._transport_capabilities.get("available"))
    muxer_ok = bool(
      self._muxer_capabilities.get("available")
      and self._muxer_capabilities.get("flv")
      and self._muxer_capabilities.get("h264")
      and self._muxer_capabilities.get("aac")
    )
    ok = bool(format_ok and rtmps_ok and transport_ok and muxer_ok)
    return {
      "ok": ok,
      "configured": bool(self.get_stream_key()),
      "format_ok": format_ok,
      "format_message": format_message,
      "rtmps_reachable": rtmps_ok,
      "rtmps_message": rtmps_message,
      "transport_available": transport_ok,
      "muxer_available": muxer_ok,
      "masked_key": _mask_stream_key(stream_key),
      "note": "YouTube only confirms whether the key is accepted when an encoder starts streaming.",
    }

  def diagnostics(self) -> dict[str, Any]:
    status = self.status()
    stream_key = self.get_stream_key()
    return {
      "generated_at": _now(),
      "status": status,
      "config": {
        "stream_key_configured": bool(stream_key),
        "stream_key_masked": _mask_stream_key(stream_key),
        "source": self._current_source(),
        "quality": self._quality_label(),
        "rtmps_ingest": f"{YOUTUBE_RTMPS_BASE}/{_mask_stream_key(stream_key)}" if stream_key else "",
      },
      "params": {
        YOUTUBE_LIVE_PARAM: self._param_enabled(),
        YOUTUBE_QUALITY_PARAM: self._param_int(YOUTUBE_QUALITY_PARAM, 0),
        YOUTUBE_TIMESTAMP_PARAM: self._param_bool(YOUTUBE_TIMESTAMP_PARAM),
        "ClusterHud": self._param_int("ClusterHud", 0),
        "DisableDM": self._param_int("DisableDM", 0),
        "IsOnroad": self._param_bool("IsOnroad", False),
      },
      "transport": {
        "name": "librtmp-rtmps",
        **self._transport_capabilities,
        "connected": self._transport_connected,
        "log_tail": [f"{event['level']}: {event['message']}" for event in list(self._events)[-12:]],
      },
      "muxer": {
        "name": "pyav-flv-aac",
        **self._muxer_capabilities,
      },
      "processes": self._process_status(),
      "state_path": str(self.state_path),
    }

  def get_stream_key(self) -> str:
    payload = _read_json(self.secret_path)
    return str(payload.get("stream_key") or "").strip()

  def set_stream_key(self, value: str) -> dict[str, Any]:
    stream_key = _extract_stream_key(value)
    if not stream_key:
      raise ValueError("stream key is required")
    _write_json_atomic(self.secret_path, {"stream_key": stream_key, "updated_at": _now()}, mode=0o600)
    self._last_error = ""
    self._write_status(force=True)
    return self.status()

  def clear_stream_key(self) -> dict[str, Any]:
    try:
      self.secret_path.unlink()
    except FileNotFoundError:
      pass
    self._write_status(force=True)
    return self.status()

  async def _run(self) -> None:
    while not self._stop_event.is_set():
      try:
        await self._tick()
      except asyncio.CancelledError:
        raise
      except Exception as exc:
        self._last_error = str(exc)
        self._set_state("error")
        await self._stop_stream()
        await asyncio.sleep(START_BACKOFF_SECONDS)
      await asyncio.sleep(0.02 if self._transport else 0.25)

  async def _tick(self) -> None:
    if not self._param_enabled():
      if self._transport is not None:
        await self._stop_stream()
      self._last_error = ""
      self._consecutive_failures = 0
      self._next_retry_mono = 0.0
      self._set_state("disabled")
      return

    stream_key = self.get_stream_key()
    if not stream_key:
      await self._stop_stream()
      self._set_state("needs_setup")
      return

    if not self._transport_capabilities.get("available"):
      await self._stop_stream()
      self._last_error = self._transport_capabilities.get("error") or "librtmp is unavailable"
      self._set_state("error")
      return

    if not (
      self._muxer_capabilities.get("available")
      and self._muxer_capabilities.get("flv")
      and self._muxer_capabilities.get("h264")
      and self._muxer_capabilities.get("aac")
    ):
      await self._stop_stream()
      self._last_error = "PyAV FLV/AAC muxer is unavailable"
      self._set_state("error")
      return

    if self._transport is not None and self._active_source and self._active_source != self._current_source():
      self._log(f"source changed to {self._current_source()}; restarting")
      await self._stop_stream()

    if self._next_retry_mono and _mono() < self._next_retry_mono:
      self._set_state("backoff")
      return

    if self._transport is not None and not await asyncio.to_thread(self._transport.is_connected):
      self._transport_connected = False
      self._last_error = "YouTube RTMPS connection closed"
      await self._stop_stream()
      self._schedule_backoff("RTMPS connection closed")
      self._set_state("backoff")
      return

    header, data, frame_id, keyframe, width, height = self._recv_frame()
    if not data:
      if self._transport is not None and self._last_frame_mono and _mono() - self._last_frame_mono > NO_FRAME_STOP_SECONDS:
        self._last_error = f"no {self._current_source()} frames"
        await self._stop_stream()
        self._schedule_backoff("frame timeout")
      self._set_state("waiting_frame" if self._transport is None else "live")
      return

    self._last_frame_at = _now()
    self._last_frame_mono = _mono()
    self._last_frame_id = frame_id
    self._frame_width = width
    self._frame_height = height

    if self._transport is None:
      if not keyframe or not header:
        self._set_state("waiting_keyframe")
        return
      if self._last_start_mono and (_mono() - self._last_start_mono) < MIN_RECONNECT_INTERVAL_SECONDS:
        # Avoid hammering YouTube with rapid reconnects (it rejects them and it
        # spins the state machine); hold briefly between start attempts.
        self._set_state("backoff")
        return
      try:
        await self._start_stream(stream_key, codec_header=header, width=width, height=height)
      except Exception as exc:
        self._last_error = f"YouTube RTMPS start failed: {exc}"
        self._schedule_backoff("RTMPS start failed")
        self._set_state("backoff")
        return

    data = self._caption_injector.inject(
      data,
      enabled=self._param_bool(YOUTUBE_TIMESTAMP_PARAM),
    )
    if await self._write_frame(data, keyframe=keyframe):
      self._set_state("live")

  def _set_state(self, state: str) -> None:
    self._state = state
    self._write_status()

  def _write_status(self, *, force: bool = False) -> None:
    mono = _mono()
    if not force and mono - self._last_status_write < STATUS_WRITE_MIN_INTERVAL:
      return
    self._last_status_write = mono
    try:
      _write_json_atomic(self.state_path, {"updated_at": _now(), **self.status()})
    except Exception:
      pass

  def _log(self, message: str, level: str = "info") -> None:
    # Rolling event history that survives restarts (unlike _last_error, which is
    # cleared on each start attempt). Surfaced via status()/diagnostics log_tail.
    text = str(message or "").strip()
    if not text:
      return
    self._events.append({"t": round(_now(), 3), "level": level, "message": text})

  def _param_enabled(self) -> bool:
    try:
      params = self._get_params()
      return bool(params and params.get_bool(YOUTUBE_LIVE_PARAM))
    except Exception:
      return False

  def _param_bool(self, name: str, default: bool = False) -> bool:
    try:
      params = self._get_params()
      if params is None:
        return default
      if hasattr(params, "get_bool"):
        return bool(params.get_bool(name))
      raw = params.get(name)
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
      return str(raw).strip() in ("1", "true", "True")
    except Exception:
      return default

  def _param_int(self, name: str, default: int = 0) -> int:
    try:
      params = self._get_params()
      if params is None:
        return default
      if hasattr(params, "get_int"):
        return int(params.get_int(name))
      raw = params.get(name)
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
      return int(str(raw).strip())
    except Exception:
      return default

  def _get_params(self) -> Any | None:
    if self._params is not None:
      return self._params
    try:
      from openpilot.common.params import Params
      self._params = Params()
    except Exception:
      self._params = None
    return self._params

  def _get_messaging(self) -> Any | None:
    if self._messaging is not None:
      return self._messaging
    try:
      import openpilot.cereal.messaging as messaging
      self._messaging = messaging
    except Exception as exc:
      self._last_error = f"messaging unavailable: {exc}"
      self._messaging = None
    return self._messaging

  def _current_source(self) -> str:
    quality = self._param_int(YOUTUBE_QUALITY_PARAM, 0)
    return SOURCE_BY_QUALITY.get(quality, SOURCE_BY_QUALITY[0])

  def _quality_label(self) -> str:
    return QUALITY_LABELS.get(self._param_int(YOUTUBE_QUALITY_PARAM, 0), "standard")

  def _get_socket(self) -> Any | None:
    source = self._current_source()
    if self._socket is not None and self._socket_source == source:
      return self._socket
    # first subscription or the selected source (quality/camera) changed
    self._socket = None
    messaging = self._get_messaging()
    if messaging is None:
      return None
    try:
      self._socket = messaging.sub_sock(source, conflate=True)
      self._socket_source = source
    except Exception as exc:
      self._last_error = f"{source} socket failed: {exc}"
      self._socket = None
    return self._socket

  def _recv_frame(self) -> tuple[bytes, bytes, int | None, bool, int, int]:
    messaging = self._get_messaging()
    sock = self._get_socket()
    if messaging is None or sock is None:
      return b"", b"", None, False, 526, 330
    try:
      msg = messaging.recv_one_or_none(sock)
      if msg is None:
        return b"", b"", None, False, 526, 330
      which = msg.which()
      frame = getattr(msg, which, None)
      if frame is None:
        return b"", b"", None, False, 526, 330
      header = bytes(getattr(frame, "header", b"") or b"")
      data = bytes(getattr(frame, "data", b"") or b"")
      frame_id = None
      flags = 0
      idx = getattr(frame, "idx", None)
      if idx is not None:
        try:
          frame_id = int(getattr(idx, "frameId", 0) or 0) or None
        except Exception:
          frame_id = None
        try:
          flags = int(getattr(idx, "flags", 0) or 0)
        except Exception:
          flags = 0
      width = int(getattr(frame, "width", 0) or 526)
      height = int(getattr(frame, "height", 0) or 330)
      keyframe = bool(header) or bool(flags & 0x8)
      return header, data, frame_id, keyframe, width, height
    except Exception as exc:
      self._last_error = f"{self._current_source()} recv failed: {exc}"
      return b"", b"", None, False, 526, 330

  async def _start_stream(self, stream_key: str, *, codec_header: bytes, width: int, height: int) -> None:
    await self._stop_stream()
    self._last_error = ""
    self._restart_count += 1
    self._last_start_mono = _mono()
    self._set_state("starting")
    self._log("connecting to YouTube RTMPS")
    rtmp_url = f"{YOUTUBE_RTMPS_BASE}/{stream_key}"
    transport = LibrtmpClient(rtmp_url)
    try:
      await asyncio.to_thread(transport.connect)
      muxer = await asyncio.to_thread(
        H264FlvMuxer,
        RtmpSink(transport),
        codec_header=codec_header,
        fps=self._frame_fps,
        width=width,
        height=height,
      )
    except Exception:
      await asyncio.to_thread(transport.close)
      raise
    self._transport = transport
    self._transport_connected = True
    self._muxer = muxer
    self._started_at = _now()
    self._started_mono = _mono()
    self._session_started_bytes = self._bytes_sent
    self._next_retry_mono = 0.0
    self._active_source = self._current_source()
    self._caption_injector.reset()
    self._log(f"stream started ({width}x{height})")

  async def _write_frame(self, payload: bytes, *, keyframe: bool) -> bool:
    transport = self._transport
    muxer = self._muxer
    if transport is None or muxer is None:
      return False
    if not await asyncio.to_thread(transport.is_connected):
      self._transport_connected = False
      self._last_error = "YouTube RTMPS connection closed"
      self._schedule_backoff("RTMPS connection closed")
      await self._stop_stream()
      return False
    try:
      await asyncio.to_thread(muxer.mux, payload, keyframe=keyframe)
      self._bytes_sent = self._session_started_bytes + transport.bytes_written
      if self._started_mono and _mono() - self._started_mono >= STREAM_STABLE_SECONDS:
        self._consecutive_failures = 0
      return True
    except Exception as exc:
      self._transport_connected = False
      self._last_error = f"YouTube RTMPS publish failed: {exc}"
      self._schedule_backoff("RTMPS publish failed")
      await self._stop_stream()
      return False

  async def _stop_stream(self) -> None:
    transport = self._transport
    self._transport = None
    self._transport_connected = False
    muxer = self._muxer
    self._muxer = None
    if transport is None and muxer is None:
      self._started_at = 0.0
      self._started_mono = 0.0
      return
    self._set_state("stopping")
    if muxer is not None:
      try:
        await asyncio.to_thread(muxer.close)
      except Exception:
        pass
    if transport is not None:
      self._bytes_sent = self._session_started_bytes + transport.bytes_written
      try:
        await asyncio.to_thread(transport.close)
      except Exception:
        pass
    self._started_at = 0.0
    self._started_mono = 0.0

  def _schedule_backoff(self, reason: str = "") -> None:
    self._consecutive_failures += 1
    delay = min(BACKOFF_MAX_SECONDS, BACKOFF_BASE_SECONDS * (2 ** max(0, self._consecutive_failures - 1)))
    self._next_retry_mono = _mono() + delay
    if reason and not self._last_error:
      self._last_error = reason
    self._log(f"retry in {delay:.0f}s: {reason or self._last_error or 'reconnect'}", "warn")

  def _process_status(self) -> dict[str, Any]:
    mono = _mono()
    if self._proc_cache is not None and (mono - self._proc_cache_mono) < PROC_CACHE_SECONDS:
      return self._proc_cache
    result = {}
    for name, match in _PROCESS_MATCHES.items():
      pids = _find_matching_pids(match)
      result[name] = {
        "running": bool(pids),
        "pids": pids[:8],
      }
    self._proc_cache = result
    self._proc_cache_mono = mono
    return result

  def _resource_status(self) -> dict[str, Any]:
    processes = self._process_status()
    cluster_param = self._param_int("ClusterHud", 0)
    disable_dm = self._param_int("DisableDM", 0)
    return {
      "cluster": {
        "enabled": cluster_param in (1, 2),
        "param": cluster_param,
        **processes.get("carrot_cluster", {"running": False, "pids": []}),
      },
      "carrot_vision": {
        "enabled": disable_dm == 2,
        "disable_dm": disable_dm,
        "webrtcd_running": bool(processes.get("webrtcd", {}).get("running")),
        "stream_encoderd_running": bool(processes.get("stream_encoderd", {}).get("running")),
        "webrtcd_pids": list(processes.get("webrtcd", {}).get("pids") or []),
        "stream_encoderd_pids": list(processes.get("stream_encoderd", {}).get("pids") or []),
      },
      "youtube_encoder": {
        **processes.get("youtube_encoderd", {"running": False, "pids": []}),
      },
    }

  def _warnings(self, resource_status: dict[str, Any]) -> list[str]:
    warnings = []
    if not self._param_enabled():
      return warnings
    cluster = resource_status.get("cluster") if isinstance(resource_status, dict) else {}
    vision = resource_status.get("carrot_vision") if isinstance(resource_status, dict) else {}
    if cluster and cluster.get("enabled"):
      warnings.append("Cluster HUD is enabled; monitor overall load and temperature during simultaneous use.")
    if vision and vision.get("enabled"):
      warnings.append("Carrot Vision is enabled; simultaneous streaming increases network and memory bandwidth use.")
    quality = self._param_int(YOUTUBE_QUALITY_PARAM, 0)
    if quality in (1, 3):
      if not (vision and vision.get("stream_encoderd_running")):
        warnings.append("The selected video mode is waiting for the shared livestream encoder to start onroad.")
    elif quality == 2:
      youtube_encoder = resource_status.get("youtube_encoder") if isinstance(resource_status, dict) else {}
      if not (youtube_encoder and youtube_encoder.get("running")):
        warnings.append("High quality is waiting for the dedicated YouTube encoder to start onroad.")
    return warnings


def _pid_cmdline(pid: int) -> str:
  try:
    return Path(f"/proc/{int(pid)}/cmdline").read_bytes().decode(errors="replace")
  except Exception:
    return ""


def _pid_alive(pid: int, match: str = "") -> bool:
  if pid <= 0:
    return False
  try:
    os.kill(pid, 0)
  except OSError:
    return False
  return not match or match in _pid_cmdline(pid)


def _find_matching_pids(match: str) -> list[int]:
  if not Path("/proc").exists():
    return []
  matches = []
  for proc_path in Path("/proc").glob("[0-9]*"):
    try:
      pid = int(proc_path.name)
    except ValueError:
      continue
    if _pid_alive(pid, match):
      matches.append(pid)
  return sorted(matches)
