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
from .youtube_h264 import validate_stream_start
from .youtube_live_captions import Cea608TimestampInjector
from .youtube_live_muxer import H264FlvMuxer, pyav_capabilities
from .youtube_live_transport import LibrtmpClient, RtmpSink, librtmp_capabilities
from .youtube_live_writer import RtmpFrameWriter
from .youtube_profiles import YOUTUBE_PROFILES, youtube_profile


YOUTUBE_LIVE_PARAM = "CarrotYouTubeLive"
YOUTUBE_QUALITY_PARAM = "CarrotYouTubeQuality"
YOUTUBE_TIMESTAMP_PARAM = "CarrotYouTubeTimestamp"
# Compatibility views for the existing API and tests. The authoritative
# profile definition lives in youtube_profiles.py.
SOURCE_BY_QUALITY = {quality: profile.source for quality, profile in YOUTUBE_PROFILES.items()}
QUALITY_LABELS = {quality: profile.label for quality, profile in YOUTUBE_PROFILES.items()}
QUALITY_TARGETS = {quality: profile.target for quality, profile in YOUTUBE_PROFILES.items()}
YOUTUBE_RTMPS_BASE = "rtmps://a.rtmps.youtube.com:443/live2"
YOUTUBE_RTMPS_HOST = "a.rtmps.youtube.com"
YOUTUBE_RTMPS_PORT = 443
NO_FRAME_STOP_SECONDS = 8.0
STATUS_WRITE_MIN_INTERVAL = 5.0
BACKOFF_BASE_SECONDS = 3.0
BACKOFF_MAX_SECONDS = 60.0
STREAM_STABLE_SECONDS = 10.0
MIN_RECONNECT_INTERVAL_SECONDS = 3.0
CONNECTED_CHECK_INTERVAL_SECONDS = 1.0
PARAM_CACHE_SECONDS = 0.5
SOURCE_WARMUP_SECONDS = 6.0
SOURCE_WARMUP_MIN_FRAMES = 45
SOURCE_WARMUP_MIN_KBPS_RATIO = 0.55
SOURCE_SAMPLE_WINDOW_SECONDS = 10.0
SOURCE_GAP_RESET_SECONDS = 2.0
UPLOAD_STARVED_GRACE_SECONDS = 20.0
UPLOAD_STARVED_SECONDS = 15.0
UPLOAD_SAMPLE_WINDOW_SECONDS = 10.0
EVENT_LOG_MAX = 50
PROC_CACHE_SECONDS = 5.0
NET_CHECK_UP_INTERVAL_SECONDS = 8.0
NET_CHECK_DOWN_INTERVAL_SECONDS = 2.0
ACTIVE_LOOP_PERIOD_SECONDS = 0.01
IDLE_LOOP_PERIOD_SECONDS = 0.25
RTMP_FRAME_QUEUE_MAX = 30
RTMP_FRAME_QUEUE_MAX_BYTES = 4 * 1024 * 1024

_PROCESS_MATCHES = {
  "carrot_cluster": "selfdrive.carrot.cluster_autorun",
  "webrtcd": "system.webrtc.carrot_webrtcd",
  "stream_encoderd": "encoderd\x00--carrot-vision-road",
  **{
    profile.process_name: f"encoderd\x00{profile.encoder_flag}\x00"
    for profile in YOUTUBE_PROFILES.values()
  },
}


def _now() -> float:
  return time.time()  # noqa: TID251 - persisted timestamps require wall-clock time


def _mono() -> float:
  # Monotonic clock for durations/intervals — immune to wall-clock jumps
  # (comma syncs system time from GPS/NTP after boot, which can step the clock).
  return time.monotonic()


def _loop_delay(started_mono: float, finished_mono: float, *, active: bool) -> float:
  period = ACTIVE_LOOP_PERIOD_SECONDS if active else IDLE_LOOP_PERIOD_SECONDS
  return max(0.0, period - max(0.0, finished_mono - started_mono))


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
  if raw.startswith(("rtmp://", "rtmps://")):
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


def _network_reachable(timeout: float = 1.5) -> bool:
  # Lightweight readiness probe: can we open a TCP connection to YouTube's RTMPS
  # ingest? Used to hold off (re)connecting until the network is actually up
  # (first boot / hotspot re-attach), instead of forming a degraded session.
  try:
    with socket.create_connection((YOUTUBE_RTMPS_HOST, YOUTUBE_RTMPS_PORT), timeout=timeout):
      return True
  except Exception:
    return False


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
    self._rtmp_sink: RtmpSink | None = None
    self._muxer: H264FlvMuxer | None = None
    self._writer: RtmpFrameWriter | None = None
    self._writer_backpressure_restarts = 0
    self._writer_discarded_frames = 0
    self._caption_injector = Cea608TimestampInjector()
    self._messaging: Any | None = None
    self._socket: Any | None = None
    self._socket_source = ""
    self._socket_quality = 0
    self._active_source = ""
    self._active_quality = 0
    self._warmup_source = ""
    self._warmup_quality = 0
    self._warmup_started_mono = 0.0
    self._warmup_frames = 0
    self._warmup_bytes = 0
    self._warmup_kbps = 0.0
    self._warmup_last_frame_id: int | None = None
    self._stream_source_samples: deque[tuple[float, int, bool]] = deque(maxlen=800)
    self._sent_samples: deque[tuple[float, int]] = deque(maxlen=800)
    self._starved_since_mono = 0.0
    self._stream_source_bytes = 0
    self._stream_source_frames = 0
    self._stream_source_keyframes = 0
    self._params: Any | None = None
    self._param_cache: dict[str, tuple[float, Any]] = {}
    self._stream_key_cache: tuple[int | None, str] = (None, "")
    self._conn_checked_mono = 0.0
    self._last_status_write = 0.0
    self._started_at = 0.0
    self._started_mono = 0.0
    self._last_frame_at = 0.0
    self._last_frame_mono = 0.0
    self._last_frame_id: int | None = None
    self._frame_width = 526
    self._frame_height = 330
    self._frame_fps = youtube_profile(0).fps
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
    self._net_ok = False
    self._net_checked_mono = 0.0
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
    rtmp_sink = self._rtmp_sink if running else None
    writer = self._writer if running else None
    mux_input_bytes = rtmp_sink.bytes_accepted if rtmp_sink is not None else 0
    mux_pending_bytes = rtmp_sink.pending_bytes if rtmp_sink is not None else 0
    mux_input_kbps = int((mux_input_bytes * 8 / 1000) / elapsed) if running else 0
    source_recent = self._stream_source_recent(mono) if running else {}
    stream_source_kbps = int((self._stream_source_bytes * 8 / 1000) / elapsed) if running else 0
    stream_source_fps = round(self._stream_source_frames / elapsed, 1) if running else 0.0
    last_frame_age_ms = int((mono - self._last_frame_mono) * 1000) if self._last_frame_mono else None
    retry_in_sec = max(0, int(self._next_retry_mono - mono)) if self._next_retry_mono else 0
    resource_status = self._resource_status()
    warnings = self._warnings(resource_status)
    target = self._quality_target()
    declared_frame_fps = int(target.get("fps") or youtube_profile(0).fps)
    observed_frame_fps = float(source_recent.get("fps") or 0.0) if running else 0.0
    warmup_age_sec = round(mono - self._warmup_started_mono, 1) if self._warmup_started_mono and not running else 0.0
    return {
      "state": self._state,
      "enabled": self._param_enabled(),
      "configured": bool(stream_key),
      "masked_key": _mask_stream_key(stream_key),
      "source": self._current_source(),
      "muxer": "flv-h264-aac",
      "muxer_available": bool(
        self._muxer_capabilities.get("available")
        and self._muxer_capabilities.get("flv")
        and self._muxer_capabilities.get("h264")
        and self._muxer_capabilities.get("aac")
      ),
      "transport": "librtmp-rtmps",
      "transport_available": bool(self._transport_capabilities.get("available")),
      "transport_connected": self._transport_connected,
      "quality": self._quality_label(),
      "requested_quality": self._param_int(YOUTUBE_QUALITY_PARAM, 0),
      "target_width": target.get("width"),
      "target_height": target.get("height"),
      "target_fps": target.get("fps"),
      "target_video_kbps": target.get("video_kbps"),
      "target_gop_seconds": target.get("gop_seconds"),
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
      "upload_recent_kbps": (self._upload_recent_kbps(mono) or 0) if running else 0,
      "mux_input_bytes": mux_input_bytes,
      "mux_input_kbps": mux_input_kbps,
      "mux_pending_bytes": mux_pending_bytes,
      "rtmp_drain_calls": rtmp_sink.drain_calls if rtmp_sink is not None else 0,
      "rtmp_partial_writes": rtmp_sink.partial_writes if rtmp_sink is not None else 0,
      "rtmp_write_ratio": round(session_bytes / mux_input_bytes, 3) if mux_input_bytes > 0 else None,
      "rtmp_writer_pending_frames": writer.pending_frames if writer is not None else 0,
      "rtmp_writer_capacity": writer.capacity if writer is not None else RTMP_FRAME_QUEUE_MAX,
      "rtmp_writer_high_watermark": writer.high_watermark if writer is not None else 0,
      "rtmp_writer_pending_bytes": writer.pending_bytes if writer is not None else 0,
      "rtmp_writer_capacity_bytes": writer.capacity_bytes if writer is not None else RTMP_FRAME_QUEUE_MAX_BYTES,
      "rtmp_writer_high_watermark_bytes": writer.high_watermark_bytes if writer is not None else 0,
      "rtmp_writer_frames_written": writer.frames_written if writer is not None else 0,
      "rtmp_writer_last_write_ms": writer.last_write_ms if writer is not None else 0,
      "rtmp_writer_max_write_ms": writer.max_write_ms if writer is not None else 0,
      "rtmp_writer_error": writer.error if writer is not None else "",
      "rtmp_writer_backpressure_restarts": self._writer_backpressure_restarts,
      "rtmp_writer_discarded_frames": self._writer_discarded_frames,
      "stream_source_kbps": stream_source_kbps,
      "stream_source_fps": stream_source_fps,
      "stream_source_frames": self._stream_source_frames if running else 0,
      "stream_source_keyframes": self._stream_source_keyframes if running else 0,
      "stream_source_recent_kbps": source_recent.get("kbps", 0),
      "stream_source_recent_fps": source_recent.get("fps", 0.0),
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
      # frame_fps is retained for the existing UI. It is the encoder contract,
      # while observed_frame_fps reports the source rate measured at runtime.
      "frame_fps": declared_frame_fps,
      "declared_frame_fps": declared_frame_fps,
      "observed_frame_fps": observed_frame_fps,
      "frame_matches_target": self._frame_matches_target(target),
      "source_warmup_sec": warmup_age_sec,
      "source_warmup_frames": self._warmup_frames if not running else 0,
      "source_warmup_kbps": int(self._warmup_kbps) if not running else 0,
      "source_warmup_required_kbps": self._source_warmup_required_kbps(target),
      "source_warmup_required_sec": SOURCE_WARMUP_SECONDS,
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
      "target": self._quality_target(),
      "resource_status": status["resource_status"],
      "warnings": status["warnings"],
      "log_tail": [f"{event['level']}: {event['message']}" for event in list(self._events)[-12:]],
      "message": "ready" if ok else "stream key, RTMPS network, librtmp, or FLV/AAC muxer is unavailable",
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
        "target": self._quality_target(),
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
        "name": "flv-h264-aac",
        **self._muxer_capabilities,
      },
      "processes": self._process_status(),
      "state_path": str(self.state_path),
    }

  def get_stream_key(self) -> str:
    # mtime-cached: this runs every service tick (up to 50Hz while live) and
    # re-reading/parsing the secret file each time is wasted disk work
    try:
      mtime_ns = os.stat(self.secret_path).st_mtime_ns
    except OSError:
      self._stream_key_cache = (None, "")
      return ""
    cached_mtime, cached_key = self._stream_key_cache
    if cached_mtime == mtime_ns:
      return cached_key
    payload = _read_json(self.secret_path)
    key = str(payload.get("stream_key") or "").strip()
    self._stream_key_cache = (mtime_ns, key)
    return key

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
      tick_started_mono = _mono()
      try:
        await self._tick()
      except asyncio.CancelledError:
        raise
      except Exception as exc:
        await self._enter_backoff(str(exc), reason="service loop failure")
      delay = _loop_delay(tick_started_mono, _mono(), active=self._transport is not None)
      await asyncio.sleep(delay)

  async def _tick(self) -> None:
    if not self._param_enabled():
      if self._transport is not None:
        await self._stop_stream()
      self._reset_source_warmup()
      self._last_error = ""
      self._consecutive_failures = 0
      self._next_retry_mono = 0.0
      self._set_state("disabled")
      return

    stream_key = self.get_stream_key()
    if not stream_key:
      await self._stop_stream()
      self._reset_source_warmup()
      self._set_state("needs_setup")
      return

    if not self._transport_capabilities.get("available"):
      await self._stop_stream()
      self._reset_source_warmup()
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
      self._reset_source_warmup()
      self._last_error = "PyAV FLV/AAC muxer is unavailable"
      self._set_state("error")
      return

    if self._transport is None and self._next_retry_mono and _mono() < self._next_retry_mono:
      self._set_state("backoff")
      return

    # The active RTMP session is the authoritative live-network signal. A
    # separate reachability probe may take 1.5s on a weak hotspot, so only run
    # it while disconnected.
    if self._transport is None:
      await self._refresh_network()

    current_source = self._current_source()
    current_quality = self._current_quality()
    if self._transport is not None and (
      (self._active_source and self._active_source != current_source)
      or self._active_quality != current_quality
    ):
      self._log(f"video mode changed to {self._quality_label()}; restarting")
      await self._stop_stream()
      self._reset_source_warmup()

    writer = self._writer
    if self._transport is not None and writer is not None and writer.error:
      await self._enter_backoff(
        f"YouTube RTMPS publish failed: {writer.error}",
        reason="RTMPS publish failed",
      )
      return

    if self._transport is not None and (_mono() - self._conn_checked_mono) >= CONNECTED_CHECK_INTERVAL_SECONDS:
      # Skip the check while RTMP_Write owns the handle instead of waiting for
      # the network stall timeout on the source-ingestion path.
      self._conn_checked_mono = _mono()
      connected = self._transport.try_is_connected()
      if connected is False:
        await self._enter_backoff(
          "YouTube RTMPS connection closed",
          reason="RTMPS connection closed",
        )
        # source is fine — keep warmup so the reconnect stays a few seconds
        return

    warmup_ready = False
    if self._transport is None:
      # Pre-connect we poll slowly while waiting for warmup/keyframe/network, and
      # the queue can hold tens of seconds of frames. Drain it every tick so the
      # stream can only start from the freshest keyframe — starting from a stale
      # one makes YouTube play the past and buffer right at go-live.
      header = data = b""
      frame_id, keyframe = None, False
      width, height = self._frame_width, self._frame_height
      while True:
        n_header, n_data, n_frame_id, n_keyframe, n_width, n_height = self._recv_frame()
        if not n_data:
          break
        header, data, frame_id, keyframe, width, height = n_header, n_data, n_frame_id, n_keyframe, n_width, n_height
        warmup_ready = self._source_warmup_ready(
          current_source,
          current_quality,
          frame_id,
          width,
          height,
          len(header) + len(data),
        )
    else:
      header, data, frame_id, keyframe, width, height = self._recv_frame()

    if not data:
      if self._transport is not None and self._last_frame_mono and _mono() - self._last_frame_mono > NO_FRAME_STOP_SECONDS:
        await self._enter_backoff(
          f"no {self._current_source()} frames",
          reason="frame timeout",
        )
        self._reset_source_warmup()
        return
      if self._transport is None and self._last_frame_mono and _mono() - self._last_frame_mono > SOURCE_GAP_RESET_SECONDS:
        # a real source gap (encoder stopped), not just an empty poll between frames
        self._reset_source_warmup()
      self._set_state("waiting_frame" if self._transport is None else "live")
      return

    self._last_frame_at = _now()
    self._last_frame_mono = _mono()
    self._last_frame_id = frame_id
    self._frame_width = width
    self._frame_height = height

    if self._transport is None:
      if not warmup_ready:
        self._set_state("waiting_stable_source")
        return
      if not keyframe or not header:
        self._set_state("waiting_keyframe")
        return
      try:
        validate_stream_start(header, data)
      except ValueError as exc:
        self._last_error = f"H.264 stream start rejected: {exc}"
        self._set_state("waiting_keyframe")
        return
      if not self._net_ok:
        # Hold the first connect until the network is actually reachable so we
        # never open a degraded session (first boot / hotspot re-attach).
        self._set_state("waiting_network")
        return
      if self._last_start_mono and (_mono() - self._last_start_mono) < MIN_RECONNECT_INTERVAL_SECONDS:
        # Avoid hammering YouTube with rapid reconnects (it rejects them and it
        # spins the state machine); hold briefly between start attempts.
        self._next_retry_mono = max(
          self._next_retry_mono,
          self._last_start_mono + MIN_RECONNECT_INTERVAL_SECONDS,
        )
        self._set_state("backoff")
        return
      try:
        await self._start_stream(stream_key, codec_header=header, width=width, height=height)
      except Exception as exc:
        error = f"YouTube RTMPS start failed: {exc}"
        # network/YouTube failure, not a source problem — keep warmup
        await self._enter_backoff(error, reason="RTMPS start failed")
        return

    data = self._caption_injector.inject(
      data,
      enabled=self._param_bool(YOUTUBE_TIMESTAMP_PARAM),
    )
    self._record_stream_source_sample(len(data), keyframe)
    if await self._write_frame(data, keyframe=keyframe):
      if await self._reconnect_if_starved():
        return
      self._set_state("live")

  def _set_state(self, state: str) -> None:
    self._state = state
    self._write_status()

  async def _enter_backoff(self, error: str, *, reason: str) -> None:
    self._transport_connected = False
    self._last_error = str(error or reason)
    self._log(self._last_error, "warn")
    await self._stop_stream()
    self._schedule_backoff(reason)
    self._set_state("backoff")

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
    return self._param_bool(YOUTUBE_LIVE_PARAM)

  def _param_cached(self, key: str, reader: Any) -> Any:
    # short TTL cache: params are file-backed reads and the live loop runs at
    # up to 50Hz; a 0.5s-stale toggle/quality value is imperceptible
    mono = _mono()
    hit = self._param_cache.get(key)
    if hit is not None and (mono - hit[0]) < PARAM_CACHE_SECONDS:
      return hit[1]
    value = reader()
    self._param_cache[key] = (mono, value)
    return value

  def _param_bool(self, name: str, default: bool = False) -> bool:
    def read() -> bool:
      params = self._get_params()
      if params is None:
        return default
      if hasattr(params, "get_bool"):
        return bool(params.get_bool(name))
      raw = params.get(name)
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
      return str(raw).strip() in ("1", "true", "True")
    try:
      return bool(self._param_cached(f"bool:{name}", read))
    except Exception:
      return default

  def _param_int(self, name: str, default: int = 0) -> int:
    def read() -> int:
      params = self._get_params()
      if params is None:
        return default
      if hasattr(params, "get_int"):
        return int(params.get_int(name))
      raw = params.get(name)
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
      return int(str(raw).strip())
    try:
      return int(self._param_cached(f"int:{name}", read))
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
    return youtube_profile(self._current_quality()).source

  def _current_quality(self) -> int:
    return youtube_profile(self._param_int(YOUTUBE_QUALITY_PARAM, 0)).quality

  def _quality_label(self) -> str:
    return youtube_profile(self._current_quality()).label

  def _quality_target(self) -> dict[str, int]:
    return dict(youtube_profile(self._current_quality()).target)

  def _frame_matches_target(self, target: dict[str, Any] | None = None) -> bool | None:
    target = target if target is not None else self._quality_target()
    target_width = int(target.get("width") or 0)
    target_height = int(target.get("height") or 0)
    if target_width <= 0 or target_height <= 0:
      return None
    return self._frame_width == target_width and self._frame_height == target_height

  def _source_warmup_required_kbps(self, target: dict[str, Any] | None = None) -> int:
    target = target if target is not None else self._quality_target()
    target_kbps = int(target.get("video_kbps") or 0)
    if target_kbps <= 0:
      return 0
    return int(target_kbps * SOURCE_WARMUP_MIN_KBPS_RATIO)

  def _reset_source_warmup(self) -> None:
    self._warmup_source = ""
    self._warmup_quality = 0
    self._warmup_started_mono = 0.0
    self._warmup_frames = 0
    self._warmup_bytes = 0
    self._warmup_kbps = 0.0
    self._warmup_last_frame_id = None

  def _source_warmup_ready(
    self,
    source: str,
    quality: int,
    frame_id: int | None,
    width: int,
    height: int,
    payload_bytes: int,
  ) -> bool:
    target = self._quality_target()
    target_width = int(target.get("width") or 0)
    target_height = int(target.get("height") or 0)
    if target_width > 0 and target_height > 0 and (width != target_width or height != target_height):
      self._reset_source_warmup()
      return False

    now = _mono()
    if self._warmup_source != source or self._warmup_quality != quality or not self._warmup_started_mono:
      self._warmup_source = source
      self._warmup_quality = quality
      self._warmup_started_mono = now
      self._warmup_frames = 0
      self._warmup_bytes = 0
      self._warmup_kbps = 0.0
      self._warmup_last_frame_id = None

    if (
      frame_id is not None
      and self._warmup_last_frame_id is not None
      and frame_id <= self._warmup_last_frame_id
    ):
      self._warmup_started_mono = now
      self._warmup_frames = 0
      self._warmup_bytes = 0
      self._warmup_kbps = 0.0

    self._warmup_last_frame_id = frame_id
    self._warmup_frames += 1
    self._warmup_bytes += max(0, int(payload_bytes))
    warmup_sec = max(0.001, now - self._warmup_started_mono)
    self._warmup_kbps = (self._warmup_bytes * 8 / 1000) / warmup_sec
    return (
      warmup_sec >= SOURCE_WARMUP_SECONDS
      and self._warmup_frames >= SOURCE_WARMUP_MIN_FRAMES
    )

  def _reset_stream_source_stats(self) -> None:
    self._stream_source_samples.clear()
    self._stream_source_bytes = 0
    self._stream_source_frames = 0
    self._stream_source_keyframes = 0

  def _record_stream_source_sample(self, payload_bytes: int, keyframe: bool) -> None:
    payload = max(0, int(payload_bytes))
    self._stream_source_samples.append((_mono(), payload, bool(keyframe)))
    self._stream_source_bytes += payload
    self._stream_source_frames += 1
    if keyframe:
      self._stream_source_keyframes += 1

  def _stream_source_recent(self, now: float) -> dict[str, Any]:
    cutoff = now - SOURCE_SAMPLE_WINDOW_SECONDS
    while self._stream_source_samples and self._stream_source_samples[0][0] < cutoff:
      self._stream_source_samples.popleft()
    if not self._stream_source_samples:
      return {"kbps": 0, "fps": 0.0}
    first_mono = self._stream_source_samples[0][0]
    duration = max(0.001, min(SOURCE_SAMPLE_WINDOW_SECONDS, now - first_mono))
    bytes_total = sum(sample[1] for sample in self._stream_source_samples)
    frames = len(self._stream_source_samples)
    return {
      "kbps": int((bytes_total * 8 / 1000) / duration),
      "fps": round(frames / duration, 1),
    }

  def _upload_recent_kbps(self, now: float) -> int | None:
    cutoff = now - UPLOAD_SAMPLE_WINDOW_SECONDS
    while self._sent_samples and self._sent_samples[0][0] < cutoff:
      self._sent_samples.popleft()
    if len(self._sent_samples) < 2:
      return None
    first_mono, first_bytes = self._sent_samples[0]
    duration = now - first_mono
    if duration < UPLOAD_SAMPLE_WINDOW_SECONDS * 0.8:
      return None
    return int(((self._bytes_sent - first_bytes) * 8 / 1000) / max(0.001, duration))

  async def _reconnect_if_starved(self) -> bool:
    # Automate the manual toggle off/on: a session formed while the network was
    # half-attached (first boot / hotspot) can stay "connected" while uploading
    # far below the CBR target, which looks like a frozen stream on YouTube.
    mono = _mono()
    self._sent_samples.append((mono, self._bytes_sent))
    if not self._started_mono or (mono - self._started_mono) < UPLOAD_STARVED_GRACE_SECONDS:
      self._starved_since_mono = 0.0
      return False
    required_kbps = self._source_warmup_required_kbps()
    upload_kbps = self._upload_recent_kbps(mono)
    if required_kbps <= 0 or upload_kbps is None:
      return False
    if upload_kbps >= required_kbps:
      self._starved_since_mono = 0.0
      return False
    if not self._starved_since_mono:
      self._starved_since_mono = mono
      return False
    if (mono - self._starved_since_mono) < UPLOAD_STARVED_SECONDS:
      return False
    self._starved_since_mono = 0.0
    # keep source warmup: the encoder is proven stable, only the network is bad.
    # skipping a full source re-warmup keeps the reconnect gap at a few seconds,
    # well inside YouTube's keep-alive window, so the broadcast is never split.
    await self._enter_backoff(
      f"upload starved: {upload_kbps} kbps below required {required_kbps} kbps",
      reason="upload starved",
    )
    return True

  async def _refresh_network(self) -> None:
    # Periodic, cached reachability probe. Poll faster while down so we notice a
    # hotspot re-attach quickly; slower while up to avoid needless connects.
    now = _mono()
    interval = NET_CHECK_UP_INTERVAL_SECONDS if self._net_ok else NET_CHECK_DOWN_INTERVAL_SECONDS
    if self._net_checked_mono and (now - self._net_checked_mono) < interval:
      return
    self._net_checked_mono = now
    ok = await asyncio.to_thread(_network_reachable)
    if ok != self._net_ok:
      self._log("network reachable" if ok else "network unreachable", "info" if ok else "warn")
    self._net_ok = ok

  def _get_socket(self) -> Any | None:
    source = self._current_source()
    quality = self._current_quality()
    if self._socket is not None and self._socket_source == source and self._socket_quality == quality:
      return self._socket
    # first subscription or the selected source (quality/camera) changed
    if self._socket is not None:
      try:
        self._socket.close()
      except Exception:
        pass
    self._socket = None
    messaging = self._get_messaging()
    if messaging is None:
      return None
    try:
      # conflate=False: never drop frames. H.264 P-frames reference earlier
      # frames, so dropping any frame corrupts the decode until the next keyframe
      # (the "blocky/깍두기" artifacts). We must relay every frame in order.
      self._socket = messaging.sub_sock(source, conflate=False)
      self._socket_source = source
      self._socket_quality = quality
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
    self._frame_fps = youtube_profile(self._current_quality()).fps
    try:
      await asyncio.to_thread(transport.connect)
      sink = RtmpSink(transport)
      muxer = await asyncio.to_thread(
        H264FlvMuxer,
        sink,
        codec_header=codec_header,
        fps=self._frame_fps,
        width=width,
        height=height,
      )
      writer = RtmpFrameWriter(
        muxer,
        max_frames=RTMP_FRAME_QUEUE_MAX,
        max_bytes=RTMP_FRAME_QUEUE_MAX_BYTES,
      )
      writer.start()
    except Exception:
      await asyncio.to_thread(transport.close)
      raise
    self._transport = transport
    self._transport_connected = True
    self._rtmp_sink = sink
    self._muxer = muxer
    self._writer = writer
    self._started_at = _now()
    self._started_mono = _mono()
    self._session_started_bytes = self._bytes_sent
    self._next_retry_mono = 0.0
    self._conn_checked_mono = self._started_mono
    self._active_source = self._current_source()
    self._active_quality = self._current_quality()
    self._caption_injector.reset()
    self._reset_stream_source_stats()
    self._sent_samples.clear()
    self._starved_since_mono = 0.0
    self._log(f"stream started ({width}x{height})")

  async def _write_frame(self, payload: bytes, *, keyframe: bool) -> bool:
    transport = self._transport
    writer = self._writer
    if transport is None or writer is None:
      return False
    # Never block source ingestion behind a network write. H.264 frames still
    # remain strictly ordered; if the bounded queue fills, restart from a fresh
    # keyframe instead of accumulating seconds of stale dependent frames.
    if not writer.enqueue(payload, keyframe=keyframe):
      reason = writer.last_rejection or "RTMP writer rejected a frame"
      if "backlog" in reason:
        self._writer_backpressure_restarts += 1
      await self._enter_backoff(
        f"YouTube RTMPS publish failed: {reason}",
        reason="RTMPS publish failed",
      )
      return False

    self._bytes_sent = self._session_started_bytes + transport.bytes_written
    if (
      self._started_mono
      and _mono() - self._started_mono >= STREAM_STABLE_SECONDS
      and writer.frames_written > 0
      and writer.pending_frames <= max(1, writer.capacity // 2)
      and writer.pending_bytes <= writer.capacity_bytes // 2
    ):
      self._consecutive_failures = 0
    return True

  async def _stop_stream(self) -> None:
    transport = self._transport
    self._transport = None
    self._transport_connected = False
    self._rtmp_sink = None
    muxer = self._muxer
    self._muxer = None
    writer = self._writer
    self._writer = None
    if transport is None and muxer is None and writer is None:
      self._started_at = 0.0
      self._started_mono = 0.0
      return
    self._set_state("stopping")
    if writer is not None:
      try:
        self._writer_discarded_frames += await writer.stop()
      except Exception:
        pass
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
    # single /proc pass matching every pattern at once
    pids_by_name: dict[str, list[int]] = {name: [] for name in _PROCESS_MATCHES}
    for proc_path in Path("/proc").glob("[0-9]*"):
      try:
        pid = int(proc_path.name)
      except ValueError:
        continue
      cmdline = _pid_cmdline(pid)
      if not cmdline:
        continue
      for name, match in _PROCESS_MATCHES.items():
        if match in cmdline:
          pids_by_name[name].append(pid)
    result = {
      name: {"running": bool(pids), "pids": sorted(pids)[:8]}
      for name, pids in pids_by_name.items()
    }
    self._proc_cache = result
    self._proc_cache_mono = mono
    return result

  def _resource_status(self) -> dict[str, Any]:
    processes = self._process_status()
    cluster_param = self._param_int("ClusterHud", 0)
    disable_dm = self._param_int("DisableDM", 0)
    quality = self._param_int(YOUTUBE_QUALITY_PARAM, 0)
    selected_youtube_encoder = youtube_profile(quality).process_name
    selected_process = processes.get(selected_youtube_encoder, {"running": False, "pids": []})
    cluster_process = processes.get("carrot_cluster", {"running": False, "pids": []})
    vision_webrtc = processes.get("webrtcd", {"running": False, "pids": []})
    vision_encoder = processes.get("stream_encoderd", {"running": False, "pids": []})
    vision_active = bool(vision_webrtc.get("running") or vision_encoder.get("running"))
    return {
      "cluster": {
        "enabled": cluster_param == 1,
        "configured": cluster_param == 1,
        "active": bool(cluster_process.get("running")),
        "param": cluster_param,
        **cluster_process,
      },
      "carrot_vision": {
        "enabled": disable_dm == 2,
        "configured": disable_dm == 2,
        "active": vision_active,
        "disable_dm": disable_dm,
        "webrtcd_running": bool(vision_webrtc.get("running")),
        "stream_encoderd_running": bool(vision_encoder.get("running")),
        "webrtcd_pids": list(vision_webrtc.get("pids") or []),
        "stream_encoderd_pids": list(vision_encoder.get("pids") or []),
      },
      "youtube_encoder": {
        "selected": selected_youtube_encoder,
        **selected_process,
      },
    }

  def _warnings(self, resource_status: dict[str, Any]) -> list[str]:
    warnings = []
    if not self._param_enabled():
      return warnings
    cluster = resource_status.get("cluster") if isinstance(resource_status, dict) else {}
    vision = resource_status.get("carrot_vision") if isinstance(resource_status, dict) else {}
    if cluster and cluster.get("active", cluster.get("running")):
      warnings.append("Cluster HUD is enabled; monitor overall load and temperature during simultaneous use.")
    vision_active = bool(
      vision
      and (
        vision.get("active")
        or vision.get("webrtcd_running")
        or vision.get("stream_encoderd_running")
      )
    )
    if vision_active:
      warnings.append("Carrot Vision is enabled; simultaneous streaming increases network and memory bandwidth use.")
    youtube_encoder = resource_status.get("youtube_encoder") if isinstance(resource_status, dict) else {}
    if not (youtube_encoder and youtube_encoder.get("running")):
      warnings.append("The selected YouTube encoder is waiting to start onroad.")
    target = self._quality_target()
    target_width = int(target.get("width") or 0)
    target_height = int(target.get("height") or 0)
    if target_width > 0 and target_height > 0 and self._last_frame_mono and not self._frame_matches_target(target):
      actual_resolution = f"{self._frame_width}x{self._frame_height}"
      warnings.append(f"YouTube encoder target is {target_width}x{target_height}, but current frames are {actual_resolution}.")
    target_kbps = int(target.get("video_kbps") or 0)
    if self._transport_connected and self._started_mono and target_kbps > 0:
      mono = _mono()
      age = mono - self._started_mono
      session_bytes = max(0, self._bytes_sent - self._session_started_bytes)
      estimated_kbps = int((session_bytes * 8 / 1000) / max(1.0, age))
      required_kbps = self._source_warmup_required_kbps(target)
      if age >= SOURCE_WARMUP_SECONDS and estimated_kbps < required_kbps:
        warnings.append(
          f"YouTube ingest may be starved: current bitrate is below {required_kbps} kbps for target {target_kbps} kbps."
        )
    return warnings


def _pid_cmdline(pid: int) -> str:
  try:
    return Path(f"/proc/{int(pid)}/cmdline").read_bytes().decode(errors="replace")
  except Exception:
    return ""
