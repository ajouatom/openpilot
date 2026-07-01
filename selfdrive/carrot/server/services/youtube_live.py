from __future__ import annotations

import asyncio
import json
import os
import re
import shutil
import socket
import ssl
import time
from collections import deque
from pathlib import Path
from typing import Any

from ..config import CARROT_YOUTUBE_LIVE_SECRET_PATH, CARROT_YOUTUBE_LIVE_STATE_PATH


YOUTUBE_LIVE_PARAM = "CarrotYouTubeLive"
YOUTUBE_QUALITY_PARAM = "CarrotYouTubeQuality"
PHASE1_SOURCE_SERVICE = "qRoadEncodeData"
PHASE1_QUALITY = "standard"
YOUTUBE_RTMPS_BASE = "rtmps://a.rtmps.youtube.com/live2"
YOUTUBE_RTMPS_HOST = "a.rtmps.youtube.com"
YOUTUBE_RTMPS_PORT = 443
NO_FRAME_STOP_SECONDS = 8.0
START_BACKOFF_SECONDS = 5.0
STATUS_WRITE_MIN_INTERVAL = 1.0
BACKOFF_BASE_SECONDS = 3.0
BACKOFF_MAX_SECONDS = 60.0

_PROCESS_MATCHES = {
  "carrot_cluster": "selfdrive.carrot.cluster_autorun",
  "webrtcd": "system.webrtc.webrtcd",
  "stream_encoderd": "system/loggerd/encoderd\x00--stream",
}


def _now() -> float:
  return time.time()


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
    self._process: asyncio.subprocess.Process | None = None
    self._stderr_task: asyncio.Task | None = None
    self._stderr_tail: deque[str] = deque(maxlen=20)
    self._messaging: Any | None = None
    self._socket: Any | None = None
    self._params: Any | None = None
    self._last_status_write = 0.0
    self._started_at = 0.0
    self._last_frame_at = 0.0
    self._last_frame_id: int | None = None
    self._bytes_sent = 0
    self._session_started_bytes = 0
    self._restart_count = 0
    self._consecutive_failures = 0
    self._next_retry_at = 0.0
    self._last_error = ""
    self._state = "disabled"
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
    await self._stop_process()
    self._task = None
    self._write_status(force=True)

  def status(self) -> dict[str, Any]:
    stream_key = self.get_stream_key()
    pid = self._process.pid if self._process and self._process.returncode is None else None
    now = _now()
    uptime = int(now - self._started_at) if self._started_at and pid else 0
    elapsed = max(1.0, now - self._started_at) if self._started_at and pid else 1.0
    last_frame_age_ms = int((now - self._last_frame_at) * 1000) if self._last_frame_at else None
    resource_status = self._resource_status()
    warnings = self._warnings(resource_status)
    return {
      "state": self._state,
      "enabled": self._param_enabled(),
      "configured": bool(stream_key),
      "masked_key": _mask_stream_key(stream_key),
      "source": PHASE1_SOURCE_SERVICE,
      "quality": PHASE1_QUALITY,
      "requested_quality": self._param_int(YOUTUBE_QUALITY_PARAM, 0),
      "phase": 1,
      "running": bool(pid),
      "pid": pid,
      "ffmpeg_available": bool(shutil.which("ffmpeg")),
      "started_at": self._started_at if pid else 0,
      "uptime_sec": uptime,
      "bytes_sent": self._bytes_sent,
      "total_mb": round(max(0, self._bytes_sent) / (1024 * 1024), 2),
      "session_bytes": max(0, self._bytes_sent - self._session_started_bytes),
      "session_mb": round(max(0, self._bytes_sent - self._session_started_bytes) / (1024 * 1024), 2),
      "estimated_kbps": int(((self._bytes_sent - self._session_started_bytes) * 8 / 1000) / elapsed) if pid else 0,
      "restart_count": self._restart_count,
      "consecutive_failures": self._consecutive_failures,
      "next_retry_at": self._next_retry_at,
      "retry_in_sec": max(0, int(self._next_retry_at - now)) if self._next_retry_at else 0,
      "last_error": self._last_error,
      "last_frame_at": self._last_frame_at,
      "last_frame_age_ms": last_frame_age_ms,
      "last_frame_id": self._last_frame_id,
      "stderr_tail": list(self._stderr_tail)[-5:],
      "resource_status": resource_status,
      "warnings": warnings,
    }

  def test_config(self) -> dict[str, Any]:
    stream_key = self.get_stream_key()
    ffmpeg_path = shutil.which("ffmpeg")
    ok = bool(stream_key and ffmpeg_path)
    status = self.status()
    return {
      "ok": ok,
      "configured": bool(stream_key),
      "ffmpeg_available": bool(ffmpeg_path),
      "ffmpeg_path": ffmpeg_path or "",
      "source": PHASE1_SOURCE_SERVICE,
      "quality": PHASE1_QUALITY,
      "resource_status": status["resource_status"],
      "warnings": status["warnings"],
      "stderr_tail": list(self._stderr_tail),
      "message": "ready" if ok else "missing stream key or ffmpeg",
    }

  def validate_stream_key(self, value: str | None = None) -> dict[str, Any]:
    stream_key = _extract_stream_key(value) if value is not None else self.get_stream_key()
    format_ok, format_message = _validate_stream_key_format(stream_key)
    rtmps_ok, rtmps_message = _check_rtmps_reachable()
    ffmpeg_path = shutil.which("ffmpeg")
    ok = bool(format_ok and rtmps_ok and ffmpeg_path)
    return {
      "ok": ok,
      "configured": bool(self.get_stream_key()),
      "format_ok": format_ok,
      "format_message": format_message,
      "rtmps_reachable": rtmps_ok,
      "rtmps_message": rtmps_message,
      "ffmpeg_available": bool(ffmpeg_path),
      "ffmpeg_path": ffmpeg_path or "",
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
        "source": PHASE1_SOURCE_SERVICE,
        "quality": PHASE1_QUALITY,
        "rtmps_ingest": f"{YOUTUBE_RTMPS_BASE}/{_mask_stream_key(stream_key)}" if stream_key else "",
      },
      "params": {
        YOUTUBE_LIVE_PARAM: self._param_enabled(),
        YOUTUBE_QUALITY_PARAM: self._param_int(YOUTUBE_QUALITY_PARAM, 0),
        "ClusterHud": self._param_int("ClusterHud", 0),
        "DisableDM": self._param_int("DisableDM", 0),
        "IsOnroad": self._param_bool("IsOnroad", False),
      },
      "ffmpeg": {
        "available": bool(shutil.which("ffmpeg")),
        "path": shutil.which("ffmpeg") or "",
        "stderr_tail": list(self._stderr_tail),
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
        await self._stop_process()
        await asyncio.sleep(START_BACKOFF_SECONDS)
      await asyncio.sleep(0.02 if self._process else 0.25)

  async def _tick(self) -> None:
    if not self._param_enabled():
      if self._process is not None:
        await self._stop_process()
      self._set_state("disabled")
      return

    stream_key = self.get_stream_key()
    if not stream_key:
      await self._stop_process()
      self._set_state("needs_setup")
      return

    if not shutil.which("ffmpeg"):
      await self._stop_process()
      self._last_error = "ffmpeg not found"
      self._set_state("error")
      return

    if self._param_int(YOUTUBE_QUALITY_PARAM, 0) > 0:
      await self._stop_process()
      self._last_error = "high quality YouTube Live is planned for Phase 3"
      self._set_state("error")
      return

    if self._next_retry_at and _now() < self._next_retry_at:
      self._set_state("backoff")
      return

    payload, frame_id = self._recv_payload()
    if not payload:
      if self._process is not None and self._last_frame_at and _now() - self._last_frame_at > NO_FRAME_STOP_SECONDS:
        self._last_error = "no qRoadEncodeData frames"
        await self._stop_process()
        self._schedule_backoff("frame timeout")
      self._set_state("waiting_frame" if self._process is None else "live")
      return

    self._last_frame_at = _now()
    self._last_frame_id = frame_id

    if self._process is None or self._process.returncode is not None:
      await self._start_process(stream_key)

    await self._write_frame(payload)
    self._set_state("live")

  def _set_state(self, state: str) -> None:
    self._state = state
    self._write_status()

  def _write_status(self, *, force: bool = False) -> None:
    now = _now()
    if not force and now - self._last_status_write < STATUS_WRITE_MIN_INTERVAL:
      return
    self._last_status_write = now
    try:
      _write_json_atomic(self.state_path, {"updated_at": now, **self.status()})
    except Exception:
      pass

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
      from cereal import messaging
      self._messaging = messaging
    except Exception as exc:
      self._last_error = f"messaging unavailable: {exc}"
      self._messaging = None
    return self._messaging

  def _get_socket(self) -> Any | None:
    if self._socket is not None:
      return self._socket
    messaging = self._get_messaging()
    if messaging is None:
      return None
    try:
      self._socket = messaging.sub_sock(PHASE1_SOURCE_SERVICE, conflate=True)
    except Exception as exc:
      self._last_error = f"{PHASE1_SOURCE_SERVICE} socket failed: {exc}"
      self._socket = None
    return self._socket

  def _recv_payload(self) -> tuple[bytes, int | None]:
    messaging = self._get_messaging()
    sock = self._get_socket()
    if messaging is None or sock is None:
      return b"", None
    try:
      msg = messaging.recv_one_or_none(sock)
      if msg is None:
        return b"", None
      which = msg.which()
      frame = getattr(msg, which, None)
      if frame is None:
        return b"", None
      header = bytes(getattr(frame, "header", b"") or b"")
      data = bytes(getattr(frame, "data", b"") or b"")
      frame_id = None
      idx = getattr(frame, "idx", None)
      if idx is not None:
        try:
          frame_id = int(getattr(idx, "frameId", 0) or 0) or None
        except Exception:
          frame_id = None
      if frame_id is None:
        try:
          frame_id = int(getattr(frame, "frameId", 0) or 0) or None
        except Exception:
          frame_id = None
      return header + data, frame_id
    except Exception as exc:
      self._last_error = f"{PHASE1_SOURCE_SERVICE} recv failed: {exc}"
      return b"", None

  async def _start_process(self, stream_key: str) -> None:
    await self._stop_process()
    self._set_state("starting")
    self._stderr_tail.clear()
    self._started_at = _now()
    self._session_started_bytes = self._bytes_sent
    rtmp_url = f"{YOUTUBE_RTMPS_BASE}/{stream_key}"
    cmd = [
      "ffmpeg",
      "-hide_banner",
      "-loglevel",
      "warning",
      "-fflags",
      "nobuffer",
      "-f",
      "h264",
      "-framerate",
      "20",
      "-i",
      "pipe:0",
      "-f",
      "lavfi",
      "-i",
      "anullsrc=r=44100:cl=stereo",
      "-c:v",
      "copy",
      "-c:a",
      "aac",
      "-b:a",
      "128k",
      "-f",
      "flv",
      rtmp_url,
    ]
    self._process = await asyncio.create_subprocess_exec(
      *cmd,
      stdin=asyncio.subprocess.PIPE,
      stdout=asyncio.subprocess.DEVNULL,
      stderr=asyncio.subprocess.PIPE,
    )
    self._restart_count += 1
    self._next_retry_at = 0.0
    self._stderr_task = asyncio.create_task(self._read_stderr(self._process), name="carrot-youtube-live-stderr")

  async def _write_frame(self, payload: bytes) -> None:
    proc = self._process
    if proc is None or proc.stdin is None:
      return
    if proc.returncode is not None:
      self._last_error = f"ffmpeg exited code={proc.returncode}"
      self._schedule_backoff("ffmpeg exited")
      await self._stop_process()
      return
    try:
      proc.stdin.write(payload)
      await proc.stdin.drain()
      self._bytes_sent += len(payload)
      self._consecutive_failures = 0
    except (BrokenPipeError, ConnectionResetError) as exc:
      self._last_error = f"ffmpeg pipe closed: {exc}"
      self._schedule_backoff("ffmpeg pipe closed")
      await self._stop_process()

  async def _read_stderr(self, proc: asyncio.subprocess.Process) -> None:
    if proc.stderr is None:
      return
    while True:
      line = await proc.stderr.readline()
      if not line:
        break
      text = line.decode("utf-8", errors="replace").strip()
      if text:
        self._stderr_tail.append(text[-300:])
        self._last_error = text[-300:]

  async def _stop_process(self) -> None:
    proc = self._process
    self._process = None
    stderr_task = self._stderr_task
    self._stderr_task = None
    if proc is None:
      if stderr_task is not None and not stderr_task.done():
        stderr_task.cancel()
      return
    self._set_state("stopping")
    if proc.stdin is not None:
      try:
        proc.stdin.close()
        await proc.stdin.wait_closed()
      except Exception:
        pass
    try:
      await asyncio.wait_for(proc.wait(), timeout=2.0)
    except asyncio.TimeoutError:
      try:
        proc.terminate()
      except ProcessLookupError:
        pass
      try:
        await asyncio.wait_for(proc.wait(), timeout=3.0)
      except asyncio.TimeoutError:
        try:
          proc.kill()
        except ProcessLookupError:
          pass
        await proc.wait()
    if stderr_task is not None and not stderr_task.done():
      stderr_task.cancel()
      try:
        await stderr_task
      except asyncio.CancelledError:
        pass
    self._started_at = 0.0

  def _schedule_backoff(self, reason: str = "") -> None:
    self._consecutive_failures += 1
    delay = min(BACKOFF_MAX_SECONDS, BACKOFF_BASE_SECONDS * (2 ** max(0, self._consecutive_failures - 1)))
    self._next_retry_at = _now() + delay
    if reason and not self._last_error:
      self._last_error = reason

  def _process_status(self) -> dict[str, Any]:
    result = {}
    for name, match in _PROCESS_MATCHES.items():
      pids = _find_matching_pids(match)
      result[name] = {
        "running": bool(pids),
        "pids": pids[:8],
      }
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
    }

  def _warnings(self, resource_status: dict[str, Any]) -> list[str]:
    warnings = []
    if not self._param_enabled():
      return warnings
    cluster = resource_status.get("cluster") if isinstance(resource_status, dict) else {}
    vision = resource_status.get("carrot_vision") if isinstance(resource_status, dict) else {}
    if cluster and cluster.get("enabled"):
      warnings.append("Cluster HUD is enabled; monitor encoder load and thermal headroom while streaming.")
    if vision and vision.get("enabled"):
      warnings.append("Carrot Vision is enabled; YouTube Live shares camera/encoder/network resources.")
    if self._param_int(YOUTUBE_QUALITY_PARAM, 0) > 0:
      warnings.append("High quality mode is planned for Phase 3; Phase 2 keeps qRoadEncodeData only.")
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
