from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from urllib.request import urlopen

from ..config import CARROT_YOUTUBE_LIVE_SECRET_PATH, CARROT_YOUTUBE_LIVE_STATE_PATH
from .youtube_h264 import validate_stream_start
from .youtube_profiles import YOUTUBE_PROFILES, youtube_profile


REPO_ROOT = Path("/data/openpilot")
STATE_PATH = Path("/tmp/carrot-youtube-test-state.json")
LOG_PATH = Path("/tmp/carrot-youtube-test.log")
REPORT_PATH = Path("/tmp/carrot-youtube-test-report.json")
TIMEOUT_SECONDS = 10 * 60
VERIFY_TIMEOUT_SECONDS = 75
VERIFY_STABLE_SECONDS = 10
RUNNER_MODULE = "openpilot.selfdrive.carrot.server.services.youtube_test"
YOUTUBE_STATUS_URL = "http://127.0.0.1:7000/api/youtube_live/status"
YOUTUBE_SOURCE = youtube_profile(0).source

_CAMERAD_SPEC = {
  "cmd": [str(REPO_ROOT / "openpilot/system/camerad/camerad")],
  "cwd": str(REPO_ROOT),
  "match": "openpilot/system/camerad/camerad",
}
_ENCODER_PATH = str(REPO_ROOT / "openpilot/system/loggerd/encoderd")
_QUALITY_SPECS = {
  quality: profile.encoder_spec(_ENCODER_PATH, cwd=str(REPO_ROOT))
  for quality, profile in YOUTUBE_PROFILES.items()
}
_CONFLICT_MATCHES = {
  "camerad": _CAMERAD_SPEC["match"],
  "carrot_vision_encoderd": "openpilot/system/loggerd/encoderd\x00--carrot-vision-road",
  **{str(spec["name"]): str(spec["match"]) for spec in _QUALITY_SPECS.values()},
}


def _params():
  from openpilot.common.params import Params
  return Params()


def _quality_spec(value: int) -> dict[str, Any]:
  return dict(_QUALITY_SPECS[youtube_profile(value).quality])


def _set_snapshot_active(active: bool) -> None:
  params = _params()
  params.put_bool("IsTakingSnapshot", active)
  try:
    from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
    set_offroad_alert("Offroad_IsTakingSnapshot", active)
  except Exception:
    pass


def _write_state(state: dict[str, Any]) -> None:
  state["updated_mono"] = time.monotonic()
  temp_path = STATE_PATH.with_suffix(".tmp")
  temp_path.write_text(json.dumps(state, sort_keys=True), encoding="utf-8")
  temp_path.replace(STATE_PATH)


def _read_json(path: Path) -> dict[str, Any]:
  try:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return raw if isinstance(raw, dict) else {}
  except Exception:
    return {}


def _read_state() -> dict[str, Any]:
  return _read_json(STATE_PATH)


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
  matches = []
  for proc_path in Path("/proc").glob("[0-9]*"):
    try:
      pid = int(proc_path.name)
    except ValueError:
      continue
    if match in _pid_cmdline(pid):
      matches.append(pid)
  return sorted(matches)


def _tail_log(lines: int) -> list[str]:
  try:
    return LOG_PATH.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
  except Exception:
    return []


def _vipc_streams() -> list[int]:
  try:
    from msgq.visionipc import VisionIpcClient
    return sorted(int(stream) for stream in VisionIpcClient.available_streams("camerad", block=False))
  except Exception:
    return []


def _fetch_youtube_status() -> dict[str, Any]:
  try:
    with urlopen(YOUTUBE_STATUS_URL, timeout=0.5) as response:
      payload = json.loads(response.read().decode("utf-8"))
      if isinstance(payload, dict):
        return payload
  except Exception:
    pass
  return _read_json(Path(CARROT_YOUTUBE_LIVE_STATE_PATH))


def _stream_key_configured() -> bool:
  payload = _read_json(Path(CARROT_YOUTUBE_LIVE_SECRET_PATH))
  return bool(str(payload.get("stream_key") or "").strip())


def _child_specs(state: dict[str, Any]) -> dict[str, dict[str, Any]]:
  quality = int(state.get("quality") or 0)
  encoder = _quality_spec(quality)
  return {
    "camerad": _CAMERAD_SPEC,
    str(encoder["name"]): encoder,
  }


def _children_status(state: dict[str, Any]) -> dict[str, dict[str, Any]]:
  children = state.get("children") if isinstance(state.get("children"), dict) else {}
  result = {}
  for name, spec in _child_specs(state).items():
    pid = int(children.get(name) or 0)
    result[name] = {"pid": pid, "alive": _pid_alive(pid, str(spec["match"]))}
  return result


def get_status() -> dict[str, Any]:
  state = _read_state()
  runner_pid = int(state.get("runner_pid") or 0)
  runner_alive = _pid_alive(runner_pid, RUNNER_MODULE)
  status = str(state.get("status") or "stopped")
  try:
    params = _params()
    device = {
      "is_offroad": bool(params.get_bool("IsOffroad")),
      "is_onroad": bool(params.get_bool("IsOnroad")),
      "live_enabled": int(params.get_int("CarrotYouTubeLive")) > 0,
      "quality": int(params.get_int("CarrotYouTubeQuality")),
      "timestamp_enabled": int(params.get_int("CarrotYouTubeTimestamp")) > 0,
    }
  except Exception:
    device = {}
  return {
    **state,
    "status": status if runner_alive or status == "error" else "stopped",
    "runner_pid": runner_pid,
    "runner_alive": runner_alive,
    "children": _children_status(state),
    "vipc_streams": _vipc_streams(),
    "youtube": _fetch_youtube_status(),
    "device": device,
    "log_path": str(LOG_PATH),
  }


def _format_duration(seconds: float) -> str:
  seconds = max(0, int(seconds))
  return f"{seconds // 3600:02d}:{(seconds % 3600) // 60:02d}:{seconds % 60:02d}"


def _diagnose_status(status: dict[str, Any]) -> dict[str, Any]:
  youtube = status.get("youtube") if isinstance(status.get("youtube"), dict) else {}
  failures: list[str] = []
  warnings = [str(item) for item in youtube.get("warnings") or [] if str(item).strip()]
  children = status.get("children") if isinstance(status.get("children"), dict) else {}

  if not status.get("runner_alive"):
    failures.append("test runner is not running")
  for name, child in children.items():
    if not isinstance(child, dict) or not child.get("alive"):
      failures.append(f"{name} is not running")

  state = str(youtube.get("state") or "")
  if state != "live":
    failures.append(f"YouTube state is {state or 'unknown'}")
  if not youtube.get("transport_connected"):
    failures.append("RTMPS transport is not connected")
  if youtube.get("frame_matches_target") is False:
    failures.append("encoder frame size does not match the selected profile")

  target_fps = max(1, int(youtube.get("target_fps") or youtube.get("declared_frame_fps") or 20))
  source_fps = float(youtube.get("stream_source_recent_fps") or 0)
  minimum_fps = target_fps * 0.75
  if source_fps < minimum_fps:
    failures.append(f"source rate is {source_fps:.1f} fps (minimum {minimum_fps:.1f})")

  target_kbps = max(1, int(youtube.get("target_video_kbps") or 1))
  upload_kbps = int(youtube.get("upload_recent_kbps") or 0)
  minimum_upload_kbps = int(target_kbps * 0.55)
  if upload_kbps < minimum_upload_kbps:
    failures.append(f"upload rate is {upload_kbps} kbps (minimum {minimum_upload_kbps} kbps)")

  frames_written = int(youtube.get("rtmp_writer_frames_written") or 0)
  if frames_written <= 0:
    failures.append("RTMP writer has not published a frame")
  writer_error = str(youtube.get("rtmp_writer_error") or "").strip()
  if writer_error:
    failures.append(f"RTMP writer error: {writer_error}")

  pending_frames = int(youtube.get("rtmp_writer_pending_frames") or 0)
  capacity_frames = max(1, int(youtube.get("rtmp_writer_capacity") or 1))
  pending_bytes = int(youtube.get("rtmp_writer_pending_bytes") or 0)
  capacity_bytes = max(1, int(youtube.get("rtmp_writer_capacity_bytes") or 1))
  if pending_frames >= capacity_frames or pending_bytes >= capacity_bytes:
    failures.append("RTMP writer backlog reached its limit")
  elif pending_frames >= capacity_frames * 0.8 or pending_bytes >= capacity_bytes * 0.8:
    warnings.append("RTMP writer backlog is above 80%")

  last_frame_age_ms = youtube.get("last_frame_age_ms")
  if last_frame_age_ms is not None and int(last_frame_age_ms) > 2_000:
    failures.append(f"last source frame is {int(last_frame_age_ms)} ms old")
  error = str(status.get("error") or youtube.get("last_error") or "").strip()
  if error:
    failures.append(error)

  failures = list(dict.fromkeys(failures))
  warnings = list(dict.fromkeys(warnings))
  return {
    "healthy": not failures,
    "verdict": "pass" if not failures else "fail",
    "failures": failures,
    "warnings": warnings,
    "thresholds": {
      "minimum_source_fps": round(minimum_fps, 1),
      "minimum_upload_kbps": minimum_upload_kbps,
    },
  }


def _compact_report(status: dict[str, Any]) -> dict[str, Any]:
  youtube = status.get("youtube") if isinstance(status.get("youtube"), dict) else {}
  youtube_fields = (
    "state",
    "quality",
    "target_width",
    "target_height",
    "target_fps",
    "target_video_kbps",
    "frame_width",
    "frame_height",
    "frame_matches_target",
    "stream_source_recent_fps",
    "stream_source_recent_kbps",
    "stream_source_keyframes",
    "upload_recent_kbps",
    "estimated_kbps",
    "transport_connected",
    "rtmp_writer_pending_frames",
    "rtmp_writer_capacity",
    "rtmp_writer_high_watermark",
    "rtmp_writer_pending_bytes",
    "rtmp_writer_capacity_bytes",
    "rtmp_writer_high_watermark_bytes",
    "rtmp_writer_frames_written",
    "rtmp_writer_last_write_ms",
    "rtmp_writer_max_write_ms",
    "rtmp_writer_error",
    "rtmp_writer_backpressure_restarts",
    "rtmp_writer_discarded_frames",
    "restart_count",
    "consecutive_failures",
    "retry_in_sec",
    "last_frame_age_ms",
    "last_error",
    "warnings",
    "log_tail",
  )
  return {
    "version": 1,
    "captured_at": time.time(),  # noqa: TID251 - report needs a shareable wall-clock timestamp
    "diagnosis": _diagnose_status(status),
    "test": {
      "status": status.get("status"),
      "runner_alive": bool(status.get("runner_alive")),
      "quality": status.get("quality_label") or status.get("quality"),
      "children": status.get("children"),
      "vipc_streams": status.get("vipc_streams"),
      "device": status.get("device"),
    },
    "youtube": {field: youtube.get(field) for field in youtube_fields},
    "runner_log_tail": _tail_log(40),
  }


def _write_report(status: dict[str, Any]) -> dict[str, Any]:
  report = _compact_report(status)
  temp_path = REPORT_PATH.with_suffix(".tmp")
  temp_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
  temp_path.replace(REPORT_PATH)
  return report


def print_status(status: dict[str, Any] | None = None) -> int:
  status = status or get_status()
  report = _write_report(status)
  diagnosis = report["diagnosis"]
  started_mono = float(status.get("started_mono") or 0)
  elapsed = _format_duration(time.monotonic() - started_mono) if started_mono else "00:00:00"
  quality = str(status.get("quality_label") or _quality_spec(int(status.get("quality") or 0))["label"])
  print(f"[youtube-test] {status['status']} elapsed={elapsed} quality={quality}")
  print(f"  runner           pid={status['runner_pid'] or '-'} alive={str(status['runner_alive']).lower()}")
  for name, child in status["children"].items():
    print(f"  {name:<16} pid={child['pid'] or '-'} alive={str(child['alive']).lower()}")
  streams = ", ".join(str(stream) for stream in status["vipc_streams"]) or "-"
  print(f"  VIPC streams     {streams}")
  youtube = status.get("youtube") if isinstance(status.get("youtube"), dict) else {}
  print(f"  YouTube state    {youtube.get('state') or '-'}")
  source_fps = youtube.get("stream_source_recent_fps") or 0
  source_kbps = youtube.get("stream_source_recent_kbps") or 0
  source_keyframes = youtube.get("stream_source_keyframes") or 0
  print(f"  source           {source_fps} fps / {source_kbps} kbps / keyframes={source_keyframes}")
  upload_kbps = youtube.get("upload_recent_kbps") or 0
  drain_calls = youtube.get("rtmp_drain_calls") or 0
  pending_bytes = youtube.get("mux_pending_bytes") or 0
  print(f"  upload           {upload_kbps} kbps / drains={drain_calls} / pending={pending_bytes} B")
  writer_pending = int(youtube.get("rtmp_writer_pending_frames") or 0)
  writer_capacity = int(youtube.get("rtmp_writer_capacity") or 0)
  writer_pending_bytes = int(youtube.get("rtmp_writer_pending_bytes") or 0)
  writer_capacity_bytes = int(youtube.get("rtmp_writer_capacity_bytes") or 0)
  last_write_ms = int(youtube.get("rtmp_writer_last_write_ms") or 0)
  max_write_ms = int(youtube.get("rtmp_writer_max_write_ms") or 0)
  print(
    f"  writer           {writer_pending}/{writer_capacity} frames / " +
    f"{writer_pending_bytes}/{writer_capacity_bytes} B / write={last_write_ms}ms max={max_write_ms}ms"
  )
  print(f"  verdict          {diagnosis['verdict']}")
  for failure in diagnosis["failures"]:
    print(f"  fail             {failure}")
  for warning in diagnosis["warnings"]:
    print(f"  warning          {warning}")
  print(f"  log              {status['log_path']}")
  print(f"  report           {REPORT_PATH}")
  error = str(status.get("error") or youtube.get("last_error") or "").strip()
  if error:
    print(f"  error            {error}")
  return 0 if status["runner_alive"] else 1


def _terminate_pid(pid: int, match: str, timeout: float = 3.0) -> None:
  if not _pid_alive(pid, match):
    return
  try:
    os.kill(pid, signal.SIGTERM)
  except OSError:
    return
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if not _pid_alive(pid, match):
      return
    time.sleep(0.1)
  if _pid_alive(pid, match):
    try:
      os.kill(pid, signal.SIGKILL)
    except OSError:
      pass


def _restore_live_param(state: dict[str, Any]) -> None:
  if not bool(state.get("forced_live")):
    return
  try:
    params = _params()
    if int(params.get_int("CarrotYouTubeLive")) > 0:
      params.put_int("CarrotYouTubeLive", int(state.get("previous_live") or 0))
  except Exception:
    pass


def _cleanup_owned_processes(state: dict[str, Any]) -> None:
  children = state.get("children") if isinstance(state.get("children"), dict) else {}
  for name, spec in reversed(tuple(_child_specs(state).items())):
    _terminate_pid(int(children.get(name) or 0), str(spec["match"]))


def start_test(*, announce_next_step: bool = True) -> int:
  status = get_status()
  if status["runner_alive"]:
    print("[youtube-test] already running")
    return print_status()

  params = _params()
  if not params.get_bool("IsOffroad"):
    print("[youtube-test] refused: device is not offroad", file=sys.stderr)
    return 1
  if params.get_bool("IsTakingSnapshot"):
    print("[youtube-test] refused: another camera test is active", file=sys.stderr)
    return 1
  if int(params.get_int("CarrotYouTubeTimestamp")) > 0:
    print("[youtube-test] refused: disable CarrotYouTubeTimestamp first", file=sys.stderr)
    return 1
  if not _stream_key_configured():
    print("[youtube-test] refused: YouTube stream key is not configured", file=sys.stderr)
    return 1

  for name, match in _CONFLICT_MATCHES.items():
    pids = _find_matching_pids(str(match))
    if pids:
      print(f"[youtube-test] refused: {name} already running pid={','.join(map(str, pids))}", file=sys.stderr)
      return 1

  quality = int(params.get_int("CarrotYouTubeQuality"))
  if quality not in _QUALITY_SPECS:
    quality = 0
  LOG_PATH.write_text("", encoding="utf-8")
  with LOG_PATH.open("a", encoding="utf-8") as log:
    proc = subprocess.Popen(
      [sys.executable, "-m", RUNNER_MODULE, "_run", "--quality", str(quality)],
      cwd=str(REPO_ROOT),
      stdin=subprocess.DEVNULL,
      stdout=log,
      stderr=subprocess.STDOUT,
      start_new_session=True,
    )

  print(f"[youtube-test] starting runner pid={proc.pid}")
  # camerad readiness (8 s) and the first encoder keyframe (12 s) are
  # sequential, so leave enough room for both plus process startup overhead.
  deadline = time.monotonic() + 30.0
  while time.monotonic() < deadline:
    time.sleep(0.25)
    status = get_status()
    if status.get("status") == "running":
      if announce_next_step:
        print("[youtube-test] ready; run 'carrot youtube-test status' after the upload metrics settle")
        return print_status(status)
      print("[youtube-test] pipeline ready; collecting verification metrics")
      return 0
    if status.get("status") == "error":
      print(f"[youtube-test] failed: {status.get('error') or 'unknown error'}", file=sys.stderr)
      return 1
    if proc.poll() is not None:
      print("[youtube-test] runner exited during startup", file=sys.stderr)
      return 1

  print("[youtube-test] startup is still in progress; run 'carrot youtube-test status'", file=sys.stderr)
  return 1


def stop_test() -> int:
  state = _read_state()
  runner_pid = int(state.get("runner_pid") or 0)
  if _pid_alive(runner_pid, RUNNER_MODULE):
    print(f"[youtube-test] stopping runner pid={runner_pid}")
    _terminate_pid(runner_pid, RUNNER_MODULE, timeout=15.0)
  elif state:
    print("[youtube-test] runner is not active; cleaning stale state")
    _cleanup_owned_processes(state)
    _restore_live_param(state)
    _set_snapshot_active(False)
  else:
    print("[youtube-test] runner is not active")

  state = _read_state()
  if state:
    state.update({"status": "stopped", "children": {}, "error": ""})
    _write_state(state)
  print("[youtube-test] stopped")
  return 0


def verify_test() -> int:
  if get_status().get("runner_alive"):
    print("[youtube-test] refused: stop the active test before one-shot verification", file=sys.stderr)
    return 1

  started = False
  result = 1
  try:
    start_result = start_test(announce_next_step=False)
    started = bool(get_status().get("runner_alive"))
    if start_result != 0:
      return 1
    deadline = time.monotonic() + VERIFY_TIMEOUT_SECONDS
    stable_since = 0.0
    final_status = get_status()

    while time.monotonic() < deadline:
      final_status = get_status()
      if final_status.get("status") == "error" or not final_status.get("runner_alive"):
        break
      diagnosis = _diagnose_status(final_status)
      if diagnosis["healthy"]:
        if not stable_since:
          stable_since = time.monotonic()
        elif time.monotonic() - stable_since >= VERIFY_STABLE_SECONDS:
          result = 0
          break
      else:
        stable_since = 0.0
      time.sleep(1.0)

    print_status(final_status)
    if result == 0:
      print(f"[youtube-test] verification passed ({VERIFY_STABLE_SECONDS}s stable)")
    else:
      print(f"[youtube-test] verification failed; attach {REPORT_PATH}", file=sys.stderr)
  except KeyboardInterrupt:
    result = 130
    print("[youtube-test] verification interrupted", file=sys.stderr)
  finally:
    if started:
      stop_test()
  return result


def print_logs(lines: int) -> int:
  print(f"[youtube-test] log={LOG_PATH}")
  for line in _tail_log(lines):
    print(line)
  return 0


def _wait_for_vipc(timeout: float) -> list[int]:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    streams = _vipc_streams()
    if streams:
      return streams
    time.sleep(0.25)
  return []


def _wait_for_youtube_keyframe(sock: Any, timeout: float) -> dict[str, int]:
  from openpilot.cereal import messaging

  deadline = time.monotonic() + timeout
  frames = 0
  while time.monotonic() < deadline:
    msg = messaging.recv_one_or_none(sock)
    if msg is None:
      time.sleep(0.02)
      continue
    frame = getattr(msg, msg.which(), None)
    if frame is None:
      continue
    data = bytes(getattr(frame, "data", b"") or b"")
    header = bytes(getattr(frame, "header", b"") or b"")
    if not data:
      continue
    frames += 1
    try:
      validate_stream_start(header, data)
    except ValueError:
      continue
    else:
      return {
        "frames": frames,
        "header_bytes": len(header),
        "frame_bytes": len(data),
        "width": int(getattr(frame, "width", 0) or 0),
        "height": int(getattr(frame, "height", 0) or 0),
      }
  return {}


def _run_test(quality: int) -> int:
  from openpilot.cereal import messaging

  stopped = False
  children: dict[str, subprocess.Popen] = {}
  encoder_spec = _quality_spec(quality)
  params = _params()
  previous_live = int(params.get_int("CarrotYouTubeLive"))
  state: dict[str, Any] = {
    "status": "starting",
    "runner_pid": os.getpid(),
    "started_mono": time.monotonic(),
    "quality": quality,
    "quality_label": encoder_spec["label"],
    "previous_live": previous_live,
    "forced_live": previous_live <= 0,
    "children": {},
    "source_sample": {},
    "error": "",
  }

  def request_stop(_signum=None, _frame=None) -> None:
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGTERM, request_stop)
  signal.signal(signal.SIGINT, request_stop)

  def log(message: str) -> None:
    print(f"[youtube-test] {message}", flush=True)

  def start_child(name: str, spec: dict[str, Any]) -> None:
    proc = subprocess.Popen(
      list(spec["cmd"]),
      cwd=str(spec["cwd"]),
      stdin=subprocess.DEVNULL,
      stdout=sys.stdout,
      stderr=subprocess.STDOUT,
    )
    children[name] = proc
    state["children"][name] = proc.pid
    _write_state(state)
    log(f"{name} started pid={proc.pid}")

  _write_state(state)
  try:
    if not params.get_bool("IsOffroad"):
      raise RuntimeError("device is not offroad")
    if int(params.get_int("CarrotYouTubeTimestamp")) > 0:
      raise RuntimeError("CarrotYouTubeTimestamp must be disabled")

    _set_snapshot_active(True)
    log("IsTakingSnapshot enabled")
    time.sleep(2.0)

    source_sock = messaging.sub_sock(YOUTUBE_SOURCE, conflate=False)
    start_child("camerad", _CAMERAD_SPEC)
    streams = _wait_for_vipc(8.0)
    if not streams:
      raise RuntimeError("camerad did not publish VisionIPC streams")
    log(f"VIPC streams ready: {','.join(map(str, streams))}")

    encoder_name = str(encoder_spec["name"])
    start_child(encoder_name, encoder_spec)
    sample = _wait_for_youtube_keyframe(source_sock, 12.0)
    if not sample:
      raise RuntimeError(f"{encoder_name} did not publish an H.264 keyframe")
    state["source_sample"] = sample
    dimensions = f"{sample['width']}x{sample['height']}"
    log(f"H.264 keyframe ready: {dimensions} header={sample['header_bytes']} frame={sample['frame_bytes']}")

    if previous_live <= 0:
      params.put_int("CarrotYouTubeLive", 1)
      log("CarrotYouTubeLive enabled for this test")

    state["status"] = "running"
    _write_state(state)
    log(f"running timeout={TIMEOUT_SECONDS}s quality={encoder_spec['label']}")

    deadline = time.monotonic() + TIMEOUT_SECONDS
    last_youtube_state = ""
    while not stopped and time.monotonic() < deadline:
      if not params.get_bool("IsOffroad"):
        log("offroad ended; stopping")
        break
      for name, proc in children.items():
        if proc.poll() is not None:
          raise RuntimeError(f"{name} exited code={proc.returncode}")
      youtube = _fetch_youtube_status()
      state["youtube"] = youtube
      _write_state(state)
      youtube_state = str(youtube.get("state") or "")
      if youtube_state and youtube_state != last_youtube_state:
        log(f"YouTube state: {youtube_state}")
        last_youtube_state = youtube_state
      time.sleep(0.5)
    if time.monotonic() >= deadline:
      log("timeout reached; stopping")
  except Exception as exc:
    state["status"] = "error"
    state["error"] = str(exc)
    _write_state(state)
    log(f"error: {exc}")
    return 1
  finally:
    for name in reversed(tuple(children)):
      spec = _CAMERAD_SPEC if name == "camerad" else encoder_spec
      _terminate_pid(children[name].pid, str(spec["match"]))
      log(f"{name} stopped")
    _restore_live_param(state)
    if state.get("forced_live"):
      log("CarrotYouTubeLive restored")
    _set_snapshot_active(False)
    state["children"] = {}
    if state["status"] != "error":
      state["status"] = "stopped"
    _write_state(state)
    log("IsTakingSnapshot cleared")

  return 0


def run_command(args: list[str]) -> int:
  if args and args[0].lower() in {"help", "-h", "--help"}:
    print("Usage: carrot youtube-test [verify|start|status|logs|stop] [--lines N]")
    print("  verify  Run one complete test, save a compact report, then stop")
    print("  start   Start an offroad YouTube camera test")
    print("  status  Show encoder and upload diagnostics")
    print("  logs    Show recent test process logs")
    print("  stop    Stop the test and restore its live toggle")
    return 0

  parser = argparse.ArgumentParser(prog="carrot youtube-test", add_help=False)
  parser.add_argument("action", nargs="?", default="start", choices=("verify", "start", "status", "logs", "stop"))
  parser.add_argument("--lines", type=int, default=80)
  try:
    options = parser.parse_args(args)
  except SystemExit:
    return 2

  if options.action == "verify":
    return verify_test()
  if options.action == "start":
    return start_test()
  if options.action == "status":
    return print_status()
  if options.action == "logs":
    return print_logs(max(1, min(500, options.lines)))
  return stop_test()


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("action", choices=("_run",))
  parser.add_argument("--quality", type=int, choices=tuple(_QUALITY_SPECS), required=True)
  options = parser.parse_args(argv)
  if options.action == "_run":
    return _run_test(options.quality)
  return 2


if __name__ == "__main__":
  raise SystemExit(main())
