from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path("/data/openpilot")
STATE_PATH = Path("/tmp/carrot-vision-test-state.json")
LOG_PATH = Path("/tmp/carrot-vision-test.log")
TIMEOUT_SECONDS = 10 * 60
RUNNER_MODULE = "selfdrive.carrot.server.services.vision_test"

_PROCESS_SPECS = {
  "camerad": {
    "cmd": [str(REPO_ROOT / "system/camerad/camerad")],
    "cwd": str(REPO_ROOT),
    "match": "system/camerad/camerad",
  },
  "stream_encoderd": {
    "cmd": [str(REPO_ROOT / "system/loggerd/encoderd"), "--stream"],
    "cwd": str(REPO_ROOT),
    "match": "system/loggerd/encoderd\x00--stream",
  },
  "webrtcd": {
    "cmd": [sys.executable, "-m", "system.webrtc.webrtcd"],
    "cwd": str(REPO_ROOT),
    "match": "system.webrtc.webrtcd",
  },
}


def _params():
  from openpilot.common.params import Params
  return Params()


def _set_snapshot_active(active: bool) -> None:
  params = _params()
  params.put_bool("IsTakingSnapshot", active)
  try:
    from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
    set_offroad_alert("Offroad_IsTakingSnapshot", active)
  except Exception:
    pass


def _write_state(state: dict[str, Any]) -> None:
  state["updated_at"] = time.time()
  temp_path = STATE_PATH.with_suffix(".tmp")
  temp_path.write_text(json.dumps(state, sort_keys=True), encoding="utf-8")
  temp_path.replace(STATE_PATH)


def _read_state() -> dict[str, Any]:
  try:
    raw = json.loads(STATE_PATH.read_text(encoding="utf-8"))
    return raw if isinstance(raw, dict) else {}
  except Exception:
    return {}


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


def _port_open(port: int) -> bool:
  try:
    with socket.create_connection(("127.0.0.1", port), timeout=0.2):
      return True
  except OSError:
    return False


def _children_status(state: dict[str, Any]) -> dict[str, dict[str, Any]]:
  children = state.get("children") if isinstance(state.get("children"), dict) else {}
  result = {}
  for name, spec in _PROCESS_SPECS.items():
    pid = int(children.get(name) or 0)
    result[name] = {"pid": pid, "alive": _pid_alive(pid, str(spec["match"]))}
  return result


def get_status() -> dict[str, Any]:
  state = _read_state()
  runner_pid = int(state.get("runner_pid") or 0)
  status = str(state.get("status") or "stopped")
  runner_alive = _pid_alive(runner_pid, RUNNER_MODULE)
  return {
    **state,
    "status": status if runner_alive or status == "error" else "stopped",
    "runner_pid": runner_pid,
    "runner_alive": runner_alive,
    "children": _children_status(state),
    "vipc_streams": _vipc_streams(),
    "webrtcd_port_open": _port_open(5001),
    "log_path": str(LOG_PATH),
  }


def _format_duration(seconds: float) -> str:
  seconds = max(0, int(seconds))
  return f"{seconds // 3600:02d}:{(seconds % 3600) // 60:02d}:{seconds % 60:02d}"


def print_status() -> int:
  status = get_status()
  started_at = float(status.get("started_at") or 0)
  elapsed = _format_duration(time.time() - started_at) if started_at else "00:00:00"
  print(f"[vision_test] {status['status']} elapsed={elapsed}")
  print(f"  runner           pid={status['runner_pid'] or '-'} alive={str(status['runner_alive']).lower()}")
  for name, child in status["children"].items():
    print(f"  {name:<16} pid={child['pid'] or '-'} alive={str(child['alive']).lower()}")
  streams = ", ".join(str(stream) for stream in status["vipc_streams"]) or "-"
  print(f"  VIPC streams     {streams}")
  print(f"  webrtcd port     5001 open={str(status['webrtcd_port_open']).lower()}")
  print(f"  log              {status['log_path']}")
  error = str(status.get("error") or "").strip()
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


def _cleanup_owned_processes(state: dict[str, Any]) -> None:
  children = state.get("children") if isinstance(state.get("children"), dict) else {}
  for name in reversed(tuple(_PROCESS_SPECS)):
    spec = _PROCESS_SPECS[name]
    _terminate_pid(int(children.get(name) or 0), str(spec["match"]))


def start_test() -> int:
  status = get_status()
  if status["runner_alive"]:
    print("[vision_test] already running")
    return print_status()

  if not _params().get_bool("IsOffroad"):
    print("[vision_test] refused: device is not offroad", file=sys.stderr)
    return 1

  for name, spec in _PROCESS_SPECS.items():
    pids = _find_matching_pids(str(spec["match"]))
    if pids:
      print(f"[vision_test] refused: {name} already running pid={','.join(map(str, pids))}", file=sys.stderr)
      return 1

  LOG_PATH.write_text("", encoding="utf-8")
  with LOG_PATH.open("a", encoding="utf-8") as log:
    proc = subprocess.Popen(
      [sys.executable, "-m", RUNNER_MODULE, "_run"],
      cwd=str(REPO_ROOT),
      stdin=subprocess.DEVNULL,
      stdout=log,
      stderr=subprocess.STDOUT,
      start_new_session=True,
    )

  print(f"[vision_test] starting runner pid={proc.pid}")
  deadline = time.monotonic() + 12.0
  while time.monotonic() < deadline:
    time.sleep(0.25)
    status = get_status()
    if status.get("status") == "running":
      print("[vision_test] ready")
      return print_status()
    if status.get("status") == "error":
      print(f"[vision_test] failed: {status.get('error') or 'unknown error'}", file=sys.stderr)
      return 1
    if proc.poll() is not None:
      print("[vision_test] runner exited during startup", file=sys.stderr)
      return 1

  print("[vision_test] startup is still in progress; run :vision_test status", file=sys.stderr)
  return 1


def stop_test() -> int:
  state = _read_state()
  runner_pid = int(state.get("runner_pid") or 0)
  if _pid_alive(runner_pid, RUNNER_MODULE):
    print(f"[vision_test] stopping runner pid={runner_pid}")
    _terminate_pid(runner_pid, RUNNER_MODULE, timeout=15.0)
  elif state:
    print("[vision_test] runner is not active; cleaning stale state")
    _cleanup_owned_processes(state)
    _set_snapshot_active(False)
  else:
    print("[vision_test] runner is not active")

  state = _read_state()
  if state:
    state.update({"status": "stopped", "children": {}, "error": ""})
    _write_state(state)
  print("[vision_test] stopped")
  return 0


def print_logs(lines: int) -> int:
  print(f"[vision_test] log={LOG_PATH}")
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


def _wait_for_port(port: int, timeout: float) -> bool:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if _port_open(port):
      return True
    time.sleep(0.25)
  return False


def _run_test() -> int:
  stopped = False
  children: dict[str, subprocess.Popen] = {}
  state: dict[str, Any] = {
    "status": "starting",
    "runner_pid": os.getpid(),
    "started_at": time.time(),
    "children": {},
    "error": "",
  }

  def request_stop(_signum=None, _frame=None) -> None:
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGTERM, request_stop)
  signal.signal(signal.SIGINT, request_stop)

  def log(message: str) -> None:
    print(f"[vision_test] {message}", flush=True)

  def start_child(name: str) -> None:
    spec = _PROCESS_SPECS[name]
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
    if not _params().get_bool("IsOffroad"):
      raise RuntimeError("device is not offroad")

    _set_snapshot_active(True)
    log("IsTakingSnapshot enabled")
    time.sleep(2.0)

    start_child("camerad")
    streams = _wait_for_vipc(8.0)
    if not streams:
      raise RuntimeError("camerad did not publish VisionIPC streams")
    log(f"VIPC streams ready: {','.join(map(str, streams))}")

    start_child("stream_encoderd")
    start_child("webrtcd")
    if not _wait_for_port(5001, 8.0):
      raise RuntimeError("webrtcd did not open port 5001")
    log("webrtcd port ready: 5001")

    state["status"] = "running"
    _write_state(state)
    log(f"running timeout={TIMEOUT_SECONDS}s")

    deadline = time.monotonic() + TIMEOUT_SECONDS
    while not stopped and time.monotonic() < deadline:
      if not _params().get_bool("IsOffroad"):
        log("offroad ended; stopping")
        break
      for name, proc in children.items():
        if proc.poll() is not None:
          raise RuntimeError(f"{name} exited code={proc.returncode}")
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
      spec = _PROCESS_SPECS[name]
      _terminate_pid(children[name].pid, str(spec["match"]))
      log(f"{name} stopped")
    _set_snapshot_active(False)
    state["children"] = {}
    if state["status"] != "error":
      state["status"] = "stopped"
    _write_state(state)
    log("IsTakingSnapshot cleared")

  return 0


def run_command(args: list[str]) -> int:
  parser = argparse.ArgumentParser(prog=":vision_test", add_help=False)
  parser.add_argument("action", nargs="?", default="start", choices=("start", "status", "logs", "stop"))
  parser.add_argument("--lines", type=int, default=80)
  try:
    options = parser.parse_args(args)
  except SystemExit:
    return 2

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
  options = parser.parse_args(argv)
  if options.action == "_run":
    return _run_test()
  return 2


if __name__ == "__main__":
  raise SystemExit(main())
