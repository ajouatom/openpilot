"""
Tool job infrastructure.

- Holds the in-process job dict (`_jobs`) used by /api/tools/start + /api/tools/job.
- Provides the streaming/capture exec helpers used by the dispatcher.
- Branch listing utilities (used by both job and sync flows for git_branch_list).
"""
from __future__ import annotations

import asyncio
import json
import os
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

from ...config import CARROT_STATE_DIR, CARROT_TOOL_JOBS_STATE_PATH

TOOL_JOB_MAX_LOG_CHARS = 60000
TOOL_JOB_KEEP_COUNT = 20
TOOL_JOB_MAX_AGE_SECONDS = 7 * 24 * 60 * 60
_jobs: Dict[str, Dict[str, Any]] = {}
_loaded = False
_last_persist_at = 0.0


def jobs() -> Dict[str, Dict[str, Any]]:
  load_persisted()
  return _jobs


def touch(job: Dict[str, Any]) -> None:
  job["updated_at"] = time.time()


def trim_log(job: Dict[str, Any]) -> None:
  text = job.get("log") or ""
  if len(text) <= TOOL_JOB_MAX_LOG_CHARS:
    return
  job["log"] = text[-TOOL_JOB_MAX_LOG_CHARS:]


def append(job: Dict[str, Any], text: Any) -> None:
  if text is None:
    return
  chunk = str(text).replace("\r\n", "\n").replace("\r", "\n")
  if not chunk:
    return
  cur = job.get("log") or ""
  if cur and not cur.endswith("\n") and not chunk.startswith("\n"):
    cur += "\n"
  job["log"] = cur + chunk
  trim_log(job)
  touch(job)
  persist_changed()


def progress(job: Dict[str, Any], *, message: Optional[str] = None,
             current: Optional[int] = None, total: Optional[int] = None,
             percent: Optional[int] = None) -> None:
  if message is not None:
    job["message"] = str(message)
  if current is not None:
    job["step_current"] = max(0, int(current))
  if total is not None:
    job["step_total"] = max(0, int(total))
  if percent is None:
    c = job.get("step_current")
    t = job.get("step_total")
    if isinstance(c, int) and isinstance(t, int) and t > 0:
      percent = int(max(0, min(100, round((c / t) * 100))))
  job["progress"] = percent
  touch(job)
  persist_changed()


def snapshot(job: Dict[str, Any]) -> Dict[str, Any]:
  result = job.get("result")
  return {
    "ok": True,
    "id": job["id"],
    "action": job["action"],
    "payload": job.get("payload") if isinstance(job.get("payload"), dict) else {},
    "status": job["status"],
    "done": job["status"] in ("done", "failed"),
    "log": job.get("log") or "",
    "progress": job.get("progress"),
    "message": job.get("message") or "",
    "step_current": job.get("step_current"),
    "step_total": job.get("step_total"),
    "error": job.get("error"),
    "error_code": job.get("error_code"),
    "error_detail": job.get("error_detail"),
    "created_at": job.get("created_at"),
    "updated_at": job.get("updated_at"),
    "result": result,
  }


def finish(job: Dict[str, Any], *, ok: bool, result: Optional[Dict[str, Any]] = None,
           error: Optional[str] = None, error_code: Optional[str] = None,
           error_detail: Optional[str] = None) -> None:
  job["status"] = "done" if ok else "failed"
  job["result"] = result or {"ok": bool(ok)}
  if error is None and result:
    error = result.get("error") or (None if result.get("ok", ok) else result.get("out"))
  job["error"] = error
  job["error_code"] = error_code or (result.get("error_code") if result else None)
  job["error_detail"] = error_detail or (result.get("error_detail") if result else None)
  if ok:
    job["progress"] = 100
  touch(job)
  prune()
  persist_now()


def prune() -> bool:
  removed = False
  now = time.time()
  finished = [
    item for item in _jobs.values()
    if item.get("status") in ("done", "failed")
  ]
  for old in finished:
    updated_at = _safe_float(old.get("updated_at") or old.get("created_at"))
    if updated_at > 0 and now - updated_at > TOOL_JOB_MAX_AGE_SECONDS:
      _jobs.pop(old["id"], None)
      removed = True
  finished = [
    item for item in _jobs.values()
    if item.get("status") in ("done", "failed")
  ]
  finished.sort(key=lambda item: _safe_float(item.get("updated_at")), reverse=True)
  for old in finished[TOOL_JOB_KEEP_COUNT:]:
    _jobs.pop(old["id"], None)
    removed = True
  return removed


def _safe_float(value: Any) -> float:
  try:
    return float(value)
  except Exception:
    return 0.0


def _sanitize_loaded_job(raw: Any) -> Optional[Dict[str, Any]]:
  if not isinstance(raw, dict):
    return None
  job_id = str(raw.get("id") or "").strip()
  action = str(raw.get("action") or "").strip()
  if not job_id or not action:
    return None

  status = str(raw.get("status") or "done").strip()
  if status == "running":
    status = "failed"

  job = {
    "id": job_id,
    "action": action,
    "payload": raw.get("payload") if isinstance(raw.get("payload"), dict) else {},
    "status": status if status in ("done", "failed") else "done",
    "log": str(raw.get("log") or ""),
    "progress": raw.get("progress"),
    "message": str(raw.get("message") or ""),
    "step_current": raw.get("step_current"),
    "step_total": raw.get("step_total"),
    "error": raw.get("error"),
    "error_code": raw.get("error_code"),
    "error_detail": raw.get("error_detail"),
    "result": raw.get("result") if isinstance(raw.get("result"), dict) else None,
    "created_at": _safe_float(raw.get("created_at") or raw.get("updated_at") or time.time()),
    "updated_at": _safe_float(raw.get("updated_at") or raw.get("created_at") or time.time()),
  }
  if raw.get("status") == "running":
    job["error"] = job["error"] or "server restarted before job completed"
    job["message"] = job["message"] or "Interrupted by server restart"
    job["result"] = job["result"] or {"ok": False, "error": job["error"]}
  trim_log(job)
  return job


def load_persisted() -> None:
  global _loaded
  if _loaded:
    return
  _loaded = True
  try:
    with open(CARROT_TOOL_JOBS_STATE_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
    raw_jobs = data.get("jobs") if isinstance(data, dict) else data
    if not isinstance(raw_jobs, list):
      return
    for raw in raw_jobs:
      job = _sanitize_loaded_job(raw)
      if job is not None:
        _jobs[job["id"]] = job
    if prune():
      persist_now()
  except Exception:
    return


def list_snapshots(limit: Optional[int] = TOOL_JOB_KEEP_COUNT) -> List[Dict[str, Any]]:
  load_persisted()
  items = list(_jobs.values())
  items.sort(key=lambda item: (
    _safe_float(item.get("updated_at")),
    _safe_float(item.get("created_at")),
  ), reverse=True)
  if limit is not None and limit > 0:
    items = items[:limit]
  return [snapshot(item) for item in items]


def clear_finished() -> int:
  load_persisted()
  remove_ids = [
    job_id for job_id, job in _jobs.items()
    if job.get("status") in ("done", "failed")
  ]
  for job_id in remove_ids:
    _jobs.pop(job_id, None)
  if remove_ids:
    persist_now()
  return len(remove_ids)


def add_notice(action: str, message: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  load_persisted()
  now = time.time()
  text = str(message or "").strip()
  job = {
    "id": uuid.uuid4().hex[:12],
    "action": str(action or "notice").strip() or "notice",
    "payload": payload if isinstance(payload, dict) else {"notice": True},
    "status": "done",
    "log": text,
    "progress": 100,
    "message": "",
    "step_current": 1,
    "step_total": 1,
    "error": None,
    "error_code": None,
    "error_detail": None,
    "result": {"ok": True, "out": text},
    "created_at": now,
    "updated_at": now,
  }
  _jobs[job["id"]] = job
  prune()
  persist_now()
  return snapshot(job)


def persist_now() -> None:
  global _last_persist_at
  try:
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    payload = {
      "version": 1,
      "updated_at": time.time(),
      "jobs": list_snapshots(TOOL_JOB_KEEP_COUNT),
    }
    tmp_path = f"{CARROT_TOOL_JOBS_STATE_PATH}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=True, separators=(",", ":"))
    os.replace(tmp_path, CARROT_TOOL_JOBS_STATE_PATH)
    _last_persist_at = time.time()
  except Exception:
    pass


def persist_changed(min_interval: float = 0.5) -> None:
  if time.time() - _last_persist_at >= min_interval:
    persist_now()


async def stream_exec(job: Dict[str, Any], cmd: List[str], *, cwd: Optional[str] = None,
                      timeout: Optional[float] = None) -> int:
  proc = await asyncio.create_subprocess_exec(
    *cmd,
    cwd=cwd,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.STDOUT,
  )

  async def _consume() -> int:
    assert proc.stdout is not None
    while True:
      chunk = await proc.stdout.read(1024)
      if not chunk:
        break
      append(job, chunk.decode("utf-8", errors="replace"))
    return await proc.wait()

  try:
    if timeout is not None:
      return await asyncio.wait_for(_consume(), timeout=timeout)
    return await _consume()
  except asyncio.TimeoutError:
    try:
      proc.kill()
    except Exception:
      pass
    try:
      await proc.wait()
    except Exception:
      pass
    append(job, "\n[timeout]\n")
    raise


async def capture_exec(cmd: List[str], *, cwd: Optional[str] = None,
                       timeout: Optional[float] = None) -> Tuple[int, str]:
  proc = await asyncio.create_subprocess_exec(
    *cmd,
    cwd=cwd,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.STDOUT,
  )
  try:
    stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout) if timeout is not None else await proc.communicate()
  except asyncio.TimeoutError:
    try:
      proc.kill()
    except Exception:
      pass
    try:
      await proc.wait()
    except Exception:
      pass
    raise
  return proc.returncode, (stdout or b"").decode("utf-8", errors="replace").strip()


def result_from_log(job: Dict[str, Any], rc: int, **extra: Any) -> Dict[str, Any]:
  out = (job.get("log") or "").strip() or "(no output)"
  return {"ok": rc == 0, "rc": rc, "out": out, **extra}


# -----------------------
# Branch listing utilities (shared by job + sync git_branch_list)
# -----------------------
def get_branch_prefix() -> str:
  try:
    from openpilot.system.hardware import HARDWARE
    return "c4" if HARDWARE.get_device_type() == "mici" else "c3"
  except Exception:
    return "c3"


def parse_remote_urls(remote_urls_out: str) -> dict[str, str]:
  remote_urls: dict[str, str] = {}
  for remote_line in (remote_urls_out or "").splitlines():
    parts = remote_line.split()
    if len(parts) >= 2 and parts[0] not in remote_urls:
      remote_urls[parts[0]] = parts[1]
  return remote_urls


def match_remote_ref(ref: str, remotes: list[str]) -> Optional[tuple[str, str]]:
  for remote in sorted(remotes, key=len, reverse=True):
    prefix = f"{remote}/"
    if not ref.startswith(prefix):
      continue
    name = ref[len(prefix):].strip()
    if name and name != "HEAD":
      return remote, name
  return None


def build_branch_items(local_refs_out: str, remote_refs_out: str, remotes: list[str]) -> list[dict[str, Any]]:
  items: list[dict[str, Any]] = []
  seen: set[tuple[str, str, str]] = set()

  for line in (local_refs_out or "").splitlines():
    name = line.strip()
    if not name:
      continue
    key = ("local", "", name)
    if key in seen:
      continue
    seen.add(key)
    items.append({
      "kind": "local",
      "ref": name,
      "name": name,
      "label": name,
    })

  for line in (remote_refs_out or "").splitlines():
    ref = line.strip()
    if not ref:
      continue
    match = match_remote_ref(ref, remotes)
    if match is None:
      continue
    remote, name = match
    key = ("remote", remote, name)
    if key in seen:
      continue
    seen.add(key)
    items.append({
      "kind": "remote",
      "ref": f"{remote}/{name}",
      "remote": remote,
      "name": name,
      "label": name,
    })

  return sorted(items, key=lambda item: (
    0 if item.get("kind") == "local" else 1,
    str(item.get("remote") or "").lower(),
    str(item.get("name") or item.get("ref") or "").lower(),
  ))
