from __future__ import annotations

import asyncio
import time
import uuid
from datetime import datetime
from typing import Any

from ...services.params import HAS_PARAMS, Params
from . import upload
from .catalog import segment_file_summary
from .paths import route_name, segment_dir, segment_index


UPLOAD_JOB_KEEP_COUNT = 12
UPLOAD_JOB_MAX_LOG_CHARS = 60000
_jobs: dict[str, dict[str, Any]] = {}


def jobs() -> dict[str, dict[str, Any]]:
  return _jobs


def has_running_job() -> bool:
  return any(job.get("status") == "running" for job in _jobs.values())


def touch(job: dict[str, Any]) -> None:
  job["updated_at"] = time.time()


def append(job: dict[str, Any], text: Any) -> None:
  if text is None:
    return
  chunk = str(text).replace("\r\n", "\n").replace("\r", "\n")
  if not chunk:
    return
  cur = job.get("log") or ""
  if cur and not cur.endswith("\n") and not chunk.startswith("\n"):
    cur += "\n"
  job["log"] = (cur + chunk)[-UPLOAD_JOB_MAX_LOG_CHARS:]
  touch(job)


def progress(
  job: dict[str, Any],
  *,
  message: str | None = None,
  current: int | None = None,
  total: int | None = None,
  percent: int | None = None,
) -> None:
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


def snapshot(job: dict[str, Any]) -> dict[str, Any]:
  return {
    "ok": True,
    "id": job["id"],
    "action": job["action"],
    "status": job["status"],
    "done": job["status"] in ("done", "failed"),
    "log": job.get("log") or "",
    "progress": job.get("progress"),
    "message": job.get("message") or "",
    "step_current": job.get("step_current"),
    "step_total": job.get("step_total"),
    "error": job.get("error"),
    "created_at": job.get("created_at"),
    "updated_at": job.get("updated_at"),
    "result": job.get("result"),
  }


def finish(job: dict[str, Any], *, ok: bool, result: dict[str, Any] | None = None, error: str | None = None) -> None:
  job["status"] = "done" if ok else "failed"
  job["result"] = result or {"ok": bool(ok)}
  job["error"] = error or (None if ok else job["result"].get("error"))
  if ok:
    job["progress"] = 100
  touch(job)
  prune()


def prune() -> None:
  finished = [job for job in _jobs.values() if job.get("status") in ("done", "failed")]
  if len(finished) <= UPLOAD_JOB_KEEP_COUNT:
    return
  finished.sort(key=lambda job: float(job.get("updated_at") or 0), reverse=True)
  for old in finished[UPLOAD_JOB_KEEP_COUNT:]:
    _jobs.pop(old["id"], None)


def create_job(segments: list[str]) -> dict[str, Any]:
  job_id = uuid.uuid4().hex[:12]
  now = time.time()
  job = {
    "id": job_id,
    "action": "dashcam_upload",
    "segments": list(segments),
    "status": "running",
    "log": "",
    "progress": 0,
    "message": "",
    "step_current": 0,
    "step_total": len(segments),
    "error": None,
    "result": None,
    "created_at": now,
    "updated_at": now,
  }
  _jobs[job_id] = job
  prune()
  return job


async def run_upload_segments(segments: list[str], job: dict[str, Any] | None = None) -> dict[str, Any]:
  params = Params() if HAS_PARAMS else None
  meta = upload.upload_metadata(params)
  car_selected = meta.get("carName") or "none"
  dongle_id = meta.get("dongleId") or "unknown"
  directory = f"{car_selected} {dongle_id}".strip()
  remote_base_path = f"routes/{directory}/".replace("\\", "/")
  total = len(segments)
  results = []

  if job:
    progress(job, message="Preparing upload", current=0, total=total, percent=0)

  for idx, segment in enumerate(segments, start=1):
    files = []
    if job:
      progress(job, message=f"Uploading {idx}/{total}", current=idx - 1, total=total)
      append(job, f"[{idx}/{total}] {segment}")
    try:
      segment_path = segment_dir(segment)
      files = await asyncio.to_thread(segment_file_summary, segment_path)
      ok = await asyncio.to_thread(
        upload.upload_folder_to_ftp,
        segment_path,
        directory,
        segment,
      )
      results.append({
        "segment": segment,
        "route": route_name(segment),
        "segmentIndex": segment_index(segment),
        "ok": bool(ok),
        "remotePath": f"{remote_base_path}{segment}",
        "files": files,
      })
      if job:
        append(job, f"[{idx}/{total}] {segment} OK")
    except Exception as e:
      results.append({
        "segment": segment,
        "route": route_name(segment),
        "segmentIndex": segment_index(segment),
        "ok": False,
        "remotePath": f"{remote_base_path}{segment}",
        "files": files,
        "error": str(e),
      })
      if job:
        append(job, f"[{idx}/{total}] {segment} FAILED: {e}")

    if job:
      progress(job, message=f"Uploaded {idx}/{total}", current=idx, total=total)

  ok_count = sum(1 for item in results if item["ok"])
  uploaded_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
  response_payload = {
    "ok": ok_count == len(results),
    "uploaded": ok_count,
    "total": len(results),
    "uploadedAt": uploaded_at,
    "remoteBasePath": remote_base_path,
    "meta": meta,
    "results": results,
    "message": f"{ok_count}/{len(results)} uploaded",
  }
  response_payload["shareText"] = upload.upload_share_text(response_payload)

  if job:
    progress(job, message="Sending notification", current=total, total=total, percent=98)
  response_payload["discord"] = await upload.send_discord_webhook(
    upload.discord_webhook_url(params),
    response_payload,
  )
  return response_payload


async def run_job(job: dict[str, Any]) -> None:
  try:
    result = await run_upload_segments(list(job.get("segments") or []), job)
    finish(job, ok=bool(result.get("ok")), result=result)
  except Exception as exc:
    result = {"ok": False, "error": str(exc)}
    append(job, f"FAILED: {exc}")
    finish(job, ok=False, result=result, error=str(exc))
