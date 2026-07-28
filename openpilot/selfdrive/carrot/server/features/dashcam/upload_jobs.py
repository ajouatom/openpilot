from __future__ import annotations

import asyncio
import os
import time
import uuid
from datetime import datetime
from typing import Any

from openpilot.selfdrive.carrot.web_upload import (
  create_web_upload_session,
  send_web_upload_complete,
  upload_device_id,
  upload_folder_to_web,
)

from ...services.params import HAS_PARAMS, Params
from . import upload
from .catalog import segment_file_summary
from .paths import route_name, segment_dir, segment_index


UPLOAD_JOB_KEEP_COUNT = 12
UPLOAD_JOB_MAX_LOG_CHARS = 60000
UPLOAD_JOB_STALE_SECONDS = 30 * 60
_jobs: dict[str, dict[str, Any]] = {}


class UploadCanceled(Exception):
  pass


def jobs() -> dict[str, dict[str, Any]]:
  return _jobs


def has_running_job() -> bool:
  expire_stale_jobs()
  return any(job.get("status") == "running" for job in _jobs.values())


def running_job() -> dict[str, Any] | None:
  expire_stale_jobs()
  for job in _jobs.values():
    if job.get("status") == "running":
      return job
  return None


def touch(job: dict[str, Any]) -> None:
  job["updated_at"] = time.time()  # noqa: TID251
  job["_activity_at"] = time.monotonic()


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


def is_cancel_requested(job: dict[str, Any] | None) -> bool:
  return bool(job and job.get("cancel_requested"))


def ensure_not_canceled(job: dict[str, Any] | None) -> None:
  if is_cancel_requested(job):
    raise UploadCanceled("upload canceled")


def cancel_job(job_id: str) -> dict[str, Any]:
  job = _jobs.get(job_id)
  if not job:
    return {"ok": False, "error": "job not found"}
  if job.get("status") in ("done", "failed", "canceled"):
    return {"ok": True, "already_done": True, **snapshot(job)}
  job["cancel_requested"] = True
  progress(job, message="Canceling upload")
  append(job, "Cancel requested")
  return {"ok": True, **snapshot(job)}


def snapshot(job: dict[str, Any]) -> dict[str, Any]:
  return {
    "ok": True,
    "id": job["id"],
    "action": job["action"],
    "status": job["status"],
    "done": job["status"] in ("done", "failed", "canceled"),
    "cancel_requested": bool(job.get("cancel_requested")),
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


def finish(
  job: dict[str, Any],
  *,
  ok: bool,
  result: dict[str, Any] | None = None,
  error: str | None = None,
  status: str | None = None,
) -> None:
  job["status"] = status or ("done" if ok else "failed")
  job["result"] = result or {"ok": bool(ok)}
  job["error"] = error or (None if ok else job["result"].get("error"))
  if ok:
    job["progress"] = 100
  touch(job)
  prune()


def fail_running_job(job: dict[str, Any], error: str) -> None:
  if job.get("status") != "running":
    return
  results = list(job.get("partial_results") or [])
  uploaded = sum(1 for item in results if item.get("ok"))
  total = len(job.get("segments") or [])
  result = {
    "ok": False,
    "error": error,
    "uploaded": uploaded,
    "total": total,
    "results": results,
    "message": f"Upload failed: {error}",
  }
  append(job, f"FAILED: {error}")
  finish(job, ok=False, result=result, error=error)


def expire_stale_jobs(now: float | None = None) -> None:
  current = time.monotonic() if now is None else float(now)
  for job in list(_jobs.values()):
    if job.get("status") != "running":
      continue
    task = job.get("_task")
    task_done = task is not None and task.done()
    activity_at = float(job.get("_activity_at") or current)
    inactive = current - activity_at >= UPLOAD_JOB_STALE_SECONDS
    if not task_done and not inactive:
      continue
    if task is not None and not task.done():
      task.cancel()
    stale_minutes = max(1, round(UPLOAD_JOB_STALE_SECONDS / 60))
    reason = (
      "upload task ended without a final state"
      if task_done
      else f"upload job expired after {stale_minutes} minutes without activity"
    )
    fail_running_job(job, reason)


def prune() -> None:
  finished = [job for job in _jobs.values() if job.get("status") in ("done", "failed", "canceled")]
  if len(finished) <= UPLOAD_JOB_KEEP_COUNT:
    return
  finished.sort(key=lambda job: float(job.get("updated_at") or 0), reverse=True)
  for old in finished[UPLOAD_JOB_KEEP_COUNT:]:
    _jobs.pop(old["id"], None)


def create_job(segments: list[str]) -> dict[str, Any]:
  job_id = uuid.uuid4().hex[:12]
  now = time.time()  # noqa: TID251
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
    "cancel_requested": False,
    "created_at": now,
    "updated_at": now,
    "_activity_at": time.monotonic(),
  }
  _jobs[job_id] = job
  prune()
  return job


def start_job(job: dict[str, Any]) -> asyncio.Task:
  task = asyncio.create_task(run_job(job))
  job["_task"] = task

  def finalize_task(done_task: asyncio.Task) -> None:
    if job.get("_task") is done_task:
      job.pop("_task", None)
    if job.get("status") != "running":
      return
    if done_task.cancelled():
      error = "upload task canceled before completion"
    else:
      exception = done_task.exception()
      error = str(exception) if exception else "upload task ended without a final state"
    fail_running_job(job, error)

  task.add_done_callback(finalize_task)
  return task


async def run_upload_segments(segments: list[str], job: dict[str, Any] | None = None) -> dict[str, Any]:
  params = Params() if HAS_PARAMS else None
  target = upload.resolve_upload_target()
  base_url, token = target["base_url"], target["token"]
  meta = upload.upload_metadata(params)
  device_id = upload_device_id(meta)
  car_selected = meta.get("carName") or "none"
  storage_directory = f"{car_selected} {device_id}".strip()
  upload_directory = device_id if target["kind"] == "carrot" else storage_directory
  if not token:
    if target["kind"] == "carrot":
      token = await create_web_upload_session(base_url, meta, "dashcam")
    else:
      raise RuntimeError("Toss upload token is not configured")
  remote_base_path = f"{base_url}/routes/{storage_directory}/".replace("\\", "/")
  total = len(segments)
  results: list[Any] = [None] * total  # filled by index so order matches input

  if job:
    job["upload_meta"] = meta
    job["remote_base_path"] = remote_base_path
    job["upload_target"] = target["kind"]
    job["partial_results"] = []
    progress(job, message="Preparing upload", current=0, total=total, percent=0)

  ensure_not_canceled(job)

  # Upload segments in parallel with bounded concurrency. Each segment uses an
  # independent HTTPS session. Keep the limit small because the upload service
  # and its backing storage are shared across devices.
  try:
    concurrency = max(1, min(6, int(os.environ.get("CARROT_WEB_UPLOAD_CONCURRENCY", "3") or "3")))
  except Exception:
    concurrency = 3
  sem = asyncio.Semaphore(concurrency)
  completed = 0

  async def upload_one(idx0: int, segment: str) -> None:
    nonlocal completed
    idx = idx0 + 1
    files: list[Any] = []
    async with sem:
      if is_cancel_requested(job):
        return
      if job:
        append(job, f"[{idx}/{total}] {segment}")
      try:
        segment_path = segment_dir(segment)
        files = await asyncio.to_thread(segment_file_summary, segment_path)
        def should_cancel() -> bool:
          if job:
            touch(job)
          return is_cancel_requested(job)

        ok = await upload_folder_to_web(
          segment_path,
          upload_directory,
          segment,
          base_url,
          token,
          should_cancel if job else None,
        )
        results[idx0] = {
          "segment": segment,
          "route": route_name(segment),
          "segmentIndex": segment_index(segment),
          "ok": bool(ok),
          "remotePath": f"{remote_base_path}{segment}",
          "files": files,
        }
        if job:
          append(job, f"[{idx}/{total}] {segment} OK")
      except Exception as e:
        if is_cancel_requested(job):
          return  # canceled mid-upload — handled after gather
        results[idx0] = {
          "segment": segment,
          "route": route_name(segment),
          "segmentIndex": segment_index(segment),
          "ok": False,
          "remotePath": f"{remote_base_path}{segment}",
          "files": files,
          "error": str(e),
        }
        if job:
          append(job, f"[{idx}/{total}] {segment} FAILED: {e}")
    # post-upload bookkeeping runs synchronously (atomic between awaits)
    completed += 1
    if job:
      job["partial_results"] = [r for r in results if r is not None]
      progress(job, message=f"Uploaded {completed}/{total}", current=completed, total=total)

  await asyncio.gather(*(upload_one(i, seg) for i, seg in enumerate(segments)))

  ensure_not_canceled(job)
  results = [r for r in results if r is not None]
  ok_count = sum(1 for item in results if item["ok"])
  uploaded_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
  response_payload = {
    "ok": ok_count == len(results),
    "uploaded": ok_count,
    "total": len(results),
    "uploadedAt": uploaded_at,
    "target": target["kind"],
    "deviceId": device_id,
    "remoteBasePath": remote_base_path,
    "meta": meta,
    "results": results,
    "message": f"{ok_count}/{len(results)} uploaded",
  }
  response_payload["shareText"] = upload.upload_share_text(response_payload)

  if job:
    progress(job, message="Sending notification", current=total, total=total, percent=98)
  ensure_not_canceled(job)
  response_payload["webComplete"] = await send_web_upload_complete(
    base_url,
    token,
    response_payload,
  )
  if target["kind"] == "carrot":
    response_payload["discord"] = await upload.send_discord_webhook(
      upload.discord_webhook_url(params),
      response_payload,
    )
  else:
    response_payload["discord"] = {"configured": False, "ok": False, "skipped": True}
  return response_payload


async def run_job(job: dict[str, Any]) -> None:
  try:
    result = await run_upload_segments(list(job.get("segments") or []), job)
    finish(job, ok=bool(result.get("ok")), result=result)
  except UploadCanceled as exc:
    results = list(job.get("partial_results") or [])
    uploaded_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ok_count = sum(1 for item in results if item.get("ok"))
    total = len(job.get("segments") or [])
    result = {
      "ok": False,
      "canceled": True,
      "uploaded": ok_count,
      "total": total,
      "uploadedAt": uploaded_at,
      "target": job.get("upload_target") or "carrot",
      "remoteBasePath": job.get("remote_base_path") or "",
      "meta": job.get("upload_meta") or {},
      "results": results,
      "message": f"Canceled {ok_count}/{total}",
      "error": str(exc),
    }
    result["shareText"] = upload.upload_share_text(result)
    append(job, "CANCELED")
    progress(job, message="Upload canceled", percent=0)
    finish(job, ok=False, result=result, error=str(exc), status="canceled")
  except Exception as exc:
    result = {"ok": False, "error": str(exc)}
    append(job, f"FAILED: {exc}")
    finish(job, ok=False, result=result, error=str(exc))
