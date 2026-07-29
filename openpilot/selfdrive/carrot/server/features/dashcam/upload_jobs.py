from __future__ import annotations

import asyncio
import os
import time
import uuid
from collections import deque
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
UPLOAD_PHASE_QUEUED = "queued"
UPLOAD_PHASE_PREPARING = "preparing"
UPLOAD_PHASE_UPLOADING = "uploading"
UPLOAD_PHASE_NOTIFYING = "notifying"
UPLOAD_PHASE_CANCELING = "canceling"
UPLOAD_PHASE_COMPLETE = "complete"
UPLOAD_PHASE_CANCELED = "canceled"
UPLOAD_PHASE_FAILED = "failed"
UPLOAD_PREPARING_END_PERCENT = 8
UPLOAD_TRANSFERRING_END_PERCENT = 97
UPLOAD_NOTIFYING_START_PERCENT = 98
UPLOAD_NOTIFYING_END_PERCENT = 99
UPLOAD_PHASES = frozenset({
  UPLOAD_PHASE_QUEUED,
  UPLOAD_PHASE_PREPARING,
  UPLOAD_PHASE_UPLOADING,
  UPLOAD_PHASE_NOTIFYING,
  UPLOAD_PHASE_CANCELING,
  UPLOAD_PHASE_COMPLETE,
  UPLOAD_PHASE_CANCELED,
  UPLOAD_PHASE_FAILED,
})
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
  phase: str | None = None,
  phase_current: int | None = None,
  phase_total: int | None = None,
  bytes_current: int | None = None,
  bytes_total: int | None = None,
  bytes_per_second: int | None = None,
) -> None:
  changed = False
  if message is not None:
    normalized_message = str(message)
    changed = changed or job.get("message") != normalized_message
    job["message"] = normalized_message
  if phase is not None:
    normalized_phase = str(phase).strip().lower()
    if normalized_phase not in UPLOAD_PHASES:
      raise ValueError(f"unsupported upload phase: {phase}")
    changed = changed or job.get("phase") != normalized_phase
    job["phase"] = normalized_phase
  if current is not None:
    normalized_current = max(0, int(current))
    changed = changed or job.get("step_current") != normalized_current
    job["step_current"] = normalized_current
  if total is not None:
    normalized_total = max(0, int(total))
    changed = changed or job.get("step_total") != normalized_total
    job["step_total"] = normalized_total
  if phase_current is not None:
    normalized_phase_current = max(0, int(phase_current))
    changed = changed or job.get("phase_current") != normalized_phase_current
    job["phase_current"] = normalized_phase_current
  if phase_total is not None:
    normalized_phase_total = max(0, int(phase_total))
    changed = changed or job.get("phase_total") != normalized_phase_total
    job["phase_total"] = normalized_phase_total
  if bytes_current is not None:
    normalized_bytes_current = max(0, int(bytes_current))
    changed = changed or job.get("bytes_current") != normalized_bytes_current
    job["bytes_current"] = normalized_bytes_current
  if bytes_total is not None:
    normalized_bytes_total = max(0, int(bytes_total))
    changed = changed or job.get("bytes_total") != normalized_bytes_total
    job["bytes_total"] = normalized_bytes_total
  if bytes_per_second is not None:
    normalized_bytes_per_second = max(0, int(bytes_per_second))
    changed = changed or job.get("bytes_per_second") != normalized_bytes_per_second
    job["bytes_per_second"] = normalized_bytes_per_second
  if percent is None:
    percent = job.get("progress")
  if percent is not None:
    normalized_percent = int(max(0, min(100, round(float(percent)))))
    previous_percent = job.get("progress")
    if isinstance(previous_percent, (int, float)) and job.get("status") == "running":
      normalized_percent = max(int(previous_percent), normalized_percent)
    changed = changed or previous_percent != normalized_percent
    job["progress"] = normalized_percent
  if changed:
    job["revision"] = max(0, int(job.get("revision") or 0)) + 1
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
  progress(job, message="Canceling upload", phase=UPLOAD_PHASE_CANCELING)
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
    "revision": max(0, int(job.get("revision") or 0)),
    "phase": job.get("phase") or UPLOAD_PHASE_QUEUED,
    "message": job.get("message") or "",
    "step_current": job.get("step_current"),
    "step_total": job.get("step_total"),
    "phase_current": job.get("phase_current"),
    "phase_total": job.get("phase_total"),
    "bytes_current": job.get("bytes_current"),
    "bytes_total": job.get("bytes_total"),
    "bytes_per_second": job.get("bytes_per_second"),
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
  final_status = status or ("done" if ok else "failed")
  job["status"] = final_status
  job["phase"] = (
    UPLOAD_PHASE_CANCELED
    if final_status == "canceled"
    else UPLOAD_PHASE_COMPLETE
    if ok
    else UPLOAD_PHASE_FAILED
  )
  job["result"] = result or {"ok": bool(ok)}
  job["error"] = error or (None if ok else job["result"].get("error"))
  if ok:
    job["progress"] = 100
    job["phase_current"] = 1
    job["phase_total"] = 1
  job["revision"] = max(0, int(job.get("revision") or 0)) + 1
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
    "revision": 0,
    "phase": UPLOAD_PHASE_QUEUED,
    "message": "",
    "step_current": 0,
    "step_total": len(segments),
    "phase_current": 0,
    "phase_total": len(segments),
    "bytes_current": 0,
    "bytes_total": 0,
    "bytes_per_second": 0,
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
  base_url, token = upload.upload_target_settings()
  meta = upload.upload_metadata(params)
  device_id = upload_device_id(meta)
  car_selected = meta.get("carName") or "none"
  storage_directory = f"{car_selected} {device_id}".strip()
  if not token:
    token = await create_web_upload_session(base_url, meta, "dashcam")
  remote_base_path = f"{base_url}/routes/{storage_directory}/".replace("\\", "/")
  total = len(segments)
  results: list[Any] = [None] * total  # filled by index so order matches input

  if job:
    job["upload_meta"] = meta
    job["remote_base_path"] = remote_base_path
    job["upload_target"] = "web"
    job["partial_results"] = []
    progress(
      job,
      message="Preparing upload",
      current=0,
      total=total,
      percent=0,
      phase=UPLOAD_PHASE_PREPARING,
      phase_current=0,
      phase_total=total,
    )

  ensure_not_canceled(job)

  # Upload segments in parallel with bounded concurrency. Each segment uses an
  # independent HTTPS session. Keep the limit small because the upload service
  # and its backing storage are shared across devices.
  try:
    concurrency = max(1, min(6, int(os.environ.get("CARROT_WEB_UPLOAD_CONCURRENCY", "3") or "3")))
  except Exception:
    concurrency = 3

  prepare_sem = asyncio.Semaphore(concurrency)

  async def prepare_one(idx0: int, segment: str) -> tuple[int, list[Any], Exception | None]:
    try:
      async with prepare_sem:
        return idx0, await asyncio.to_thread(segment_file_summary, segment_dir(segment)), None
    except Exception as exc:
      return idx0, [], exc

  prepared: list[tuple[list[Any], Exception | None] | None] = [None] * total
  prepared_count = 0
  prepare_tasks = [
    asyncio.create_task(prepare_one(idx0, segment))
    for idx0, segment in enumerate(segments)
  ]
  try:
    for prepare_task in asyncio.as_completed(prepare_tasks):
      idx0, files, prepare_error = await prepare_task
      prepared[idx0] = (files, prepare_error)
      prepared_count += 1
      ensure_not_canceled(job)
      if job:
        prepare_percent = (
          round((prepared_count / total) * UPLOAD_PREPARING_END_PERCENT)
          if total > 0
          else UPLOAD_PREPARING_END_PERCENT
        )
        progress(
          job,
          message=f"Prepared {prepared_count}/{total}",
          current=prepared_count,
          total=total,
          percent=prepare_percent,
          phase=UPLOAD_PHASE_PREPARING,
          phase_current=prepared_count,
          phase_total=total,
        )
  except BaseException:
    for prepare_task in prepare_tasks:
      if not prepare_task.done():
        prepare_task.cancel()
    await asyncio.gather(*prepare_tasks, return_exceptions=True)
    raise

  normalized_prepared = [
    item if item is not None else ([], RuntimeError("segment preparation did not complete"))
    for item in prepared
  ]
  total_bytes = sum(
    max(0, int(item.get("size") or 0))
    for files, error in normalized_prepared
    if error is None
    for item in files
  )
  if job:
    progress(
      job,
      message="Uploading files",
      current=0,
      total=total,
      percent=UPLOAD_PREPARING_END_PERCENT,
      phase=UPLOAD_PHASE_UPLOADING,
      phase_current=0,
      phase_total=total_bytes if total_bytes > 0 else total,
      bytes_current=0,
      bytes_total=total_bytes,
      bytes_per_second=0,
    )

  sem = asyncio.Semaphore(concurrency)
  completed = 0
  logical_bytes = 0
  transmitted_bytes = 0
  logical_by_file: dict[tuple[int, str], int] = {}
  transfer_started = time.monotonic()
  speed_samples: deque[tuple[float, int]] = deque([(transfer_started, 0)])

  def current_percent() -> int:
    byte_ratio = logical_bytes / total_bytes if total_bytes > 0 else 0
    step_ratio = completed / total if total > 0 else 0
    transfer_ratio = max(0.0, min(1.0, max(byte_ratio, step_ratio)))
    transfer_span = UPLOAD_TRANSFERRING_END_PERCENT - UPLOAD_PREPARING_END_PERCENT
    return min(
      UPLOAD_TRANSFERRING_END_PERCENT,
      UPLOAD_PREPARING_END_PERCENT + round(transfer_ratio * transfer_span),
    )

  async def upload_one(idx0: int, segment: str) -> None:
    nonlocal completed, logical_bytes, transmitted_bytes
    idx = idx0 + 1
    files, prepare_error = normalized_prepared[idx0]

    def on_file_progress(filename: str, sent: int, file_size: int, delta: int) -> None:
      nonlocal logical_bytes, transmitted_bytes
      if not job:
        return
      key = (idx0, filename)
      previous = logical_by_file.get(key, 0)
      bounded = max(0, min(int(sent), max(0, int(file_size))))
      current = max(previous, bounded)
      logical_by_file[key] = current
      logical_bytes += current - previous
      transmitted_bytes += max(0, int(delta))

      now = time.monotonic()
      speed_samples.append((now, transmitted_bytes))
      while len(speed_samples) > 2 and now - speed_samples[0][0] > 3.0:
        speed_samples.popleft()
      sample_time, sample_bytes = speed_samples[0]
      elapsed = max(0.001, now - sample_time)
      current_bytes = min(total_bytes, logical_bytes)
      progress(
        job,
        percent=current_percent(),
        phase=UPLOAD_PHASE_UPLOADING,
        phase_current=current_bytes if total_bytes > 0 else completed,
        phase_total=total_bytes if total_bytes > 0 else total,
        bytes_current=current_bytes,
        bytes_total=total_bytes,
        bytes_per_second=max(0, round((transmitted_bytes - sample_bytes) / elapsed)),
      )

    async with sem:
      if is_cancel_requested(job):
        return
      if job:
        append(job, f"[{idx}/{total}] {segment}")
      try:
        if prepare_error is not None:
          raise prepare_error
        segment_path = segment_dir(segment)

        def should_cancel() -> bool:
          if job:
            touch(job)
          return is_cancel_requested(job)

        ok = await upload_folder_to_web(
          segment_path,
          device_id,
          segment,
          base_url,
          token,
          should_cancel if job else None,
          filenames=[str(item["name"]) for item in files],
          on_progress=on_file_progress if job else None,
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
      progress(
        job,
        message=f"Uploaded {completed}/{total}",
        current=completed,
        total=total,
        percent=current_percent(),
        phase=UPLOAD_PHASE_UPLOADING,
        phase_current=min(total_bytes, logical_bytes) if total_bytes > 0 else completed,
        phase_total=total_bytes if total_bytes > 0 else total,
      )

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
    "target": "web",
    "deviceId": device_id,
    "remoteBasePath": remote_base_path,
    "meta": meta,
    "results": results,
    "message": f"{ok_count}/{len(results)} uploaded",
  }
  response_payload["shareText"] = upload.upload_share_text(response_payload)

  if job:
    progress(
      job,
      message="Sending notification",
      current=total,
      total=total,
      percent=UPLOAD_NOTIFYING_START_PERCENT,
      phase=UPLOAD_PHASE_NOTIFYING,
      phase_current=0,
      phase_total=2,
    )
  ensure_not_canceled(job)
  response_payload["webComplete"] = await send_web_upload_complete(
    base_url,
    token,
    response_payload,
  )
  if job:
    progress(
      job,
      message="Sending notification",
      current=total,
      total=total,
      percent=UPLOAD_NOTIFYING_END_PERCENT,
      phase=UPLOAD_PHASE_NOTIFYING,
      phase_current=1,
      phase_total=2,
    )
  response_payload["discord"] = await upload.send_discord_webhook(
    upload.discord_webhook_url(params),
    response_payload,
  )
  if job:
    progress(
      job,
      message="Finalizing upload",
      current=total,
      total=total,
      percent=UPLOAD_NOTIFYING_END_PERCENT,
      phase=UPLOAD_PHASE_NOTIFYING,
      phase_current=2,
      phase_total=2,
    )
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
      "target": job.get("upload_target") or "web",
      "remoteBasePath": job.get("remote_base_path") or "",
      "meta": job.get("upload_meta") or {},
      "results": results,
      "message": f"Canceled {ok_count}/{total}",
      "error": str(exc),
    }
    result["shareText"] = upload.upload_share_text(result)
    append(job, "CANCELED")
    progress(job, message="Upload canceled", percent=0, phase=UPLOAD_PHASE_CANCELED)
    finish(job, ok=False, result=result, error=str(exc), status="canceled")
  except Exception as exc:
    result = {"ok": False, "error": str(exc)}
    append(job, f"FAILED: {exc}")
    finish(job, ok=False, result=result, error=str(exc))
