"""Minimal in-memory job runner for the carrot web model selector routes.

Mirrors the polling-job shape of carrot's own web server
(`openpilot/selfdrive/carrot/server/app.py`) so the frontend can reuse the
same polling UI (progress bar, message, log).  Kept in a dedicated module so
the web routes stay declarative and unit-testable.
"""
from __future__ import annotations

import asyncio
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable


@dataclass
class Job:
    id: str
    action: str
    status: str = "running"  # running | done | failed
    message: str = ""
    progress: int | None = None
    step_current: int | None = None
    step_total: int | None = None
    log: str = ""
    error: str | None = None
    result: dict[str, Any] | None = None
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

    def touch(self) -> None:
        self.updated_at = time.time()

    def append_log(self, text: str) -> None:
        if not text:
            return
        if self.log and not self.log.endswith("\n") and not text.startswith("\n"):
            self.log += "\n"
        self.log += text
        self.touch()

    def set_progress(self, *, message: str | None = None,
                     current: int | None = None, total: int | None = None,
                     percent: int | None = None) -> None:
        if message is not None:
            self.message = message
        if current is not None:
            self.step_current = max(0, int(current))
        if total is not None:
            self.step_total = max(0, int(total))
        if percent is None:
            if self.step_current is not None and self.step_total and self.step_total > 0:
                percent = int(max(0, min(100, round((self.step_current / self.step_total) * 100))))
        if percent is not None:
            self.progress = percent
        self.touch()

    def finish(self, *, ok: bool, result: dict[str, Any] | None = None,
               error: str | None = None) -> None:
        self.status = "done" if ok else "failed"
        self.result = result or {"ok": bool(ok)}
        self.error = error
        if ok:
            self.progress = 100
        self.touch()

    def snapshot(self) -> dict[str, Any]:
        return {
            "ok": True,
            "id": self.id,
            "action": self.action,
            "status": self.status,
            "done": self.status in ("done", "failed"),
            "log": self.log,
            "progress": self.progress,
            "message": self.message,
            "step_current": self.step_current,
            "step_total": self.step_total,
            "error": self.error,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "result": self.result,
        }


KEEP_COUNT = 16
_jobs: dict[str, Job] = {}


def _prune() -> None:
    finished = sorted(
        (j for j in _jobs.values() if j.status in ("done", "failed")),
        key=lambda j: j.updated_at,
        reverse=True,
    )
    for old in finished[KEEP_COUNT:]:
        _jobs.pop(old.id, None)


JobFn = Callable[[Job], Awaitable[None]]


def start(action: str, fn: JobFn) -> Job:
    """Create a job and schedule `fn(job)` on the running event loop."""
    job = Job(id=uuid.uuid4().hex[:12], action=action)
    _jobs[job.id] = job

    async def _wrapper() -> None:
        try:
            await fn(job)
            if job.status == "running":
                job.finish(ok=True)
        except Exception as e:
            job.append_log(f"[error] {e}\n")
            job.finish(ok=False, error=str(e))
        finally:
            _prune()

    asyncio.get_event_loop().create_task(_wrapper())
    return job


def get(job_id: str) -> Job | None:
    return _jobs.get(job_id)


def list_recent(limit: int = 16) -> list[Job]:
    items = sorted(_jobs.values(), key=lambda j: j.updated_at, reverse=True)
    return items[:limit]
