import asyncio
import time
import uuid

from aiohttp import web

from . import jobs
from .dispatcher import dispatch_sync, run_tool_job


async def api_tools_start(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  action = body.get("action")
  if not action:
    return web.json_response({"ok": False, "error": "missing action"}, status=400)

  job_id = uuid.uuid4().hex[:12]
  job = {
    "id": job_id,
    "action": str(action),
    "payload": dict(body),
    "status": "running",
    "log": "",
    "progress": 0,
    "message": "",
    "step_current": 0,
    "step_total": 0,
    "error": None,
    "error_code": None,
    "error_detail": None,
    "result": None,
    "created_at": time.time(),
    "updated_at": time.time(),
  }
  jobs.jobs()[job_id] = job
  jobs.prune()
  asyncio.create_task(run_tool_job(job))
  return web.json_response({"ok": True, "job_id": job_id, "status": job["status"]})


async def api_tools_job(request: web.Request) -> web.Response:
  job_id = (request.query.get("id") or request.match_info.get("job_id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)

  job = jobs.jobs().get(job_id)
  if not job:
    return web.json_response({"ok": False, "error": "job not found"}, status=404)

  return web.json_response(jobs.snapshot(job))


async def api_tools(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)
  return await dispatch_sync(request, body)


def register(app: web.Application) -> None:
  app.router.add_post("/api/tools", api_tools)
  app.router.add_post("/api/tools/start", api_tools_start)
  app.router.add_get("/api/tools/job", api_tools_job)
