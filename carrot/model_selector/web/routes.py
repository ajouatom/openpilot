"""aiohttp routes for the model selector (plugged into carrot web)."""
from __future__ import annotations

import asyncio
import shutil
import subprocess
from pathlib import Path

from aiohttp import web

from openpilot.common.params import Params

from ..config import (
    MODELS_DIR,
    MODELS_TMP_DIR,
    PARAM_DRIVING_MODEL_NAME,
    PARAM_PENDING_MODEL_NAME,
)
from ..downloader import DownloadError, download_model
from ..installer import reset_to_default
from ..jobs import Job, get as job_get, start as job_start
from ..manifest import ManifestError, ModelEntry, fetch_and_verify
from ..validator import (
    describe,
    has_off_policy,
    has_on_policy,
    has_policy,
    has_supercombo,
    has_vision,
    is_valid_model_dir,
)

WEB_DIR = Path(__file__).parent / "frontend"


# -- helpers ---------------------------------------------------------------

def _disk_free_mb(path: Path) -> int:
    try:
        usage = shutil.disk_usage(str(path if path.exists() else path.parent))
        return usage.free // (1024 * 1024)
    except Exception:
        return -1


def _model_entry_json(entry: ModelEntry) -> dict:
    return {
        "id": entry.id,
        "name": entry.name,
        "base_url": entry.base_url,
        "added_at": entry.added_at,
        "files": [
            {"name": f.name, "size": f.size, "sha256": f.sha256}
            for f in entry.files.values()
        ],
        "total_size": sum(f.size for f in entry.files.values()),
        "has_off_policy": "driving_off_policy.onnx" in entry.files,
        "has_on_policy": "driving_on_policy.onnx" in entry.files,
        "has_supercombo": "driving_supercombo.onnx" in entry.files,
        "minimum_selector_version": entry.minimum_selector_version,
    }


# -- endpoints -------------------------------------------------------------

async def api_list(request: web.Request) -> web.Response:
    try:
        entries = await asyncio.get_event_loop().run_in_executor(None, fetch_and_verify)
    except ManifestError as e:
        return web.json_response({"ok": False, "error": f"manifest: {e}"}, status=502)
    return web.json_response({
        "ok": True,
        "models": [_model_entry_json(e) for e in entries],
    })


async def api_status(request: web.Request) -> web.Response:
    params = Params()
    current = params.get(PARAM_DRIVING_MODEL_NAME) or ""
    pending = params.get(PARAM_PENDING_MODEL_NAME) or ""
    return web.json_response({
        "ok": True,
        "current_model": current,
        "pending_model": pending,
        "is_custom_installed": is_valid_model_dir(MODELS_DIR),
        "flags": {
            "vision": has_vision(MODELS_DIR),
            "on_policy": has_on_policy(MODELS_DIR),
            "policy": has_policy(MODELS_DIR),
            "off_policy": has_off_policy(MODELS_DIR),
            "supercombo": has_supercombo(MODELS_DIR),
        },
        "description": describe(MODELS_DIR),
        "disk_free_mb": _disk_free_mb(MODELS_DIR),
    })


async def api_install(request: web.Request) -> web.Response:
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "bad json"}, status=400)
    model_id = str(payload.get("id") or "").strip()
    if not model_id:
        return web.json_response({"ok": False, "error": "missing id"}, status=400)

    # Look up the manifest entry so we have size/sha256/baseUrl to verify against.
    try:
        entries = await asyncio.get_event_loop().run_in_executor(None, fetch_and_verify)
    except ManifestError as e:
        return web.json_response({"ok": False, "error": f"manifest: {e}"}, status=502)
    entry = next((e for e in entries if e.id == model_id), None)
    if entry is None:
        return web.json_response({"ok": False, "error": "model not in manifest"}, status=404)

    async def run(job: Job) -> None:
        job.set_progress(message=f"preparing {entry.id}", current=0, total=len(entry.files))

        # Drop any previous pending name before touching tmp: download_model
        # wipes tmp first, so on failure a stale name must not survive and get
        # attached to leftover (or partially re-downloaded) files at boot.
        Params().remove(PARAM_PENDING_MODEL_NAME)

        loop = asyncio.get_event_loop()
        files = list(sorted(entry.files.keys()))
        completed = 0

        def progress_cb(name: str, done: int, total: int) -> None:
            # File-level percent feeds into job-level percent.
            file_idx = files.index(name)
            overall = int((file_idx + (done / total if total else 0)) / len(files) * 100)
            job.set_progress(
                message=f"{name} {done // (1024 * 1024)}/{total // (1024 * 1024)}MB",
                percent=max(0, min(99, overall)),
            )

        try:
            await loop.run_in_executor(None, lambda: download_model(entry, progress_cb))
            completed = len(files)
        except DownloadError as e:
            job.finish(ok=False, error=f"download: {e}")
            return

        # Mark pending; actual compile happens on next boot via boot_compile.run.
        Params().put(PARAM_PENDING_MODEL_NAME, entry.id)
        job.set_progress(message=f"downloaded {completed} files — will compile on reboot",
                         current=completed, total=len(files), percent=100)
        job.finish(ok=True, result={
            "ok": True,
            "id": entry.id,
            "pending": True,
            "tmp_dir": str(MODELS_TMP_DIR),
        })

    job = job_start("install_model", run)
    return web.json_response({"ok": True, "id": job.id})


async def api_job(request: web.Request) -> web.Response:
    job_id = request.query.get("id") or ""
    job = job_get(job_id)
    if job is None:
        return web.json_response({"ok": False, "error": "unknown job id"}, status=404)
    return web.json_response(job.snapshot())


async def api_apply(request: web.Request) -> web.Response:
    # Pending name is set at the end of install; apply just triggers reboot.
    params = Params()
    if not params.get(PARAM_PENDING_MODEL_NAME):
        return web.json_response({"ok": False, "error": "no pending model"}, status=409)

    async def reboot_soon() -> None:
        await asyncio.sleep(5.0)
        subprocess.Popen(["sudo", "reboot"])

    asyncio.get_event_loop().create_task(reboot_soon())
    return web.json_response({"ok": True, "reboot_in": 5})


async def api_reset(request: web.Request) -> web.Response:
    try:
        await asyncio.get_event_loop().run_in_executor(None, reset_to_default)
    except Exception as e:
        return web.json_response({"ok": False, "error": str(e)}, status=500)

    async def reboot_soon() -> None:
        await asyncio.sleep(5.0)
        subprocess.Popen(["sudo", "reboot"])

    asyncio.get_event_loop().create_task(reboot_soon())
    return web.json_response({"ok": True, "reboot_in": 5})


# -- frontend --------------------------------------------------------------

async def page_html(request: web.Request) -> web.Response:
    return web.FileResponse(WEB_DIR / "page_models.html")


async def page_js(request: web.Request) -> web.Response:
    return web.FileResponse(WEB_DIR / "page_models.js")


# -- registration ----------------------------------------------------------

def register(app: web.Application) -> None:
    app.router.add_get("/api/models/list", api_list)
    app.router.add_get("/api/models/status", api_status)
    app.router.add_post("/api/models/install", api_install)
    app.router.add_get("/api/models/job", api_job)
    app.router.add_post("/api/models/apply", api_apply)
    app.router.add_post("/api/models/reset", api_reset)

    # Serve the page fragment + its JS alongside the rest of the carrot
    # static assets.  index.html will fetch these on demand.
    app.router.add_get("/models/page_models.html", page_html)
    app.router.add_get("/models/page_models.js", page_js)
