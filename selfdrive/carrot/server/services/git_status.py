from __future__ import annotations

import asyncio
import time
from typing import Any


REPO_DIR = "/data/openpilot"
GIT_STATUS_TTL = 600.0
FETCH_TIMEOUT = 25.0
GIT_TIMEOUT = 8.0

_cache: dict[str, Any] | None = None
_lock: asyncio.Lock | None = None


def _now() -> float:
  return time.time()


def _lock_for_loop() -> asyncio.Lock:
  global _lock
  if _lock is None:
    _lock = asyncio.Lock()
  return _lock


def _error_state(message: str, **extra: Any) -> dict[str, Any]:
  return {
    "available": False,
    "state": "error",
    "behind": 0,
    "ahead": 0,
    "branch": "",
    "upstream": "",
    "checked_at": int(_now()),
    "error": message,
    **extra,
  }


async def _git(args: list[str], timeout: float = GIT_TIMEOUT) -> tuple[int, str]:
  try:
    proc = await asyncio.create_subprocess_exec(
      "git",
      *args,
      cwd=REPO_DIR,
      stdout=asyncio.subprocess.PIPE,
      stderr=asyncio.subprocess.STDOUT,
    )
    out_bytes, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    out = (out_bytes or b"").decode("utf-8", "replace").strip()
    return int(proc.returncode or 0), out
  except asyncio.TimeoutError:
    return 124, "timeout"
  except Exception as exc:
    return 1, str(exc)


async def _git_text(args: list[str], timeout: float = GIT_TIMEOUT) -> str:
  rc, out = await _git(args, timeout=timeout)
  return out if rc == 0 else ""


async def _resolve_upstream(branch: str) -> str:
  upstream = await _git_text(["rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"])
  if upstream:
    return upstream

  if not branch:
    return ""

  remotes = (await _git_text(["remote"])).split()
  for remote in ["origin", *[r for r in remotes if r != "origin"]]:
    candidate = f"{remote}/{branch}"
    rc, _ = await _git(["rev-parse", "--verify", "--quiet", candidate])
    if rc == 0:
      return candidate
  return ""


async def _read_status() -> dict[str, Any]:
  rc, inside = await _git(["rev-parse", "--is-inside-work-tree"])
  if rc != 0 or inside.lower() != "true":
    return _error_state("not a git repository")

  branch = await _git_text(["branch", "--show-current"])
  if not branch:
    branch = await _git_text(["rev-parse", "--short", "HEAD"])

  remote = await _git_text(["config", "--get", f"branch.{branch}.remote"]) if branch else ""
  if not remote:
    remotes = (await _git_text(["remote"])).split()
    remote = "origin" if "origin" in remotes else (remotes[0] if remotes else "")

  fetch_rc = 0
  fetch_out = ""
  if remote:
    fetch_rc, fetch_out = await _git(["fetch", "--quiet", "--prune", remote], timeout=FETCH_TIMEOUT)

  upstream = await _resolve_upstream(branch)
  if not upstream:
    return {
      "available": False,
      "state": "no_upstream",
      "behind": 0,
      "ahead": 0,
      "branch": branch,
      "upstream": "",
      "checked_at": int(_now()),
      "error": "no upstream branch",
      "fetch_error": fetch_out if fetch_rc != 0 else "",
    }

  rc, counts = await _git(["rev-list", "--left-right", "--count", f"HEAD...{upstream}"])
  if rc != 0:
    return _error_state(counts or "failed to compare git refs", branch=branch, upstream=upstream)

  parts = counts.split()
  ahead = int(parts[0]) if len(parts) > 0 and parts[0].isdigit() else 0
  behind = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0

  return {
    "available": fetch_rc == 0,
    "state": "ok" if fetch_rc == 0 else "fetch_error",
    "behind": behind,
    "ahead": ahead,
    "branch": branch,
    "upstream": upstream,
    "checked_at": int(_now()),
    "error": fetch_out if fetch_rc != 0 else "",
  }


async def get_git_status(force: bool = False) -> dict[str, Any]:
  global _cache
  if _cache and not force and (_now() - float(_cache.get("checked_at", 0))) < GIT_STATUS_TTL:
    return dict(_cache)

  async with _lock_for_loop():
    if _cache and not force and (_now() - float(_cache.get("checked_at", 0))) < GIT_STATUS_TTL:
      return dict(_cache)
    _cache = await _read_status()
    return dict(_cache)


def clear_git_status_cache() -> None:
  global _cache
  _cache = None
