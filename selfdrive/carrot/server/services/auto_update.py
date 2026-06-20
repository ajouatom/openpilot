from __future__ import annotations

import asyncio
import os
import re
import time

from .git_state import did_git_pull_update, write_git_pull_time
from .git_status import REPO_DIR, clear_git_status_cache, get_git_status
from .web_settings import read_web_settings

# Device-side auto update. Runs inside carrot_server (always_run), so it works
# whenever the device is on — no browser/web tab needed. Mirrors the manual
# "git pull" tool (hard reset + pull). Never reboots; the user reboots later.
AUTO_UPDATE_POLL_INTERVAL = 60.0
AUTO_UPDATE_COOLDOWN = 300.0       # min seconds between pulls
AUTO_UPDATE_INITIAL_DELAY = 30.0
RESET_TIMEOUT = 120.0
PULL_TIMEOUT = 180.0
GIT_INFO_TIMEOUT = 10.0    # cheap rev-parse/log/diff lookups
NOTIFY_TIMEOUT = 4.0       # CWP push POST (fire-and-forget)

_last_pull_at = 0.0


async def _git(args: list[str], timeout: float) -> tuple[int, str]:
  try:
    proc = await asyncio.create_subprocess_exec(
      "git",
      *args,
      cwd=REPO_DIR,
      stdout=asyncio.subprocess.PIPE,
      stderr=asyncio.subprocess.STDOUT,
    )
    out_bytes, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    return int(proc.returncode or 0), (out_bytes or b"").decode("utf-8", "replace").strip()
  except asyncio.TimeoutError:
    return 124, "timeout"
  except Exception as exc:
    return 1, str(exc)


def _auto_update_enabled() -> bool:
  try:
    return bool(read_web_settings().get("auto_update_git_pull"))
  except Exception:
    return False


async def _diff_stats(old_head: str, new_head: str) -> tuple[int, int, int]:
  # Aggregate "N files changed, X insertions(+), Y deletions(-)" across the pull.
  _, out = await _git(["diff", "--shortstat", old_head, new_head], GIT_INFO_TIMEOUT)
  files = additions = deletions = 0
  m = re.search(r"(\d+) files? changed", out)
  if m:
    files = int(m.group(1))
  m = re.search(r"(\d+) insertions?\(\+\)", out)
  if m:
    additions = int(m.group(1))
  m = re.search(r"(\d+) deletions?\(-\)", out)
  if m:
    deletions = int(m.group(1))
  return files, additions, deletions


async def _notify_cwp(old_head: str) -> None:
  # Best-effort "what changed" push to the CWP NAS, reusing the cweb_push client
  # helpers. Never raises (caller wraps too) and runs the blocking POST in a
  # thread so the aiohttp event loop is never stalled by a dead/slow server.
  from openpilot.common.params import Params

  from ...cweb_push import (
    _decode_default_report_url,
    _default_notify_url,
    device_id,
    post_json,
  )

  if not old_head:
    return

  rc, new_head = await _git(["rev-parse", "HEAD"], GIT_INFO_TIMEOUT)
  new_head = new_head.strip()
  if rc != 0 or not new_head or new_head == old_head:
    return

  _, branch = await _git(["branch", "--show-current"], GIT_INFO_TIMEOUT)
  branch = branch.strip()

  rc, log_out = await _git(["log", "--pretty=%h|%s", f"{old_head}..{new_head}"], GIT_INFO_TIMEOUT)
  commits = []
  if rc == 0:
    for line in log_out.splitlines():
      h, sep, subject = line.partition("|")
      if sep:
        commits.append({"hash": h.strip(), "subject": subject.strip()})
  if not commits:
    return

  files, additions, deletions = await _diff_stats(old_head, new_head)

  report_url = os.environ.get("CWEB_PUSH_REPORT_URL") or _decode_default_report_url()
  notify_url = os.environ.get("CWEB_PUSH_NOTIFY_URL") or _default_notify_url(report_url)

  payload = {
    "deviceId": device_id(Params()),
    "branch": branch,
    "count": len(commits),
    "head": new_head[:7],
    "commits": commits[:10],
    "files": files,
    "additions": additions,
    "deletions": deletions,
  }
  token = (os.environ.get("CWEB_PUSH_REPORT_TOKEN") or "").strip()
  if token:
    payload["token"] = token

  ok, status, _ = await asyncio.to_thread(post_json, notify_url, payload, NOTIFY_TIMEOUT)
  print(f"[auto_update] notify {'sent' if ok else 'failed'} commits={len(commits)} http={status}", flush=True)


async def _run_git_pull() -> bool:
  rc, old_head = await _git(["rev-parse", "HEAD"], GIT_INFO_TIMEOUT)
  old_head = old_head.strip() if rc == 0 else ""
  # Same as the manual git pull button: hard reset then pull. No reboot.
  await _git(["reset", "--hard"], RESET_TIMEOUT)
  rc, out = await _git(["pull"], PULL_TIMEOUT)
  if rc == 0 and did_git_pull_update(out):
    try:
      write_git_pull_time()
    except Exception:
      pass
    try:
      await _notify_cwp(old_head)
    except Exception as exc:
      print(f"[auto_update] notify skipped: {exc}", flush=True)
  return rc == 0


async def auto_update_loop(
  interval: float = AUTO_UPDATE_POLL_INTERVAL,
  initial_delay: float = AUTO_UPDATE_INITIAL_DELAY,
) -> None:
  global _last_pull_at
  if initial_delay > 0:
    await asyncio.sleep(initial_delay)

  while True:
    try:
      if _auto_update_enabled():
        status = await get_git_status()
        behind = int(status.get("behind") or 0)
        if behind > 0 and (time.time() - _last_pull_at) >= AUTO_UPDATE_COOLDOWN:
          print(f"[auto_update] behind {behind} commit(s) -> git pull")
          _last_pull_at = time.time()
          ok = await _run_git_pull()
          clear_git_status_cache()
          print(f"[auto_update] git pull {'ok' if ok else 'failed'}")
    except asyncio.CancelledError:
      raise
    except Exception as exc:
      print(f"[auto_update] loop error: {exc}")
    await asyncio.sleep(interval)
