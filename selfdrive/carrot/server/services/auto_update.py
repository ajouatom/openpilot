from __future__ import annotations

import asyncio
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


async def _run_git_pull() -> bool:
  # Same as the manual git pull button: hard reset then pull. No reboot.
  await _git(["reset", "--hard"], RESET_TIMEOUT)
  rc, out = await _git(["pull"], PULL_TIMEOUT)
  if rc == 0 and did_git_pull_update(out):
    try:
      write_git_pull_time()
    except Exception:
      pass
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
