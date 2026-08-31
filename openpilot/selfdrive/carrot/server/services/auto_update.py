from __future__ import annotations

import asyncio
import os
import re
import time

from .git_state import read_auto_update_state, write_auto_update_event, write_git_pull_time
from .git_status import REPO_DIR, clear_git_status_cache, get_git_status
from .web_settings import read_web_settings

# Device-side auto update. Runs inside carrot_server (always_run), so it works
# whenever the device is on — no browser/web tab needed. Mirrors the manual
# "git pull" tool (hard reset + pull). An optional reboot is armed only when
# the pull actually changes HEAD, then waits for the configured condition.
AUTO_UPDATE_POLL_INTERVAL = 60.0
AUTO_UPDATE_COOLDOWN = 300.0       # min seconds between pulls
AUTO_UPDATE_INITIAL_DELAY = 30.0
AUTO_REBOOT_POLL_INTERVAL = 0.1
AUTO_REBOOT_DISENGAGED_DELAY = 1.0
RESET_TIMEOUT = 120.0
PULL_TIMEOUT = 180.0
GIT_INFO_TIMEOUT = 10.0    # cheap rev-parse/log/diff lookups
NOTIFY_TIMEOUT = 4.0       # CWP push POST (fire-and-forget)
AUTO_UPDATE_ERROR_ALERT = "Offroad_CarrotAutoUpdateFailed"
AUTO_UPDATE_ERROR_DETAIL_LIMIT = 2000

AUTO_REBOOT_OFF = "off"
AUTO_REBOOT_PARK = "park"
AUTO_REBOOT_DISENGAGED = "disengaged"
AUTO_REBOOT_MODES = {AUTO_REBOOT_OFF, AUTO_REBOOT_PARK, AUTO_REBOOT_DISENGAGED}

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


def _auto_reboot_mode() -> str:
  try:
    mode = str(read_web_settings().get("auto_update_reboot") or AUTO_REBOOT_OFF).strip().lower()
  except Exception:
    return AUTO_REBOOT_OFF
  return mode if mode in AUTO_REBOOT_MODES else AUTO_REBOOT_OFF


def _verified_update_target(status: dict) -> tuple[int, str]:
  # A failed fetch can leave a stale remote-tracking ref that still reports
  # `behind > 0`. Never use that stale count to start an automatic pull.
  if not bool(status.get("available")) or status.get("state") != "ok":
    return 0, ""
  behind = max(0, int(status.get("behind") or 0))
  target_head = str(status.get("target_head") or "").strip()
  return (behind, target_head) if behind > 0 and target_head else (0, "")


def _message_valid(sm, service: str) -> bool:
  try:
    return bool(sm.valid[service]) and bool(sm.alive[service])
  except Exception:
    return False


def _is_park(gear_shifter) -> bool:
  try:
    return str(gear_shifter).strip().lower().rsplit(".", 1)[-1] == "park"
  except Exception:
    return False


class AutoRebootCondition:
  """Tracks continuous vehicle state for one pending post-update reboot."""

  def __init__(self, mode: str, ready_delay: float = AUTO_REBOOT_DISENGAGED_DELAY) -> None:
    self.mode = mode if mode in AUTO_REBOOT_MODES else AUTO_REBOOT_OFF
    self.ready_delay = max(0.0, float(ready_delay))
    self.ready_since: float | None = None

  def update(
    self,
    *,
    now: float,
    selfdrive_valid: bool,
    engaged: bool,
    car_state_valid: bool = False,
    gear_shifter=None,
    device_state_valid: bool = False,
    device_started: bool = True,
  ) -> bool:
    if self.mode == AUTO_REBOOT_PARK:
      return selfdrive_valid and not engaged and car_state_valid and _is_park(gear_shifter)

    if self.mode != AUTO_REBOOT_DISENGAGED:
      return False

    disengaged = selfdrive_valid and not engaged
    offroad = device_state_valid and not device_started
    if not (disengaged or offroad):
      self.ready_since = None
      return False
    if self.ready_since is None:
      self.ready_since = now
    return now - self.ready_since >= self.ready_delay


def _request_reboot() -> None:
  from openpilot.common.params import Params

  Params().put_bool("DoReboot", True)


def _short_error(output: str, fallback: str) -> str:
  detail = " ".join(str(output or "").strip().split())
  return (detail or fallback)[-AUTO_UPDATE_ERROR_DETAIL_LIMIT:]


def _set_auto_update_alert(show: bool, detail: str = "") -> None:
  try:
    from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert

    set_offroad_alert(AUTO_UPDATE_ERROR_ALERT, show, extra_text=detail if show else None)
  except Exception as exc:
    print(f"[auto_update] offroad alert error: {exc}", flush=True)


def _record_error(error_code: str, detail: str, *, blocked: bool = False, **fields) -> None:
  error = _short_error(detail, error_code)
  status = "reboot_blocked" if blocked else "error"
  write_auto_update_event(status, error_code=error_code, error=error, **fields)
  _set_auto_update_alert(True, error)
  print(f"[auto_update] {status} code={error_code}: {error}", flush=True)


async def _wait_for_auto_reboot(initial_mode: str, updated_head: str) -> None:
  # Create a dedicated SubMaster only while an actual update is pending, so
  # ordinary auto-update polling adds no continuous msgq workload.
  from openpilot.cereal import messaging

  sm = messaging.SubMaster(["carState", "selfdriveState", "deviceState"])
  mode = initial_mode
  condition = AutoRebootCondition(mode)
  pending = write_auto_update_event(
    "reboot_pending",
    new_head=updated_head,
    target_head=updated_head,
    error_code="",
    error="",
    reboot_mode=mode,
  )
  if not pending:
    _record_error("state_write_failed", "Unable to save automatic-update reboot state")
    return
  print(f"[auto_update] reboot armed mode={mode}", flush=True)

  while True:
    selected_mode = _auto_reboot_mode()
    if selected_mode == AUTO_REBOOT_OFF:
      write_auto_update_event("updated", new_head=updated_head, target_head=updated_head, reboot_mode=AUTO_REBOOT_OFF)
      print("[auto_update] reboot cancelled", flush=True)
      return
    if selected_mode != mode:
      mode = selected_mode
      condition = AutoRebootCondition(mode)
      print(f"[auto_update] reboot mode changed mode={mode}", flush=True)

    sm.update(0)
    selfdrive_valid = _message_valid(sm, "selfdriveState")
    car_state_valid = _message_valid(sm, "carState")
    device_state_valid = _message_valid(sm, "deviceState")
    engaged = bool(sm["selfdriveState"].enabled) if selfdrive_valid else False
    gear_shifter = sm["carState"].gearShifter if car_state_valid else None
    device_started = bool(sm["deviceState"].started) if device_state_valid else True
    if condition.update(
      now=time.monotonic(),
      selfdrive_valid=selfdrive_valid,
      engaged=engaged,
      car_state_valid=car_state_valid,
      gear_shifter=gear_shifter,
      device_state_valid=device_state_valid,
      device_started=device_started,
    ):
      state = read_auto_update_state()
      if state.get("reboot_requested_head") == updated_head:
        _record_error(
          "duplicate_reboot_blocked",
          f"Automatic reboot for {updated_head[:12]} was already requested",
          blocked=True,
          new_head=updated_head,
          target_head=updated_head,
          reboot_requested_head=updated_head,
          reboot_mode=mode,
        )
        return
      requested = write_auto_update_event(
        "reboot_requested",
        new_head=updated_head,
        target_head=updated_head,
        error_code="",
        error="",
        reboot_requested_head=updated_head,
        reboot_mode=mode,
      )
      if not requested:
        _record_error("state_write_failed", "Unable to save automatic-update reboot request")
        return
      print(f"[auto_update] reboot condition met mode={mode}", flush=True)
      _request_reboot()
      return
    await asyncio.sleep(AUTO_REBOOT_POLL_INTERVAL)


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


async def _run_git_pull(target_head: str = "") -> tuple[bool, bool, str]:
  previous = read_auto_update_state()
  if target_head and previous.get("reboot_requested_head") == target_head:
    if previous.get("status") != "reboot_blocked" or previous.get("target_head") != target_head:
      _record_error(
        "duplicate_reboot_blocked",
        f"Automatic reboot for {target_head[:12]} was already requested",
        blocked=True,
        target_head=target_head,
        reboot_requested_head=target_head,
      )
    return False, False, ""

  rc, old_head = await _git(["rev-parse", "HEAD"], GIT_INFO_TIMEOUT)
  old_head = old_head.strip() if rc == 0 else ""
  if not old_head:
    _record_error("head_read_failed", "Unable to read the current Git HEAD", target_head=target_head)
    return False, False, ""

  attempted_at = int(time.time())
  started = write_auto_update_event(
    "pulling",
    attempted_at=attempted_at,
    old_head=old_head,
    new_head="",
    target_head=target_head,
    reset_rc=None,
    pull_rc=None,
    error_code="",
    error="",
  )
  if not started:
    _record_error("state_write_failed", "Unable to save automatic-update attempt")
    return False, False, ""

  # Keep the existing hard-reset behavior, but never pull or reboot if it fails.
  reset_rc, reset_out = await _git(["reset", "--hard"], RESET_TIMEOUT)
  if reset_rc != 0:
    _record_error(
      "reset_failed",
      reset_out,
      attempted_at=attempted_at,
      old_head=old_head,
      target_head=target_head,
      reset_rc=reset_rc,
    )
    return False, False, ""

  # Automatic updates must be a clean fast-forward; never create a merge commit.
  pull_rc, pull_out = await _git(["pull", "--ff-only"], PULL_TIMEOUT)
  if pull_rc != 0:
    _record_error(
      "pull_failed",
      pull_out,
      attempted_at=attempted_at,
      old_head=old_head,
      target_head=target_head,
      reset_rc=reset_rc,
      pull_rc=pull_rc,
    )
    return False, False, ""

  head_rc, new_head = await _git(["rev-parse", "HEAD"], GIT_INFO_TIMEOUT)
  new_head = new_head.strip() if head_rc == 0 else ""
  upstream_rc, upstream_head = await _git(["rev-parse", "@{u}"], GIT_INFO_TIMEOUT)
  upstream_head = upstream_head.strip() if upstream_rc == 0 else ""
  verified_target = upstream_head or target_head
  common_fields = {
    "attempted_at": attempted_at,
    "old_head": old_head,
    "new_head": new_head,
    "target_head": verified_target,
    "reset_rc": reset_rc,
    "pull_rc": pull_rc,
  }
  if not new_head:
    _record_error("head_read_failed", "Unable to verify Git HEAD after pull", **common_fields)
    return False, False, ""
  if not upstream_head:
    _record_error("upstream_read_failed", "Unable to verify the upstream Git HEAD after pull", **common_fields)
    return False, False, new_head
  if new_head == old_head:
    _record_error("head_unchanged", "git pull completed but HEAD did not change", **common_fields)
    return True, False, new_head
  if new_head != upstream_head:
    _record_error(
      "head_mismatch",
      f"Pulled HEAD {new_head[:12]} does not match upstream {upstream_head[:12]}",
      **common_fields,
    )
    return True, False, new_head

  if previous.get("reboot_requested_head") == new_head:
    write_git_pull_time()
    _record_error(
      "duplicate_reboot_blocked",
      f"Automatic reboot for {new_head[:12]} was already requested",
      blocked=True,
      reboot_requested_head=new_head,
      **common_fields,
    )
    return True, False, new_head

  updated = write_auto_update_event(
    "updated",
    error_code="",
    error="",
    **common_fields,
  )
  if not updated:
    _record_error("state_write_failed", "Unable to save the verified automatic update", **common_fields)
    return True, False, new_head

  _set_auto_update_alert(False)
  try:
    write_git_pull_time()
  except Exception:
    pass
  try:
    await _notify_cwp(old_head)
  except Exception as exc:
    print(f"[auto_update] notify skipped: {exc}", flush=True)
  return True, True, new_head


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
        behind, target_head = _verified_update_target(status)
        if behind > 0 and (time.time() - _last_pull_at) >= AUTO_UPDATE_COOLDOWN:
          print(f"[auto_update] behind {behind} commit(s) -> git pull")
          _last_pull_at = time.time()
          ok, updated, new_head = await _run_git_pull(target_head)
          clear_git_status_cache()
          print(f"[auto_update] git pull {'ok' if ok else 'failed'}")
          mode = _auto_reboot_mode()
          if updated and mode != AUTO_REBOOT_OFF:
            try:
              await _wait_for_auto_reboot(mode, new_head)
            except asyncio.CancelledError:
              raise
            except Exception as exc:
              _record_error(
                "reboot_monitor_failed",
                str(exc),
                new_head=new_head,
                target_head=new_head,
                reboot_mode=mode,
              )
              print(f"[auto_update] reboot monitor error: {exc}", flush=True)
    except asyncio.CancelledError:
      raise
    except Exception as exc:
      print(f"[auto_update] loop error: {exc}")
    await asyncio.sleep(interval)
