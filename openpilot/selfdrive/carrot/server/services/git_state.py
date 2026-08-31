import json
import os
import re
import time
from typing import Any, Dict, Optional

from ..config import CARROT_GIT_STATE_PATH, CARROT_STATE_DIR


AUTO_UPDATE_HISTORY_LIMIT = 20


def read_git_state() -> Dict[str, Any]:
  try:
    with open(CARROT_GIT_STATE_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
    return data if isinstance(data, dict) else {}
  except Exception:
    return {}


def write_git_state(data: Dict[str, Any]) -> bool:
  try:
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    tmp_path = f"{CARROT_GIT_STATE_PATH}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      json.dump(data, f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_path, CARROT_GIT_STATE_PATH)
    return True
  except Exception:
    return False


def read_custom_meta_value(name: str) -> Optional[str]:
  if name != "GitPullTime":
    return None

  try:
    value = read_git_state().get("git_pull_time")
    if value is None:
      return None
    return str(value).strip()
  except Exception:
    return None


def write_git_pull_time(ts: Optional[int] = None) -> None:
  value = int(ts if ts is not None else time.time())
  data = read_git_state()
  data["git_pull_time"] = value
  data["git_pull_ok"] = True
  write_git_state(data)


def read_auto_update_state() -> Dict[str, Any]:
  try:
    state = read_git_state().get("auto_update")
    return dict(state) if isinstance(state, dict) else {}
  except Exception:
    return {}


def write_auto_update_event(status: str, **fields: Any) -> Dict[str, Any]:
  """Persist the latest automatic-update state and a bounded event history.

  This lives under /data rather than in the checkout, so a repo reset or reboot
  cannot erase the evidence needed to diagnose a failed update/reboot cycle.
  """
  data = read_git_state()
  previous = data.get("auto_update")
  state = dict(previous) if isinstance(previous, dict) else {}
  event_id = str(time.time_ns())
  now = int(time.time())
  state.update(fields)
  state.update({
    "status": str(status or "unknown"),
    "event_id": event_id,
    "updated_at": now,
  })
  data["auto_update"] = state

  history = data.get("auto_update_history")
  history = list(history) if isinstance(history, list) else []
  event = {
    "status": state["status"],
    "event_id": event_id,
    "updated_at": now,
  }
  for key in (
    "attempted_at", "old_head", "new_head", "target_head",
    "reset_rc", "pull_rc", "error_code", "error", "reboot_mode",
    "reboot_requested_head",
  ):
    if key in fields:
      event[key] = fields[key]
  history.append(event)
  data["auto_update_history"] = history[-AUTO_UPDATE_HISTORY_LIMIT:]
  return state if write_git_state(data) else {}


def did_git_pull_update(output: str) -> bool:
  body = str(output or "").strip().lower()
  if not body:
    return False
  if "already up to date" in body or "already up-to-date" in body:
    return False
  return (
    "fast-forward" in body or
    "merge made by" in body or
    "updating " in body or
    bool(re.search(r"[0-9]+\s+files?\s+changed", body))
  )
