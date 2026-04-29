import json
import os
import re
import time
from typing import Any, Dict, Optional

from ..config import CARROT_GIT_STATE_PATH, CARROT_STATE_DIR


def read_git_state() -> Dict[str, Any]:
  try:
    with open(CARROT_GIT_STATE_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
    return data if isinstance(data, dict) else {}
  except Exception:
    return {}


def write_git_state(data: Dict[str, Any]) -> None:
  try:
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    tmp_path = f"{CARROT_GIT_STATE_PATH}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      json.dump(data, f, ensure_ascii=True, separators=(",", ":"))
    os.replace(tmp_path, CARROT_GIT_STATE_PATH)
  except Exception:
    pass


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
