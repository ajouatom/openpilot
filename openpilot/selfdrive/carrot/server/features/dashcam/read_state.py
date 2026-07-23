import json
import os
import threading
import time

from aiohttp import web

from ...config import CARROT_DASHCAM_READ_STATE_PATH
from .paths import safe_segment


_state_lock = threading.Lock()


def normalize_recent_segment(value: object) -> str:
  try:
    return safe_segment(str(value or "").strip())
  except web.HTTPException:
    return ""


def read_dashcam_read_state() -> dict:
  try:
    with open(CARROT_DASHCAM_READ_STATE_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    raw = {}
  recent_segment = normalize_recent_segment(raw.get("recentSegment") if isinstance(raw, dict) else "")
  return {"recentSegment": recent_segment}


def write_dashcam_recent_segment(segment: object) -> dict:
  recent_segment = normalize_recent_segment(segment)
  if not recent_segment:
    raise ValueError("invalid recent segment")

  payload = {
    "version": 1,
    "recentSegment": recent_segment,
    "updatedAt": int(time.time()),
  }
  directory = os.path.dirname(CARROT_DASHCAM_READ_STATE_PATH)
  temp_path = CARROT_DASHCAM_READ_STATE_PATH + ".tmp"
  with _state_lock:
    os.makedirs(directory, exist_ok=True)
    with open(temp_path, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=False, separators=(",", ":"))
      f.write("\n")
    os.replace(temp_path, CARROT_DASHCAM_READ_STATE_PATH)
  return {"recentSegment": recent_segment}
