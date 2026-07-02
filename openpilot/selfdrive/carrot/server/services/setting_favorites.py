from collections.abc import Iterable
import json
import os
from typing import Any, Dict, List, Optional

from ..config import CARROT_SETTING_FAVORITES_PATH


MAX_SETTING_FAVORITES = 200
DEFAULT_SETTING_FAVORITES: Dict[str, Any] = {
  "favorites": [],
}


def _normalize_favorites(value: Any) -> List[str]:
  if not isinstance(value, Iterable) or isinstance(value, (str, bytes, bytearray, dict)):
    return []

  out: List[str] = []
  seen = set()
  for item in value:
    name = str(item or "").strip()
    if not name or name in seen:
      continue
    seen.add(name)
    out.append(name)
    if len(out) >= MAX_SETTING_FAVORITES:
      break
  return out


def sanitize_setting_favorites(raw: Optional[Dict[str, Any]]) -> Dict[str, Any]:
  raw = raw or {}
  return {
    "favorites": _normalize_favorites(raw.get("favorites")),
  }


def read_setting_favorites() -> Dict[str, Any]:
  try:
    with open(CARROT_SETTING_FAVORITES_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    return dict(DEFAULT_SETTING_FAVORITES)
  return sanitize_setting_favorites(raw if isinstance(raw, dict) else {})


def write_setting_favorites(settings: Dict[str, Any]) -> Dict[str, Any]:
  clean = sanitize_setting_favorites(settings)
  os.makedirs(os.path.dirname(CARROT_SETTING_FAVORITES_PATH), exist_ok=True)
  tmp_path = CARROT_SETTING_FAVORITES_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(clean, f, ensure_ascii=False, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, CARROT_SETTING_FAVORITES_PATH)
  return clean


def update_setting_favorites(updates: Dict[str, Any]) -> Dict[str, Any]:
  if not isinstance(updates, dict):
    updates = {}
  current = read_setting_favorites()
  if "favorites" in updates:
    current["favorites"] = updates.get("favorites")
  return write_setting_favorites(current)
