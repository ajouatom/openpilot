"""Per-parameter step multiplier (the `x1` button) for the settings screen.

This used to live in browser localStorage, which meant it was lost whenever the
user switched phone or browser -- and, more often, whenever they were told to
clear their cache to chase an unrelated problem. Storing it with the other
per-device settings makes it survive both.
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict

from ..config import CARROT_SETTING_UNIT_INDEX_PATH, UNIT_CYCLE

# Bounded so a client that keeps inventing names cannot grow the file forever.
MAX_SETTING_UNIT_ENTRIES = 400

DEFAULT_SETTING_UNIT_INDEX: Dict[str, Any] = {"units": {}}


def _normalize_units(value: Any) -> Dict[str, int]:
  if not isinstance(value, dict):
    return {}

  out: Dict[str, int] = {}
  for raw_name, raw_index in value.items():
    name = str(raw_name or "").strip()
    if not name:
      continue
    try:
      index = int(raw_index)
    except (TypeError, ValueError):
      continue
    # Index 0 is the default multiplier, so storing it only adds noise.
    if index <= 0 or index >= len(UNIT_CYCLE):
      continue
    out[name] = index
    if len(out) >= MAX_SETTING_UNIT_ENTRIES:
      break
  return out


def sanitize_setting_unit_index(raw: Dict[str, Any] | None) -> Dict[str, Any]:
  raw = raw or {}
  return {"units": _normalize_units(raw.get("units"))}


def read_setting_unit_index() -> Dict[str, Any]:
  try:
    with open(CARROT_SETTING_UNIT_INDEX_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    return {"units": {}}
  return sanitize_setting_unit_index(raw if isinstance(raw, dict) else {})


def write_setting_unit_index(settings: Dict[str, Any]) -> Dict[str, Any]:
  clean = sanitize_setting_unit_index(settings)
  os.makedirs(os.path.dirname(CARROT_SETTING_UNIT_INDEX_PATH), exist_ok=True)
  tmp_path = CARROT_SETTING_UNIT_INDEX_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(clean, f, ensure_ascii=False, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, CARROT_SETTING_UNIT_INDEX_PATH)
  return clean


def update_setting_unit_index(updates: Dict[str, Any]) -> Dict[str, Any]:
  """Merge one or more entries. Index 0 removes the entry rather than storing it."""
  if not isinstance(updates, dict):
    updates = {}

  current = read_setting_unit_index()
  units = dict(current.get("units") or {})
  for raw_name, raw_index in (updates.get("units") or {}).items():
    name = str(raw_name or "").strip()
    if not name:
      continue
    try:
      index = int(raw_index)
    except (TypeError, ValueError):
      continue
    if index <= 0 or index >= len(UNIT_CYCLE):
      units.pop(name, None)
    else:
      units[name] = index

  return write_setting_unit_index({"units": units})
