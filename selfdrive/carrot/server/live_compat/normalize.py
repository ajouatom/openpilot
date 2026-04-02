from __future__ import annotations

from typing import Any


def safe_get(obj: Any, attr: str, default: Any = None) -> Any:
  if obj is None:
    return default
  try:
    return getattr(obj, attr)
  except Exception:
    return default


def safe_chain(obj: Any, *attrs: str, default: Any = None) -> Any:
  current = obj
  for attr in attrs:
    current = safe_get(current, attr, default=None)
    if current is None:
      return default
  return current


def safe_float(value: Any, default: float | None = None) -> float | None:
  try:
    if value is None:
      return default
    return float(value)
  except Exception:
    return default


def safe_int(value: Any, default: int | None = None) -> int | None:
  try:
    if value is None:
      return default
    return int(value)
  except Exception:
    return default


def safe_bool(value: Any, default: bool = False) -> bool:
  try:
    if value is None:
      return default
    return bool(value)
  except Exception:
    return default


def safe_text(value: Any, default: str | None = None) -> str | None:
  try:
    if value is None:
      return default
    if isinstance(value, bytes):
      return value.decode("utf-8", errors="replace")
    return str(value)
  except Exception:
    return default


def to_list(value: Any, limit: int | None = None) -> list[Any]:
  if value is None:
    return []
  try:
    data = list(value)
  except Exception:
    return []
  if limit is not None:
    return data[:limit]
  return data


def pick_first(*values: Any, default: Any = None) -> Any:
  for value in values:
    if value is not None:
      return value
  return default
