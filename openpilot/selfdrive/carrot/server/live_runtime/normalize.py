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


def safe_enum_int(value: Any, default: int | None = None) -> int | None:
  if value is None:
    return default
  raw = safe_get(value, "raw")
  parsed = safe_int(raw, default=None)
  if parsed is not None:
    return parsed
  enum_value = safe_get(value, "value")
  parsed = safe_int(enum_value, default=None)
  if parsed is not None:
    return parsed
  return safe_int(value, default=default)


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


def to_transport_safe(value: Any) -> Any:
  if value is None or isinstance(value, (str, int, float, bool)):
    return value

  raw = safe_get(value, "raw")
  if raw is not None:
    return safe_int(raw, default=safe_text(raw, default=None))

  enum_value = safe_get(value, "value")
  if enum_value is not None:
    return safe_int(enum_value, default=safe_text(enum_value, default=None))

  if isinstance(value, bytes):
    return value.decode("utf-8", errors="replace")

  if isinstance(value, dict):
    return {str(key): to_transport_safe(item) for key, item in value.items()}

  if isinstance(value, (list, tuple, set)):
    return [to_transport_safe(item) for item in value]

  tolist = safe_get(value, "tolist")
  if callable(tolist):
    try:
      return to_transport_safe(tolist())
    except Exception:
      pass

  return safe_text(value, default=str(type(value).__name__))
