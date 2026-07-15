"""Best-effort metadata snapshots for CarrotLog tmux uploads."""

import os
from dataclasses import dataclass
from datetime import UTC, datetime


@dataclass(frozen=True)
class TmuxCapture:
  path: str
  metadata: dict[str, str | None]


class TmuxCaptureStore:
  """Keep independent pending capture metadata for each upload reason."""

  def __init__(self):
    self._captures: dict[str, TmuxCapture] = {}

  def put(self, key, path, metadata):
    capture = TmuxCapture(str(path), dict(metadata))
    self._captures[str(key)] = capture
    return capture

  def get(self, key):
    return self._captures.get(str(key))

  def pop(self, key):
    return self._captures.pop(str(key), None)


def _param_text(params, key):
  try:
    value = params.get(key)
  except Exception:
    return ""
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="ignore").strip()
  return str(value or "").strip()


def _current_segment(log_root, route_id):
  if not route_id:
    return None
  prefix = f"{route_id}--"
  segments = []
  try:
    with os.scandir(log_root) as entries:
      for entry in entries:
        if not entry.name.startswith(prefix) or not entry.is_dir(follow_symlinks=False):
          continue
        suffix = entry.name[len(prefix):]
        if suffix.isdigit():
          segments.append(int(suffix))
  except (FileNotFoundError, NotADirectoryError, OSError):
    return None
  return str(max(segments)) if segments else None


def capture_tmux_metadata(params, log_root, now=None):
  """Capture route/segment/time once so upload retries keep the same metadata."""
  captured = now or datetime.now(UTC)
  if captured.tzinfo is None:
    captured = captured.replace(tzinfo=UTC)
  captured_at = captured.astimezone(UTC).isoformat().replace("+00:00", "Z")

  route_id = _param_text(params, "CurrentRoute")
  dongle_id = _param_text(params, "DongleId")
  route = None
  if route_id:
    route = route_id if "|" in route_id or "/" in route_id else f"{dongle_id}|{route_id}" if dongle_id else route_id

  return {
    "route": route,
    "segment": _current_segment(log_root, route_id),
    "captured_at": captured_at,
  }
