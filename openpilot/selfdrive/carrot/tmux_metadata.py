"""Best-effort metadata snapshot for CarrotLog tmux uploads."""

from datetime import UTC, datetime


def _param_text(params, key):
  try:
    value = params.get(key)
  except Exception:
    return ""
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="ignore").strip()
  return str(value or "").strip()


def _service_flag(sm, attribute, service, default=False):
  try:
    values = getattr(sm, attribute)
    return bool(values.get(service, default))
  except (AttributeError, TypeError):
    return default


def capture_tmux_metadata(params, sm, now=None):
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

  segment = None
  if _service_flag(sm, "seen", "roadEncodeIdx") and _service_flag(sm, "valid", "roadEncodeIdx", default=True):
    try:
      segment_num = int(sm["roadEncodeIdx"].segmentNum)
      if segment_num >= 0:
        segment = str(segment_num)
    except (AttributeError, KeyError, TypeError, ValueError):
      pass

  return {
    "route": route,
    "segment": segment,
    "captured_at": captured_at,
  }
