from __future__ import annotations

import re
from typing import Any
from urllib.parse import quote, unquote, urlsplit, urlunsplit


SEGMENT_RE = re.compile(r"^(?P<route>[A-Za-z0-9_.|-]{20})--(?P<index>\d+)$")


def public_upload_url(value: Any) -> str:
  """Quote an uploader-produced HTTP URL without changing its route structure."""
  text = str(value or "").strip()
  try:
    parsed = urlsplit(text)
  except ValueError:
    return text
  if parsed.scheme not in {"http", "https"} or not parsed.netloc:
    return text
  path = quote(unquote(parsed.path), safe="/:@-._~|")
  return urlunsplit((parsed.scheme, parsed.netloc, path, parsed.query, parsed.fragment))


def _segment_parts(item: dict[str, Any]) -> tuple[str, int] | None:
  segment = str(item.get("segment") or "").strip()
  match = SEGMENT_RE.fullmatch(segment)
  if not match:
    return None
  route = str(item.get("route") or match.group("route")).strip()
  if route != match.group("route"):
    return None
  try:
    index = int(item.get("segmentIndex", match.group("index")))
  except (TypeError, ValueError):
    return None
  if index != int(match.group("index")):
    return None
  return route, index


def _item_url(payload: dict[str, Any], item: dict[str, Any]) -> str:
  remote_path = str(item.get("remotePath") or "").strip()
  if not remote_path:
    base = str(payload.get("remoteBasePath") or "").strip().rstrip("/")
    segment = str(item.get("segment") or "").strip()
    if base and segment:
      remote_path = f"{base}/{segment}"
  return public_upload_url(remote_path)


def _consecutive_runs(items: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
  runs: list[list[dict[str, Any]]] = []
  current: list[dict[str, Any]] = []
  current_route = ""
  previous_index = -2
  for item in items:
    parts = _segment_parts(item)
    if parts is None:
      if len(current) > 1:
        runs.append(current)
      current = []
      current_route = ""
      previous_index = -2
      continue
    route, index = parts
    if current and route == current_route and index == previous_index + 1:
      current.append(item)
    else:
      if len(current) > 1:
        runs.append(current)
      current = [item]
      current_route = route
    previous_index = index
  if len(current) > 1:
    runs.append(current)
  return runs


def _run_url(payload: dict[str, Any], run: list[dict[str, Any]]) -> str:
  first_parts = _segment_parts(run[0])
  last_parts = _segment_parts(run[-1])
  first_url = _item_url(payload, run[0])
  if first_parts is None or last_parts is None or not first_url or "/" not in first_url:
    return ""
  route, first_index = first_parts
  _, last_index = last_parts
  prefix = first_url.rsplit("/", 1)[0]
  return public_upload_url(f"{prefix}/{route}--{first_index}:{last_index + 1}")


def upload_message_lines(payload: dict[str, Any], max_results: int | None = None) -> list[str]:
  meta = payload.get("meta") or {}
  commit = str(meta.get("commit") or "").strip()
  commit_date = meta.get("commitDate") or "unknown"
  commit_text = (
    f"[{commit}](https://github.com/ajouatom/openpilot/commit/{commit})"
    if commit and commit != "unknown"
    else "unknown"
  )
  uploaded = [item for item in payload.get("results") or [] if item.get("ok")]
  failed = [item for item in payload.get("results") or [] if not item.get("ok")]
  result_items = uploaded + failed
  visible_items = result_items if max_results is None else result_items[:max_results]
  visible_uploaded = [item for item in visible_items if item.get("ok")]
  lines = [
    "# Carrot Dashcam Upload",
    "### Upload",
    f"- Time: {payload.get('uploadedAt') or ''}",
    f"- Path: {public_upload_url(payload.get('remoteBasePath'))}",
    "### Device",
    f"- Car name: {meta.get('carName') or 'none'}",
    f"- DongleId: {meta.get('dongleId') or 'unknown'}",
    f"- Serial: {meta.get('serial') or 'unknown'}",
    f"- Branch: {meta.get('branch') or 'unknown'}",
    f"- Commit: {commit_text} ({commit_date})",
  ]

  runs = _consecutive_runs(visible_uploaded)
  if runs:
    lines.append("### Open & Analyze")
    for run in runs:
      first_parts = _segment_parts(run[0])
      last_parts = _segment_parts(run[-1])
      url = _run_url(payload, run)
      if first_parts is None or last_parts is None or not url:
        continue
      _, first_index = first_parts
      _, last_index = last_parts
      lines.append(
        f"- [Segments {first_index}–{last_index} ({len(run)} logs) · Web/Video/Tools]({url})",
      )

  lines.append("### Result")
  for item in visible_items:
    segment = str(item.get("segment") or "unknown")
    if item.get("ok"):
      url = _item_url(payload, item)
      lines.append(f"- [{segment} OK · Open]({url})" if url else f"- {segment} OK")
    else:
      error = str(item.get("error") or "").strip()
      suffix = f": {error}" if error else ""
      lines.append(f"- {segment} FAILED{suffix}")

  hidden_count = len(result_items) - len(visible_items)
  if hidden_count > 0:
    lines.append(f"- ... +{hidden_count} more")
  if not result_items:
    lines.append("- none")
  return lines


def upload_share_text(payload: dict[str, Any]) -> str:
  return "\n".join(upload_message_lines(payload)).strip()


def discord_content(payload: dict[str, Any]) -> str:
  content = "\n".join(upload_message_lines(payload, max_results=24)).strip()
  if len(content) <= 1900:
    return content

  content = "\n".join(upload_message_lines(payload, max_results=10)).strip()
  if len(content) <= 1900:
    return content

  return "\n".join(upload_message_lines(payload, max_results=3)).strip()[:1900]
