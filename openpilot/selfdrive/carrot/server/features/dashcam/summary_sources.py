import os
from urllib.parse import quote

from aiohttp import web

from .catalog import source_qlog, source_rlog
from .paths import segment_dir, segment_index


def _compression(name: str) -> str:
  if name.endswith(".zst"):
    return "zstd"
  if name.endswith(".bz2"):
    return "bzip2"
  return "none"


def _segment_source(segment: str) -> dict | None:
  path = segment_dir(segment)
  # qlog retains the services needed by the summary at reduced frequency and is
  # substantially cheaper for the browser to transfer, decompress, and scan.
  kind = "qlog"
  try:
    log_path, log_name = source_qlog(path)
  except web.HTTPNotFound:
    kind = "rlog"
    try:
      log_path, log_name = source_rlog(path)
    except web.HTTPNotFound:
      return None
  stat = os.stat(log_path, follow_symlinks=False)
  encoded = quote(segment, safe="")
  return {
    "segment": segment,
    "index": segment_index(segment),
    "kind": kind,
    "name": log_name,
    "size": stat.st_size,
    "modifiedMs": stat.st_mtime_ns // 1_000_000,
    "compression": _compression(log_name),
    "url": f"/api/dashcam/replay-source/{encoded}/{kind}",
  }


def build_route_summary_source(route: str, segments: list[str]) -> dict:
  """Build cheap browser-analysis metadata; never parse or decompress logs."""
  sources = []
  skipped = 0
  for segment in sorted(segments, key=lambda value: (segment_index(value), value)):
    source = _segment_source(segment)
    if source is None:
      skipped += 1
      continue
    sources.append(source)
  return {
    "ok": True,
    "mode": "client-worker",
    "schemaVersion": 2,
    "route": route,
    "segments": sources,
    "segmentCount": len(segments),
    "skippedSegments": skipped,
  }
