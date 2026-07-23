from __future__ import annotations

import asyncio
import json
import os
import struct
import threading
import warnings
from collections import OrderedDict
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any
from urllib.parse import quote

from aiohttp import web

from openpilot.selfdrive.carrot.realtime.compact_state import (
  CARROT_STATE_SERVICES,
  COMPACT_BATCH_WINDOW_SECONDS,
  compact_service_interval,
  encode_carrot_state_batch,
  encode_carrot_state_frame,
)

from .paths import cache_path, safe_segment, segment_dir, segment_index
from .replay_events import ReplayEventIndexer, event_index_metadata
from .replay_index import indexed_segment_stats, segment_build_lock, update_replay_index
from .replay_query import QueryCancelled, query_field_file, query_snapshot_file
from .replay_schema import field_definition, schema_response, service_definition
from .replay_stats import ReplayScanStats

try:
  from openpilot.selfdrive.carrot.realtime.compact_state_pyx import encode_frame as encode_compact_frame_native
except ImportError:
  encode_compact_frame_native = None


REPLAY_CACHE_VERSION = 8
REPLAY_TIMELINE_VERSION = 3
REPLAY_TIMELINE_MAGIC = b"CVR1"
REPLAY_TIMELINE_HEADER = struct.Struct("<4sHHII")
REPLAY_TIMELINE_RECORD = struct.Struct("<II")
REPLAY_MAX_DURATION_MS = 3 * 60 * 1000
RLOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog")


@dataclass(slots=True)
class ReplaySample:
  service: str
  log_mono_ns: int
  frame_id: int | None
  frame: bytes


@dataclass(slots=True)
class CameraIndex:
  log_mono_ns: int
  timestamp_sof_ns: int
  frame_id: int
  segment_num: int
  segment_id: int


def _safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return default


def _event_bytes(event: Any) -> bytes:
  builder_method = getattr(event, "as_builder", None)
  builder = builder_method() if callable(builder_method) else event
  return builder.to_bytes()


def _compact_frame(event: Any, service: str, sequence: int) -> bytes:
  if encode_compact_frame_native is not None:
    try:
      return encode_compact_frame_native(service, _event_bytes(event), sequence)
    except Exception:
      pass
  return encode_carrot_state_frame(service, getattr(event, service), sequence)


def _frame_id(event: Any, service: str) -> int | None:
  if service not in ("modelV2", "roadCameraState"):
    return None
  value = _safe_int(getattr(getattr(event, service), "frameId", -1), -1)
  return value if value >= 0 else None


def _camera_index(event: Any) -> CameraIndex:
  value = event.qRoadEncodeIdx
  return CameraIndex(
    log_mono_ns=_safe_int(getattr(event, "logMonoTime", 0)),
    timestamp_sof_ns=_safe_int(getattr(value, "timestampSof", 0)),
    frame_id=_safe_int(getattr(value, "frameId", 0)),
    segment_num=_safe_int(getattr(value, "segmentNum", -1), -1),
    segment_id=_safe_int(getattr(value, "segmentId", 0)),
  )


def _select_camera_indexes(indexes: list[CameraIndex], expected_segment: int) -> list[CameraIndex]:
  matching = [item for item in indexes if item.segment_num == expected_segment]
  selected = matching or indexes
  selected.sort(key=lambda item: (item.segment_id, item.log_mono_ns))
  return selected


def build_replay_timeline(events: Iterable[Any], expected_segment: int) -> tuple[bytes, dict[str, Any]]:
  selected_events: dict[str, dict[int, Any]] = {service: {} for service in CARROT_STATE_SERVICES}
  camera_indexes: list[CameraIndex] = []
  scan_stats = ReplayScanStats()
  event_indexer = ReplayEventIndexer()
  iterator = iter(events)

  while True:
    try:
      event = next(iterator)
    except StopIteration:
      break
    except Exception as exc:
      scan_stats.mark_partial(exc)
      break
    try:
      service = event.which()
    except Exception:
      continue
    scan_stats.observe(event, service)
    event_indexer.ingest(event, service)
    if service == "qRoadEncodeIdx":
      camera_indexes.append(_camera_index(event))
      continue
    if service not in selected_events:
      continue
    if not bool(getattr(event, "valid", True)):
      continue
    log_mono_ns = _safe_int(getattr(event, "logMonoTime", 0))
    if log_mono_ns <= 0:
      continue
    interval_ns = max(1, int(compact_service_interval(service) * 1_000_000_000))
    selected_events[service][log_mono_ns // interval_ns] = event

  selected_indexes = _select_camera_indexes(camera_indexes, expected_segment)
  if selected_indexes:
    base_log_mono_ns = selected_indexes[0].log_mono_ns
    timestamp_base_ns = selected_indexes[0].timestamp_sof_ns
  else:
    first_times = [
      _safe_int(getattr(event, "logMonoTime", 0))
      for service_events in selected_events.values()
      for event in service_events.values()
    ]
    if not first_times:
      raise ValueError("rlog has no Carrot Vision display data")
    base_log_mono_ns = min(first_times)
    timestamp_base_ns = 0

  frame_video_time_ms: dict[int, int] = {}
  camera_duration_ms = 0
  for item in selected_indexes:
    if timestamp_base_ns > 0 and item.timestamp_sof_ns >= timestamp_base_ns:
      video_time_ms = round((item.timestamp_sof_ns - timestamp_base_ns) / 1_000_000)
    else:
      video_time_ms = round((item.log_mono_ns - base_log_mono_ns) / 1_000_000)
    if 0 <= video_time_ms <= REPLAY_MAX_DURATION_MS:
      frame_video_time_ms[item.frame_id] = video_time_ms
      camera_duration_ms = max(camera_duration_ms, video_time_ms)

  sequence: dict[str, int] = {}
  samples: list[ReplaySample] = []
  service_counts: dict[str, int] = {}
  ordered_events = sorted(
    (
      event
      for service_events in selected_events.values()
      for event in service_events.values()
    ),
    key=lambda event: _safe_int(getattr(event, "logMonoTime", 0)),
  )
  for event in ordered_events:
    service = event.which()
    next_sequence = (sequence.get(service, 0) + 1) & 0xffff
    frame = _compact_frame(event, service, next_sequence)
    sequence[service] = next_sequence
    service_counts[service] = service_counts.get(service, 0) + 1
    samples.append(ReplaySample(
      service=service,
      log_mono_ns=_safe_int(getattr(event, "logMonoTime", 0)),
      frame_id=_frame_id(event, service),
      frame=frame,
    ))

  batch_window_ms = max(1, round(COMPACT_BATCH_WINDOW_SECONDS * 1000))
  batches: dict[int, OrderedDict[str, tuple[int, bytes]]] = {}
  last_sample_ms = 0
  for sample in samples:
    video_time_ms = round((sample.log_mono_ns - base_log_mono_ns) / 1_000_000)
    if sample.frame_id is not None and sample.frame_id in frame_video_time_ms:
      video_time_ms = frame_video_time_ms[sample.frame_id]
    video_time_ms = max(0, min(REPLAY_MAX_DURATION_MS, video_time_ms))
    last_sample_ms = max(last_sample_ms, video_time_ms)
    bucket = video_time_ms // batch_window_ms
    pending = batches.setdefault(bucket, OrderedDict())
    pending[sample.service] = (video_time_ms, sample.frame)
    pending.move_to_end(sample.service)

  duration_ms = max(camera_duration_ms, last_sample_ms)
  event_index = event_indexer.finalize(base_log_mono_ns, duration_ms)
  records: list[tuple[int, bytes]] = []
  for bucket in sorted(batches):
    pending = batches[bucket]
    record_time_ms = max(item[0] for item in pending.values())
    records.append((record_time_ms, encode_carrot_state_batch(tuple(item[1] for item in pending.values()))))

  out = bytearray(REPLAY_TIMELINE_HEADER.pack(
    REPLAY_TIMELINE_MAGIC,
    REPLAY_TIMELINE_VERSION,
    0,
    duration_ms,
    len(records),
  ))
  for time_ms, payload in records:
    out.extend(REPLAY_TIMELINE_RECORD.pack(time_ms, len(payload)))
    out.extend(payload)

  metadata = {
    "durationMs": duration_ms,
    "recordCount": len(records),
    "sampleCount": len(samples),
    "services": service_counts,
    "cameraFrameCount": len(selected_indexes),
    "syncMode": "camera-frame" if frame_video_time_ms else "log-time",
    **event_index_metadata(event_index),
    **scan_stats.as_dict(),
  }
  return bytes(out), metadata


def _rlog_source(segment_path: str) -> tuple[str, str]:
  for name in RLOG_NAMES:
    path = os.path.join(segment_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, name
  raise web.HTTPNotFound(text="rlog not found")


def _source_fingerprint(path: str) -> str:
  stat = os.stat(path, follow_symlinks=False)
  return f"{REPLAY_CACHE_VERSION}:{stat.st_size}:{stat.st_mtime_ns}"


def _read_cached_manifest(path: str, fingerprint: str, timeline_path: str) -> dict[str, Any] | None:
  try:
    with open(path, encoding="utf-8") as file:
      manifest = json.load(file)
    if manifest.get("fingerprint") != fingerprint:
      return None
    if not os.path.isfile(timeline_path) or os.path.getsize(timeline_path) <= REPLAY_TIMELINE_HEADER.size:
      return None
    return manifest
  except Exception:
    return None


def _atomic_write(path: str, data: bytes | str) -> None:
  temp_path = f"{path}.{os.getpid()}.tmp"
  mode = "wb" if isinstance(data, bytes) else "w"
  kwargs = {} if isinstance(data, bytes) else {"encoding": "utf-8"}
  with open(temp_path, mode, **kwargs) as file:
    file.write(data)
  os.replace(temp_path, path)


def prepare_replay(segment: str) -> tuple[dict[str, Any], str]:
  segment = safe_segment(segment)
  segment_path = segment_dir(segment)
  rlog_path, rlog_name = _rlog_source(segment_path)
  fingerprint = _source_fingerprint(rlog_path)
  timeline_path = cache_path("replay", segment, ".cvr")
  manifest_path = cache_path("replay", segment, ".json")

  with segment_build_lock(segment):
    cached = _read_cached_manifest(manifest_path, fingerprint, timeline_path)
    if cached is not None:
      try:
        if indexed_segment_stats(segment, fingerprint) is None:
          update_replay_index(segment, cached)
      except Exception:
        pass
      return cached, timeline_path

    from openpilot.tools.lib.logreader import LogReader

    with warnings.catch_warnings(record=True) as reader_warnings:
      warnings.simplefilter("always", RuntimeWarning)
      timeline, metadata = build_replay_timeline(
        LogReader(rlog_path, sort_by_time=True, only_union_types=True),
        expected_segment=segment_index(segment),
      )
    if reader_warnings and metadata.get("rawParseStatus") == "complete":
      metadata["rawParseStatus"] = "partial"
      metadata["rawParseError"] = str(reader_warnings[0].message)[:320]
    _atomic_write(timeline_path, timeline)
    manifest = {
      "ok": True,
      "version": REPLAY_CACHE_VERSION,
      "segment": segment,
      "source": rlog_name,
      "fingerprint": fingerprint,
      "videoUrl": f"/api/dashcam/video/{quote(segment, safe='')}",
      "thumbnailUrl": f"/api/dashcam/thumbnail/{quote(segment, safe='')}",
      "timelineUrl": f"/api/dashcam/replay/{quote(segment, safe='')}/timeline",
      "timelineBytes": len(timeline),
      **metadata,
    }
    _atomic_write(manifest_path, json.dumps(manifest, ensure_ascii=False, separators=(",", ":")))
    try:
      update_replay_index(segment, manifest)
    except Exception:
      pass
    return manifest, timeline_path


async def api_dashcam_replay_manifest(request: web.Request) -> web.Response:
  try:
    segment = request.match_info.get("segment", "")
    manifest, _ = await asyncio.to_thread(prepare_replay, segment)
    public_manifest = {key: value for key, value in manifest.items() if key != "fingerprint"}
    return web.json_response(public_manifest)
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def api_dashcam_replay_timeline(request: web.Request) -> web.StreamResponse:
  try:
    segment = request.match_info.get("segment", "")
    _, timeline_path = await asyncio.to_thread(prepare_replay, segment)
    return web.FileResponse(timeline_path, headers={
      "Content-Type": "application/octet-stream",
      "Cache-Control": "private, max-age=3600",
    })
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def api_dashcam_replay_stats(request: web.Request) -> web.Response:
  try:
    segment = request.match_info.get("segment", "")
    manifest, _ = await asyncio.to_thread(prepare_replay, segment)
    indexed = await asyncio.to_thread(indexed_segment_stats, segment, str(manifest.get("fingerprint") or ""))
    stats = indexed or {
      key: manifest.get(key)
      for key in (
        "segment", "source", "recordedSchema", "rawDurationMs", "rawEventCount",
        "rawValidEventCount", "rawInvalidEventCount", "rawParseStatus",
        "rawParseError", "syncMode", "rawServiceStats",
      )
    }
    return web.json_response({"ok": True, **stats})
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def api_dashcam_replay_schema(request: web.Request) -> web.Response:
  try:
    segment = request.match_info.get("segment", "")
    service_filter = str(request.query.get("service") or "").strip()
    include_fields = bool(service_filter) or str(request.query.get("all") or "") == "1"
    manifest, _ = await asyncio.to_thread(prepare_replay, segment)
    raw_services = manifest.get("rawServiceStats") if isinstance(manifest.get("rawServiceStats"), dict) else {}
    response = await asyncio.to_thread(
      schema_response,
      manifest.get("recordedSchema") if isinstance(manifest.get("recordedSchema"), dict) else {},
      set(raw_services),
      service_filter,
      include_fields,
    )
    if service_filter and not response["services"]:
      raise web.HTTPNotFound(text="service schema not found")
    return web.json_response({"ok": True, **response})
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def api_dashcam_replay_query(request: web.Request) -> web.Response:
  try:
    segment = safe_segment(request.match_info.get("segment", ""))
    try:
      body = await request.json()
    except Exception:
      body = {}
    service = str(body.get("service") or "").strip()
    path = str(body.get("fieldPath") or "").strip()
    if not service or not path:
      raise web.HTTPBadRequest(text="missing service or fieldPath")
    field_meta = field_definition(service, path)
    if field_meta is None:
      raise web.HTTPNotFound(text="field schema not found")
    try:
      max_points = max(50, min(3000, int(body.get("maxPoints") or 1500)))
    except (TypeError, ValueError):
      max_points = 1500
    def optional_float(name: str) -> float | None:
      value = body.get(name)
      if value is None or value == "":
        return None
      try:
        return float(value)
      except (TypeError, ValueError):
        raise web.HTTPBadRequest(text=f"bad {name}") from None
    start_ms = optional_float("startMs")
    end_ms = optional_float("endMs")
    if start_ms is not None and end_ms is not None and end_ms < start_ms:
      raise web.HTTPBadRequest(text="endMs precedes startMs")
    include_sensitive = body.get("includeSensitive") is True
    manifest, _ = await asyncio.to_thread(prepare_replay, segment)
    rlog_path, _rlog_name = _rlog_source(segment_dir(segment))
    try:
      base_mono_ns = int(str(manifest.get("rawFirstMonoTimeNanos") or "0"))
    except ValueError:
      base_mono_ns = 0
    cancel_event = threading.Event()
    task = asyncio.create_task(asyncio.to_thread(
      query_field_file,
      rlog_path=rlog_path,
      segment=segment,
      fingerprint=str(manifest.get("fingerprint") or ""),
      service=service,
      path=path,
      base_mono_ns=base_mono_ns,
      field_meta=field_meta,
      max_points=max_points,
      start_ms=start_ms,
      end_ms=end_ms,
      include_sensitive=include_sensitive,
      should_cancel=cancel_event.is_set,
    ))
    try:
      result = await task
    except asyncio.CancelledError:
      cancel_event.set()
      raise
    except QueryCancelled:
      raise web.HTTPRequestTimeout(text="field query cancelled") from None
    return web.json_response({"ok": True, **result})
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def api_dashcam_replay_snapshot(request: web.Request) -> web.Response:
  try:
    segment = safe_segment(request.match_info.get("segment", ""))
    try:
      body = await request.json()
    except Exception:
      body = {}
    service = str(body.get("service") or "").strip()
    if not service:
      raise web.HTTPBadRequest(text="missing service")
    definition = service_definition(service)
    if definition is None:
      raise web.HTTPNotFound(text="service schema not found")
    try:
      target_ms = max(0.0, float(body.get("timeMs") or 0.0))
    except (TypeError, ValueError):
      raise web.HTTPBadRequest(text="bad timeMs") from None
    manifest, _ = await asyncio.to_thread(prepare_replay, segment)
    target_ms = min(target_ms, float(manifest.get("rawDurationMs") or manifest.get("durationMs") or target_ms))
    rlog_path, _rlog_name = _rlog_source(segment_dir(segment))
    try:
      base_mono_ns = int(str(manifest.get("rawFirstMonoTimeNanos") or "0"))
    except ValueError:
      base_mono_ns = 0
    cancel_event = threading.Event()
    task = asyncio.create_task(asyncio.to_thread(
      query_snapshot_file,
      rlog_path=rlog_path,
      service=service,
      target_ms=target_ms,
      base_mono_ns=base_mono_ns,
      fields=definition["fields"],
      include_sensitive=body.get("includeSensitive") is True,
      should_cancel=cancel_event.is_set,
    ))
    try:
      result = await task
    except asyncio.CancelledError:
      cancel_event.set()
      raise
    except QueryCancelled:
      raise web.HTTPRequestTimeout(text="snapshot query cancelled") from None
    return web.json_response({"ok": True, **result})
  except web.HTTPException as exc:
    return web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)


def register(app: web.Application) -> None:
  app.router.add_get("/api/dashcam/replay/{segment}", api_dashcam_replay_manifest)
  app.router.add_get("/api/dashcam/replay/{segment}/timeline", api_dashcam_replay_timeline)
  app.router.add_get("/api/dashcam/replay/{segment}/stats", api_dashcam_replay_stats)
  app.router.add_get("/api/dashcam/replay/{segment}/schema", api_dashcam_replay_schema)
  app.router.add_post("/api/dashcam/replay/{segment}/query", api_dashcam_replay_query)
  app.router.add_post("/api/dashcam/replay/{segment}/snapshot", api_dashcam_replay_snapshot)
