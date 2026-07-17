from __future__ import annotations

import hashlib
import json
import math
import os
import random
import threading
import time
import warnings
from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass, field
from decimal import Decimal
from typing import Any

from .paths import cache_path
from .replay_semantics.policies import sequence_like as semantic_sequence_like


QUERY_CACHE_VERSION = 4
MAX_QUANTILE_VALUES = 50_000
MAX_CACHE_FILES = 96
MAX_CACHE_BYTES = 128 * 1024 * 1024
MAX_SNAPSHOT_DEPTH = 8
MAX_SNAPSHOT_LIST_ITEMS = 80
MAX_SNAPSHOT_OBJECT_ITEMS = 240
MAX_SNAPSHOT_TEXT_CHARS = 512
SENSITIVE_PREFIXES = (
  "route-location", "biometric", "audio", "potential-sensitive", "vehicle-identifier",
)
_query_slot = threading.Semaphore(1)


class QueryCancelled(Exception):
  pass


def _acquire_query_slot(should_cancel: Any = None) -> None:
  while not _query_slot.acquire(timeout=0.05):
    if callable(should_cancel) and should_cancel():
      raise QueryCancelled()


def _safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return default


def _components(service: str, path: str) -> tuple[bool, list[tuple[str, bool]]]:
  raw = str(path or "").strip()
  if raw == service:
    return False, []
  root_list = raw.startswith(f"{service}[]")
  if root_list:
    raw = raw[len(service) + 2:]
  elif raw.startswith(f"{service}."):
    raw = raw[len(service) + 1:]
  else:
    raise ValueError("field path does not belong to service")
  raw = raw.lstrip(".")
  tokens = []
  for part in raw.split(".") if raw else []:
    expand = part.endswith("[]")
    name = part[:-2] if expand else part
    if not name or name.startswith("_"):
      raise ValueError("bad field path")
    tokens.append((name, expand))
  return root_list, tokens


def _iterable(value: Any) -> list[Any]:
  if value is None or isinstance(value, (str, bytes, bytearray, memoryview, dict)):
    return []
  try:
    return list(value)
  except Exception:
    return []


def _extract(value: Any, tokens: list[tuple[str, bool]]) -> list[Any]:
  if not tokens:
    return [value]
  name, expand = tokens[0]
  try:
    child = getattr(value, name)
  except Exception:
    if isinstance(value, dict):
      child = value.get(name)
    else:
      return []
  if expand:
    result = []
    for item in _iterable(child):
      result.extend(_extract(item, tokens[1:]))
    return result
  return _extract(child, tokens[1:])


def extract_field_values(payload: Any, service: str, path: str) -> list[Any]:
  root_list, tokens = _components(service, path)
  if root_list:
    result = []
    for item in _iterable(payload):
      result.extend(_extract(item, tokens))
    return result
  return _extract(payload, tokens)


def _scalar(value: Any) -> tuple[str, Any]:
  if isinstance(value, bool):
    return "bool", value
  if isinstance(value, (int, float)) and not isinstance(value, bool):
    number = float(value)
    return ("number", number) if math.isfinite(number) else ("nonfinite", str(number))
  if isinstance(value, (bytes, bytearray, memoryview)):
    data = bytes(value)
    return "binary", {"length": len(data), "hash": hashlib.sha256(data).hexdigest()[:16]}
  if isinstance(value, str):
    return "text", value
  try:
    if hasattr(value, "to_dict"):
      converted = value.to_dict()
      return "object", {"keys": len(converted) if isinstance(converted, dict) else 0}
  except Exception:
    pass
  if isinstance(value, (list, tuple)):
    return "list", {"length": len(value)}
  return "text", str(value)


@dataclass(slots=True)
class NumericStats:
  count: int = 0
  invalid_count: int = 0
  minimum: float = math.inf
  maximum: float = -math.inf
  mean: float = 0.0
  m2: float = 0.0
  sum_squares: float = 0.0
  nonzero_count: int = 0
  reservoir: list[float] = field(default_factory=list)
  randomizer: random.Random = field(default_factory=lambda: random.Random(0xCA7707))

  def observe(self, value: float, valid: bool) -> None:
    self.count += 1
    if not valid:
      self.invalid_count += 1
    self.minimum = min(self.minimum, value)
    self.maximum = max(self.maximum, value)
    delta = value - self.mean
    self.mean += delta / self.count
    self.m2 += delta * (value - self.mean)
    self.sum_squares += value * value
    if value != 0.0:
      self.nonzero_count += 1
    if len(self.reservoir) < MAX_QUANTILE_VALUES:
      self.reservoir.append(value)
    else:
      index = self.randomizer.randrange(self.count)
      if index < MAX_QUANTILE_VALUES:
        self.reservoir[index] = value

  def as_dict(self, redact: bool = False) -> dict[str, Any]:
    if not self.count:
      return {"count": 0, "invalidCount": 0}
    ordered = sorted(self.reservoir)
    def percentile(ratio: float) -> float:
      return ordered[min(len(ordered) - 1, max(0, round((len(ordered) - 1) * ratio)))]
    result = {
      "count": self.count,
      "invalidCount": self.invalid_count,
      "nonDefaultCount": self.nonzero_count,
      "sampledForQuantiles": len(ordered),
    }
    if not redact:
      result.update({
        "min": self.minimum,
        "max": self.maximum,
        "mean": self.mean,
        "stddev": math.sqrt(self.m2 / self.count) if self.count else 0.0,
        "rms": math.sqrt(self.sum_squares / self.count) if self.count else 0.0,
        "p01": percentile(0.01),
        "p05": percentile(0.05),
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
      })
      if self.minimum == self.maximum:
        result["histogram"] = {
          "sampledCount": len(ordered),
          "bins": [{"min": self.minimum, "max": self.maximum, "count": len(ordered)}],
        }
      else:
        bin_count = min(16, max(4, round(math.sqrt(len(ordered)))))
        width = (self.maximum - self.minimum) / bin_count
        counts = [0] * bin_count
        for value in ordered:
          index = min(bin_count - 1, max(0, int((value - self.minimum) / width)))
          counts[index] += 1
        result["histogram"] = {
          "sampledCount": len(ordered),
          "bins": [
            {
              "min": self.minimum + index * width,
              "max": self.minimum + (index + 1) * width,
              "count": count,
            }
            for index, count in enumerate(counts)
          ],
        }
    return result


@dataclass(slots=True)
class Integer64Stats:
  count: int = 0
  invalid_count: int = 0
  nonzero_count: int = 0
  minimum: int | None = None
  maximum: int | None = None
  total: int = 0
  reservoir: list[int] = field(default_factory=list)
  randomizer: random.Random = field(default_factory=lambda: random.Random(0x64CA7707))

  def observe(self, value: int, valid: bool) -> None:
    self.count += 1
    self.invalid_count += int(not valid)
    self.nonzero_count += int(value != 0)
    self.minimum = value if self.minimum is None else min(self.minimum, value)
    self.maximum = value if self.maximum is None else max(self.maximum, value)
    self.total += value
    if len(self.reservoir) < MAX_QUANTILE_VALUES:
      self.reservoir.append(value)
    else:
      index = self.randomizer.randrange(self.count)
      if index < MAX_QUANTILE_VALUES:
        self.reservoir[index] = value

  def as_dict(self, redact: bool = False) -> dict[str, Any]:
    if not self.count:
      return {"count": 0, "invalidCount": 0}
    ordered = sorted(self.reservoir)
    def percentile(ratio: float) -> str:
      return str(ordered[min(len(ordered) - 1, max(0, round((len(ordered) - 1) * ratio)))])
    result = {
      "count": self.count,
      "invalidCount": self.invalid_count,
      "nonDefaultCount": self.nonzero_count,
      "sampledForQuantiles": len(ordered),
    }
    if redact:
      return result
    result.update({
      "min": str(self.minimum),
      "max": str(self.maximum),
      "mean": format(Decimal(self.total) / Decimal(self.count), "f"),
      "p01": percentile(0.01),
      "p05": percentile(0.05),
      "p50": percentile(0.50),
      "p95": percentile(0.95),
      "p99": percentile(0.99),
    })
    return result


def _minmax_downsample(points: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
  if len(points) <= limit:
    return points
  if limit < 4:
    return [points[0], points[-1]][:limit]
  result = [points[0]]
  interior = points[1:-1]
  bucket_count = max(1, (limit - 2) // 2)
  for bucket in range(bucket_count):
    start = math.floor(bucket * len(interior) / bucket_count)
    end = math.floor((bucket + 1) * len(interior) / bucket_count)
    values = interior[start:max(start + 1, end)]
    low = min(values, key=lambda item: item["value"])
    high = max(values, key=lambda item: item["value"])
    result.extend(sorted((low, high), key=lambda item: item["timeMs"]))
  result.append(points[-1])
  return result[:limit]


def _redacted_text(value: str) -> dict[str, Any]:
  encoded = value.encode("utf-8", errors="replace")
  return {"length": len(value), "bytes": len(encoded), "hash": hashlib.sha256(encoded).hexdigest()[:16]}


def _snapshot_value(
  value: Any,
  path: str,
  field_map: dict[str, dict[str, Any]],
  *,
  include_sensitive: bool,
  depth: int = 0,
) -> Any:
  if depth >= MAX_SNAPSHOT_DEPTH:
    return {"truncated": True, "reason": "depth"}
  if isinstance(value, dict):
    result = {}
    items = list(value.items())
    for key, child in items[:MAX_SNAPSHOT_OBJECT_ITEMS]:
      child_path = f"{path}.{key}" if path else str(key)
      result[str(key)] = _snapshot_value(
        child, child_path, field_map, include_sensitive=include_sensitive, depth=depth + 1,
      )
    if len(items) > MAX_SNAPSHOT_OBJECT_ITEMS:
      result["__truncatedItems"] = len(items) - MAX_SNAPSHOT_OBJECT_ITEMS
    return result
  if isinstance(value, (list, tuple)):
    list_path = path if path.endswith("[]") else f"{path}[]"
    result = [
      _snapshot_value(item, list_path, field_map, include_sensitive=include_sensitive, depth=depth + 1)
      for item in list(value)[:MAX_SNAPSHOT_LIST_ITEMS]
    ]
    if len(value) > MAX_SNAPSHOT_LIST_ITEMS:
      result.append({"truncated": True, "remainingItems": len(value) - MAX_SNAPSHOT_LIST_ITEMS})
    return result

  meta = field_map.get(path) or field_map.get(f"{path}[]") or {}
  privacy = str(meta.get("privacy") or "")
  sensitive = privacy.startswith(SENSITIVE_PREFIXES)
  if sensitive and not include_sensitive:
    if isinstance(value, str):
      return {"redacted": True, **_redacted_text(value)}
    if isinstance(value, (bytes, bytearray, memoryview)):
      kind, summary = _scalar(value)
      return {"redacted": True, "kind": kind, **summary}
    return {"redacted": True, "type": str(meta.get("type") or type(value).__name__)}
  if str(meta.get("type") or "").lower() in ("int64", "uint64"):
    try:
      return str(int(value))
    except Exception:
      return str(value)
  if isinstance(value, str) and len(value) > MAX_SNAPSHOT_TEXT_CHARS:
    return {
      "text": value[:MAX_SNAPSHOT_TEXT_CHARS],
      "length": len(value),
      "truncated": True,
    }
  kind, scalar = _scalar(value)
  return scalar


def query_snapshot_events(
  events: Iterable[Any],
  *,
  service: str,
  target_ms: float,
  base_mono_ns: int,
  fields: list[dict[str, Any]],
  include_sensitive: bool = False,
  should_cancel: Any = None,
) -> dict[str, Any]:
  iterator = iter(events)
  event_count = 0
  matching_event_count = 0
  best_event: Any = None
  best_time_ms = 0.0
  best_distance_ms = math.inf
  parse_status = "complete"
  parse_error = ""
  while True:
    if event_count % 256 == 0 and callable(should_cancel) and should_cancel():
      raise QueryCancelled()
    try:
      event = next(iterator)
    except StopIteration:
      break
    except Exception as exc:
      parse_status = "partial"
      parse_error = f"{type(exc).__name__}: {str(exc).replace(chr(10), ' ')}"[:320]
      break
    event_count += 1
    try:
      if event.which() != service:
        continue
    except Exception:
      continue
    matching_event_count += 1
    mono_ns = _safe_int(getattr(event, "logMonoTime", 0))
    time_ms = (mono_ns - base_mono_ns) / 1_000_000 if base_mono_ns > 0 else 0.0
    distance_ms = abs(time_ms - target_ms)
    if distance_ms < best_distance_ms:
      best_event = event
      best_time_ms = time_ms
      best_distance_ms = distance_ms

  if best_event is None:
    return {
      "service": service,
      "found": False,
      "eventCount": event_count,
      "matchingEventCount": matching_event_count,
      "parseStatus": parse_status,
      "parseError": parse_error,
    }
  payload = getattr(best_event, service)
  try:
    raw_snapshot = payload.to_dict()
  except Exception:
    raw_snapshot = payload
  field_map = {str(field.get("path") or ""): field for field in fields}
  return {
    "service": service,
    "found": True,
    "targetMs": round(target_ms, 3),
    "timeMs": round(best_time_ms, 3),
    "distanceMs": round(best_distance_ms, 3),
    "logMonoTimeNanos": str(_safe_int(getattr(best_event, "logMonoTime", 0))),
    "valid": bool(getattr(best_event, "valid", True)),
    "eventCount": event_count,
    "matchingEventCount": matching_event_count,
    "snapshot": _snapshot_value(
      raw_snapshot, service, field_map, include_sensitive=include_sensitive,
    ),
    "parseStatus": parse_status,
    "parseError": parse_error,
  }


def query_field_events(
  events: Iterable[Any],
  *,
  service: str,
  path: str,
  base_mono_ns: int,
  field_meta: dict[str, Any],
  max_points: int = 1500,
  start_ms: float | None = None,
  end_ms: float | None = None,
  include_sensitive: bool = False,
  should_cancel: Any = None,
) -> dict[str, Any]:
  max_points = max(50, min(3000, int(max_points)))
  privacy = str(field_meta.get("privacy") or "")
  redact = not include_sensitive and privacy.startswith(SENSITIVE_PREFIXES)
  numeric = NumericStats()
  integer64 = Integer64Stats()
  raw_numeric_points: list[dict[str, Any]] = []
  transitions: list[dict[str, Any]] = []
  value_counts: Counter[str] = Counter()
  event_count = 0
  matching_event_count = 0
  extracted_value_count = 0
  invalid_event_count = 0
  missing_value_count = 0
  list_lengths: list[int] = []
  value_lengths: list[int] = []
  nonfinite_count = 0
  last_signature = object()
  transition_sample_count = 0
  last_state_signature: str | None = None
  last_state_time_ms: float | None = None
  state_durations_ms: Counter[str] = Counter()
  state_transitions: Counter[str] = Counter()
  last_numeric_time_ms: float | None = None
  last_numeric_value: float | None = None
  weighted_total = 0.0
  weighted_duration_ms = 0.0
  max_abs_delta = 0.0
  max_abs_rate = 0.0
  increasing_count = 0
  decreasing_count = 0
  duplicate_count = 0
  backwards_count = 0
  sequence_like = bool(field_meta.get("sequenceLike", semantic_sequence_like(path)))
  field_type = str(field_meta.get("type") or "").lower()
  exact_integer = field_type in ("int64", "uint64", "list<int64>", "list<uint64>")
  last_integer_value: int | None = None
  integer_duplicate_count = 0
  integer_backwards_count = 0
  integer_max_step = 0
  value_kind = "unknown"
  parse_status = "complete"
  parse_error = ""
  iterator = iter(events)
  while True:
    if event_count % 256 == 0 and callable(should_cancel) and should_cancel():
      raise QueryCancelled()
    try:
      event = next(iterator)
    except StopIteration:
      break
    except Exception as exc:
      parse_status = "partial"
      parse_error = f"{type(exc).__name__}: {str(exc).replace(chr(10), ' ')}"[:320]
      break
    event_count += 1
    try:
      event_service = event.which()
    except Exception:
      continue
    if event_service != service:
      continue
    mono_ns = _safe_int(getattr(event, "logMonoTime", 0))
    time_ms = (mono_ns - base_mono_ns) / 1_000_000 if base_mono_ns > 0 else 0.0
    if start_ms is not None and time_ms < start_ms:
      continue
    if end_ms is not None and time_ms > end_ms:
      continue
    matching_event_count += 1
    valid = bool(getattr(event, "valid", True))
    if not valid:
      invalid_event_count += 1
    try:
      payload = getattr(event, service)
      values = extract_field_values(payload, service, path)
    except Exception:
      values = []
    if not values:
      if "[]" in path:
        list_lengths.append(0)
      missing_value_count += 1
      continue
    if "[]" in path:
      list_lengths.append(len(values))
    extracted_value_count += len(values)
    if exact_integer and all(isinstance(value, int) and not isinstance(value, bool) for value in values):
      integers = [int(value) for value in values]
      value_kind = "integer64-list" if len(integers) > 1 else "integer64"
      for value in integers:
        integer64.observe(value, valid)
      if redact:
        continue
      if len(integers) == 1:
        current = integers[0]
        if last_integer_value is not None:
          integer_duplicate_count += int(current == last_integer_value)
          integer_backwards_count += int(current < last_integer_value)
          integer_max_step = max(integer_max_step, abs(current - last_integer_value))
        last_integer_value = current
        display_value = str(current)
      else:
        display_value = {
          "length": len(integers),
          "min": str(min(integers)),
          "max": str(max(integers)),
        }
      signature = json.dumps(display_value, sort_keys=True, ensure_ascii=False)
      if signature != last_signature:
        transition_sample_count += 1
        if len(transitions) < 6000:
          transitions.append({"timeMs": round(time_ms, 3), "value": display_value, "valid": valid})
      last_signature = signature
      continue
    normalized = [_scalar(value) for value in values]
    kinds = {kind for kind, _value in normalized}
    if kinds <= {"number", "nonfinite"}:
      value_kind = "number-list" if len(values) > 1 else "number"
      finite = [float(value) for kind, value in normalized if kind == "number"]
      nonfinite_count += sum(1 for kind, _value in normalized if kind == "nonfinite")
      for value in finite:
        numeric.observe(value, valid)
      if finite:
        representative = finite[0] if len(finite) == 1 else sum(finite) / len(finite)
        if last_numeric_time_ms is not None and last_numeric_value is not None and time_ms >= last_numeric_time_ms:
          delta_ms = time_ms - last_numeric_time_ms
          if delta_ms > 0:
            weighted_total += last_numeric_value * delta_ms
            weighted_duration_ms += delta_ms
            delta = representative - last_numeric_value
            max_abs_delta = max(max_abs_delta, abs(delta))
            max_abs_rate = max(max_abs_rate, abs(delta) * 1000.0 / delta_ms)
            increasing_count += int(delta > 0)
            decreasing_count += int(delta < 0)
          if sequence_like:
            duplicate_count += int(representative == last_numeric_value)
            backwards_count += int(representative < last_numeric_value)
        last_numeric_time_ms = time_ms
        last_numeric_value = representative
      if finite and not redact:
        if len(finite) == 1:
          raw_numeric_points.append({"timeMs": round(time_ms, 3), "value": finite[0], "valid": valid})
        else:
          raw_numeric_points.append({
            "timeMs": round(time_ms, 3),
            "value": sum(finite) / len(finite),
            "min": min(finite),
            "max": max(finite),
            "length": len(finite),
            "valid": valid,
          })
      continue

    value_kind = next(iter(kinds)) if len(kinds) == 1 else "mixed"
    if len(normalized) > 1:
      display_value: Any = {
        "length": len(normalized),
        "kinds": sorted(kinds),
      }
    else:
      kind, display_value = normalized[0]
      if kind == "text":
        value_lengths.append(len(str(display_value)))
      elif kind == "binary" and isinstance(display_value, dict):
        value_lengths.append(int(display_value.get("length") or 0))
      if kind == "text" and redact:
        display_value = _redacted_text(str(display_value))
    signature = json.dumps(display_value, sort_keys=True, ensure_ascii=False, default=str)
    value_counts[signature] += 1
    if last_state_signature is not None and last_state_time_ms is not None and time_ms >= last_state_time_ms:
      state_durations_ms[last_state_signature] += time_ms - last_state_time_ms
      if signature != last_state_signature:
        state_transitions[f"{last_state_signature}->{signature}"] += 1
    last_state_signature = signature
    last_state_time_ms = time_ms
    if signature != last_signature:
      transition_sample_count += 1
      if len(transitions) < 6000:
        transitions.append({"timeMs": round(time_ms, 3), "value": display_value, "valid": valid})
      last_signature = signature

  stats: dict[str, Any] = {
    "eventCount": event_count,
    "matchingEventCount": matching_event_count,
    "extractedValueCount": extracted_value_count,
    "invalidEventCount": invalid_event_count,
    "missingValueCount": missing_value_count,
    "nonfiniteCount": nonfinite_count,
  }
  if numeric.count:
    numeric_stats = numeric.as_dict(redact=redact)
    if not redact:
      numeric_stats.update({
        "timeWeightedMean": weighted_total / weighted_duration_ms if weighted_duration_ms > 0 else numeric.mean,
        "weightedDurationMs": weighted_duration_ms,
        "maxAbsDelta": max_abs_delta,
        "maxAbsRatePerSecond": max_abs_rate,
        "increasingCount": increasing_count,
        "decreasingCount": decreasing_count,
      })
      if sequence_like:
        numeric_stats["sequenceQuality"] = {
          "duplicateCount": duplicate_count,
          "backwardsCount": backwards_count,
        }
    stats["numeric"] = numeric_stats
  if integer64.count:
    integer_stats = integer64.as_dict(redact=redact)
    if not redact:
      integer_stats["sequenceQuality"] = {
        "duplicateCount": integer_duplicate_count,
        "backwardsCount": integer_backwards_count,
        "maxStep": str(integer_max_step),
      }
    stats["integer64"] = integer_stats
  if list_lengths:
    stats["listLength"] = {
      "count": len(list_lengths),
      "min": min(list_lengths),
      "max": max(list_lengths),
      "mean": sum(list_lengths) / len(list_lengths),
    }
  if value_counts:
    stats["distinctValueCount"] = len(value_counts)
    stats["topValueCounts"] = [
      {"signature": signature[:240], "count": count}
      for signature, count in value_counts.most_common(20)
    ]
    stats["stateDurationsMs"] = [
      {"signature": signature[:240], "durationMs": round(duration, 3)}
      for signature, duration in state_durations_ms.most_common(20)
    ]
    stats["stateTransitions"] = [
      {"transition": transition[:480], "count": count}
      for transition, count in state_transitions.most_common(40)
    ]
  if value_lengths:
    stats["valueLength"] = {
      "count": len(value_lengths),
      "min": min(value_lengths),
      "max": max(value_lengths),
      "mean": sum(value_lengths) / len(value_lengths),
    }
  return {
    "service": service,
    "field": dict(field_meta),
    "kind": value_kind,
    "redacted": redact,
    "stats": stats,
    "series": _minmax_downsample(raw_numeric_points, max_points) if raw_numeric_points else transitions[:max_points],
    "rawSeriesCount": len(raw_numeric_points) if raw_numeric_points else transition_sample_count,
    "returnedSeriesCount": min(max_points, len(raw_numeric_points) if raw_numeric_points else len(transitions)),
    "downsample": (
      "minmax-envelope" if len(raw_numeric_points) > max_points
      else "state-transition-cap" if not raw_numeric_points and transition_sample_count > max_points
      else "none"
    ),
    "parseStatus": parse_status,
    "parseError": parse_error,
  }


def _cache_file(
  segment: str,
  fingerprint: str,
  service: str,
  path: str,
  max_points: int,
  field_meta: dict[str, Any],
) -> str:
  key = json.dumps([
    QUERY_CACHE_VERSION,
    segment,
    fingerprint,
    service,
    path,
    max_points,
    str(field_meta.get("type") or ""),
    str(field_meta.get("privacy") or ""),
    int(field_meta.get("semanticVersion") or 0),
    int(field_meta.get("statisticsVersion") or 0),
    str(field_meta.get("statisticsPolicy") or ""),
    field_meta.get("sequenceLike") is True,
  ], separators=(",", ":"))
  return cache_path("replay_field", key, ".json")


def _prune_cache(directory: str) -> None:
  try:
    files = sorted(
      (entry for entry in os.scandir(directory) if entry.is_file() and entry.name.endswith(".json")),
      key=lambda entry: entry.stat().st_mtime,
      reverse=True,
    )
    total = 0
    for index, entry in enumerate(files):
      size = entry.stat().st_size
      total += size
      if index >= MAX_CACHE_FILES or total > MAX_CACHE_BYTES:
        try:
          os.unlink(entry.path)
        except OSError:
          pass
  except OSError:
    pass


def _read_query_cache(cache_file: str, fingerprint: str) -> dict[str, Any] | None:
  try:
    with open(cache_file, encoding="utf-8") as file:
      cached = json.load(file)
    if cached.get("fingerprint") != fingerprint or cached.get("cacheVersion") != QUERY_CACHE_VERSION:
      return None
    os.utime(cache_file, None)
    return {**cached, "cacheHit": True}
  except Exception:
    return None


def query_field_file(
  *,
  rlog_path: str,
  segment: str,
  fingerprint: str,
  service: str,
  path: str,
  base_mono_ns: int,
  field_meta: dict[str, Any],
  max_points: int,
  start_ms: float | None,
  end_ms: float | None,
  include_sensitive: bool,
  should_cancel: Any = None,
) -> dict[str, Any]:
  use_cache = not include_sensitive and start_ms is None and end_ms is None
  cache_file = _cache_file(segment, fingerprint, service, path, max_points, field_meta)
  if use_cache:
    cached = _read_query_cache(cache_file, fingerprint)
    if cached is not None:
      return cached

  _acquire_query_slot(should_cancel)

  try:
    if callable(should_cancel) and should_cancel():
      raise QueryCancelled()
    if use_cache:
      cached = _read_query_cache(cache_file, fingerprint)
      if cached is not None:
        return cached
    from openpilot.tools.lib.logreader import LogReader

    started = time.monotonic()
    with warnings.catch_warnings(record=True) as reader_warnings:
      warnings.simplefilter("always", RuntimeWarning)
      result = query_field_events(
        LogReader(rlog_path, sort_by_time=False, only_union_types=True),
        service=service,
        path=path,
        base_mono_ns=base_mono_ns,
        field_meta=field_meta,
        max_points=max_points,
        start_ms=start_ms,
        end_ms=end_ms,
        include_sensitive=include_sensitive,
        should_cancel=should_cancel,
      )
    if reader_warnings and result.get("parseStatus") == "complete":
      result["parseStatus"] = "partial"
      result["parseError"] = str(reader_warnings[0].message)[:320]
  finally:
    _query_slot.release()
  result.update({
    "cacheVersion": QUERY_CACHE_VERSION,
    "fingerprint": fingerprint,
    "segment": segment,
    "elapsedMs": round((time.monotonic() - started) * 1000, 2),
    "cacheHit": False,
  })
  if use_cache:
    temp = f"{cache_file}.{os.getpid()}.tmp"
    try:
      with open(temp, "w", encoding="utf-8") as file:
        json.dump(result, file, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
      os.replace(temp, cache_file)
      _prune_cache(os.path.dirname(cache_file))
    except Exception:
      try:
        os.unlink(temp)
      except OSError:
        pass
  return result


def query_snapshot_file(
  *,
  rlog_path: str,
  service: str,
  target_ms: float,
  base_mono_ns: int,
  fields: list[dict[str, Any]],
  include_sensitive: bool,
  should_cancel: Any = None,
) -> dict[str, Any]:
  _acquire_query_slot(should_cancel)
  try:
    if callable(should_cancel) and should_cancel():
      raise QueryCancelled()
    from openpilot.tools.lib.logreader import LogReader
    started = time.monotonic()
    with warnings.catch_warnings(record=True) as reader_warnings:
      warnings.simplefilter("always", RuntimeWarning)
      result = query_snapshot_events(
        LogReader(rlog_path, sort_by_time=False, only_union_types=True),
        service=service,
        target_ms=target_ms,
        base_mono_ns=base_mono_ns,
        fields=fields,
        include_sensitive=include_sensitive,
        should_cancel=should_cancel,
      )
    if reader_warnings and result.get("parseStatus") == "complete":
      result["parseStatus"] = "partial"
      result["parseError"] = str(reader_warnings[0].message)[:320]
    result["elapsedMs"] = round((time.monotonic() - started) * 1000, 2)
    return result
  finally:
    _query_slot.release()
