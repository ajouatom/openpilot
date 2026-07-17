from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from openpilot.cereal.services import SERVICE_LIST


def _safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return default


def _safe_text(value: Any) -> str:
  try:
    return str(value or "")
  except Exception:
    return ""


@dataclass(slots=True)
class ServiceAccumulator:
  count: int = 0
  valid_count: int = 0
  invalid_count: int = 0
  missing_time_count: int = 0
  first_mono_ns: int = 0
  last_mono_ns: int = 0
  max_gap_ns: int = 0
  large_gap_count: int = 0

  def observe(self, mono_ns: int, valid: bool, nominal_hz: float) -> None:
    self.count += 1
    if valid:
      self.valid_count += 1
    else:
      self.invalid_count += 1

    if mono_ns <= 0:
      self.missing_time_count += 1
      return

    if self.first_mono_ns <= 0:
      self.first_mono_ns = mono_ns
    if self.last_mono_ns > 0:
      gap_ns = max(0, mono_ns - self.last_mono_ns)
      self.max_gap_ns = max(self.max_gap_ns, gap_ns)
      if nominal_hz > 0 and gap_ns > max(1_000_000_000, int(3_000_000_000 / nominal_hz)):
        self.large_gap_count += 1
    self.last_mono_ns = max(self.last_mono_ns, mono_ns)

  def as_dict(self, service: str, raw_duration_ns: int) -> dict[str, Any]:
    config = SERVICE_LIST.get(service)
    nominal_hz = float(config.frequency) if config is not None else 0.0
    span_ns = max(0, self.last_mono_ns - self.first_mono_ns)
    observed_hz = ((self.count - 1) * 1_000_000_000 / span_ns) if self.count > 1 and span_ns > 0 else 0.0
    expected_count = nominal_hz * raw_duration_ns / 1_000_000_000 if nominal_hz > 0 and raw_duration_ns > 0 else 0.0
    coverage_pct = min(100.0, self.count * 100.0 / expected_count) if expected_count > 0 else None
    return {
      "count": self.count,
      "validCount": self.valid_count,
      "invalidCount": self.invalid_count,
      "invalidPct": round(self.invalid_count * 100.0 / self.count, 4) if self.count else 0.0,
      "missingTimeCount": self.missing_time_count,
      # Keep nanosecond clocks exact in JSON/JavaScript.
      "firstMonoTimeNanos": str(self.first_mono_ns) if self.first_mono_ns > 0 else "",
      "lastMonoTimeNanos": str(self.last_mono_ns) if self.last_mono_ns > 0 else "",
      "observedHz": round(observed_hz, 4),
      "nominalHz": nominal_hz,
      "coveragePct": round(coverage_pct, 3) if coverage_pct is not None and math.isfinite(coverage_pct) else None,
      "maxGapMs": round(self.max_gap_ns / 1_000_000, 3),
      "largeGapCount": self.large_gap_count,
      "shouldLog": bool(config.should_log) if config is not None else None,
      "qlogDecimation": int(config.decimation) if config is not None and config.decimation is not None else None,
    }


@dataclass(slots=True)
class ReplayScanStats:
  services: dict[str, ServiceAccumulator] = field(default_factory=dict)
  total_count: int = 0
  valid_count: int = 0
  invalid_count: int = 0
  first_event_mono_ns: int = 0
  last_event_mono_ns: int = 0
  sentinel_start_ns: int = 0
  sentinel_end_ns: int = 0
  parse_status: str = "complete"
  parse_error: str = ""
  schema: dict[str, Any] = field(default_factory=dict)

  def observe(self, event: Any, service: str) -> None:
    mono_ns = _safe_int(getattr(event, "logMonoTime", 0))
    valid = bool(getattr(event, "valid", True))
    config = SERVICE_LIST.get(service)
    nominal_hz = float(config.frequency) if config is not None else 0.0

    self.total_count += 1
    if valid:
      self.valid_count += 1
    else:
      self.invalid_count += 1

    accumulator = self.services.setdefault(service, ServiceAccumulator())
    accumulator.observe(mono_ns, valid, nominal_hz)

    if service != "initData" and mono_ns > 0:
      if self.first_event_mono_ns <= 0:
        self.first_event_mono_ns = mono_ns
      self.first_event_mono_ns = min(self.first_event_mono_ns, mono_ns)
      self.last_event_mono_ns = max(self.last_event_mono_ns, mono_ns)

    if service == "initData" and not self.schema:
      value = getattr(event, "initData", None)
      self.schema = {
        "gitCommit": _safe_text(getattr(value, "gitCommit", "")),
        "gitBranch": _safe_text(getattr(value, "gitBranch", "")),
        "gitSrcCommit": _safe_text(getattr(value, "gitSrcCommit", "")),
        "version": _safe_text(getattr(value, "version", "")),
        "deviceType": _safe_text(getattr(value, "deviceType", "")),
        "dirty": bool(getattr(value, "dirty", False)),
        "passive": bool(getattr(value, "passive", False)),
      }
    elif service == "sentinel":
      sentinel_type = _safe_text(getattr(getattr(event, "sentinel", None), "type", ""))
      if sentinel_type in ("startOfRoute", "startOfSegment"):
        self.sentinel_start_ns = mono_ns
      elif sentinel_type in ("endOfRoute", "endOfSegment"):
        self.sentinel_end_ns = mono_ns

  def mark_partial(self, error: BaseException) -> None:
    self.parse_status = "partial"
    message = _safe_text(error).replace("\r", " ").replace("\n", " ")
    self.parse_error = f"{type(error).__name__}: {message}"[:320]

  def as_dict(self) -> dict[str, Any]:
    if self.sentinel_start_ns > 0 and self.sentinel_end_ns > self.sentinel_start_ns:
      start_ns = self.sentinel_start_ns
      end_ns = self.sentinel_end_ns
      duration_basis = "sentinel"
    else:
      start_ns = self.first_event_mono_ns
      end_ns = self.last_event_mono_ns
      duration_basis = "event-span"
    duration_ns = max(0, end_ns - start_ns)
    return {
      "rawEventCount": self.total_count,
      "rawValidEventCount": self.valid_count,
      "rawInvalidEventCount": self.invalid_count,
      "rawInvalidPct": round(self.invalid_count * 100.0 / self.total_count, 4) if self.total_count else 0.0,
      "rawDurationMs": round(duration_ns / 1_000_000),
      "rawDurationBasis": duration_basis,
      "rawFirstMonoTimeNanos": str(start_ns) if start_ns > 0 else "",
      "rawLastMonoTimeNanos": str(end_ns) if end_ns > 0 else "",
      "rawParseStatus": self.parse_status,
      "rawParseError": self.parse_error,
      "recordedSchema": dict(self.schema),
      "rawServiceStats": {
        service: accumulator.as_dict(service, duration_ns)
        for service, accumulator in sorted(self.services.items())
      },
    }
