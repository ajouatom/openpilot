"""Persistent status shared by the optional big-model downloader, build, and UI."""

from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any


STATUS_FILENAME = "status.json"
STATUS_SCHEMA_VERSION = 1
VALID_STATES = {
  "checking",
  "downloading",
  "verifying",
  "ready",
  "waiting_for_ignition",
  "compiling",
  "compiled",
  "error",
}


def status_path(cache_dir: Path) -> Path:
  return cache_dir / STATUS_FILENAME


def read_big_model_status(cache_dir: Path) -> dict[str, Any] | None:
  try:
    with status_path(cache_dir).open(encoding="utf-8") as f:
      value = json.load(f)
    if not isinstance(value, dict) or value.get("schema_version") != STATUS_SCHEMA_VERSION:
      return None
    if value.get("state") not in VALID_STATES:
      return None
    return value
  except (FileNotFoundError, OSError, TypeError, ValueError, json.JSONDecodeError):
    return None


def write_big_model_status(cache_dir: Path, state: str, *, model_id: str | None = None,
                           sha256: str | None = None, downloaded_bytes: int | None = None,
                           total_bytes: int | None = None, detail: str | None = None,
                           started_at: float | None = None) -> dict[str, Any]:
  if state not in VALID_STATES:
    raise ValueError(f"invalid big model status: {state}")

  now = time.time()
  previous = read_big_model_status(cache_dir) or {}
  if started_at is None:
    if previous.get("state") == state:
      started_at = float(previous.get("started_at") or now)
    else:
      started_at = now

  value: dict[str, Any] = {
    "schema_version": STATUS_SCHEMA_VERSION,
    "state": state,
    "started_at": started_at,
    "updated_at": now,
  }
  for key, item in (
    ("model_id", model_id),
    ("sha256", sha256),
    ("downloaded_bytes", downloaded_bytes),
    ("total_bytes", total_bytes),
    ("detail", detail),
  ):
    if item is not None:
      value[key] = item

  cache_dir.mkdir(parents=True, exist_ok=True)
  fd, tmp_name = tempfile.mkstemp(prefix=".status-", suffix=".json", dir=cache_dir)
  try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(value, f, separators=(",", ":"), sort_keys=True)
      f.write("\n")
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_name, status_path(cache_dir))
  finally:
    try:
      os.unlink(tmp_name)
    except FileNotFoundError:
      pass
  return value


class BigModelStatusReporter:
  """Rate-limited status writer for multi-gigabyte model transfers."""

  def __init__(self, cache_dir: Path, min_interval: float = 1.0):
    self.cache_dir = cache_dir
    self.min_interval = min_interval
    self._last_write = 0.0
    self._started_at = time.time()

  def update(self, state: str, *, force: bool = True, **values: Any) -> dict[str, Any] | None:
    now = time.monotonic()
    if not force and now - self._last_write < self.min_interval:
      return None
    self._last_write = now
    if state != "downloading":
      self._started_at = time.time()
    return write_big_model_status(self.cache_dir, state, started_at=self._started_at, **values)

  def download_progress(self, manifest: Any, downloaded_bytes: int, total_bytes: int) -> None:
    self.update(
      "downloading",
      force=downloaded_bytes >= total_bytes,
      model_id=manifest.model_id,
      sha256=manifest.sha256,
      downloaded_bytes=max(0, downloaded_bytes),
      total_bytes=max(0, total_bytes),
    )
