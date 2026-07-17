from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from collections import OrderedDict
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

from ...config import DASHCAM_CACHE_DIR


INDEX_VERSION = 2
INDEX_DIRECTORY = os.path.join(DASHCAM_CACHE_DIR, "replay_index")
INDEX_PATH = os.path.join(INDEX_DIRECTORY, "segments.sqlite3")
_schema_lock = threading.Lock()
_schema_ready = False
_lock_guard = threading.Lock()


@dataclass(slots=True)
class _LockEntry:
  lock: threading.Lock
  users: int = 0


_segment_locks: OrderedDict[str, _LockEntry] = OrderedDict()


def _connection() -> sqlite3.Connection:
  os.makedirs(INDEX_DIRECTORY, exist_ok=True)
  connection = sqlite3.connect(INDEX_PATH, timeout=2.0)
  connection.execute("PRAGMA journal_mode=WAL")
  connection.execute("PRAGMA synchronous=NORMAL")
  connection.execute("PRAGMA temp_store=MEMORY")
  connection.execute("PRAGMA busy_timeout=2000")
  return connection


def _ensure_schema() -> None:
  global _schema_ready
  if _schema_ready:
    return
  with _schema_lock:
    if _schema_ready:
      return
    with _connection() as connection:
      connection.executescript("""
        CREATE TABLE IF NOT EXISTS replay_segments (
          segment TEXT PRIMARY KEY,
          index_version INTEGER NOT NULL,
          fingerprint TEXT NOT NULL,
          source TEXT NOT NULL,
          schema_commit TEXT NOT NULL,
          schema_branch TEXT NOT NULL,
          software_version TEXT NOT NULL,
          duration_ms INTEGER NOT NULL,
          raw_event_count INTEGER NOT NULL,
          valid_event_count INTEGER NOT NULL,
          invalid_event_count INTEGER NOT NULL,
          parse_status TEXT NOT NULL,
          parse_error TEXT NOT NULL,
          sync_mode TEXT NOT NULL,
          updated_at REAL NOT NULL,
          schema_json TEXT NOT NULL,
          stats_json TEXT NOT NULL DEFAULT '{}'
        );
        CREATE TABLE IF NOT EXISTS replay_services (
          segment TEXT NOT NULL,
          service TEXT NOT NULL,
          message_count INTEGER NOT NULL,
          valid_count INTEGER NOT NULL,
          invalid_count INTEGER NOT NULL,
          first_mono_ns TEXT NOT NULL,
          last_mono_ns TEXT NOT NULL,
          observed_hz REAL NOT NULL,
          nominal_hz REAL NOT NULL,
          coverage_pct REAL,
          max_gap_ms REAL NOT NULL,
          large_gap_count INTEGER NOT NULL,
          should_log INTEGER,
          qlog_decimation INTEGER,
          PRIMARY KEY (segment, service),
          FOREIGN KEY (segment) REFERENCES replay_segments(segment) ON DELETE CASCADE
        );
        CREATE INDEX IF NOT EXISTS replay_services_service_idx
          ON replay_services(service, segment);
      """)
      segment_columns = {row[1] for row in connection.execute("PRAGMA table_info(replay_segments)")}
      if "stats_json" not in segment_columns:
        connection.execute("ALTER TABLE replay_segments ADD COLUMN stats_json TEXT NOT NULL DEFAULT '{}'")
    _schema_ready = True


@contextmanager
def segment_build_lock(segment: str) -> Iterator[None]:
  with _lock_guard:
    entry = _segment_locks.get(segment)
    if entry is None:
      entry = _LockEntry(threading.Lock())
      _segment_locks[segment] = entry
    entry.users += 1
    _segment_locks.move_to_end(segment)
  entry.lock.acquire()
  try:
    yield
  finally:
    entry.lock.release()
    with _lock_guard:
      entry.users = max(0, entry.users - 1)
      # Keep recent locks, but never evict a lock that another caller references.
      if len(_segment_locks) > 128:
        for key, candidate in list(_segment_locks.items()):
          if len(_segment_locks) <= 96:
            break
          if candidate.users == 0 and not candidate.lock.locked():
            _segment_locks.pop(key, None)


def update_replay_index(segment: str, manifest: dict[str, Any]) -> None:
  _ensure_schema()
  schema = manifest.get("recordedSchema") if isinstance(manifest.get("recordedSchema"), dict) else {}
  services = manifest.get("rawServiceStats") if isinstance(manifest.get("rawServiceStats"), dict) else {}
  raw_stats = {key: value for key, value in manifest.items() if key.startswith("raw")}
  now = time.monotonic()
  with _connection() as connection:
    connection.execute("PRAGMA foreign_keys=ON")
    connection.execute("""
      INSERT INTO replay_segments (
        segment, index_version, fingerprint, source, schema_commit, schema_branch,
        software_version, duration_ms, raw_event_count, valid_event_count,
        invalid_event_count, parse_status, parse_error, sync_mode, updated_at, schema_json,
        stats_json
      ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
      ON CONFLICT(segment) DO UPDATE SET
        index_version=excluded.index_version,
        fingerprint=excluded.fingerprint,
        source=excluded.source,
        schema_commit=excluded.schema_commit,
        schema_branch=excluded.schema_branch,
        software_version=excluded.software_version,
        duration_ms=excluded.duration_ms,
        raw_event_count=excluded.raw_event_count,
        valid_event_count=excluded.valid_event_count,
        invalid_event_count=excluded.invalid_event_count,
        parse_status=excluded.parse_status,
        parse_error=excluded.parse_error,
        sync_mode=excluded.sync_mode,
        updated_at=excluded.updated_at,
        schema_json=excluded.schema_json,
        stats_json=excluded.stats_json
    """, (
      segment,
      INDEX_VERSION,
      str(manifest.get("fingerprint") or ""),
      str(manifest.get("source") or ""),
      str(schema.get("gitCommit") or ""),
      str(schema.get("gitBranch") or ""),
      str(schema.get("version") or ""),
      int(manifest.get("rawDurationMs") or manifest.get("durationMs") or 0),
      int(manifest.get("rawEventCount") or 0),
      int(manifest.get("rawValidEventCount") or 0),
      int(manifest.get("rawInvalidEventCount") or 0),
      str(manifest.get("rawParseStatus") or "unknown"),
      str(manifest.get("rawParseError") or ""),
      str(manifest.get("syncMode") or ""),
      now,
      json.dumps(schema, ensure_ascii=False, separators=(",", ":")),
      json.dumps(raw_stats, ensure_ascii=False, separators=(",", ":")),
    ))
    connection.execute("DELETE FROM replay_services WHERE segment = ?", (segment,))
    for service, stats in services.items():
      if not isinstance(stats, dict):
        continue
      connection.execute("""
        INSERT INTO replay_services (
          segment, service, message_count, valid_count, invalid_count,
          first_mono_ns, last_mono_ns, observed_hz, nominal_hz, coverage_pct,
          max_gap_ms, large_gap_count, should_log, qlog_decimation
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
      """, (
        segment,
        str(service),
        int(stats.get("count") or 0),
        int(stats.get("validCount") or 0),
        int(stats.get("invalidCount") or 0),
        str(stats.get("firstMonoTimeNanos") or ""),
        str(stats.get("lastMonoTimeNanos") or ""),
        float(stats.get("observedHz") or 0.0),
        float(stats.get("nominalHz") or 0.0),
        float(stats["coveragePct"]) if stats.get("coveragePct") is not None else None,
        float(stats.get("maxGapMs") or 0.0),
        int(stats.get("largeGapCount") or 0),
        None if stats.get("shouldLog") is None else int(bool(stats.get("shouldLog"))),
        int(stats["qlogDecimation"]) if stats.get("qlogDecimation") is not None else None,
      ))


def indexed_segment_stats(segment: str, fingerprint: str = "") -> dict[str, Any] | None:
  _ensure_schema()
  with _connection() as connection:
    connection.row_factory = sqlite3.Row
    row = connection.execute("SELECT * FROM replay_segments WHERE segment = ?", (segment,)).fetchone()
    if (
      row is None
      or int(row["index_version"]) != INDEX_VERSION
      or (fingerprint and row["fingerprint"] != fingerprint)
    ):
      return None
    services = connection.execute(
      "SELECT * FROM replay_services WHERE segment = ? ORDER BY service", (segment,),
    ).fetchall()
  try:
    raw_stats = json.loads(row["stats_json"] or "{}")
  except Exception:
    raw_stats = {}
  response = {
    "segment": segment,
    "fingerprint": row["fingerprint"],
    "source": row["source"],
    "recordedSchema": json.loads(row["schema_json"] or "{}"),
    "rawDurationMs": row["duration_ms"],
    "rawEventCount": row["raw_event_count"],
    "rawValidEventCount": row["valid_event_count"],
    "rawInvalidEventCount": row["invalid_event_count"],
    "rawParseStatus": row["parse_status"],
    "rawParseError": row["parse_error"],
    "syncMode": row["sync_mode"],
    "rawServiceStats": {
      item["service"]: {
        "count": item["message_count"],
        "validCount": item["valid_count"],
        "invalidCount": item["invalid_count"],
        "firstMonoTimeNanos": item["first_mono_ns"],
        "lastMonoTimeNanos": item["last_mono_ns"],
        "observedHz": item["observed_hz"],
        "nominalHz": item["nominal_hz"],
        "coveragePct": item["coverage_pct"],
        "maxGapMs": item["max_gap_ms"],
        "largeGapCount": item["large_gap_count"],
        "shouldLog": None if item["should_log"] is None else bool(item["should_log"]),
        "qlogDecimation": item["qlog_decimation"],
      }
      for item in services
    },
  }
  response.update(raw_stats)
  return response
