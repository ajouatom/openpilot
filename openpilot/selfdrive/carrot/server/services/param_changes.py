"""Append-only history of every settings write, with a verifiable hash chain.

Why this exists: a parameter can be changed from the web UI, by applying a
profile, by restoring a backup, or by a steering-wheel button in the driving
code. Until now nothing recorded which of those happened, so a value that
turned up different could not be explained. The log answers "who changed this,
when, and from what".

Integrity is a real concern here even though nobody is attacking the device:
the log lives on a car computer that loses power mid-drive, so a truncated or
half-written file is normal wear. Each record therefore carries the hash of the
record before it. A break in that chain marks exactly where the file stopped
being trustworthy, which is the difference between a log you can act on and one
you can only guess at.

Hashing runs here rather than in the browser because the web UI is served over
plain HTTP on the local network, so it is not a secure context and
`crypto.subtle` is unavailable. Verifying a server-held log in a client that
received it from that same server would prove nothing anyway.
"""
from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from typing import Any, Dict, List, Optional

from ..config import CARROT_FINGERPRINT_BASELINE_PATH, CARROT_PARAM_CHANGES_PATH

# Enough to cover a long session of tinkering without letting the file grow
# without bound on a device that is never garbage collected.
MAX_PARAM_CHANGE_RECORDS = 1000

# Where a write came from. Anything outside this set is recorded as "unknown"
# so a caller cannot invent a source that hides what actually happened.
PARAM_CHANGE_SOURCES = frozenset({
  "web_ui",
  "profile",
  "restore",
  "reset_defaults",
  "intro",
  "undo",
  # Something outside the web server changed the value -- in practice the
  # driving code, which writes MyDrivingMode and LongitudinalPersonality
  # straight to Params when the steering-wheel gap button is pressed. All we
  # can prove is "not us", so the label says exactly that.
  "device",
  "unknown",
})

GENESIS_HASH = "0" * 64

_write_lock = threading.Lock()


def _canonical(payload: Dict[str, Any]) -> str:
  """Stable serialization so the same record always hashes the same way."""
  return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def record_hash(record: Dict[str, Any]) -> str:
  """Hash of a record's content plus the previous record's hash."""
  body = {key: record.get(key) for key in ("ts", "name", "prev", "next", "source", "engaged")}
  body["prev_hash"] = record.get("prev_hash", GENESIS_HASH)
  return hashlib.sha256(_canonical(body).encode("utf-8")).hexdigest()


def normalize_source(source: Any) -> str:
  text = str(source or "").strip()
  return text if text in PARAM_CHANGE_SOURCES else "unknown"


def _read_lines() -> List[str]:
  try:
    with open(CARROT_PARAM_CHANGES_PATH, "r", encoding="utf-8") as f:
      return [line for line in f.read().splitlines() if line.strip()]
  except Exception:
    return []


def _parse(line: str) -> Optional[Dict[str, Any]]:
  try:
    record = json.loads(line)
  except Exception:
    return None
  return record if isinstance(record, dict) else None


def read_param_changes(limit: int = 0, name: str = "", source: str = "") -> List[Dict[str, Any]]:
  """Return records newest first, optionally narrowed to one parameter and/or
  one source (e.g. every change a profile apply made)."""
  records = [record for record in (_parse(line) for line in _read_lines()) if record is not None]
  if name:
    wanted = str(name)
    records = [record for record in records if str(record.get("name", "")) == wanted]
  if source:
    wanted_source = str(source)
    records = [record for record in records if str(record.get("source", "")) == wanted_source]
  records.reverse()
  if limit and limit > 0:
    records = records[:limit]
  return records


def last_record() -> Optional[Dict[str, Any]]:
  lines = _read_lines()
  for line in reversed(lines):
    record = _parse(line)
    if record is not None:
      return record
  return None


def append_param_change(
  name: str,
  prev: Any,
  next_value: Any,
  source: str = "web_ui",
  engaged: bool = False,
  ts: Optional[int] = None,
) -> Optional[Dict[str, Any]]:
  """Append one record. Never raises: a failed log must not fail the write."""
  key = str(name or "").strip()
  if not key:
    return None

  # Keep the drift baseline in step with our own writes, otherwise the next
  # read would report this very change a second time as if it came from
  # outside.
  note_known_value(key, next_value)

  try:
    with _write_lock:
      previous = last_record()
      record = {
        "ts": int(ts if ts is not None else time.time()),
        "name": key,
        "prev": prev,
        "next": next_value,
        "source": normalize_source(source),
        "engaged": bool(engaged),
        "prev_hash": str(previous.get("hash") or GENESIS_HASH) if previous else GENESIS_HASH,
      }
      record["hash"] = record_hash(record)

      os.makedirs(os.path.dirname(CARROT_PARAM_CHANGES_PATH), exist_ok=True)
      with open(CARROT_PARAM_CHANGES_PATH, "a", encoding="utf-8") as f:
        f.write(_canonical(record) + "\n")
      _trim_locked()
      return record
  except Exception:
    return None


def _trim_locked() -> None:
  """Drop the oldest records once the file outgrows the ring size.

  Trimming necessarily breaks the chain at the new first record, so that record
  is re-anchored to the genesis hash and the ones after it are re-linked. The
  alternative -- reporting a permanent break after the first trim -- would make
  verification useless.
  """
  lines = _read_lines()
  if len(lines) <= MAX_PARAM_CHANGE_RECORDS:
    return

  records = [record for record in (_parse(line) for line in lines) if record is not None]
  kept = records[-MAX_PARAM_CHANGE_RECORDS:]
  prev_hash = GENESIS_HASH
  for record in kept:
    record["prev_hash"] = prev_hash
    record["hash"] = record_hash(record)
    prev_hash = record["hash"]

  tmp_path = CARROT_PARAM_CHANGES_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    for record in kept:
      f.write(_canonical(record) + "\n")
  os.replace(tmp_path, CARROT_PARAM_CHANGES_PATH)


def verify_param_changes() -> Dict[str, Any]:
  """Walk the chain from the start and report the first break, if any.

  A log nobody checks is not tamper evident, so this is the counterpart that
  makes the hashes mean something.
  """
  lines = _read_lines()
  prev_hash = GENESIS_HASH
  checked = 0

  for index, line in enumerate(lines):
    record = _parse(line)
    if record is None:
      return _broken(index, checked, "record is not valid JSON")
    if str(record.get("prev_hash") or "") != prev_hash:
      return _broken(index, checked, "record does not link to the previous hash")
    if str(record.get("hash") or "") != record_hash(record):
      return _broken(index, checked, "record content does not match its hash")
    prev_hash = str(record.get("hash"))
    checked += 1

  return {"ok": True, "valid": True, "checked": checked, "broken_at": None, "reason": ""}


def _broken(index: int, checked: int, reason: str) -> Dict[str, Any]:
  return {"ok": True, "valid": False, "checked": checked, "broken_at": index, "reason": reason}


# Last value this server knows about, per parameter. Populated by our own
# writes and by every read that passes through observe_param_values().
_known_values: Dict[str, Any] = {}
_known_lock = threading.Lock()


def note_known_value(name: str, value: Any) -> None:
  with _known_lock:
    _known_values[str(name)] = value


def observe_param_values(values: Dict[str, Any], allowed: Optional[set] = None) -> int:
  """Record values that changed without the web server doing it.

  The driving code writes some parameters directly to Params, so those changes
  can never reach append_param_change() on their own. Rather than adding a
  background poller to a car computer -- or making safety-critical code depend
  on this service -- drift is picked up on the reads the web already performs.

  `allowed`, when given, restricts watching to that set of names. Callers pass
  the catalog keys so synthetic values (GitPullTime) and hardware readouts
  (DeviceType) -- which change constantly and are not settings -- never land in
  the change history.

  The consequence, stated plainly: `ts` is when the change was *noticed*, not
  when it happened. A parameter nobody looks at is not compared, which is fine,
  because a value nobody looks at is not confusing anybody either.

  Returns the number of records appended.
  """
  if not isinstance(values, dict) or not values:
    return 0

  drifted = []
  with _known_lock:
    for name, value in values.items():
      key = str(name)
      if allowed is not None and key not in allowed:
        continue
      if key not in _known_values:
        # First sighting since boot: adopt it as the baseline rather than
        # reporting the whole parameter set as changed.
        _known_values[key] = value
        continue
      if _known_values[key] != value:
        drifted.append((key, _known_values[key], value))
        _known_values[key] = value

  for key, previous, current in drifted:
    append_param_change(key, previous, current, source="device")
  return len(drifted)


def read_fingerprint_baseline() -> Optional[Dict[str, Any]]:
  """The saved reference fingerprint, or None if none has been set yet."""
  try:
    with open(CARROT_FINGERPRINT_BASELINE_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
  except Exception:
    return None
  if not isinstance(data, dict) or not str(data.get("fingerprint") or ""):
    return None
  return {"fingerprint": str(data["fingerprint"]), "ts": int(data.get("ts") or 0)}


def write_fingerprint_baseline(fingerprint: str, ts: Optional[int] = None) -> Dict[str, Any]:
  """Save the current fingerprint as the reference to compare against."""
  record = {"fingerprint": str(fingerprint), "ts": int(ts if ts is not None else time.time())}
  os.makedirs(os.path.dirname(CARROT_FINGERPRINT_BASELINE_PATH), exist_ok=True)
  tmp_path = CARROT_FINGERPRINT_BASELINE_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(record, f, ensure_ascii=False)
    f.write("\n")
  os.replace(tmp_path, CARROT_FINGERPRINT_BASELINE_PATH)
  return record


def count_changes_since(ts: int, allowed: Optional[set] = None) -> int:
  """How many distinct parameters changed at or after `ts`.

  Distinct, not raw records: pressing a segment three times is "1 setting
  changed", which is what a person means by "what's different".
  """
  names = set()
  for record in read_param_changes():
    if int(record.get("ts") or 0) < ts:
      continue
    name = str(record.get("name") or "")
    if not name or (allowed is not None and name not in allowed):
      continue
    names.add(name)
  return len(names)


def param_fingerprint(values: Dict[str, Any]) -> Dict[str, Any]:
  """Short digest of the whole parameter set.

  Comparing one short string answers "did anything change since the last known
  good state?" without diffing 165 values -- the question nobody could answer
  when settings started drifting.
  """
  clean = {str(key): values.get(key) for key in sorted(values or {})}
  digest = hashlib.sha256(_canonical(clean).encode("utf-8")).hexdigest()
  return {"fingerprint": digest[:8], "digest": digest, "count": len(clean)}
