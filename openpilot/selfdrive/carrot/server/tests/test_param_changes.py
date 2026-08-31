import json

import pytest

from openpilot.selfdrive.carrot.server.services import param_changes


@pytest.fixture(autouse=True)
def log_path(tmp_path, monkeypatch):
  path = tmp_path / "state" / "param_changes.jsonl"
  monkeypatch.setattr(param_changes, "CARROT_PARAM_CHANGES_PATH", str(path))
  return path


def read_records(path):
  return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def test_missing_log_reads_as_empty_and_verifies_clean(log_path):
  assert param_changes.read_param_changes() == []
  assert param_changes.verify_param_changes() == {
    "ok": True, "valid": True, "checked": 0, "broken_at": None, "reason": "",
  }


def test_append_records_the_before_and_after_value(log_path):
  record = param_changes.append_param_change("ApplyModelSpeed", 0, -1, source="web_ui", engaged=True)

  assert record["name"] == "ApplyModelSpeed"
  assert record["prev"] == 0
  assert record["next"] == -1
  assert record["source"] == "web_ui"
  assert record["engaged"] is True
  assert record["prev_hash"] == param_changes.GENESIS_HASH
  assert len(record["hash"]) == 64
  assert len(read_records(log_path)) == 1


def test_records_link_into_a_chain(log_path):
  first = param_changes.append_param_change("A", 1, 2)
  second = param_changes.append_param_change("B", 3, 4)

  assert first["prev_hash"] == param_changes.GENESIS_HASH
  assert second["prev_hash"] == first["hash"]
  assert param_changes.verify_param_changes()["valid"] is True


def test_unknown_sources_are_not_silently_accepted(log_path):
  assert param_changes.append_param_change("A", 1, 2, source="made_up")["source"] == "unknown"
  assert param_changes.append_param_change("B", 1, 2, source="")["source"] == "unknown"
  # The driving code writes params too, so this source has to be expressible.
  assert param_changes.append_param_change("C", 1, 2, source="device")["source"] == "device"


def test_a_nameless_change_is_rejected(log_path):
  assert param_changes.append_param_change("", 1, 2) is None
  assert param_changes.append_param_change("   ", 1, 2) is None
  assert param_changes.read_param_changes() == []


def test_history_is_newest_first_and_filterable(log_path):
  param_changes.append_param_change("A", 1, 2)
  param_changes.append_param_change("B", 1, 2)
  param_changes.append_param_change("A", 2, 3)

  assert [r["name"] for r in param_changes.read_param_changes()] == ["A", "B", "A"]
  assert [r["next"] for r in param_changes.read_param_changes(name="A")] == [3, 2]
  assert len(param_changes.read_param_changes(limit=2)) == 2


def test_history_is_filterable_by_source(log_path):
  param_changes.append_param_change("A", 1, 2, source="web_ui")
  param_changes.append_param_change("B", 1, 2, source="profile")
  param_changes.append_param_change("C", 1, 2, source="profile")

  profile_changes = param_changes.read_param_changes(source="profile")
  assert [r["name"] for r in profile_changes] == ["C", "B"]
  assert len(param_changes.read_param_changes(source="web_ui")) == 1
  assert param_changes.read_param_changes(source="nope") == []


def test_name_and_source_filters_combine(log_path):
  param_changes.append_param_change("A", 1, 2, source="web_ui")
  param_changes.append_param_change("A", 2, 3, source="profile")

  narrowed = param_changes.read_param_changes(name="A", source="profile")
  assert [r["next"] for r in narrowed] == [3]


def test_edited_content_is_detected_at_the_edited_record(log_path):
  param_changes.append_param_change("A", 1, 2)
  param_changes.append_param_change("B", 3, 4)
  param_changes.append_param_change("C", 5, 6)

  records = read_records(log_path)
  records[1]["next"] = 999
  log_path.write_text(
    "".join(json.dumps(r, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n" for r in records),
    encoding="utf-8",
  )

  result = param_changes.verify_param_changes()
  assert result["valid"] is False
  assert result["broken_at"] == 1
  assert result["checked"] == 1


def test_a_removed_record_breaks_the_chain(log_path):
  param_changes.append_param_change("A", 1, 2)
  param_changes.append_param_change("B", 3, 4)
  param_changes.append_param_change("C", 5, 6)

  records = read_records(log_path)
  del records[1]
  log_path.write_text(
    "".join(json.dumps(r, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n" for r in records),
    encoding="utf-8",
  )

  assert param_changes.verify_param_changes()["broken_at"] == 1


def test_a_truncated_write_is_reported_rather_than_crashing(log_path):
  param_changes.append_param_change("A", 1, 2)
  with log_path.open("a", encoding="utf-8") as f:
    f.write('{"ts": 1, "name": "B", "prev"\n')

  result = param_changes.verify_param_changes()
  assert result["valid"] is False
  assert result["broken_at"] == 1
  assert "JSON" in result["reason"]


def test_the_ring_keeps_the_newest_records_and_stays_verifiable(log_path, monkeypatch):
  monkeypatch.setattr(param_changes, "MAX_PARAM_CHANGE_RECORDS", 5)
  for index in range(9):
    param_changes.append_param_change("A", index, index + 1)

  records = read_records(log_path)
  assert len(records) == 5
  assert [r["prev"] for r in records] == [4, 5, 6, 7, 8]
  # Trimming re-anchors the chain; otherwise every log would read as broken
  # forever after the first overflow.
  assert param_changes.verify_param_changes()["valid"] is True


def test_a_log_write_failure_never_breaks_the_setting_write(log_path, monkeypatch):
  def explode(*args, **kwargs):
    raise OSError("disk full")

  monkeypatch.setattr(param_changes.os, "makedirs", explode)
  assert param_changes.append_param_change("A", 1, 2) is None


def test_fingerprint_is_stable_and_order_independent():
  left = param_changes.param_fingerprint({"B": 2, "A": 1})
  right = param_changes.param_fingerprint({"A": 1, "B": 2})

  assert left["digest"] == right["digest"]
  assert left["fingerprint"] == left["digest"][:8]
  assert left["count"] == 2


def test_fingerprint_changes_when_a_single_value_changes():
  before = param_changes.param_fingerprint({"ApplyModelSpeed": 0})
  after = param_changes.param_fingerprint({"ApplyModelSpeed": -1})

  assert before["fingerprint"] != after["fingerprint"]


def test_fingerprint_handles_an_empty_parameter_set():
  assert param_changes.param_fingerprint({})["count"] == 0


# --- fingerprint baseline --------------------------------------------------

@pytest.fixture(autouse=True)
def baseline_path(tmp_path, monkeypatch):
  path = tmp_path / "state" / "fingerprint_baseline.json"
  monkeypatch.setattr(param_changes, "CARROT_FINGERPRINT_BASELINE_PATH", str(path))
  return path


def test_no_baseline_reads_as_none(baseline_path):
  assert param_changes.read_fingerprint_baseline() is None


def test_a_saved_baseline_round_trips(baseline_path):
  param_changes.write_fingerprint_baseline("abc123", ts=1000)
  assert param_changes.read_fingerprint_baseline() == {"fingerprint": "abc123", "ts": 1000}


def test_a_corrupt_or_empty_baseline_reads_as_none(baseline_path):
  baseline_path.parent.mkdir(parents=True, exist_ok=True)
  baseline_path.write_text("{ bad", encoding="utf-8")
  assert param_changes.read_fingerprint_baseline() is None
  baseline_path.write_text('{"fingerprint": ""}', encoding="utf-8")
  assert param_changes.read_fingerprint_baseline() is None


# "Distinct parameters", not raw records: three presses of one setting is one.
def test_count_changes_since_counts_distinct_parameters(log_path, baseline_path):
  param_changes.append_param_change("A", 1, 2, ts=100)
  param_changes.append_param_change("A", 2, 3, ts=150)
  param_changes.append_param_change("B", 1, 2, ts=200)
  # An older change is excluded by the timestamp cutoff.
  param_changes.append_param_change("C", 1, 2, ts=50)

  assert param_changes.count_changes_since(100) == 2  # A, B (not the ts=50 C)


def test_count_changes_since_respects_the_allowed_set(log_path, baseline_path):
  param_changes.append_param_change("Real", 1, 2, ts=100)
  param_changes.append_param_change("GitPullTime", 1, 2, ts=100)
  assert param_changes.count_changes_since(0, allowed={"Real"}) == 1


# --- drift detection -------------------------------------------------------
# The driving code writes MyDrivingMode straight to Params, so those changes can
# never reach append_param_change() on their own.

@pytest.fixture(autouse=True)
def clean_baseline():
  param_changes._known_values.clear()
  yield
  param_changes._known_values.clear()


def test_the_first_sighting_seeds_the_baseline_without_logging(log_path):
  assert param_changes.observe_param_values({"MyDrivingMode": 1, "A": 2}) == 0
  assert param_changes.read_param_changes() == []


def test_a_value_changed_outside_the_web_server_is_recorded(log_path):
  param_changes.observe_param_values({"MyDrivingMode": 1})
  assert param_changes.observe_param_values({"MyDrivingMode": 2}) == 1

  record = param_changes.read_param_changes()[0]
  assert record["name"] == "MyDrivingMode"
  assert record["prev"] == 1
  assert record["next"] == 2
  assert record["source"] == "device"


def test_an_unchanged_read_records_nothing(log_path):
  param_changes.observe_param_values({"A": 1})
  assert param_changes.observe_param_values({"A": 1}) == 0
  assert param_changes.read_param_changes() == []


# Otherwise every write would be logged twice: once by us, once as "device".
def test_our_own_write_is_not_reported_as_an_outside_change(log_path):
  param_changes.observe_param_values({"A": 1})
  param_changes.append_param_change("A", 1, 5, source="web_ui")

  assert param_changes.observe_param_values({"A": 5}) == 0
  assert [r["source"] for r in param_changes.read_param_changes()] == ["web_ui"]


def test_several_drifted_values_are_each_recorded(log_path):
  param_changes.observe_param_values({"A": 1, "B": 1, "C": 1})
  assert param_changes.observe_param_values({"A": 2, "B": 1, "C": 3}) == 2
  assert sorted(r["name"] for r in param_changes.read_param_changes()) == ["A", "C"]


# GitPullTime and DeviceType change on every read and are not settings; they
# must never reach the change history.
def test_allowed_restricts_watching_to_real_settings(log_path):
  allowed = {"ApplyModelSpeed"}
  param_changes.observe_param_values({"ApplyModelSpeed": 0, "GitPullTime": 1}, allowed=allowed)
  assert param_changes.observe_param_values({"ApplyModelSpeed": 1, "GitPullTime": 2}, allowed=allowed) == 1

  changes = param_changes.read_param_changes()
  assert [r["name"] for r in changes] == ["ApplyModelSpeed"]


def test_without_allowed_everything_is_watched(log_path):
  param_changes.observe_param_values({"A": 1, "B": 1})
  assert param_changes.observe_param_values({"A": 2, "B": 2}) == 2


def test_an_unusable_payload_is_ignored(log_path):
  assert param_changes.observe_param_values(None) == 0
  assert param_changes.observe_param_values({}) == 0
  assert param_changes.observe_param_values("nope") == 0
