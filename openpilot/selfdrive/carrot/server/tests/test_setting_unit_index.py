import json

import pytest

from openpilot.selfdrive.carrot.server.services import setting_unit_index as unit_index


@pytest.fixture(autouse=True)
def store_path(tmp_path, monkeypatch):
  path = tmp_path / "state" / "setting_unit_index.json"
  monkeypatch.setattr(unit_index, "CARROT_SETTING_UNIT_INDEX_PATH", str(path))
  return path


def test_a_missing_store_reads_as_empty(store_path):
  assert unit_index.read_setting_unit_index() == {"units": {}}


def test_a_saved_multiplier_survives_a_reload(store_path):
  unit_index.update_setting_unit_index({"units": {"ApplyModelSpeed": 3}})
  assert unit_index.read_setting_unit_index() == {"units": {"ApplyModelSpeed": 3}}


def test_updates_merge_instead_of_replacing(store_path):
  unit_index.update_setting_unit_index({"units": {"A": 1}})
  unit_index.update_setting_unit_index({"units": {"B": 2}})
  assert unit_index.read_setting_unit_index()["units"] == {"A": 1, "B": 2}


# Index 0 is the default multiplier, so persisting it would only add noise.
def test_returning_to_the_default_multiplier_drops_the_entry(store_path):
  unit_index.update_setting_unit_index({"units": {"A": 3}})
  unit_index.update_setting_unit_index({"units": {"A": 0}})
  assert unit_index.read_setting_unit_index()["units"] == {}


def test_an_out_of_range_index_is_rejected(store_path):
  unit_index.update_setting_unit_index({"units": {"A": 99, "B": -1, "C": 2}})
  assert unit_index.read_setting_unit_index()["units"] == {"C": 2}


def test_unusable_names_and_values_are_ignored(store_path):
  unit_index.update_setting_unit_index({"units": {"": 2, "   ": 2, "A": "x", "B": None, "C": 1}})
  assert unit_index.read_setting_unit_index()["units"] == {"C": 1}


def test_a_corrupt_store_falls_back_to_empty(store_path):
  store_path.parent.mkdir(parents=True, exist_ok=True)
  store_path.write_text("{ not json", encoding="utf-8")
  assert unit_index.read_setting_unit_index() == {"units": {}}


def test_a_non_object_payload_is_tolerated(store_path):
  assert unit_index.sanitize_setting_unit_index({"units": ["nope"]}) == {"units": {}}
  assert unit_index.sanitize_setting_unit_index(None) == {"units": {}}
  assert unit_index.update_setting_unit_index("nope") == {"units": {}}


def test_the_store_is_written_atomically_and_readable(store_path):
  unit_index.update_setting_unit_index({"units": {"A": 1}})
  assert json.loads(store_path.read_text(encoding="utf-8")) == {"units": {"A": 1}}
  assert not (store_path.parent / (store_path.name + ".tmp")).exists()


def test_the_entry_count_is_bounded(store_path, monkeypatch):
  monkeypatch.setattr(unit_index, "MAX_SETTING_UNIT_ENTRIES", 3)
  unit_index.write_setting_unit_index({"units": {f"P{i}": 1 for i in range(10)}})
  assert len(unit_index.read_setting_unit_index()["units"]) == 3
