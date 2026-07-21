"""Profile storage: create, update, delete and the round trip through disk.

These paths had no tests, yet a profile is exactly where the reported "오류 3"
surfaced. Two silent-loss bugs are covered here: a create past the limit that
reports success while being dropped, and an update whose values clean down to
nothing, which used to delete the profile as a side effect.
"""
import json

import pytest

from openpilot.selfdrive.carrot.server.services import setting_profiles


CATALOG = {
  "ApplyModelSpeed": {"default": 0, "min": -120, "max": 120},
  "TFollowDecelBoost": {"default": 50, "min": 0, "max": 100},
  "CruiseSpeed1": {"default": 30, "min": 0, "max": 160},
}


@pytest.fixture(autouse=True)
def profile_store(tmp_path, monkeypatch):
  path = tmp_path / "state" / "setting_profiles.json"
  monkeypatch.setattr(setting_profiles, "CARROT_SETTING_PROFILES_PATH", str(path))
  # get_settings_cached returns (data, groups, by_name, groups_list); only
  # by_name is used, as the set of allowed parameter names.
  monkeypatch.setattr(setting_profiles, "get_settings_cached", lambda: (None, None, CATALOG, None))
  # No git and no live params in the test environment.
  monkeypatch.setattr(setting_profiles, "read_git_profile_meta", lambda: {})
  monkeypatch.setattr(setting_profiles, "snapshot_current_setting_values",
                      lambda: {"ApplyModelSpeed": 0, "TFollowDecelBoost": 50})
  return path


def test_creating_and_reading_a_profile_round_trips(profile_store):
  created = setting_profiles.create_setting_profile("mackorea 260720")
  assert created["name"] == "mackorea 260720"
  assert created["values"] == {"ApplyModelSpeed": 0, "TFollowDecelBoost": 50}

  read = setting_profiles.read_setting_profiles()["profiles"]
  assert [p["id"] for p in read] == [created["id"]]


def test_a_nameless_profile_is_rejected(profile_store):
  with pytest.raises(ValueError):
    setting_profiles.create_setting_profile("   ")


def test_unknown_parameters_are_dropped_from_stored_values(profile_store, monkeypatch):
  monkeypatch.setattr(setting_profiles, "snapshot_current_setting_values",
                      lambda: {"ApplyModelSpeed": 5, "GhostParam": 1})
  created = setting_profiles.create_setting_profile("p")
  assert "GhostParam" not in created["values"]
  assert created["values"]["ApplyModelSpeed"] == 5


# The new one used to be appended then trimmed away, reporting success.
def test_creating_past_the_limit_fails_instead_of_silently_dropping(profile_store, monkeypatch):
  monkeypatch.setattr(setting_profiles, "MAX_SETTING_PROFILES", 2)
  setting_profiles.create_setting_profile("a")
  setting_profiles.create_setting_profile("b")

  with pytest.raises(ValueError):
    setting_profiles.create_setting_profile("c")
  assert len(setting_profiles.read_setting_profiles()["profiles"]) == 2


def test_renaming_keeps_the_values(profile_store):
  created = setting_profiles.create_setting_profile("old")
  updated = setting_profiles.update_setting_profile(created["id"], {"name": "new"})
  assert updated["name"] == "new"
  assert updated["values"] == created["values"]


def test_updating_values_replaces_them(profile_store):
  created = setting_profiles.create_setting_profile("p")
  updated = setting_profiles.update_setting_profile(created["id"], {"values": {"CruiseSpeed1": 45}})
  assert updated["values"] == {"CruiseSpeed1": 45}


# The bug: cleaning down to {} used to delete the profile on the next read.
def test_an_update_that_cleans_to_nothing_is_rejected_not_applied(profile_store):
  created = setting_profiles.create_setting_profile("p")
  with pytest.raises(ValueError):
    setting_profiles.update_setting_profile(created["id"], {"values": {"OnlyGhost": 1}})

  survived = setting_profiles.get_setting_profile(created["id"])
  assert survived is not None
  assert survived["values"] == created["values"], "the profile must keep its old values"


def test_renaming_to_blank_is_rejected(profile_store):
  created = setting_profiles.create_setting_profile("p")
  with pytest.raises(ValueError):
    setting_profiles.update_setting_profile(created["id"], {"name": "   "})


# The UI translates by code, so each rejection must carry a stable one.
def test_rejections_carry_a_translatable_code(profile_store, monkeypatch):
  monkeypatch.setattr(setting_profiles, "MAX_SETTING_PROFILES", 1)
  created = setting_profiles.create_setting_profile("a")

  with pytest.raises(setting_profiles.SettingProfileError) as limit:
    setting_profiles.create_setting_profile("b")
  assert limit.value.code == "PROFILE_LIMIT"

  with pytest.raises(setting_profiles.SettingProfileError) as no_values:
    setting_profiles.update_setting_profile(created["id"], {"values": {"Ghost": 1}})
  assert no_values.value.code == "PROFILE_NO_VALUES"

  with pytest.raises(setting_profiles.SettingProfileError) as no_name:
    setting_profiles.create_setting_profile("  ")
  assert no_name.value.code == "PROFILE_NAME_REQUIRED"


def test_updating_a_missing_profile_raises(profile_store):
  with pytest.raises(KeyError):
    setting_profiles.update_setting_profile("nope", {"name": "x"})


def test_deleting_removes_only_the_target(profile_store):
  a = setting_profiles.create_setting_profile("a")
  b = setting_profiles.create_setting_profile("b")
  setting_profiles.delete_setting_profile(a["id"])

  remaining = [p["id"] for p in setting_profiles.read_setting_profiles()["profiles"]]
  assert remaining == [b["id"]]


def test_deleting_a_missing_profile_raises(profile_store):
  with pytest.raises(KeyError):
    setting_profiles.delete_setting_profile("nope")


def test_a_corrupt_store_reads_as_empty(profile_store):
  profile_store.parent.mkdir(parents=True, exist_ok=True)
  profile_store.write_text("{ not json", encoding="utf-8")
  assert setting_profiles.read_setting_profiles() == {"profiles": []}


def test_duplicate_ids_are_collapsed_on_read(profile_store):
  profile_store.parent.mkdir(parents=True, exist_ok=True)
  entry = {"id": "dup", "name": "p", "values": {"ApplyModelSpeed": 1}}
  profile_store.write_text(json.dumps({"profiles": [entry, entry]}), encoding="utf-8")
  assert len(setting_profiles.read_setting_profiles()["profiles"]) == 1


def test_the_store_is_written_atomically(profile_store):
  setting_profiles.create_setting_profile("p")
  assert not (profile_store.parent / (profile_store.name + ".tmp")).exists()
  on_disk = json.loads(profile_store.read_text(encoding="utf-8"))
  assert len(on_disk["profiles"]) == 1
