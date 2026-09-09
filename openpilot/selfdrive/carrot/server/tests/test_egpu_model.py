import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld.big_model_status import write_big_model_status


@pytest.fixture
def feature(monkeypatch, tmp_path):
  spec = importlib.util.spec_from_file_location(
    "openpilot.selfdrive.carrot.server.features.egpu_model_test",
    Path(__file__).parents[1] / "features/egpu_model.py",
  )
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  monkeypatch.setattr(module, "model_cache_dir", lambda: tmp_path)
  monkeypatch.setattr(module, "active_manifest", lambda: SimpleNamespace(model_id="old-model", sha256="a" * 64, size=100))
  monkeypatch.setattr(module, "active_model_compiled", lambda: True)
  return module


def payload(feature, engaged=False):
  return feature.build_status_payload(SimpleNamespace(get_bool=lambda key: key == "UsbGpuHardwareSeen" or (key == "IsEngaged" and engaged)))


@pytest.mark.parametrize("state", ["checking", "downloading", "verifying", "compiling", "error"])
@pytest.mark.parametrize("sha", ["a" * 64, "b" * 64])
def test_update_phase_is_not_hidden_by_compiled_model(feature, tmp_path, state, sha):
  write_big_model_status(tmp_path, state, model_id="updating-model", sha256=sha,
                         downloaded_bytes=0, total_bytes=200, detail="current update")
  result = payload(feature)
  assert result["state"] == state
  assert result["model_id"] == "updating-model"
  assert result["sha256"] == sha
  assert result["compiled"] is False
  assert result["downloaded_bytes"] == 0
  assert result["total_bytes"] == 200
  assert result["progress"] == 0
  assert result["detail"] == "current update"
  assert result["can_restart"] is (state == "error" and sha == "a" * 64)


def test_downloading_model_becomes_ready_then_compiled(feature, monkeypatch, tmp_path):
  new = SimpleNamespace(model_id="new-model", sha256="b" * 64, size=200)
  write_big_model_status(tmp_path, "downloading", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=50, total_bytes=200)
  result = payload(feature)
  assert (result["state"], result["progress"], result["compiled"], result["can_restart"]) == ("downloading", 25, False, False)

  # Verification finishes before the downloader replaces its status record.
  monkeypatch.setattr(feature, "active_manifest", lambda: new)
  monkeypatch.setattr(feature, "active_model_compiled", lambda: False)
  write_big_model_status(tmp_path, "verifying", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=200, total_bytes=200)
  assert payload(feature)["state"] == "verifying"
  write_big_model_status(tmp_path, "ready", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=200, total_bytes=200)
  result = payload(feature)
  assert (result["state"], result["progress"], result["compiled"], result["can_restart"]) == ("ready", 100, False, True)
  assert payload(feature, engaged=True)["can_restart"] is False
  write_big_model_status(tmp_path, "waiting_for_ignition", model_id=new.model_id, sha256=new.sha256)
  assert payload(feature)["state"] == "waiting_for_ignition"
  write_big_model_status(tmp_path, "compiling", model_id=new.model_id, sha256=new.sha256)
  assert payload(feature)["can_restart"] is False
  monkeypatch.setattr(feature, "active_model_compiled", lambda: True)
  assert payload(feature)["state"] == "compiling"
  write_big_model_status(tmp_path, "compiled", model_id=new.model_id, sha256=new.sha256)
  result = payload(feature)
  assert (result["state"], result["compiled"], result["can_restart"]) == ("compiled", True, False)


@pytest.mark.parametrize("state", ["ready", "waiting_for_ignition", "compiled"])
def test_other_models_status_cannot_offer_compilation(feature, tmp_path, state):
  write_big_model_status(tmp_path, state, model_id="new-model", sha256="b" * 64)
  result = payload(feature)
  assert result["state"] == "checking"
  assert result["compiled"] is False
  assert result["can_restart"] is False
  assert result["downloaded_bytes"] == 0
  assert result["total_bytes"] == 0


def test_missing_compiled_artifact_downgrades_saved_compiled_status(feature, monkeypatch, tmp_path):
  write_big_model_status(tmp_path, "compiled", model_id="old-model", sha256="a" * 64)
  monkeypatch.setattr(feature, "active_model_compiled", lambda: False)
  result = payload(feature)
  assert (result["state"], result["compiled"], result["can_restart"]) == ("ready", False, True)


def test_unidentified_update_does_not_borrow_old_model_details(feature, tmp_path):
  write_big_model_status(tmp_path, "checking")
  result = payload(feature)
  assert result["state"] == "checking"
  assert result["model_id"] is None
  assert result["sha256"] is None
  assert result["total_bytes"] == 0
  assert result["compiled"] is False


def test_no_status_uses_verified_active_model(feature):
  result = payload(feature)
  assert result["model_id"] == "old-model"
  assert result["state"] == "compiled"
  assert result["compiled"] is True
  assert result["progress"] == 100


def test_card_hidden_without_egpu_history(feature):
  assert feature.build_status_payload(SimpleNamespace(get_bool=lambda _key: False)) == {"ok": True, "available": False}
