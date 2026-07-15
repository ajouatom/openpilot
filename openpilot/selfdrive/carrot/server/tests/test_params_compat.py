from pathlib import Path

import pytest

from openpilot.selfdrive.carrot import carrot_navi
from openpilot.selfdrive.carrot.server.services import params as params_service


class FakeUnknownKeyName(Exception):
  pass


class StaleParams:
  def __init__(self, root: Path) -> None:
    self.root = root

  def get_type(self, key: str):
    raise FakeUnknownKeyName(key)

  def get(self, key: str):
    raise FakeUnknownKeyName(key)

  def get_int(self, key: str):
    raise FakeUnknownKeyName(key)

  def put(self, key: str, value):
    raise FakeUnknownKeyName(key)

  def put_int(self, key: str, value: int):
    raise FakeUnknownKeyName(key)

  def get_param_path(self, key: str = "") -> str:
    return str(self.root / key) if key else str(self.root)


def int_setting(default: int = 0) -> dict:
  return {"min": 0, "max": 60, "default": default}


def test_unregistered_json_setting_uses_atomic_file_fallback(tmp_path, monkeypatch):
  stale_params = StaleParams(tmp_path)
  monkeypatch.setattr(params_service, "HAS_PARAMS", True)
  monkeypatch.setattr(params_service, "Params", lambda: stale_params)
  monkeypatch.setattr(params_service, "UnknownKeyName", FakeUnknownKeyName)

  params_service.set_param_value("FutureIntSetting", 42, int_setting())

  assert (tmp_path / "FutureIntSetting").read_text(encoding="utf-8") == "42"
  assert params_service._read_param_value(stale_params, "FutureIntSetting", 0) == 42


def test_unregistered_fallback_rejects_names_outside_settings(tmp_path, monkeypatch):
  stale_params = StaleParams(tmp_path)
  monkeypatch.setattr(params_service, "HAS_PARAMS", True)
  monkeypatch.setattr(params_service, "Params", lambda: stale_params)
  monkeypatch.setattr(params_service, "UnknownKeyName", FakeUnknownKeyName)

  with pytest.raises(FakeUnknownKeyName):
    params_service.set_param_value("NotInCarrotSettings", 1)


def test_unregistered_read_fallback_rejects_path_traversal(tmp_path):
  stale_params = StaleParams(tmp_path)
  outside = tmp_path.parent / f"{tmp_path.name}_outside"
  outside.write_text("123", encoding="utf-8")

  try:
    assert params_service._read_param_value(stale_params, f"../{outside.name}", 0) == 0
  finally:
    outside.unlink(missing_ok=True)


def test_unregistered_fallback_does_not_hide_other_write_failures(tmp_path, monkeypatch):
  class BrokenParams(StaleParams):
    def put_int(self, key: str, value: int):
      raise OSError("disk full")

  broken_params = BrokenParams(tmp_path)
  monkeypatch.setattr(params_service, "HAS_PARAMS", True)
  monkeypatch.setattr(params_service, "Params", lambda: broken_params)
  monkeypatch.setattr(params_service, "UnknownKeyName", FakeUnknownKeyName)

  with pytest.raises(OSError, match="disk full"):
    params_service.set_param_value("FutureIntSetting", 1, int_setting())


def test_map_param_reader_reads_map_fps_file_while_native_registry_is_stale(tmp_path):
  stale_params = StaleParams(tmp_path)
  (tmp_path / "ClusterNaviMapFps").write_text("3", encoding="utf-8")

  reader = carrot_navi.ClusterNaviMapParamReader(stale_params)

  assert reader._read_int("ClusterNaviMapFps", 1) == 3
