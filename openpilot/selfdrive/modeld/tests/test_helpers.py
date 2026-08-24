from pathlib import Path

import pytest

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.big_model import BigModelManifest, _write_state
import openpilot.selfdrive.modeld.helpers as helpers
from openpilot.selfdrive.modeld.helpers import modeld_pkl_path, select_vision_streams, usbgpu_compiled_path


@pytest.mark.parametrize(
  ("available", "use_wide_camera", "main", "use_extra"),
  [
    ({"road", "wide"}, True, "road", True),
    ({"road", "wide"}, False, "road", False),
    ({"road"}, True, "road", False),
    ({"wide"}, True, "wide", False),
    ({"wide"}, False, None, False),
    (set(), True, None, False),
  ],
)
def test_select_vision_streams(available, use_wide_camera, main, use_extra):
  assert select_vision_streams(available, "road", "wide", use_wide_camera) == (main, use_extra)


def test_usbgpu_artifact_uses_checked_in_model_name():
  assert modeld_pkl_path(False).name == "driving_tinygrad.pkl"
  assert modeld_pkl_path(True, "a" * 64).name == "big_driving_aaaaaaaaaaaaaaaa_tinygrad.pkl"


def test_usbgpu_present_accepts_both_supported_usb_ids(monkeypatch, tmp_path: Path):
  monkeypatch.setattr(helpers.Path, "glob", lambda _self, _pattern: [tmp_path])

  for vendor in ("add1", "3801"):
    (tmp_path / "idVendor").write_text(vendor)
    (tmp_path / "idProduct").write_text("0001")
    assert helpers.usbgpu_present()

  (tmp_path / "idVendor").write_text("ffff")
  assert not helpers.usbgpu_present()


def test_usbgpu_compiled_path_falls_back_to_previous_model(monkeypatch, tmp_path: Path):
  cache_dir = tmp_path / "cache"
  compiled_dir = tmp_path / "compiled"
  monkeypatch.setenv("CARROT_BIG_MODEL_DIR", str(cache_dir))
  monkeypatch.setattr(helpers, "MODELS_DIR", compiled_dir)
  active = BigModelManifest("big-401", "big_driving_supercombo.onnx", 1, "a" * 64, "https://example.com/401.onnx")
  previous = BigModelManifest("big-400", "big_driving_supercombo.onnx", 1, "b" * 64, "https://example.com/400.onnx")
  _write_state(active, previous, cache_dir)

  previous_pkl = modeld_pkl_path(True, previous.sha256)
  Path(get_manifest_path(previous_pkl)).parent.mkdir(parents=True)
  Path(get_manifest_path(previous_pkl)).write_text("1")
  assert usbgpu_compiled_path() == previous_pkl
