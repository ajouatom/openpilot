from pathlib import Path

import pytest

import openpilot.selfdrive.modeld.helpers as helpers
from openpilot.selfdrive.modeld.helpers import modeld_pkl_path, select_vision_streams


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
  assert modeld_pkl_path(True).name == "usbgpu_driving_tinygrad.pkl"


def test_usbgpu_present_accepts_both_supported_usb_ids(monkeypatch, tmp_path: Path):
  monkeypatch.setattr(helpers.Path, "glob", lambda _self, _pattern: [tmp_path])

  for vendor in ("add1", "3801"):
    (tmp_path / "idVendor").write_text(vendor)
    (tmp_path / "idProduct").write_text("0001")
    assert helpers.usbgpu_present()

  (tmp_path / "idVendor").write_text("ffff")
  assert not helpers.usbgpu_present()
