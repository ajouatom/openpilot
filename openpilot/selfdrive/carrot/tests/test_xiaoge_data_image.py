import importlib.util
import sys
import types
from io import BytesIO
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


XIAOGE_DATA_PATH = Path(__file__).parents[1] / "xiaoge_data.py"


@pytest.fixture
def xiaoge_data_module(monkeypatch):
  cereal = types.ModuleType("openpilot.cereal")
  cereal.messaging = types.SimpleNamespace()
  cereal.car = types.SimpleNamespace(CarParams=object)

  class CANParser:
    pass

  class Params:
    pass

  class Ratekeeper:
    pass

  stubs = {
    "openpilot.cereal": cereal,
    "openpilot.cereal.messaging": cereal.messaging,
    "opendbc.can": types.SimpleNamespace(CANParser=CANParser),
    "openpilot.common.params": types.SimpleNamespace(Params=Params),
    "openpilot.common.realtime": types.SimpleNamespace(Ratekeeper=Ratekeeper),
  }
  for name, stub in stubs.items():
    monkeypatch.setitem(sys.modules, name, stub)

  module_name = f"_test_xiaoge_data_{id(cereal)}"
  spec = importlib.util.spec_from_file_location(module_name, XIAOGE_DATA_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module


def test_y_to_jpeg_center_crops_stride_padded_frame(xiaoge_data_module):
  module = xiaoge_data_module
  width, height, stride = 8, 4, 10
  rows = np.full((height, stride), 255, dtype=np.uint8)
  rows[:, :width] = np.arange(width, dtype=np.uint8) * 30

  jpeg = module.XiaogeDataBroadcaster.y_to_jpeg(rows.tobytes(), width, height, stride)

  with Image.open(BytesIO(jpeg)) as image:
    assert image.size == (module.TARGET_WIDTH, module.TARGET_HEIGHT)
    assert image.mode == "L"
    pixels = np.asarray(image)
  # The centered square uses source columns 2..5, not the stride padding or full-width edges.
  assert 55 <= float(pixels[:, 0].mean()) <= 70
  assert 145 <= float(pixels[:, -1].mean()) <= 160


def test_y_to_jpeg_rejects_short_plane(xiaoge_data_module):
  with pytest.raises(ValueError, match="short Y plane"):
    xiaoge_data_module.XiaogeDataBroadcaster.y_to_jpeg(b"\0" * 15, 4, 4, 4)
