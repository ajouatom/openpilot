import math

import numpy as np
import pytest

from openpilot.selfdrive.carrot.xiaoge.lane_inference import LaneInference
from openpilot.selfdrive.carrot.xiaoge.v_asm_inference import VASMInference


@pytest.mark.parametrize("extra_rows", [0, 2, 4])
@pytest.mark.parametrize("padding", [0, 2])
@pytest.mark.parametrize("width,height", [(8, 4), (4, 8)])
def test_lane_preprocessing_ignores_chroma_and_padding(extra_rows, padding, width, height):
  frame = np.full((height + extra_rows, width + padding), 255, dtype=np.uint8)
  if width > height:
    frame[:height, :width] = np.arange(width, dtype=np.uint8)[None, :] * 30
  else:
    frame[:height, :width] = np.arange(height, dtype=np.uint8)[:, None] * 30

  blob = LaneInference().preprocess_image(frame, width, height)

  assert blob.shape == (1, 3, 416, 416)
  assert blob.dtype == np.float32
  np.testing.assert_array_equal(blob[0, 0], blob[0, 1])
  np.testing.assert_array_equal(blob[0, 1], blob[0, 2])
  # The square contains source columns/rows 2..5; neither the edges nor padding.
  assert float(blob.min()) == pytest.approx(60 / 255)
  assert float(blob.max()) == pytest.approx(150 / 255)


@pytest.mark.parametrize("shape,width,height", [((3, 4), 4, 4), ((4, 3), 4, 4), ((4, 4), 0, 4), ((4,), 4, 4)])
def test_lane_preprocessing_rejects_incomplete_frames(shape, width, height):
  with pytest.raises(ValueError):
    LaneInference().preprocess_image(np.zeros(shape, dtype=np.uint8), width, height)


def test_lane_onnx_loads_and_infers():
  inference = LaneInference()
  assert inference.load(), inference.error

  result = inference.infer(np.zeros((624, 416), dtype=np.uint8), 416, 416)

  assert result["valid"], result["error"]
  assert result["leftLine"] in (-1, 0, 1)
  assert result["rightLine"] in (-1, 0, 1)


def test_blindspot_onnx_loads_and_infers_target_side_only():
  inference = VASMInference()
  assert inference.load(), inference.error
  inference.load_config({
    "width": 352, "height": 256,
    "poly_left": [[0, 0], [174, 0], [174, 254], [0, 254]],
    "poly_right": [[176, 0], [350, 0], [350, 254], [176, 254]],
  })
  frame = np.zeros((384, 352), dtype=np.uint8)
  frame[256:] = 128

  inference.update(frame, 352, 256, "left", 0.94, 0.2, 0.25)

  assert math.isfinite(inference.confidence["left"])
  assert 0 <= inference.confidence["left"] <= 1
  assert inference.confidence["right"] == 0
  assert not inference.active["right"]


def test_blindspot_classifier_maps_car_class_and_letterboxes_roi():
  inference = VASMInference()
  inference.load_config({
    "width": 8, "height": 4,
    "poly_left": [[0, 0], [7, 0], [7, 3], [0, 3]],
    "poly_right": [],
  })
  recorded = {}

  class FakeNet:
    def setInput(self, blob):
      recorded["blob"] = blob

    def forward(self):
      return np.array([[0.03, 0.96, 0.01]], dtype=np.float32)

  inference.net = FakeNet()
  inference.valid = True
  frame = np.zeros((6, 8), dtype=np.uint8)
  frame[:4] = 255
  frame[4:] = 128

  inference.update(frame, 8, 4, "left", 0.94, 0.2, 0.25)

  assert inference.confidence["left"] == pytest.approx(0.96)
  assert inference.active["left"]
  assert recorded["blob"].shape == (1, 3, 352, 352)
  assert np.all(recorded["blob"][:, :, :88] == 0)
  assert np.any(recorded["blob"][:, :, 88:264] > 0)
  assert np.all(recorded["blob"][:, :, 264:] == 0)


def test_blindspot_classifier_uses_dual_threshold_hysteresis():
  inference = VASMInference()
  inference.load_config({
    "width": 4, "height": 4,
    "poly_left": [[0, 0], [3, 0], [3, 3], [0, 3]],
    "poly_right": [],
  })

  class FakeNet:
    confidence = 0.95

    def setInput(self, _blob):
      pass

    def forward(self):
      return np.array([[1.0 - self.confidence, self.confidence]], dtype=np.float32)

  inference.net = FakeNet()
  inference.valid = True
  frame = np.zeros((6, 4), dtype=np.uint8)

  inference.update(frame, 4, 4, "left", 0.94, 0.2, 0.2)
  assert inference.active["left"]
  inference.net.confidence = 0.80
  inference.update(frame, 4, 4, "left", 0.94, 0.2, 0.2)
  assert inference.active["left"]
  inference.net.confidence = 0.78
  inference.update(frame, 4, 4, "left", 0.94, 0.2, 0.2)
  assert not inference.active["left"]
