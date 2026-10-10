import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.signal_color_shadow import (
  MODEL_SHA256, next_delay, requested, result_fields, rgb_from_nv12, scan_image, select_candidates,
  validate_artifact, window_boxes,
)


def test_selection_is_off_by_default_and_rejects_wrong_model(tmp_path):
  assert not requested(tmp_path)
  (tmp_path / 'enabled').write_text('1')
  assert requested(tmp_path)
  meta = {'version': 1, 'mode': 'camera_comparison_only', 'model_sha256': MODEL_SHA256,
          'classes': ['background', 'red', 'green']}
  (tmp_path / 'installed.json').write_text(json.dumps(meta))
  (tmp_path / 'signal_color_pool.onnx').write_bytes(b'wrong')
  with pytest.raises(ValueError, match='checksum'):
    validate_artifact(tmp_path)
  meta['mode'] = 'control'
  (tmp_path / 'installed.json').write_text(json.dumps(meta))
  with pytest.raises(ValueError, match='manifest'):
    validate_artifact(tmp_path)
  (tmp_path / 'enabled').write_text('0')
  assert not requested(tmp_path)


def test_nv12_padding_copy_and_reference_colors():
  # Two rows of black, white, red and green pairs. Rows and UV offset are padded.
  raw = np.full(60, 77, np.uint8)
  raw[:8] = raw[12:20] = [16, 16, 235, 235, 81, 81, 145, 145]
  raw[36:44] = [128, 128, 128, 128, 90, 240, 54, 34]
  before = raw.copy()
  rgb = rgb_from_nv12(raw, 8, 2, 12, 36)
  expected = np.array([[0, 0, 0], [255, 255, 255], [255, 0, 0], [0, 255, 1]], np.uint8).repeat(2, axis=0)
  np.testing.assert_allclose(rgb[0], expected, atol=1)
  np.testing.assert_array_equal(rgb[0], rgb[1])
  np.testing.assert_array_equal(raw, before)


@pytest.mark.parametrize('width,height,stride,offset,size', [(3, 2, 4, 8, 12), (4, 2, 2, 8, 12),
                                                           (4, 2, 4, 4, 12), (4, 2, 4, 8, 11)])
def test_invalid_camera_layout_rejected(width, height, stride, offset, size):
  with pytest.raises(ValueError):
    rgb_from_nv12(bytes(size), width, height, stride, offset)


def test_search_preserves_strongest_window_and_separates_alternatives():
  boxes = [(0, 0, 100, 50), (1, 0, 101, 50), (200, 0, 300, 50)]
  p = np.array([[.01, .98, .01], [.02, .02, .96], [.05, .05, .9]])
  selected = select_candidates(p, boxes)
  assert selected[0]['color'] == 'red'
  assert selected[0]['box_xyxy'] == list(boxes[0])
  assert selected[1]['box_xyxy'] == list(boxes[2])
  assert len(selected) == 2
  assert result_fields(selected, 100)['prediction'] == 'red'
  assert result_fields(selected, 1501)['prediction'] == 'unknown'
  assert not result_fields(selected, -1)['usable']
  selected[0]['score'] = .79
  assert result_fields(selected, 100)['prediction'] == 'unknown'


@pytest.mark.parametrize('values', [[[float('nan'), 0, 1]], [[0, 2, -1]], [[.2, .2, .2]], [[1, 0]]])
def test_bad_probabilities_never_logged_as_signal(values):
  with pytest.raises(ValueError):
    select_candidates(values, [(0, 0, 4, 2)])


def test_all_windows_are_evaluated_in_bounded_batches():
  from PIL import Image
  source = np.random.default_rng(1).integers(0, 256, (760, 1344, 3), dtype=np.uint8)
  reference = Image.fromarray(source)
  class Session:
    def __init__(self):
      self.n = 0
      self.max_batch = 0

    def run(self, outputs, feeds):
      x = feeds['rgb']
      assert x.shape[1:] == (3, 32, 96) and x.dtype == np.float32
      assert x.min() >= 0 and x.max() <= 1
      expected = np.stack([np.asarray(reference.crop(b).resize((96, 32), Image.Resampling.BILINEAR), np.float32)
                           .transpose(2, 0, 1) / 255 for b in window_boxes()[self.n:self.n + len(x)]])
      np.testing.assert_array_equal(x, expected)
      self.n += len(x)
      self.max_batch = max(self.max_batch, len(x))
      return [np.tile([.99, .005, .005], (len(x), 1))]
  session = Session()
  before = source.copy()
  candidates = scan_image(source, session)
  assert len(window_boxes()) == session.n == 667
  assert session.max_batch == 32
  assert result_fields(candidates, 100)['prediction'] == 'unknown'
  np.testing.assert_array_equal(source, before)


@pytest.mark.parametrize('wall,cpu', [(.2, .1), (.8, .2), (4, .3), (.01, .01)])
def test_rate_and_cpu_budget_do_not_catch_up(wall, cpu):
  delay = next_delay(wall, cpu)
  assert wall + delay >= 1
  assert cpu / (cpu + delay) <= .25 + 1e-10


def test_worker_has_no_control_publication():
  # Keep the integration boundary explicit; regression against accidental
  # control publication/Params writes, not a proof of freedom from CPU contention.
  source = (Path(__file__).parents[1] / 'signal_color_shadow.py').read_text()
  assert 'PubMaster' not in source
  assert 'modelV2' not in source
  assert 'Params(' not in source


def test_error_latches_without_repeated_inference(monkeypatch):
  from openpilot.selfdrive.modeld import signal_color_shadow as module
  failures, sleeps = [], []

  def broken():
    failures.append('called')
    raise ValueError('invalid artifact')

  enabled = iter((True, True, False))
  monkeypatch.setattr(module, 'run', broken)
  monkeypatch.setattr(module, 'requested', lambda: next(enabled))
  monkeypatch.setattr(module.time, 'sleep', sleeps.append)
  errors = []
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(exception=errors.append)))
  module.main()
  assert failures == ['called']
  assert len(errors) == 1 and sleeps == [1, 1]
