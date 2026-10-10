from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.signal_tracking_shadow import copy_nv12_rgb, next_delay, result_fields, tracking_requested


def test_tracking_is_explicit_opt_in(tmp_path):
  assert not tracking_requested(tmp_path)
  (tmp_path / 'tracking_enabled').write_text('1')
  assert tracking_requested(tmp_path)
  (tmp_path / 'tracking_enabled').write_text('0')
  assert not tracking_requested(tmp_path)


@pytest.mark.parametrize('age', [-1, 201, float('nan')])
def test_stale_output_cannot_be_green(age):
  result = result_fields({'state': 'green', 'reason': 'agreeing_visible_tracks'}, age)
  assert result['prediction'] == 'unknown' and not result['fresh'] and not result['control_permission']


@pytest.mark.parametrize('wall,cpu', [(.01, .01), (.08, .04), (.3, .2)])
def test_cpu_and_rate_budget(wall, cpu):
  delay = next_delay(wall, cpu)
  assert wall + delay >= .05
  assert cpu / (cpu + delay) <= .5


def test_nv12_padding_and_no_input_mutation():
  width, height, stride = 1344, 760, 1408
  offset = stride * height + 128
  raw = np.full(offset + stride * height // 2, 99, np.uint8)
  y = raw[:stride * height].reshape(height, stride)
  uv = raw[offset:].reshape(height // 2, stride)
  y[:, :width] = 16
  y[:, width // 2:width] = 235
  uv[:, :width] = 128
  before = raw.copy()
  frame = SimpleNamespace(width=width, height=height, stride=stride, uv_offset=offset, data=raw)
  rgb = copy_nv12_rgb(frame)
  assert rgb.shape == (height, width, 3)
  assert (rgb[:, :width // 2] == 0).all() and (rgb[:, width // 2:] == 255).all()
  np.testing.assert_array_equal(raw, before)
  frame.data = raw[:-1]
  with pytest.raises(ValueError, match='truncated'):
    copy_nv12_rgb(frame)
  frame.width = 1008
  with pytest.raises(ValueError, match='geometry'):
    copy_nv12_rgb(frame)


def test_existing_worker_dispatches_only_with_opt_in(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld import signal_color_shadow, signal_tracking_shadow
  calls = []
  (tmp_path / 'tracking_enabled').write_text('1')
  monkeypatch.setattr(signal_tracking_shadow, 'run', lambda directory, duration: calls.append((directory, duration)))
  signal_color_shadow.run(tmp_path, 2)
  assert calls == [(tmp_path, 2)]
