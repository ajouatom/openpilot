"""Offline same-frame preparation order, exact bytes and failure drain."""
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import model
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from test_nav_rl import buffers, frame_inputs


def fake_warp(monkeypatch, calls, read_error=False):
  monkeypatch.setitem(sys.modules, 'tinygrad', NS(Tensor=NS(from_blob=lambda *a, **kw: object())))
  warp = object.__new__(model.Warp)
  warp.size = 16
  warp.blobs = {}
  warp.transforms = {k: np.eye(3, dtype=np.float32) for k in ('tfm', 'big_tfm')}
  warp.inputs = {}
  output = np.arange(16, dtype=np.uint8)
  def submit(**kw):
    calls.append('submit')
    def read():
      calls.append('read')
      if read_error:
        raise RuntimeError('GPU read failed')
      return output
    return NS(numpy=read)
  warp.run_warp = submit
  return warp, {k: NS(data=output) for k in ('img', 'big_img')}, {k: np.eye(3) for k in ('img', 'big_img')}


def test_overlap_preserves_every_image_context_and_desire_bit(monkeypatch):
  calls = []
  warp, bufs, transforms = fake_warp(monkeypatch, calls)
  original, old_views = buffers()
  candidate, new_views = buffers()
  old_desire, new_desire = np.zeros(8, np.float32), np.zeros(8, np.float32)
  inputs = frame_inputs()
  for have_route in (True, True, False):
    if not have_route:
      del inputs['nav_map'], inputs['navigation']
    fill_inputs(old_views, inputs, old_desire)
    expected_image = warp(bufs, transforms)
    calls.clear()
    def prepare():
      calls.append('prepare')
      fill_inputs(new_views, inputs, new_desire)
    actual_image = warp(bufs, transforms, prepare=prepare)
    assert calls == ['submit', 'prepare', 'read']
    assert actual_image.tobytes() == expected_image.tobytes()
    assert candidate.tobytes() == original.tobytes()
    assert new_desire.tobytes() == old_desire.tobytes()


def test_invalid_context_still_drains_gpu_before_raising(monkeypatch):
  calls = []
  warp, bufs, transforms = fake_warp(monkeypatch, calls)
  _, views = buffers()
  inputs = frame_inputs()
  inputs['navigation'][3] = np.nan
  def prepare():
    calls.append('prepare')
    fill_inputs(views, inputs, np.zeros(8, np.float32))
  with pytest.raises(ValueError, match='Invalid Jetlink navigation'):
    warp(bufs, transforms, prepare=prepare)
  assert calls == ['submit', 'prepare', 'read']


def test_gpu_read_failure_is_not_replaced_with_a_previous_image(monkeypatch):
  calls = []
  warp, bufs, transforms = fake_warp(monkeypatch, calls, read_error=True)
  with pytest.raises(RuntimeError, match='GPU read failed'):
    warp(bufs, transforms, prepare=lambda: calls.append('prepare'))
  assert calls == ['submit', 'prepare', 'read']


