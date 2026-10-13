"""Same-frame preparation order, failure drain and Jetson-only selection."""
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import model
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from test_hotplug_resources import joining_model
from test_nav_rl import SPEC, buffers, frame_inputs


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


@pytest.mark.parametrize('host', ['jetson', 'mac', '', None])
@pytest.mark.parametrize('failure', [None, 'input', 'read', 'link'])
def test_joining_overlaps_only_jetson_and_faults_never_publish(monkeypatch, tmp_path, host, failure):
  m = joining_model(monkeypatch)
  m.spec = SPEC
  m.packed, m.views = buffers()
  m.diagnostics = NS(record=lambda **kw: None)
  m.active = True
  events = []
  class Warp:
    timings = (0, 0, 0)
    stage_timings = {}
    def __call__(self, *args, prepare=None):
      events.append(('warp', prepare is not None))
      if prepare is not None:
        prepare()
      if failure == 'read':
        raise RuntimeError('read failed')
      return np.zeros(SPEC.warped_shape, np.uint8)
  m.warp = Warp()
  def infer(image, packed, frame, reset, source_sof):
    events.append(('send', frame, reset))
    assert np.isfinite(packed).all()
    if failure == 'link':
      raise ConnectionError('USB lost')
    return np.zeros(SPEC.output_nelem, np.float32)
  m.client = NS(peer={'carrot_host': host}, infer=infer, close=lambda: events.append('close'), timings=(0, 0, 0))
  m.small.reset = lambda: events.append('reset')
  fault = tmp_path / 'fault'
  monkeypatch.setattr(model, 'FAULT', fault)
  inputs = frame_inputs()
  if failure == 'input':
    inputs['navigation'][3] = np.nan
  result = m.run({}, {}, inputs, False)
  if failure is None:
    assert result == 'external' and m.active
    assert events[0] == ('warp', host == 'jetson')
    assert events[1] == ('send', 1, True)
    assert not fault.exists()
  else:
    assert result == 'local' and not m.active and m.client is None
    assert events[-2:] == ['close', 'reset'] and fault.exists()
    if failure != 'link':
      assert all(not isinstance(e, tuple) or e[0] != 'send' for e in events)
