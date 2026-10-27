"""Offline same-frame preparation order, exact bytes and failure drain."""
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import model
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from test_nav_rl import buffers, frame_inputs
from test_nav_rl import SPEC
from test_hotplug_resources import joining_model


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
    def copyout(dest):
      dest[:] = memoryview(read()).cast('B')
    return NS(numpy=read, shape=output.shape, _buffer=lambda: NS(copyout=copyout))
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


@pytest.mark.parametrize("overlap", [True, False])
def test_joining_adapter_prepares_same_frame_before_usb(monkeypatch, overlap):
  monkeypatch.setenv("JETLINK_CPU_OVERLAP", str(int(overlap)))
  m = joining_model(monkeypatch)
  calls = []
  m.warp, bufs, transforms = fake_warp(monkeypatch, calls)
  m.spec = SPEC
  m.packed, m.views = buffers()
  m.source_sof = 123456
  inputs = frame_inputs()
  expected, views = buffers()
  desire = np.zeros(8, np.float32)
  original_fill = model.fill_inputs
  def fill(*args):
    calls.append('prepare')
    original_fill(*args)
  monkeypatch.setattr(model, 'fill_inputs', fill)
  def infer(image, packed, frame, reset, source_sof):
    calls.append('usb')
    assert calls == (['submit', 'prepare', 'read', 'usb'] if overlap else ['prepare', 'submit', 'read', 'usb'])
    assert frame == m.frame and reset == (frame == 1) and source_sof == 123456
    assert image.tobytes() == np.arange(16, dtype=np.uint8).tobytes()
    assert packed.tobytes() == expected.tobytes()
    assert m.prev_desire.tobytes() == desire.tobytes()
    return np.zeros(SPEC.output_nelem, np.float32)
  m.client = NS(image_buffer=lambda: None, infer=infer, timings=(0, 0, 0), close=lambda: pytest.fail('unexpected fallback'))
  for frame in range(1, 33):
    inputs['desire_pulse'][:] = np.eye(8, dtype=np.float32)[frame % 8]
    if frame == 16:
      del inputs['nav_map'], inputs['navigation']
    original_fill(views, inputs, desire)
    calls.clear()
    assert m.run(bufs, transforms, inputs, False) == 'external'


@pytest.mark.parametrize('failure', ['context', 'gpu', 'usb'])
def test_joining_adapter_drains_before_fault_and_local_fallback(monkeypatch, tmp_path, failure):
  m = joining_model(monkeypatch)
  calls = []
  m.warp, bufs, transforms = fake_warp(monkeypatch, calls, read_error=failure == 'gpu')
  m.spec = SPEC
  m.packed, m.views = buffers()
  m.active = True
  inputs = frame_inputs()
  if failure == 'context': inputs['navigation'][3] = np.nan
  def infer(*a, **kw):
    calls.append('usb')
    assert failure == 'usb'
    raise ConnectionError('bounded USB timeout')
  m.client = NS(image_buffer=lambda: None, infer=infer, close=lambda: calls.append('close'))
  m.small.reset = lambda: calls.append('reset')
  def fallback(*a):
    calls.append('local')
    return 'local'
  m.small.run = fallback
  monkeypatch.setattr(model, 'FAULT', tmp_path / 'fault')
  assert m.run(bufs, transforms, inputs, False) == 'local'
  assert calls == ['submit', 'read', *(['usb'] if failure == 'usb' else []), 'close', 'reset', 'local']
  assert (tmp_path / 'fault').exists() and not m.active and m.client is None


@pytest.mark.parametrize('fault', ['shape', 'dtype', 'strided', 'readonly'])
def test_invalid_shared_destination_still_drains_submitted_gpu(fault):
  calls = []
  result = NS(shape=(16,), numpy=lambda: calls.append('drain'))
  target = np.zeros(16, np.uint8)
  if fault == 'shape': target = target.reshape(2, 8)
  if fault == 'dtype': target = target.astype(np.float32)
  if fault == 'strided': target = np.zeros(32, np.uint8)[::2]
  if fault == 'readonly': target.flags.writeable = False
  with pytest.raises(ValueError, match='invalid warp destination'):
    model.Warp.read_result(result, target)
  assert calls == ['drain']


@pytest.mark.parametrize('bad_context', [False, True])
def test_same_submission_copyout_uses_reserved_buffer_and_keeps_fence(monkeypatch, bad_context):
  calls = []
  warp, bufs, transforms = fake_warp(monkeypatch, calls)
  target = np.zeros(16, np.uint8)
  def prepare():
    calls.append('prepare')
    if bad_context: raise ValueError('bad context')
  if bad_context:
    with pytest.raises(ValueError, match='bad context'):
      warp(bufs, transforms, prepare=prepare, output_buffer=target)
  else:
    assert warp(bufs, transforms, prepare=prepare, output_buffer=target) is target
  assert calls == ['submit', 'prepare', 'read']
  assert target.tobytes() == np.arange(16, dtype=np.uint8).tobytes()
