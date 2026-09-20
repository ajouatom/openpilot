from types import SimpleNamespace
import sys

import numpy as np
import pytest

from openpilot.selfdrive.modeld import local_gpu_warp as local


@pytest.mark.parametrize('device,expected', [('tici', True), ('tizi', True), ('mici', False), ('pc', False), ('', False)])
def test_local_warp_is_only_selected_for_c3(device, expected):
  assert local.use_local_warp(device) is expected


@pytest.mark.parametrize('device,fault', [('tizi', None), ('tizi', 'init'), ('tizi', 'bind'), ('mici', None)])
def test_local_warp_failures_retain_original_backend(monkeypatch, device, fault):
  calls, errors = [], []
  class Amd:
    def __init__(self, *args):
      calls.append(type(self).__name__)
      self.args = args
    def bind_shared(self, packed):
      self.packed = packed
  class Qcom(Amd):
    def __init__(self, *args):
      super().__init__(*args)
      if fault == 'init':
        raise RuntimeError('QCOM compile failed')
    def bind_shared(self, packed):
      if fault == 'bind':
        raise RuntimeError('pixel mismatch')
      super().bind_shared(packed)
  monkeypatch.setattr(local, 'GenericModelRuntime', Amd)
  monkeypatch.setattr(local, 'LocalWarpRuntime', Qcom)
  args, packed = ('model', 1928, 1208), object()
  adapter = local.create_runtime(args, device, errors.append)
  adapter = local.bind_runtime(adapter, args, packed, errors.append)
  assert type(adapter) is (Qcom if device == 'tizi' and fault is None else Amd)
  assert adapter.packed is packed and adapter.args == args
  assert len(errors) == int(fault is not None)
  if device == 'mici':
    assert calls == ['Amd']


def test_amd_binding_failure_is_not_hidden(monkeypatch):
  class Amd:
    def bind_shared(self, packed):
      raise RuntimeError('AMD failure')
  with pytest.raises(RuntimeError, match='AMD failure'):
    local.bind_runtime(Amd(), (), object(), lambda _: pytest.fail('unexpected local fallback'))


def test_prepared_transfer_preserves_controls_and_recurrent_state():
  runtime = object.__new__(local.LocalWarpRuntime)
  runtime.frames_offset = 512
  images = np.arange(2 * 6 * 128 * 256, dtype=np.uint8).reshape(2, 6, 128, 256)
  runtime.raw = np.full(7471616, 99, np.uint8)
  runtime.raw[:512] = np.arange(512, dtype=np.uint8)
  runtime.compact = np.zeros(512 + images.size, np.uint8)
  runtime.host = runtime.compact
  calls = []
  runtime.prepare_images = lambda: images
  def upload(host):
    assert host.size == 393728
    np.testing.assert_array_equal(host[:512], runtime.raw[:512])
    np.testing.assert_array_equal(host[512:], images.reshape(-1))
    calls.append('upload')
  runtime.device_buffer = SimpleNamespace(copy_from=upload)
  state = np.zeros(1)
  runtime.queues = {'new_img': object(), 'state': state}
  runtime.outputs = {'next_state': state, 'outputs': SimpleNamespace(numpy=lambda: np.array([[1., 2.]], np.float32))}
  def model(output_buffers, **inputs):
    assert inputs['state'] is output_buffers['next_state']
    assert inputs['new_img'] is runtime.queues['new_img']
    state[:] += 1
    calls.append('model')
  runtime.run_model = model
  for _ in range(2):
    np.testing.assert_array_equal(runtime.run(), [1, 2])
  assert state[0] == 2
  assert calls == ['upload', 'model', 'upload', 'model']
  assert runtime.last_timings['local_prepare_ms'] >= 0
  assert runtime.last_timings['input_upload_ms'] >= 0


@pytest.mark.parametrize('fault', [None, 'pixel', 'dtype', 'shape', 'exception'])
def test_device_validation_rejects_different_warp_and_clears_probe_input(monkeypatch, fault):
  runtime = object.__new__(local.LocalWarpRuntime)
  runtime.raw = np.zeros(256, np.uint8)
  runtime.device = 'AMD'
  runtime.frames_offset, runtime.frame_size = 128, 64
  runtime.specs = {'new_img': ((2, 6, 128, 256), 'uint8', 'AMD')}
  runtime.local_host = object()
  expected = np.zeros((2, 6, 128, 256), np.uint8)
  reference = SimpleNamespace(copy_from=lambda _: None)
  monkeypatch.setitem(sys.modules, 'tinygrad', SimpleNamespace(Tensor=lambda *a, **kw: SimpleNamespace(_buffer=lambda: reference)))
  monkeypatch.setitem(sys.modules, 'tinygrad.dtype', SimpleNamespace(dtypes=SimpleNamespace(uint8='uint8', float32='float32')))
  monkeypatch.setattr(local, 'input_view', lambda *args: object())
  runtime.amd_warp = lambda **kw: SimpleNamespace(numpy=lambda: expected)
  probes = []
  def prepare():
    probes.append(np.ndarray((2, 3, 3), np.float32, buffer=runtime.raw).copy())
    if fault == 'exception':
      raise RuntimeError('device failure')
    actual = expected.copy()
    if fault == 'pixel':
      actual[1, 5, 127, 255] = 1
    if fault == 'dtype':
      actual = actual.astype(np.float32)
    if fault == 'shape':
      actual = actual.reshape(-1)
    return actual
  runtime.prepare_images = prepare
  if fault:
    with pytest.raises(RuntimeError):
      runtime.validate_warp()
  else:
    runtime.validate_warp()
    assert len(probes) == 3
    assert not np.array_equal(probes[0], probes[1])
  assert not np.any(runtime.raw)
