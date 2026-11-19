import io
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld import cheddar_model as client, cheddar_runtime as runtime


def fake_model():
  model = object.__new__(client.CheddarModelState)
  model.frame_size = 12
  model.camera_size = 2, 2
  model.inference_timeout = .15
  model.views = {name: np.zeros(12, np.uint8) for name in ('img', 'big_img')}
  model.views.update(desire=np.zeros(8, np.float32), traffic_convention=np.zeros((1, 2), np.float32),
                     action_t=np.zeros((1, 2), np.float32), tfm=np.zeros((3, 3)), big_tfm=np.zeros((3, 3)))
  model.prev_desire = np.zeros(8, np.float32)
  model.output = np.arange(2580, dtype=np.float32)
  model.output_slices = {'outputs': slice(0, 2580)}
  model.parser = SimpleNamespace(parse_outputs=lambda value: value)
  model.process = SimpleNamespace(stdin=io.BytesIO())
  model._receive = lambda timeout: b'1\n'
  model.closed = False
  def close():
    model.closed, model.process = True, None
  model.close = close
  return model


def arguments():
  frames = {name: SimpleNamespace(data=np.arange(12, dtype=np.uint8)) for name in ('img', 'big_img')}
  transforms = {name: np.eye(3, dtype=np.float32) for name in frames}
  inputs = {'desire_pulse': np.array([1, 1, 0, 0, 0, 0, 0, 0], np.float32),
            'traffic_convention': [1, 0], 'action_t': [.2, .3]}
  return frames, transforms, inputs


def test_full_history_policy_runs_on_catch_up_without_mutating_caller_desire():
  model = fake_model()
  frames, transforms, inputs = arguments()
  try:
    original = inputs['desire_pulse'].copy()
    first = model.run(frames, transforms, inputs, True)
    np.testing.assert_array_equal(model.views['desire'], [0, 1, 0, 0, 0, 0, 0, 0])
    np.testing.assert_array_equal(inputs['desire_pulse'], original)
    np.testing.assert_array_equal(first['outputs'][0], model.output)
    model.run(frames, transforms, inputs, False)
    assert not model.views['desire'].any()
    inputs['desire_pulse'][1] = 0
    model.run(frames, transforms, inputs, False)
    inputs['desire_pulse'][1] = 1
    model.run(frames, transforms, inputs, False)
    assert model.views['desire'][1] == 1
    assert model.process.stdin.getvalue() == b'rrrr'
  finally:
    model.close()


@pytest.mark.parametrize('failure', ['nan', 'short', 'reply', 'timeout', 'frame'])
def test_invalid_execution_closes_worker_and_never_publishes(failure):
  model = fake_model()
  frames, transforms, inputs = arguments()
  if failure == 'nan':
    model.output[100] = np.nan
  elif failure == 'short':
    model.output = model.output[:-1]
  elif failure == 'reply':
    model._receive = lambda timeout: b'0\n'
  elif failure == 'timeout':
    def receive(timeout):
      assert timeout == .15
      raise TimeoutError('test')
    model._receive = receive
  else:
    frames['img'].data = np.zeros(3, np.uint8)
  with pytest.raises((ValueError, RuntimeError, TimeoutError)):
    model.run(frames, transforms, inputs, False)
  assert model.closed


@pytest.mark.parametrize('reply', [b'Z\n', b'1\n'])
def test_reset_waits_for_gpu_ack_before_clearing_pulse_history(reply):
  model = fake_model()
  model.prev_desire[:] = 1
  model._receive = lambda timeout: reply
  if reply == b'Z\n':
    model.reset()
    assert not model.prev_desire.any()
    assert model.process.stdin.getvalue() == b'z'
    model.close()
  else:
    with pytest.raises(RuntimeError, match='reset'):
      model.reset()
    assert model.closed and model.prev_desire.all()


def test_rejects_wrong_runtime_archive_before_extraction(tmp_path, monkeypatch):
  archive = tmp_path / 'bad.tar.gz'
  archive.write_bytes(b'not the reviewed runtime')
  monkeypatch.setattr(runtime, 'ARCHIVE', archive)
  with pytest.raises(ValueError, match='checksum'):
    runtime.ensure_runtime(tmp_path / 'runtime')
  assert not (tmp_path / 'runtime').exists()


def test_runtime_activation_refuses_an_existing_tinygrad_interpreter(monkeypatch, tmp_path):
  import sys
  monkeypatch.setitem(sys.modules, 'tinygrad', SimpleNamespace())
  with pytest.raises(RuntimeError, match='isolated'):
    runtime.activate(tmp_path)


def test_parent_death_removes_owned_shared_storage_before_worker_exit(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld import cheddar_worker as worker
  path = tmp_path / 'owned-shared-buffer'
  path.write_bytes(b'camera buffer')
  parents = iter([123, 1])
  monkeypatch.setattr(worker.os, 'getppid', lambda: next(parents))
  monkeypatch.setattr(worker.time, 'sleep', lambda delay: None)
  monkeypatch.setattr(worker.os, 'sched_setscheduler', lambda *args: None, raising=False)
  monkeypatch.setattr(worker.os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(worker.os, 'sched_param', lambda value: value, raising=False)
  def exit_worker(code):
    assert not path.exists()
    raise SystemExit(code)
  monkeypatch.setattr(worker.os, '_exit', exit_worker)
  with pytest.raises(SystemExit) as raised:
    worker.watch_parent(123, path)
  assert raised.value.code == 1


def test_tightly_packed_pc_nv12_keeps_pixels_and_zeros_venus_padding():
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, yh, uvh, _ = get_nv12_info(6, 4)
  destination = np.full(stride * (yh + uvh), 255, np.uint8)
  source = np.arange(36, dtype=np.uint8)
  frame = SimpleNamespace(data=source, stride=6, uv_offset=24)
  client.copy_camera_frame(destination, frame, 6, 4)
  expected = np.zeros_like(destination)
  for row in range(4):
    expected[row * stride:row * stride + 6] = source[row * 6:row * 6 + 6]
  for row in range(2):
    expected[(yh + row) * stride:(yh + row) * stride + 6] = source[24 + row * 6:30 + row * 6]
  np.testing.assert_array_equal(destination, expected)


def test_venus_frame_uses_only_image_planes_not_kernel_padding():
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, yh, uvh, total = get_nv12_info(6, 4)
  destination = np.zeros(stride * (yh + uvh), np.uint8)
  source = np.arange(total, dtype=np.uint8)
  client.copy_camera_frame(destination, SimpleNamespace(data=source, stride=stride, uv_offset=stride * yh), 6, 4)
  np.testing.assert_array_equal(destination, source[:destination.size])


