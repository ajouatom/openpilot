import io
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld import cheddar_model as client, cheddar_runtime as runtime


def fake_model():
  model = object.__new__(client.CheddarModelState)
  model.frame_size = 12
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


