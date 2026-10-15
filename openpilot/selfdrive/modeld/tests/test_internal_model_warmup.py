from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.internal_model_warmup import warm_internal_model


def test_warmup_uses_one_allocated_buffer_without_waiting_for_extra_camera_backlog(monkeypatch):
  from openpilot.selfdrive.modeld import internal_model_warmup as warm
  frame = object()
  buffers = iter([None, None, frame])
  model = SimpleNamespace(vision_input_names=['img', 'big_img'])
  calls = []
  monkeypatch.setattr(warm, 'warm_internal_model', lambda *args: calls.append(args))
  warm.warm_internal_model_from_camera(model, SimpleNamespace(recv=lambda: next(buffers)))
  assert calls == [(model, {'img': frame, 'big_img': frame})]


def test_missing_startup_camera_has_bounded_wait(monkeypatch):
  from openpilot.selfdrive.modeld import internal_model_warmup as warm
  clock = [0.]
  def receive():
    clock[0] += .1
    return None
  monkeypatch.setattr(warm.time, 'monotonic', lambda: clock[0])
  with pytest.raises(RuntimeError, match='camera buffer unavailable'):
    warm.warm_internal_model_from_camera(object(), SimpleNamespace(recv=receive))
  assert 2 <= clock[0] < 2.2


@pytest.mark.parametrize('failure', [False, True])
def test_warmup_executes_full_inference_then_discards_synthetic_history(failure):
  original_shadow, image = object(), object()
  calls, resets = [], []
  def run(bufs, transforms, inputs, prepare_only):
    assert model.signal_shadow is None and not prepare_only
    assert bufs == {'img': image, 'big_img': image}
    np.testing.assert_array_equal(transforms['img'], np.eye(3))
    assert not inputs['desire_pulse'].any()
    calls.append(True)
    model.history = 99
    if failure:
      raise RuntimeError('warmup failure')
    return {'output': True}
  def reset():
    model.history = 0
    resets.append(True)
  model = SimpleNamespace(run=run, reset=reset, history=0, signal_shadow=original_shadow)
  if failure:
    with pytest.raises(RuntimeError, match='warmup failure'):
      warm_internal_model(model, {'img': image, 'big_img': image})
  else:
    warm_internal_model(model, {'img': image, 'big_img': image})
  assert len(calls) == (1 if failure else 3)
  assert resets == [True] and model.history == 0 and model.signal_shadow is original_shadow
