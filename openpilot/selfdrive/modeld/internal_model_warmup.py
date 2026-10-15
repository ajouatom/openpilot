"""Prepare the standby model before live eGPU publication, without publishing."""
import numpy as np
import time


def warm_internal_model_from_camera(model, camera):
  # Synthetic inputs need one correctly allocated VisionIPC buffer, not a
  # synchronized live pair. The non-conflated extra stream can have an old
  # queue after the long external-model load; normal live pairing drains it.
  deadline = time.monotonic() + 2
  while time.monotonic() < deadline:
    buf = camera.recv()
    if buf is not None:
      warm_internal_model(model, dict.fromkeys(model.vision_input_names, buf))
      return
  raise RuntimeError('camera buffer unavailable for internal standby warmup')


def warm_internal_model(model, bufs):
  transforms = {name: np.eye(3, dtype=np.float32) for name in bufs}
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1., 0.], np.float32),
            'action_t': np.zeros(2, np.float32)}
  shadow, model.signal_shadow = model.signal_shadow, None
  try:
    # Exercise warp, policy dispatch and synchronous readback. Constructing the
    # model alone leaves expensive first-use work on the future fallback path.
    for _ in range(3):
      if model.run(bufs, transforms, inputs, False) is None:
        raise RuntimeError('internal standby warmup produced no output')
  finally:
    model.reset()  # synthetic warmup must not become live recurrent history
    model.signal_shadow = shadow
