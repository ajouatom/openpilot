"""Execute the production worker, reset and catch-up paths on a built artifact."""
import argparse
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.modeld.cheddar_model import CheddarModelState
from openpilot.selfdrive.modeld.internal_model_warmup import warm_internal_model


def check(path, width, height):
  model = CheddarModelState(width, height, path)
  try:
    frames = {name: SimpleNamespace(data=np.full(model.frame_size, 128, np.uint8)) for name in model.vision_input_names}
    transforms = {name: np.diag([width / 512, height / 256, 1]).astype(np.float32) for name in frames}
    inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
              'action_t': np.array([.3, .3], np.float32)}
    warm_internal_model(model, frames)
    first = model.run(frames, transforms, inputs, False)['raw_pred']
    assert first.shape == (2580,) and np.isfinite(first).all()
    inputs['desire_pulse'][1] = 1
    frames['img'].data[:] = 64
    changed = model.run(frames, transforms, inputs, True)['raw_pred']
    assert np.isfinite(changed).all() and not np.array_equal(first, changed)
    model.reset()
    inputs['desire_pulse'][:] = 0
    frames['img'].data[:] = 128
    np.testing.assert_array_equal(model.run(frames, transforms, inputs, False)['raw_pred'], first)
    print(f'Cheddar {width}x{height}: warmup, changed input, full catch-up and reset PASS', flush=True)
  finally:
    model.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('model', type=Path)
  args = parser.parse_args()
  os.environ['SEND_RAW_PRED'] = '1'
  for size in ((1928, 1208), (1344, 760)):
    check(args.model, *size)
