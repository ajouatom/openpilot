"""Run the installed CD210 artifact through modeld's actual standby/fallback path."""
import argparse
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.modeld.internal_model_warmup import warm_internal_model
from openpilot.selfdrive.modeld.modeld import ModelState, SEND_RAW_PRED


def check(path, width, height):
  model = ModelState(width, height, False, path, enable_shadow=False)
  frames = {name: SimpleNamespace(data=np.full(model.frame_buf_params[name][3], 128, np.uint8))
            for name in model.vision_input_names}
  transforms = {name: np.diag([width / 512, height / 256, 1]).astype(np.float32) for name in frames}
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
            'action_t': np.array([.3, .3], np.float32)}
  warm_internal_model(model, frames)
  first = model.run(frames, transforms, inputs, False)['raw_pred']
  assert first.shape == (2576,) and np.isfinite(first).all()
  inputs['desire_pulse'][1] = 1
  frames['img'].data[:] = 64
  assert model.run(frames, transforms, inputs, True) is None
  changed = model.run(frames, transforms, inputs, False)['raw_pred']
  assert np.isfinite(changed).all() and not np.array_equal(first, changed)
  model.reset()
  inputs['desire_pulse'][:] = 0
  frames['img'].data[:] = 128
  np.testing.assert_array_equal(model.run(frames, transforms, inputs, False)['raw_pred'], first)
  print(f'CD210 {width}x{height}: warmup, changed input, catch-up and reset PASS', flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('model', type=Path)
  args = parser.parse_args()
  assert SEND_RAW_PRED, 'Run with SEND_RAW_PRED=1'
  for size in ((1928, 1208), (1344, 760)):
    check(args.model, *size)
