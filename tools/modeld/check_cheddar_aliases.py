"""Compare actual compiled Cheddar aliases with separate state destinations."""
import argparse
from pathlib import Path

import numpy as np

from openpilot.selfdrive.modeld.cheddar_runtime import activate, load_bundle


def check(path):
  activate(path.parent / '.cheddar-runtime')
  from tinygrad import Tensor
  from openpilot.selfdrive.modeld.cheddar_adapter import CheddarAdapter

  candidate = CheddarAdapter(load_bundle(path), 1928, 1208)
  reference = CheddarAdapter(load_bundle(path), 1928, 1208)
  for adapter in (candidate, reference):
    adapter.bind_shared(adapter.packed)
    adapter.views['traffic_convention'][:] = [1, 0]
    adapter.views['action_t'][:] = [.3, .3]
    for name in ('tfm', 'big_tfm'):
      adapter.views[name][:] = np.diag([1928 / 512, 1208 / 256, 1]).astype(np.float32)
  for name, next_name in reference.state_pairs.items():
    shape, dtype, device = reference.specs[name]
    reference.outputs[next_name] = Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
  for index in range(40):
    for adapter in (candidate, reference):
      adapter.views['img'][:] = 80 + index % 11
      adapter.views['big_img'][:] = 120 + index % 7
      adapter.views['desire'][:] = 0
      adapter.views['desire'][1] = int(index % 15 == 0)
    actual, expected = candidate.run().copy(), reference.run().copy()
    for name, next_name in reference.state_pairs.items():
      reference.queues[name]._buffer().copy_from(reference.outputs[next_name]._buffer())
    np.testing.assert_array_equal(actual, expected, err_msg=f'recurrent frame {index}')
  print('Cheddar actual model: 40 aliased and separately copied recurrent frames match exactly', flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('model', type=Path)
  check(parser.parse_args().model)
