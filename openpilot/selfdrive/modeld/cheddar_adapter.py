"""Official state aliases and transfers for the internal Cheddar model."""
import pickle
import numpy as np

from openpilot.selfdrive.modeld.cheddar_runtime import CHECKPOINT, MODEL_SHA256, REVISION
from openpilot.selfdrive.modeld.generic_model_runtime import GenericModelRuntime, host_input_layout, input_view, model_metadata
from openpilot.selfdrive.modeld.precompiled_artifact import prepare_jit


class CheddarAdapter(GenericModelRuntime):
  def __init__(self, bundle, width, height):
    from tinygrad import Tensor
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

    if bundle.get('model_sha256') != MODEL_SHA256 or bundle.get('runtime_revision') != REVISION:
      raise ValueError('internal model/runtime identity mismatch')
    self.device = bundle['input_specs']['new_img'][2]
    if self.device.split(':')[0] not in ('QCOM', 'CPU', 'METAL', 'CUDA', 'NV'):
      raise ValueError('unsupported internal model device')
    self.checkpoint, self.output_slices, self.state_pairs, self.count = model_metadata(bundle, device=self.device)
    if self.checkpoint != CHECKPOINT or self.count != 2580:
      raise ValueError('internal model output contract mismatch')
    self.specs = bundle['input_specs']
    expected = {'state_img_q': ((2, 5, 6, 128, 256), 'uint8'),
                'state_desire_q': ((132, 1, 8), 'float32'), 'state_feat_q': ((128, 1, 512), 'float32')}
    if set(self.state_pairs) != set(expected) or any(
      tuple(self.specs[name][0]) != shape or np.dtype(self.specs[name][1]) != np.dtype(dtype)
      for name, (shape, dtype) in expected.items()):
      raise ValueError('internal model recurrent contract mismatch')
    stride, yh, uvh, _ = get_nv12_info(width, height)
    self.frame_size = stride * (yh + uvh)
    self.input_shapes = {'img': [self.frame_size], 'big_img': [self.frame_size]}
    self.packed, self.views, self.frames_offset = host_input_layout(self.specs, self.frame_size, self.state_pairs)
    self.queues = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                   for name, (shape, dtype, device) in self.specs.items() if name in self.state_pairs}
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in bundle['output_specs'].items() if name not in self.state_pairs.values()}
    for name, next_name in self.state_pairs.items():
      state = self.queues[name]
      self.outputs[next_name] = input_view(state._buffer(), state.shape, state.dtype)
    self.run_model = prepare_jit(bundle['run'])
    self.run_warp = prepare_jit(pickle.loads(bundle['warps'][(width, height)])['run'])
    self.compiled_copies = True
