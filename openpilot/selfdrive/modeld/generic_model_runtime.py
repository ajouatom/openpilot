"""Adapter for comma's generic ONNX artifact, using its isolated pinned tinygrad."""
import base64
import math
import os
from pathlib import Path
import pickle
import time

import numpy as np


def input_view(buffer, shape, dtype, offset=0):
  from tinygrad import Tensor
  from tinygrad.uop.ops import UOp

  buf = buffer.view(math.prod(shape), dtype, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(buf)).reshape(shape)


def model_metadata(jits):
  metadata = jits['metadata']
  specs = jits['input_specs']
  outputs = jits['output_specs']
  if tuple(specs['new_img'][0]) != (2, 6, 128, 256) or np.dtype(specs['new_img'][1]) != np.uint8:
    raise ValueError('unsupported generic model image input')
  state_pairs = {name: f'next_{name}' for name in specs if f'next_{name}' in outputs}
  if set(specs) != {'new_img', 'desire', 'traffic_convention', 'action_t'} | set(state_pairs):
    raise ValueError('unsupported generic model inputs')
  for name, next_name in state_pairs.items():
    shape, dtype, device = specs[name]
    out_shape, out_dtype, out_device = outputs[next_name]
    if tuple(shape) != tuple(out_shape) or np.dtype(dtype) != np.dtype(out_dtype) or device != out_device:
      raise ValueError('incompatible generic model state feedback')
  if not state_pairs or any(spec[2] != 'AMD' for spec in list(specs.values()) + list(outputs.values())):
    raise ValueError('unsupported generic model device/state')
  slices = pickle.loads(base64.b64decode(metadata['metadata']['output_slices']))
  count = math.prod(outputs['outputs'][0])
  if any(s.start < 0 or s.stop > count or s.step not in (None, 1) for s in slices.values()):
    raise ValueError('invalid generic model output slices')
  return metadata['metadata']['model_checkpoint'], slices, state_pairs, count


class GenericModelRuntime:
  def __init__(self, jits, width, height, runtime_dir: Path, frame_info):
    # Imports must happen after the worker selects the artifact's tinygrad.
    from tinygrad import Tensor, Device
    from examples.openpilot.compile_warp import NV12Frame, compile_warp

    self.checkpoint, self.output_slices, self.state_pairs, self.count = model_metadata(jits)
    self.device = jits['input_specs']['new_img'][2]
    self.gpu_arch = Device[self.device].arch
    if self.gpu_arch != 'gfx1200':
      raise ValueError('precompiled GPU architecture mismatch')
    self.specs = jits['input_specs']
    self.run_model = jits['run']
    stride, y_height, uv_height = frame_info[:3]
    self.frame_size = stride * (y_height + uv_height)
    self.input_shapes = {'img': [self.frame_size], 'big_img': [self.frame_size]}
    shapes = {name: tuple(spec[0]) for name, spec in self.specs.items() if name not in self.state_pairs and name != 'new_img'}
    sizes = {name: (math.prod(shape) * 4 + 127) // 128 * 128 for name, shape in shapes.items()}
    self.packed = np.zeros(128 + sum(sizes.values()) + 2 * self.frame_size, np.uint8)
    self.views = {'tfm': np.ndarray((3, 3), np.float32, buffer=self.packed, offset=0),
                  'big_tfm': np.ndarray((3, 3), np.float32, buffer=self.packed, offset=36)}
    offset = 128
    for name, shape in shapes.items():
      self.views[name] = np.ndarray(shape, np.float32, buffer=self.packed, offset=offset)
      offset += sizes[name]
    self.frames_offset = offset
    for name in ('img', 'big_img'):
      self.views[name] = np.ndarray((self.frame_size,), np.uint8, buffer=self.packed, offset=offset)
      offset += self.frame_size
    self.queues = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                   for name, (shape, dtype, device) in self.specs.items() if name in self.state_pairs}
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in jits['output_specs'].items() if name not in self.state_pairs.values()}
    # Match upstream: next_state outputs alias the existing state input buffers.
    for name, next_name in self.state_pairs.items():
      state = self.queues[name]
      self.outputs[next_name] = input_view(state._buffer(), state.shape, state.dtype)
    warp_path = runtime_dir / f'warp-{self.gpu_arch}-{width}x{height}.pkl'
    if warp_path.is_file():
      with warp_path.open('rb') as f:
        warp = pickle.load(f)
    else:
      warp = compile_warp(NV12Frame(width, height, stride, y_height, uv_height, self.frame_size),
                          (512, 256), layout='yuv420', frames=2, benchmark_runs=1)
      temporary = warp_path.with_suffix('.tmp')
      with temporary.open('wb') as f:
        pickle.dump(warp, f)
      os.replace(temporary, warp_path)
    self.run_warp = warp['run']

  def bind_shared(self, packed):
    from tinygrad import Tensor
    from tinygrad.dtype import dtypes

    self.host = Tensor(packed, device='NPY')._buffer()
    self.device_buffer = Tensor(np.zeros_like(packed), device=self.device)._buffer()

    for name, value in self.views.items():
      if name not in ('img', 'big_img', 'tfm', 'big_tfm'):
        self.queues[name] = input_view(self.device_buffer, value.shape, dtypes.float32, value.ctypes.data - self.packed.ctypes.data)
    self.transforms = input_view(self.device_buffer, (2, 3, 3), dtypes.float32)
    self.frames = input_view(self.device_buffer, (2, self.frame_size), dtypes.uint8, self.frames_offset)

  def run(self):
    started, cpu_started = time.monotonic(), time.thread_time()
    self.device_buffer.copy_from(self.host)
    uploaded, cpu_uploaded = time.monotonic(), time.thread_time()
    self.queues['new_img'] = self.run_warp(input_frame=self.frames, M_inv=self.transforms)
    warped, cpu_warped = time.monotonic(), time.thread_time()
    self.run_model(output_buffers=self.outputs, **self.queues)
    dispatched, cpu_dispatched = time.monotonic(), time.thread_time()
    result = self.outputs['outputs'].numpy().reshape(-1)
    finished, cpu_finished = time.monotonic(), time.thread_time()
    # Call boundaries only: GPU work may finish in a later call. Do not insert
    # synchronizations that change pipelining or label these as kernel timings.
    # In particular, generic output download is inside adapter.run(), whereas
    # the worker's result_sync_ms only measures validation/shared-output copy.
    self.last_timings = {
      'input_upload_ms': (uploaded - started) * 1000,
      'input_upload_cpu_ms': (cpu_uploaded - cpu_started) * 1000,
      'warp_call_ms': (warped - uploaded) * 1000,
      'warp_call_cpu_ms': (cpu_warped - cpu_uploaded) * 1000,
      'model_call_ms': (dispatched - warped) * 1000,
      'model_call_cpu_ms': (cpu_dispatched - cpu_warped) * 1000,
      'output_read_ms': (finished - dispatched) * 1000,
      'output_read_cpu_ms': (cpu_finished - cpu_dispatched) * 1000,
    }
    return result
