"""C3 pre-upload warp using the model artifact's own pinned warp implementation."""
import os
from pathlib import Path
import pickle
import time

import numpy as np

from openpilot.selfdrive.modeld.generic_model_runtime import GenericModelRuntime, input_view


def use_local_warp(device_type: str) -> bool:
  return device_type in ('tici', 'tizi')


def vehicle_device_type() -> str:
  try:
    return Path('/sys/firmware/devicetree/base/model').read_text().strip('\x00').split('comma ')[-1]
  except OSError:
    return ''


def create_runtime(args, device_type, report_failure):
  if use_local_warp(device_type):
    try:
      return LocalWarpRuntime(*args)
    except Exception:
      report_failure('QCOM pre-upload warp initialization failed; using AMD warp')
  return GenericModelRuntime(*args)


def bind_runtime(adapter, args, packed, report_failure):
  try:
    adapter.bind_shared(packed)
  except Exception:
    if not isinstance(adapter, LocalWarpRuntime):
      raise
    report_failure('QCOM pre-upload warp validation failed; using AMD warp')
    adapter = GenericModelRuntime(*args)
    adapter.bind_shared(packed)
  return adapter


class LocalWarpRuntime(GenericModelRuntime):
  """Keep the shared-input protocol, but transfer only prepared images to AMD.

  QCOM gets an explicit local copy each frame: mapping mutable CPU memory once
  with from_blob would not establish CPU/GPU cache coherence for later writes.
  No inference runs during validation, so recurrent state is not advanced.
  """
  def __init__(self, jits, width, height, runtime_dir, frame_info):
    from tinygrad import Context
    from examples.openpilot.compile_warp import NV12Frame, compile_warp

    super().__init__(jits, width, height, runtime_dir, frame_info)
    self.amd_warp = self.run_warp
    cache = runtime_dir / f'warp-qcom-preupload-v1-{width}x{height}.pkl'
    with Context(DEV='QCOM'):
      if cache.is_file():
        with cache.open('rb') as f:
          warp = pickle.load(f)
      else:
        warp = compile_warp(NV12Frame(width, height, *frame_info[:3], self.frame_size),
                            (512, 256), layout='yuv420', frames=2, benchmark_runs=1)
        temporary = cache.with_suffix('.tmp')
        with temporary.open('wb') as f:
          pickle.dump(warp, f)
        os.replace(temporary, cache)
    self.run_warp = warp['run']
    self.upload_bytes = self.frames_offset + int(np.prod(self.specs['new_img'][0]))

  def bind_shared(self, packed):
    from tinygrad import Tensor
    from tinygrad.dtype import dtypes

    self.raw = packed
    self.local_host = Tensor(packed, device='NPY')._buffer()
    self.local_buffer = Tensor(np.zeros_like(packed), device='QCOM')._buffer()
    self.frames = input_view(self.local_buffer, (2, self.frame_size), dtypes.uint8, self.frames_offset)
    self.transforms = input_view(self.local_buffer, (2, 3, 3), dtypes.float32)
    self.compact = np.zeros(self.upload_bytes, np.uint8)
    self.host = Tensor(self.compact, device='NPY')._buffer()
    self.device_buffer = Tensor(np.zeros_like(self.compact), device=self.device)._buffer()
    for name, value in self.views.items():
      if name not in ('img', 'big_img', 'tfm', 'big_tfm'):
        self.queues[name] = input_view(self.device_buffer, value.shape, dtypes.float32,
                                       value.ctypes.data - self.packed.ctypes.data)
    self.queues['new_img'] = input_view(self.device_buffer, tuple(self.specs['new_img'][0]), dtypes.uint8, self.frames_offset)
    self.validate_warp()

  def prepare_images(self):
    self.local_buffer.copy_from(self.local_host)
    return self.run_warp(input_frame=self.frames, M_inv=self.transforms).numpy()

  def validate_warp(self):
    """Compare actual QCOM/AMD kernels before accepting this device's path."""
    from tinygrad import Tensor
    from tinygrad.dtype import dtypes

    reference = Tensor(np.zeros_like(self.raw), device=self.device)._buffer()
    frames = input_view(reference, (2, self.frame_size), dtypes.uint8, self.frames_offset)
    transforms = input_view(reference, (2, 3, 3), dtypes.float32)
    matrices = np.ndarray((2, 3, 3), np.float32, buffer=self.raw)
    try:
      self.raw[:] = np.random.default_rng(0).integers(0, 256, self.raw.size, dtype=np.uint8)
      for matrix in (np.eye(3), [[2.3, .01, 20.2], [-.02, 2.1, 40.3], [.0001, -.0002, 1]],
                     [[1, 0, -200], [0, 1, -100], [0, 0, 1]]):
        matrices[:] = matrix
        reference.copy_from(self.local_host)
        expected = self.amd_warp(input_frame=frames, M_inv=transforms).numpy()
        actual = self.prepare_images()
        if actual.dtype != np.uint8 or actual.shape != tuple(self.specs['new_img'][0]) or not np.array_equal(actual, expected):
          raise RuntimeError('QCOM pre-upload warp differs from artifact AMD warp')
    finally:
      self.raw[:] = 0

  def run(self):
    started, cpu_started = time.monotonic(), time.thread_time()
    images = self.prepare_images()
    np.copyto(self.compact[:self.frames_offset], self.raw[:self.frames_offset])
    np.copyto(self.compact[self.frames_offset:], images.reshape(-1))
    prepared, cpu_prepared = time.monotonic(), time.thread_time()
    self.device_buffer.copy_from(self.host)
    uploaded, cpu_uploaded = time.monotonic(), time.thread_time()
    self.run_model(output_buffers=self.outputs, **self.queues)
    dispatched, cpu_dispatched = time.monotonic(), time.thread_time()
    result = self.outputs['outputs'].numpy().reshape(-1)
    finished, cpu_finished = time.monotonic(), time.thread_time()
    self.last_timings = {
      'local_prepare_ms': (prepared - started) * 1000,
      'local_prepare_cpu_ms': (cpu_prepared - cpu_started) * 1000,
      'input_upload_ms': (uploaded - prepared) * 1000,
      'input_upload_cpu_ms': (cpu_uploaded - cpu_prepared) * 1000,
      'model_call_ms': (dispatched - uploaded) * 1000,
      'model_call_cpu_ms': (cpu_dispatched - cpu_uploaded) * 1000,
      'output_read_ms': (finished - dispatched) * 1000,
      'output_read_cpu_ms': (cpu_finished - cpu_dispatched) * 1000,
    }
    return result
