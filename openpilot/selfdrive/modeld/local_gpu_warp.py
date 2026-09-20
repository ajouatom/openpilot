"""C3 pre-upload warp using the model artifact's own pinned warp implementation."""
import os
import json
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


def warp_difference(actual, expected):
  detail = {'actual_shape': list(actual.shape), 'expected_shape': list(expected.shape),
            'actual_dtype': str(actual.dtype), 'expected_dtype': str(expected.dtype)}
  if actual.shape != expected.shape or actual.dtype != expected.dtype:
    return detail
  indices = np.flatnonzero(actual.reshape(-1) != expected.reshape(-1))
  if not len(indices):
    return None
  detail.update(mismatched_pixels=int(indices.size), total_pixels=int(actual.size),
                max_abs_error=int(np.abs(actual.astype(np.int16) - expected.astype(np.int16)).max()),
                samples=[{'index': list(map(int, np.unravel_index(int(i), actual.shape))),
                          'qcom': int(actual.reshape(-1)[i]), 'amd': int(expected.reshape(-1)[i])} for i in indices[:8]])
  return detail


def only_sampling_boundary_differences(actual, expected, raw, frames_offset, frame_size, frame_info, camera_size, matrices):
  """Explain every differing value from the original NV12 pixels, not a % tolerance.

  Different float32 GPU division/FMA implementations can straddle a nearest
  neighbour half-pixel boundary. Permit only adjacent samples within 0.00025
  source pixels of that boundary; both results must match those source values.
  This bound covers the reproduced EV9 samples (maximum distance 0.000124).
  No arbitrary intensity error, layout error or non-boundary difference passes.
  """
  if actual.shape != (2, 6, 128, 256) or expected.shape != actual.shape or actual.dtype != np.uint8 or expected.dtype != np.uint8:
    return False
  stride, y_height = frame_info[:2]
  width, height = camera_size
  for camera, channel, row, col in np.argwhere(actual != expected):
    uv = channel >= 4
    x, y = (col, row) if uv else (2 * col + channel // 2, 2 * row + channel % 2)
    matrix = matrices[camera].astype(np.float64)
    if uv:
      matrix *= np.array([[1, 1, .5], [1, 1, .5], [2, 2, 1]])
    projected = matrix @ [x, y, 1]
    if not np.isfinite(projected).all() or abs(projected[2]) < .5:
      return False
    source = projected[:2] / projected[2]
    candidates, boundary = [], False
    for value, limit in zip(source, (width // 2, height // 2) if uv else (width, height), strict=True):
      lo = np.floor(value)
      near = abs(value - lo - .5) <= .00025
      boundary |= near
      coords = (int(lo), int(lo + 1)) if near else (int(np.rint(value)),)
      candidates.append({min(max(c, 0), limit - 1) for c in coords})
    if not boundary:
      return False
    values = set()
    for sy in candidates[1]:
      for sx in candidates[0]:
        offset = stride * y_height + sy * stride + 2 * sx + channel - 4 if uv else sy * stride + sx
        values.add(int(raw[frames_offset + camera * frame_size + offset]))
    if int(actual[camera, channel, row, col]) not in values or int(expected[camera, channel, row, col]) not in values:
      return False
  return True


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
    self.camera_size, self.frame_info = (width, height), frame_info
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
    differences = []
    try:
      self.raw[:] = np.random.default_rng(0).integers(0, 256, self.raw.size, dtype=np.uint8)
      probes = [('identity', np.eye(3)),
                ('projective', [[2.3, .01, 20.2], [-.02, 2.1, 40.3], [.0001, -.0002, 1]]),
                ('border', [[1, 0, -200], [0, 1, -100], [0, 0, 1]])]
      for name, matrix in probes:
        matrices[:] = matrix
        reference.copy_from(self.local_host)
        expected = self.amd_warp(input_frame=frames, M_inv=transforms).numpy()
        actual = self.prepare_images()
        if expected.dtype != np.uint8 or expected.shape != tuple(self.specs['new_img'][0]):
          raise RuntimeError('artifact AMD warp has unexpected output contract')
        if (difference := warp_difference(actual, expected)) is not None:
          difference['probe'] = name
          difference['repeat_matches_first'] = bool(np.array_equal(actual, self.prepare_images()))
          explained = difference['repeat_matches_first'] and only_sampling_boundary_differences(
            actual, expected, self.raw, self.frames_offset, self.frame_size, self.frame_info, self.camera_size, matrices)
          difference['sampling_boundary_only'] = explained
          if explained:
            from openpilot.common.swaglog import cloudlog
            cloudlog.event('precompiledWarpSamplingBoundary', **difference)
          else:
            differences.append(difference)
      if differences:
        raise RuntimeError('QCOM pre-upload warp differs from artifact AMD warp: ' + json.dumps(differences))
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
