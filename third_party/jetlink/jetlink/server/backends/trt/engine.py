"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

TensorRT execution.

Device buffers, pinned host buffers and the stream are all preallocated at load
time, so the steady state never allocates: the 50 ms frame budget is broken by
the tail, not the mean. Callers write into the pinned arrays `host_input()`
hands out, which saves a copy on the way to the GPU.
"""
from __future__ import annotations

import ctypes
import time
from dataclasses import dataclass

import numpy as np
import tensorrt as trt

from jetlink.server.backends.trt import cudart

TRT_TO_NP = {
  trt.DataType.FLOAT: np.float32,
  trt.DataType.HALF: np.float16,
  trt.DataType.INT8: np.int8,
  trt.DataType.INT32: np.int32,
  trt.DataType.INT64: np.int64,
  trt.DataType.BOOL: np.bool_,
  trt.DataType.UINT8: np.uint8,
}


@dataclass
class Binding:
  name: str
  shape: tuple[int, ...]
  dtype: np.dtype
  nbytes: int
  device_ptr: int
  host_ptr: int
  host: np.ndarray
  is_input: bool


def _np_from_ptr(ptr: int, shape, dtype) -> np.ndarray:
  """Writable numpy view over pinned host memory, so writes need no further copy.

  Through a raw byte buffer, because np.ctypeslib.as_ctypes_type cannot
  represent float16, which is most of this model.
  """
  dtype = np.dtype(dtype)
  nbytes = int(np.prod(shape)) * dtype.itemsize
  buf = (ctypes.c_char * nbytes).from_address(ptr)
  return np.frombuffer(buf, dtype=dtype).reshape(shape)


class TrtEngine:
  def __init__(self, plan_path: str, log_severity=trt.Logger.WARNING, device: int | None = None):
    self.logger = trt.Logger(log_severity)
    # Absent on a TensorRT that dropped the V2 plugin family; the driving
    # models use no plugins either way.
    init_plugins = getattr(trt, 'init_libnvinfer_plugins', None)
    if init_plugins is not None:
      init_plugins(self.logger, '')
    if device is not None:
      cudart.set_device(device)
    self.runtime = trt.Runtime(self.logger)
    with open(plan_path, 'rb') as f:
      self.engine = self.runtime.deserialize_cuda_engine(f.read())
    if self.engine is None:
      raise RuntimeError(f"failed to deserialize engine {plan_path}")
    self.context = self.engine.create_execution_context()
    self.stream = cudart.stream_create()

    self.bindings: dict[str, Binding] = {}
    for i in range(self.engine.num_io_tensors):
      name = self.engine.get_tensor_name(i)
      is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
      shape = tuple(self.engine.get_tensor_shape(name))
      if any(d < 0 for d in shape):
        raise RuntimeError(f"tensor {name} has a dynamic shape {shape}; "
                           "jetlink builds fixed-shape engines")
      dtype = np.dtype(TRT_TO_NP[self.engine.get_tensor_dtype(name)])
      nbytes = int(np.prod(shape)) * dtype.itemsize
      dev = cudart.malloc(nbytes)
      host = cudart.host_alloc(nbytes)
      self.bindings[name] = Binding(name, shape, dtype, nbytes, int(dev), int(host),
                                    _np_from_ptr(int(host), shape, dtype), is_input)
      self.context.set_tensor_address(name, int(dev))

    self.inputs = {n: b for n, b in self.bindings.items() if b.is_input}
    self.outputs = {n: b for n, b in self.bindings.items() if not b.is_input}
    self.last_gpu_us = 0
    self.graph_exec = None

  # -- introspection --------------------------------------------------------

  @property
  def input_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: b.shape for n, b in self.inputs.items()}

  @property
  def output_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: b.shape for n, b in self.outputs.items()}

  def host_input(self, name: str) -> np.ndarray:
    """Pinned array for an input. Write into it, then call run()."""
    return self.inputs[name].host

  # -- execution ------------------------------------------------------------

  def load_inputs(self, values: dict[str, np.ndarray]) -> None:
    from jetlink.server.backends.base import load_inputs
    load_inputs(self, values)

  def _enqueue(self) -> None:
    for b in self.inputs.values():
      cudart.memcpy_h2d_async(b.device_ptr, b.host_ptr, b.nbytes, self.stream)
    if not self.context.execute_async_v3(self.stream):
      raise RuntimeError("execute_async_v3 failed")
    for b in self.outputs.values():
      cudart.memcpy_d2h_async(b.host_ptr, b.device_ptr, b.nbytes, self.stream)

  def capture_graph(self) -> bool:
    """Capture the per-frame sequence into a CUDA graph.

    A replay skips the per-launch CPU work, worth a couple of ms and most of the
    launch jitter. Valid only because no buffer ever moves. Call after at least
    one warm run.
    """
    if self.graph_exec is not None:
      return True
    try:
      cudart.stream_begin_capture(self.stream)
      self._enqueue()
      graph = cudart.stream_end_capture(self.stream)
      self.graph_exec = cudart.graph_instantiate(graph)
      cudart.graph_destroy(graph)
      return True
    except Exception:
      # Not fatal: fall back to enqueueing each frame.
      self.graph_exec = None
      return False

  def warm(self) -> str:
    """One plain run so lazy CUDA state is paid for, then the graph, then one
    replay so the steady state is never the first thing a frame does."""
    self.run()
    if self.capture_graph():
      self.run()
      return 'cuda graph captured'
    return 'cuda graph unavailable, enqueueing per frame'

  def run(self) -> dict[str, np.ndarray]:
    """Run one frame. Returns views over pinned output memory, valid until the next run."""
    t0 = time.perf_counter()
    if self.graph_exec is not None:
      cudart.graph_launch(self.graph_exec, self.stream)
    else:
      self._enqueue()
    cudart.stream_sync(self.stream)
    self.last_gpu_us = int((time.perf_counter() - t0) * 1e6)
    return {n: b.host for n, b in self.outputs.items()}

  def infer(self, values: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    self.load_inputs(values)
    return self.run()

  # No __del__: host_input() hands out views over cudaHostAlloc memory with no
  # reference back here, so a GC-driven close would free pages someone is still
  # writing into. Whoever swaps an engine out closes it.
  def close(self) -> None:
    for b in self.bindings.values():
      try:
        cudart.free(b.device_ptr)
        cudart.host_free(b.host_ptr)
      except Exception:
        pass
    self.bindings.clear()
    if self.graph_exec is not None:
      cudart.graph_exec_destroy(self.graph_exec)
      self.graph_exec = None
    try:
      cudart.stream_destroy(self.stream)
    except Exception:
      pass
