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
import logging
import os
import time
from contextlib import ExitStack
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


def _release_resource(release, resource) -> None:
  # 생성 실패의 원래 예외를 보존하면서 이미 확보한 나머지 자원도 계속 회수한다.
  try:
    release(resource)
  except Exception as error:  # 생성 실패의 원예외를 보존하고 나머지 회수를 계속한다.
    logging.getLogger('jetlink.trt').warning('CUDA cleanup failed (%s)', type(error).__name__)


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
    # 외부로 엔진을 넘기기 전까지 획득 직후 회수 책임을 등록한다.
    with ExitStack() as cleanup:
      self.stream = cudart.stream_create()
      cleanup.callback(_release_resource, cudart.stream_destroy, self.stream)

      self.bindings: dict[str, Binding] = {}
      for i in range(self.engine.num_io_tensors):
        name = self.engine.get_tensor_name(i)
        is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
        shape = tuple(self.engine.get_tensor_shape(name))
        if any(d < 0 for d in shape):
          raise RuntimeError(f"tensor {name} has a dynamic shape {shape}; jetlink builds fixed-shape engines")
        dtype = np.dtype(TRT_TO_NP[self.engine.get_tensor_dtype(name)])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        dev = cudart.malloc(nbytes)
        cleanup.callback(_release_resource, cudart.free, dev)
        host = cudart.host_alloc(nbytes)
        cleanup.callback(_release_resource, cudart.host_free, host)
        self.bindings[name] = Binding(name, shape, dtype, nbytes, int(dev), int(host),
                                      _np_from_ptr(int(host), shape, dtype), is_input)
        self.context.set_tensor_address(name, int(dev))

      self.inputs = {n: b for n, b in self.bindings.items() if b.is_input}
      self.outputs = {n: b for n, b in self.bindings.items() if not b.is_input}
      self.last_gpu_us = 0
      self.graph_exec = None
      self._state_host_sources = {}
      self.device_state = None
      # 정상 반환 뒤의 소유권과 close 계약은 원형 그대로다.
      cleanup.pop_all()

  # -- introspection --------------------------------------------------------

  @property
  def input_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: b.shape for n, b in self.inputs.items()}

  @property
  def output_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: b.shape for n, b in self.outputs.items()}

  def host_input(self, name: str) -> np.ndarray:
    """Pinned array for an input. Write into it, then call run()."""
    return self._state_host_sources.get(name, self.inputs[name]).host

  def configure_state_feedback(self, pairs):
    if self.graph_exec is not None:
      raise RuntimeError('State staging must be configured before CUDA graph capture')
    from jetlink.server.backends.trt.state_buffers import feedback_sources
    self._state_host_sources = feedback_sources(self.inputs, self.outputs, pairs)
    # Stateful NAV keeps recurrence on the GPU by default. Explicit 0 retains
    # host staging for diagnosis; queued models never configure this path.
    if os.getenv('CARROT_TRT_DEVICE_STATE', '1') == '1':
      if self.device_state is not None:
        raise RuntimeError('Device state already configured')
      from jetlink.server.backends.trt.device_state import DeviceState
      self.device_state = DeviceState(self.inputs, self.outputs, pairs, self.stream)

  # -- execution ------------------------------------------------------------

  def load_inputs(self, values: dict[str, np.ndarray]) -> None:
    if self.device_state is not None:
      raise RuntimeError('Device recurrence requires queue-managed inputs; explicit state upload is unsupported')
    from jetlink.server.backends.base import load_inputs
    load_inputs(self, values)

  def _enqueue(self, *, capturing=False) -> None:
    for b in self.inputs.values():
      if self.device_state is not None and b.name in self.device_state.input_names:
        continue
      # All H2D copies complete in this stream before execution, and D2H
      # overwrites the host outputs only afterwards. Reuse the previous state
      # output's pinned staging memory; keep distinct GPU input/output buffers.
      source = self._state_host_sources.get(b.name, b)
      cudart.memcpy_h2d_async(b.device_ptr, source.host_ptr, b.nbytes, self.stream)
    if not self.context.execute_async_v3(self.stream):
      raise RuntimeError("execute_async_v3 failed")
    for b in self.outputs.values():
      if self.device_state is not None and b.name in self.device_state.output_names:
        continue
      cudart.memcpy_d2h_async(b.host_ptr, b.device_ptr, b.nbytes, self.stream)
    if self.device_state is not None:
      self.device_state.enqueue_check()
      cudart.event_record(self.device_state.reply, self.stream, external=capturing)
      self.device_state.enqueue_feedback()

  def capture_graph(self) -> bool:
    """Capture the per-frame sequence into a CUDA graph.

    A replay skips the per-launch CPU work, worth a couple of ms and most of the
    launch jitter. Valid only because no buffer ever moves. Call after at least
    one warm run.
    """
    if self.graph_exec is not None:
      return True
    capturing = False
    graph = None
    try:
      cudart.stream_begin_capture(self.stream)
      capturing = True
      self._enqueue(capturing=True)
      capturing = False  # EndCapture also terminates an invalidated capture.
      graph = cudart.stream_end_capture(self.stream)
      self.graph_exec = cudart.graph_instantiate(graph)
      return True
    except Exception:
      if capturing:
        try:
          graph = cudart.stream_end_capture(self.stream)
        except Exception:
          pass
      # Not fatal: fall back to enqueueing each frame.
      self.graph_exec = None
      if self.device_state is not None:
        # A device-state CUDA failure must not silently appear validated.
        raise
      return False
    finally:
      if graph is not None:
        cudart.graph_destroy(graph)

  def warm(self) -> str:
    """One plain run so lazy CUDA state is paid for, then the graph, then one
    replay so the steady state is never the first thing a frame does."""
    self.run()
    if self.device_state is not None:
      self.device_state.validate()
      cudart.stream_sync(self.stream)  # trailing feedback before capture
    if self.capture_graph():
      self.run()
      if self.device_state is not None:
        self.device_state.validate()
      return ('cuda graph captured; device state, finite scan and reply event enabled'
              if self.device_state is not None else 'cuda graph captured')
    return 'cuda graph unavailable, enqueueing per frame'

  def run(self) -> dict[str, np.ndarray]:
    """Run one frame. Returns views over pinned output memory, valid until the next run."""
    t0 = time.perf_counter()
    if self.device_state is not None:
      self.device_state.before_run()
    if self.graph_exec is not None:
      cudart.graph_launch(self.graph_exec, self.stream)
    else:
      self._enqueue()
    if self.device_state is not None:
      cudart.event_sync(self.device_state.reply)
    else:
      cudart.stream_sync(self.stream)
    self.last_gpu_us = int((time.perf_counter() - t0) * 1e6)
    return {n: b.host for n, b in self.outputs.items()
            if self.device_state is None or n not in self.device_state.output_names}

  def infer(self, values: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    self.load_inputs(values)
    return self.run()

  # No __del__: host_input() hands out views over cudaHostAlloc memory with no
  # reference back here, so a GC-driven close would free pages someone is still
  # writing into. Whoever swaps an engine out closes it.
  def close(self) -> None:
    # A reply event can finish before the trailing GPU feedback copy.
    if self.device_state is not None:
      cudart.stream_sync(self.stream)
    if self.graph_exec is not None:
      cudart.graph_exec_destroy(self.graph_exec)
      self.graph_exec = None
    if self.device_state is not None:
      self.device_state.close()
      self.device_state = None
    for b in self.bindings.values():
      try:
        cudart.free(b.device_ptr)
        cudart.host_free(b.host_ptr)
      except Exception:
        pass
    self.bindings.clear()
    try:
      cudart.stream_destroy(self.stream)
    except Exception:
      pass
