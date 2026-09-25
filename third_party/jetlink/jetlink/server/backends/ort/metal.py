"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Keep Metal active between latency-sensitive CoreML predictions.

On an M2 Pro, pauses in a 20 Hz stream caused GPU clocks to fall and frame
times to exceed 50 ms, despite continuous inference taking only 34 ms. A
small, independent GPU workload prevents that downclocking. It uses no model
buffers and only one command at a time; idle workers submit no GPU work.
"""
from __future__ import annotations

import ctypes as C
import logging
import os
import sys
import threading
import time

log = logging.getLogger('jetlink.ort.metal')
IDLE_SECONDS = 1.0
# Measured at about 0.3 ms at full clock on M2 Pro. A finite loop also bounds
# work if the CPU stops submitting. Only a single 32-thread group is needed.
ROUNDS = 10_000
SHADER = b'''
#include <metal_stdlib>
using namespace metal;
kernel void keep_active(device uint *out [[buffer(0)]],
                        constant uint &rounds [[buffer(1)]],
                        uint tid [[thread_position_in_grid]]) {
  uint x = out[tid];
  for (uint i = 0; i < rounds; i++) x = x * 1664525u + 1013904223u;
  out[tid] = x;
}
'''


class _Size(C.Structure):
  _fields_ = [('width', C.c_size_t), ('height', C.c_size_t), ('depth', C.c_size_t)]


class _MetalWork:
  """A private Metal queue, used and destroyed on its owning Python thread.

  Direct, fixed-signature Objective-C calls avoid a PyObjC dependency or a
  native build step for the Python package. Metal compiles its own shader.
  """

  def __init__(self):
    self._owned = []
    self._foundation = C.CDLL('/System/Library/Frameworks/Foundation.framework/Foundation')
    self._metal = C.CDLL('/System/Library/Frameworks/Metal.framework/Metal')
    self._objc = C.CDLL('/usr/lib/libobjc.A.dylib')
    self._objc.objc_getClass.argtypes = [C.c_char_p]
    self._objc.objc_getClass.restype = C.c_void_p
    self._objc.sel_registerName.argtypes = [C.c_char_p]
    self._objc.sel_registerName.restype = C.c_void_p
    self._metal.MTLCreateSystemDefaultDevice.argtypes = []
    self._metal.MTLCreateSystemDefaultDevice.restype = C.c_void_p
    ptr, size = C.c_void_p, C.c_size_t
    self._new = self._method(ptr, b'new')
    self._release = self._method(None, b'release')
    self._drain = self._method(None, b'drain')
    self._pool_class = self._objc.objc_getClass(b'NSAutoreleasePool')
    string_class = self._objc.objc_getClass(b'NSString')
    string = self._method(ptr, b'stringWithUTF8String:', C.c_char_p)
    self._description = self._method(ptr, b'localizedDescription')
    self._utf8 = self._method(C.c_char_p, b'UTF8String')
    self._command = self._method(ptr, b'commandBuffer')
    self._encoder = self._method(ptr, b'computeCommandEncoder')
    self._set_pipeline = self._method(None, b'setComputePipelineState:', ptr)
    self._set_buffer = self._method(None, b'setBuffer:offset:atIndex:', ptr, size, size)
    self._set_bytes = self._method(None, b'setBytes:length:atIndex:', ptr, size, size)
    self._dispatch = self._method(None, b'dispatchThreadgroups:threadsPerThreadgroup:', _Size, _Size)
    self._end = self._method(None, b'endEncoding')
    self._commit = self._method(None, b'commit')
    self._wait = self._method(None, b'waitUntilCompleted')
    self._status = self._method(size, b'status')
    self._error = self._method(ptr, b'error')
    self._rounds = C.c_uint(ROUNDS)

    pool = self._new(self._pool_class)
    try:
      device = self._own(self._metal.MTLCreateSystemDefaultDevice(), 'Metal device unavailable')
      error = ptr()
      library = self._method(ptr, b'newLibraryWithSource:options:error:', ptr, ptr, C.POINTER(ptr))(
        device, string(string_class, SHADER), None, C.byref(error))
      library = self._own(library, self._error_text(error))
      function = self._own(self._method(ptr, b'newFunctionWithName:', ptr)(
        library, string(string_class, b'keep_active')), 'Metal function unavailable')
      pipeline = self._method(ptr, b'newComputePipelineStateWithFunction:error:', ptr, C.POINTER(ptr))(
        device, function, C.byref(error))
      self._pipeline = self._own(pipeline, self._error_text(error))
      self._queue = self._own(self._method(ptr, b'newCommandQueue')(device), 'Metal queue unavailable')
      # MTLResourceStorageModeShared = 0. This buffer is unrelated to model IO.
      self._buffer = self._own(self._method(ptr, b'newBufferWithLength:options:', size, size)(device, 128, 0),
                               'Metal buffer unavailable')
    except Exception:
      self.close()
      raise
    finally:
      self._drain(pool)

  def _method(self, result, name: bytes, *args):
    # objc_msgSend is variadic in C, but Apple silicon needs the exact ABI of
    # each method, including the by-value MTLSize structs used for dispatch.
    function = C.CFUNCTYPE(result, C.c_void_p, C.c_void_p, *args)(('objc_msgSend', self._objc))
    selector = self._objc.sel_registerName(name)
    return lambda obj, *values: function(obj, selector, *values)

  def _error_text(self, error) -> str:
    if error:
      text = self._utf8(self._description(error))
      if text:
        return text.decode('utf-8', errors='replace')
    return 'Metal keep-alive initialization failed'

  def _own(self, obj, error: str):
    if not obj:
      raise RuntimeError(error)
    self._owned.append(obj)
    return obj

  def run(self) -> None:
    pool = self._new(self._pool_class)
    try:
      command = self._command(self._queue)
      if not command:
        raise RuntimeError('Metal command buffer unavailable')
      encoder = self._encoder(command)
      if not encoder:
        raise RuntimeError('Metal compute encoder unavailable')
      self._set_pipeline(encoder, self._pipeline)
      self._set_buffer(encoder, self._buffer, 0, 0)
      self._set_bytes(encoder, C.byref(self._rounds), C.sizeof(self._rounds), 1)
      self._dispatch(encoder, _Size(1, 1, 1), _Size(32, 1, 1))
      self._end(encoder)
      self._commit(command)
      # Never enqueue a backlog or overlap keep-alive commands. ctypes
      # releases the GIL here, so inference can continue on the worker thread.
      self._wait(command)
      if self._status(command) != 4:  # MTLCommandBufferStatusCompleted
        raise RuntimeError(self._error_text(self._error(command)))
    finally:
      self._drain(pool)

  def close(self) -> None:
    for obj in reversed(self._owned):
      self._release(obj)
    self._owned.clear()


class MetalKeepAlive:
  """An idle-expiring stream lease; all Metal objects belong to one thread."""

  def __init__(self):
    self._condition = threading.Condition()
    self._ready = threading.Event()
    self._idle = threading.Event()
    self._deadline = 0.0
    self._closed = False
    self._error = None
    self._thread = threading.Thread(target=self._loop, name='jetlink-metal-keepalive', daemon=True)
    self._thread.start()
    if not self._ready.wait(15):
      self.close()
      raise RuntimeError('Metal keep-alive initialization timed out')
    if self._error is not None:
      self.close()
      raise RuntimeError(f'Metal keep-alive initialization failed: {self._error}')

  def pulse(self) -> None:
    with self._condition:
      if not self._closed and self._error is None:
        self._deadline = time.monotonic() + IDLE_SECONDS
        self._idle.clear()
        self._condition.notify()

  def pause(self) -> None:
    with self._condition:
      self._deadline = 0.0
      self._condition.notify()

  def close(self) -> None:
    with self._condition:
      self._closed = True
      self._condition.notify()
    # The enclosing worker process has its own bounded shutdown if a driver
    # stalls. Do not let an optional helper hold its control pipe indefinitely.
    self._thread.join(2)

  def _loop(self) -> None:
    work = None
    try:
      work = _MetalWork()
      self._ready.set()
      while True:
        with self._condition:
          while not self._closed and time.monotonic() >= self._deadline:
            self._idle.set()
            self._condition.wait()
          if self._closed:
            return
        work.run()
    except Exception as e:
      with self._condition:
        self._error = e
      if self._ready.is_set():
        log.warning('Metal keep-alive stopped; continuing inference without it: %s', e)
    finally:
      try:
        if work is not None:
          work.close()
      finally:
        self._idle.set()
        self._ready.set()


def create_keepalive(sessions: list[tuple[str, list]]) -> MetalKeepAlive | None:
  if sys.platform != 'darwin' or os.environ.get('JETLINK_METAL_KEEPALIVE', '1') == '0':
    return None
  # The ANE path has different power/performance behavior. Only enable this
  # for the CPUAndGPU configuration measured with a paced frame stream.
  units = [p[1].get('MLComputeUnits') if isinstance(p, tuple) else None
           for _, providers in sessions for p in providers
           if (p[0] if isinstance(p, tuple) else p) == 'CoreMLExecutionProvider']
  if not units or any(unit != 'CPUAndGPU' for unit in units):
    return None
  try:
    return MetalKeepAlive()
  except (OSError, AttributeError, RuntimeError) as e:
    log.warning('Metal keep-alive unavailable; continuing inference without it: %s', e)
    return None
