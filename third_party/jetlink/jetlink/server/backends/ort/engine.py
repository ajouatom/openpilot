"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The server's side of an onnxruntime session running in a worker process.

`host_input()` hands out views into a shared-memory block the child has
attached, so the queues gather into the child's feeds directly; `run()` is a
message on a pipe and a wait. Why a child at all is in worker.py.
"""
from __future__ import annotations

import logging
import multiprocessing as mp
import time
from multiprocessing import shared_memory

import numpy as np

from jetlink.server.backends.base import IO
from jetlink.server.backends.ort import worker

log = logging.getLogger('jetlink.ort')

ORT_DTYPES = {
  'tensor(uint8)': np.uint8,
  'tensor(int32)': np.int32,
  'tensor(int64)': np.int64,
  'tensor(float16)': np.float16,
  'tensor(float)': np.float32,
  'tensor(double)': np.float64,
  'tensor(bool)': np.bool_,
}

# A frame that takes longer than this in the child is a dead child, not a slow
# model; the comma gave up on the link long before.
RUN_TIMEOUT = 30.0
# Tick this often while the child creates its session, so a caller waiting on
# a CoreML compile can report that it is still going. The CoreML stages read
# the bytes in the cache directory on every tick, which is a scandir walk of a
# few thousand entries: cheap beside a compile that writes gigabytes.
TICK = 2.0


class WorkerDied(RuntimeError):
  pass


class OrtEngine:
  def __init__(self, sessions: list[tuple[str, list]], device: str, log_severity: int = 3,
               on_tick=None):
    """Start the child and wait for its sessions, `[(model path, providers)]`
    run back to back. `on_tick(elapsed_s, worker_pid)` is called every couple
    of seconds meanwhile; the build and the load use it for progress, and the
    pid is how a load that writes nothing is measured."""
    self.device = device
    self.last_gpu_us = 0
    self._block = None
    self._proc = None
    ctx = mp.get_context('spawn')
    self._conn, child_conn = ctx.Pipe()
    self._proc = ctx.Process(target=worker.main, args=(child_conn, [(str(m), p) for m, p in sessions], log_severity),
                             name='jetlink-ort', daemon=True)
    self._proc.start()
    child_conn.close()
    try:
      t0 = time.monotonic()
      while not self._conn.poll(TICK):
        if not self._proc.is_alive():
          raise WorkerDied('the onnxruntime worker exited before its session was ready')
        if on_tick is not None:
          on_tick(time.monotonic() - t0, self._proc.pid)
      msg = self._recv()
      if msg[0] == 'error':
        raise RuntimeError(f"onnxruntime worker: {msg[1].strip()}")
      _, inputs, outputs = msg
      laid_in, size_in = worker.layout(inputs)
      # float32 out whatever the graph says, as the protocol carries it; the
      # child casts on the way into the block.
      laid_out, size_out = worker.layout([(n, s, 'float32') for n, s, _ in outputs])
      laid_out = [(n, s, d, off + size_in) for n, s, d, off in laid_out]
      self._block = shared_memory.SharedMemory(create=True, size=size_in + size_out)
      self._conn.send(('attach', self._block.name, laid_in, laid_out))
      msg = self._recv()
      if msg[0] != 'ready':
        raise RuntimeError(f"onnxruntime worker: {msg[1] if len(msg) > 1 else msg}")
      self.providers = msg[1]   # per session, as onnxruntime reports them
    except BaseException:
      self.close()
      raise
    self._host = worker.views(self._block, laid_in)
    self._out = worker.views(self._block, laid_out)
    self.inputs = {n: IO(n, s, np.dtype(d)) for n, s, d, _ in laid_in}
    self.outputs = {n: IO(n, s, np.dtype(d)) for n, s, d, _ in laid_out}

  def _recv(self):
    try:
      return self._conn.recv()
    except (EOFError, OSError) as e:
      # A child that could not even start: the usual cause is a parent whose
      # __main__ the spawn cannot re-import (a script fed on stdin). The
      # server and the tests are importable; say so rather than EOFError.
      raise WorkerDied('the onnxruntime worker went away before answering; its stderr says why') from e

  @property
  def input_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: io.shape for n, io in self.inputs.items()}

  @property
  def output_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: io.shape for n, io in self.outputs.items()}

  def host_input(self, name: str) -> np.ndarray:
    return self._host[name]

  def run(self) -> dict[str, np.ndarray]:
    if self._proc is None:
      raise WorkerDied('engine is closed')
    self._conn.send(('run',))
    if not self._conn.poll(RUN_TIMEOUT):
      raise WorkerDied(f'no answer from the onnxruntime worker in {RUN_TIMEOUT:.0f} s')
    msg = self._recv()
    if msg[0] != 'ok':
      raise RuntimeError(f"onnxruntime worker: {msg[1]}")
    self.last_gpu_us = int(msg[1])
    return self._out

  def warm(self) -> str:
    # CoreML allocates its working set on the first run and the second is the
    # steady state; the CPU and CUDA providers are cheap either way.
    self.run()
    self.run()
    return f'onnxruntime on {self.device} in a worker process, providers {self.providers}'

  def close(self) -> None:
    proc, self._proc = self._proc, None
    if proc is not None:
      try:
        self._conn.send(('close',))
      except (OSError, ValueError):
        pass
      proc.join(5.0)
      if proc.is_alive():
        proc.terminate()
        proc.join(5.0)
      self._conn.close()
    # Views into the block go with it: nobody reads a closed engine's outputs.
    self._host, self._out = {}, {}
    if self._block is not None:
      self._block.close()
      try:
        self._block.unlink()
      except FileNotFoundError:
        pass
      self._block = None
