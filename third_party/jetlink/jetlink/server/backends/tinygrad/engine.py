"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Running a captured tinygrad JIT a frame at a time.

The staging arrays are numpy arrays that tinygrad's NPY device wraps without
copying, so the session's queues gather straight into them and the copy to
the GPU is the first thing the replay does. The output comes back through
`.numpy()`, 74 KB a frame, into a persistent float32 array.

Everything that touches the JIT runs on the tinygrad thread (owner.py); the
staging arrays are plain host memory and any thread may fill them first.
"""
from __future__ import annotations

import gc
import time

import numpy as np

from jetlink.server.backends.base import IO, ArtifactInvalid
from jetlink.server.backends.tinygrad.build import (
  FORMAT,
  Staged,
  load_oob,
  make_staging,
)
from jetlink.server.backends.tinygrad.owner import on_owner


class TinygradEngine:
  def __init__(self, artifact: str, device: str):
    self.device = device
    self.last_gpu_us = 0
    on_owner(self._load, artifact)

  # -- on the tinygrad thread ------------------------------------------------

  def _load(self, artifact: str) -> None:
    try:
      with open(artifact, 'rb') as f:
        payload = load_oob(f)
      if not isinstance(payload, dict) or payload.get('format') != FORMAT:
        raise ArtifactInvalid(f"{artifact}: not a format {FORMAT} jetlink tinygrad artifact")
      if payload['device'] != self.device:
        raise ArtifactInvalid(f"{artifact}: captured on {payload['device']}, this is {self.device}")
      jit = payload['jit']
      plan = [Staged(n, tuple(shape), hd, md) for n, shape, hd, md in payload['inputs']]
      outputs = payload['outputs']
    except ArtifactInvalid:
      raise
    except Exception as e:
      # An unpickle that fails is a pickle from another tinygrad, not a
      # machine problem: the host deletes it and builds a fresh one.
      raise ArtifactInvalid(f"{artifact}: {type(e).__name__}: {e}") from e

    self.jit = jit
    self.identity = payload.get('tinygrad', '')
    self._arrays, self._tensors = make_staging(plan)
    self.inputs = {s.name: IO(s.name, s.shape, np.dtype(s.host_dtype)) for s in plan}
    self.outputs = {n: IO(n, tuple(shape), np.dtype(dt)) for n, shape, dt in outputs}
    self._out = {n: np.zeros(io.shape, io.dtype) for n, io in self.outputs.items()}

  def _run(self) -> dict[str, np.ndarray]:
    t0 = time.perf_counter()
    out = self.jit(**self._tensors)
    dest = self._out['outputs']
    dest[...] = out.numpy().reshape(dest.shape)
    self.last_gpu_us = int((time.perf_counter() - t0) * 1e6)
    return self._out

  def _close(self) -> None:
    jit, self.jit = self.jit, None
    self._tensors = {}
    captured = getattr(jit, 'captured', None)
    if captured is not None and hasattr(captured, 'free_intermediates'):
      # Drops the graph runners and the arena; the weights go with the last
      # reference. Done here so the device memory is back before the next
      # engine loads, not when the collector gets round to it.
      captured.free_intermediates()
    del jit, captured
    gc.collect()

  # -- the Engine protocol ---------------------------------------------------

  @property
  def input_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: io.shape for n, io in self.inputs.items()}

  @property
  def output_shapes(self) -> dict[str, tuple[int, ...]]:
    return {n: io.shape for n, io in self.outputs.items()}

  def host_input(self, name: str) -> np.ndarray:
    return self._arrays[name]

  def run(self) -> dict[str, np.ndarray]:
    return on_owner(self._run)

  def warm(self) -> str:
    # The pickle is a captured JIT, so every call is a replay; two runs pay
    # for the first allocation of the intermediates and the output copy.
    self.run()
    self.run()
    return f'tinygrad jit replaying on {self.device}'

  def close(self) -> None:
    if getattr(self, 'jit', None) is not None:
      on_owner(self._close)
