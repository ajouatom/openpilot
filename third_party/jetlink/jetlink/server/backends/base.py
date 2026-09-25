"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The seam between the server and whatever runs the model.

A Backend turns an ONNX file into an artifact it can load quickly (a TensorRT
plan, a pickled tinygrad JIT, a compiled CoreML model) and loads one into an
Engine. An Engine runs one frame: the session writes the model's inputs into
the staging arrays `host_input` hands out, calls `run`, and reads the output
views back. Everything else in the server - the protocol, the queues, the cache,
the request loop - only ever sees these two shapes.

The protocols are what tests/test_session.py's FakeEngine already implemented
before any second backend existed, which is the evidence the seam sits where
the code already bent.
"""
from __future__ import annotations

import re
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

ProgressFn = Callable[[str, float, str], None]


class ArtifactInvalid(Exception):
  """The artifact on disk is not one this backend can load.

  Raised by `Backend.load` for something that is wrong with the file itself: a
  JIT pickled by another tinygrad, a compiled-model cache another runtime wrote.
  The host answers by deleting it and rebuilding from the ONNX if it has one,
  so a runtime upgrade heals itself instead of failing every connect. Anything
  else a load can hit (out of memory, a device that went away) is a plain
  exception and the artifact stays.
  """


@dataclass(frozen=True)
class IO:
  """One model input or output as the engine stages it on the host."""
  name: str
  shape: tuple[int, ...]
  dtype: np.dtype


class Engine(Protocol):
  """A loaded model, ready to run a frame at a time."""
  inputs: dict[str, IO]      # values need .shape and .dtype; TensorRT's Binding qualifies
  outputs: dict[str, IO]
  last_gpu_us: int           # how long the last run() took on the accelerator

  def host_input(self, name: str) -> np.ndarray:
    """Writable staging array for an input, in the engine's host dtype. Write
    into it, then call run()."""

  def run(self) -> dict[str, np.ndarray]:
    """Run one frame. Views over engine-owned memory, valid until the next run()."""

  def warm(self) -> str:
    """Run whatever is in the staging arrays until the steady state is reached
    (CUDA graph capture, JIT replay). Returns a line worth logging."""

  def close(self) -> None:
    """Release device memory. Nothing else is called afterwards."""


class Backend(Protocol):
  name: str          # 'trt' | 'tinygrad' | 'ort', as the hello reports it
  suffix: str        # the artifact's extension, '.plan' | '.pkl' | '.ortcache'

  def tag(self) -> str:
    """What an artifact is valid for: runtime version and device, sanitized
    for a filename. The cache key is '<sha16>.<tag>' and a tag that changes
    means a rebuild, so it has to change with anything that changes the
    artifact and nothing else."""

  def describe(self) -> dict:
    """{'backend', 'runtime_version', 'device'} for the hello response."""

  def build(self, onnx_path: Path, out_path: Path, report: ProgressFn | None = None,
            meta_extra: dict | None = None) -> Path:
    """ONNX in, artifact at out_path out, with a json sidecar next to it. Must
    write the artifact atomically: a build the OOM killer ends must not leave
    something the next load mistakes for an engine."""

  def load(self, artifact: Path, report: ProgressFn | None = None) -> Engine:
    """Load an artifact this backend built. Raises ArtifactInvalid for a file
    that is wrong, anything else for a machine that is.

    `report` is the same callback `build` takes, for a load slow enough to
    need one: CoreML compiles for nine minutes whether or not there is a
    cache, and a progress bar that never moves looks like a hang."""


def sanitize(s: str) -> str:
  """A version or device name as a filename component."""
  return re.sub(r'[^A-Za-z0-9._-]', '_', s)


def write_sidecar(artifact: Path, meta: dict) -> Path:
  """The json next to an artifact. Named by replacing the suffix, so
  '<key>.plan' pairs with '<key>.json' whatever the key's own dots."""
  path = artifact.with_suffix('.json')
  import json
  path.write_text(json.dumps(meta, indent=2))
  return path


def input_shapes(engine: Engine) -> dict[str, tuple[int, ...]]:
  return {n: tuple(io.shape) for n, io in engine.inputs.items()}


def output_shapes(engine: Engine) -> dict[str, tuple[int, ...]]:
  return {n: tuple(io.shape) for n, io in engine.outputs.items()}


def load_inputs(engine: Engine, values: dict[str, np.ndarray]) -> None:
  """Copy named arrays into the staging buffers, casting to the engine's dtype.
  The bench tools' path; the server's hot path writes through the queues."""
  for name, value in values.items():
    io = engine.inputs.get(name)
    if io is None:
      raise KeyError(f"engine has no input {name!r}; has {sorted(engine.inputs)}")
    if int(np.prod(value.shape)) != int(np.prod(io.shape)):
      raise ValueError(f"{name}: {value.shape} has {value.size} elements, "
                       f"engine wants {tuple(io.shape)} ({int(np.prod(io.shape))})")
    np.copyto(engine.host_input(name), value.reshape(io.shape), casting='unsafe')


def infer(engine: Engine, values: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
  load_inputs(engine, values)
  return engine.run()
