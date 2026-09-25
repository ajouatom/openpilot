"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Which runtime runs the model.

    trt       TensorRT on an NVIDIA GPU: the Jetson, and a desktop or laptop
    ort       onnxruntime: CoreML on Apple silicon, CUDA or CPU elsewhere
    tinygrad  tinygrad on whatever it drives: Metal, CUDA, AMD

Runtimes are imported only when chosen, never here: the tests import the
server on machines with none of them, and a Jetson must not pay for a Mac's
imports.

`auto` prefers TensorRT, then on a Mac CoreML through onnxruntime, then
tinygrad, then onnxruntime on whatever it has. The Mac order is measured
(docs/platforms.md): CoreML runs the frame in 39 ms to tinygrad's 66 on an
M1 Pro, and 66 is over the budget. CoreML's price is nine minutes to create
its session every time the process starts, paid in a worker while the
server keeps answering, which is the same wait a Jetson rebuilding a plan
costs and as rare. --backend tinygrad is the switch.
"""
from __future__ import annotations

import importlib.util
import logging
import sys

from jetlink.server.backends.base import Backend

log = logging.getLogger('jetlink.backends')

NAMES = ('trt', 'tinygrad', 'ort')


def _importable(module: str) -> bool:
  try:
    return importlib.util.find_spec(module) is not None
  except (ImportError, ValueError):
    return False


def _make(name: str, device: str) -> Backend:
  if name == 'trt':
    from jetlink.server.backends.trt import TrtBackend
    return TrtBackend(device)
  if name == 'ort':
    from jetlink.server.backends.ort import OrtBackend
    return OrtBackend(device)
  if name == 'tinygrad':
    from jetlink.server.backends.tinygrad import TinygradBackend
    return TinygradBackend(device)
  raise ValueError(f"unknown backend {name!r}; one of {NAMES} or auto")


def available() -> list[str]:
  """Backends whose runtime is installed, in auto's order of preference."""
  found = []
  if _importable('tensorrt') and (_importable('cuda.bindings') or _importable('cuda')):
    found.append('trt')
  if _importable('onnxruntime') and sys.platform == 'darwin':
    found.append('ort')
  if _importable('tinygrad'):
    found.append('tinygrad')
  if _importable('onnxruntime') and 'ort' not in found:
    found.append('ort')
  return found


def _candidates(device: str) -> list[tuple[str, str]]:
  """(backend, device) pairs auto tries, in order. On a Mac the first try is
  CoreML by name, so a Mac whose onnxruntime has no CoreML provider falls
  through to tinygrad rather than serving off the CPU; a --device the user
  gave goes to every candidate, and the ones it means nothing to skip."""
  out = []
  for name in available():
    if name == 'ort' and sys.platform == 'darwin' and device == 'auto':
      out.append(('ort', 'coreml'))
      out.append(('tinygrad', device))
      continue
    out.append((name, device))
  seen, unique = set(), []
  for pair in out:
    if pair not in seen and pair[0] in available():
      seen.add(pair)
      unique.append(pair)
  return unique


def select(name: str = 'auto', device: str = 'auto') -> Backend:
  """The backend to serve with. A named backend that will not come up raises;
  `auto` moves on to the next and says why."""
  if name != 'auto':
    return _make(name, device)
  candidates = _candidates(device)
  if not candidates:
    raise RuntimeError("no inference runtime is installed: pip install one of "
                       "'jetlink[trt]', 'jetlink[ort]' or 'jetlink[tinygrad]'")
  reasons = []
  for candidate, dev in candidates:
    try:
      backend = _make(candidate, dev)
    except Exception as e:
      reasons.append(f"{candidate} ({dev}): {type(e).__name__}: {e}")
      log.warning("backend %s (%s) not used: %s", candidate, dev, e)
      continue
    log.info("backend %s %s on %s", backend.name, backend.describe()['runtime_version'],
             backend.describe()['device'])
    return backend
  raise RuntimeError("no backend came up:\n  " + "\n  ".join(reasons))
