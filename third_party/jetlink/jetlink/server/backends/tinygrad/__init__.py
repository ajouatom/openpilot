"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

tinygrad: the model on whatever GPU tinygrad drives, Metal first.

The exporter's own runtime, so the ONNX needs no surgery: tinygrad takes the
uint8 image inputs as declared and runs its `org.tinygrad` layout ops
natively. Measured on an M1 Pro (docs/backends.md#mac-measured) the 766 MB model
replays in 65 ms, over the 50 ms frame budget, and the time is per-kernel
launch overhead rather than arithmetic; a newer Mac or a newer scheduler is
what moves it. onnxruntime's CoreML provider (backends/ort) is the faster Mac
path today and the default; this is --backend tinygrad.
"""
from __future__ import annotations

import os
from pathlib import Path

from jetlink.server.backends.base import ProgressFn, sanitize
from jetlink.server.platform import gpu_name


def select_device(device: str) -> str:
  """Make `device` tinygrad's default and return what it settled on.

  DEV has to be set before tinygrad reads it, which is at import, so this runs
  before the first tinygrad import in the process; DEV.value is also set for a
  process that imported tinygrad already (onnx_meta does, to parse a spec).
  """
  wanted = (device or 'auto').upper()
  if wanted != 'AUTO':
    os.environ['DEV'] = wanted
  from tinygrad import Device
  if wanted != 'AUTO':
    try:
      from tinygrad.helpers import DEV
      DEV.value = wanted
    except (ImportError, AttributeError):
      pass
    if Device.DEFAULT != wanted:
      raise RuntimeError(f"tinygrad would not select {wanted} (it chose {Device.DEFAULT})")
  return Device.DEFAULT


class TinygradBackend:
  name = 'tinygrad'
  suffix = '.pkl'

  def __init__(self, device: str = 'auto'):
    self.device = select_device(device)
    if self.device in ('CPU', 'PYTHON', 'NPY'):
      import logging
      logging.getLogger('jetlink.tinygrad').warning(
        "tinygrad on %s will not make the frame budget; fine for a bench, not a car", self.device)

  @property
  def runtime_version(self) -> str:
    from jetlink.server.backends.tinygrad.build import tinygrad_identity
    return tinygrad_identity()

  def device_tag(self) -> str:
    return sanitize(f"{self.device}-{gpu_name()}")

  def tag(self) -> str:
    # BEAM changes the generated kernels, so a searched build is its own artifact.
    beam = os.environ.get('BEAM', '')
    suffix = f".beam{beam}" if beam not in ('', '0') else ''
    return f"tg{sanitize(self.runtime_version)}.{self.device_tag()}{suffix}"

  def describe(self) -> dict:
    return {'backend': self.name, 'runtime_version': self.runtime_version, 'device': self.device_tag()}

  def build(self, onnx_path: Path, out_path: Path, report: ProgressFn | None = None,
            meta_extra: dict | None = None) -> Path:
    # On the tinygrad thread, not the caller's: the server builds on a job
    # thread that exits afterwards, and a thread that has compiled Metal
    # kernels crashes on its way out (owner.py).
    from jetlink.server.backends.tinygrad.build import build_jit
    from jetlink.server.backends.tinygrad.owner import on_owner
    return on_owner(build_jit, onnx_path, out_path, self.device, report=report, meta_extra=meta_extra)

  def load(self, artifact: Path, report=None):
    from jetlink.server.backends.tinygrad.engine import TinygradEngine
    return TinygradEngine(str(artifact), self.device)
