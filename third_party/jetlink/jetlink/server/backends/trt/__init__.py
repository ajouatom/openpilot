"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

TensorRT on an NVIDIA GPU: the Jetson's backend, and a desktop's.

The cache tag is exactly what the pre-backend server wrote,
'trt<version>.<device>', so a Jetson's existing plans load without a rebuild.
"""
from __future__ import annotations

from pathlib import Path

import tensorrt as trt

from jetlink.server.backends.base import ProgressFn
from jetlink.server.backends.trt import build as _build


class TrtBackend:
  name = 'trt'
  suffix = '.plan'

  def __init__(self, device: str = 'auto'):
    # The CUDA device index; 'auto' is 0, which is the only one a Jetson has.
    self.device = 0 if device in ('auto', '', None) else int(device)

  @property
  def runtime_version(self) -> str:
    return trt.__version__

  def tag(self) -> str:
    return _build.version_tag(self.device)

  def describe(self) -> dict:
    return {'backend': self.name, 'runtime_version': self.runtime_version,
            'device': _build.device_tag(self.device)}

  def build(self, onnx_path: Path, out_path: Path, report: ProgressFn | None = None,
            meta_extra: dict | None = None) -> Path:
    if self.device:
      from jetlink.server.backends.trt import cudart
      cudart.set_device(self.device)
    return _build.build_engine(onnx_path, out_path, report=report, meta_extra=meta_extra,
                               timing_cache=_build.timing_cache_path(Path(out_path).parent, self.device))

  def load(self, artifact: Path, report=None):
    # A plan that will not deserialize is not treated as ArtifactInvalid on
    # purpose: plans are keyed by TensorRT version and GPU, so the file cannot
    # be stale, and the other reasons (memory, a device fault) are not cured by
    # deleting 770 MB and building for three minutes.
    from jetlink.server.backends.trt.engine import TrtEngine
    return TrtEngine(str(artifact), device=self.device if self.device else None)
