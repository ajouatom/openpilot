"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Compatibility: the CUDA runtime wrapper is jetlink.server.backends.trt.cudart.
"""
from __future__ import annotations


def __getattr__(name: str):
  from jetlink.server.backends.trt import cudart
  return getattr(cudart, name)
