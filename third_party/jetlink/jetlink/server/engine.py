"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Compatibility: TensorRT execution is jetlink.server.backends.trt.engine.
"""
from __future__ import annotations


def __getattr__(name: str):
  from jetlink.server.backends.trt import engine
  return getattr(engine, name)
