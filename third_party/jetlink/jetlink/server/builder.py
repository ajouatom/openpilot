"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Compatibility: the engine cache is jetlink.server.cache and the TensorRT build
is jetlink.server.backends.trt.build. Kept for one release so a script in a
container image or a playbook that imports the old name keeps working.
"""
from __future__ import annotations

import importlib

_HOMES = ('jetlink.server.cache', 'jetlink.server.platform', 'jetlink.server.backends.trt.build')


def __getattr__(name: str):
  if name == 'DEFAULT_CACHE':
    from jetlink.server.platform import default_cache_dir
    return default_cache_dir()
  for home in _HOMES:
    try:
      module = importlib.import_module(home)
    except ImportError:
      continue
    if hasattr(module, name):
      return getattr(module, name)
  raise AttributeError(f"module 'jetlink.server.builder' has no attribute {name!r}")
