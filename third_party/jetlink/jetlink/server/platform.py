"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The few places the server has to know what it is running on.

Everything here fails open to a sensible answer: a wrong guess costs a cache
directory in an odd place or a flat memory cap, never a refusal to serve.
"""
from __future__ import annotations

import os
import platform
import subprocess
import sys
from pathlib import Path

# Where the Jetson image keeps its cache; docker/Dockerfile sets JETLINK_CACHE
# to the same path, so the env var wins and this is the fallback for a
# container started without it.
JETSON_CACHE = Path('/mnt/data/jetlink')

# Any one of these says Tegra. Three, because the container mounts /sys but not
# /proc/device-tree, and a bare L4T host has the release file but may lack the
# gpu node's exact address on another SoC.
_TEGRA_SIGNS = (
  '/sys/firmware/devicetree/base/compatible',
  '/proc/device-tree/compatible',
)
_TEGRA_FILES = (
  '/etc/nv_tegra_release',
  '/sys/devices/platform/bus@0/17000000.gpu',
)


def is_jetson() -> bool:
  for p in _TEGRA_SIGNS:
    try:
      if b'tegra' in Path(p).read_bytes().lower():
        return True
    except OSError:
      pass
  return any(Path(p).exists() for p in _TEGRA_FILES)


def available_bytes() -> int:
  """Memory a build workspace can live in, or 0 for "no idea, use the cap".

  MemAvailable on Linux, so free plus what the kernel would reclaim. Swap does
  not count: on Tegra the GPU's allocations are pinned system RAM and cannot
  page out, and the bench Jetson has 25 GB of swap to be fooled by. Only
  TensorRT sizes anything from this, so macOS answers 0 and Windows asks the
  kernel the one way it offers.
  """
  if sys.platform.startswith('linux'):
    try:
      with open('/proc/meminfo') as f:
        for line in f:
          key, _, rest = line.partition(':')
          if key == 'MemAvailable':
            return int(rest.split()[0]) * 1024
    except OSError:
      pass
    return 0
  if sys.platform == 'win32':
    try:
      import ctypes

      class MemoryStatusEx(ctypes.Structure):
        _fields_ = [('dwLength', ctypes.c_ulong), ('dwMemoryLoad', ctypes.c_ulong),
                    ('ullTotalPhys', ctypes.c_ulonglong), ('ullAvailPhys', ctypes.c_ulonglong),
                    ('ullTotalPageFile', ctypes.c_ulonglong), ('ullAvailPageFile', ctypes.c_ulonglong),
                    ('ullTotalVirtual', ctypes.c_ulonglong), ('ullAvailVirtual', ctypes.c_ulonglong),
                    ('ullAvailExtendedVirtual', ctypes.c_ulonglong)]
      st = MemoryStatusEx()
      st.dwLength = ctypes.sizeof(st)
      if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(st)):
        return int(st.ullAvailPhys)
    except Exception:
      pass
  return 0


def default_cache_dir() -> Path:
  """JETLINK_CACHE, else the Jetson's mount, else the user's cache directory."""
  env = os.environ.get('JETLINK_CACHE')
  if env:
    return Path(env)
  if JETSON_CACHE.is_dir() or is_jetson():
    return JETSON_CACHE
  if sys.platform == 'darwin':
    return Path.home() / 'Library' / 'Caches' / 'jetlink'
  if sys.platform == 'win32':
    return Path(os.environ.get('LOCALAPPDATA', Path.home() / 'AppData' / 'Local')) / 'jetlink'
  return Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'jetlink'


def _run(cmd: list[str]) -> str:
  try:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
  except (OSError, subprocess.SubprocessError):
    return ''


def gpu_name() -> str:
  """A human name for the accelerator, for cache keys and the hello.

  Apple silicon: the SoC name, which is the GPU. NVIDIA on a desktop: what
  nvidia-smi says. Anything else: the machine architecture, which at least
  keeps two laptops' artifacts apart when they are copied around.
  """
  if sys.platform == 'darwin':
    name = _run(['sysctl', '-n', 'machdep.cpu.brand_string'])
    if name:
      return name
  name = _run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'])
  if name:
    return name.splitlines()[0].strip()
  return platform.machine() or 'unknown'


def can_suspend() -> bool:
  """Whether --sleep-after has a kernel interface to talk to at all."""
  return Path('/sys/power/state').exists()
