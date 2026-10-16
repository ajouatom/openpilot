"""Install a process-local cleanup adapter before the HUD imports comma raylib."""
from __future__ import annotations

import ctypes
import hashlib
import importlib.metadata
import os
from pathlib import Path
import sys

# The adapter closes descriptors that these exact ClosePlatform implementations
# omit. Do not apply it to an unknown/upstream-fixed binary, or to the main UI.
KNOWN_BINARIES = {
  '6.0.0.1.post98': 'b1cc6eb0241081f4fe826f14d80f8a6e6f2b1953440ce6cf5f34c4af5e3660fb',
  '6.0.0.1.post103': 'ebb0b6c4112a39d700e06fd40947023485d3c14c0d0ac36abef8c6a9551d7502',
}
_library = None


def ensure_clean_autorun_process() -> None:
  """Discard manager-preimported graphics in this HUD child, keeping its PID."""
  if not Path('/TICI').is_file() or os.environ.get('RAYLIB_BACKEND', 'comma') != 'comma':
    return
  if 'raylib' not in sys.modules and 'pyray' not in sys.modules:
    return
  if os.environ.get('CARROT_HUD_CLEAN_EXEC') == '1':
    raise RuntimeError('HUD graphics imported before cleanup in a fresh process')
  env = dict(os.environ, CARROT_HUD_CLEAN_EXEC='1')
  print('[HUD graphics] discarding inherited graphics state in HUD child', flush=True)
  os.execve(sys.executable, [sys.executable, '-m', 'openpilot.selfdrive.carrot.cluster_autorun'], env)


def install_graphics_cleanup() -> bool:
  global _library
  if not Path('/TICI').is_file() or os.environ.get('RAYLIB_BACKEND', 'comma') != 'comma':
    return False
  if _library is not None:
    return True
  if 'raylib' in sys.modules or 'pyray' in sys.modules:
    raise RuntimeError('HUD graphics cleanup must load before raylib; restart the HUD process')
  dist = importlib.metadata.distribution('comma-deps-raylib')
  binary = Path(dist.locate_file('raylib/_raylib_cffi_comma.abi3.so'))
  expected = KNOWN_BINARIES.get(dist.version)
  if expected is None or hashlib.sha256(binary.read_bytes()).hexdigest() != expected:
    raise RuntimeError('HUD graphics cleanup: unverified raylib binary; refusing leaking retries')
  path = Path(__file__).with_name('libcarrot_gbm_cleanup.so')
  library = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
  library.carrot_gbm_cleanup_enable.argtypes = [ctypes.c_int]
  library.carrot_gbm_cleanup_enable.restype = ctypes.c_int
  if not library.carrot_gbm_cleanup_enable(int('DRM_FD' not in os.environ)):
    raise RuntimeError('HUD graphics cleanup initialization failed')
  _library = library  # Keep it resident for all GBM calls and subsequent sessions.
  print(f'[HUD graphics] native cleanup enabled for raylib {dist.version}', flush=True)
  return True
