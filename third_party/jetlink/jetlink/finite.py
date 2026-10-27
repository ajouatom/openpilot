"""Optional fused CPU validation, built once before accepting model frames."""
import ctypes
from functools import lru_cache
import logging
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np


@lru_cache(maxsize=1)
def native_checker():
  # No compilation, allocation or process creation in the inference loop.
  # TemporaryDirectory owns a private directory; never load a shared /tmp file.
  if sys.platform != 'linux':
    return None
  compiler = shutil.which('cc')
  if compiler is None:
    return None
  try:
    with tempfile.TemporaryDirectory(prefix='jetlink-finite-') as directory:
      library = Path(directory) / 'finite.so'
      subprocess.run([compiler, '-O3', '-fPIC', '-shared', '-fno-fast-math',
                      str(Path(__file__).with_suffix('.c')), '-o', str(library)],
                     check=True, capture_output=True, timeout=10)
      dll = ctypes.CDLL(str(library))
      functions = {np.dtype(np.float16): dll.jetlink_finite16, np.dtype(np.float32): dll.jetlink_finite32}
      for fn in functions.values():
        fn.argtypes = (ctypes.c_void_p, ctypes.c_size_t)
        fn.restype = ctypes.c_int
      # Keep the loaded library alive after its private build files are removed.
      def check(value, _library=dll):
        fn = functions.get(value.dtype)
        if fn is None or not value.flags.c_contiguous or not value.flags.aligned or not value.dtype.isnative:
          return bool(np.isfinite(value).all())
        return bool(fn(value.ctypes.data, value.size))
      for dtype in functions:
        for values in ([0., -0., 1., -1.], [np.nan], [np.inf], [-np.inf]):
          probe = np.asarray(values, dtype=dtype)
          if check(probe) != bool(np.isfinite(probe).all()):
            raise ValueError('native finiteness self-test failed')
      return check
  except (OSError, subprocess.SubprocessError, ValueError) as exc:
    logging.getLogger(__name__).warning('Native validation unavailable; retaining NumPy checks: %s', type(exc).__name__)
    return None
