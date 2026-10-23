"""Opt-in exact image conversion, compiled and self-tested at load only."""
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
def native_u8_to_f16():
  if sys.platform != 'linux' or (compiler := shutil.which('cc')) is None:
    return None
  try:
    with tempfile.TemporaryDirectory(prefix='jetlink-input-cast-') as directory:
      library = Path(directory) / 'input_cast.so'
      subprocess.run([compiler, '-O3', '-fPIC', '-shared', '-fno-fast-math',
                      str(Path(__file__).with_suffix('.c')), '-o', str(library)],
                     check=True, capture_output=True, timeout=10)
      dll = ctypes.CDLL(str(library))
    function = dll.jetlink_u8_f16
    function.argtypes = (ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t)
    function.restype = None
    table = np.arange(256, dtype=np.uint8).astype(np.float16).view(np.uint16)
    table.flags.writeable = False

    def convert(source, dest, _library=dll):
      if (source.dtype != np.uint8 or dest.dtype != np.float16 or source.size != dest.size
          or not source.flags.c_contiguous or not dest.flags.c_contiguous or not dest.flags.aligned
          or not dest.flags.writeable or np.shares_memory(source, dest)):
        raise ValueError('Invalid native uint8/float16 input buffers')
      function(source.ctypes.data, dest.ctypes.data, table.ctypes.data, source.size)

    # Every byte value and short/tail sizes before accepting model frames.
    for length in (0, 1, 7, 8, 15, 16, 17, 31, 32, 33, 255, 256, 257):
      source = np.arange(length, dtype=np.uint8)
      dest = np.empty(length, np.float16)
      convert(source, dest)
      if dest.tobytes() != source.astype(np.float16).tobytes():
        raise ValueError('Native input conversion self-test failed')
    return convert
  except (OSError, subprocess.SubprocessError, ValueError, AttributeError) as exc:
    logging.getLogger(__name__).warning('Native input conversion unavailable: %s', type(exc).__name__)
    return None
