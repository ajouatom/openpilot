"""Opt-in, fixed-depth FunctionFS read pipeline; no model/protocol changes."""
import ctypes
from functools import lru_cache
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


@lru_cache(maxsize=1)
def load_library():
  if sys.platform != 'linux':
    return None
  compiler, chrt = shutil.which('cc'), shutil.which('chrt')
  if compiler is None or chrt is None:
    return None
  with tempfile.TemporaryDirectory(prefix='jetlink-aio-') as directory:
    library = Path(directory) / 'aio.so'
    subprocess.run([chrt, '-o', '0', compiler, '-O2', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                    str(Path(__file__).with_suffix('.c')), '-o', str(library)],
                   check=True, capture_output=True, timeout=15)
    dll = ctypes.CDLL(str(library), use_errno=True)
    dll.jetlink_aio_create.argtypes = (ctypes.c_int, ctypes.c_uint64)
    dll.jetlink_aio_create.restype = ctypes.c_void_p
    dll.jetlink_aio_read.argtypes = (ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t)
    dll.jetlink_aio_read.restype = ctypes.c_int
    dll.jetlink_aio_close.argtypes = (ctypes.c_void_p,)
    dll.jetlink_aio_close.restype = ctypes.c_int
    return dll


class Reader:
  def __init__(self, fd, library, *, offset_step=0):
    self.library = library
    self.context = library.jetlink_aio_create(fd, offset_step)
    if not self.context:
      error = ctypes.get_errno()
      raise OSError(error, 'FunctionFS AIO context unavailable')

  def read_into(self, buffer):
    if self.context is None:
      raise RuntimeError('AIO reader already closed')
    view = (ctypes.c_char * len(buffer)).from_buffer(buffer)
    result = self.library.jetlink_aio_read(self.context, view, len(buffer))
    if result < 0:
      raise OSError(-result, 'FunctionFS AIO read failed')
    return result

  def close(self):
    if self.context is not None:
      result = self.library.jetlink_aio_close(self.context)
      if result < 0:
        raise OSError(-result, 'FunctionFS AIO cleanup failed; buffers retained')
      self.context = None
