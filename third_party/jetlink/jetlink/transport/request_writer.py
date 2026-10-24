"""Bounded Linux AIO writes with an opt-in completion-deferred sender."""
import ctypes
from functools import lru_cache
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

@lru_cache(maxsize=1)
def library():
  if sys.platform != 'linux' or not shutil.which('cc'):
    return None
  try:
    with tempfile.TemporaryDirectory(prefix='jetlink-writer-') as folder:
      path = Path(folder) / 'writer.so'
      command = ['cc', '-O2', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                 str(Path(__file__).with_suffix('.c')), '-o', str(path)]
      if shutil.which('chrt'):
        command = ['chrt', '-o', '0', *command]
      subprocess.run(command, check=True, capture_output=True, timeout=15)
      lib = ctypes.CDLL(str(path), use_errno=True)
    lib.jetlink_writer_create.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int)
    lib.jetlink_writer_create.restype = ctypes.c_void_p
    lib.jetlink_writer_send.argtypes = (ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                      ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint, ctypes.c_double)
    lib.jetlink_writer_send.restype = ctypes.c_int
    lib.jetlink_writer_poll.argtypes = (ctypes.c_void_p,)
    lib.jetlink_writer_poll.restype = ctypes.c_int
    lib.jetlink_writer_close.argtypes = (ctypes.c_void_p,)
    lib.jetlink_writer_close.restype = ctypes.c_int
    return lib
  except (OSError, subprocess.SubprocessError):
    return None

class Writer:
  def __init__(self, fd, lib, *, regular=False, deferred=False, quantum=32768):
    if quantum not in (8192, 32768) or (not deferred and quantum != 32768):
      raise ValueError('unsupported USB write quantum')
    self.lib = lib
    self.deferred = deferred
    self.quantum = quantum
    self.context = lib.jetlink_writer_create(fd, regular, deferred, quantum)
    if not self.context:
      raise OSError(ctypes.get_errno(), 'bounded USB writer unavailable')

  def send(self, packets, budget):
    if self.context is None:
      raise RuntimeError('bounded USB writer closed')
    if not packets or len(packets) > 34 or any(not 0 < len(p) <= 32768 for p in packets):
      raise ValueError('bounded USB packet sizes')
    data = b''.join(packets)
    sizes = [min(self.quantum, len(p)-at) for p in packets for at in range(0, len(p), self.quantum)]
    lengths = (ctypes.c_uint32 * len(sizes))(*sizes)
    result = self.lib.jetlink_writer_send(self.context, data, len(data), lengths, len(sizes), budget)
    if result < 0:
      raise OSError(-result, 'bounded USB write failed')

  def poll(self):
    if self.context is None:
      raise RuntimeError('bounded USB writer closed')
    result = self.lib.jetlink_writer_poll(self.context)
    if result < 0:
      raise OSError(-result, 'bounded USB completion failed')

  def close(self):
    if self.context is not None:
      result = self.lib.jetlink_writer_close(self.context)
      if result < 0:
        raise OSError(-result, 'bounded USB writer cleanup failed; buffers retained')
      self.context = None
