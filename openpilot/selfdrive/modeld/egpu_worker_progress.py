"""Small shared progress record: no per-frame files, GPU probes or network I/O."""
import json
import mmap
from pathlib import Path
import struct
import threading
import time

STAGES = ('starting', 'verify_model', 'load_model', 'prepare_runtime', 'bind_buffers', 'idle',
          'input_upload', 'warp', 'model_call', 'output_read', 'publish', 'local_prepare')
RECORD = struct.Struct('<QQQddQ')  # generation, stage, frame, monotonic, CPU, thread id


class WorkerProgress:
  def __init__(self, path: Path):
    self.frame = self.generation = 0
    self.memory = None
    try:
      with path.open('w+b') as f:
        f.truncate(RECORD.size)
        self.memory = mmap.mmap(f.fileno(), RECORD.size)
      self.mark('starting')
    except OSError:
      pass  # optional diagnostics must not prevent inference

  def mark(self, stage: str):
    if self.memory is None:
      return
    self.generation += 2
    struct.pack_into('<Q', self.memory, 0, self.generation - 1)
    RECORD.pack_into(self.memory, 0, self.generation - 1, STAGES.index(stage), self.frame,
                     time.monotonic(), time.thread_time(), threading.get_native_id())
    struct.pack_into('<Q', self.memory, 0, self.generation)

  def close(self):
    if self.memory is not None:
      self.memory.close()
      self.memory = None


def worker_snapshot(path: Path, pid: int, proc_root: Path = Path('/proc')) -> dict:
  """Best-effort bounded reads before termination, without waiting for a stuck worker."""
  result = {'pid': pid}
  try:
    with path.open('rb') as f:
      first = f.read(RECORD.size)
      f.seek(0)
      second = f.read(RECORD.size)
    if first == second and len(first) == RECORD.size:
      generation, stage, frame, mono, cpu, tid = RECORD.unpack(first)
      age = time.monotonic() - mono
      if generation and generation % 2 == 0 and stage < len(STAGES) and 0 <= age < 86400:
        result.update(stage=STAGES[stage], frame=frame, stage_age_ms=round(age * 1000, 3),
                      thread_cpu_seconds=cpu, thread_id=tid)
  except (OSError, ValueError, struct.error):
    pass
  task = proc_root / str(pid) / 'task' / str(result.get('thread_id', pid))
  for name in ('stat', 'schedstat', 'wchan'):
    try:
      with (task / name).open() as f:
        result[name] = f.read(2048).strip()
    except OSError:
      pass
  return result


def runtime_failure_code(error: str) -> str:
  text = error.lower()
  if 'pcie link not up' in text:
    return 'pcie'
  if 'precompiled egpu worker timed out' in text:
    return 'timeout'
  if any(marker in text for marker in ('libusb', 'bulk out', 'bulk in', 'input/output error')):
    return 'usb'
  return 'runtime'


def boot_identity() -> str:
  try:
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  except OSError:
    return ''


def current_failure(cache: Path, sha: str) -> dict:
  """Only use current-boot evidence for a currently failed runtime badge."""
  if len(sha) != 64 or any(c not in '0123456789abcdef' for c in sha):
    return {}
  try:
    root = cache / 'precompiled' / sha
    with (root / 'last_failure.json').open() as f:
      value = json.loads(f.read(32768))
    with (root / 'installed.json').open() as f:
      installed = json.loads(f.read(32768))
    boot_id = boot_identity()
    # Legacy catalogs use an ONNX source hash for the directory, unlike generic PKLs.
    if not boot_id or value.get('boot_id') != boot_id or value.get('pickle_sha256') != installed['pickle']['sha256']:
      return {}
    worker = value.get('worker')
    return {'error_code': runtime_failure_code(str(value.get('error', ''))),
            'worker': worker if isinstance(worker, dict) else {}, 'phase': value.get('phase')}
  except (OSError, ValueError, TypeError, AttributeError, KeyError):
    return {}
