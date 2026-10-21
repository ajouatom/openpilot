"""Lease FIFO2 to observed DWC3 workers for one Jetson USB session.

Run as root with a pipe on stdin owned by jetlinkd. No global tracing, pool
number assumptions, CPU pinning, per-frame subprocesses or transport changes.
"""
import json
import os
from pathlib import Path
import re
import select
import signal
import sys
import time

PRIORITY = 2
LEASE_SECONDS = 3.
MAX_WORKERS = 16
INSTANCE = 'carrot-jetlink-usb-priority'
WORKER = re.compile(r'kworker/u\d+:\d+(?:H)?$')
EVENT = re.compile(r'^\s*.+-(\d+)\s+\[\d+\].*\bworkqueue_execute_start:.*\bfunction[ =]+dwc3_bh_work(?:\+[^\s]+)?\s*$')


class RestorationError(RuntimeError):
  pass


def supported():
  try:
    return (os.uname().release == '4.9.103' and os.uname().machine == 'aarch64'
            and Path('/proc/device-tree/model').read_bytes().rstrip(b'\0\n') == b'comma mici')
  except (OSError, AttributeError):
    return False


def identity(pid):
  fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
  # PF_KTHREAD, kernel parent and an unbound-worker name: never elevate userspace.
  name = Path(f'/proc/{pid}/comm').read_text().strip()
  if not (int(fields[6]) & 0x00200000 and int(fields[1]) == 2 and WORKER.fullmatch(name)):
    raise ValueError('not an unbound kernel worker')
  return fields[19]


def policy(pid):
  return [os.sched_getscheduler(pid), os.sched_getparam(pid).sched_priority]


def set_policy(pid, value):
  os.sched_setscheduler(pid, value[0], os.sched_param(value[1]))


class Leases:
  def __init__(self, journal, boot, *, identify=identity, get=policy, put=set_policy):
    self.journal, self.boot = journal, boot
    self.identify, self.get, self.put = identify, get, put
    self.entries = {}

  def save(self):
    tmp = self.journal.with_suffix('.tmp')
    tmp.write_text(json.dumps({'boot': self.boot, 'entries': self.entries}))
    tmp.replace(self.journal)

  def recover(self):
    if self.journal.exists():
      data = json.loads(self.journal.read_text())
      if data['boot'] == self.boot:
        self.entries = data['entries']
        if len(self.entries) > MAX_WORKERS:
          raise ValueError('invalid priority journal')
        self.expire(float('inf'))
      else:
        self.save()

  def observe(self, pid, now):
    key = str(pid)
    try:
      token = self.identify(pid)
      previous = self.entries.get(key)
      if previous is not None and previous['identity'] == token:
        previous['last'] = now
        return  # Do not fight another scheduler owner.
      if previous is not None:
        del self.entries[key]
      original = self.get(pid)
      if original != [os.SCHED_OTHER, 0] or len(self.entries) >= MAX_WORKERS:
        return
      self.entries[key] = {'identity': token, 'original': original, 'last': now}
      self.save()  # Persist restoration before changing scheduling.
      if self.identify(pid) != token:
        return
      self.put(pid, [os.SCHED_FIFO, PRIORITY])
      print(f'Jetlink USB priority: worker {pid} FIFO{PRIORITY}', flush=True)
    except (FileNotFoundError, ProcessLookupError, ValueError):
      return

  def expire(self, now):
    errors = []
    changed = False
    for key, entry in list(self.entries.items()):
      if now - entry['last'] < LEASE_SECONDS:
        continue
      pid = int(key)
      try:
        if self.identify(pid) == entry['identity'] and self.get(pid) == [os.SCHED_FIFO, PRIORITY]:
          self.put(pid, entry['original'])
          if self.identify(pid) == entry['identity'] and self.get(pid) != entry['original']:
            raise RuntimeError(f'worker {pid} restoration did not stick')
      except (FileNotFoundError, ProcessLookupError, ValueError):
        pass
      except (OSError, RuntimeError) as exc:
        errors.append(exc)
        continue  # Restore the other workers even after one failure.
      del self.entries[key]
      changed = True
    if changed:
      try:
        self.save()
      except OSError as exc:
        errors.append(exc)
    if errors:
      raise RestorationError(f'USB priority restoration failed: {errors}')


class Trace:
  def __init__(self, root=Path('/sys/kernel/debug/tracing')):
    self.path = root / 'instances' / INSTANCE
    self.fd = None
    self.pending = b''

  def open(self):
    addresses = [int(line.split()[0], 16) for line in Path('/proc/kallsyms').read_text().splitlines()
                 if line.endswith(' dwc3_bh_work')]
    if len(addresses) != 1 or addresses[0] == 0:
      raise RuntimeError('DWC3 completion symbol unavailable')
    self.path.mkdir(exist_ok=True)
    (self.path / 'tracing_on').write_text('0')
    (self.path / 'events/enable').write_text('0')
    (self.path / 'current_tracer').write_text('nop')
    (self.path / 'buffer_size_kb').write_text('64')
    (self.path / 'trace').write_text('')
    event = self.path / 'events/workqueue/workqueue_execute_start'
    (event / 'filter').write_text(f'function == {addresses[0]}')
    (event / 'enable').write_text('1')
    self.fd = os.open(self.path / 'trace_pipe', os.O_RDONLY | os.O_NONBLOCK)
    (self.path / 'tracing_on').write_text('1')

  def read(self):
    try:
      data = os.read(self.fd, 262144)
    except BlockingIOError:
      return set()
    lines = (self.pending + data).split(b'\n')
    self.pending = lines.pop()
    if len(self.pending) > 4096:
      raise RuntimeError('malformed USB worker trace')
    return {int(match[1]) for line in lines if (match := EVENT.match(line.decode('ascii', errors='replace')))}

  def close(self):
    if self.fd is not None:
      os.close(self.fd)
      self.fd = None
    if self.path.exists():
      (self.path / 'tracing_on').write_text('0')
      (self.path / 'events/enable').write_text('0')
      self.path.rmdir()


def run(owner_fd=0):
  import fcntl
  import stat
  if not supported() or os.geteuid() != 0:
    print('Jetlink USB priority: unsupported hardware/kernel; unchanged', flush=True)
    return
  if not stat.S_ISFIFO(os.fstat(owner_fd).st_mode):
    raise ValueError('USB priority requires an owner-lifetime pipe')
  os.umask(0o077)
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, {0, 1, 2, 3})
  os.nice(10)
  with Path('/run/carrot-jetlink-usb-priority.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    leases = Leases(Path('/run/carrot-jetlink-usb-priority.json'), Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    trace = Trace()
    stopping = False

    def stop(*_):
      nonlocal stopping
      stopping = True

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
      signal.signal(sig, stop)
    try:
      leases.recover()
      trace.open()
      print('Jetlink USB priority: observing DWC3 completion workers', flush=True)
      while not stopping:
        ready, _, _ = select.select([owner_fd], [], [], .2)
        if ready and not os.read(owner_fd, 4096):
          break
        now = time.monotonic()
        for pid in trace.read():
          leases.observe(pid, now)
        leases.expire(now)
    finally:
      try:
        trace.close()
      finally:
        leases.expire(float('inf'))
      print('Jetlink USB priority: restored', flush=True)


if __name__ == '__main__':
  try:
    run()
  except Exception as exc:
    print(f'Jetlink USB priority: {exc}', file=sys.stderr, flush=True)
    sys.exit(2 if isinstance(exc, RestorationError) else 1)
