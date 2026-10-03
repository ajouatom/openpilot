import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.system import proclog_smaps as smaps


@pytest.fixture
def clock(monkeypatch):
  state = SimpleNamespace(now=10.0, cpu=0.0)
  monkeypatch.setattr(smaps.time, 'monotonic', lambda: state.now)
  monkeypatch.setattr(smaps.time, 'thread_time', lambda: state.cpu)
  return state


def make_process(root, pid, start=123, data=b'Pss: 10 kB\nPss_Anon: 3 kB\nPss_Shmem: 2 kB\n'):
  path = root / str(pid)
  path.mkdir(exist_ok=True)
  fields = ['0'] * 50
  fields[0], fields[19] = 'S', str(start)
  (path / 'stat').write_text(f'{pid} (name with ) brackets) ' + ' '.join(fields))
  (path / 'smaps').write_bytes(data)


def advance(sampler, clock):
  clock.now = max(clock.now, sampler.next_step)
  sampler.step()


def test_chunk_boundaries_and_no_partial_publication(tmp_path, clock, monkeypatch):
  monkeypatch.setattr(smaps, 'READ_BYTES', 16)
  make_process(tmp_path, 1, data=b'Pss: 10 kB\nPss_Anon: 3 kB\nPss_Shmem: 2 kB\nPss: 4 kB')
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  assert sampler.file.tell() == 16
  assert sampler.get(1, 123, clock.now).mono_time == 0
  for _ in range(10):
    if sampler.active is None:
      break
    advance(sampler, clock)
  assert sampler.get(1, 123, clock.now) == smaps.SmapsSample(14*1024, 3*1024, 2*1024, 10.0)
  assert sampler.file is None


def test_many_processes_are_not_swept_in_one_step(tmp_path, clock):
  for pid in range(1, 46):
    make_process(tmp_path, pid)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update(dict.fromkeys(range(1, 46), 123))
  sampler.step()
  assert sampler.active == (1, 123)
  assert not sampler.cache
  sampler.step()  # same time: no second read or process
  assert not sampler.cache
  advance(sampler, clock)  # EOF completes only pid1
  assert list(sampler.cache) == [1]
  for _ in range(88):
    advance(sampler, clock)
  assert set(sampler.cache) == set(range(1, 46))
  assert len({sample.mono_time for sample in sampler.cache.values()}) == 45
  assert clock.now >= 10.0 + 89 * smaps.MIN_PAUSE_SECONDS - 1e-9


def test_refresh_no_catchup_and_expiry(tmp_path, clock):
  make_process(tmp_path, 1)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  advance(sampler, clock)
  sample = sampler.get(1, 123, clock.now)
  clock.now += 30
  sampler.step()
  assert sampler.active is None
  clock.now += 1000
  sampler.step()
  assert sampler.active == (1, 123)
  assert sampler.get(1, 123, clock.now).mono_time == 0  # stale cache is not reported
  assert sampler.cache[1] == sample  # replace only on completed scan
  sampler.close()


def test_cpu_duty_pause_and_slow_read_do_not_catch_up(tmp_path, clock, monkeypatch):
  make_process(tmp_path, 1)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  original = sampler._parse

  def costly_parse(*args, **kwargs):
    clock.cpu += .02
    clock.now += .3
    original(*args, **kwargs)

  monkeypatch.setattr(sampler, '_parse', costly_parse)
  sampler.step()
  assert sampler.next_step == pytest.approx(10.3 + .02 * 19)
  pos = sampler.file.tell()
  clock.now += .1
  sampler.step()
  assert sampler.file.tell() == pos
  sampler.close()


@pytest.mark.parametrize('stage', ['before_open', 'during_scan', 'after_cache'])
def test_pid_reuse_never_inherits_pss(tmp_path, clock, stage):
  make_process(tmp_path, 1)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  if stage != 'before_open':
    sampler.step()
  if stage == 'after_cache':
    advance(sampler, clock)
    assert sampler.cache[1].pss
  make_process(tmp_path, 1, start=456)
  if stage == 'during_scan':
    advance(sampler, clock)  # post-read identity check, without another stat snapshot
    assert not sampler.cache
  elif stage == 'before_open':
    sampler.step()
    assert sampler.file is None
  sampler.update({1: 456})
  assert sampler.get(1, 123, clock.now).mono_time == 0
  assert sampler.get(1, 456, clock.now).mono_time == 0
  assert sampler.file is None


def test_exit_and_small_process_remove_cache_and_close_reader(tmp_path, clock):
  make_process(tmp_path, 1)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  file = sampler.file
  sampler.update({})
  assert file.closed
  assert sampler.active is None
  assert not sampler.due and not sampler.cache


@pytest.mark.parametrize('data', [b'Pss: bad kB\n', b'Pss:\n', b'x' * (smaps.READ_BYTES * 3)],
                         ids=['invalid_number', 'missing_amount', 'oversized_line'])
def test_bad_scan_discards_partial_results(tmp_path, clock, data):
  make_process(tmp_path, 1, data=data)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  advance(sampler, clock)
  assert sampler.file is None
  assert not sampler.cache


def test_missing_file_is_nonfatal(tmp_path, clock):
  make_process(tmp_path, 1)
  (tmp_path / '1' / 'smaps').unlink()
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  assert sampler.file is None
  assert sampler.due[1] > clock.now


def test_rollup_preserves_anonymous_and_shared_pss(tmp_path, clock):
  make_process(tmp_path, 1, data=b'Pss: 99 kB\n')
  (tmp_path / '1' / 'smaps_rollup').write_bytes(b'Pss: 20 kB\nPss_Anon: 8 kB\nPss_Shmem: 7 kB\n')
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.step()
  advance(sampler, clock)
  assert sampler.get(1, 123, clock.now) == smaps.SmapsSample(20*1024, 8*1024, 7*1024, 10.0)


@pytest.mark.skipif(sys.platform != 'linux', reason='requires real Linux procfs')
def test_linux_procfs_scan_completes(clock):
  import os
  sampler = smaps.SmapsSampler()
  pid = os.getpid()
  identity = sampler._starttime(pid)
  sampler.update({pid: identity})
  try:
    for _ in range(2000):
      advance(sampler, clock)
      if pid in sampler.cache:
        break
    sample = sampler.get(pid, identity, clock.now)
    assert sample.pss > 0
    assert sample.pss >= sample.pss_anon + sample.pss_shmem
    assert clock.now - sample.mono_time <= smaps.MAX_AGE_SECONDS
  finally:
    sampler.close()


@pytest.fixture
def proclog_module(monkeypatch):
  # Import the real builder with only Windows-incompatible infrastructure stubbed.
  import os
  monkeypatch.setattr(os, 'sysconf', lambda key: 4096 if key == 'SC_PAGE_SIZE' else 100, raising=False)
  monkeypatch.setattr(os, 'sysconf_names', {'SC_CLK_TCK': 'SC_CLK_TCK', 'SC_PAGE_SIZE': 'SC_PAGE_SIZE'}, raising=False)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal.messaging', SimpleNamespace())
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(exception=lambda *_: None)))
  spec = importlib.util.spec_from_file_location('_test_proclogd', Path(__file__).parents[1] / 'proclogd.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_proclog_publication_reads_cache_without_smaps_io(tmp_path, clock, monkeypatch, proclog_module):
  module = proclog_module
  stat = {'pid': 1, 'name': 'test', 'state': 'S', 'ppid': 0, 'utime': 100, 'stime': 200, 'cutime': 0, 'cstime': 0,
          'priority': 20, 'nice': 0, 'num_threads': 1, 'starttime': 123, 'vms': 10000000, 'rss': 2000, 'processor': 4}
  monkeypatch.setattr(module, '_procs', lambda: [stat])
  monkeypatch.setattr(module, '_get_proc_extra', lambda *_: {'exe': '', 'cmdline': []})
  monkeypatch.setattr(module, '_cpu_times', list)
  monkeypatch.setattr(module, '_mem_info', lambda: dict.fromkeys(
    ['MemTotal:', 'MemFree:', 'MemAvailable:', 'Buffers:', 'Cached:', 'Active:', 'Inactive:', 'Shmem:'], 0))
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123})
  sampler.cache[1] = smaps.SmapsSample(1024, 512, 256, 9.0)
  monkeypatch.setattr(sampler, 'step', lambda: pytest.fail('publication must not scan'))
  event = log.Event.new_message()
  event.init('procLog')
  module.build_proc_log_message(event, sampler)
  proc = event.procLog.procs[0]
  assert (proc.cpuUser, proc.cpuSystem, proc.memRss) == (1, 2, 2000*4096)
  assert (proc.memPss, proc.memPssAnon, proc.memPssShmem, proc.memPssMonoTime) == (1024, 512, 256, 9_000_000_000)
  clock.now = 130
  module.build_proc_log_message(log.Event.new_message(procLog={}), sampler)
  assert sampler.get(1, 123, clock.now).mono_time == 0


def test_incomplete_scan_expires_and_other_processes_progress(tmp_path, clock):
  make_process(tmp_path, 1)
  make_process(tmp_path, 2)
  sampler = smaps.SmapsSampler(tmp_path)
  sampler.update({1: 123, 2: 123})
  sampler.step()
  clock.now += smaps.MAX_AGE_SECONDS + 1
  sampler.step()
  assert sampler.file is None and not sampler.cache
  advance(sampler, clock)
  assert sampler.active == (2, 123)
  advance(sampler, clock)
  assert sampler.cache[2].pss == 10 * 1024


def test_main_keeps_lightweight_cadence_and_never_replays_missed_slots(tmp_path, clock, monkeypatch, proclog_module):
  module = proclog_module
  published = []
  sampler = smaps.SmapsSampler(tmp_path)
  original_step = sampler.step
  stalled = False

  def step():
    nonlocal stalled
    if clock.now >= 12 and not stalled:
      clock.now += 5  # an individual slow kernel read cannot be preempted
      stalled = True
    original_step()

  class Finished(Exception):
    pass

  def publish(*_):
    published.append(clock.now)
    if len(published) == 5:
      raise Finished

  monkeypatch.setattr(sampler, 'step', step)
  monkeypatch.setattr(module, 'SmapsSampler', lambda: sampler)
  monkeypatch.setattr(module, 'build_proc_log_message', lambda *_: None)
  monkeypatch.setattr(module, 'messaging', SimpleNamespace(
    PubMaster=lambda *_: SimpleNamespace(send=publish), new_message=lambda *_a, **_kw: None))
  monkeypatch.setattr(module.time, 'sleep', lambda duration: setattr(clock, 'now', clock.now + duration))
  with pytest.raises(Finished):
    module.main()
  assert published == pytest.approx([10, 12, 17, 19, 21])
  assert sampler.file is None
