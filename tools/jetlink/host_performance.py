"""Read-only Jetson performance sampling, run outside the inference process."""
import json
import os
from pathlib import Path
import re
import subprocess
import time

STATUS = Path('/run/carrot-jetlink-performance/status.json')


class Sampler:
  def __init__(self, root=Path('/')):
    self.root = root
    self.previous_cpu = {}
    self.previous_processes = {}
    self.previous_threads = {}
    self.server_threads = []
    self.previous_time = None
    self.mode = {}
    self.mode_time = float('-inf')
    self.hz = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100
    self.errors = {}

  def read(self, path):
    try:
      return (self.root / path.lstrip('/')).read_text().strip().replace('\x00', '')[:16384]
    except OSError as exc:
      self.errors[path] = type(exc).__name__
      return None

  def number(self, path, scale=1):
    value = self.read(path)
    try:
      return round(int(value) / scale, 3)
    except (ValueError, TypeError):
      if value is not None:
        self.errors[path] = 'invalid_number'
      return None

  def power_mode(self, now):
    if now - self.mode_time >= 30:
      self.mode_time = now
      try:
        result = subprocess.run(['/usr/sbin/nvpmodel', '-q'], capture_output=True, text=True, timeout=1)
        match = re.search(r'NV Power Mode:\s*([^\r\n]+)\s*\n\s*(\d+)\s*$', result.stdout)
        self.mode = ({'name': match[1].strip()[:80], 'id': int(match[2]), 'status': 'ok'}
                     if result.returncode == 0 and match else {'status': 'query_failed', 'returncode': result.returncode})
      except (OSError, subprocess.SubprocessError):
        self.mode = {'status': 'query_unavailable'}
    return {**self.mode, 'age_s': round(now - self.mode_time, 3)}

  def cpu(self):
    current, usage = {}, {}
    for line in (self.read('/proc/stat') or '').splitlines():
      fields = line.split()
      if not fields or not re.fullmatch(r'cpu\d+', fields[0]):
        continue
      try:
        values = [int(v) for v in fields[1:9]]
        total, idle = sum(values), values[3] + values[4]
      except (ValueError, IndexError):
        continue
      name = fields[0]
      current[name] = total, idle
      previous = self.previous_cpu.get(name)
      usage[name] = (round(100 * (1 - (idle - previous[1]) / (total - previous[0])), 1)
                     if previous and total > previous[0] else None)
    self.previous_cpu = current
    result = []
    for path in sorted((self.root / 'sys/devices/system/cpu').glob('cpu[0-9]*'))[:32]:
      relative = '/' + path.relative_to(self.root).as_posix()
      online = True if path.name == 'cpu0' else self.read(relative + '/online') == '1'
      item = {'cpu': path.name, 'online': online, 'busy_pct': usage.get(path.name)}
      if online:
        item.update({key: self.number(relative + '/cpufreq/' + filename, 1000) for key, filename in (
          ('clock_mhz', 'scaling_cur_freq'), ('min_mhz', 'scaling_min_freq'), ('max_mhz', 'scaling_max_freq'))})
        item['governor'] = self.read(relative + '/cpufreq/scaling_governor')
      result.append(item)
    return result

  def processes(self, now):
    current, ranked, threads = {}, [], {}
    self.server_threads = []
    for path in (self.root / 'proc').glob('[0-9]*'):
      try:
        raw = (path / 'stat').read_text()
        end = raw.rindex(')')
        fields = raw[end + 2:].split()
        identity = (int(path.name), int(fields[19]))  # PID and starttime avoid PID reuse.
        ticks = int(fields[11]) + int(fields[12])
        current[identity] = ticks
        role = None
        try:
          command = (path / 'cmdline').read_bytes()[:4096].split(b'\0')
          for name in ('server.py', 'hud.py', 'hud_navi_receiver.py', 'host_performance.py'):
            if any(arg.endswith(('/tools/jetlink/' + name).encode()) for arg in command):
              role = name.removesuffix('.py')
              break
        except OSError:
          pass
        if role == 'server':
          for task in list((path / 'task').glob('[0-9]*'))[:128]:
            try:
              stat = (task / 'stat').read_text().rsplit(')', 1)[1].split()
              key = (identity, int(task.name), int(stat[19]))
              cpu, wait, _ = map(int, (task / 'schedstat').read_text().split())
              threads[key] = cpu, wait, now
              previous_thread = self.previous_threads.get(key)
              if previous_thread and now > previous_thread[2] and cpu >= previous_thread[0] and wait >= previous_thread[1]:
                self.server_threads.append({'tid': key[1], 'cpu': int(stat[36]),
                  'cpu_pct': round((cpu - previous_thread[0]) / 1e7 / (now - previous_thread[2]), 2),
                  'runqueue_wait_ms': round((wait - previous_thread[1]) / 1e6, 3),
                  'interval_s': round(now - previous_thread[2], 3)})
            except (OSError, ValueError, IndexError):
              continue
        previous = self.previous_processes.get(identity)
        if previous is not None and self.previous_time is not None and now > self.previous_time and ticks >= previous:
          ranked.append({'pid': identity[0], 'name': raw[raw.index('(') + 1:end][:32], 'role': role,
                         'cpu_pct': round(100 * (ticks - previous) / self.hz / (now - self.previous_time), 1)})
      except (OSError, ValueError, IndexError):
        continue  # Processes can exit during a sample.
    self.previous_processes, self.previous_time = current, now
    self.previous_threads = threads
    self.server_threads = sorted(self.server_threads,
                                key=lambda item: item['cpu_pct'] + item['runqueue_wait_ms'] / item['interval_s'] / 10,
                                reverse=True)[:8]
    return sorted(ranked, key=lambda item: item['cpu_pct'], reverse=True)[:5]

  def sample(self):
    started = time.monotonic()
    self.errors = {}
    # Board data excludes serial numbers and network identity.
    result = {'version': 1, 'sample_mono': started,
              'board': self.read('/proc/device-tree/model'), 'power_mode': self.power_mode(started),
              'cpu': self.cpu(), 'top_processes': self.processes(started)}
    result['server_threads'] = self.server_threads
    result['schedstats_enabled'] = self.read('/proc/sys/kernel/sched_schedstats')
    gpu_paths = [path for path in sorted((self.root / 'sys/class/devfreq').glob('*'))
                 if any(name in path.name for name in ('gpu', 'ga10b', 'gv11b'))]
    result['gpu'] = {}
    for path in gpu_paths[:2]:
      relative = '/' + path.relative_to(self.root).as_posix()
      result['gpu'][path.name] = {
        'clock_mhz': self.number(relative + '/cur_freq', 1e6),
        'min_mhz': self.number(relative + '/min_freq', 1e6),
        'max_mhz': self.number(relative + '/max_freq', 1e6),
        'governor': self.read(relative + '/governor'),
        'load_pct': self.number(relative + '/device/load', 10),
      }
    if not gpu_paths:
      self.errors['/sys/class/devfreq/*gpu*'] = 'not_found'
    emc = '/sys/kernel/debug/bpmp/debug/clk/emc/'
    result['emc'] = {key: self.number(emc + filename, scale) for key, filename, scale in (
      ('clock_mhz', 'rate', 1e6), ('max_mhz', 'max_rate', 1e6), ('rate_locked', 'mrq_rate_locked', 1))}
    result['cooling'] = []
    for path in sorted((self.root / 'sys/class/thermal').glob('cooling_device*'))[:32]:
      relative = '/' + path.relative_to(self.root).as_posix()
      result['cooling'].append({'name': self.read(relative + '/type'),
                                'state': self.number(relative + '/cur_state'), 'max_state': self.number(relative + '/max_state')})
    result['power_rails'] = []
    for path in sorted((self.root / 'sys/class/hwmon').glob('hwmon*')):
      if self.read('/' + (path / 'name').relative_to(self.root).as_posix()) == 'ina3221':
        for label in sorted(path.glob('in*_label'))[:8]:
          base = '/' + path.relative_to(self.root).as_posix() + '/'
          channel = label.stem.removeprefix('in').removesuffix('_label')
          voltage = self.number(base + f'in{channel}_input')
          current = self.number(base + f'curr{channel}_input')
          result['power_rails'].append({'name': self.read(base + label.name), 'mv': voltage, 'ma': current,
                                        'watts': round(voltage * current / 1e6, 3) if voltage is not None and current is not None else None})
    memory = {}
    for line in (self.read('/proc/meminfo') or '').splitlines():
      fields = line.split()
      if fields and fields[0] in ('MemTotal:', 'MemAvailable:', 'SwapTotal:', 'SwapFree:'):
        memory[fields[0].rstrip(':') + '_kb'] = int(fields[1])
    result.update(memory=memory, unavailable=dict(list(self.errors.items())[:24]),
                  sample_duration_ms=round((time.monotonic() - started) * 1000, 3))
    return result


def read_snapshot(path=STATUS):
  try:
    with path.open() as stream:
      value = json.loads(stream.read(16385))
    if not isinstance(value, dict):
      return {'status': 'invalid'}
    age = time.monotonic() - value['sample_mono']
    if value.get('version') == 1 and 0 <= age < 3:
      return {**value, 'age_s': round(age, 3)}
    return {'status': 'stale'}
  except (OSError, ValueError, TypeError, KeyError):
    return {'status': 'unavailable'}


def main():
  sampler = Sampler()
  while True:
    started = time.monotonic()
    try:
      value = sampler.sample()
    except Exception as exc:
      value = {'version': 1, 'sample_mono': started, 'status': 'sample_failed', 'error': type(exc).__name__}
    temporary = STATUS.with_suffix('.tmp')
    temporary.write_text(json.dumps(value), encoding='utf-8')
    temporary.chmod(0o644)
    temporary.replace(STATUS)
    time.sleep(max(.1, 1 - (time.monotonic() - started)))


if __name__ == '__main__':
  main()
