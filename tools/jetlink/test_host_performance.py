"""Performance samples remain observable without delaying inference."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

import host_performance as performance
from openpilot.selfdrive.modeld.jetlink.performance_log import PerformanceLog


def write(root, path, value):
  target = root / path.lstrip('/')
  target.parent.mkdir(parents=True, exist_ok=True)
  target.write_text(str(value))


def process(root, ticks, start=10):
  fields = ['0'] * 22
  fields[11], fields[12], fields[19] = str(ticks), '0', str(start)
  write(root, '/proc/123/stat', '123 (worker name) ' + ' '.join(fields))


@pytest.fixture
def sample_env(tmp_path, monkeypatch):
  clock = [10.]
  monkeypatch.setattr(performance.time, 'monotonic', lambda: clock[0])
  calls = []
  def query(*args, **kwargs):
    calls.append(args)
    return NS(returncode=0, stdout='NV Power Mode: MAXN_SUPER\n2\n')
  monkeypatch.setattr(performance.subprocess, 'run', query)
  write(tmp_path, '/proc/device-tree/model', 'Jetson Orin Nano\x00')
  write(tmp_path, '/proc/stat', 'cpu0 100 0 0 100 0 0 0 0\n')
  write(tmp_path, '/proc/meminfo', 'MemTotal: 8000000 kB\nMemAvailable: 2000000 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB')
  base = '/sys/devices/system/cpu/cpu0/cpufreq/'
  for filename, value in {'scaling_cur_freq': 1510400, 'scaling_min_freq': 729600,
                          'scaling_max_freq': 1510400, 'scaling_governor': 'schedutil'}.items():
    write(tmp_path, base + filename, value)
  base = '/sys/class/devfreq/17000000.gpu/'
  for filename, value in {'cur_freq': 1020000000, 'min_freq': 306000000,
                          'max_freq': 1020000000, 'governor': 'nvhost_podgov', 'device/load': 980}.items():
    write(tmp_path, base + filename, value)
  for filename, value in {'rate': 2133000000, 'max_rate': 2133000000, 'mrq_rate_locked': 1}.items():
    write(tmp_path, '/sys/kernel/debug/bpmp/debug/clk/emc/' + filename, value)
  for filename, value in {'name': 'ina3221', 'in1_label': 'VDD_IN', 'in1_input': 5000, 'curr1_input': 2000}.items():
    write(tmp_path, '/sys/class/hwmon/hwmon0/' + filename, value)
  process(tmp_path, 100)
  return tmp_path, clock, calls


def test_sample_clocks_units_cpu_deltas_and_pid_reuse(sample_env):
  root, clock, calls = sample_env
  sampler = performance.Sampler(root)
  sampler.hz = 100
  first = sampler.sample()
  assert first['power_mode']['name'] == 'MAXN_SUPER'
  assert first['gpu']['17000000.gpu']['clock_mhz'] == 1020
  assert first['gpu']['17000000.gpu']['load_pct'] == 98
  assert first['emc']['clock_mhz'] == 2133
  assert first['cpu'][0]['clock_mhz'] == 1510.4
  assert first['cpu'][0]['busy_pct'] is None
  assert first['top_processes'] == []
  assert first['power_rails'][0]['watts'] == 10
  clock[0] += 1
  write(root, '/proc/stat', 'cpu0 175 0 0 125 0 0 0 0\n')
  process(root, 250)
  second = sampler.sample()
  assert second['cpu'][0]['busy_pct'] == 75
  assert second['top_processes'][0]['cpu_pct'] == 150
  assert len(calls) == 1
  clock[0] += 30
  process(root, 9999, start=42)
  assert sampler.sample()['top_processes'] == []
  assert len(calls) == 2


def test_server_scheduler_wait_has_real_interval_and_role(sample_env):
  root, clock, _ = sample_env
  write(root, '/proc/123/cmdline', '/usr/bin/python3\x00/opt/carrot-jetlink/current/tools/jetlink/server.py\x00')
  fields = ['0'] * 37
  fields[19], fields[36] = '10', '4'
  write(root, '/proc/123/task/124/stat', '124 (python3) ' + ' '.join(fields))
  write(root, '/proc/123/task/124/schedstat', '1000000000 1000000 20')
  sampler = performance.Sampler(root)
  assert sampler.sample()['server_threads'] == []
  clock[0] += 1
  write(root, '/proc/123/task/124/schedstat', '1250000000 3000000 30')
  value = sampler.sample()
  assert value['server_threads'] == [{'tid': 124, 'cpu': 4, 'cpu_pct': 25, 'runqueue_wait_ms': 2, 'interval_s': 1}]
  assert value['top_processes'][0]['role'] == 'server'


def test_unavailable_and_stale_values_are_explicit(sample_env):
  root, clock, _ = sample_env
  (root / 'sys/kernel/debug/bpmp/debug/clk/emc/rate').unlink()
  value = performance.Sampler(root).sample()
  assert value['emc']['clock_mhz'] is None
  assert value['unavailable']['/sys/kernel/debug/bpmp/debug/clk/emc/rate'] == 'FileNotFoundError'
  status = root / 'status.json'
  status.write_text(json.dumps(value))
  assert performance.read_snapshot(status)['age_s'] == 0
  clock[0] += 3
  assert performance.read_snapshot(status) == {'status': 'stale'}
  status.write_text('{broken')
  assert performance.read_snapshot(status) == {'status': 'unavailable'}


@pytest.mark.parametrize('result', [NS(returncode=1, stdout=''), NS(returncode=0, stdout='unexpected')])
def test_mode_failure_preserves_other_sensors(sample_env, monkeypatch, result):
  root, _, _ = sample_env
  monkeypatch.setattr(performance.subprocess, 'run', lambda *a, **kw: result)
  value = performance.Sampler(root).sample()
  assert value['power_mode']['status'] == 'query_failed'
  assert value['emc']['clock_mhz'] == 2133


def test_cached_sample_reaches_route_event_and_expires(sample_env, monkeypatch):
  root, clock, _ = sample_env
  import openpilot.selfdrive.modeld.jetlink.performance_log as recorder
  monkeypatch.setattr(recorder.time, 'monotonic', lambda: clock[0])
  status = root / 'status.json'
  status.write_text(json.dumps(performance.Sampler(root).sample()))
  events = []
  logger = PerformanceLog(lambda name, **data: events.append((name, data)),
                          {'carrot_host': 'jetson', 'carrot_source_commit': 'a' * 40}, 'b' * 64)
  state = {'carrot_health': {'performance': performance.read_snapshot(status), 'age_s': .1,
                            'temp_c': 60, 'thermal_severity': 'ok', 'addresses': ['private']}}
  logger.record(state, 10, 200, 42, [39000, 800, 40000])
  name, event = events[0]
  assert name == 'jetsonPerformance' and event['status'] == 'ok'
  assert event['frame_id'] == 200 and event['usb_seq'] == 42 and event['server_execution_us'] == 39000
  assert event['sample']['emc']['clock_mhz'] == 2133
  assert 'private' not in json.dumps(event)
  logger.record(state, 10, 201, 43, [39000, 800, 40000])
  assert len(events) == 1
  clock[0] += 3
  logger.record(state, 10, 260, 60, [39000, 800, 40000])
  assert events[-1][1]['status'] == 'stale' and events[-1][1]['sample'] is None
  clock[0] += 1
  logger.record({}, clock[0], 280, 80, [39000, 800, 40000])
  assert events[-1][1]['status'] == 'unavailable'


def test_service_and_production_wiring():
  import install_updates
  unit = install_updates.PERFORMANCE_UNIT
  assert 'Nice=19' in unit and 'ProtectSystem=strict' in unit
  assert 'host_performance.py' in unit and 'PartOf=carrot-jetlink.service' in unit
  source = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/daemon.py'
  tree = ast.parse(source.read_text())
  loop = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_serve_local')
  calls = [ast.unparse(n.func) for n in ast.walk(loop) if isinstance(n, ast.Call)]
  assert 'performance_log.record' in calls
  assert 'time.monotonic() - last_state_request >= 1' in ast.unparse(loop)
  assert 'frame % 20' not in ast.unparse(loop)


def test_runtime_installs_only_volatile_observer(tmp_path, monkeypatch):
  import install_updates
  calls = []
  monkeypatch.setattr(install_updates.subprocess, 'run', lambda command, **kwargs: calls.append(command))
  unit = tmp_path / 'carrot-jetlink-performance-observer.service'
  install_updates.configure_runtime_performance(unit)
  assert unit.read_text() == install_updates.PERFORMANCE_UNIT
  assert calls == [['systemctl', 'daemon-reload'], ['systemctl', 'restart', unit.name]]


def test_observer_failure_does_not_block_bootstrap(tmp_path, monkeypatch):
  import install_updates
  def fail(*args, **kwargs):
    raise OSError('sudo unavailable')
  monkeypatch.setattr(install_updates.subprocess, 'run', fail)
  install_updates.migrate_running_release(tmp_path)


def test_log_sink_failure_and_non_jetson_are_isolated(sample_env):
  def fail(*args, **kwargs):
    raise OSError('log unavailable')
  logger = PerformanceLog(fail, {'carrot_host': 'jetson'}, None)
  logger.record({}, 10, 1, 1, [0, 0, 0])
  logger = PerformanceLog(lambda *a, **k: pytest.fail('non-Jetson record'), {'carrot_host': 'mac'}, None)
  logger.record({}, 10, 1, 1, [0, 0, 0])
