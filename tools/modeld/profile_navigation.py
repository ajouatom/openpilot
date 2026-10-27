"""Measure whole-core load on a target, or benchmark the CPU map renderer locally.

python tools/modeld/profile_navigation.py --seconds 60 --output /tmp/nav-cpu.json
python tools/modeld/profile_navigation.py --benchmark --output /tmp/nav-render.json
"""
import argparse
import json
from pathlib import Path
import platform
import time


def sample(seconds):
  import psutil
  records, processes = [], {}
  psutil.cpu_percent(percpu=True)
  deadline = time.monotonic() + seconds
  while time.monotonic() < deadline:
    for process in psutil.process_iter(['pid', 'name', 'cmdline']):
      label = ' '.join(process.info['cmdline'] or [process.info['name'] or ''])
      if any(name in label for name in ('mapsd', 'modeld', 'carrot_man', 'controlsd', 'radard', 'camerad', 'cluster')):
        if process.pid not in processes:
          process.cpu_percent()
          processes[process.pid] = process
    cores = psutil.cpu_percent(interval=1., percpu=True)
    work = []
    for pid, process in list(processes.items()):
      try:
        work.append({'pid': pid, 'name': process.name(), 'cpu_percent': process.cpu_percent(),
                     'affinity': process.cpu_affinity(), 'threads': process.num_threads()})
      except (psutil.NoSuchProcess, psutil.AccessDenied):
        processes.pop(pid, None)
    records.append({'cores': cores, 'processes': work})
  return {'host': platform.machine(), 'system': platform.system(), 'seconds': seconds,
          'core_mean_percent': [sum(r['cores'][i] for r in records) / len(records) for i in range(len(records[0]['cores']))],
          'core_max_percent': [max(r['cores'][i] for r in records) for i in range(len(records[0]['cores']))], 'samples': records}


def benchmark(frames=120, paced=False):
  import numpy as np
  from types import SimpleNamespace
  from openpilot.selfdrive.navigation.mapsd import render_interval
  from openpilot.selfdrive.navigation.renderer import MapRenderer, Position, mercator, _native_draw_segments
  # Dense synthetic local roads, with a maximum-size route. No network or GPU.
  center = np.array([127., 37.])
  scale = np.array([111320 * np.cos(np.radians(center[1])), 110540])
  offsets = np.linspace(-180, 180, 60)
  starts = np.vstack((np.column_stack((offsets, np.full(60, -200))), np.column_stack((np.full(60, -200), offsets))))
  ends = np.vstack((np.column_stack((offsets, np.full(60, 250))), np.column_stack((np.full(60, 250), offsets))))
  renderer = MapRenderer(SimpleNamespace(min_zoom=15, max_zoom=15))
  tiles, _ = renderer.view_tiles(*center)
  data = (mercator(center + starts / scale), mercator(center + ends / scale), np.full(120, 2),
          np.zeros(120), np.ones((120, 2), bool))
  empty = (np.empty((0, 2)), np.empty((0, 2)), np.empty(0, int), np.empty(0), np.empty((0, 2), bool))
  renderer.roads = lambda x, y: data if (x, y) == tiles[0] else empty
  route = center + np.column_stack((np.zeros(4096), np.linspace(-200, 100000, 4096))) / scale
  wall, cpu = [], []
  for frame in range(frames):
    start, cpu_start = time.perf_counter(), time.process_time()
    result = renderer.render(Position(center[1], center[0], frame / 4), route)
    cpu.append(time.process_time() - cpu_start)
    wall.append(time.perf_counter() - start)
    assert result.valid and result.image.shape == (256, 256)
    if paced:
      time.sleep(render_interval(cpu[-1]))
  mean = float(np.mean(cpu))
  return {'kind': 'synthetic dense roads; not road validation', 'host': platform.machine(),
          'native_rasterizer': _native_draw_segments is not None, 'frames': len(cpu),
          'wall_ms_mean': float(np.mean(wall) * 1000), 'wall_ms_p95': float(np.percentile(wall, 95) * 1000),
          'cpu_ms_mean': mean * 1000, 'uncapped_20hz_one_core_percent': mean * 20 * 100,
          'adaptive_one_core_percent': mean / (float(np.mean(wall)) + render_interval(mean)) * 100}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--seconds', type=int, default=30)
  parser.add_argument('--benchmark', action='store_true')
  parser.add_argument('--frames', type=int, default=120)
  parser.add_argument('--paced', action='store_true', help='apply the live render duty limit during a target benchmark')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if not 1 <= args.seconds <= 3600:
    parser.error('seconds must be between 1 and 3600')
  if not 1 <= args.frames <= 600:
    parser.error('frames must be between 1 and 600')
  report = benchmark(args.frames, args.paced) if args.benchmark else sample(args.seconds)
  args.output.write_text(json.dumps(report, indent=2) + '\n')
  print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2))
