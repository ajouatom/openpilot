"""Separate CPU observer. No experimental output is returned to modeld/control."""
import argparse
import hashlib
import json
import os
import pickle
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

SHADOW_DIR = Path('/data/signal-model-shadow')
BASE_SIZE = 2576
SHADOW_SIZE = 99
ROWS = np.array([15 * t + c for t in range(33) for c in (0, 3, 6)])
BASE_ROWS = 1576 + ROWS


def sha256_file(path):
  digest = hashlib.sha256()
  with open(path, 'rb') as stream:
    for data in iter(lambda: stream.read(1024 * 1024), b''):
      digest.update(data)
  return digest.hexdigest()


def validate_metadata(meta):
  if not isinstance(meta, dict) or meta.get('version') != 2 or meta.get('mode') != 'comparison_only':
    raise ValueError('not a separate comparison-only policy model')
  if meta.get('base_size') != BASE_SIZE or meta.get('shadow_size') != SHADOW_SIZE:
    raise ValueError('signal comparison output layout mismatch')
  for key in ('base_sha256', 'candidate_sha256'):
    value = meta.get(key, '')
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
      raise ValueError(f'invalid {key}')
  return meta


def selected_artifact(base_onnx, directory=SHADOW_DIR):
  directory = Path(directory)
  if not (directory / 'enabled').is_file() or (directory / 'enabled').read_text().strip() != '1':
    return None
  manifest = json.loads((directory / 'installed.json').read_text())
  meta = validate_metadata(manifest['signal_shadow'])
  artifact = directory / 'policy_shadow.onnx'
  if sha256_file(base_onnx) != meta['base_sha256'] or sha256_file(artifact) != manifest['policy_sha256']:
    raise ValueError('signal comparison artifact checksum mismatch')
  return artifact, meta


class ShadowHistory:
  """Mirror original queues: previous features and max-pooled desire pulses."""
  def __init__(self):
    self.features = np.zeros((96, 512), np.float32)
    self.desires = np.zeros((100, 8), np.float32)
    self.count = 0

  def capture(self, output, inputs):
    self.features[:-1] = self.features[1:]
    self.features[-1] = inputs['prev_feat'].reshape(512)
    self.desires[:-1] = self.desires[1:]
    self.desires[-1] = inputs['desire']
    self.count += 1
    if self.count % 4:
      return None
    feeds = {'features_buffer': self.features[::4][None].astype(np.float16),
             'desire_pulse': self.desires.reshape(25, 4, 8).max(1)[None].astype(np.float16),
             'current_hidden_flat': output[1064:1576][None].astype(np.float16),
             'traffic_convention': inputs['traffic_convention'].astype(np.float16).copy(),
             'action_t': inputs['action_t'].astype(np.float16).copy()}
    return {'feeds': feeds, 'baseline': output[BASE_ROWS].copy()}


class ShadowClient:
  @classmethod
  def optional(cls, base_onnx):
    try:
      return cls() if selected_artifact(base_onnx) is not None else None
    except Exception:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception('signal comparison unavailable; original model remains active')
      return None

  def __init__(self):
    self.history = ShadowHistory()
    self.failed = False
    self.dropped = 0
    self.sock, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    self.sock.setblocking(False)
    try:
      env = {**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
      self.process = subprocess.Popen(['chrt', '-o', '0', 'nice', '-n', '19', 'taskset', '-c', '0-3',
                                       sys.executable, '-m', 'openpilot.selfdrive.modeld.signal_shadow',
                                       '--worker', str(child.fileno()), '--parent', str(os.getpid())],
                                      pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL, env=env)
    except Exception:
      self.sock.close()
      raise
    finally:
      child.close()

  def capture(self, output, inputs):
    if self.failed:
      return None
    try:
      sample = self.history.capture(output, inputs)
      return (self, sample) if sample is not None else None
    except Exception:
      self.failed = True
      return None

  def submit(self, sample, frame_id, frame_id_extra, timestamp_eof):
    if self.failed:
      return
    try:
      if self.process.poll() is not None:
        self.failed = True
        self.sock.close()
        return
      sample.update(frame_id=int(frame_id), frame_id_extra=int(frame_id_extra), timestamp_eof=int(timestamp_eof),
                    dropped=self.dropped, submitted=time.monotonic())
      self.sock.send(pickle.dumps(sample, protocol=5))
    except BlockingIOError:
      self.dropped += 1
    except Exception:
      self.failed = True
      self.sock.close()


def worker(fd, parent):
  # Private socketpair; no modelV2/control publication or prediction return path.
  os.sched_setaffinity(0, set(os.sched_getaffinity(0)) & {0, 1, 2, 3})
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.nice(19 - os.getpriority(os.PRIO_PROCESS, 0))
  from openpilot.common.swaglog import cloudlog
  sys.path.insert(0, str(SHADOW_DIR / 'runtime'))
  import onnxruntime as ort
  manifest = json.loads((SHADOW_DIR / 'installed.json').read_text())
  meta = validate_metadata(manifest['signal_shadow'])
  path = SHADOW_DIR / 'policy_shadow.onnx'
  if sha256_file(path) != manifest['policy_sha256']:
    raise ValueError('comparison worker policy checksum mismatch')
  options = ort.SessionOptions()
  options.intra_op_num_threads = 1
  options.inter_op_num_threads = 1
  options.add_session_config_entry('session.intra_op.allow_spinning', '0')
  session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
  sock = socket.socket(fileno=fd)
  sock.settimeout(1)
  cloudlog.event('signalModelShadowLoaded', **meta, runtime=ort.__version__, separate_cpu_worker=True)
  while os.getppid() == parent:
    try:
      data = sock.recv(131072)
    except TimeoutError:
      continue
    # Only the parent can write this inherited, unnamed socketpair.
    sample = pickle.loads(data)
    if time.monotonic() - sample['submitted'] > .4:
      continue
    start = time.perf_counter()
    policy, candidate = session.run(None, sample['feeds'])
    elapsed = (time.perf_counter() - start) * 1000
    arrays = {'baseline': sample['baseline'], 'policy_baseline': policy[0, ROWS], 'candidate': candidate[0]}
    if any(a.size != SHADOW_SIZE or not np.isfinite(a).all() for a in arrays.values()):
      cloudlog.event('signalModelShadowDisabled', reason='nonfinite or malformed comparison output')
      break
    fields = {f'{name}_{axis}': values.reshape(33, 3)[:, i].astype(float).round(5).tolist()
              for name, values in arrays.items() for i, axis in enumerate(('x', 'v', 'a'))}
    cloudlog.event('signalModelShadow', mode='comparison_only', frame_id=sample['frame_id'],
                  frame_id_extra=sample['frame_id_extra'], timestamp_eof=sample['timestamp_eof'],
                  base_sha256=meta['base_sha256'], candidate_sha256=meta['candidate_sha256'],
                  inference_ms=elapsed, dropped=sample['dropped'], **fields)
  sock.close()


def main():
  parser = argparse.ArgumentParser(description='Comparison logging at next modeld startup; original model always controls')
  parser.add_argument('mode', nargs='?', choices=['on', 'off', 'status'])
  parser.add_argument('--worker', type=int)
  parser.add_argument('--parent', type=int)
  args = parser.parse_args()
  if args.worker is not None:
    worker(args.worker, args.parent)
    return
  if args.mode in ('on', 'off'):
    if args.mode == 'on':
      validate_metadata(json.loads((SHADOW_DIR / 'installed.json').read_text())['signal_shadow'])
    SHADOW_DIR.mkdir(parents=True, exist_ok=True)
    temporary = SHADOW_DIR / 'enabled.tmp'
    temporary.write_text('1' if args.mode == 'on' else '0')
    temporary.replace(SHADOW_DIR / 'enabled')
  print(json.dumps({'comparison_requested': (SHADOW_DIR / 'enabled').is_file() and
                   (SHADOW_DIR / 'enabled').read_text().strip() == '1',
                   'applies': 'next modeld startup', 'control': 'original model only'}))


if __name__ == '__main__':
  main()
