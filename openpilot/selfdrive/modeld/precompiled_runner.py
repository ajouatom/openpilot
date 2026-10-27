"""ModelState-compatible client for the isolated precompiled AMD runtime."""
import json
import mmap
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.egpu_worker_progress import worker_snapshot
from openpilot.selfdrive.modeld.egpu_output_recovery import MAX_HOLD_SECONDS

INFERENCE_TIMEOUT = .15  # three 20 Hz periods; first-use initialization has its own budget

_retiring_workers = []
_retiring_lock = threading.Lock()


def _reap_worker(process):
  # An uninterruptible kernel USB operation can delay even SIGKILL. Keep its
  # ownership visible, but never wait for it on modeld's publication thread.
  process.wait()
  with _retiring_lock:
    _retiring_workers.remove(process)


class PrecompiledModelState:
  def __init__(self, cam_w: int, cam_h: int, pkl_path: Path, *, recovery_cancel=None):
    self.pkl_path = pkl_path
    self.process = None
    with _retiring_lock:
      if any(process.poll() is None for process in _retiring_workers):
        raise RuntimeError('previous eGPU worker is still exiting')
    self.shared = None
    self.views = {}
    self.output = None
    self.usbgpu = True
    self.first_run = True
    self.last_good_time = None
    self.held_output = False
    self.recovery_cancel = recovery_cancel
    self.parser = Parser()
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    # A named temporary file permits independently opened mmap in the worker.
    self.file = tempfile.NamedTemporaryFile(prefix='carrot-model-', delete=False)
    try:
      worker = Path(__file__).with_name('precompiled_worker.py')
      arguments = [sys.executable, str(worker), str(pkl_path), self.file.name, str(cam_w), str(cam_h)]
      if recovery_cancel is not None:
        arguments.append('--recovery')
      self.process = subprocess.Popen(arguments,
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
      info = json.loads(self._receive(110))
      self.shared = mmap.mmap(self.file.fileno(), info['size'])
      for name, spec in info['layout'].items():
        self.views[name] = np.ndarray(spec['shape'], np.dtype(spec['dtype']), buffer=self.shared, offset=spec['offset'])
      self.output = np.ndarray((info['output_count'],), np.float32, buffer=self.shared, offset=info['input_bytes'])
      self.output_slices = {k: slice(*v) for k, v in info['output_slices'].items()}
      self.input_shapes = info['input_shapes']
      self.vision_input_names = [name for name in self.input_shapes if 'img' in name]
      self.frame_size = info['frame_size']
      self.checkpoint = info['checkpoint']
    except BaseException as exc:
      self.failed = True
      self.close()
      if isinstance(exc, Exception) and not (self.recovery_cancel is not None and self.recovery_cancel.is_set()):
        from openpilot.selfdrive.modeld.precompiled_model import record_failure
        record_failure(self.pkl_path, exc, 'load')
      raise

  def _receive(self, timeout: float) -> bytes:
    with selectors.DefaultSelector() as selector:
      selector.register(self.process.stdout, selectors.EVENT_READ)
      deadline, ready, value = time.monotonic() + timeout, False, b''
      while not ready and time.monotonic() < deadline:
        if self.recovery_cancel is not None and self.recovery_cancel.is_set():
          raise InterruptedError('eGPU recovery cancelled')
        if selector.select(min(.05, max(0, deadline - time.monotonic()))):
          chunk = os.read(self.process.stdout.fileno(), 65536)
          if not chunk:
            raise RuntimeError(f'precompiled eGPU worker exited ({self.process.poll()})')
          value += chunk
          if len(value) > 128 * 1024:
            raise RuntimeError('oversized eGPU worker response')
          ready = value.endswith(b'\n')
      if not ready:
        error = TimeoutError('precompiled eGPU worker timed out')
        error.worker_diagnostics = worker_snapshot(Path(self.file.name + '.progress'), self.process.pid)
        raise error
    if not value:
      raise RuntimeError(f'precompiled eGPU worker exited ({self.process.poll()})')
    if value.startswith(b'ERROR '):
      raise RuntimeError(json.loads(value[6:]))
    return value

  def reset_after_probe(self):
    self.process.stdin.write(b'z')
    if self._receive(1) != b'Z\n':
      raise RuntimeError('invalid eGPU recovery reset response')
    self.prev_desire[:] = 0
    self.last_good_time = None
    self.held_output = False

  def run(self, bufs, transforms, inputs, prepare_only):
    for name in ('img', 'big_img'):
      data = np.frombuffer(bufs[name].data, dtype=np.uint8, count=self.frame_size)
      np.copyto(self.views[name], data)
    inputs['desire_pulse'][0] = 0
    previous_desire = self.prev_desire.copy()
    self.views['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    for name in ('traffic_convention', 'action_t'):
      self.views[name][:] = inputs[name]
    for name in ('nav_map', 'navigation'):
      if name in self.views:
        self.views[name][:] = inputs.get(name, 0)
    self.views['tfm'][:] = transforms['img']
    self.views['big_tfm'][:] = transforms['big_img']
    try:
      self.process.stdin.write(b'r')
      response = self._receive(20 if self.first_run else INFERENCE_TIMEOUT)
      if (response == b'0\n' and not self.first_run and not self.held_output and self.last_good_time is not None
          and 0 <= time.monotonic() - self.last_good_time <= MAX_HOLD_SECONDS):
        # Worker restored recurrent history. Preserve the matching pulse history
        # and let consumers retain their last timestamped result for this cycle.
        self.prev_desire[:] = previous_desire
        self.held_output = True
        return None
      if response != b'1\n':
        raise RuntimeError('invalid model worker response')
      self.first_run = False
      result = self.output.copy()
      if not np.isfinite(result).all():
        raise ValueError('non-finite model output')
      self.last_good_time = time.monotonic()
      self.held_output = False
    except BaseException as exc:
      self.failed = True
      self.close()
      # The manager sends SIGINT when ignition turns off. KeyboardInterrupt and
      # SystemExit release the worker but must not blacklist a healthy artifact.
      cancel = getattr(self, 'recovery_cancel', None)
      if isinstance(exc, Exception) and not (cancel is not None and cancel.is_set()):
        from openpilot.selfdrive.modeld.precompiled_model import record_failure
        record_failure(self.pkl_path, exc, 'inference')
      raise
    if 'prev_feat' in self.views:
      self.views['prev_feat'][:] = result[self.output_slices['hidden_state']]
    # Generic ONNX artifacts advance all recurrent state inside their graph.
    # The fused graph advances image and policy history together, including dropped-frame catch-up.
    # It also produces a complete current-frame policy: publishing it avoids
    # an extra output gap after a dropped camera frame. The split backend still
    # uses prepare_only to catch up its image history without policy inference.
    outputs = self.parser.parse_outputs({k: result[np.newaxis, section] for k, section in self.output_slices.items()})
    if os.getenv('SEND_RAW_PRED'):
      outputs['raw_pred'] = result
    return outputs

  def close(self):
    if self.process is not None:
      if self.process.poll() is None:
        # A failed worker has no recoverable GPU state. Do not spend two more
        # seconds in driver/atexit cleanup before the internal model can run.
        failed = getattr(self, 'failed', False)
        if failed:
          self.process.kill()
        else:
          self.process.terminate()
        try:
          self.process.wait(timeout=.1 if failed else 2)
        except subprocess.TimeoutExpired:
          if not failed:
            self.process.kill()
          with _retiring_lock:
            _retiring_workers.append(self.process)
          threading.Thread(target=_reap_worker, args=(self.process,), name='egpu-reap', daemon=True).start()
      self.process.stdin.close()
      self.process.stdout.close()
      self.process = None
    self.views.clear()
    self.output = None
    if self.shared is not None:
      self.shared.close()
      self.shared = None
    self.file.close()
    Path(self.file.name).unlink(missing_ok=True)
    Path(self.file.name + '.progress').unlink(missing_ok=True)

  def __del__(self):
    if getattr(self, 'process', None) is not None:
      self.close()


def smoke_test(path: Path, camera_sizes=((1928, 1208), (1344, 760)), runs=5):
  """Load and execute the exact downloaded graph, without compiling model kernels."""
  from types import SimpleNamespace
  import time
  results = []
  for width, height in camera_sizes:
    started = time.monotonic()
    model = PrecompiledModelState(width, height, path)
    load_seconds = time.monotonic() - started
    try:
      frames = {k: SimpleNamespace(data=np.zeros(model.frame_size, dtype=np.uint8)) for k in ('img', 'big_img')}
      transforms = {k: np.eye(3, dtype=np.float32) for k in frames}
      inputs = {'desire_pulse': np.zeros(ModelConstants.DESIRE_LEN, np.float32),
                'traffic_convention': np.array([1, 0], np.float32), 'action_t': np.zeros(2, np.float32)}
      timings = []
      for _ in range(runs):
        started = time.monotonic()
        if model.run(frames, transforms, inputs, False) is None:
          raise ValueError('invalid output during precompiled smoke test')
        timings.append(time.monotonic() - started)
      results.append({'camera': [width, height], 'checkpoint': model.checkpoint,
                      'load_seconds': load_seconds, 'inference_seconds': timings})
    finally:
      model.close()
  return results


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(description='Validate a precompiled model on selected camera resolutions')
  parser.add_argument('path', type=Path)
  parser.add_argument('--camera', action='append', choices=('1928x1208', '1344x760'))
  args = parser.parse_args()
  sizes = tuple(tuple(map(int, size.split('x'))) for size in (args.camera or ('1928x1208', '1344x760')))
  print(json.dumps(smoke_test(args.path, camera_sizes=sizes), indent=2))
