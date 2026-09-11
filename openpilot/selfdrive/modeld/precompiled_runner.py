"""ModelState-compatible client for the isolated precompiled AMD runtime."""
import json
import mmap
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


class PrecompiledModelState:
  def __init__(self, cam_w: int, cam_h: int, pkl_path: Path):
    self.pkl_path = pkl_path
    self.process = None
    self.shared = None
    self.views = {}
    self.output = None
    self.usbgpu = True
    self.first_run = True
    self.parser = Parser()
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    # A named temporary file permits independently opened mmap in the worker.
    self.file = tempfile.NamedTemporaryFile(prefix='carrot-model-', delete=False)
    try:
      worker = Path(__file__).with_name('precompiled_worker.py')
      self.process = subprocess.Popen([sys.executable, str(worker), str(pkl_path), self.file.name, str(cam_w), str(cam_h)],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
      info = json.loads(self._receive(38))
      self.shared = mmap.mmap(self.file.fileno(), info['size'])
      for name, spec in info['layout'].items():
        self.views[name] = np.ndarray(spec['shape'], np.dtype(spec['dtype']), buffer=self.shared, offset=spec['offset'])
      self.output = np.ndarray((info['output_count'],), np.float32, buffer=self.shared, offset=info['input_bytes'])
      self.output_slices = {k: slice(*v) for k, v in info['output_slices'].items()}
      self.input_shapes = info['input_shapes']
      self.vision_input_names = [name for name in self.input_shapes if 'img' in name]
      self.frame_size = info['frame_size']
      self.checkpoint = info['checkpoint']
    except BaseException:
      self.close()
      raise

  def _receive(self, timeout: float) -> bytes:
    with selectors.DefaultSelector() as selector:
      selector.register(self.process.stdout, selectors.EVENT_READ)
      if not selector.select(timeout):
        raise TimeoutError('precompiled eGPU worker timed out')
    value = self.process.stdout.readline()
    if not value:
      raise RuntimeError(f'precompiled eGPU worker exited ({self.process.poll()})')
    return value

  def run(self, bufs, transforms, inputs, prepare_only):
    for name in ('img', 'big_img'):
      data = np.frombuffer(bufs[name].data, dtype=np.uint8, count=self.frame_size)
      np.copyto(self.views[name], data)
    inputs['desire_pulse'][0] = 0
    self.views['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    for name in ('traffic_convention', 'action_t'):
      self.views[name][:] = inputs[name]
    self.views['tfm'][:] = transforms['img']
    self.views['big_tfm'][:] = transforms['big_img']
    try:
      self.process.stdin.write(b'r')
      if self._receive(20 if self.first_run else 1) != b'1\n':
        raise RuntimeError('invalid model worker response')
      self.first_run = False
      result = self.output.copy()
      if not np.isfinite(result).all():
        raise ValueError('non-finite model output')
    except BaseException as exc:
      self.close()
      # The manager sends SIGINT when ignition turns off. KeyboardInterrupt and
      # SystemExit release the worker but must not blacklist a healthy artifact.
      if isinstance(exc, Exception):
        from openpilot.selfdrive.modeld.precompiled_model import reject
        reject(self.pkl_path)
      raise
    self.views['prev_feat'][:] = result[self.output_slices['hidden_state']]
    # The fused graph advances image and policy history together, including dropped-frame catch-up.
    if prepare_only:
      return None
    outputs = self.parser.parse_outputs({k: result[np.newaxis, section] for k, section in self.output_slices.items()})
    if os.getenv('SEND_RAW_PRED'):
      outputs['raw_pred'] = result
    return outputs

  def close(self):
    if self.process is not None:
      if self.process.poll() is None:
        self.process.terminate()
        try:
          self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
          self.process.kill()
          self.process.wait()
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
        model.run(frames, transforms, inputs, False)
        timings.append(time.monotonic() - started)
      results.append({'camera': [width, height], 'checkpoint': model.checkpoint,
                      'load_seconds': load_seconds, 'inference_seconds': timings})
    finally:
      model.close()
  return results


if __name__ == '__main__':
  print(json.dumps(smoke_test(Path(sys.argv[1])), indent=2))
