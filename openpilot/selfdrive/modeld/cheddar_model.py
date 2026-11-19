"""ModelState interface for the isolated internal Cheddar execution process."""
import json
import mmap
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile
import time

import numpy as np

from openpilot.selfdrive.modeld.cheddar_runtime import CHECKPOINT
from openpilot.selfdrive.modeld.helpers import get_tg_input_devices
from openpilot.selfdrive.modeld.output_validation import validate_compiled_output
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


class CheddarModelState:
  vision_input_names = ('img', 'big_img')
  usbgpu = False
  signal_shadow = None  # The optional observer's CD210 feature contract is incompatible.

  def __init__(self, cam_w, cam_h, pkl_path):
    self.process = self.shared = self.file = None
    self.views = {}
    self.output = None
    self.prev_desire = np.zeros(8, np.float32)
    self.parser = Parser()
    try:
      self.file = tempfile.NamedTemporaryFile(prefix='carrot-internal-', delete=False)
      worker = Path(__file__).with_name('cheddar_worker.py')
      device = get_tg_input_devices('openpilot.selfdrive.modeld.modeld', False)['WARP_DEV']
      # The device file stores the backend name, while Linux compilation uses
      # the LLVM renderer. Preserve that choice for runtime transfer kernels.
      if device == 'CPU' and sys.platform != 'darwin':
        device = 'CPU:LLVM'
      self.inference_timeout = .15 if device.split(':')[0] == 'QCOM' else 10
      env = os.environ | {'DEV': device, 'IMAGE': '1', 'FLOAT16': '1', 'NOLOCALS': '1',
                          'JIT_BATCH_SIZE': '0', 'OPENPILOT_HACKS': '1'}
      self.process = subprocess.Popen([sys.executable, str(worker), str(pkl_path), self.file.name, str(cam_w), str(cam_h)],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0, env=env)
      self.worker_info = info = json.loads(self._receive(110))
      if info['checkpoint'] != CHECKPOINT or info['output_count'] != 2580:
        raise ValueError('internal worker model contract mismatch')
      self.shared = mmap.mmap(self.file.fileno(), info['size'])
      self.views = {name: np.ndarray(spec['shape'], np.dtype(spec['dtype']), buffer=self.shared, offset=spec['offset'])
                    for name, spec in info['layout'].items()}
      self.output = np.ndarray((info['output_count'],), np.float32, buffer=self.shared, offset=info['input_bytes'])
      self.frame_size = info['frame_size']
      self.input_shapes = {name: [self.frame_size] for name in self.vision_input_names}
      self.output_slices = {name: slice(*value) for name, value in info['output_slices'].items()}
    except BaseException:
      self.close()
      raise

  def _receive(self, timeout):
    with selectors.DefaultSelector() as selector:
      selector.register(self.process.stdout, selectors.EVENT_READ)
      deadline, response = time.monotonic() + timeout, b''
      while time.monotonic() < deadline:
        if selector.select(max(0, deadline - time.monotonic())):
          chunk = os.read(self.process.stdout.fileno(), 16384)
          if not chunk:
            raise RuntimeError(f'internal model worker exited ({self.process.poll()})')
          response += chunk
          if len(response) > 32768:
            raise ValueError('oversized internal worker response')
          if response.endswith(b'\n'):
            return response
    raise TimeoutError('internal model worker timed out')

  def reset(self):
    try:
      self.process.stdin.write(b'z')
      if self._receive(1) != b'Z\n':
        raise RuntimeError('invalid internal reset response')
      self.prev_desire[:] = 0
    except BaseException:
      self.close()
      raise

  def run(self, bufs, transforms, inputs, prepare_only):
    try:
      for name in self.vision_input_names:
        np.copyto(self.views[name], np.frombuffer(bufs[name].data, np.uint8, count=self.frame_size))
      desire = np.asarray(inputs['desire_pulse']).copy()
      desire[0] = 0
      self.views['desire'][:] = np.where(desire - self.prev_desire > .99, desire, 0)
      self.prev_desire[:] = desire
      for name in ('traffic_convention', 'action_t'):
        self.views[name][:] = inputs[name]
      self.views['tfm'][:] = transforms['img']
      self.views['big_tfm'][:] = transforms['big_img']
      self.process.stdin.write(b'r')
      if self._receive(self.inference_timeout) != b'1\n':
        raise RuntimeError('invalid internal inference response')
      result = self.output.copy()
      validate_compiled_output(result, 2580)
    except BaseException:
      self.close()
      raise
    # Cheddar advances image/desire/feature history together inside ONNX. Even
    # catch-up calls produce a complete current-frame policy, as upstream does.
    raw = result.copy() if os.getenv('SEND_RAW_PRED') else None
    outputs = self.parser.parse_outputs({name: result[np.newaxis, section] for name, section in self.output_slices.items()})
    if os.getenv('SEND_RAW_PRED'):
      outputs['raw_pred'] = raw
    return outputs

  def close(self):
    process, self.process = self.process, None
    if process is not None:
      if process.poll() is None:
        process.kill()
      try:
        process.wait(timeout=.2)
      except subprocess.TimeoutExpired:
        # No new context is opened in this instance after any failure.
        pass
      process.stdin.close()
      process.stdout.close()
    self.views.clear()
    self.output = None
    if self.shared is not None:
      self.shared.close()
      self.shared = None
    if self.file is not None:
      self.file.close()
      Path(self.file.name).unlink(missing_ok=True)
      self.file = None

  def __del__(self):
    if getattr(self, 'process', None) is not None:
      self.close()
