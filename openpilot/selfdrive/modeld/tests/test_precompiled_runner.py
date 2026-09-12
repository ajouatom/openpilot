import io
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld import precompiled_runner as runner


def test_precompiled_state_exposes_camera_inputs_required_by_modeld(tmp_path, monkeypatch):
  info = {'size': 48, 'input_bytes': 32, 'output_count': 4,
          'layout': {'img': {'offset': 0, 'shape': [16], 'dtype': 'uint8'},
                     'big_img': {'offset': 16, 'shape': [16], 'dtype': 'uint8'}},
          'output_slices': {'hidden_state': [0, 4, None]},
          'input_shapes': {'img': [1, 12, 128, 256], 'big_img': [1, 12, 128, 256], 'desire_pulse': [1, 25, 8]},
          'frame_size': 16, 'checkpoint': 'test-checkpoint'}
  def start(command, **kwargs):
    Path(command[3]).write_bytes(bytes(info['size']))
    return SimpleNamespace(stdin=io.BytesIO(), stdout=io.BytesIO(), poll=lambda: 0)
  monkeypatch.setattr(runner.subprocess, 'Popen', start)
  monkeypatch.setattr(runner.PrecompiledModelState, '_receive', lambda self, timeout: json.dumps(info).encode())
  model = runner.PrecompiledModelState(1344, 760, tmp_path / 'model.pkl')
  try:
    # modeld uses this contract to select camera buffers before calling run().
    buffers = {'img': object(), 'big_img': object()}
    assert {name: buffers[name] for name in model.vision_input_names} == buffers
    assert model.input_shapes == info['input_shapes']
    assert model.checkpoint == info['checkpoint']
  finally:
    model.close()


@pytest.mark.parametrize('error,rejected', [(KeyboardInterrupt(), False), (SystemExit(), False),
                                           (TimeoutError('worker timeout'), False), (ValueError('invalid output'), True),
                                           (RuntimeError('PCIe link not up (LTSSM=0x00)'), False),
                                           (BrokenPipeError(), False), (RuntimeError('precompiled eGPU worker exited (-9)'), False)])
def test_inference_shutdown_releases_worker_without_rejecting_artifact(monkeypatch, tmp_path, error, rejected):
  model = object.__new__(runner.PrecompiledModelState)
  model.pkl_path = tmp_path / 'model.pkl'
  model.process = SimpleNamespace(stdin=io.BytesIO())
  model.first_run = False
  model.frame_size = 16
  model.prev_desire = np.zeros(8, np.float32)
  model.views = {key: np.zeros(shape, np.float32) for key, shape in
                 {'img': 16, 'big_img': 16, 'desire': 8, 'traffic_convention': 2, 'action_t': 2,
                  'tfm': (3, 3), 'big_tfm': (3, 3)}.items()}
  closed, rejections = [], []
  def close(self):
    closed.append(True)
    self.process = None
  def receive(self, timeout):
    raise error
  monkeypatch.setattr(runner.PrecompiledModelState, 'close', close)
  monkeypatch.setattr(runner.PrecompiledModelState, '_receive', receive)
  from openpilot.selfdrive.modeld import precompiled_model
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  monkeypatch.setattr(precompiled_model, 'reject', rejections.append)
  frames = {key: SimpleNamespace(data=np.zeros(16, np.uint8)) for key in ('img', 'big_img')}
  transforms = {key: np.eye(3, dtype=np.float32) for key in frames}
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.zeros(2), 'action_t': np.zeros(2)}
  with pytest.raises(type(error)):
    model.run(frames, transforms, inputs, False)
  assert closed == [True]
  assert rejections == ([model.pkl_path] if rejected else [])
  if isinstance(error, Exception):
    failure = json.loads((tmp_path / 'last_failure.json').read_text())
    assert failure['phase'] == 'inference' and failure['rejected'] == rejected
  else:
    assert not (tmp_path / 'last_failure.json').exists()


@pytest.mark.parametrize('message,rejected', [('RuntimeError: PCIe link not up (LTSSM=0x00)', False),
                                             ('ValueError: precompiled checkpoint mismatch', True)])
def test_worker_error_protocol_preserves_cause_and_artifact_decision(tmp_path, monkeypatch, message, rejected):
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  payload = b'ERROR ' + json.dumps(message).encode() + b'\n'
  process = SimpleNamespace(stdin=io.BytesIO(), stdout=io.BytesIO(payload), poll=lambda: 1)
  monkeypatch.setattr(runner.subprocess, 'Popen', lambda *a, **kw: process)
  class ReadySelector:
    def __enter__(self):
      return self
    def __exit__(self, *args):
      pass
    def register(self, *args):
      pass
    def select(self, timeout):
      return [True]
  monkeypatch.setattr(runner.selectors, 'DefaultSelector', ReadySelector)
  with pytest.raises(RuntimeError, match=message.split(':')[0]):
    runner.PrecompiledModelState(1928, 1208, tmp_path / 'model.pkl')
  failure = json.loads((tmp_path / 'last_failure.json').read_text())
  assert failure['error'] == message and failure['phase'] == 'load'
  assert failure['rejected'] == rejected
  assert (tmp_path / 'rejected').exists() == rejected
  assert process.stdin.closed and process.stdout.closed


def test_worker_reports_real_checksum_failure_before_gpu_access(tmp_path):
  pkl = tmp_path / 'model.pkl'
  pkl.write_bytes(b'corrupt model')
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  worker = Path(runner.__file__).with_name('precompiled_worker.py')
  result = subprocess.run([sys.executable, str(worker), str(pkl), str(tmp_path / 'shared'), '1928', '1208'],
                          capture_output=True, timeout=10)
  assert result.returncode != 0
  assert result.stdout.startswith(b'ERROR ')
  assert 'precompiled PKL checksum mismatch' in json.loads(result.stdout[6:])
