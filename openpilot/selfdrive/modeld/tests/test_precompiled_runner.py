import io
import json
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
                                           (TimeoutError('worker timeout'), True), (ValueError('invalid output'), True)])
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
  monkeypatch.setattr(precompiled_model, 'reject', rejections.append)
  frames = {key: SimpleNamespace(data=np.zeros(16, np.uint8)) for key in ('img', 'big_img')}
  transforms = {key: np.eye(3, dtype=np.float32) for key in frames}
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.zeros(2), 'action_t': np.zeros(2)}
  with pytest.raises(type(error)):
    model.run(frames, transforms, inputs, False)
  assert closed == [True]
  assert rejections == ([model.pkl_path] if rejected else [])
