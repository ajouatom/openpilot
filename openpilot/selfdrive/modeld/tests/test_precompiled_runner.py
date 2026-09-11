import io
import json
from pathlib import Path
from types import SimpleNamespace

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
