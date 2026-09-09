import json

import numpy as np
import pytest

from openpilot.selfdrive.modeld.egpu_signal_cpu import MODEL_ID, configured, road_rgb


@pytest.mark.parametrize('y,u,v,channel', [(76, 84, 255, 0), (150, 44, 21, 1), (29, 255, 107, 2)])
def test_full_range_primary_colors_and_stride_padding(y, u, v, channel):
  stride, width, height = 528, 512, 256
  frame = np.full(stride*height*3//2, 255, dtype=np.uint8)
  frame[:stride*height].reshape(height, stride)[:, :width] = y
  uv = frame[stride*height:].reshape(height//2, stride)
  uv[:, :width:2], uv[:, 1:width:2] = u, v
  rgb = road_rgb(frame, width=width, height=height, stride=stride, uv_offset=stride*height, transform=np.eye(3))
  assert rgb.shape == (1, 3, 256, 512)
  assert rgb[0, channel].min() > .98
  assert np.delete(rgb, channel, axis=1).max() < .02


@pytest.mark.parametrize('matrix', [np.zeros((3, 3)), np.full((3, 3), np.nan), np.eye(2)])
def test_invalid_warp_is_rejected(matrix):
  with pytest.raises(ValueError):
    road_rgb(np.zeros(512*256*3//2, dtype=np.uint8), width=512, height=256,
             stride=512, uv_offset=512*256, transform=matrix)


def test_enablement_is_explicit_and_never_combines_gpu_observers(tmp_path, monkeypatch):
  monkeypatch.setenv('EGPU_YOLO_DIR', str(tmp_path))
  (tmp_path/'signal-v36-cpu-int8').mkdir()
  (tmp_path/'signal-v36-cpu-int8/model.onnx').touch()
  (tmp_path/'signal-v35-observation/ort-deps/onnxruntime').mkdir(parents=True)
  assert not configured()
  (tmp_path/'signal_cpu.json').write_text(json.dumps({'enabled': True, 'model_id': MODEL_ID}))
  assert configured()
  (tmp_path/'auto_enabled.json').write_text('{"enabled": true}')
  assert not configured()
  (tmp_path/'auto_enabled.json').write_text('{"enabled": false}')
  assert configured()
  (tmp_path/'qcom_enabled').touch()
  assert not configured()
