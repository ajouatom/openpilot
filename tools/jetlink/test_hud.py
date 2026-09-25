import base64
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import hud_protocol as p
from hud_camera import preview_nv12, WIDTH, HEIGHT


def test_preview_preserves_full_camera_field_and_uv_order():
  width, height, stride = WIDTH * 2, HEIGHT * 2, WIDTH * 2 + 16
  y = np.arange(height * stride, dtype=np.uint8).reshape(height, stride)
  uv = np.zeros((height // 2, stride), np.uint8)
  uv[:, 0::2] = 71
  uv[:, 1::2] = 183
  frame = SimpleNamespace(width=width, height=height, stride=stride, uv_offset=height * stride,
                          data=y.tobytes() + uv.tobytes())
  out = np.frombuffer(preview_nv12(frame), np.uint8)
  np.testing.assert_array_equal(out[:WIDTH * HEIGHT].reshape(HEIGHT, WIDTH), y[::2, :width:2])
  assert np.all(out[WIDTH * HEIGHT::2] == 71)
  assert np.all(out[WIDTH * HEIGHT + 1::2] == 183)


def test_stale_and_oversized_display_data_are_not_shown(tmp_path, monkeypatch):
  path = tmp_path / 'hud'
  monkeypatch.setattr(p, 'HUD_PACKET', path)
  monkeypatch.setattr(p.time, 'monotonic', lambda: 10.)
  for data in (p.HEADER.pack(9.) + b'{"version":1}', b'x' * (p.MAX_HUD_BYTES + 20), b'bad'):
    path.write_bytes(data)
    monkeypatch.setattr(p, '_next_snapshot_read', 0.)
    assert p.read_snapshot() is None
  path.write_bytes(p.HEADER.pack(9.9) + json.dumps({'version':1, 'params':{}}).encode())
  monkeypatch.setattr(p, '_next_snapshot_read', 0.)
  assert p.read_snapshot()[1]['version'] == 1
