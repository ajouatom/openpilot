import base64
import hashlib
import io
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import ast

from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import hud_nav_map as n
import hud_protocol as p


def png(size=(256, 256)):
  output = io.BytesIO()
  Image.new('L', size, 100).save(output, format='PNG')
  return output.getvalue()


def record(data=None, age=.1):
  data = png() if data is None else data
  return {'params': {'ClusterHudScreenMode': 'Nw=='},
          'nav_map_preview': {'age': age, 'sha256': hashlib.sha256(data).hexdigest(),
                              'png': base64.b64encode(data).decode()}}


def test_sender_source_expiry_and_mode_switch(tmp_path, monkeypatch):
  path = tmp_path / 'source.png'
  path.write_bytes(png())
  os.utime(path, (1000, 1000))
  clock = [10.]
  monkeypatch.setattr(n.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(n.time, 'time', lambda: 1000. + clock[0] - 10.)
  sender = n.NavPreviewPublisher(path)
  first = sender.packet(True)
  assert first['age'] == 0.
  clock[0] += .2
  assert sender.packet(True)['age'] == pytest.approx(.2)
  clock[0] += 2.
  assert sender.packet(True) is None  # Re-reading cannot renew a stopped source.
  assert sender.packet(False) is None and sender.cached is None
  path.write_bytes(b'x' * (n.MAX_BYTES + 1))
  os.utime(path, (1002, 1002))
  assert sender.packet(True) is None


@pytest.mark.parametrize('age', [-1, 2, float('nan'), float('inf')])
def test_receiver_rejects_invalid_age(tmp_path, monkeypatch, age):
  monkeypatch.setattr(n.time, 'monotonic', lambda: 10.)
  assert not n.RemoteNavPreview(lambda: (9.9, record(age=age)), tmp_path / 'map.png').fresh()


@pytest.mark.parametrize('data', [b'bad', png((257, 256)), png()[:30], b'x' * (n.MAX_BYTES + 1)],
                         ids=['not-png', 'wrong-dimensions', 'truncated', 'oversized'])
def test_receiver_rejects_malformed_or_unbounded_image(tmp_path, monkeypatch, data):
  monkeypatch.setattr(n.time, 'monotonic', lambda: 10.)
  path = tmp_path / 'map.png'
  assert not n.RemoteNavPreview(lambda: (9.9, record(data)), path).fresh()
  assert not path.exists()


def test_receiver_transport_age_identity_and_disconnect(tmp_path, monkeypatch):
  clock = [10.]
  value = [record(age=1.7)]
  monkeypatch.setattr(n.time, 'monotonic', lambda: clock[0])
  receiver = n.RemoteNavPreview(lambda: (9.9, value[0]) if value[0] else None, tmp_path / 'map.png')
  assert receiver.fresh()
  mtime = receiver.path.stat().st_mtime_ns
  clock[0] = 10.1
  assert receiver.fresh() and receiver.path.stat().st_mtime_ns == mtime
  clock[0] = 10.21
  assert not receiver.fresh()  # Sender age plus transport/local time, independent of wall clock.
  clock[0] = 10.
  value[0]['nav_map_preview']['sha256'] = '0' * 64
  assert not receiver.fresh()
  value[0] = record()
  value[0]['params']['ClusterHudScreenMode'] = 'MA=='
  assert not receiver.fresh()
  value[0] = None
  assert not receiver.fresh() and receiver.status == {'fresh': False}


def test_real_renderer_receives_remote_png_and_hides_lost_stream(tmp_path, monkeypatch):
  from openpilot.selfdrive.navigation import preview
  clock = [10.]
  value = [record()]
  monkeypatch.setattr(n.time, 'monotonic', lambda: clock[0])
  receiver = n.RemoteNavPreview(lambda: (clock[0] - .1, value[0]) if value[0] else None, tmp_path / 'map.png')
  monkeypatch.setattr(preview, 'PREVIEW_PATH', receiver.path)
  monkeypatch.setattr(preview, 'preview_fresh', receiver.fresh)
  tree = ast.parse((Path(__file__).parents[2] / 'openpilot/selfdrive/carrot/cluster/cluster_renderer.py').read_text(encoding='utf-8'))
  method = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == '_draw_nav_model_map')
  calls = []
  def load(path):
    with Image.open(path) as image:
      assert image.size == (256, 256) and image.getpixel((0, 0)) == 100
    calls.append('load')
    return NS(id=1)
  rl = NS(load_texture=load, unload_texture=lambda _: calls.append('unload'), draw_rectangle=lambda *a: None,
          draw_texture_pro=lambda *a: calls.append('draw'), Rectangle=lambda *a: a,
          Vector2=lambda *a: a, WHITE=object(), BLACK=object())
  scope = dict(rl=rl, time=n.time, DESIGN_HEIGHT=480, NAVI_LIVE_PANEL_X=1124, NAVI_LIVE_PANEL_W=792)
  exec(compile(ast.Module(body=[method], type_ignores=[]), '<remote NAV preview>', 'exec'), scope)
  renderer = NS(_nav_model_checked=0., _nav_model_fresh=False, _nav_model_mtime=0, _nav_model_texture=None,
                _draw_text=lambda *a: calls.append(a[0]))
  draw = scope['_draw_nav_model_map']
  draw(renderer)
  clock[0] += .6
  draw(renderer)
  assert calls.count('load') == 1 and calls.count('draw') == 2
  value[0] = None
  clock[0] += .6
  draw(renderer)
  assert calls.count('draw') == 2 and calls[-1].startswith('NAV MAP: waiting')


def test_optional_preview_cannot_expand_snapshot_budget():
  value = record()
  value['events'] = {'modelV2': 'x' * 70000, 'navRoute': 'route'}
  value['nav_map_preview']['png'] = 'x' * 44000
  value['cameras'] = {'road': 'x' * 40000}
  result = json.loads(p.encode_snapshot(value))
  assert len(p.encode_snapshot(result)) <= 96 * 1024
  assert 'nav_map_preview' not in result and result['events']['navRoute'] == 'route'
  assert result['cameras'] == {}


def test_only_wifi_explicitly_enables_preview(monkeypatch):
  commands = []
  monkeypatch.setattr(p.subprocess, 'Popen', lambda command, **kwargs: commands.append(command))
  p.Publisher()
  assert '--nav-map' not in commands[-1]
  p.Publisher(nav_map=True)
  assert '--nav-map' in commands[-1]
  source = (Path(__file__).parent / 'hud_wifi_worker.py').read_text()
  assert 'Publisher(navi=True, nav_map=True)' in source
