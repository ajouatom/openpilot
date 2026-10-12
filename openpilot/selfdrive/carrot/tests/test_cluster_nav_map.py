import ast
from pathlib import Path
from types import SimpleNamespace as NS

from openpilot.selfdrive.carrot.cluster.cluster_config import normalize_cluster_screen_mode
from openpilot.selfdrive.navigation import preview


def test_usb_map_reuses_texture_and_hides_expired_image(monkeypatch, tmp_path):
  # Exercise the actual draw method without opening a physical Raylib/USB device.
  tree = ast.parse((Path(__file__).parents[1] / 'cluster/cluster_renderer.py').read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_draw_nav_model_map')
  now, fresh, calls = [10.], [True], []
  path = tmp_path / 'map.png'
  path.write_bytes(b'preview')
  monkeypatch.setattr(preview, 'PREVIEW_PATH', path)
  monkeypatch.setattr(preview, 'preview_fresh', lambda: fresh[0])
  def load(path):
    calls.append('load')
    return NS(id=1)
  destinations = []
  def draw_texture(*args):
    calls.append('draw')
    destinations.append(args[2])
  rl = NS(load_texture=load, unload_texture=lambda _: calls.append('unload'), draw_rectangle=lambda *args: None,
          draw_texture_pro=draw_texture, Rectangle=lambda *args: args,
          Vector2=lambda *args: args, WHITE=object(), BLACK=object())
  scope = dict(rl=rl, time=NS(monotonic=lambda: now[0]), DESIGN_HEIGHT=480,
               NAVI_LIVE_PANEL_X=1124, NAVI_LIVE_PANEL_W=792)
  exec(compile(ast.Module(body=[method], type_ignores=[]), '<USB nav map>', 'exec'), scope)
  renderer = NS(_nav_model_checked=0., _nav_model_fresh=False, _nav_model_mtime=0, _nav_model_texture=None,
                _draw_text=lambda *args: calls.append(args[0]))
  draw = scope['_draw_nav_model_map']
  draw(renderer)
  now[0] += .6
  draw(renderer)
  assert calls.count('load') == 1 and calls.count('draw') == 2
  x, y, w, h = destinations[0]
  assert 1124 <= x < x + w <= 1916 and 0 <= y < y + h < 480
  now[0] += .6
  fresh[0] = False
  draw(renderer)
  assert calls.count('draw') == 2 and calls[-1].startswith('NAV MAP: waiting')
  assert normalize_cluster_screen_mode(7) == normalize_cluster_screen_mode('nav-model') == 7
