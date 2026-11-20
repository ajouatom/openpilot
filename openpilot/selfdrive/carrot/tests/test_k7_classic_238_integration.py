# K7 0x238 radar display and settings coexist with the shared radar and UI processing.
import ast
import json
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]


def test_overlay_keeps_custom_tracks_with_upstream_render_diagnostics():
  tree = ast.parse((ROOT / 'openpilot/selfdrive/ui/onroad/model_renderer.py').read_text(encoding='utf-8'))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ModelRenderer')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '_draw_carrot_overlays')
  native = SimpleNamespace(active=lambda: False)
  namespace = {'native_draw': native, 'native_geometry': native, 'native_text': native}
  exec(compile(ast.Module(body=[method], type_ignores=[]), '<merged-overlay>', 'exec'), namespace)
  calls = []
  renderer = SimpleNamespace(_render_diagnostics=SimpleNamespace(
    start=lambda: calls.append('start'), finish=lambda: calls.append('finish'),
    call=lambda name, callback, *args: callback(*args), values={}))
  for name in ('path', 'lane_lines', 'blind_spot', 'radar_info', 'classic_radar_tracks'):
    setattr(renderer, '_draw_' + name + '_carrot', lambda _, name=name: calls.append(name))
  namespace['_draw_carrot_overlays'](renderer, object())
  assert calls == ['start', 'path', 'lane_lines', 'blind_spot', 'radar_info', 'classic_radar_tracks', 'finish']


def test_ui_subscribes_to_custom_and_upstream_radar_streams():
  tree = ast.parse((ROOT / 'openpilot/selfdrive/ui/ui_state.py').read_text(encoding='utf-8'))
  subscriptions = next(node.args[0] for node in ast.walk(tree)
                       if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                       and node.func.attr == 'SubMaster')
  names = ast.literal_eval(subscriptions)
  assert 'classicRadarTracks' in names
  assert 'liveTracks' in names
  assert len(names) == len(set(names))


def test_radar_reset_rebuilds_upstream_state_and_custom_display_history():
  tree = ast.parse((ROOT / 'openpilot/selfdrive/carrot/radar/radarcan.py').read_text(encoding='utf-8'))
  reset = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
               and isinstance(node.test, ast.Name) and node.test.id == 'needs_reset')
  calls = [node.func.id for node in ast.walk(reset)
           if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)]
  assert '_rebuild_radar_interface' in calls
  assert 'Classic238DisplayTracker' in calls


def test_settings_preserve_k7_modes_and_updated_scc_exclusion():
  data = json.loads((ROOT / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8'))

  def settings(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from settings(child)
    elif isinstance(value, list):
      for child in value:
        yield from settings(child)

  radar = next(item for item in settings(data) if item.get('name') == 'EnableRadarTracks')
  assert (radar['min'], radar['max'], radar['default']) == (-2, 3, 0)
  assert 'K7 2020' in radar['descr'] and 'K7 2020' in radar['edescr']
  assert '0x238' in radar['descr'] and '0x238' in radar['edescr']
  assert '옆 차로' in radar['descr'] and 'another lane' in radar['edescr']
