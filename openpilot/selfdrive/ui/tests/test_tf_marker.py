"""Exercise the production distance marker without a GPU or native UI imports."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('distance,expected', [(25.1, '25 m'), (35.4, '35 m'), (0., None)])
def test_marker_labels_the_published_target_in_metres(distance, expected):
  path = Path(__file__).resolve().parents[1] / 'onroad/model_renderer.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_draw_tf_marker_carrot')
  texts, lines = [], []
  namespace = {'rl': SimpleNamespace(Color=lambda *args: args),
               'draw_text_ui_style': lambda *args, **kwargs: texts.append(args[0])}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  renderer = SimpleNamespace(_carrot_tf_distance=distance, _carrot_tf_left=(10, 20), _carrot_tf_right=(30, 20),
                             _draw_line_segment_carrot=lambda *args: lines.append(args))
  namespace[method.name](renderer)
  assert texts == ([] if expected is None else [expected])
  assert len(lines) == (0 if expected is None else 1)
