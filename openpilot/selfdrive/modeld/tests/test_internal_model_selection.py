import ast
import hashlib
from pathlib import Path

from openpilot.selfdrive.modeld import cheddar_model as client
from openpilot.selfdrive.modeld.cheddar_runtime import MODEL_SHA256


def test_internal_default_bytes_match_the_compiler_contract():
  path = Path(__file__).parents[1] / 'models/driving_supercombo.onnx'
  with path.open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == MODEL_SHA256


def test_model_factory_selects_cheddar_for_internal_and_keeps_legacy_external(monkeypatch):
  # Exercise the production factory without loading vehicle-only native modules.
  path = Path(__file__).parents[1] / 'modeld.py'
  cls = next(node for node in ast.parse(path.read_text(encoding='utf8')).body
             if isinstance(node, ast.ClassDef) and node.name == 'ModelState')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '__new__')
  minimal = ast.ClassDef(name='ModelState', bases=[], keywords=[], body=[method], decorator_list=[])
  scope = {'modeld_pkl_path': lambda external: Path('internal.pkl')}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[minimal], type_ignores=[])), str(path), 'exec'), scope)
  calls = []
  monkeypatch.setattr(client, 'CheddarModelState', lambda *args: calls.append(args) or 'internal')
  assert scope['ModelState'](1928, 1208, False) == 'internal'
  assert calls == [(1928, 1208, Path('internal.pkl'))]
  assert isinstance(scope['ModelState'].__new__(scope['ModelState'], 1928, 1208, True), scope['ModelState'])
