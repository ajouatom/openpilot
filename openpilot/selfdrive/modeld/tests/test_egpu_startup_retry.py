"""Exercise the production loader with failed and recovering device constructors."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld import precompiled_runner
from openpilot.selfdrive.modeld.helpers import usbgpu_pcie_not_ready


@pytest.mark.parametrize('precompiled,failures,expected_calls', [(True, 10, 2), (True, 1, 2), (False, 10, 6)])
@pytest.mark.parametrize('message', ['PCIe link not up (LTSSM=0x00)', 'bulk OUT 0x02 failed: Input/Output Error'])
def test_link_retry_stops_then_preserves_internal_fallback(monkeypatch, tmp_path, precompiled, failures, expected_calls, message):
  source = Path(__file__).parents[1] / 'modeld.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  constants = [node for node in tree.body if isinstance(node, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id.startswith('USBGPU_') for t in node.targets)]
  loader = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'load_usbgpu_model')
  # The enclosing modeld loop needs device services. Run its unmodified loader body
  # with closure storage represented by globals, plus the actual attempt selection.
  loader.body = [ast.copy_location(ast.Global(names=n.names), n) if isinstance(n, ast.Nonlocal) else n for n in loader.body]
  selection = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'init_attempts' for t in n.targets))
  calls, queued, sleeps, refreshes = [], [], [], []
  sentinel = object()
  def construct(*args):
    calls.append(args)
    if len(calls) <= failures:
      raise RuntimeError(message)
    return sentinel
  monkeypatch.setattr(precompiled_runner, 'PrecompiledModelState', construct)
  namespace = {
    'precompiled': precompiled, 'usbgpu_model': None, 'usbgpu_pkl_path': tmp_path / 'model.pkl',
    'vipc_client_main': SimpleNamespace(width=1928, height=1208), 'ModelState': construct,
    'usbgpu_pcie_not_ready': usbgpu_pcie_not_ready, 'time': SimpleNamespace(sleep=sleeps.append),
    'refresh_usbgpu_device_cache': lambda: refreshes.append(True),
    'cloudlog': SimpleNamespace(warning=lambda *a: None, exception=lambda *a: None),
    'queue_usbgpu_error_tmux': lambda *a: queued.append(a), 'params': object(),
  }
  exec(compile(ast.fix_missing_locations(ast.Module(body=[*constants, selection, loader], type_ignores=[])), str(source), 'exec'), namespace)
  namespace['load_usbgpu_model']()
  assert len(calls) == expected_calls
  assert len(sleeps) == len(refreshes) == expected_calls - 1
  if failures < expected_calls:
    assert namespace['usbgpu_model'] is sentinel and not queued
  else:
    assert namespace['usbgpu_model'] is None and len(queued) == 1
