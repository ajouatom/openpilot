from types import SimpleNamespace as NS

import pytest
from openpilot.common.params import Params
from openpilot.system.manager import process


@pytest.mark.parametrize('name', ['modeld', 'controlsd', 'selfdrived', 'radard', 'plannerd', 'card', 'pandad'])
def test_stale_auto_session_and_branch_flags_do_not_hold_process_start(tmp_path, monkeypatch, name):
  p = Params(str(tmp_path / 'params'))
  for key in ('AutoVehicleSession', 'AutoVehicleBranchPending', 'AutoVehicleSwitch'):
    p.put_bool(key, True)
  monkeypatch.setattr(process, 'flush_crash_reports', lambda: None)
  starts = []
  item = NS(
    name=name, enabled=True, proc=None, restart_if_crash=False, should_run=lambda *args: True, start=lambda: starts.append(name), stop=lambda **kw: None
  )
  process.ensure_running([item], True, p, NS())
  assert starts == [name]


def test_manager_init_applies_parked_choice_before_starting_processes(tmp_path, monkeypatch):
  from openpilot.common import vehicle_selection

  monkeypatch.setattr(vehicle_selection, '_boot_id', lambda: 'boot-a')
  import ast
  from pathlib import Path
  from openpilot.common.params import ParamKeyFlag
  from openpilot.common.vehicle_selection import (
    IONIQ5,
    PRESETS,
    PENDING,
    select_manual_vehicle,
    retire_automatic_switch,
    finish_pending_selection,
  )

  p = Params(str(tmp_path / 'params'))
  p.put_bool('IsOnroad', True)
  select_manual_vehicle(p, 1, parked=True)
  source = Path(__file__).resolve().parents[1] / 'manager.py'
  method = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'manager_init')
  end = next(
    i
    for i, n in enumerate(method.body)
    if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == 'finish_pending_selection'
  )
  method.body = method.body[: end + 1]
  namespace = {
    'save_bootlog': lambda: None,
    'get_build_metadata': lambda: NS(release_channel=False),
    'UpdateStatus': lambda _: None,
    'BASEDIR': str(tmp_path),
    'Params': lambda *args: p,
    'ParamKeyFlag': ParamKeyFlag,
    'retire_automatic_switch': retire_automatic_switch,
    'finish_pending_selection': finish_pending_selection,
  }
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
  namespace['manager_init']()
  assert not p.get('CarName')
  assert p.get(PENDING)
  monkeypatch.setattr(vehicle_selection, '_boot_id', lambda: 'boot-b')
  namespace['manager_init']()
  assert p.get('CarName') == IONIQ5
  assert {key: p.get_int(key) for key in PRESETS[IONIQ5]} == PRESETS[IONIQ5]
  assert not p.get(PENDING)
