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
