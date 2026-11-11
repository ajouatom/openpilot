from types import SimpleNamespace

import pytest

from openpilot.common.auto_vehicle import WAIT_PROCESSES
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.system.manager import process


class Dummy:
  proc = None
  enabled = True
  restart_if_crash = False

  def __init__(self, name):
    self.name = name
    self.starts = 0

  def should_run(self, started, params, cp):
    return started

  def start(self):
    self.starts += 1

  def stop(self, **kw):
    pass


@pytest.mark.parametrize('name', sorted(WAIT_PROCESSES))
def test_settings_consumers_wait_but_identification_and_panda_start(tmp_path, monkeypatch, name):
  params = Params(str(tmp_path / 'params'))
  params.put_bool('AutoVehicleSession', True)
  monkeypatch.setattr(process, 'flush_crash_reports', lambda: None)
  procs = [Dummy(name), Dummy('card'), Dummy('pandad')]
  process.ensure_running(procs, True, params, SimpleNamespace())
  assert [p.starts for p in procs] == [0, 1, 1]
  params.put_bool('AutoVehicleReady', True)
  process.ensure_running(procs, True, params, SimpleNamespace())
  assert [p.starts for p in procs] == [1, 2, 2]
  # Turning OFF mid-route does not restart/stop processes or change the latch.
  params.put_bool('AutoVehicleSwitch', False)
  assert params.get_bool('AutoVehicleSession')
  process.ensure_running(procs, True, params, SimpleNamespace())
  assert procs[0].starts == 2


def test_toggle_on_mid_drive_has_no_effect_until_next_session(tmp_path, monkeypatch):
  params = Params(str(tmp_path / 'params'))
  monkeypatch.setattr(process, 'flush_crash_reports', lambda: None)
  params.put_bool('AutoVehicleSwitch', True)
  proc = Dummy('controlsd')
  process.ensure_running([proc], True, params, SimpleNamespace())
  assert proc.starts == 1


def test_ready_and_session_are_not_reused_on_a_new_route(tmp_path):
  params = Params(str(tmp_path / 'params'))
  params.put_bool('AutoVehicleSwitch', True)
  params.put_bool('AutoVehicleSession', True)
  params.put_bool('AutoVehicleReady', True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  assert not params.get_bool('AutoVehicleReady')
  assert not params.get_bool('AutoVehicleSession')
  assert params.get_bool('AutoVehicleSwitch')
