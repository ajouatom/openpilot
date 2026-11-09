import time
from types import SimpleNamespace

import pytest

from openpilot.common import auto_vehicle_branch as avb
from openpilot.common.auto_vehicle import IONIQ5
from openpilot.system.manager import auto_vehicle_branch as manager_branch, process
from openpilot.common.tests.test_auto_vehicle_branch import repo as repo, params as params


def request(params, parked=True):
  now = time.monotonic()
  params.put('AutoVehicleBranchRequest', {'boot': avb.boot_id(), 'vehicle': IONIQ5, 'time': now, 'parked': parked})
  return now


def test_worker_needs_nooutput_and_stopped_consumers_before_any_git(repo, params):
  params.put_bool('AutoVehicleBranchPending', True)
  worker = manager_branch.AutoVehicleBranchManager(params, repo.root)
  worker.update(request(params), no_output=False, workers_stopped=True)
  assert worker.future is None
  worker.update(request(params), no_output=True, workers_stopped=False)
  assert worker.future is None
  assert repo.branch() == 'c3-pht-porter'
  worker.update(request(params), no_output=True, workers_stopped=True)
  for _ in range(200):
    if worker.future.done():
      break
    request(params)
    time.sleep(0.005)
  assert worker.future.done()
  assert repo.branch() == 'c3-pht'
  worker.update(request(params, parked=False), no_output=True, workers_stopped=True)
  assert not params.get_bool('DoReboot')
  worker.update(request(params), no_output=True, workers_stopped=True)
  assert params.get_bool('DoReboot')


@pytest.mark.parametrize('saved_state', ['prepared', 'checked_out'])
def test_manager_restart_after_checkout_recovers_without_another_checkout(repo, params, saved_state):
  avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  record = params.get('AutoVehicleBranchHandoff')
  record['state'] = saved_state
  params.put('AutoVehicleBranchHandoff', record)
  params.put_bool('AutoVehicleBranchPending', True)
  worker = manager_branch.AutoVehicleBranchManager(params, repo.root)
  worker.update(request(params), no_output=False, workers_stopped=True)
  assert not params.get_bool('DoReboot')
  worker.update(request(params), no_output=True, workers_stopped=True)
  assert params.get_bool('DoReboot')
  assert worker.future is None


def test_failed_worker_holds_control_and_latches_error_without_retry(repo, params):
  (repo.root / 'version').write_text('keep local edits')
  params.put_bool('AutoVehicleBranchPending', True)
  worker = manager_branch.AutoVehicleBranchManager(params, repo.root)
  worker.update(request(params), no_output=True, workers_stopped=True)
  for _ in range(200):
    if worker.future.done():
      break
    time.sleep(0.005)
  worker.update(request(params), no_output=True, workers_stopped=True)
  assert worker.failed
  assert not params.get_bool('DoReboot')
  assert params.get('AutoVehicleBranchStatus').startswith('blocked:')
  future = worker.future
  worker.update(request(params), no_output=True, workers_stopped=True)
  assert worker.future is future
  assert repo.branch() == 'c3-pht-porter'


def test_pending_overrides_ready_flag_and_holds_all_checkout_readers(params, monkeypatch):
  class Dummy:
    proc = None
    enabled = True
    restart_if_crash = False

    def __init__(self, name):
      self.name, self.starts = name, 0

    def should_run(self, *args):
      return True

    def start(self):
      self.starts += 1

    def stop(self, **kwargs):
      pass

  params.put_bool('AutoVehicleReady', True)
  params.put_bool('AutoVehicleBranchPending', True)
  monkeypatch.setattr(process, 'flush_crash_reports', lambda: None)
  procs = [Dummy(n) for n in ('card', 'pandad', 'ui', 'controlsd', 'modeld', 'updated', 'carrot_server', 'calibrationd')]
  process.ensure_running(procs, True, params, SimpleNamespace())
  assert [p.starts for p in procs] == [1, 1, 1, 0, 0, 0, 0, 0]


@pytest.mark.parametrize(
  'mode,controls,panda_type,fresh,expected',
  [
    ('noOutput', False, 'tres', True, True),
    ('noOutput', True, 'tres', True, False),
    ('hyundaiCanfd', False, 'tres', True, False),
    ('elm327', False, 'tres', True, False),
    ('noOutput', False, 'unknown', True, False),
    ('noOutput', False, 'tres', False, False),
  ],
)
def test_real_panda_schema_permission_check(mode, controls, panda_type, fresh, expected):
  from openpilot.cereal import log

  ps = log.PandaState.new_message(safetyModel=mode, controlsAllowed=controls, pandaType=panda_type)

  class SM(dict):
    def all_checks(self, services):
      assert services == ['deviceState', 'pandaStates']
      return fresh

  sm = SM(pandaStates=[ps])
  assert manager_branch.pandas_no_output(sm) == expected
  assert not manager_branch.pandas_no_output(SM(pandaStates=[]))
