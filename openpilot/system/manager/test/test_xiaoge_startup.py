import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.system.manager.xiaoge_startup import XiaogeStartupGate


class Messages(dict):
  def __init__(self):
    super().__init__(deviceState=SimpleNamespace(started=True),
                     selfdriveState=SimpleNamespace(engageable=True, enabled=False),
                     modelV2=SimpleNamespace(), carState=SimpleNamespace(canValid=True, canTimeout=False),
                     pandaStates=[SimpleNamespace(safetyModel='hyundai', safetyParam=13)])
    self.logMonoTime = dict.fromkeys(XiaogeStartupGate.SERVICES, 0)
    self.bad = set()

  def all_checks(self, services):
    return not self.bad.intersection(services)

  def tick(self, now):
    self.logMonoTime = dict.fromkeys(self.logMonoTime, int(now * 1e9))


@pytest.fixture
def setup():
  gate, sm = XiaogeStartupGate(), Messages()
  cp = SimpleNamespace(safetyConfigs=[SimpleNamespace(safetyModel='hyundai', safetyParam=13)])
  sm.tick(-1)
  assert not gate.update(True, sm, cp, True, -1)
  return gate, sm, cp


def update(setup, now, *, started=True, controls=True):
  gate, sm, cp = setup
  sm.tick(now)
  return gate.update(started, sm, cp, controls, now)


def test_timeout_and_controls_ready_do_not_bypass_model_or_can_readiness(setup):
  gate, sm, _ = setup
  sm.bad.add('modelV2')
  for t in (0, 6.01, 7, 8, 9):
    assert not update(setup, t)
  sm.bad.clear()
  sm['carState'].canTimeout = True
  assert not update(setup, 9.5)
  sm['carState'].canTimeout = False
  assert not update(setup, 10)
  assert not update(setup, 10.49)
  assert update(setup, 10.5)
  assert gate.ready
  assert not sm['selfdriveState'].enabled  # No actual engagement is needed.


@pytest.mark.parametrize('service', XiaogeStartupGate.SERVICES)
def test_unhealthy_or_missing_service_resets_settling(setup, service):
  _, sm, _ = setup
  assert not update(setup, 0)
  sm.bad.add(service)
  assert not update(setup, 0.4)
  sm.bad.clear()
  assert not update(setup, 0.5)
  assert update(setup, 1)


@pytest.mark.parametrize('service', XiaogeStartupGate.SERVICES)
def test_previous_session_messages_cannot_release_startup(setup, service):
  gate, sm, cp = setup
  assert not gate.update(False, sm, cp, True, 9)
  sm.tick(9)
  assert not gate.update(True, sm, cp, True, 9)
  sm.tick(10)
  sm.logMonoTime[service] = int(9e9)
  assert not gate.update(True, sm, cp, True, 10)
  assert not gate.update(True, sm, cp, True, 10.5)
  assert not update(setup, 11)
  assert update(setup, 11.5)


@pytest.mark.parametrize('case', ['controls', 'engageable', 'can_valid', 'safety_model', 'safety_param', 'missing_panda', 'missing_config', 'extra_elm'])
def test_incomplete_initialization_is_not_ready(setup, case):
  _, sm, cp = setup
  if case == 'engageable':
    sm['selfdriveState'].engageable = False
  if case == 'can_valid':
    sm['carState'].canValid = False
  if case == 'safety_model':
    sm['pandaStates'][0].safetyModel = 'elm327'
  if case == 'safety_param':
    sm['pandaStates'][0].safetyParam = 0
  if case == 'missing_panda':
    sm['pandaStates'] = []
  if case == 'missing_config':
    cp.safetyConfigs = []
  if case == 'extra_elm':
    sm['pandaStates'].append(SimpleNamespace(safetyModel='elm327'))
  for t in (0, 0.5, 1):
    assert not update(setup, t, controls=case != 'controls')


def test_latch_avoids_restart_on_later_fault_but_resets_offroad(setup):
  _, sm, _ = setup
  assert not update(setup, 0)
  assert update(setup, 0.5)
  sm.bad.add('modelV2')
  sm['selfdriveState'].engageable = False
  assert update(setup, 1)
  assert not update(setup, 2, started=False)
  assert not update(setup, 3)
  sm.bad.clear()
  sm['selfdriveState'].engageable = True
  assert not update(setup, 3.5)
  assert update(setup, 4)


def test_missing_observations_do_not_count_as_settling(setup):
  assert not update(setup, 0)
  assert not update(setup, 5)
  assert update(setup, 5.5)


def test_manager_restart_has_no_persistent_ready_latch(setup):
  _, sm, cp = setup
  assert not update(setup, 0)
  assert update(setup, 0.5)
  gate = XiaogeStartupGate()
  assert not gate.update(True, sm, cp, True, 5)


def test_real_cereal_safety_configs_and_extra_silent_panda():
  cp = car.CarParams.new_message(safetyConfigs=[{'safetyModel': 'hyundai', 'safetyParam': 13}])
  ps = log.Event.new_message()
  ps.init('pandaStates', 2)
  ps.pandaStates[0].safetyModel = 'hyundai'
  ps.pandaStates[0].safetyParam = 13
  ps.pandaStates[1].safetyModel = 'silent'
  sm = Messages()
  sm['pandaStates'] = ps.pandaStates
  setup = XiaogeStartupGate(), sm, cp
  assert not update(setup, -1)
  assert not update(setup, 0)
  assert update(setup, 0.5)


def test_publisher_clock_offsets_do_not_release_old_messages(setup):
  gate, sm, cp = setup
  assert not gate.update(False, sm, cp, True, 1)
  sm.logMonoTime = {s: int((100 + i) * 1e9) for i, s in enumerate(gate.SERVICES)}
  assert not gate.update(True, sm, cp, True, 2)
  assert not gate.update(True, sm, cp, True, 3)
  for s in gate.SERVICES:
    sm.logMonoTime[s] += int(0.5e9)
  assert not gate.update(True, sm, cp, True, 3.5)
  for s in gate.SERVICES:
    sm.logMonoTime[s] += int(0.5e9)
  assert gate.update(True, sm, cp, True, 4)


def test_process_selection_retains_live_sharedata_and_excludes_offroad():
  # Load the production predicate without importing Windows-unavailable Params.
  path = Path(__file__).parents[1] / 'process_config.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  predicate = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'enable_xiaoge_data')
  namespace = {'car': car}
  exec(compile(ast.Module(body=[predicate], type_ignores=[]), str(path), 'exec'), namespace)
  check = namespace['enable_xiaoge_data']
  cp = car.CarParams.new_message()
  for enabled in (False, True):
    params = SimpleNamespace(get_bool=lambda key, enabled=enabled: enabled)
    assert not check(False, params, cp)
    assert check(True, params, cp) == enabled
