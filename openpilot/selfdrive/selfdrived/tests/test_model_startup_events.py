"""Exercise startup event selection and state transitions without device drivers."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log

ROOT = Path(__file__).resolve().parents[1]
EventName = log.OnroadEvent.EventName
State = log.SelfdriveState.OpenpilotState


def load_policy():
  tree = ast.parse((ROOT / 'selfdrived.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_events')
  start = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Assign) and
               any(isinstance(t, ast.Name) and t.id == 'big_model_settling' for t in n.targets))
  end = next(i for i, n in enumerate(method.body) if isinstance(n, ast.If) and
             ast.unparse(n.test) == "not self.sm.valid['pandaStates']")
  pose = next(n for n in method.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'not self.CP.notCar')
  wrapper = ast.parse('def check(self): pass')
  wrapper.body[0].body = method.body[start:end] + [pose]
  ns = {'EventName': EventName, 'log': log, 'cal_status': log.LiveCalibrationData.Status.calibrated,
        'TESTING_CLOSET': False, 'SIMULATION': False, 'REPLAY': False}
  exec(compile(ast.fix_missing_locations(wrapper), '<startup event policy>', 'exec'), ns)
  return ns['check']


def event_types():
  tree = ast.parse((ROOT / 'events.py').read_text(encoding='utf8'))
  assignment = next(n for n in tree.body if isinstance(n, (ast.Assign, ast.AnnAssign)) and
                    any(isinstance(t, ast.Name) and t.id == 'EVENTS'
                        for t in (n.targets if isinstance(n, ast.Assign) else [n.target])))
  return {getattr(EventName, key.attr): {t.attr for t in value.keys}
          for key, value in zip(assignment.value.keys, assignment.value.values, strict=True)}


class Events(set):
  types = event_types()

  @property
  def events(self):
    return list(self)

  @property
  def names(self):
    return list(self)

  def contains(self, kind):
    return any(kind in self.types[e] for e in self)


def state_machine():
  tree = ast.parse((ROOT / 'state.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StateMachine')
  active = (State.enabled, State.softDisabling, State.overriding)
  ns = {'Events': Events, 'State': State, 'DT_CTRL': 0.01, 'SOFT_DISABLE_TIME': 3,
        'ACTIVE_STATES': active, 'ENABLED_STATES': (State.preEnabled, *active),
        'ET': SimpleNamespace(**{key: key for kinds in Events.types.values() for key in kinds})}
  exec(compile(ast.Module(body=[cls], type_ignores=[]), '<state machine>', 'exec'), ns)
  return ns['StateMachine']()


class Messages(dict):
  def __init__(self, *, valid, healthy):
    super().__init__(radarState=SimpleNamespace(radarErrors=SimpleNamespace(
      canError=False, radarUnavailableTemporary=False, radarFault=False, wrongConfig=False)),
      livePose=SimpleNamespace(inputsOK=True, sensorsOK=True, posenetOK=True),
      liveParameters=SimpleNamespace(valid=True))
    self.valid = {'radarState': valid}
    self.seen = {'livePose': True, 'liveParameters': True}
    self.healthy = healthy

  def all_checks(self, services=None):
    return self.healthy


def drive(*, valid=False, healthy=False, settling=True, enabled=False):
  return SimpleNamespace(sm=Messages(valid=valid, healthy=healthy), events=Events(), enabled=enabled, model_startup_complete=False,
                         CP=SimpleNamespace(notCar=False), _big_model_settling=lambda: settling)


def test_cold_model_startup_blocks_enable_until_downstream_readiness():
  check = load_policy()
  state = drive()
  state.sm['livePose'].inputsOK = False
  machine = state_machine()
  for seconds in (6, 29.6, 30.0):
    state.events.clear()
    check(state)
    assert state.events == {EventName.selfdriveInitializing}, seconds
    state.events.add(EventName.buttonEnable)
    assert machine.update(state.events) == (False, False)
  # Radar recovers before pose, which must still block engagement.
  state.sm.valid['radarState'] = True
  state.events.clear()
  check(state)
  assert EventName.selfdriveInitializing in state.events
  state.events.add(EventName.buttonEnable)
  assert machine.update(state.events) == (False, False)
  state.sm.healthy = state.sm['livePose'].inputsOK = True
  state.events.clear()
  check(state)
  assert not state.events
  state.events.add(EventName.buttonEnable)
  assert machine.update(state.events) == (True, True)
  # A later dropout after readiness must not return to the startup exemption,
  # even if the five-second settling interval has not elapsed yet.
  state.sm.healthy = state.sm.valid['radarState'] = False
  state.events.clear()
  check(state)
  assert state.events == {EventName.commIssue}


@pytest.mark.parametrize('fault,event', [('canError', EventName.canError),
  ('radarUnavailableTemporary', EventName.radarTempUnavailable),
  ('radarFault', EventName.radarFault), ('wrongConfig', EventName.radarFault)])
@pytest.mark.parametrize('settling,enabled', [(True, False), (True, True), (False, False), (False, True)])
def test_real_radar_faults_keep_no_entry_and_disable(fault, event, settling, enabled):
  state = drive(settling=settling, enabled=enabled)
  setattr(state.sm['radarState'].radarErrors, fault, True)
  load_policy()(state)
  assert event in state.events
  assert EventName.selfdriveInitializing not in state.events
  assert state.events.contains('NO_ENTRY')
  assert state.events.contains('IMMEDIATE_DISABLE' if fault == 'canError' else 'SOFT_DISABLE')
  machine = state_machine()
  machine.state = State.enabled
  machine.update(state.events)
  assert machine.state == (State.disabled if fault == 'canError' else State.softDisabling)


@pytest.mark.parametrize('settling,enabled', [(False, False), (False, True), (True, True)])
def test_unexplained_invalid_radar_is_communication_error_outside_disabled_startup(settling, enabled):
  state = drive(settling=settling, enabled=enabled)
  load_policy()(state)
  assert state.events == {EventName.commIssue}
  machine = state_machine()
  machine.state = State.enabled
  machine.update(state.events)
  assert machine.state == State.softDisabling
  for _ in range(300):
    machine.update(state.events)
  assert machine.state == State.disabled


@pytest.mark.parametrize('field', ['sensorsOK', 'posenetOK'])
def test_pose_sensor_or_sanity_failure_is_not_relabelled_as_startup(field):
  state = drive(valid=True, healthy=True)
  state.sm['livePose'].inputsOK = False
  setattr(state.sm['livePose'], field, False)
  load_policy()(state)
  assert EventName.locationdTemporaryError in state.events
  assert EventName.selfdriveInitializing not in state.events
  if field == 'posenetOK':
    assert EventName.posenetInvalid in state.events


def test_settling_ends_five_seconds_after_loading_finishes():
  tree = ast.parse((ROOT / 'selfdrived.py').read_text(encoding='utf8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_big_model_settling')
  clock = SimpleNamespace(now=100.0)
  ns = {'time': SimpleNamespace(monotonic=lambda: clock.now)}
  exec(compile(ast.Module(body=[method], type_ignores=[]), '<settling>', 'exec'), ns)
  params = {'UsbGpuLoading': True, 'UsbGpuActive': True}
  state = SimpleNamespace(params=SimpleNamespace(get_bool=params.__getitem__), big_model_loading=False,
                          big_model_active=False, big_model_ready_t=0.0)
  check = ns['_big_model_settling']
  assert check(state)
  params['UsbGpuLoading'] = False
  assert check(state)
  clock.now += 4.99
  assert check(state)
  clock.now += 0.01
  assert not check(state)


def test_missing_first_pose_is_startup_not_a_sensor_fault():
  state = drive()
  state.sm.seen['livePose'] = False
  state.sm['livePose'].inputsOK = state.sm['livePose'].sensorsOK = state.sm['livePose'].posenetOK = False
  load_policy()(state)
  assert state.events == {EventName.selfdriveInitializing}


def test_engaged_communication_failure_is_not_exempt_during_gpu_settling():
  state = drive(valid=True, enabled=True)
  load_policy()(state)
  tree = ast.parse((ROOT / 'selfdrived.py').read_text(encoding='utf8'))
  condition = next(n.test for n in ast.walk(tree) if isinstance(n, ast.If) and
                   'no_system_errors' in ast.unparse(n.test))
  ns = {'self': state, 'no_system_errors': True, 'model_starting': False, 'big_model_settling': True}
  assert eval(compile(ast.Expression(condition), '<communication check>', 'eval'), ns)
