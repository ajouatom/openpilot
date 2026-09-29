import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived import events
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager


EMPTY = events.EmptyAlert


def ready_check(*, replay=False, simulation=False, method_name='update_system_ready_alert'):
  path = Path(__file__).resolve().parents[1] / 'selfdrived.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method_name)
  namespace = {'EventName': log.OnroadEvent.EventName, 'ET': events.ET,
               'DT_CTRL': 0.01, 'EmptyAlert': EMPTY, 'REPLAY': replay, 'SIMULATION': simulation,
               'LONGITUDINAL_PERSONALITY_MAP': {1: 'standard'}}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), 'exec'), namespace)
  return namespace[method_name]


class Messages(dict):
  frame = 100
  healthy = True

  def all_checks(self):
    return self.healthy


class Events:
  blocked = False

  def __init__(self):
    self.names = []

  def contains(self, event_type):
    assert event_type == 'noEntry'
    return self.blocked

  def add(self, event):
    self.names.append(event)


def drive():
  state = SimpleNamespace(initialized=True, CP=SimpleNamespace(passive=False),
                          sm=Messages(deviceState=SimpleNamespace(started=True)),
                          events=Events(), AM=SimpleNamespace(current_alert=EMPTY),
                          system_ready_alerted=False, system_ready_since=None, updates=[])
  state.update_alerts = lambda cs: state.updates.append(cs)
  return state, SimpleNamespace(canValid=True, canTimeout=False)


def test_ready_only_once_after_stable_health_and_again_next_onroad():
  check = ready_check()
  for _ in range(2):
    state, cs = drive()  # selfdrived is an onroad process
    check(state, cs)
    state.sm.frame += 49
    check(state, cs)
    assert not state.events.names
    state.sm.frame += 1
    check(state, cs)
    assert state.events.names == [log.OnroadEvent.EventName.systemReady]
    assert state.updates == [cs]
    state.sm.healthy = False
    check(state, cs)
    state.sm.healthy = True
    state.sm.frame += 1000
    check(state, cs)
    assert len(state.events.names) == 1


@pytest.mark.parametrize('failure', ['initializing', 'passive', 'offroad', 'canInvalid', 'canTimeout', 'services', 'noEntry'])
def test_unready_conditions_reset_continuous_health(failure):
  check = ready_check()
  state, cs = drive()
  check(state, cs)
  state.sm.frame += 49
  target, attr, value = {
    'initializing': (state, 'initialized', False),
    'passive': (state.CP, 'passive', True),
    'offroad': (state.sm['deviceState'], 'started', False),
    'canInvalid': (cs, 'canValid', False),
    'canTimeout': (cs, 'canTimeout', True),
    'services': (state.sm, 'healthy', False),
    'noEntry': (state.events, 'blocked', True),
  }[failure]
  setattr(target, attr, value)
  check(state, cs)
  assert state.system_ready_since is None
  assert not state.events.names
  setattr(target, attr, not value)
  state.sm.frame += 1
  check(state, cs)
  assert not state.events.names
  state.sm.frame += 50
  check(state, cs)
  assert state.system_ready_alerted


def test_waits_for_existing_alert_and_does_not_announce_during_fault():
  check = ready_check()
  state, cs = drive()
  state.AM.current_alert = object()
  check(state, cs)
  state.sm.frame += 1000
  check(state, cs)
  assert not state.system_ready_alerted
  state.events.blocked = True
  state.AM.current_alert = EMPTY
  check(state, cs)
  assert not state.system_ready_alerted
  state.events.blocked = False
  check(state, cs)
  state.sm.frame += 50
  check(state, cs)
  assert state.system_ready_alerted


@pytest.mark.parametrize('replay,simulation', [(True, False), (False, True)])
def test_no_ready_chime_in_replay_or_simulation(replay, simulation):
  state, cs = drive()
  check = ready_check(replay=replay, simulation=simulation)
  check(state, cs)
  state.sm.frame += 1000
  check(state, cs)
  assert not state.events.names


def test_real_event_and_alert_manager_delivery_and_warning_priority():
  state, cs = drive()
  state.events = events.Events()
  state.AM = AlertManager()
  state.enabled = False
  state.is_metric = True
  state.personality = 1
  state.state_machine = SimpleNamespace(current_alert_types=[events.ET.PERMANENT], soft_disable_timer=0)
  update_alerts = ready_check(method_name='update_alerts')
  state.update_alerts = lambda cs: update_alerts(state, cs)
  check = ready_check()
  check(state, cs)
  state.sm.frame += 50
  check(state, cs)
  alert = state.AM.current_alert
  assert alert.alert_type == 'systemReady/permanent'
  assert alert.audible_alert == events.AudibleAlert.systemReady
  assert alert.alert_size == events.AlertSize.none
  assert alert.priority == events.Priority.LOWEST
  assert set(events.EVENTS[log.OnroadEvent.EventName.systemReady]) == {events.ET.PERMANENT}
  event, = state.events.to_msg()
  with log.OnroadEvent.from_bytes(event.to_bytes()) as decoded:
    assert decoded.name == 'systemReady'
    assert decoded.permanent and not decoded.noEntry and not decoded.enable
  state.sm.frame += 1
  state.events.clear()
  state.events.add(log.OnroadEvent.EventName.driverDistracted3)
  state.update_alerts(cs)
  assert state.AM.current_alert.audible_alert == events.AudibleAlert.warningImmediate
