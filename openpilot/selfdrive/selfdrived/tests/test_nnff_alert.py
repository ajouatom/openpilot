"""NNFF startup notification must survive delayed readiness and alert contention."""
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived import events
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.tests.test_system_ready import drive, ready_check


EventName = log.OnroadEvent.EventName


@pytest.fixture
def session(monkeypatch):
  state, cs = drive()
  state.sm.frame = 550
  state.nnff_alerted = False
  state.params = {'NNFFModelName': 'TEST CAR'}
  monkeypatch.setattr(events, 'Params', lambda: state.params)
  state.events = events.Events()
  state.AM = AlertManager()
  state.enabled = False
  state.is_metric = True
  state.personality = 1
  state.state_machine = SimpleNamespace(current_alert_types=[events.ET.PERMANENT], soft_disable_timer=0)
  select = ready_check(method_name='update_alerts')
  notify = ready_check(method_name='update_nnff_alert')
  state.update_alerts = lambda cs: select(state, cs)
  state.update_nnff_alert = lambda cs: notify(state, cs)
  return state, cs


def tick(state, cs, frame, event=None):
  state.sm.frame = frame
  state.events.clear()
  if event is not None:
    state.events.add(event)
  state.update_alerts(cs)
  state.update_nnff_alert(cs)


def test_initialization_later_than_old_trigger_still_notifies_once(session):
  state, cs = session
  state.initialized = False
  tick(state, cs, 550)
  assert not state.nnff_alerted
  state.initialized = True
  tick(state, cs, 900)
  assert state.nnff_alerted
  alert = state.AM.current_alert
  assert alert.alert_type == 'torqueNNLoad/permanent'
  assert alert.alert_text_1 == 'NNFF Torque Controller loaded'
  assert alert.alert_text_2 == 'TEST CAR'
  assert alert.duration == 500
  assert alert.audible_alert == events.AudibleAlert.prompt
  tick(state, cs, 1450)
  assert state.AM.current_alert is events.EmptyAlert
  tick(state, cs, 5000)
  assert EventName.torqueNNLoad not in state.events.names


@pytest.mark.parametrize('failure', ['initializing', 'passive', 'offroad', 'canInvalid', 'canTimeout', 'services'])
def test_waits_for_health_even_after_initialization_timeout(session, failure):
  state, cs = session
  target, attr, value = {
    'initializing': (state, 'initialized', False),
    'passive': (state.CP, 'passive', True),
    'offroad': (state.sm['deviceState'], 'started', False),
    'canInvalid': (cs, 'canValid', False),
    'canTimeout': (cs, 'canTimeout', True),
    'services': (state.sm, 'healthy', False),
  }[failure]
  setattr(target, attr, value)
  tick(state, cs, 900)
  assert not state.nnff_alerted
  setattr(target, attr, not value)
  tick(state, cs, 950)
  assert state.nnff_alerted


def test_no_entry_waits_even_without_visible_alert(session):
  state, cs = session
  tick(state, cs, 900, EventName.selfdriveInitializing)
  assert state.AM.current_alert is events.EmptyAlert
  assert not state.nnff_alerted
  tick(state, cs, 950)
  assert state.nnff_alerted


def test_waits_for_same_frame_warning_and_yields_to_later_warning(session):
  state, cs = session
  tick(state, cs, 900, EventName.driverDistracted3)
  assert state.AM.current_alert.alert_type == 'driverDistracted3/permanent'
  assert not state.nnff_alerted
  tick(state, cs, 1000)
  assert state.AM.current_alert.alert_type == 'torqueNNLoad/permanent'
  tick(state, cs, 1001, EventName.driverDistracted3)
  assert state.AM.current_alert.alert_type == 'driverDistracted3/permanent'
  assert state.nnff_alerted


def test_late_model_name_is_retried_without_100hz_param_reads(session):
  state, cs = session
  state.params.clear()
  tick(state, cs, 900)
  assert not state.nnff_alerted
  state.params['NNFFModelName'] = 'TEST CAR'
  tick(state, cs, 901)
  assert not state.nnff_alerted
  tick(state, cs, 950)
  assert state.nnff_alerted


def test_preserves_minimum_startup_delay(session):
  state, cs = session
  tick(state, cs, 500)
  assert not state.nnff_alerted
  tick(state, cs, 550)
  assert state.nnff_alerted


def test_empty_model_name_keeps_existing_unavailable_notice(session):
  state, cs = session
  state.params['NNFFModelName'] = ''
  tick(state, cs, 900)
  assert state.AM.current_alert.alert_text_1 == 'NNFF Torque Controller not available'


def test_actual_step_schedules_ready_chime_before_nnff(session):
  state, cs = session
  state.system_ready_since = 850
  state.sm.frame = 900
  ready = ready_check()
  step = ready_check(method_name='step')
  state.data_sample = lambda: cs
  state.update_impact_dashcam = lambda cs: None
  state.update_events = lambda cs: state.events.clear()
  state.impact_dashcam = SimpleNamespace(pending=False, committed=False)
  state.state_machine.update = lambda _: (False, False)
  state.update_system_ready_alert = lambda cs: ready(state, cs)
  published = []
  state.publish_selfdriveState = lambda cs: published.append(state.AM.current_alert.alert_type)
  step(state)
  assert state.system_ready_alerted
  assert not state.nnff_alerted
  assert published == ['systemReady/permanent']
  state.sm.frame = 950
  step(state)
  assert state.nnff_alerted
  assert published[-1] == 'torqueNNLoad/permanent'
