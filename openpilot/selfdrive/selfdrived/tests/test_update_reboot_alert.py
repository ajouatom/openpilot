import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived import events
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, load_translations


def alert_check(*, replay=False, simulation=False):
  # Execute the actual method without importing device-only IPC/runtime modules.
  path = Path(__file__).resolve().parents[1] / 'selfdrived.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_reboot_alert')
  namespace = {'EventName': log.OnroadEvent.EventName, 'DT_CTRL': 0.01, 'REPLAY': replay, 'SIMULATION': simulation}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), 'exec'), namespace)
  return namespace['update_reboot_alert']


class Messages(dict):
  frame = 0
  healthy = True

  def all_checks(self, services):
    assert services == ['managerState']
    return self.healthy


def drive():
  # Exercise the serialized managerState field, including its default false.
  msg = log.ManagerState.new_message(rebootRequired=True)
  with log.ManagerState.from_bytes(msg.to_bytes()) as decoded:
    sm = Messages(managerState=SimpleNamespace(rebootRequired=decoded.rebootRequired))
  return SimpleNamespace(sm=sm, events=events.Events(), update_reboot_alerted=False)


def test_delayed_notice_once_per_drive_and_again_next_ignition():
  check = alert_check()
  state = drive()
  state.sm.frame = 1499
  check(state)
  assert state.events.names == []
  state.sm.frame = 1500
  check(state)
  assert state.events.names == [log.OnroadEvent.EventName.updateRebootRequired]
  state.events.clear()
  state.sm.frame = 3000
  check(state)
  assert state.events.names == []
  # Onroad processes restart, while the manager retains its startup identity.
  state = drive()
  state.sm.frame = 1500
  check(state)
  assert state.events.names == [log.OnroadEvent.EventName.updateRebootRequired]


@pytest.mark.parametrize('healthy,pending,replay,simulation', [
  (False, True, False, False), (True, False, False, False),
  (True, True, True, False), (True, True, False, True),
])
def test_no_notice_for_missing_status_applied_update_or_replay(healthy, pending, replay, simulation):
  state = drive()
  state.sm.frame = 2000
  state.sm.healthy = healthy
  state.sm['managerState'].rebootRequired = pending
  alert_check(replay=replay, simulation=simulation)(state)
  assert state.events.names == []
  assert not state.update_reboot_alerted


def test_later_update_is_still_reported_after_startup():
  state = drive()
  state.sm.frame = 5000
  state.sm['managerState'].rebootRequired = False
  check = alert_check()
  check(state)
  state.sm.frame += 1
  state.sm['managerState'].rebootRequired = True
  check(state)
  assert state.events.names == [log.OnroadEvent.EventName.updateRebootRequired]


def test_notice_has_no_disengage_and_yields_to_critical_alerts(monkeypatch):
  monkeypatch.setattr(events, 'tr', lambda text: text)
  event = log.OnroadEvent.EventName.updateRebootRequired
  assert set(events.EVENTS[event]) == {events.ET.PERMANENT}
  pending = events.Events()
  pending.add(event)
  alert, = pending.create_alerts([events.ET.PERMANENT])
  assert alert.alert_size == events.AlertSize.mid
  assert alert.duration == 800
  assert alert.audible_alert == events.AudibleAlert.prompt
  machine = StateMachine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  assert machine.update(pending) == (True, True)
  am = AlertManager()
  am.add_many(1500, [alert])
  pending.add(log.OnroadEvent.EventName.fcw)
  am.add_many(1500, pending.create_alerts([events.ET.PERMANENT]))
  am.process_alerts(1500, set())
  assert am.current_alert.priority > alert.priority


@pytest.mark.parametrize('locale', ['ko', 'en', 'zh-CHS'])
def test_notice_translations(locale):
  strings, _ = load_translations(TRANSLATIONS_DIR / f'app_{locale}.po')
  for source in ('Reboot to Apply Update', 'Park safely before rebooting your device'):
    assert strings[source]
  if locale == 'ko':
    assert strings['Reboot to Apply Update'] == '업데이트 적용을 위해 재부팅하세요'
