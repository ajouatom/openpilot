from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.cereal import car, log
from openpilot.selfdrive.selfdrived import events
from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, load_translations


@pytest.mark.parametrize('event_name', [log.OnroadEvent.EventName.canError, log.OnroadEvent.EventName.canBusMissing])
@pytest.mark.parametrize('brand, flags, setting, detected, expected', [
  ('hyundai', 0, 0, True, True),
  ('hyundai', 0, 0, False, False),
  ('hyundai', 0, 1, True, False),
  ('hyundai', 0, 2, True, False),
  ('hyundai', 0, 3, True, False),
  ('hyundai', HyundaiFlags.CAMERA_SCC, 0, True, False),
  ('toyota', 0, 0, True, False),
])
def test_can_error_hint_and_existing_disable_behavior(monkeypatch, event_name, brand, flags, setting, detected, expected):
  params = SimpleNamespace(get_int=lambda key: setting, get_bool=lambda key: detected,
                           get=lambda key: 'existing parser diagnostic')
  monkeypatch.setattr(events, 'Params', lambda: params)
  monkeypatch.setattr(events, 'tr', lambda text: text)
  cp = car.CarParams(brand=brand, flags=int(flags))
  args = [cp, car.CarState(), None, True, 0, None]
  alerts = events.Events()
  # Detection alone never creates an event or blocks engagement.
  assert alerts.create_alerts([events.ET.PERMANENT], args) == []
  alerts.add(event_name)
  alerts.event_counters[event_name] = 100
  alert, = alerts.create_alerts([events.ET.PERMANENT], args)
  if expected:
    assert alert.alert_text_1 == 'CAN Error: Enable CameraSCC'
    assert alert.alert_text_2 == 'SCC detected on camera bus'
    assert alert.alert_size == events.AlertSize.mid  # Both lines are rendered.
  else:
    assert alert.alert_text_1 == 'CAN Error: Check Connections!!'
    assert alert.alert_text_2 == 'existing parser diagnostic'
    assert alert.alert_size == events.AlertSize.small
  assert alert.priority == events.Priority.LOW
  assert alert.audible_alert == events.AudibleAlert.none
  assert alert.creation_delay == 1.
  immediate, = alerts.create_alerts([events.ET.IMMEDIATE_DISABLE], args)
  assert immediate.priority == events.Priority.HIGHEST
  assert immediate.alert_size == events.AlertSize.full
  assert immediate.audible_alert == events.AudibleAlert.warningImmediate
  assert alerts.contains(events.ET.NO_ENTRY)


@pytest.mark.parametrize('locale', ['ko', 'zh-CHS'])
def test_camera_scc_hint_translations(locale):
  translations, _ = load_translations(TRANSLATIONS_DIR / f'app_{locale}.po')
  for source in ('CAN Error: Enable CameraSCC', 'SCC detected on camera bus'):
    assert translations[source]
    assert translations[source] != source
