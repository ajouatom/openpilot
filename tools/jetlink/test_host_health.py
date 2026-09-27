import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from host_health import thermal_health
from openpilot.common import jetlink_status as status


def test_thermal_uses_zone_limits_and_ignores_disconnected_sensors(tmp_path):
  zone = tmp_path / 'thermal_zone0'
  zone.mkdir()
  (zone / 'type').write_text('tj-thermal')
  (zone / 'trip_point_0_type').write_text('passive')
  (zone / 'trip_point_0_temp').write_text('92000')
  for temp, severity in ((86000, 'ok'), (87000, 'warning'), (92000, 'error'), (60000, 'ok'), (-256000, 'unknown')):
    (zone / 'temp').write_text(str(temp))
    assert thermal_health(tmp_path)['severity'] == severity


def test_expiry_and_error_override_active_model(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20.)
  model.write_text(json.dumps({'updated': 20., 'active': True}))
  record = {'updated': 20., 'state': 'ready', 'peer': {'carrot_host': 'jetson'},
            'telemetry_updated': 19., 'telemetry': {'carrot_health': {
              'age_s': .1, 'severity': 'warning', 'temp_c': 87.,
              'addresses': [{'address': '192.168.0.199'}, {'address': '<script>'}, {'address': '127.0.0.1'}]}}}
  link.write_text(json.dumps(record))
  assert status.diagnostics()['addresses'] == ['192.168.0.199']
  assert status.badge() == ('jetSON HOT', 'loading')
  record['telemetry_updated'] = 10.
  link.write_text(json.dumps(record))
  assert status.diagnostics()['addresses'] == []
  assert status.diagnostics()['temp_c'] is None
  assert status.badge() == ('jetSON DATA?', 'loading')
  record.update(state='retrying', error='engine failed')
  link.write_text(json.dumps(record))
  assert status.badge() == ('jetSON ERROR', 'error')


def test_hello_telemetry_never_becomes_current_health(tmp_path, monkeypatch):
  link = tmp_path / 'link'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', tmp_path / 'missing')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  link.write_text(json.dumps({'updated': 10., 'state': 'ready', 'peer': {
    'carrot_host': 'jetson', 'telemetry': {'temp_c': 42}}}))
  assert status.diagnostics()['temp_c'] is None
  monkeypatch.setattr(status.time, 'monotonic', lambda: 15.)
  assert status.badge() == ('jetSON ERROR', 'error')
