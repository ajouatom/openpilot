import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.common import jetson_maintenance as maintenance
from openpilot.selfdrive.modeld.jetlink.startup import wait_for_legacy_update


def vehicle():
  values = {'carState': NS(gearShifter='park', standstill=True, vEgoRaw=0),
            'selfdriveState': NS(enabled=False, active=False),
            'carControl': NS(enabled=False, latActive=False, longActive=False)}
  class SM(dict):
    valid = dict.fromkeys(values, True)
    alive = dict.fromkeys(values, True)
  return SM(values)


@pytest.mark.parametrize('service,key,value', [
  ('carState', 'gearShifter', 'drive'), ('carState', 'standstill', False), ('carState', 'vEgoRaw', .01),
  ('selfdriveState', 'enabled', True), ('selfdriveState', 'active', True),
  ('carControl', 'enabled', True), ('carControl', 'latActive', True), ('carControl', 'longActive', True),
])
def test_entry_requires_park_and_inactive_controls(service, key, value):
  sm = vehicle()
  assert maintenance.parked(sm)
  setattr(sm[service], key, value)
  assert not maintenance.parked(sm)


@pytest.mark.parametrize('service', ['carState', 'selfdriveState', 'carControl'])
@pytest.mark.parametrize('quality', ['valid', 'alive'])
def test_no_entry_on_invalid_or_stale_vehicle_signals(service, quality):
  sm = vehicle()
  getattr(sm, quality)[service] = False
  assert not maintenance.parked(sm)


@pytest.fixture
def peer(tmp_path, monkeypatch):
  pin = tmp_path / 'release.json'
  pin.write_text(json.dumps({'source_commit': 'a' * 40}))
  monkeypatch.setattr(maintenance, 'RELEASE', pin)
  return {'carrot_host': 'jetson', 'carrot_source_commit': 'a' * 40, 'carrot_boot_update_installed': True}


def test_only_current_normal_runtime_receipt_completes_transition(peer):
  assert maintenance.migrated(peer)
  for change in ({'carrot_host': 'mac'}, {'carrot_source_commit': 'b' * 40},
                 {'carrot_boot_update_installed': False}, {'carrot_boot_update_installed': 'true'},
                 {'carrot_boot_update_v1': True}):
    assert not maintenance.migrated({**peer, **change})
  assert not maintenance.migrated({})


class Params:
  def __init__(self):
    self.pending = True

  def get_bool(self, key):
    assert key == maintenance.PENDING
    return self.pending

  def put_bool(self, key, value):
    assert key == maintenance.PENDING
    self.pending = value


def test_old_runtime_wait_keeps_provisioning_without_model_calls(peer):
  params, events = Params(), []
  client = NS(state=lambda: events.append('state'))  # deliberately no infer/engine API
  wifi = NS(send=lambda client: events.append('wifi'))
  ticks = iter([True, True, False])
  wait_for_legacy_update(client, {}, wifi, params, lambda: next(ticks),
                         lambda state, **kw: events.append(state), sleep=lambda _: None)
  assert events == ['wifi', 'state', 'maintenance'] * 2
  assert params.pending  # disconnect/power loss is not completion
  wait_for_legacy_update(client, peer, wifi, params, lambda: True, lambda *a, **k: None)
  assert not params.pending and len(events) == 6


def test_signed_runtime_installs_bootstrap_once_without_reboot(tmp_path, monkeypatch):
  import install_updates
  source = tmp_path / 'current/tools/jetlink'
  source.mkdir(parents=True)
  (source / 'boot_update.py').write_text('installed')
  calls = []
  monkeypatch.setattr(install_updates.subprocess, 'run', lambda args, **kw: calls.append((args, kw)))
  install_updates.migrate_running_release(tmp_path)
  args, kwargs = calls[0]
  assert args[:2] == ['sudo', '-n'] and args[-1] == '--bootstrap-only'
  assert kwargs == {'check': True, 'timeout': 30}
  (tmp_path / 'boot-update-required').write_text('this-boot')
  install_updates.migrate_running_release(tmp_path)
  assert len(calls) == 1


def test_alert_explicitly_describes_legacy_limitation():
  assert '다운로드 완료 여부를 알려주지 않습니다' in maintenance.alert_text('ko-KR')
  assert 'cannot report download completion' in maintenance.alert_text('en-US')


def test_wait_clock_survives_usb_loss_but_never_clears_hold(tmp_path, monkeypatch):
  clock = tmp_path / 'wait-start'
  monkeypatch.setattr(maintenance, 'WAIT_CLOCK', clock)
  now = [100.]
  monkeypatch.setattr(maintenance.time, 'monotonic', lambda: now[0])
  link = {'state': 'maintenance', 'peer': {'carrot_host': 'jetson'}}
  monkeypatch.setattr(maintenance, '_fresh', lambda *args: link)
  params = Params()
  assert maintenance.status(params)['wait_elapsed_seconds'] == 0
  now[0] += 125
  link.clear()  # Jetson power cycle or unplug does not reset the C4 clock.
  result = maintenance.status(params)
  assert not result['connected'] and result['wait_elapsed_seconds'] == 125
  now[0] += 3600
  result = maintenance.status(params)
  assert result['pending'] and result['wait_elapsed_seconds'] == 3725
  params.pending = False
  assert maintenance.status(params)['wait_elapsed_seconds'] is None
  assert not clock.exists()
  params.pending = True
  assert maintenance.status(params)['wait_elapsed_seconds'] == 0
  clock.unlink()  # /dev/shm is cleared by C4 reboot; pending remains persistent.
  assert maintenance.status(params)['wait_elapsed_seconds'] == 0
  assert params.pending


@pytest.mark.parametrize('corrupt', ['garbage', 'nan', 'inf', '-1', '99999'])
def test_invalid_wait_clock_restarts_measurement(tmp_path, monkeypatch, corrupt):
  clock = tmp_path / 'wait-start'
  clock.write_text(corrupt)
  monkeypatch.setattr(maintenance, 'WAIT_CLOCK', clock)
  assert maintenance.wait_elapsed(True, 10.) == 0
  assert maintenance.wait_elapsed(True, 72.) == 62


def test_wait_clock_storage_failure_does_not_change_pending(tmp_path, monkeypatch):
  monkeypatch.setattr(maintenance, 'WAIT_CLOCK', tmp_path / 'missing' / 'wait-start')
  monkeypatch.setattr(maintenance, '_fresh', lambda *args: {})
  params = Params()
  result = maintenance.status(params)
  assert result['pending'] and result['wait_elapsed_seconds'] is None


@pytest.mark.parametrize('safe', [True, False])
def test_web_entry_checks_continuous_vehicle_state(monkeypatch, safe):
  import asyncio
  import importlib.util
  import sys
  path = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/server/features/tools/jetson_update.py'
  spec = importlib.util.spec_from_file_location('jetson_update_api', path)
  api = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(api)
  params = Params()
  params.pending = False
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: params))
  sm = vehicle()
  sm.update = lambda _: None
  if not safe:
    sm['carControl'].latActive = True
  monkeypatch.setitem(sys.modules, 'openpilot.cereal.messaging', NS(SubMaster=lambda _: sm))
  monkeypatch.setattr(api, 'status', lambda _: {'connected': True, 'migrated': False})
  clock = [0.]
  monkeypatch.setattr(api, 'time', NS(monotonic=lambda: clock[0]))
  async def sleep(delay):
    clock[0] += delay
  monkeypatch.setattr(api.asyncio, 'sleep', sleep)
  async def body():
    return {'enabled': True}
  response = asyncio.run(api.request_wait(NS(json=body)))
  assert response.status == (200 if safe else 409)
  assert params.pending is safe
  assert clock[0] >= 1


def test_maintenance_cannot_be_set_or_restored_through_settings():
  from openpilot.selfdrive.carrot.server.services import params as service
  with pytest.raises(ValueError, match='controlled internally'):
    service.set_param_value(maintenance.PENDING, True)
  assert service.filter_param_backup_values({maintenance.PENDING: '1', 'Other': '5'}) == {'Other': '5'}


@pytest.mark.parametrize('onroad,offroad,encoded', [(True, False, 'MQ=='), (False, False, 'MQ=='), (False, True, 'MA==')])
def test_pre_wifi_hosts_receive_actual_offroad_state_without_preview_workers(onroad, offroad, encoded):
  params = NS(get_bool=lambda name: {maintenance.PENDING: True, 'IsOnroad': onroad, 'IsOffroad': offroad}[name])
  packets = []
  client = NS(t=NS(send_json=lambda *args: packets.append(args)), _next_seq=lambda: 1, state=dict)
  connected = iter([True, False])
  wait_for_legacy_update(client, {'carrot_hud_v1': True}, None, params, lambda: next(connected),
                         lambda *a, **kw: None, sleep=lambda _: None)
  message, _, value = packets[0]
  assert message == 0x4000 and value['params']['IsOnroad'] == encoded
  assert value['jetson_release']['signature'] and value['events'] == {}
