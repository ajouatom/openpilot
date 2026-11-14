import json
import subprocess

import pytest

from openpilot.common import jetlink_status as status


@pytest.mark.parametrize('active', [False, True])
def test_status_replaced_during_read_is_not_a_future_timestamp(monkeypatch, active):
  clock = [100.]
  monkeypatch.setattr(status.time, 'monotonic', lambda: clock[0])

  def concurrent_read(path):
    # The writer atomically replaces the report after the UI starts reading.
    clock[0] += .001
    if path == status.MODEL_STATUS:
      return {'updated': clock[0], 'active': active, 'warp_preparing': not active}
    return {'updated': clock[0], 'state': 'ready', 'peer': {'carrot_host': 'jetson'},
            'telemetry_updated': clock[0],
            'telemetry': {'carrot_health': {'age_s': 0., 'severity': 'ok'}}}

  monkeypatch.setattr(status, '_read', concurrent_read)
  expected = ('jetSON 사용 중', 'active') if active else ('jetSON 콤마 영상 준비 중', 'loading')
  assert status.badge() == expected
  assert status.diagnostics()['severity'] == 'ok'


@pytest.mark.parametrize('reason', [
  '[adb-handoff:opt_in_missing] ADB handoff permission is missing (/data/jetlink-adb-handoff)',
  "[adb-handoff:offroad] Offroad required: IsOffroad='0', IsOnroad='1' (pre-write recheck)",
  '[adb-handoff:io_error] unbind g1 UDC: PermissionError, errno=13',
])
def test_handoff_subprocess_error_reaches_screen_diagnostics(tmp_path, monkeypatch, reason):
  from openpilot.selfdrive.modeld.jetlink import mobile
  def rejected(command, **kwargs):
    raise subprocess.CalledProcessError(1, command, stderr=f'jetlink: {reason}\n')
  monkeypatch.setattr(mobile.subprocess, 'run', rejected)
  monkeypatch.setattr(mobile.os, 'geteuid', lambda: 0, raising=False)
  with pytest.raises(mobile.MobileLinkError) as error:
    mobile._root('setup_gadget.sh')
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  link.write_text(json.dumps({'updated': 10., 'state': 'retrying', 'error': str(error.value)[:300]}))
  diagnostic = status.diagnostics()
  assert diagnostic['severity'] == 'error'
  assert reason in diagnostic['reason']


@pytest.mark.parametrize(('peer', 'expected'), (
  ({'carrot_host': 'jetson'}, 'jetSON'), ({'carrot_host': 'mac'}, 'MAC'),
  ({'backend': 'trt', 'device': 'Orin-sm87'}, 'jetSON'),
  ({'protocol': 2, 'backend': 'ort', 'device': 'coreml-Apple-M1'}, 'MAC'),
  ({'protocol': 2, 'backend': 'ort', 'device': 'ane-Apple_M1_Pro'}, 'MAC'),
  ({'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4'}, 'MAC'),
  ({'protocol': 2, 'backend': 'ort', 'device': 'coreml-Apple_A17_Pro'}, 'Jetlink'),
  ({'backend': 'trt', 'device': 'RTX-sm89'}, 'Jetlink'),
  ({'carrot_host': 'untrusted text'}, 'Jetlink'), ({'carrot_host': []}, 'Jetlink'), (None, 'Jetlink'),
))
def test_host_identity_compatibility(peer, expected):
  assert status.host_label(peer) == expected


def test_phone_label_uses_explicit_transport_not_apple_m_chip_alone():
  ipad = {'protocol': 3, 'backend': 'ort', 'device': 'ane-whole-Apple_M4'}
  android = {'protocol': 3, 'backend': 'litert', 'device': 'gpu-Tensor_G4'}
  assert status.host_label(ipad, 'ios') == 'iOS'
  assert status.host_label(android, 'android') == 'Android'
  assert status.host_label(ipad) == 'MAC'
  assert status.host_label(android) == 'Jetlink'


def test_active_ready_and_expired_host_status(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  link.write_text(json.dumps({'updated': 9.9, 'state': 'ready', 'peer': {'carrot_host': 'mac'}}))
  assert status.badge() == ('MAC READY', 'ready')
  model.write_text(json.dumps({'updated': 9.9, 'active': True}))
  assert status.badge() == ('MAC', 'active')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 14.)
  assert status.badge() == ('MAC ERROR', 'error')
  link.write_text('invalid')
  model.write_text('[]')
  assert status.badge() is None


def test_remote_badge_disappears_with_expired_snapshot(monkeypatch):
  import hud
  params = hud.DisplayParams()
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (10., {'external_compute_label': 'jetSON'}))
  assert params.external_compute_label() == 'jetSON'
  monkeypatch.setattr(hud, 'read_snapshot', lambda: None)
  assert params.external_compute_label() == ''


def test_unplugged_optional_host_is_quiet_before_model_start_and_recovers_to_ready(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20.)
  link.write_text(json.dumps({'updated': 20., 'state': 'waiting', 'peer': {'carrot_host': 'jetson'}}))
  assert status.badge() is None
  assert status.diagnostics()['reason'] == 'Host not connected'
  for report in ({'updated': 20., 'active': False, 'error': ''},
                 {'updated': 10., 'active': False}):
    model.write_text(json.dumps(report))
    assert status.badge() is None
  for report in ({'updated': 20., 'active': False, 'error': 'inference timeout'},
                 {'updated': 20., 'active': True}):
    model.write_text(json.dumps(report))
    assert status.badge() == ('jetSON ERROR', 'error')
  model.unlink()
  link.write_text(json.dumps({'updated': 20., 'state': 'ready', 'peer': {'carrot_host': 'jetson'},
                             'telemetry_updated': 20., 'telemetry': {'carrot_health': {'age_s': 0., 'severity': 'ok'}}}))
  assert status.badge() == ('jetSON READY', 'ready')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 24.)
  assert status.badge() == ('jetSON ERROR', 'error')
