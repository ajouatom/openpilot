import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import urllib.error

import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'third_party/jetlink')]
import boot_update as boot
import update_host as update
from jetlink import protocol as p
from test_update_host import manifest


@pytest.fixture
def env(tmp_path, monkeypatch):
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  monkeypatch.setattr(update, 'PROTECTED_MARKER', tmp_path / 'no-protection')
  monkeypatch.setattr(update, 'verify_signature', lambda _: None)
  monkeypatch.setattr(boot, 'READY', tmp_path / 'ready')
  monkeypatch.setattr(boot, 'STATUS', tmp_path / 'status')
  monkeypatch.setattr(boot, 'BOOT_ID', tmp_path / 'boot-id')
  boot.BOOT_ID.write_text('boot1')
  (tmp_path / 'current').mkdir()
  (tmp_path / 'current/SOURCE_COMMIT').write_text('d' * 40)
  (tmp_path / 'boot-update-required').write_text('prior-boot')
  return tmp_path


def test_matching_signed_release_needs_no_internet_and_ready_is_boot_specific(env, monkeypatch):
  value = manifest()
  (env / 'current/SOURCE_COMMIT').write_text(value['source_commit'])
  monkeypatch.setattr(update, 'stage_manifest', lambda *a, **k: pytest.fail('unnecessary download'))
  gate = boot.Gate()
  gate.tick()
  assert not gate.done and not boot.runtime_ready()
  gate.select(value)
  gate.tick()
  assert gate.done and boot.runtime_ready()
  boot.BOOT_ID.write_text('boot2')
  assert not boot.runtime_ready()


def test_required_update_waits_for_internet_then_activates_before_ready(env, monkeypatch):
  gate = boot.Gate()
  gate.select(manifest())
  calls = []
  def offline(*a, **k):
    raise urllib.error.URLError('offline')
  monkeypatch.setattr(update, 'stage_manifest', offline)
  monkeypatch.setattr(boot.subprocess, 'run', lambda command, **k: calls.append(command))
  gate.tick()
  gate.worker.join(1)
  assert gate.state == 'waiting_internet' and not boot.runtime_ready() and not calls
  gate.tick()
  assert gate.state == 'waiting_internet'  # retry delay, no fallback to old model
  def stage(value, progress):
    calls.append('download')
    progress(.5)
    assert gate.fraction == .5
  def activate():
    calls.append('activate')
    assert calls[-2][:2] == ['systemctl', 'stop']
    (env / 'current/SOURCE_COMMIT').write_text('a' * 40)
  monkeypatch.setattr(update, 'stage_manifest', stage)
  monkeypatch.setattr(update, 'activate', activate)
  gate.next_try = 0
  gate.tick()
  gate.worker.join(1)
  assert not boot.runtime_ready()
  gate.tick()
  assert gate.done and boot.runtime_ready()


def test_rejected_candidate_or_stale_pin_never_starts_old_runtime(env, monkeypatch):
  gate = boot.Gate()
  gate.select(manifest())
  monkeypatch.setattr(update, 'stage_manifest', lambda *a, **k: None)
  monkeypatch.setattr(update, 'activate', lambda: None)  # existing updater rolled back
  monkeypatch.setattr(boot.subprocess, 'run', lambda *a, **k: None)
  gate.tick()
  gate.worker.join(1)
  gate.tick()
  assert gate.state == 'failed' and not gate.done and not boot.runtime_ready()
  (env / 'current/SOURCE_COMMIT').write_text('a' * 40)
  gate.received -= 10
  gate.tick()
  assert not gate.done


def test_invalid_signature_cannot_approve_matching_release(env, monkeypatch):
  gate = boot.Gate()
  (env / 'current/SOURCE_COMMIT').write_text('a' * 40)
  def reject(_):
    raise ValueError('invalid signature')
  monkeypatch.setattr(update, 'verify_signature', reject)
  with pytest.raises(ValueError):
    gate.select(manifest())
  gate.tick()
  assert not gate.done


def test_installer_defers_policy_until_next_boot_and_runtime_guard_waits(env, monkeypatch):
  assert boot.enabled()
  (env / 'boot-update-required').write_text('boot1')
  assert not boot.enabled()
  boot.BOOT_ID.write_text('boot2')
  assert boot.enabled()
  calls = []
  def release(_):
    calls.append('waiting')
    update.atomic_json(boot.READY, {'boot_id': 'boot2', 'source_commit': 'd' * 40})
  monkeypatch.setattr(boot.time, 'sleep', release)
  boot.wait_for_runtime()
  assert calls == ['waiting']


def test_boot_usb_handles_version_wifi_status_but_never_inference(env, monkeypatch):
  import wifi_protocol
  sent, wifi = [], []
  monkeypatch.setattr(boot, 'receive_wifi', wifi.append)
  transport = NS(send_json=lambda kind, seq, value: sent.append((kind, value)), send=lambda *a: None)
  gate = boot.Gate()
  session = boot.BootstrapSession(transport, gate)
  for seq,kind,payload in [(1, p.Msg.HELLO_REQ, b'{}'), (2, wifi_protocol.MESSAGE, b'private'),
                           (3, boot.MESSAGE, json.dumps(manifest()).encode()), (4, p.Msg.STATE_REQ, b'{}'),
                           (5, p.Msg.ENGINE_REQ, b'{}'), (6, p.Msg.INFER_REQ, b'')]:
    session.handle(NS(seq=seq, msg_type=kind, payload=payload))
  assert sent[0][1][boot.CAPABILITY] is True and sent[0][1]['loaded'] is None
  assert wifi == [b'private'] and gate.selected == manifest()
  assert sent[1][0] == p.Msg.STATE_RESP
  assert [s[0] for s in sent[2:]] == [p.Msg.ERROR, p.Msg.ERROR]
  assert 'private' not in json.dumps(sent)


def test_vehicle_pumps_updates_before_engine_and_ignores_non_gate_peers(tmp_path, monkeypatch):
  from openpilot.selfdrive.modeld.jetlink import startup
  release = tmp_path / 'release'
  release.write_text(json.dumps(manifest()))
  monkeypatch.setattr(startup, 'RELEASE', release)
  calls = []
  states = iter(['waiting_internet', 'downloading', 'verifying', 'ready'])
  client = NS(t=NS(send_json=lambda *a: calls.append('manifest')), _next_seq=lambda: 1,
              state=lambda: {'carrot_update': {'state': next(states)}})
  wifi = NS(send=lambda _: calls.append('wifi'))
  startup.wait_for_boot_update(client, {'carrot_host': 'mac'}, wifi, lambda: True, lambda *a, **k: None)
  assert calls == []
  reports = []
  with pytest.raises(ConnectionError, match='reconnecting'):
    startup.wait_for_boot_update(client, {'carrot_host': 'jetson', startup.CAPABILITY: True}, wifi,
                                lambda: True, lambda *a, **k: reports.append(k['host_update']['state']), sleep=lambda _:None)
  assert reports == ['waiting_internet', 'downloading', 'verifying', 'ready']
  assert calls == ['manifest', 'wifi'] * 4


def test_device_badge_describes_update_and_expires(tmp_path, monkeypatch):
  from openpilot.common import jetlink_status as status
  path = tmp_path / 'link'
  monkeypatch.setattr(status, 'LINK_STATUS', path)
  monkeypatch.setattr(status, 'MODEL_STATUS', tmp_path / 'absent')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  path.write_text(json.dumps({'updated': 10., 'state': 'updating', 'peer': {'carrot_host': 'jetson'},
                             'host_update': {'state': 'waiting_internet'}}))
  assert '인터넷 연결 대기' in status.badge()[0]
  assert '업데이트 필요' in status.diagnostics()['reason']
  assert not status.diagnostics()['active']
  monkeypatch.setattr(status.time, 'monotonic', lambda: 14.)
  assert '인터넷' not in status.badge()[0]


def test_bootstrap_migration_never_writes_immutable_os_and_defers_current_boot(tmp_path, monkeypatch):
  import install_updates
  source = Path(__file__).parent
  real_read = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda path, *a, **k:
                      'current-boot' if path.as_posix() == '/proc/sys/kernel/random/boot_id' else real_read(path, *a, **k))
  install_updates.bootstrap(tmp_path, source, defer_this_boot=True)
  target = tmp_path / 'opt/carrot-jetlink'
  assert (target / 'boot-update-required').read_text() == 'current-boot'
  for name in ('boot_update.py', 'update_host.py', 'wifi_protocol.py', 'release-signing-public.pem'):
    assert (target / 'updater' / name).read_bytes() == (source / name).read_bytes()
  assert not (tmp_path / 'etc').exists()


def test_existing_apply_service_launches_gate_without_old_start_timeout(env, monkeypatch):
  monkeypatch.setattr(sys, 'argv', ['update_host.py', 'activate'])
  monkeypatch.setattr(update.os, 'geteuid', lambda: 0, raising=False)
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  calls = []
  monkeypatch.setattr(update.subprocess, 'run', lambda command, **kwargs: calls.append(command))
  update.main()
  command, = calls
  assert command[0] == 'systemd-run' and command[-1] == 'boot'
  assert '--property=Restart=on-failure' in command
  assert not any('reboot' in part for part in command)
