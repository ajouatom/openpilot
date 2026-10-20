"""Real files, hashes and activation transactions over an in-memory USB transport.

No installed services, vehicle CAN, TensorRT execution or device access.
"""
import builtins
import copy
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from test_local_experiment import env as _catalog_fixture, archive, select_and_activate
import boot_update as boot
import install_usb_receiver as enroll
import prepare_usb_release as prepare
import update_host as update
import usb_release as usb
from jetlink import protocol as p
from openpilot.common import jetson_model
from openpilot.selfdrive.modeld.jetlink import startup
from tools.jetlink import usb_release_client as sender  # noqa: TID251 -- standalone updater

env = _catalog_fixture  # Reuse the official root-owned filesystem fixture.


@pytest.fixture
def installed(env, monkeypatch):
  monkeypatch.setattr(usb.os, 'sync', lambda: None)
  env.value = prepare.prepare(env.package / 'host.tar.gz', env.package / 'model.onnx', 'test-usb',
                              'https://example.invalid/host.tar.gz', 'https://example.invalid/model.onnx', env.temporary / 'public')
  env.official_key = (env.root / 'updater/release-signing-public.pem').read_bytes()
  enroll.install(env.root)
  return env


class Wire:
  def __init__(self):
    self.responses, self.kinds, self.seq = [], [], 0
    self.session = boot.BootstrapSession(self, boot.Gate())
    self.client = NS(t=self, _next_seq=self.next_seq,
                     state=lambda: {'carrot_update': {'state': 'ready'}})

  def next_seq(self):
    self.seq += 1
    return self.seq

  def send_json(self, kind, seq, value):
    if kind == startup.MESSAGE:
      self.session.handle(NS(msg_type=kind, seq=seq, payload=json.dumps(value).encode()))
    else:
      self.responses.append(NS(msg_type=kind, seq=seq, payload=json.dumps(value).encode()))

  def send(self, kind, seq, parts, **_):
    self.kinds.append(kind)
    self.session.handle(NS(msg_type=kind, seq=seq, payload=b''.join(parts)))

  def recv(self, **_):
    return self.responses.pop(0)


def cache_assets(env):
  cache = env.temporary / 'cache'
  cache.mkdir(exist_ok=True)
  for kind, name in zip(usb.KINDS, usb.NAMES, strict=True):
    (cache / env.value[kind]['sha256']).write_bytes((env.package / name).read_bytes())
  return cache


def upload(env, allowed=lambda: True):
  wire = Wire()
  try:
    sender.upload(wire.client, env.value, cache_assets(env), allowed, lambda *a: None)
  finally:
    wire.session.close()
  return wire


def deny_crypto(monkeypatch):
  original = builtins.__import__
  def check(name, *args, **kwargs):
    if name.startswith('Crypto'):
      pytest.fail('SHA-only USB must not need signing keys or Crypto imports')
    return original(name, *args, **kwargs)
  monkeypatch.setattr(builtins, '__import__', check)


def test_keyless_upload_registration_activation_and_cached_reuse(installed, monkeypatch):
  deny_crypto(monkeypatch)
  monkeypatch.setattr(sender.urllib.request, 'urlopen', lambda *a, **k: pytest.fail('cached upload attempted HTTP'))
  wire = upload(installed)
  assert wire.kinds[0] == usb.BEGIN and wire.kinds[-1] == usb.FINISH
  assert (installed.root / 'current').resolve() == installed.previous
  assert not (installed.root / 'updates/pending.json').exists()
  assert select_and_activate(installed.value).done
  assert (installed.root / 'current/SOURCE_COMMIT').read_text() == 'b' * 40
  assert upload(installed).kinds == [usb.BEGIN]
  assert (installed.root / 'updater/release-signing-public.pem').read_bytes() == installed.official_key


def test_unsigned_usb_does_not_bypass_official_nas_signature(installed, monkeypatch):
  from test_update_host import manifest
  calls = []
  def reject(value):
    calls.append(value)
    raise ValueError('signature required')
  monkeypatch.setattr(update, 'verify_signature', reject)
  with pytest.raises(ValueError, match='signature required'):
    update.selected_commit(manifest())
  assert len(calls) == 1


def test_official_return_uses_existing_signed_https_path(installed, monkeypatch):
  upload(installed)
  assert select_and_activate(installed.value).done
  model = b'official return model'
  bundle = installed.temporary / 'official.tar.gz'
  archive(bundle, 'c' * 40, model)
  model_path = installed.temporary / 'official.onnx'
  model_path.write_bytes(model)
  official = {'format': 1, 'source_commit': 'c' * 40, 'runtime': usb.RUNTIME, 'storage_format': 1,
              'bundle': prepare.identity(bundle, 'https://upload.shind0.synology.me/models/test/host.tar.gz'),
              'model': prepare.identity(model_path, 'https://upload.shind0.synology.me/models/test/model.onnx')}
  verified, downloads = [], []
  monkeypatch.setattr(update, 'verify_signature', lambda value: verified.append(value))
  def download(url, **_):
    downloads.append(url)
    response = io.BytesIO(bundle.read_bytes() if url.endswith('host.tar.gz') else model)
    response.url = url
    return response
  monkeypatch.setattr(update.urllib.request, 'urlopen', download)
  boot.BOOT_ID.write_text('boot-2')
  assert select_and_activate(official).done
  assert (installed.root / 'current/SOURCE_COMMIT').read_text() == 'c' * 40
  assert verified and all(value == official for value in verified)
  assert downloads == [official[k]['url'] for k in usb.KINDS]
  assert (installed.root / 'updater/release-signing-public.pem').read_bytes() == installed.official_key


@pytest.mark.parametrize('fault', ['missing', 'permissions', 'symlink', 'owner', 'boolean', 'unknown', 'oversize'])
def test_local_policy_is_required_before_allocating_or_advertising(installed, fault):
  import os
  policy = installed.root / 'updater' / usb.SHA256_POLICY
  if fault == 'missing':
    policy.unlink()
  elif fault == 'permissions':
    policy.chmod(0o666)
  elif fault == 'symlink':
    saved = policy.with_suffix('.saved')
    policy.rename(saved)
    policy.symlink_to(saved)
  elif fault == 'owner':
    os.chown(policy, 1000, 1000)
  else:
    policy.write_text({'boolean': '{"format":true,"integrity":"sha256","transport":"usb"}',
                       'unknown': '{"format":1,"integrity":"other","transport":"usb"}', 'oversize': ' ' * 257}[fault])
  assert not usb.sha256_enabled(installed.root)
  wire = Wire()
  wire.session.handle(NS(msg_type=p.Msg.HELLO_REQ, seq=1, payload=b'{}'))
  assert json.loads(wire.recv().payload)[usb.SHA256_CAPABILITY] is False
  with pytest.raises(ValueError, match='local owner policy'):
    usb.Receiver(installed.root).begin(installed.value)
  assert not (installed.root / 'usb-incoming').exists()


@pytest.mark.parametrize('fault', ['hash', 'length', 'offset', 'incomplete', 'profile_collision', 'no_space', 'probe'])
def test_failed_transfer_or_probe_preserves_current(installed, monkeypatch, fault):
  receiver = usb.Receiver(installed.root)
  value = copy.deepcopy(installed.value)
  if fault == 'hash':
    value['model']['sha256'] = '0' * 64
  if fault == 'length':
    value['model']['size'] += 1
  if fault == 'no_space':
    monkeypatch.setattr(usb.shutil, 'disk_usage', lambda _: NS(free=0))
    with pytest.raises(OSError, match='space'):
      receiver.begin(value)
    assert receiver.directory is None
    return
  if fault == 'profile_collision':
    upload(installed)
    value['source_commit'] = 'c' * 40
    with pytest.raises(ValueError, match='another release'):
      receiver.begin(value)
    return
  receiver.begin(value)
  if fault == 'offset':
    with pytest.raises(ValueError, match='offset'):
      receiver.chunk(usb.HEADER.pack(0, 1) + b'x')
  elif fault != 'incomplete':
    for index, name in enumerate(usb.NAMES):
      data = (installed.package / name).read_bytes()
      receiver.chunk(usb.HEADER.pack(index, 0) + data)
  if fault == 'probe':
    receiver.finish()
    def fail(_):
      raise RuntimeError('synthetic probe failure')
    monkeypatch.setattr(update, 'probe_release', fail)
    gate = select_and_activate(value)
    assert not gate.done and gate.state == 'failed'
    assert (installed.root / 'cache/last-loaded.json').read_text() == 'original-model'
  else:
    with pytest.raises(ValueError, match='checksum/length'):
      receiver.finish()
    assert not (installed.root / 'local-releases' / value['profile']).exists()
  receiver.close()
  assert (installed.root / 'current').resolve() == installed.previous
  assert not list((installed.root / 'usb-incoming').iterdir())


def test_interrupted_transfer_and_reconnect_restart_without_activation(installed, monkeypatch):
  wire = Wire()
  with pytest.raises(ConnectionError, match='offroad'):
    sender.upload(wire.client, installed.value, cache_assets(installed), lambda: len(wire.kinds) < 2, lambda *a: None)
  assert usb.FINISH not in wire.kinds
  wire.session.close()
  assert (installed.root / 'current').resolve() == installed.previous
  assert not (installed.root / 'local-releases').exists()
  assert usb.registered(installed.root, installed.value) is False
  upload(installed)
  assert usb.registered(installed.root, installed.value)


@pytest.mark.parametrize('bad', [{'profile': '../outside'}, {'runtime': {}}, {'format': True}, {'signature': ''},
                               {'source_commit': 123}, {'storage_format': True}])
def test_manifest_rejects_path_traversal_abi_and_extra_fields(installed, bad):
  value = {**installed.value, **bad}
  with pytest.raises(ValueError):
    usb.Receiver(installed.root).begin(value)
  assert not (installed.root / 'usb-incoming').exists()


def test_selection_is_opt_in_pins_model_and_rejects_local_conflict(installed, monkeypatch):
  path = installed.temporary / 'public/host_release.json'
  monkeypatch.setattr(jetson_model, 'read_spec', lambda: {'sha256': installed.value['model']['sha256'],
                                                       'nbytes': installed.value['model']['size']})
  assert jetson_model.selected_release(path, required=True) == installed.value
  local = path.with_name('local_host_release.json')
  local.write_text('{}')
  with pytest.raises(ValueError, match='Choose one'):
    jetson_model.selected_release(path, required=True)
  local.unlink()
  monkeypatch.setattr(jetson_model, 'read_spec', lambda: {'sha256': '0' * 64, 'nbytes': 1})
  with pytest.raises(ValueError, match='branch model'):
    jetson_model.selected_release(path, required=True)


def test_startup_checks_capability_and_offroad_before_usb_transfer(installed, monkeypatch):
  monkeypatch.setattr(startup, 'selected_release', lambda *a, **k: installed.value)
  wire = Wire()
  peer = {'carrot_host': 'jetson', startup.CAPABILITY: True, usb.SHA256_CAPABILITY: False}
  with pytest.raises(ValueError, match='locally enable'):
    startup.wait_for_boot_update(wire.client, peer, None, lambda: True, lambda *a, **k: None)
  assert not wire.kinds
  peer.update({usb.SHA256_CAPABILITY: True, 'usb_source_commit': 'a' * 40})
  values = {'IsOffroad': False, 'IsOnroad': True}
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(get_bool=values.get)))
  original = sender.upload
  monkeypatch.setattr(sender, 'upload', lambda client, value, cache, allowed, progress:
                      original(client, value, cache_assets(installed), allowed, progress))
  reports = []
  def offroad(_):
    assert not wire.kinds
    values.update(IsOffroad=True, IsOnroad=False)
  with pytest.raises(ConnectionError, match='reconnecting'):
    startup.wait_for_boot_update(wire.client, peer, None, lambda: True,
                                lambda *a, **k: reports.append(k['host_update']['state']), sleep=offroad)
  assert reports[0] == 'waiting_offroad' and 'transferring' in reports
  assert usb.registered(installed.root, installed.value)
  wire.session.close()


def test_receiver_rejects_while_activation_worker_is_running(installed):
  wire = Wire()
  wire.session.gate.worker = NS(is_alive=lambda: True)
  wire.send(usb.BEGIN, 1, [json.dumps(installed.value).encode()])
  reply = json.loads(wire.recv().payload)
  assert reply['ok'] is False and 'retained' in reply['error']
  assert not (installed.root / 'usb-incoming').exists()


def test_enrollment_preflight_preserves_unrelated_trust_and_current(installed):
  updater = installed.root / 'updater'
  before = {name: (updater / name).read_bytes() for name in enroll.FILES}
  (updater / 'boot_update.py').chmod(0o666)
  with pytest.raises(ValueError, match='root-owned'):
    enroll.install(installed.root)
  assert before == {name: (updater / name).read_bytes() for name in enroll.FILES}
  assert (updater / 'release-signing-public.pem').read_bytes() == installed.official_key
  assert (installed.root / 'current').resolve() == installed.previous


@pytest.mark.parametrize('registered', [False, True])
def test_matching_current_source_requires_read_only_registration_check(installed, monkeypatch, registered):
  if registered:
    upload(installed)
  monkeypatch.setattr(startup, 'selected_release', lambda *a, **k: installed.value)
  peer = {'carrot_host': 'jetson', startup.CAPABILITY: True, usb.SHA256_CAPABILITY: True,
          usb.CHECK_CAPABILITY: True, 'usb_source_commit': installed.value['source_commit']}
  values = {'IsOffroad': False, 'IsOnroad': True}
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(get_bool=values.get)))
  original = sender.upload
  monkeypatch.setattr(sender, 'upload', lambda client, value, cache, allowed, progress:
                      original(client, value, cache_assets(installed), allowed, progress))
  wire, reports = Wire(), []
  def offroad(_):
    assert wire.kinds == [usb.CHECK]  # No allocation before offroad, even with matching source.
    assert not (installed.root / 'usb-incoming').exists()
    values.update(IsOffroad=True, IsOnroad=False)
  with pytest.raises(ConnectionError, match='reconnecting'):
    startup.wait_for_boot_update(wire.client, peer, None, lambda: True,
                                lambda *a, **k: reports.append(k['host_update']['state']), sleep=offroad)
  if registered:
    assert wire.kinds == [usb.CHECK] and 'waiting_offroad' not in reports
  else:
    assert wire.kinds[0] == usb.CHECK and 'waiting_offroad' in reports and usb.FINISH in wire.kinds
  wire.session.close()


@pytest.mark.parametrize('fault', ['hash', 'length', 'redirect', 'interrupted'])
def test_download_failure_never_publishes_a_cache_entry(installed, monkeypatch, fault):
  item = installed.value['model']
  data = (installed.package / 'model.onnx').read_bytes()
  if fault == 'hash':
    data = b'x' * len(data)
  if fault == 'length':
    data += b'x'
  response = io.BytesIO(data)
  response.url = 'http://example.invalid/file' if fault == 'redirect' else item['url']
  monkeypatch.setattr(sender.urllib.request, 'urlopen', lambda *a, **k: response)
  cache = installed.temporary / 'failed-cache'
  checks = []
  def allowed():
    checks.append(1)
    return fault != 'interrupted' or len(checks) < 2
  with pytest.raises((ValueError, ConnectionError)):
    sender.asset(item, cache, allowed)
  assert not list(cache.iterdir())


@pytest.mark.parametrize('fault', ['sequence', 'kind', 'size', 'json'])
def test_unrelated_or_malformed_acknowledgement_cannot_complete(installed, fault):
  reply = NS(msg_type=usb.ACK, seq=1, payload=b'{"ok":true,"complete":true}')
  if fault == 'sequence':
    reply.seq = 2
  if fault == 'kind':
    reply.msg_type = p.Msg.STATE_RESP
  if fault == 'size':
    reply.payload = b' ' * 65537
  if fault == 'json':
    reply.payload = b'[]'
  client = NS(_next_seq=lambda: 1, t=NS(send=lambda *a, **k: None, recv=lambda **k: reply))
  with pytest.raises(ValueError):
    sender.available(client, installed.value, lambda: True)


def test_boot_heartbeat_does_not_rehash_large_registered_assets(installed, monkeypatch):
  upload(installed)
  monkeypatch.setattr(update, 'digest', lambda _: pytest.fail('heartbeat hashed a large asset'))
  for _ in range(20):
    assert update.selected_commit(installed.value) == installed.value['source_commit']


def test_bootstrap_copies_receiver_without_enabling_policy(installed):
  import install_updates
  image = installed.temporary / 'image'
  install_updates.bootstrap(image, Path(__file__).parent)
  updater = image / 'opt/carrot-jetlink/updater'
  assert (updater / 'usb_release.py').read_bytes() == (Path(__file__).with_name('usb_release.py')).read_bytes()
  assert not (updater / usb.SHA256_POLICY).exists()
  assert not usb.sha256_enabled(updater.parent)


def test_enrollment_retains_only_immediately_previous_helpers(installed):
  updater = installed.root / 'updater'
  before = (updater / 'boot_update.py').read_bytes() + b'\n# preceding installation\n'
  (updater / 'boot_update.py').write_bytes(before)
  enroll.install(installed.root)
  assert (updater / 'before-usb-experiments/boot_update.py').read_bytes() == before
  assert not (updater / 'before-usb-experiments/before-usb-experiments').exists()


def test_disconnect_removes_partial_transfer_without_registering(installed):
  wire = Wire()
  sender.exchange(wire.client, usb.BEGIN, installed.value, lambda: True)
  sender.exchange(wire.client, usb.CHUNK, usb.HEADER.pack(0, 0) + b'x', lambda: True, raw=True)
  incoming = wire.session.usb_receiver.directory
  assert incoming.is_dir()
  wire.session.close()
  wire.session.close()
  assert not incoming.exists()
  assert not usb.registered(installed.root, installed.value)
  assert (installed.root / 'current').resolve() == installed.previous
  assert not (installed.root / 'updates/pending.json').exists()


def test_revoked_local_policy_prevents_completed_transfer_registration(installed):
  wire = Wire()
  sender.exchange(wire.client, usb.BEGIN, installed.value, lambda: True)
  for index, name in enumerate(usb.NAMES):
    sender.exchange(wire.client, usb.CHUNK,
                    usb.HEADER.pack(index, 0) + (installed.package / name).read_bytes(), lambda: True, raw=True)
  incoming = wire.session.usb_receiver.directory
  (installed.root / 'updater' / usb.SHA256_POLICY).unlink()
  with pytest.raises(ValueError, match='rejected USB update'):
    sender.exchange(wire.client, usb.FINISH, {}, lambda: True)
  assert not incoming.exists()
  assert not usb.registered(installed.root, installed.value)
  assert (installed.root / 'current').resolve() == installed.previous


def test_onroad_transition_after_upload_does_not_request_activation(installed, monkeypatch):
  monkeypatch.setattr(startup, 'selected_release', lambda *a, **k: installed.value)
  values = {'IsOffroad': True, 'IsOnroad': False}
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(get_bool=values.get)))
  original = sender.upload
  def transition(client, value, cache, allowed, progress):
    original(client, value, cache_assets(installed), allowed, progress)
    values.update(IsOffroad=False, IsOnroad=True)
  monkeypatch.setattr(sender, 'upload', transition)
  wire = Wire()
  peer = {'carrot_host': 'jetson', startup.CAPABILITY: True, usb.SHA256_CAPABILITY: True,
          'usb_source_commit': 'a' * 40}
  with pytest.raises(ConnectionError, match='activation requires offroad'):
    startup.wait_for_boot_update(wire.client, peer, None, lambda: True, lambda *a, **k: None)
  assert usb.registered(installed.root, installed.value)
  assert wire.session.gate.selected is None
  assert not (installed.root / 'updates/pending.json').exists()
  assert (installed.root / 'current').resolve() == installed.previous
  wire.session.close()
