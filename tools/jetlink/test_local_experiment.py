"""Offline registration, local selection and official Internet restore transactions."""
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tarfile
from types import SimpleNamespace as NS

import pytest

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parents[1]), str(HERE.parents[1] / 'third_party/jetlink')]
import boot_update as boot
import install_local_experiment as installer
import local_release as local
import prepare_local_experiment as package_tool
import update_host as update
from openpilot.common import jetson_model
from openpilot.selfdrive.modeld.jetlink import startup


def archive(path, commit, model, selection=None):
  pin = 'jetson_model.json' if selection else 'cinque_v2.json'
  files = {'SOURCE_COMMIT': commit.encode(),
           'openpilot/selfdrive/modeld/jetlink/' + pin: json.dumps({'sha256': hashlib.sha256(model).hexdigest(), 'nbytes': len(model)}).encode(),
           'tools/jetlink/runtime_compatibility.json': b'{"protected_storage_format":1}'}
  if selection:
    files['openpilot/selfdrive/modeld/jetlink/local_host_release.json'] = json.dumps(selection).encode()
  for name in (*installer.HELPERS, 'install_local_experiment.py'):
    files['tools/jetlink/' + name] = (HERE / name).read_bytes()
  with tarfile.open(path, 'w:gz') as output:
    for name, data in files.items():
      member = tarfile.TarInfo(name)
      member.size, member.mode = len(data), 0o644
      output.addfile(member, io.BytesIO(data))


@pytest.fixture
def env(tmp_path, monkeypatch):
  if getattr(os, 'geteuid', lambda: -1)() != 0:
    pytest.skip('Root-owned catalog integration requires a Linux root test environment')
  runtime = tmp_path / 'runtime'
  runtime.mkdir()
  (runtime / 'updater').mkdir()
  for name in ('update_host.py', 'boot_update.py', 'release-signing-public.pem'):
    (runtime / 'updater' / name).write_bytes((HERE / name).read_bytes())
  (runtime / 'protected-runtime.json').write_text('{"format":1}')
  (runtime / 'boot-update-required').write_text('next-boot')
  previous = runtime / 'releases' / ('a' * 40)
  previous.mkdir(parents=True)
  (previous / 'SOURCE_COMMIT').write_text('a' * 40)
  (runtime / 'current').symlink_to(previous)
  (runtime / 'cache').mkdir()
  (runtime / 'cache/last-loaded.json').write_text('original-model')
  model = tmp_path / 'model.onnx'
  model.write_bytes(b'fixture-stateful-onnx')
  selection = {'format': 1, 'delivery': 'local', 'profile': 'test-resaction',
               'model': package_tool.identity(model)}
  bundle = tmp_path / 'host.tar.gz'
  archive(bundle, 'b' * 40, model.read_bytes(), selection)
  package = tmp_path / 'package'
  package_tool.prepare(bundle, model, package)
  monkeypatch.setattr(update, 'ROOT', runtime)
  monkeypatch.setattr(update, 'PROTECTED_MARKER', tmp_path / 'protected')
  monkeypatch.setattr(update, 'check_runtime', lambda _: None)
  monkeypatch.setattr(boot, 'READY', tmp_path / 'ready')
  monkeypatch.setattr(boot, 'STATUS', tmp_path / 'status')
  monkeypatch.setattr(boot, 'BOOT_ID', tmp_path / 'boot-id')
  boot.BOOT_ID.write_text('boot-1')
  monkeypatch.setattr(update.subprocess, 'run', lambda *a, **k: NS(returncode=3))
  import pwd
  monkeypatch.setattr(pwd, 'getpwnam', lambda _: NS(pw_uid=0, pw_gid=0))
  monkeypatch.setattr(update, 'probe_release', lambda release:
                      (runtime / 'cache/last-loaded.json').write_text(release.name))
  return NS(root=runtime, previous=previous, selection=selection, package=package, temporary=tmp_path)


def select_and_activate(selection):
  gate = boot.Gate()
  gate.select(selection)
  gate.tick()
  if gate.worker:
    gate.worker.join(5)
    assert not gate.worker.is_alive()
  gate.tick()
  return gate


def test_install_preserves_official_runtime_key_and_policy_and_is_idempotent(env):
  watched = ['updater/release-signing-public.pem', 'boot-update-required', 'cache/last-loaded.json']
  before = {name: (env.root / name).read_bytes() for name in watched}
  installer.install(env.package, env.root)
  installer.install(env.package, env.root)
  assert (env.root / 'current').resolve() == env.previous
  assert before == {name: (env.root / name).read_bytes() for name in watched}
  record = local.resolve(env.root, env.selection)
  local.verify_assets(record)
  assert record['source_commit'] == 'b' * 40
  assert (env.root / 'updater/before-local-experiments/boot_update.py').is_file()
  assert not (env.root / 'updates/pending.json').exists()


def test_local_to_official_internet_restore_and_back_on_next_boot(env, monkeypatch):
  installer.install(env.package, env.root)
  downloads, verified = [], []
  monkeypatch.setattr(update.urllib.request, 'urlopen', lambda *a, **k: pytest.fail('Local profile attempted HTTP'))
  assert select_and_activate(env.selection).done
  assert (env.root / 'current/SOURCE_COMMIT').read_text() == 'b' * 40
  assert boot.runtime_ready()

  # Model and bundle for a signed official selection, deliberately not cached.
  model = b'fixture-official-onnx'
  bundle = env.temporary / 'official.tar.gz'
  archive(bundle, 'c' * 40, model)
  official = {'format': 1, 'source_commit': 'c' * 40, 'runtime': local.RUNTIME, 'storage_format': 1,
              'bundle': {**package_tool.identity(bundle), 'url': 'https://upload.shind0.synology.me/models/official/host.tar.gz'},
              'model': {'sha256': hashlib.sha256(model).hexdigest(), 'size': len(model),
                        'url': 'https://upload.shind0.synology.me/models/official/model.onnx'}}
  monkeypatch.setattr(update, 'verify_signature', lambda value: verified.append(value))
  class Response(io.BytesIO):
    def __init__(self, url):
      downloads.append(url)
      self.url = url
      super().__init__(bundle.read_bytes() if url.endswith('host.tar.gz') else model)
  monkeypatch.setattr(update.urllib.request, 'urlopen', lambda url, **k: Response(url))
  boot.BOOT_ID.write_text('boot-2')
  assert not boot.runtime_ready()
  assert select_and_activate(official).done
  assert (env.root / 'current/SOURCE_COMMIT').read_text() == 'c' * 40
  assert downloads == [official['bundle']['url'], official['model']['url']]
  assert verified and all(value == official for value in verified)
  assert (env.root / 'cache/last-loaded.json').read_text() == 'c' * 40
  boot.BOOT_ID.write_text('boot-3')
  assert select_and_activate(env.selection).done
  assert (env.root / 'current/SOURCE_COMMIT').read_text() == 'b' * 40
  assert len(downloads) == 2  # returning to the local experiment is offline


def test_failed_experiment_probe_preserves_official_release_and_model(env, monkeypatch):
  installer.install(env.package, env.root)
  def reject(_):
    (env.root / 'cache/last-loaded.json').write_text('failed-candidate')
    raise RuntimeError('Synthetic inference failure')
  monkeypatch.setattr(update, 'probe_release', reject)
  gate = select_and_activate(env.selection)
  assert not gate.done and gate.state == 'failed'
  assert (env.root / 'current').resolve() == env.previous
  assert (env.root / 'cache/last-loaded.json').read_text() == 'original-model'


@pytest.mark.parametrize('failure', ['missing', 'model', 'writable', 'symlink'])
def test_unregistered_or_corrupt_local_assets_cannot_become_pending(env, failure):
  if failure != 'missing':
    installer.install(env.package, env.root)
    directory = env.root / 'local-releases/test-resaction'
    if failure == 'model':
      (directory / 'model.onnx').write_bytes(b'wrong model')
    elif failure == 'writable':
      (directory / 'record.json').chmod(0o666)
    elif failure == 'symlink':
      (directory / 'record.json').rename(directory / 'original.json')
      (directory / 'record.json').symlink_to(directory / 'original.json')
  with pytest.raises((ValueError, FileNotFoundError)):
    update.stage_manifest(env.selection)
  assert not (env.root / 'updates/pending.json').exists()
  assert (env.root / 'current').resolve() == env.previous


@pytest.mark.parametrize('failure', ['model', 'key'])
def test_invalid_package_or_different_trust_root_leaves_runtime_untouched(env, failure):
  if failure == 'model':
    (env.package / 'model.onnx').write_bytes(b'corrupt')
  else:
    (env.root / 'updater/release-signing-public.pem').write_bytes(b'other trust root')
  with pytest.raises(ValueError):
    installer.install(env.package, env.root)
  assert not (env.root / 'local-releases').exists()
  assert (env.root / 'current').resolve() == env.previous


@pytest.mark.parametrize('change', [{'profile': '../escape'}, {'url': 'https://example.invalid/bundle'}, {'format': True}])
def test_network_peer_cannot_register_or_name_paths(change):
  value = {'format': 1, 'delivery': 'local', 'profile': 'resaction-v1', 'model': {'sha256': 'b' * 64, 'size': 12}}
  value.update(change)
  with pytest.raises(ValueError):
    local.validate_selection(value)


def test_client_selects_local_profile_and_stock_manifest_still_verifies(tmp_path, monkeypatch):
  stock = HERE.parents[1] / 'openpilot/selfdrive/modeld/jetlink/host_release.json'
  path = tmp_path / 'host_release.json'
  path.write_bytes(stock.read_bytes())
  spec = jetson_model.read_spec()
  local_selection = {'format': 1, 'delivery': 'local', 'profile': 'fixture-nav',
                     'model': {'sha256': spec['sha256'], 'size': spec['nbytes']}}
  path.with_name('local_host_release.json').write_text(json.dumps(local_selection))
  monkeypatch.setattr(startup, 'RELEASE', path)
  selection = jetson_model.selected_release(path, required=True)
  assert selection['delivery'] == 'local'
  assert 'url' not in json.dumps(selection)
  update.verify_signature(json.loads(path.read_text()))
  calls = []
  client = NS(t=NS(send_json=lambda *a: calls.append(a[-1])), _next_seq=lambda: 1,
              state=lambda: {'carrot_update': {'state': 'ready'}})
  peer = {'carrot_host': 'jetson', startup.CAPABILITY: True, boot.LOCAL_CAPABILITY: True}
  with pytest.raises(ConnectionError, match='reconnecting'):
    startup.wait_for_boot_update(client, peer, None, lambda: True, lambda *a, **k: None)
  assert calls == [selection]
