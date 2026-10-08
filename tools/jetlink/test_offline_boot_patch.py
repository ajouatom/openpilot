import base64
import copy
import hashlib
import io
from contextlib import contextmanager
from pathlib import Path
import zipfile

import pytest

import offline_bootstrap
from offline_boot_patch import select, validate_index
from offline_hotfix import apply


def sha(body):
  return hashlib.sha256(body).hexdigest()


def fixture():
  # Distinct APP identities, common first-boot allocation; identity/data outside it.
  base = bytearray(b'G' * 512 + b'A' * 512 + b'B' * 512 + b'P' * 512)
  profiles = []
  for name, tail, replacement in [('original', b'B', b'N'), ('portable', b'C', b'V')]:
    before, after = b'A' * 512, replacement * 512
    profiles.append({'format': 2, 'release': name, 'root_offset': 512, 'root_bytes': 1024, 'image_bytes': len(base),
      'root_sha256': sha(before + tail * 512), 'partition_guard': {'offset': 0, 'data': base64.b64encode(base[:512]).decode()},
      'patches': [{'offset': 512, 'before': base64.b64encode(before).decode(), 'after': base64.b64encode(after).decode(),
                    'before_sha256': sha(before), 'after_sha256': sha(after)}]})
  return base, {'format': 'carrot-offline-boot-v1', 'payload_sha256': 'a' * 64, 'payload_bytes': 123, 'profiles': profiles}


@pytest.mark.parametrize('variant', [0, 1])
def test_exact_selection_readback_retry_and_identity_preservation(variant):
  data, index = fixture()
  if variant:
    data[1024:1536] = b'C' * 512
  stream = io.BytesIO(data)
  selected = select(stream, index, report=lambda _: None)
  assert selected is index['profiles'][variant]
  assert not apply(stream, selected, verify_only=True, report=lambda _: None)
  assert stream.getvalue() == data
  apply(stream, selected, report=lambda _: None)
  assert stream.getvalue()[:512] == data[:512] and stream.getvalue()[1024:] == data[1024:]
  assert select(stream, index, report=lambda _: None) is selected
  assert apply(stream, selected, verify_only=True, report=lambda _: None)
  stream.seek(512)
  stream.write(b'X' * 200)  # Interrupted patch bytes normalize only inside the approved allocation.
  apply(stream, select(stream, index, report=lambda _: None), report=lambda _: None)
  assert apply(stream, selected, verify_only=True, report=lambda _: None)


@pytest.mark.parametrize('offset', [0, 1024])
def test_other_disk_or_modified_app_rejected_without_writes(offset):
  data, index = fixture()
  data[offset] = ord('X')
  stream = io.BytesIO(data)
  with pytest.raises(RuntimeError, match='Unsupported'):
    select(stream, index, report=lambda _: None)
  assert stream.getvalue() == data


def test_profile_ambiguity_and_mismatched_normalization_rejected():
  _, index = fixture()
  bad = copy.deepcopy(index)
  bad['profiles'][1]['root_sha256'] = bad['profiles'][0]['root_sha256']
  with pytest.raises(ValueError, match='Ambiguous'):
    validate_index(bad)
  bad = copy.deepcopy(index)
  bad['profiles'][1]['root_bytes'] += 512
  with pytest.raises(ValueError, match='normalization'):
    validate_index(bad)


def test_installer_enables_once_and_preserves_newer_updater_and_identity(tmp_path, monkeypatch):
  import install_updates
  root = tmp_path
  (root / 'run').mkdir()
  (root / 'run/carrot-storage.json').write_text('{"state":"protected"}')
  source = root / 'source'
  source.mkdir()
  names = ('boot_update.py', 'update_host.py', 'wifi_protocol.py', 'hud_protocol.py',
           'finalize_sd_image.py', 'release-signing-public.pem')
  for name in names:
    (source / name).write_text(name)
  identity = root / 'identity.json'
  identity.write_text('preserve')
  monkeypatch.setattr(install_updates.os, 'sync', lambda: None, raising=False)
  offline_bootstrap.install(source, root)
  runtime = root / 'opt/carrot-jetlink'
  assert (runtime / 'boot-update-required').read_text() == 'next-boot\n'
  assert (root / 'run/carrot-offline-bootstrap-ready').is_file()
  updated = runtime / 'updater/update_host.py'
  updated.write_text('newer release')
  offline_bootstrap.install(source, root)
  assert updated.read_text() == 'newer release' and identity.read_text() == 'preserve'


def test_installer_refuses_recovery_or_partial_helpers(tmp_path, monkeypatch):
  (tmp_path / 'run').mkdir()
  status = tmp_path / 'run/carrot-storage.json'
  status.write_text('{"state":"base-recovery"}')
  with pytest.raises(RuntimeError, match='DATA'):
    offline_bootstrap.install(tmp_path, tmp_path)
  status.write_text('{"state":"protected"}')
  runtime = tmp_path / 'opt/carrot-jetlink'
  runtime.mkdir(parents=True)
  (runtime / 'boot-update-required').write_text('already enabled')
  with pytest.raises(RuntimeError, match='incomplete'):
    offline_bootstrap.install(tmp_path, tmp_path)
  assert not (tmp_path / 'run/carrot-offline-bootstrap-ready').exists()


def test_old_runtime_guards_wait_without_timeout_and_depend_on_setup(tmp_path, monkeypatch):
  calls = []
  monkeypatch.setattr(offline_bootstrap.subprocess, 'run', lambda args, **kw: calls.append(args))
  offline_bootstrap.guard_units(tmp_path)
  for name in ('carrot-jetlink', 'carrot-jetlink-hud'):
    body = (tmp_path / (name + '.service.d/offline-bootstrap.conf')).read_text()
    assert 'Requires=carrot-image-setup.service carrot-jetlink-update-apply.service' in body
    assert 'TimeoutStartSec=0' in body and 'wait_for_runtime()' in body
    assert 'ExecStartPre=/usr/bin/test -f /run/carrot-offline-bootstrap-ready' in body
  assert calls == [['systemctl', 'daemon-reload']]


def test_signed_runtime_still_migrates_once_without_reboot(tmp_path, monkeypatch):
  import install_updates
  source = tmp_path / 'current/tools/jetlink'
  source.mkdir(parents=True)
  (source / 'boot_update.py').write_text('installed')
  calls = []
  monkeypatch.setattr(install_updates.subprocess, 'run', lambda args, **kw: calls.append(args))
  install_updates.migrate_running_release(tmp_path)
  assert calls[0][:2] == ['sudo', '-n'] and calls[0][-1] == '--bootstrap-only'
  (tmp_path / 'boot-update-required').write_text('this-boot')
  install_updates.migrate_running_release(tmp_path)
  assert len(calls) == 1


def test_retired_hold_cannot_strand_updated_comma():
  root = Path(__file__).resolve().parents[2]
  params = (root / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')
  assert '{"JetsonLegacyUpdatePending", {CLEAR_ON_MANAGER_START, BOOL}}' in params
  for name in ('openpilot/system/hardware/hardwared.py', 'openpilot/selfdrive/modeld/jetlink/daemon.py'):
    text = (root / name).read_text(encoding='utf-8')
    assert 'JETSON_UPDATE_PENDING' not in text and 'wait_for_legacy_update' not in text
  assert 'wait_for_boot_update' in (root / 'openpilot/selfdrive/modeld/jetlink/daemon.py').read_text(encoding='utf-8')
  assert 'jetsonUpdateCard' not in (root / 'openpilot/selfdrive/carrot/web/index.html').read_text(encoding='utf-8')


@pytest.fixture
def boot_payload(tmp_path, monkeypatch):
  import offline_boot_hook as hook
  import protected_storage
  root = tmp_path
  (root / 'run').mkdir()
  (root / 'run/carrot-storage.json').write_text('{"state":"protected"}')
  setup = root / 'setup'
  setup.mkdir()
  body = io.BytesIO()
  with zipfile.ZipFile(body, 'w') as archive:
    for name in sorted(hook.NAMES):
      archive.writestr(name, (Path(__file__).parent / name).read_bytes())
  data = body.getvalue()
  (setup / 'carrot-boot-update.zip').write_bytes(data)
  monkeypatch.setattr(hook, 'PAYLOAD_SHA', sha(data))
  monkeypatch.setattr(hook, 'PAYLOAD_BYTES', len(data))
  @contextmanager
  def mounted():
    yield setup / 'identity'
  monkeypatch.setattr(protected_storage, 'setup_mount', mounted)
  calls = []
  monkeypatch.setattr(hook.subprocess, 'run', lambda args, **kw: calls.append(args))
  return hook, root, setup, calls


def test_verified_payload_is_cached_and_later_boot_needs_no_pc_payload(boot_payload):
  hook, root, setup, calls = boot_payload
  hook.offline_bootstrap(root)
  assert calls[-1][-1].endswith('offline_bootstrap.py')
  folder = root / 'opt/carrot-jetlink/offline-bootstrap' / hook.PAYLOAD_SHA
  copied = folder / 'boot_update.py'
  original_mtime = copied.stat().st_mtime_ns
  (setup / 'carrot-boot-update.zip').unlink()
  hook.offline_bootstrap(root)
  assert copied.stat().st_mtime_ns == original_mtime
  guard = root / 'run/systemd/system/carrot-jetlink.service.d/offline-bootstrap.conf'
  assert 'wait_for_runtime()' in guard.read_text()


def test_corrupt_payload_blocks_old_runtime_before_install(boot_payload):
  hook, root, setup, calls = boot_payload
  (setup / 'carrot-boot-update.zip').write_bytes(b'corrupt')
  with pytest.raises(RuntimeError, match='verification'):
    hook.offline_bootstrap(root)
  assert calls == [['systemctl', 'daemon-reload']]
  assert not (root / 'run/carrot-offline-bootstrap-ready').exists()
  guard = root / 'run/systemd/system/carrot-jetlink.service.d/offline-bootstrap.conf'
  assert 'Requires=carrot-image-setup.service' in guard.read_text()
  assert 'ExecStartPre=/usr/bin/test -f /run/carrot-offline-bootstrap-ready' in guard.read_text()


def test_recovery_storage_never_installs_or_blocks_baseline(boot_payload):
  hook, root, setup, calls = boot_payload
  (root / 'run/carrot-storage.json').write_text('{"state":"base-recovery"}')
  hook.offline_bootstrap(root)
  assert calls == [] and not (root / 'run/systemd/system').exists()
