import io
import json
from pathlib import Path
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from refresh_sd_image import audit_unprovisioned, refresh


@pytest.fixture
def base(tmp_path):
  root, setup = tmp_path/'root', tmp_path/'setup'
  (root/'etc').mkdir(parents=True)
  setup.mkdir()
  (root/'etc/machine-id').write_text('')
  (root/'etc/shadow').write_text('root:!:::::::\njetlink:!:::::::\n')
  return root, setup


@pytest.mark.parametrize('name', ['etc/ssh/ssh_host_ed25519_key', 'home/jetlink/.ssh/authorized_keys',
                                  'etc/NetworkManager/system-connections/personal',
                                  'var/lib/carrot-jetlink/provisioned.json',
                                  'var/lib/carrot-jetlink/root-expanded.json'])
def test_reject_personally_used_image(base, name):
  root, setup = base
  path = root/name
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text('synthetic')
  with pytest.raises(ValueError):
    audit_unprovisioned(root, setup)


@pytest.mark.parametrize('name', ['setup.json', 'SETUP-RESULT.json'])
def test_reject_setup_state(base, name):
  root, setup = base
  (setup/name).write_text('{}')
  with pytest.raises(ValueError):
    audit_unprovisioned(root, setup)


def test_reject_machine_identity_and_password(base):
  root, setup = base
  (root/'etc/machine-id').write_text('a' * 32)
  with pytest.raises(ValueError):
    audit_unprovisioned(root, setup)
  (root/'etc/machine-id').write_text('')
  (root/'etc/shadow').write_text('jetlink:$6$synthetic:::::::\n')
  with pytest.raises(ValueError):
    audit_unprovisioned(root, setup)


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux image symlink and ownership semantics')
def test_refresh_selects_committed_release_and_repairs_hud(base, tmp_path, monkeypatch):
  root, setup = base
  runtime = root/'opt/carrot-jetlink'
  (runtime/'releases/old').mkdir(parents=True)
  (runtime/'current').symlink_to('releases/old')
  (runtime/'carrot').symlink_to('current')
  (root/'etc/carrot-jetlink-image.json').write_text(json.dumps({'source_commit': 'old', 'root_start': 3188736}))
  bundle = tmp_path/'host.tgz'
  commit = 'a' * 40
  with tarfile.open(bundle, 'w:gz') as t:
    data = (commit + '\n').encode()
    member = tarfile.TarInfo('SOURCE_COMMIT')
    member.size = len(data)
    t.addfile(member, io.BytesIO(data))
    for name in ('update_host.py', 'finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem',
                 'wifi_protocol.py', 'boot_update.py'):
      t.add(Path(__file__).with_name(name), arcname='tools/jetlink/' + name)
  import install_boot_display
  diagnostic_calls = []
  monkeypatch.setattr(install_boot_display, 'configure', lambda r, s: diagnostic_calls.append((r, s)))
  marker = refresh(root, setup, bundle)
  assert diagnostic_calls == [(root, runtime/'releases'/commit/'tools/jetlink')]
  assert marker['state'] == 'CANDIDATE_PHYSICAL_BOOT_PENDING'
  assert marker['root_start'] == 3188736
  assert (runtime/'current/SOURCE_COMMIT').read_text().strip() == commit
  assert (runtime/'updater/boot_update.py').read_bytes() == Path(__file__).with_name('boot_update.py').read_bytes()
  assert (runtime/'updater/wifi_protocol.py').read_bytes() == Path(__file__).with_name('wifi_protocol.py').read_bytes()
  assert (runtime/'boot-update-required').read_text() == 'next-boot\n'
  assert not (runtime/'releases/old').exists()
  assert (root/'var/spool/anacron').is_dir() and (root/'etc/openvpn').is_dir()
  assert 'WorkingDirectory=/var/log/carrot-jetlink-hud' in (
    root/'etc/systemd/system/carrot-jetlink-hud.service.d/log-directory.conf').read_text()
  assert json.loads((setup/'IMAGE.json').read_text()) == marker
  audit_unprovisioned(root, setup)
