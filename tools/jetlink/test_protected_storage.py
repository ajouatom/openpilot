import base64
import json
from pathlib import Path
import sys
import os
import subprocess

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import persistent_state as store
from install_protected import configure
from build_protected_image import validate_base


def files(host='carrot-test'):
  return {'etc/hostname': base64.b64encode((host + '\n').encode()).decode(),
          'etc/NetworkManager/system-connections/example.nmconnection': base64.b64encode(b'synthetic secret').decode()}


def test_unchanged_state_does_not_write_flash(tmp_path, monkeypatch):
  dirs = [tmp_path / 'data', tmp_path / 'backup']
  assert store.save(dirs, files()) == 1
  monkeypatch.setattr(store, 'durable_write', lambda *a: pytest.fail('unchanged state rewritten'))
  assert store.save(dirs, files()) == 1
  assert store.latest(dirs)['files'] == files()


def test_torn_latest_record_recovers_previous_and_separate_partition(tmp_path):
  dirs = [tmp_path / 'data', tmp_path / 'backup']
  store.save(dirs, files('old'))
  store.save(dirs, files('new'))
  (dirs[0] / 'identity-0.json').write_bytes(b'{torn')
  assert store.latest(dirs)['files'] == files('new')
  (dirs[1] / 'identity-0.json').write_bytes(b'{torn')
  assert store.latest(dirs)['files'] == files('old')


@pytest.mark.parametrize('stage', ['before', 'after'])
def test_power_cut_during_save_always_leaves_a_valid_generation(tmp_path, monkeypatch, stage):
  dirs = [tmp_path]
  store.save(dirs, files('old'))
  original = store.durable_write
  def interrupted(*args):
    if stage == 'after':
      original(*args)
    raise OSError('simulated power cut')
  monkeypatch.setattr(store, 'durable_write', interrupted)
  with pytest.raises(OSError):
    store.save(dirs, files('new'))
  assert store.latest(dirs)['files'] == files('new' if stage == 'after' else 'old')


def test_bad_hash_and_unexpected_paths_are_not_restored(tmp_path):
  store.save([tmp_path], files())
  path = tmp_path / 'identity-0.json'
  value = json.loads(path.read_text())
  value['body']['files']['etc/hostname'] = base64.b64encode(b'changed').decode()
  path.write_text(json.dumps(value))
  assert store.read_record(path) is None
  for name in ('../escape', 'etc/shadow', 'etc/systemd/system/malicious.service', '/etc/hosts'):
    with pytest.raises(ValueError):
      store.validate_files({name: 'eA=='})


def test_restore_identity_does_not_modify_os_or_serialize_machine_id(tmp_path):
  store.restore(tmp_path, dict(files=files()))
  (tmp_path / 'etc/machine-id').write_text('a' * 32)
  assert store.collect(tmp_path) == files()
  assert (tmp_path / 'etc/hostname').read_text() == 'carrot-test\n'


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux symlink semantics')
@pytest.mark.parametrize('target', ['releases/updated', '/opt/carrot-jetlink/releases/updated'])
def test_updated_data_runtime_is_resolved_before_bind_mount(tmp_path, target):
  from protected_storage import valid_runtime
  release = tmp_path / 'releases/updated'
  for name in ('tools/jetlink/server.py', 'SOURCE_COMMIT'):
    p = release / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text('synthetic')
  for name in ('venv/bin/python', 'cache/last-loaded.json'):
    p = tmp_path / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text('synthetic')
  (tmp_path / 'protected-runtime.json').write_text('{"format":1}')
  (tmp_path / 'current').symlink_to(target)
  assert valid_runtime(tmp_path)
  (tmp_path / 'current').unlink()
  (tmp_path / 'current').symlink_to('/etc')
  assert not valid_runtime(tmp_path)


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux symlink semantics')
def test_protected_image_wiring_is_offline_only_and_network_independent(tmp_path, monkeypatch):
  import initrd_readonly
  monkeypatch.setattr(initrd_readonly, 'configure', lambda root: None)
  with pytest.raises(ValueError):
    configure(Path('/'), Path(__file__).parent)
  (tmp_path / 'etc').mkdir()
  (tmp_path / 'etc/carrot-jetlink-image.json').write_text('{}')
  boot = tmp_path / 'boot/extlinux/extlinux.conf'
  boot.parent.mkdir(parents=True)
  boot.write_text('LABEL primary\n  APPEND root=/dev/mmcblk0p1 rw rootwait\n')
  configure(tmp_path, Path(__file__).parent)
  assert ' rw' not in boot.read_text() and ' ro' in boot.read_text()
  assert '/dev/root / ext4 ro,noload 0 0' in (tmp_path / 'etc/fstab').read_text()
  wifi = (tmp_path / 'etc/systemd/system/carrot-jetlink-wifi.service').read_text()
  assert 'update-apply' not in wifi
  assert 'ExecStart=/usr/bin/python3 /usr/local/lib/carrot-jetlink/network/current/wifi_apply.py' in wifi
  assert (tmp_path / 'usr/lib/carrot-jetlink-storage/wifi_apply.py').is_file()
  assert 'Legacy APP growth disabled' in (tmp_path / 'etc/systemd/system/carrot-image-grow.service').read_text()


def test_legacy_expanded_card_and_unexpected_layout_cannot_be_converted():
  parts = [{'name': 'boot', 'start': i * 100, 'size': 50} for i in range(14)]
  parts += [{'name': 'CARROT_SETUP', 'start': 3057664, 'size': 131072},
            {'name': 'APP', 'start': 3188736, 'size': 40000000}]
  layout = dict(label='gpt', sectorsize=512, partitions=parts)
  validate_base(layout, 24 * (1 << 30))
  with pytest.raises(ValueError):
    validate_base(layout, 128 * (1 << 30))
  parts[-1]['start'] += 2048
  with pytest.raises(ValueError):
    validate_base(layout, 24 * (1 << 30))


@pytest.mark.skipif(os.environ.get('CARROT_STORAGE_MOUNTS') != '1', reason='Explicit isolated Linux root integration run required')
def test_real_readonly_filesystem_survives_runtime_writes_and_second_boot(tmp_path):
  import hashlib
  from protected_storage import mount_volatile
  def run(*args):
    return subprocess.check_output(list(map(str, args)), text=True).strip()
  image = tmp_path / 'synthetic-root.ext4'
  with image.open('wb') as output:
    output.truncate(64 * (1 << 20))
  root, ram = tmp_path / 'root', tmp_path / 'ram'
  root.mkdir()
  run('mkfs.ext4', '-q', '-F', '-m', '0', image)
  loop = run('losetup', '--find', '--show', image)
  try:
    assert Path(run('losetup', '-n', '-O', 'BACK-FILE', loop)).resolve() == image.resolve()
    run('mount', loop, root)
    for name in ('etc', 'var/log', 'home/jetlink', 'root', 'tmp', 'usr', 'opt', 'mnt', 'lib/firmware'):
      (root / name).mkdir(parents=True, exist_ok=True)
    (root / 'etc/machine-id').write_text('a' * 32 + '\n')
    (root / 'usr/base.txt').write_text('immutable OS')
    (root / 'lib/firmware/pva_auth_allowlist').write_bytes(b'baseline-authentication')
    (root / 'root').chmod(0o700)
    run('umount', root)
    before = hashlib.sha256(image.read_bytes()).hexdigest()
    run('blockdev', '--setro', loop)
    for boot in range(2):
      run('mount', '-o', 'ro', loop, root)
      try:
        mount_volatile(root, ram)
        assert not (root / 'etc/temporary.conf').exists()
        (root / 'etc/temporary.conf').write_text('RAM only')
        (root / 'var/log/log.log').write_text('vendor log')
        assert (root / 'lib/firmware/pva_auth_allowlist').read_bytes() == b'baseline-authentication'
        (root / 'lib/firmware/pva_auth_allowlist').write_bytes(b'regenerated-authentication')
        (root / 'mnt/nvidia-temporary').mkdir()
        (root / 'home/jetlink/.cache').mkdir()
        assert (root / 'root').stat().st_mode & 0o777 == 0o700
        with pytest.raises(OSError):
          (root / 'usr/base.txt').write_text('must fail')
        run('sync')
        assert hashlib.sha256(image.read_bytes()).hexdigest() == before
      finally:
        run('umount', '-R', root)
        run('umount', ram)
    assert hashlib.sha256(image.read_bytes()).hexdigest() == before
  finally:
    subprocess.run(['umount', '-R', str(root)], capture_output=True)
    subprocess.run(['umount', str(ram)], capture_output=True)
    run('blockdev', '--setrw', loop)
    run('losetup', '-d', loop)
