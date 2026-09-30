import base64
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from build_sd_nvme_patch import APP_UUID, Overlay, fit_text, packed_python
from offline_hotfix import apply, load_manifest, validate
from portable_initrd import RESOLVE, patch
import protected_storage as storage


@pytest.mark.parametrize('disk', ['mmcblk0', 'mmcblk1', 'nvme0n1', 'nvme1n1'])
def test_resolve_all_partitions_from_mounted_root_not_other_media(tmp_path, monkeypatch, disk):
  partition = tmp_path / disk / (disk + 'p1')
  partition.mkdir(parents=True)
  (partition / 'partition').write_text('1')
  original = Path.resolve
  sys_root = tmp_path / 'sys'
  monkeypatch.setattr(Path, 'resolve', lambda self, **kw: partition if self == sys_root / '259:1' else original(self, **kw))
  calls = []
  def run(*args):
    calls.append(args)
    if args == ('findmnt', '-n', '-o', 'MAJ:MIN', '/'):
      return '259:1'
    if args == ('blkid', '-s', 'PARTUUID', '-o', 'value', '/dev/' + disk + 'p1'):
      return APP_UUID
    pytest.fail(str(args))
  monkeypatch.setattr(storage, 'run', run)
  config = storage.portable_configuration(sys_root)
  assert config == dict(format=2, root=f'/dev/{disk}p1', data=f'/dev/{disk}p17', setup=f'/dev/{disk}p16', efi=f'/dev/{disk}p10')
  assert len(calls) == 2  # no global DATA/SETUP label resolution


@pytest.mark.parametrize('disk,part', [('sda', 'sda1'), ('loop0', 'loop0p1'), ('nvme0n1', 'nvme0n1p2'), ('dm-0', 'dm-0p1')])
def test_wrong_root_kind_or_partition_is_rejected(tmp_path, monkeypatch, disk, part):
  partition = tmp_path / disk / part
  partition.mkdir(parents=True)
  (partition / 'partition').write_text('1')
  monkeypatch.setattr(Path, 'resolve', lambda *a, **kw: partition)
  monkeypatch.setattr(storage, 'run', lambda *a: '259:1')
  with pytest.raises(ValueError):
    storage.portable_configuration(tmp_path)


def test_legacy_layout_is_unchanged_and_unrecognized_markers_rejected(tmp_path, monkeypatch):
  marker = tmp_path / 'marker'
  monkeypatch.setattr(storage, 'MARKER', marker)
  legacy = dict(format=1, root='/dev/mmcblk0p1', data='/dev/mmcblk0p17', setup='/dev/mmcblk0p16')
  marker.write_text(json.dumps(legacy))
  monkeypatch.setattr(storage, 'portable_configuration', lambda: pytest.fail('legacy changed'))
  assert storage.configuration() == legacy
  marker.write_text('{"format":2,"layout":"unknown"}')
  with pytest.raises(ValueError):
    storage.configuration()


def test_bad_setup_does_not_prevent_independent_recovery(tmp_path, monkeypatch):
  monkeypatch.setattr(storage, 'configuration', lambda: dict(format=2, setup='/dev/nvme0n1p16'))
  monkeypatch.setattr(storage, 'SETUP', tmp_path)
  monkeypatch.setattr(storage, 'run', lambda *a: 'WRONG')
  with storage.setup_mount() as backup:
    assert backup is None


@pytest.mark.skipif(os.name == 'nt', reason='Linux absolute symlink semantics')
@pytest.mark.parametrize('disk', ['mmcblk0', 'nvme0n1'])
@pytest.mark.parametrize('data_ok', [True, False])
def test_boot_keeps_root_readonly_and_uses_same_disk_data(tmp_path, monkeypatch, disk, data_ok):
  from contextlib import contextmanager
  config = dict(format=2, root=f'/dev/{disk}p1', data=f'/dev/{disk}p17', setup=f'/dev/{disk}p16', efi=f'/dev/{disk}p10')
  monkeypatch.setattr(storage, 'configuration', lambda: config)
  monkeypatch.setattr(storage, 'DATA', tmp_path / 'data')
  monkeypatch.setattr(storage, 'STATUS', tmp_path / 'status')
  monkeypatch.setattr(storage, 'stage', lambda *a: None)
  monkeypatch.setattr(storage, 'mount_volatile', lambda *a: None)
  monkeypatch.setattr(storage, 'valid_runtime', lambda *a: True)
  monkeypatch.setattr(storage, 'latest', lambda *a: None)
  monkeypatch.setattr(storage, 'durable_write', lambda *a: None)
  @contextmanager
  def setup(*a, **kw):
    yield None
  monkeypatch.setattr(storage, 'setup_mount', setup)
  original_path = storage.Path
  monkeypatch.setattr(storage, 'Path', lambda value: tmp_path / 'efi' if value == '/run/carrot-efi' else original_path(value))
  commands = []
  def run(*args, **kw):
    commands.append(args)
    if args == ('findmnt', '-n', '-o', 'SOURCE', '/'):
      return config['root']
    if args == ('findmnt', '-n', '-o', 'OPTIONS', '/'):
      return 'ro,noload'
    if args == ('blkid', '-s', 'LABEL', '-o', 'value', config['data']):
      return 'CARROTDATA' if data_ok else 'WRONG'
    return ''
  monkeypatch.setattr(storage, 'run', run)
  monkeypatch.setattr(storage.subprocess, 'run', lambda cmd, **kw: subprocess.CompletedProcess(cmd, 1 if cmd[0] == 'findmnt' else 0))
  storage.boot()
  assert ('blockdev', '--setro', config['root']) in commands
  assert (tmp_path / 'efi').readlink().as_posix() == config['efi']
  mounts = [cmd for cmd in commands if cmd[:2] == ('mount', '-t')]
  assert len(mounts) == int(data_ok)
  if data_ok:
    assert mounts[0][-2] == config['data']


@pytest.mark.skipif(os.name == 'nt' or not shutil.which('bash'), reason='Execute Linux initrd shell logic')
@pytest.mark.parametrize('matches,expected', [('/dev/mmcblk0p1', 'mmcblk0p1'), ('/dev/nvme0n1p1', 'nvme0n1p1'),
                                           ('/dev/nvme1n1p1', 'nvme1n1p1'), ('', None),
                                           ('/dev/mmcblk0p1\n/dev/nvme0n1p1', None),
                                           ('/dev/sda1', None), ('/dev/nvme0n1p2', None)])
def test_actual_initrd_shell_selects_one_supported_partition(matches, expected):
  stubs = '''blkid() { printf '%s\\n' "$MATCHES"; }
readlink() { printf '%s\\n' "${@: -1}"; }
sleep() { :; }
exec() { exit 77; }
'''
  script = stubs + RESOLVE.replace('/dev/kmsg', '/dev/null') + '\nprintf "ROOT=%s" "$rootdev"\n'
  result = subprocess.run(['bash', '-c', script], env=dict(os.environ, MATCHES=matches), capture_output=True, text=True)
  assert result.returncode == (0 if expected else 77)
  if expected:
    assert result.stdout == 'ROOT=' + expected


def test_unknown_initrd_and_oversized_replacements_rejected():
  with pytest.raises(ValueError):
    patch(b'unknown init')
  with pytest.raises(ValueError):
    fit_text(b'long', 3)


def test_packed_python_is_exact_source_and_preserves_module_globals():
  source = b'ANSWER=42\ndef answer():\n return ANSWER\n'
  result = packed_python(source, 2048)
  ns = {'__name__': 'test', '__file__': '/synthetic.py'}
  exec(result, ns)
  assert ns['answer']() == 42 and len(result) == 2048


def test_large_patch_compressed_manifest_and_virtual_reads(tmp_path):
  from test_offline_hotfix import package
  original = b'A' * (512 << 10)
  replacement = b'N' * (128 << 10)
  manifest = package(original, replacement=replacement)
  with pytest.raises(ValueError):
    validate(manifest)  # format 1 retains its original 64 KiB limit
  manifest['format'] = 2
  path = tmp_path / 'patch.json.gz'
  path.write_bytes(gzip.compress(json.dumps(manifest).encode()))
  loaded = load_manifest(path, hashlib.sha256(path.read_bytes()).hexdigest())
  with pytest.raises(RuntimeError):
    load_manifest(path, '0' * 64)
  patches = validate(loaded)
  stream = io.BytesIO(original)
  overlay = Overlay(stream, patches)
  expected = original[:4096] + replacement + original[4096+len(replacement):]
  assert overlay.read() == expected
  assert stream.getvalue() == original
  assert apply(stream, loaded)
  assert stream.getvalue() == expected
  assert apply(stream, loaded)  # repeated physical patch is harmless
