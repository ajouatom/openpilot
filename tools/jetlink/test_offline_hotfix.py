import base64
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from build_offline_hotfix import BOOT_SHA, bootstrap
from offline_hotfix import apply, validate


def sha(data):
  return hashlib.sha256(data).hexdigest()


def package(data, offset=4096, replacement=b'N' * 512, start=2048):
  original = data[offset:offset + len(replacement)]
  return dict(format=1, root_offset=start, root_bytes=len(data)-start,
              root_sha256=sha(data[start:]), partition_guard=dict(offset=1024, data=base64.b64encode(data[1024:1536]).decode()),
              patches=[dict(offset=offset, before=base64.b64encode(original).decode(),
                            after=base64.b64encode(replacement).decode(), before_sha256=sha(original), after_sha256=sha(replacement))])


def test_only_patch_sectors_change_and_retry_is_idempotent():
  original = b'A' * 16384
  disk = io.BytesIO(original)
  manifest = package(original)
  assert apply(disk, manifest)
  expected = original[:4096] + b'N' * 512 + original[4608:]
  assert disk.getvalue() == expected
  assert apply(disk, manifest)
  assert disk.getvalue() == expected


@pytest.mark.parametrize('offset', [1024, 2048, 12000])
def test_wrong_partition_or_modified_root_refuses_without_any_write(offset):
  original = b'A' * 16384
  corrupt = original[:offset] + b'!' + original[offset + 1:]
  disk = io.BytesIO(corrupt)
  with pytest.raises(RuntimeError):
    apply(disk, package(original))
  assert disk.getvalue() == corrupt


def test_torn_patch_is_recoverable_but_other_corruption_is_not():
  original = b'A' * 16384
  disk = io.BytesIO(original[:4096] + b'X' * 300 + original[4396:])
  assert apply(disk, package(original))
  assert disk.getvalue()[4096:4608] == b'N' * 512


def test_verify_only_and_fat_changes_do_not_write():
  original = b'A' * 16384
  current = b'F' * 512 + original[512:]
  disk = io.BytesIO(current)
  assert not apply(disk, package(original), verify_only=True)
  assert disk.getvalue() == current


def test_failed_flush_rolls_back():
  original = b'A' * 16384
  disk = io.BytesIO(original)
  calls = []
  def sync():
    calls.append(1)
    if len(calls) == 1:
      raise OSError('simulated I/O failure')
  with pytest.raises(OSError):
    apply(disk, package(original), sync=sync)
  assert disk.getvalue() == original


@pytest.mark.parametrize('change', ['unaligned', 'overlap', 'outside', 'checksum', 'empty'])
def test_bad_manifest_never_writes(change):
  original = b'A' * 16384
  manifest = package(original)
  if change == 'unaligned': manifest['patches'][0]['offset'] += 1
  if change == 'overlap': manifest['patches'] *= 2
  if change == 'outside': manifest['patches'][0]['offset'] = 16384
  if change == 'checksum': manifest['patches'][0]['after_sha256'] = '0' * 64
  if change == 'empty': manifest['patches'] = []
  disk = io.BytesIO(original)
  with pytest.raises(ValueError): apply(disk, manifest)
  assert disk.getvalue() == original


def test_release_payload_preserves_original_provisioning_and_helpers():
  source = Path(__file__).parent
  pieces = [(source/name).read_bytes() for name in ('image_first_boot.py', 'install_usbc.py', 'usbc_host.py')]
  assert sha(pieces[0]) == BOOT_SHA
  code = bootstrap(*pieces)
  assert len(code) == len(pieces[0])
  # Parse the generated constant without executing privileged bootstrap code.
  import ast
  tree = ast.parse(code)
  # Find b85decode's literal independently of the surrounding statement layout.
  literals = [n.args[0].value for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'b85decode']
  assert len(literals) == 1
  assert zlib.decompress(base64.b85decode(literals[0])).split(b'\0') == pieces


@pytest.mark.parametrize('fail', [False, True])
def test_first_boot_installs_before_provisioning_and_failure_keeps_provisioning(tmp_path, monkeypatch, fail):
  record = tmp_path/'provisioned'
  original = (f'from pathlib import Path\nPath({str(record)!r}).write_text("done")\n' + '#' + ' ' * 6000).encode()
  installed = tmp_path/'installed'
  installer = (f'from pathlib import Path\ndef configure(root, source):\n'
               f' assert root == "/"\n assert Path(source,"usbc_host.py").read_bytes()==b"helper"\n'
               f' Path({str(installed)!r}).write_text("done")\n').encode()
  code = bootstrap(original, installer, b'helper')
  is_file = Path.is_file
  # Windows represents an absolute POSIX path with backslashes.
  monkeypatch.setattr(Path, 'is_file', lambda p: True if p.as_posix() == '/etc/carrot-jetlink-image.json' else is_file(p))
  monkeypatch.setattr(os, 'geteuid', lambda: 0, raising=False)
  commands = []
  def run(command, **kwargs):
    assert installed.exists() and not record.exists()
    commands.append(command)
    if fail: raise OSError('simulated systemd failure')
  monkeypatch.setattr(subprocess, 'run', run)
  if fail:
    with pytest.raises(OSError): exec(code, {'__name__': '__main__'})
  else:
    exec(code, {'__name__': '__main__'})
    assert commands == [['systemctl', 'daemon-reload'], ['systemctl', 'start', 'carrot-jetlink-usbc-host.service']]
  assert record.read_text() == 'done'


@pytest.mark.skipif(sys.platform == 'win32' or not shutil.which('mkfs.ext4'), reason='Linux e2fsprogs integration')
def test_real_ext4_data_only_replacement_passes_fsck(tmp_path):
  tree = tmp_path/'root'
  tree.mkdir()
  original = (Path(__file__).parent/'image_first_boot.py').read_bytes()
  replacement = bootstrap(original, (Path(__file__).parent/'install_usbc.py').read_bytes(),
                          (Path(__file__).parent/'usbc_host.py').read_bytes())
  (tree/'boot.py').write_bytes(original)
  filesystem = tmp_path/'root.ext4'
  with filesystem.open('wb') as f: f.truncate(32 << 20)
  subprocess.run(['mkfs.ext4', '-q', '-F', '-b', '4096', '-d', str(tree), str(filesystem)], check=True)
  subprocess.run(['e2fsck', '-fn', str(filesystem)], check=True, capture_output=True)
  output = subprocess.check_output(['debugfs', '-R', 'blocks /boot.py', str(filesystem)], text=True)
  blocks = [int(value) for value in output.split()]
  # Pin the production block size; host mke2fs.conf defaults vary.
  assert blocks == list(range(blocks[0], blocks[0] + len(blocks)))
  base = b'G' * 2048 + filesystem.read_bytes()
  offset = 2048 + blocks[0] * 4096
  assert base[offset:offset+len(original)] == original
  padded = replacement + base[offset+len(original):offset + ((len(original)+511)//512*512)]
  disk = io.BytesIO(base)
  apply(disk, package(base, offset, padded))
  filesystem.write_bytes(disk.getvalue()[2048:])
  subprocess.run(['e2fsck', '-fn', str(filesystem)], check=True, capture_output=True)
  result = subprocess.check_output(['debugfs', '-R', 'cat /boot.py', str(filesystem)])
  assert result == replacement
