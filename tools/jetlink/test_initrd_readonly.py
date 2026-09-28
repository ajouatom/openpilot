import gzip
import hashlib

import pytest

import initrd_readonly as initrd


def fixture_init():
  return b'''#!/bin/bash
_mount_root() {
			mount -r "${dev}" "${mnt}"
}
mount_root() {
		_mount_root "${dev}" "/mnt" "${retry}" 0
}
overlayfs_check
cd /usr/sbin;
cp /etc/resolv.conf etc/resolv.conf
exec chroot . /sbin/init 2;
'''


def test_unknown_boot_program_is_rejected():
  with pytest.raises(ValueError, match='Unreviewed'):
    initrd.patch_init(fixture_init())


def test_first_mount_has_no_journal_writes_and_pid1_sees_block_readonly(monkeypatch):
  source = fixture_init()
  monkeypatch.setattr(initrd, 'INIT_SHA256', hashlib.sha256(source).hexdigest())
  result = initrd.patch_init(source).decode()
  assert 'mount -t ext4 -o ro,noload' in result
  assert '_mount_root "${dev}" "/mnt" "${retry}" 1' in result
  assert 'overlayfs_enabled=0' in result and '\noverlayfs_check\n' not in result
  assert result.index('--setro /dev/mmcblk0p1') < result.index('exec chroot . /sbin/init')
  assert 'cp /etc/resolv.conf' not in result
  assert '"${rootdev}" != "mmcblk0p1"' in result
  with pytest.raises(ValueError, match='already patched'):
    initrd.patch_init(result.encode())


def test_archive_retains_modes_owners_links_and_binary_payload():
  fields = [42, 0o100755, 0, 0, 1, 12345, 0, 0, 0, 0, 0, 0, 0]
  link = fields.copy()
  link[1] = 0o120777
  members = [('init', fields, b'#!/bin/bash\n'), ('bin/alias', link, b'tool'),
             ('binary', fields, bytes(range(256))), ('TRAILER!!!', fields, b'')]
  output = initrd.encode(members)
  reread = list(initrd.entries(output))
  assert [(n, b) for n, _, b in reread] == [(n, b) for n, _, b in members]
  for (_, before, _), (_, after, _) in zip(members, reread):
    assert [v for i, v in enumerate(before) if i not in (6, 11)] == [v for i, v in enumerate(after) if i not in (6, 11)]
  with pytest.raises(ValueError):
    list(initrd.entries(gzip.compress(gzip.decompress(output)[:120])))
