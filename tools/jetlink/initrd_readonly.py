"""Patch the reviewed L4T36.4.7 SD initrd, without unpacking paths onto the host."""
import gzip
import hashlib
from pathlib import Path

INIT_SHA256 = '29a31aa301a9befc107c561bf0f348037ce84d0a2dfc7905609348fc77b1b677'
MARKER = '# Carrot protected SD: first root mount is ro,noload'


def patch_init(body):
  text = body.decode('utf-8')
  if MARKER in text:
    raise ValueError('Initrd is already patched; use the pristine input')
  if hashlib.sha256(body).hexdigest() != INIT_SHA256:
    raise ValueError('Unreviewed NVIDIA initrd init; refusing to guess boot changes')
  replacements = {
    '\noverlayfs_check\n': '\n' + MARKER + '\noverlayfs_enabled=0\n',
    '\ncd /usr/sbin;': '\nif [ "${rootdev}" != "mmcblk0p1" ]; then\n'
      '\techo "CARROT: unsupported protected root" > /dev/kmsg\n\texec /bin/bash\nfi\n\ncd /usr/sbin;',
    '\t\t\tmount -r "${dev}" "${mnt}"': '\t\t\tmount -t ext4 -o ro,noload "${dev}" "${mnt}"',
    '\t\t_mount_root "${dev}" "/mnt" "${retry}" 0': '\t\t_mount_root "${dev}" "/mnt" "${retry}" 1',
    'cp /etc/resolv.conf etc/resolv.conf':
      '# NetworkManager supplies DNS after the RAM etc overlay is mounted.\n'
      'if ! chroot . /sbin/blockdev --setro /dev/mmcblk0p1; then\n'
      '\techo "CARROT: root block protection failed" > /dev/kmsg\n\texec /bin/bash\nfi\n'
      'echo "CARROT: APP first mount ro,noload; block read-only before PID1" > /dev/kmsg',
  }
  for old, new in replacements.items():
    if text.count(old) != 1:
      raise ValueError('Unexpected initrd patch anchor')
    text = text.replace(old, new, 1)
  return text.encode('utf-8')


def entries(blob):
  """Read only newc, retaining every header field and member, including links."""
  data = gzip.decompress(blob)
  offset = 0
  while offset + 110 <= len(data):
    if data[offset:offset + 6] != b'070701':
      raise ValueError('Expected unchecksummed newc initrd')
    fields = [int(data[offset + 6 + n * 8:offset + 14 + n * 8], 16) for n in range(13)]
    size, length = fields[6], fields[11]
    name_start = offset + 110
    body_start = (name_start + length + 3) & ~3
    end = body_start + size
    if length < 1 or end > len(data) or data[name_start + length - 1] != 0:
      raise ValueError('Truncated initrd')
    name = data[name_start:name_start + length - 1].decode('utf-8')
    yield name, fields, data[body_start:end]
    offset = (end + 3) & ~3
    if name == 'TRAILER!!!':
      if any(data[offset:]):
        raise ValueError('Unexpected trailing initrd data')
      return
  raise ValueError('Missing initrd trailer')


def encode(members):
  output = bytearray()
  for name, fields, body in members:
    encoded = name.encode('utf-8') + b'\0'
    fields = list(fields)
    fields[6], fields[11] = len(body), len(encoded)
    output += b'070701' + ''.join(f'{value:08x}' for value in fields).encode('ascii')
    output += encoded
    output += b'\0' * (-len(output) % 4)
    output += body
    output += b'\0' * (-len(output) % 4)
  output += b'\0' * (-len(output) % 512)
  return gzip.compress(bytes(output), compresslevel=6, mtime=0)


def configure(root):
  root = Path(root)
  if root.resolve() == Path('/') or not (root / 'etc/carrot-jetlink-image.json').is_file():
    raise ValueError('Offline Carrot image required')
  path = root / 'boot/initrd'
  members = list(entries(path.read_bytes()))
  if sum(name == 'init' for name, _, _ in members) != 1:
    raise ValueError('Expected one init program')
  modified = [(name, fields, patch_init(body) if name == 'init' else body)
              for name, fields, body in members]
  blob = encode(modified)
  reread = list(entries(blob))
  assert [(n, b) for n, _, b in reread] == [(n, b) for n, _, b in modified]
  # No firmware/kernel replacement; only the reviewed init program changes.
  temporary = path.with_name('initrd.carrot-new')
  temporary.write_bytes(blob)
  temporary.chmod(path.stat().st_mode & 0o777)
  temporary.replace(path)
  return hashlib.sha256(blob).hexdigest()
