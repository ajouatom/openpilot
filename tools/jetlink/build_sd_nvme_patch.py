"""Generate an R2 SD/NVMe sector patch; open the existing image READ ONLY.

Publisher dependency: ext4==1.4.1. No rebuild, recompression of the disk image,
new filesystem, inode edit or GPT edit. Replacements must fit existing files.
"""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import struct
import uuid
import zlib

from initrd_readonly import entries, encode
from portable_initrd import APP_UUID, patch as patch_init
from offline_hotfix import validate

BASE_SHA = 'b11f5601d3a713ad0de23315ee90daddf5452f8e548f2c87c8eeec28d321e55f'
BASE_BYTES = 40 * (1 << 30)


def digest(body):
  return hashlib.sha256(body).hexdigest()


def fit_text(body, size):
  if len(body) > size:
    raise ValueError('Replacement exceeds existing file size')
  return body + b' ' * (size - len(body))


def packed_python(body, size):
  compile(body, '<reviewed-source>', 'exec')
  packed = base64.b85encode(zlib.compress(body, 9))
  code = (b'# Carrot R2 SD/NVMe patch; source: tools/jetlink in carrot-wip.\n'
          b'import base64,zlib\nexec(compile(zlib.decompress(base64.b85decode(' + repr(packed).encode()
          + b')),__file__,"exec"))\n')
  return fit_text(code, size)


class Overlay:
  """Read-only view for independent ext4 decoding and full result hashing."""
  def __init__(self, stream, patches):
    self.stream, self.patches = stream, patches

  def tell(self):
    return self.stream.tell()

  def seek(self, *args):
    return self.stream.seek(*args)

  def read(self, count=-1):
    position = self.tell()
    body = bytearray(self.stream.read(count))
    for offset, _, after in self.patches:
      lo, hi = max(position, offset), min(position + len(body), offset + len(after))
      if lo < hi:
        body[lo-position:hi-position] = after[lo-offset:hi-offset]
    return bytes(body)

  def peek(self, count=0):
    position = self.tell()
    body = self.read(count)
    self.seek(position)
    return body


def file_patches(stream, volume, start, path, replacement):
  inode = volume.inode_at(path)
  original = inode.open().read()
  if len(replacement) != len(original) or inode.i_flags != 0x80000 or volume.block_size != 4096:
    raise ValueError('Only same-size plain extent files are supported')
  result, covered = [], 0
  for extent in inode.extents:
    if not extent.is_initialized or extent.ee_block * 4096 != covered:
      raise ValueError('Sparse/uninitialized/noncontiguous logical file')
    count = min(extent.len * 4096, len(original) - covered)
    if count <= 0:
      break
    offset = start + extent.ee_start * 4096
    stream.seek(offset)
    before = stream.read((count + 511) // 512 * 512)
    if before[:count] != original[covered:covered+count]:
      raise ValueError('Independent extent/file mapping mismatch')
    after = replacement[covered:covered+count] + before[count:]
    if before != after:
      result.append((offset, before, after))
    covered += count
  if covered != len(original):
    raise ValueError('File allocation is incomplete')
  return result


def build(image, output):
  from ext4 import Volume
  source = Path(__file__).parent
  if image.stat().st_size != BASE_BYTES:
    raise ValueError('Expected the 40 GiB R2 image')
  with image.open('rb') as stream:
    print('Verifying existing R2 image (read only)', flush=True)
    if hashlib.file_digest(stream, 'sha256').hexdigest() != BASE_SHA:
      raise ValueError('Wrong R2 base image')
    stream.seek(1024)
    guard = stream.read(512)
    first, last = struct.unpack_from('<QQ', guard, 32)
    start, length = first * 512, (last - first + 1) * 512
    if str(uuid.UUID(bytes_le=guard[16:32])) != APP_UUID:
      raise ValueError('Unexpected APP GPT identity')
    volume = Volume(stream, offset=start)
    def read(path):
      return volume.inode_at(path).open().read()
    replacements = {}
    path = '/boot/extlinux/extlinux.conf'
    original = read(path)
    boot = '\n'.join(line for line in original.decode().splitlines() if not line.lstrip().startswith('#')) + '\n'
    if boot.count('root=/dev/mmcblk0p1') != 1:
      raise ValueError('Unexpected boot command line')
    boot = boot.replace('root=/dev/mmcblk0p1', 'root=PARTUUID=' + APP_UUID)
    replacements[path] = fit_text(boot.encode(), len(original))
    path = '/boot/initrd'
    original = read(path)
    members = list(entries(original))
    if sum(name == 'init' for name, _, _ in members) != 1:
      raise ValueError('Expected exactly one init program')
    changed = [(name, fields, patch_init(body) if name == 'init' else body) for name, fields, body in members]
    # A stronger gzip level fits the modified archive within its original
    # allocation. Trailing zero padding is permitted by gzip and Linux initramfs.
    packed = gzip.compress(gzip.decompress(encode(changed)), compresslevel=9, mtime=0)
    if len(packed) > len(original):
      raise ValueError('Initrd exceeds existing allocation; no patch produced')
    replacements[path] = packed + b'\0' * (len(original) - len(packed))
    if list(entries(replacements[path])) != list(entries(encode(changed))):
      raise ValueError('Padded initrd roundtrip mismatch')
    for name in ('protected_storage.py', 'protected_first_boot.py'):
      path = '/usr/lib/carrot-jetlink-storage/' + name
      replacements[path] = packed_python((source / name).read_bytes(), len(read(path)))
    path = '/etc/carrot-jetlink-protected.json'
    replacements[path] = fit_text(b'{"format":2,"layout":"carrot-r2-sd-nvme"}\n', len(read(path)))
    path = '/etc/fstab'
    replacements[path] = fit_text(b'/dev/root / ext4 ro,noload 0 0\n/run/carrot-efi /boot/efi vfat ro,nofail 0 0\n', len(read(path)))
    patches = sorted(p for path, body in replacements.items() for p in file_patches(stream, volume, start, path, body))
    manifest = dict(format=2, release='r2-sd-nvme-v2-candidate', base_image_sha256=BASE_SHA,
                    image_bytes=BASE_BYTES, root_offset=start, root_bytes=length,
                    root_sha256='80fe3f9b746734696b1502820e9dd015f79d8465387886e34a7ffd88bd477ac0',
                    partition_guard=dict(offset=1024, data=base64.b64encode(guard).decode()),
                    files={p: digest(b) for p, b in replacements.items()},
                    patches=[dict(offset=o, before=base64.b64encode(a).decode(), after=base64.b64encode(b).decode(),
                                  before_sha256=digest(a), after_sha256=digest(b)) for o, a, b in patches])
    validate(manifest)
    overlay = Overlay(stream, patches)
    reread = Volume(overlay, offset=start)
    for path, expected in replacements.items():
      if reread.inode_at(path).open().read() != expected:
        raise ValueError('Independent patched ext4 file read mismatch')
    print('Checking complete virtual result; no new image is written', flush=True)
    overlay.seek(0)
    result = hashlib.sha256()
    while block := overlay.read(8 << 20):
      result.update(block)
    manifest['patched_image_sha256'] = result.hexdigest()
  output.mkdir(parents=True, exist_ok=True)
  blob = gzip.compress(json.dumps(manifest, separators=(',', ':')).encode(), compresslevel=9, mtime=0)
  (output / 'sd-nvme-patch.json.gz').write_bytes(blob)
  metadata = dict(format=1, version=manifest['release'], patch_sha256=digest(blob), patch_bytes=len(blob),
                  base_image_sha256=BASE_SHA, patched_image_sha256=manifest['patched_image_sha256'],
                  image_bytes=BASE_BYTES, root_end=start+length, physical_boot_tested=False,
                  validation='Exact R2 hash, same-size extents, independent ext4 overlay reads and full virtual result hash',
                  changed_files=manifest['files'], changed_bytes=sum(len(b) for _, _, b in patches))
  (output / 'sd-nvme-release.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
  print(json.dumps(metadata), flush=True)
  return metadata


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('image', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  build(args.image, args.output)
