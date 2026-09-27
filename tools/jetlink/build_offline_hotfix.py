"""Publisher-only generator. Requires ext4==1.4.1; never modifies the input.

Pinned v0.2.0 only. Same-size data replacement keeps inode, allocation, journal,
GPT and every other file unchanged. The packed script is generated, not edited.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import zlib

BASE_SHA = '5a1e7a3ba6156c621d8a01412f062b6c16ecb8ef2274b82b6acdaadf4b19a516'
BOOT_SHA = 'ffbebb76b9de08e58fcda32e3d3121cdc0908798da83c6a6c766df8f5d66f719'
RELEASE = 'f2b22dcf0bd0708658668f7efa2dfb29f81b3bd1'
BOOT_PATH = f'/opt/carrot-jetlink/releases/{RELEASE}/tools/jetlink/image_first_boot.py'


def bootstrap(original, installer, helper):
  payload = base64.b85encode(zlib.compress(b'\0'.join([original, installer, helper]), 9)).decode()
  code = '''# Carrot v0.2.0 offline USB-C bootstrap; see OFFLINE-HOTFIX.md.
import base64,zlib,tempfile,subprocess,os
from pathlib import Path
a,b,c=zlib.decompress(base64.b85decode(PAYLOAD)).split(b'\\0')
if os.geteuid()!=0 or not Path('/etc/carrot-jetlink-image.json').is_file():
 raise RuntimeError('Not a Carrot SD image')
try:
 with tempfile.TemporaryDirectory() as d:
  Path(d,'usbc_host.py').write_bytes(c)
  ns={'__name__':'offline_hotfix'}
  exec(b,ns)
  ns['configure']('/',d)
 subprocess.run(['systemctl','daemon-reload'],check=True)
 subprocess.run(['systemctl','start','carrot-jetlink-usbc-host.service'],check=True)
finally:
 exec(a,{'__name__':'__main__'})
'''.replace('PAYLOAD', repr(payload)).encode()
  padding = len(original) - len(code)
  if padding < 2:
    raise ValueError('Bootstrap does not fit the existing file allocation')
  result = code + b'#' + b' ' * (padding - 2) + b'\n'
  compile(result, BOOT_PATH, 'exec')
  return result


def main():
  from ext4 import Volume
  from offline_hotfix import normalized_hash
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('image', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  source = Path(__file__).parent
  with args.image.open('rb') as stream:
    digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if digest != BASE_SHA:
      raise ValueError('Unsupported base image')
    stream.seek(1024)
    guard = stream.read(512)
    first, last = struct.unpack_from('<QQ', guard, 32)
    start, length = first * 512, (last - first + 1) * 512
    volume = Volume(stream, offset=start)
    inode = volume.inode_at(BOOT_PATH)
    original = inode.open().read()
    if hashlib.sha256(original).hexdigest() != BOOT_SHA or inode.i_flags != 0x80000 or len(inode.extents) != 1:
      raise ValueError('Unexpected bootstrap inode, flags or contents')
    extent = inode.extents[0]
    if extent.ee_block != 0 or not extent.is_initialized or extent.len != 2 or volume.block_size != 4096:
      raise ValueError('Unexpected allocation')
    replacement = bootstrap(original, (source/'install_usbc.py').read_bytes(), (source/'usbc_host.py').read_bytes())
    offset = start + extent.ee_start * volume.block_size
    count = (len(original) + 511) // 512 * 512
    stream.seek(offset)
    before = stream.read(count)
    if before[:len(original)] != original:
      raise ValueError('Extent mapping disagrees with file read')
    after = replacement + before[len(original):]
    manifest = dict(format=1, release='v0.2.0-preview + offline-usbc-v1', base_image_sha256=BASE_SHA,
                    root_offset=start, root_bytes=length,
                    root_sha256=normalized_hash(stream, start, length, []),
                    partition_guard=dict(offset=1024, data=base64.b64encode(guard).decode()),
                    file=BOOT_PATH, file_before_sha256=BOOT_SHA,
                    file_after_sha256=hashlib.sha256(replacement).hexdigest(),
                    patches=[dict(offset=offset, before=base64.b64encode(before).decode(),
                                  after=base64.b64encode(after).decode(),
                                  before_sha256=hashlib.sha256(before).hexdigest(),
                                  after_sha256=hashlib.sha256(after).hexdigest())])
  args.output.mkdir(parents=True, exist_ok=True)
  (args.output/'offline-usbc.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
  (args.output/'bootstrap-review.py').write_bytes(replacement)
  print('MANIFEST_SHA256', hashlib.sha256((args.output/'offline-usbc.json').read_bytes()).hexdigest())


if __name__ == '__main__':
  main()
