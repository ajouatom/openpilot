"""Publish a small R2 boot-update patch without mounting or rewriting an image.

Known APP variants are verified in full. DATA, GPT and identity remain untouched.
Publisher dependency: ext4==1.4.1; users need only bundled portable Python.
"""
import argparse
import ast
import base64
import gzip
import hashlib
import io
import json
from pathlib import Path
import struct
import subprocess
import zipfile
import zlib

from build_sd_nvme_patch import BASE_SHA, BASE_BYTES, Overlay, file_patches, packed_python
from offline_hotfix import normalized_hash, validate

HOST_SOURCE = '84087a5b78118acc40234bfd8ef64235421f1aab'
HELPERS = ('boot_update.py', 'update_host.py', 'wifi_protocol.py', 'hud_protocol.py',
           'finalize_sd_image.py', 'release-signing-public.pem', 'install_updates.py')
FIRST_BOOT = '/usr/lib/carrot-jetlink-storage/protected_first_boot.py'


def digest(data):
  return hashlib.sha256(data).hexdigest()


def payload(source):
  output = io.BytesIO()
  with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    for name in (*HELPERS, 'offline_bootstrap.py'):
      data = ((source / name).read_bytes() if name == 'offline_bootstrap.py' else
              subprocess.check_output(['git', 'show', f'{HOST_SOURCE}:tools/jetlink/{name}'], cwd=source))
      entry = zipfile.ZipInfo(name, date_time=(2026, 10, 7, 0, 0, 0))
      entry.compress_type = zipfile.ZIP_DEFLATED
      archive.writestr(entry, data.replace(b'\r\n', b'\n'))
  return output.getvalue()


def unpack_source(original):
  if b'exec(compile(zlib.decompress(base64.b85decode(' not in original:
    return original.decode().rstrip() + '\n'
  tree = ast.parse(original)
  calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
           isinstance(node.func, ast.Attribute) and node.func.attr == 'b85decode']
  if len(calls) != 1 or len(calls[0].args) != 1:
    raise ValueError('Unknown packed image helper')
  return zlib.decompress(base64.b85decode(ast.literal_eval(calls[0].args[0]))).decode()


def replacement(original, hook, blob):
  text = unpack_source(original)
  entry = "if __name__ == '__main__':\n  main()"
  if text.count(entry) != 1:
    raise ValueError('Unexpected first-boot entry point')
  text = text.replace(entry, '')
  hook = hook.replace('PAYLOAD_SHA_PLACEHOLDER', digest(blob)).replace(
    '0  # PAYLOAD_BYTES_PLACEHOLDER', str(len(blob)))
  text += '\n' + hook + "\nif __name__ == '__main__':\n  offline_bootstrap()\n  main()\n"
  return packed_python(text.encode(), len(original))


def encode_patch(item):
  offset, before, after = item
  return {'offset': offset, 'before': base64.b64encode(before).decode(), 'after': base64.b64encode(after).decode(),
              'before_sha256': digest(before), 'after_sha256': digest(after)}


def build(image, portable_zip, output):
  from ext4 import Volume
  source = Path(__file__).resolve().parent
  blob = payload(source)
  hook = (source / 'offline_boot_hook.py').read_text()
  with zipfile.ZipFile(portable_zip) as package:
    meta = json.loads(package.read('support/sd-nvme-release.json'))
    packed = package.read('support/sd-nvme-patch.json.gz')
  if digest(packed) != meta['patch_sha256']:
    raise ValueError('Portable patch checksum mismatch')
  portable = json.loads(gzip.decompress(packed))
  if portable['patched_image_sha256'] != '2f97e66ea533c34750ba676a81df51e4485eb6ed742dcbe48a324ad605ade62b':
    raise ValueError('Only the published v2 portable patch is supported')
  overlays = validate(portable)
  if image.stat().st_size != BASE_BYTES:
    raise ValueError('Expected the published 40 GiB R2 image')
  output.mkdir(parents=True, exist_ok=True)
  (output / 'carrot-boot-update.zip').write_bytes(blob)
  with image.open('rb') as stream:
    print('Verify base image, read only', flush=True)
    if hashlib.file_digest(stream, 'sha256').hexdigest() != BASE_SHA:
      raise ValueError('Wrong R2 image; nothing was patched')
    stream.seek(1024)
    guard = stream.read(512)
    first, last = struct.unpack_from('<QQ', guard, 32)
    start, length = first * 512, (last - first + 1) * 512
    stream.seek(1024 + 15 * 128)
    setup = stream.read(128)
    setup_first, setup_last = struct.unpack_from('<QQ', setup, 32)
    profiles = []
    canonical = None
    for name, changes in [('r2-original', []), ('r2-sd-nvme-v2', overlays)]:
      view = Overlay(stream, changes)
      volume = Volume(view, offset=start)
      original = volume.inode_at(FIRST_BOOT).open().read()
      changed = replacement(original, hook, blob)
      patches = file_patches(view, volume, start, FIRST_BOOT, changed)
      if canonical is None:
        canonical = patches
      elif [(o, len(a)) for o, a, _ in patches] != [(o, len(a)) for o, a, _ in canonical]:
        raise ValueError('Variant changed the first-boot allocation')
      patches = [(o, c[1], after) for (o, _, after), c in zip(patches, canonical, strict=True)]
      manifest = {'format': 2, 'release': name, 'image_bytes': BASE_BYTES, 'root_offset': start, 'root_bytes': length,
                      'partition_guard': {'offset': 1024, 'data': base64.b64encode(guard).decode()},
                      'root_sha256': normalized_hash(view, start, length, patches),
                      'patches': [encode_patch(p) for p in patches]}
      validate(manifest)
      result = Volume(Overlay(view, patches), offset=start).inode_at(FIRST_BOOT).open().read()
      if result != changed:
        raise ValueError('Independent ext4 readback mismatch')
      profiles.append(manifest)
      print('Verified profile', name, len(original), sum(len(p[2]) for p in patches), flush=True)
  index = {'format': 'carrot-offline-boot-v1', 'host_source': HOST_SOURCE,
               'payload_sha256': digest(blob), 'payload_bytes': len(blob), 'profiles': profiles,
               'setup_offset': setup_first * 512, 'setup_bytes': (setup_last - setup_first + 1) * 512}
  (output / 'boot-patch.json').write_text(json.dumps(index, separators=(',', ':')) + '\n')
  return index


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('image', type=Path)
  parser.add_argument('portable_zip', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  build(args.image, args.portable_zip, args.output)
