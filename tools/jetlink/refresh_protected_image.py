"""Rebuild the public R2 SD image in an isolated Linux image-building job."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import urllib.request

from refresh_sd_image import audit_unprovisioned, digest, refresh, run
from update_host import fetch, verify_signature

BASE_URL = 'https://upload.shind0.synology.me/downloads/jetson/v0.4.0-boot-preview/carrot-jetson.img.zst'
BASE_SHA = 'b277f3c6318d5b3210eee6e6aa4e74e492f73ffe2f9a669e108822a7ceb6c614'
RAW_SHA = 'b11f5601d3a713ad0de23315ee90daddf5452f8e548f2c87c8eeec28d321e55f'
CACHED_SHA = '4cb5db4e006e41433adcc866f10252d9bb2fec68ad1f7f75229e217fb8c80620'
CACHED_RAW_SHA = '5e0da1b97485f3f06cfc809d6ddff446a868a8dfe0fd1c65e90a3ed386cc8c54'


def unpack_base(work, cached=None):
  import zstandard
  compressed = work / 'base.img.zst'
  if cached is None:
    urllib.request.urlretrieve(BASE_URL, compressed)
  else:
    shutil.copyfile(cached, compressed)
  if digest(compressed) != (BASE_SHA if cached is None else CACHED_SHA):
    raise ValueError('Public R2 compressed image hash mismatch')
  image = work / 'carrot-jetson.img'
  h = hashlib.sha256()
  with compressed.open('rb') as source, image.open('xb') as target:
    with zstandard.ZstdDecompressor().stream_reader(source) as reader:
      while block := reader.read(4 << 20):
        h.update(block)
        if block == bytes(len(block)):
          target.seek(len(block), 1)
        else:
          target.write(block)
      target.truncate()
  if image.stat().st_size != 40 * (1 << 30) or h.hexdigest() != (RAW_SHA if cached is None else CACHED_RAW_SHA):
    raise ValueError('Public R2 raw image hash mismatch')
  compressed.unlink()
  return image


def rebuild(work, manifest, cached=None):
  if os.geteuid() != 0 or work.exists():
    raise ValueError('Use root and a new image workspace')
  verify_signature(manifest)
  work.mkdir(parents=True)
  image = unpack_base(work, cached)
  bundle = work / 'runtime.tar.gz'
  fetch(manifest['bundle']['url'], bundle, manifest['bundle']['sha256'], manifest['bundle']['size'], None)
  model = work / 'model.onnx'
  fetch(manifest['model']['url'], model, manifest['model']['sha256'], manifest['model']['size'], None)
  layout = json.loads(subprocess.check_output(['sfdisk', '--json', str(image)]))['partitiontable']
  parts = layout['partitions']
  if (layout['label'] != 'gpt' or len(parts) != 17 or parts[0]['start'] != 3188736
      or parts[16]['start'] != 24 * (1 << 30) // 512):
    raise ValueError('Unexpected R2 partition layout')
  loop = subprocess.check_output(['losetup', '--find', '--show', '--partscan', str(image)], text=True).strip()
  mounted = []
  try:
    backing = subprocess.check_output(['losetup', '-n', '-O', 'BACK-FILE', loop], text=True).strip()
    if not loop.startswith('/dev/loop') or Path(backing).resolve() != image.resolve():
      raise ValueError('Unexpected output loop backing file')
    run('udevadm', 'settle', '--timeout=10')
    root, setup, data = [work / name for name in ('rootfs', 'setup', 'data')]
    for number, path in ((1, root), (16, setup), (17, data)):
      path.mkdir()
      run('mount', loop + f'p{number}', path)
      mounted.append(path)
    if (data / 'identity').exists() or (setup / 'identity').exists():
      raise ValueError('Personal identity in base image')
    if cached is None:
      marker = refresh(root, setup, bundle)
    else:
      audit_unprovisioned(root, setup)
      marker = json.loads((root / 'etc/carrot-jetlink-image.json').read_text())
      if marker['source_commit'] != manifest['source_commit']:
        raise ValueError('Cached image runtime differs from the signed manifest')
    source = Path(__file__).resolve().parent
    from install_super_bootstrap import configure as configure_super
    configure_super(root, source, work)
    from install_boot_display import configure as configure_boot_display
    configure_boot_display(root, source)
    shutil.copyfile(source / 'protected_storage.py', root / 'usr/lib/carrot-jetlink-storage/protected_storage.py')
    runtime = root / 'opt/carrot-jetlink'
    from model_pin import read_spec
    spec = read_spec(runtime / 'current')
    if spec['sha256'] != manifest['model']['sha256'] or spec['nbytes'] != model.stat().st_size:
      raise ValueError('Image runtime and model contract differ')
    # Only the new, mounted output image is changed. Discard reference-board
    # TensorRT caches so the recipient builds for its own GPU at first startup.
    cache = runtime / 'cache'
    shutil.rmtree(cache)
    (cache / 'models').mkdir(parents=True)
    shutil.copyfile(model, cache / 'models' / (spec['sha256'][:16] + '.onnx'))
    run('chown', '-R', '1000:1000', cache)
    shutil.rmtree(data / 'runtime')
    run('cp', '-a', runtime, data / 'runtime')
    (data / 'runtime/protected-runtime.json').write_text(json.dumps({
      'format': 1, 'preloaded_model': {'sha256': spec['sha256'], 'bytes': spec['nbytes']}}) + '\n')
    marker.update(state='PROTECTED_CANDIDATE_NOT_BOOT_TESTED', storage_format=1,
                  builder_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  data_partition=17, data_grow=False, image_bytes=image.stat().st_size,
                  model_sha256=spec['sha256'], engine_prebuilt=False,
                  power_setup='automatic NVIDIA Super capsule provisioning and MAXN_SUPER verification',
                  automatic_super_firmware=True,
                  firmware_provisioning='Orin Nano 8GB developer kit; bootable JetPack 6.x SD required')
    for path in (root / 'etc/carrot-jetlink-image.json', setup / 'IMAGE.json'):
      path.write_text(json.dumps(marker, indent=2) + '\n')
    audit_unprovisioned(root, setup)
    for copy in (runtime, data / 'runtime'):
      assert (copy / 'current/SOURCE_COMMIT').read_text().strip() == manifest['source_commit']
      assert digest(copy / 'cache/models' / (spec['sha256'][:16] + '.onnx')) == spec['sha256']
    # Exercise the actual pre-bind admission using the installed image helper.
    # A fresh image has ONNX but no last-loaded record or TensorRT engine yet.
    probe = root / 'run/carrot-data'
    probe.mkdir(parents=True, exist_ok=True)
    run('mount', '--bind', data, probe)
    mounted.append(probe)
    run('chroot', root, '/usr/bin/python3', '-B', '-c',
        'import sys; from pathlib import Path; sys.path.insert(0,"/usr/lib/carrot-jetlink-storage"); ' +
        'from protected_storage import valid_runtime; assert valid_runtime(Path("/run/carrot-data/runtime"))')
    run('umount', probe)
    mounted.pop()
    run('chroot', root, '/opt/carrot-jetlink/venv/bin/python',
        '/opt/carrot-jetlink/current/tools/jetlink/server.py', '--check-imports')
    run('chroot', root, '/opt/carrot-jetlink/venv/bin/python',
        '/opt/carrot-jetlink/current/tools/jetlink/hud.py', '--help')
    run('chroot', root, '/usr/sbin/visudo', '-cf', '/etc/sudoers')
    run('chroot', root, '/usr/bin/python3', '-B', '-c',
        'import sys; sys.path.insert(0,"/usr/lib/carrot-jetlink-firmware"); ' +
        'import super_bootstrap,hashlib; ' +
        'assert hashlib.sha256(super_bootstrap.CAPSULE.read_bytes()).hexdigest()==super_bootstrap.CAPSULE_SHA')
    run('systemd-analyze', '--root=' + str(root), 'verify',
        'carrot-super-firmware.service',
        'carrot-jetlink-performance.service', 'carrot-jetlink-update-apply.service',
        'carrot-jetlink.service', 'carrot-jetlink-hud.service')
    run('sync', '-f', image)
    for path in reversed(mounted):
      run('umount', path)
    mounted.clear()
    for number in (1, 17):
      result = subprocess.run(['e2fsck', '-f', '-p', loop + f'p{number}'])
      if result.returncode not in (0, 1):
        raise RuntimeError('Image filesystem validation failed')
      run('zerofree', loop + f'p{number}')
    # The public base's ESP retains a FAT dirty flag. Firmware staging needs a
    # checked writable ESP, so repair only this new file-backed image offline.
    result = subprocess.run(['fsck.vfat', '-a', loop + 'p10'])
    if result.returncode not in (0, 1):
      raise RuntimeError('EFI filesystem repair failed')
    run('fsck.vfat', '-n', loop + 'p10')
    run('fsck.vfat', '-n', loop + 'p16')
  finally:
    for path in reversed(mounted):
      run('umount', path)
    run('losetup', '-d', loop)
  gpt = subprocess.check_output(['sgdisk', '-v', str(image)], text=True)
  if 'No problems found.' not in gpt:
    raise RuntimeError('GPT verification did not confirm a valid image')
  compressed = image.with_suffix('.img.zst')
  run('zstd', '-T2', '-3', image, '-o', compressed)
  marker.update(image_sha256=digest(image), compressed_bytes=compressed.stat().st_size,
                compressed_sha256=digest(compressed), base_raw_sha256=RAW_SHA if cached is None else CACHED_RAW_SHA,
                bundle_sha256=manifest['bundle']['sha256'],
                offline_checks={'runtime_imports': True, 'model_hashes': True, 'privacy': True,
                                'filesystems': True, 'gpt': True, 'systemd_units': True,
                                'firmware_capsule': True, 'fresh_data_admission': True},
                validation='Offline import, model hash, privacy and filesystem checks; physical boot pending')
  (work / 'candidate.json').write_text(json.dumps(marker, indent=2) + '\n')
  (work / 'SHA256SUMS').write_text(f'{marker["image_sha256"]}  {image.name}\n' +
                                f'{marker["compressed_sha256"]}  {compressed.name}\n')
  print(json.dumps(marker), flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--manifest', type=Path, required=True)
  parser.add_argument('--output-dir', type=Path, required=True)
  parser.add_argument('--cached-base', type=Path)
  args = parser.parse_args()
  rebuild(args.output_dir.resolve(), json.loads(args.manifest.read_text()), args.cached_base)
