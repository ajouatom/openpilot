"""Refresh a hash-pinned, unprovisioned SD image without copying the live host.

Only a newly created file-backed image is mounted. Output is a candidate, never
a boot-validated release. Run at low CPU/I/O priority on an aarch64 build host.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

from finalize_sd_image import extract_bundle, write


def digest(path):
  h = hashlib.sha256()
  with path.open('rb') as f:
    for block in iter(lambda: f.read(4 << 20), b''):
      h.update(block)
  return h.hexdigest()


def run(*args):
  print('RUN', *map(str, args), flush=True)
  subprocess.run(list(map(str, args)), check=True)


def audit_unprovisioned(root, setup):
  for pattern in ('etc/ssh/ssh_host_*', 'etc/NetworkManager/system-connections/*',
                  'home/*/.ssh/*', 'root/.ssh/*', 'var/lib/carrot-jetlink/provisioned.json',
                  'var/lib/carrot-jetlink/root-expanded.json'):
    if list(root.glob(pattern)):
      raise ValueError('Provisioned/private state in base image: ' + pattern)
  if (root/'etc/machine-id').read_bytes():
    raise ValueError('Base image machine-id is not empty')
  if (setup/'setup.json').exists() or (setup/'SETUP-RESULT.json').exists():
    raise ValueError('Personal setup or previous provisioning result in image')
  for line in (root/'etc/shadow').read_text().splitlines():
    if line.split(':')[1] != '!':
      raise ValueError('Image contains an unlocked/password-bearing account')


def refresh(root, setup, bundle):
  audit_unprovisioned(root, setup)
  runtime = root/'opt/carrot-jetlink'
  releases = runtime/'releases'
  staging = releases/'refresh-staging'
  staging.mkdir()
  extract_bundle(bundle, staging)
  commit = (staging/'SOURCE_COMMIT').read_text().strip()
  if not re.fullmatch('[0-9a-f]{40}', commit):
    raise ValueError('Invalid bundle source commit')
  release = releases/commit
  if release.exists():
    raise ValueError('This image already contains the requested release')
  staging.rename(release)
  for name, target in [('current', 'releases/' + commit), ('carrot', 'current')]:
    link = runtime/name
    if not link.is_symlink():
      raise ValueError('Expected image release symlink: ' + name)
    link.unlink()
    link.symlink_to(target)
  for previous in releases.iterdir():
    if previous != release:
      if previous.is_symlink() or not previous.is_dir():
        raise ValueError('Unexpected previous release entry')
      shutil.rmtree(previous)
  for name in ('var/spool/anacron', 'etc/openvpn'):
    (root/name).mkdir(parents=True, exist_ok=True)
  write(root, '/etc/systemd/system/carrot-jetlink-hud.service.d/log-directory.conf',
        '[Service]\nLogsDirectory=carrot-jetlink-hud\nLogsDirectoryMode=0700\n'
        'WorkingDirectory=/var/log/carrot-jetlink-hud\n')
  marker_path = root/'etc/carrot-jetlink-image.json'
  marker = json.loads(marker_path.read_text())
  marker.update(state='CANDIDATE_PHYSICAL_BOOT_PENDING', source_commit=commit)
  marker_path.write_text(json.dumps(marker, indent=2) + '\n')
  (setup/'IMAGE.json').write_text(json.dumps(marker, indent=2) + '\n')
  audit_unprovisioned(root, setup)
  return marker


def main():
  p = argparse.ArgumentParser(description=__doc__)
  for name in ('base', 'bundle', 'work-dir', 'zerofree'):
    p.add_argument('--' + name, type=Path, required=True)
  for name in ('base-sha256', 'raw-sha256', 'bundle-sha256'):
    p.add_argument('--' + name, required=True)
  args = p.parse_args()
  if os.geteuid() != 0 or os.uname().machine != 'aarch64':
    raise RuntimeError('Run as root on an aarch64 build host')
  for path, expected in ((args.base, args.base_sha256), (args.bundle, args.bundle_sha256)):
    if path.is_symlink() or not path.is_file() or digest(path) != expected:
      raise ValueError('Input hash/type mismatch: ' + str(path))
  work = args.work_dir.absolute()
  if work.exists() or work.parent.resolve() != work.parent:
    raise ValueError('Use a new output directory under a real parent')
  work.mkdir(mode=0o700)
  image = work/'carrot-jetson.img'
  run('zstd', '-d', '--sparse', args.base, '-o', image)
  if image.stat().st_size != 24 * (1 << 30) or digest(image) != args.raw_sha256:
    raise ValueError('Uncompressed image hash/size mismatch')
  loop = subprocess.check_output(['losetup', '--find', '--show', '--partscan', str(image)], text=True).strip()
  mounted = []
  try:
    backing = subprocess.check_output(['losetup', '-n', '-O', 'BACK-FILE', loop], text=True).strip()
    if Path(backing).resolve() != image.resolve():
      raise RuntimeError('Unexpected loop backing file')
    layout = json.loads(subprocess.check_output(['sfdisk', '--json', str(image)]))['partitiontable']
    if layout['label'] != 'gpt' or len(layout['partitions']) != 16:
      raise ValueError('Unexpected base partition layout')
    root, setup = work/'rootfs', work/'setup'
    root.mkdir(); setup.mkdir()
    for dev, target in ((loop+'p1', root), (loop+'p16', setup)):
      run('mount', dev, target); mounted.append(target)
    marker = refresh(root, setup, args.bundle)
    # No host /dev, /sys, GPU or USB is exposed to these import checks.
    run('chroot', root, '/opt/carrot-jetlink/venv/bin/python', '-c',
        'import sys; sys.path[:0]=["/opt/carrot-jetlink/current", "/opt/carrot-jetlink/current/tools/jetlink"]; '
        'import numpy,onnx,usb1,tensorrt,pyray,PIL,usb,capnp,zstandard,serial,Crypto,aiohttp,imageio_ffmpeg,av; '
        'import server; from openpilot.cereal import log; print("ALL_RUNTIME_IMPORTS_OK")')
    run('chroot', root, '/usr/sbin/visudo', '-cf', '/etc/sudoers')
    spec = json.loads((root/'opt/carrot-jetlink/current/openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
    model = root/'opt/carrot-jetlink/cache/models'/(spec['sha256'][:16] + '.onnx')
    if digest(model) != spec['sha256'] or model.stat().st_size != spec['nbytes']:
      raise ValueError('Pinned model changed')
    audit_unprovisioned(root, setup)
    run('sync', '-f', image)
    for target in reversed(mounted):
      run('umount', target)
    mounted.clear()
    run('e2fsck', '-f', '-p', loop+'p1')
    run(args.zerofree, loop+'p1')
    run('fsck.vfat', '-n', loop+'p16')
  finally:
    for target in reversed(mounted):
      run('umount', target)
    run('losetup', '-d', loop)
  run('sgdisk', '-v', image)
  compressed = image.with_suffix('.img.zst')
  run('zstd', '-T1', '-3', image, '-o', compressed)
  marker.update(base_sha256=args.base_sha256, base_raw_sha256=args.raw_sha256,
                bundle_sha256=args.bundle_sha256, image=image.name, image_bytes=image.stat().st_size,
                image_sha256=digest(image), compressed=compressed.name,
                compressed_bytes=compressed.stat().st_size, compressed_sha256=digest(compressed),
                validation='Offline imports, pinned model, privacy and filesystems checked; new image physical boot pending')
  (work/'release.json').write_text(json.dumps(marker, indent=2) + '\n')
  (work/'SHA256SUMS').write_text(f'{marker["image_sha256"]}  {image.name}\n'
                               f'{marker["compressed_sha256"]}  {compressed.name}\n')
  print('CANDIDATE_PACKAGED', json.dumps(marker), flush=True)


if __name__ == '__main__':
  main()
