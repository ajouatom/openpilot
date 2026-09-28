"""Build a PRIVATE CANDIDATE from a verified pristine raw image, on Linux.

Only file-backed loop devices in a new output directory are modified. By
default the input is copied. --consume-staging-base explicitly consumes a
disposable sibling *.STAGING.img file, avoiding another full SD write. The live
SD is never a writable target. Physical boot/power-cut tests remain mandatory
before replacing a public download or enabling automatic vehicle migration.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess

from install_protected import configure
from refresh_sd_image import audit_unprovisioned, digest, refresh

GIB = 1 << 30


def validate_base(layout, size):
  if size != 24 * GIB or layout['label'] != 'gpt' or layout['sectorsize'] != 512:
    raise ValueError('Expected pristine 24 GiB GPT image')
  parts = layout['partitions']
  if len(parts) != 16 or sum(p['name'] == 'APP' for p in parts) != 1:
    raise ValueError('Unexpected image partitions')
  app = next(p for p in parts if p['name'] == 'APP')
  if app['start'] != 3188736 or any(p['start'] + p['size'] > app['start'] for p in parts if p is not app):
    raise ValueError('Unexpected APP placement')
  if app['start'] + app['size'] > size // 512 - 33:
    raise ValueError('APP exceeds input image')
  if not any(p['name'] == 'CARROT_SETUP' for p in parts):
    raise ValueError('Missing private setup partition')


def run(*args):
  subprocess.run([str(a) for a in args], check=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  for name in ('base', 'bundle', 'output-dir'):
    parser.add_argument('--' + name, type=Path, required=True)
  parser.add_argument('--base-sha256', required=True)
  parser.add_argument('--bundle-sha256', required=True)
  parser.add_argument('--consume-staging-base', action='store_true')
  args = parser.parse_args()
  if os.name != 'posix' or os.geteuid() != 0:
    raise RuntimeError('Root on a Linux build host required')
  for path, expected in ((args.base, args.base_sha256), (args.bundle, args.bundle_sha256)):
    if path.is_symlink() or not path.is_file() or digest(path) != expected:
      raise ValueError('Input file/hash mismatch')
  work = args.output_dir.absolute()
  if work.exists() or work.parent.resolve() != work.parent:
    raise ValueError('A new output directory under a real parent is required')
  if args.consume_staging_base and (args.base.resolve().parent != work.parent
                                    or not args.base.name.endswith('.STAGING.img')
                                    or args.base.stat().st_nlink != 1):
    raise ValueError('Only a disposable, unshared sibling STAGING image may be consumed')
  layout = json.loads(subprocess.check_output(['sfdisk', '--json', str(args.base)]))['partitiontable']
  validate_base(layout, args.base.stat().st_size)
  work.mkdir(mode=0o700)
  image = work / 'carrot-jetson-protected-CANDIDATE.img'
  if args.consume_staging_base:
    args.base.rename(image)
  else:
    run('cp', '--reflink=auto', '--sparse=always', '--', args.base, image)
  with image.open('r+b') as output:
    output.truncate(40 * GIB)
  run('sgdisk', '-e', image)
  # APP and all firmware partitions retain their existing start/size/content.
  run('sgdisk', f'-n17:{24 * GIB // 512}:{40 * GIB // 512 - 2049}', '-t17:8300', '-c17:CARROT_DATA', image)
  loop = subprocess.check_output(['losetup', '--find', '--show', '--partscan', str(image)], text=True).strip()
  mounts = []
  try:
    if not loop.startswith('/dev/loop'):
      raise RuntimeError('Unexpected loop device')
    backing = subprocess.check_output(['losetup', '-n', '-O', 'BACK-FILE', loop], text=True).strip()
    if Path(backing).resolve() != image.resolve():
      raise RuntimeError('Unexpected loop backing file')
    # BLKROSET can survive loop-device reuse on the reference kernel. Clear it
    # only after proving this is our disposable file-backed output, never on a
    # physical disk. Wait for udev before using newly scanned partition nodes.
    run('udevadm', 'settle', '--timeout=10')
    run('blockdev', '--setrw', loop)
    for number in (1, 16, 17):
      run('blockdev', '--setrw', loop + f'p{number}')
    root, setup, data = [work / name for name in ('rootfs', 'setup', 'data')]
    for path in (root, setup, data):
      path.mkdir()
    run('mkfs.ext4', '-F', '-m', '0', '-L', 'CARROTDATA', loop + 'p17')
    for device, path in ((loop + 'p1', root), (loop + 'p16', setup), (loop + 'p17', data)):
      run('mount', device, path)
      mounts.append(path)
    audit_unprovisioned(root, setup)
    marker = refresh(root, setup, args.bundle)
    source = root / 'opt/carrot-jetlink/current/tools/jetlink'
    configure(root, source)
    runtime = root / 'opt/carrot-jetlink'
    # A factory recovery runtime is intentionally retained on immutable APP.
    # The updateable copy lives wholly on DATA, including transaction records.
    run('cp', '-a', '--', runtime, data / 'runtime')
    (data / 'runtime/protected-runtime.json').write_text('{"format":1}\n')
    marker.update(state='PROTECTED_CANDIDATE_NOT_BOOT_TESTED', storage_format=1,
                  data_partition=17, data_grow=False, image_bytes=40 * GIB)
    (root / 'etc/carrot-jetlink-image.json').write_text(json.dumps(marker, indent=2) + '\n')
    (setup / 'IMAGE.json').write_text(json.dumps(marker, indent=2) + '\n')
    audit_unprovisioned(root, setup)
    if (data / 'identity').exists() or (setup / 'identity').exists():
      raise ValueError('Private identity in candidate')
    run('sync', '-f', image)
    for path in reversed(mounts):
      run('umount', path)
    mounts.clear()
    for number in (1, 17):
      check = subprocess.run(['e2fsck', '-f', '-p', loop + f'p{number}'])
      if check.returncode not in (0, 1):
        raise RuntimeError('Candidate filesystem check failed')
    run('fsck.vfat', '-n', loop + 'p16')
  finally:
    for path in reversed(mounts):
      run('umount', path)
    run('losetup', '-d', loop)
  run('sgdisk', '-v', image)
  marker.update(image_sha256=digest(image), source_base_sha256=args.base_sha256,
                bundle_sha256=args.bundle_sha256)
  (work / 'candidate.json').write_text(json.dumps(marker, indent=2) + '\n')
  print('PRIVATE_CANDIDATE_ONLY: boot and power-cut validation still required')


if __name__ == '__main__':
  main()
