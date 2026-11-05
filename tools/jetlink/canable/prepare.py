"""Prepare pinned CANable sources; requires Python 3.12+ and patch, never flashes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parent


def verified(path, url, digest):
  if not path.exists():
    with urllib.request.urlopen(url, timeout=60) as source:
      path.write_bytes(source.read())
  if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
    raise ValueError('Source hash mismatch: '+str(path))
  return path


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('work', type=Path, help='New output directory')
  args = parser.parse_args()
  args.work.mkdir(parents=True, exist_ok=False)
  manifest = json.loads((ROOT/'firmware.json').read_text())
  archive = verified(args.work/'source.tar.gz',
                     'https://api.github.com/repos/normaldotcom/canable2-fw/tarball/'+manifest['source_commit'],
                     manifest['source_archive_sha256'])
  with tarfile.open(archive) as source:
    source.extractall(args.work/'source', filter='data')
  tree, = (args.work/'source').iterdir()
  patch = verified(ROOT/'carrot-b1.patch', '', manifest['files']['carrot-b1.patch'])
  subprocess.run(['patch', '-p1', '--batch', '-i', str(patch)], cwd=tree, check=True)
  for name, digest in manifest['printf'].items():
    source = verified(args.work/name,
                      'https://raw.githubusercontent.com/mpaland/printf/'+manifest['printf_commit']+'/'+name, digest)
    shutil.copyfile(source, tree/'src'/name)
  shutil.copyfile(args.work/'printf.h', tree/'inc/printf.h')
  print(tree)
  print('Build there with arm-none-eabi-gcc/newlib installed:')
  print('make -j2 GIT_VERSION=carrot-b1-slcan-2 GIT_REMOTE=github.com/normaldotcom/canable2.git')


if __name__ == '__main__':
  main()
