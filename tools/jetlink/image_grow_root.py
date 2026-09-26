"""Expand only the marked Carrot image's last APP partition on its SD card."""
import json
import os
from pathlib import Path
import subprocess


def run(*args):
  return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def validate_layout(layout, expected_start, disk_sectors):
  if layout['label'] != 'gpt' or layout['sectorsize'] != 512:
    raise ValueError('Unexpected SD partition table')
  parts = layout['partitions']
  app = next(p for p in parts if p['node'] == '/dev/mmcblk0p1')
  setup = next(p for p in parts if p['node'] == '/dev/mmcblk0p16')
  if len(parts) != 16 or app['name'] != 'APP' or app['start'] != expected_start or setup['name'] != 'CARROT_SETUP':
    raise ValueError('Not the expected Carrot SD layout')
  if any(p['start'] + p['size'] > app['start'] for p in parts if p is not app):
    raise ValueError('APP must be the last physical partition')
  if app['start'] + app['size'] > disk_sectors - 33:
    raise ValueError('APP exceeds disk capacity')
  return app


def main():
  if os.geteuid() != 0:
    raise RuntimeError('Root required')
  marker = json.loads(Path('/etc/carrot-jetlink-image.json').read_text())
  state = Path('/var/lib/carrot-jetlink/root-expanded.json')
  if state.exists():
    return
  if run('findmnt', '-n', '-o', 'SOURCE', '/').strip() != '/dev/mmcblk0p1':
    raise RuntimeError('Expected SD root filesystem')
  layout = json.loads(run('sfdisk', '--json', '/dev/mmcblk0'))['partitiontable']
  sectors = int(run('blockdev', '--getsz', '/dev/mmcblk0'))
  app = validate_layout(layout, marker['root_start'], sectors)
  if sectors - app['start'] - app['size'] > 4096:
    run('sgdisk', '-e', '/dev/mmcblk0')
    # Explicit end avoids parted's interactive GPT-repair prompt.
    run('parted', '--script', '/dev/mmcblk0', 'unit', 's', 'resizepart', '1', str(sectors - 34) + 's')
    run('partx', '--update', '--nr', '1', '/dev/mmcblk0')
  run('resize2fs', '/dev/mmcblk0p1')
  state.parent.mkdir(parents=True, exist_ok=True)
  state.write_text(json.dumps({'disk_sectors': sectors}) + '\n')
  os.sync()


if __name__ == '__main__':
  main()
