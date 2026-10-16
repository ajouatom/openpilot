#!/usr/bin/env python3
"""Package an ESP-IDF build without ever reading a device's private NVS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
VERSION = '1.2.0'
VENDOR_HASHES = {
  'esptool-js-bundle.js': '475e0e98fd051dc11bf34d8f83410764da11f3a0c40ecd33fd9647dc46b098b8',
  'esptool-js-LICENSE.txt': '1c25f29242785d63e9adb0be7fdc137551dfbbc0756b622fcd26d9cd7de3a4f3',
  'pako-LICENSE.txt': 'a04665b3b2de56c66730c1f720f528175739e4104f79073614aa611da1e85539',
}


def digest(data, algorithm='sha256'):
  return hashlib.new(algorithm, data).hexdigest()


def package(build: Path, output: Path, vendor: Path, sdkconfig: Path):
  output.mkdir(parents=True, exist_ok=True)
  sources = [('bootloader/bootloader.bin', 'bootloader.bin', 0, 0x8000),
             ('partition_table/partition-table.bin', 'partition-table.bin', 0x8000, 0x1000)]
  app = next((p for p in [build/'carrot_s3_display.bin', build/'s3_usb_probe.bin'] if p.is_file()), None)
  if app is None:
    raise ValueError('Built application not found')
  sources.append((app.name, 'carrot-s3.bin', 0x10000, 0x300000))
  parts = []
  for source, name, offset, maximum in sources:
    data = (build/source).read_bytes()
    if not 0 < len(data) <= maximum or len(data) % 4:
      raise ValueError(f'Invalid binary size: {name}')
    if offset == 0:
      if data[0] != 0xe9 or data[2:4] != bytes([2, 0x4f]):
        raise ValueError('Bootloader must be preconfigured DIO / 16 MB / 80 MHz')
    elif offset == 0x8000:
      rows = [struct.unpack('<HBBII16sI', data[i:i+32]) for i in (0, 32, 64)]
      expected = [(1, 2, 0x9000, 0x6000), (1, 1, 0xf000, 0x1000), (0, 0, 0x10000, 0x300000)]
      if any(row[0] != 0x50aa or row[1:5] != want for row, want in zip(rows, expected, strict=True)):
        raise ValueError('Partition table is not the supported CARROT layout')
    elif VERSION.encode()+b'\0' not in data[0x20:0x120]:
      raise ValueError('Application version does not match release')
    (output/name).write_bytes(data)
    parts.append({'path':name, 'offset':offset, 'size':len(data), 'sha256':digest(data), 'md5':digest(data, 'md5')})
  shutil.copytree(HERE/'installer', output, dirs_exist_ok=True)
  (output/'vendor').mkdir(exist_ok=True)
  for name, expected in VENDOR_HASHES.items():
    if digest((vendor/name).read_bytes()) != expected:
      raise ValueError('Unverified installer dependency: '+name)
    shutil.copyfile(vendor/name, output/'vendor'/name)
  for name in ['INSTALL_KO.md', 'README.md']:
    shutil.copyfile(HERE/name, output/name)
  shutil.copytree(HERE/'firmware', output/'source/firmware', dirs_exist_ok=True,
                  ignore=shutil.ignore_patterns('build', 'sdkconfig', 'sdkconfig.old'))
  shutil.copyfile(sdkconfig, output/'source/sdkconfig.release')
  config = sdkconfig.read_text(encoding='utf8')
  for setting in ['CONFIG_SPIRAM_MODE_OCT=y', 'CONFIG_SPIRAM_SPEED_80M=y', 'CONFIG_USB_HOST_HUBS_SUPPORTED=y', 'CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y']:
    if setting not in config:
      raise ValueError('Missing build setting: '+setting)
  source_hashes = {str(p.relative_to(HERE)).replace('\\', '/'):digest(p.read_bytes())
                   for p in sorted((HERE/'firmware').rglob('*')) if p.is_file() and 'build' not in p.parts and p.name not in ('sdkconfig', 'sdkconfig.old')}
  revision = subprocess.check_output(['git', '-C', str(HERE), 'rev-parse', 'HEAD'], text=True).strip()
  manifest = {'version':VERSION, 'chip':'ESP32-S3', 'board':'N16R8', 'idf':'5.4.0',
              'source_commit':revision, 'firmware_sources':source_hashes, 'parts':parts,
              'esptool_js':'0.5.6', 'nvs_preserved':[0x9000, 0xf000]}
  (output/'release.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf8')
  (output/'OPEN-INSTALLER.url').write_text('[InternetShortcut]\nURL=https://upload.shind0.synology.me/models/carrot-s3/index.html\n', encoding='ascii')
  zip_path=output/f'carrot-s3-{VERSION}.zip'
  files=[p for p in sorted(output.rglob('*')) if p.is_file() and p.name not in (zip_path.name, 'SHA256SUMS.txt')]
  with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as archive:
    for path in files:
      archive.write(path, str(path.relative_to(output)))
  files.append(zip_path)
  (output/'SHA256SUMS.txt').write_text(''.join(f'{digest(p.read_bytes())}  {p.relative_to(output).as_posix()}\n' for p in files), encoding='ascii')
  print(json.dumps({'version':VERSION, 'app_bytes':parts[-1]['size'], 'zip_bytes':zip_path.stat().st_size, 'files':len(files)}))


if __name__ == '__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  for name in ['build', 'output', 'vendor', 'sdkconfig']:
    parser.add_argument('--'+name, required=True, type=Path)
  args=parser.parse_args()
  package(args.build, args.output, args.vendor, args.sdkconfig)
