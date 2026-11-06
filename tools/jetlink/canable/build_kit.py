"""Build offline beginner guides and PC/Jetson ZIPs from reviewed sources."""
import argparse
import hashlib
import html
from pathlib import Path
import shutil
import tarfile
import zipfile

import markdown

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
DFU_SHA = '6450de30a7dcd8d8c1273f43f0b153f054fd24d85f7f38296b1ad8edbd2ddb25'
SERIAL_SHA = 'c4451db6ba391ca6ca299fb3ec7bae67a5c55dde170964c7a14ceefec02f2cf0'
STYLE = '''body{font-family:system-ui,"Malgun Gothic",sans-serif;background:#f3f6fa;color:#182b3b;margin:0;line-height:1.8}main{max-width:980px;margin:30px auto;padding:30px;background:white;border-radius:16px}a{color:#075fba}nav{display:flex;gap:22px;flex-wrap:wrap;border-bottom:1px solid #dce4ec;padding-bottom:16px}h1{font-size:30px;line-height:1.4}h2{margin-top:42px;border-top:1px solid #dce4ec;padding-top:20px}img{max-width:100%;height:auto}table{border-collapse:collapse;width:100%}th,td{border:1px solid #d1dbe5;padding:12px;text-align:left}th{background:#eaf1f8}pre{background:#10283b;color:#eaf7ff;padding:18px;overflow:auto;border-radius:8px}code{font-size:.94em}li{margin:.5em 0}@media(max-width:700px){main{margin:0;padding:18px;border-radius:0}h1{font-size:25px}td,th{padding:7px}}'''


def check(path, expected):
  if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
    raise ValueError('Dependency hash mismatch: '+str(path))


def guide(target):
  target.mkdir(parents=True, exist_ok=True)
  (target/'assets').mkdir(exist_ok=True)
  for name in ('canable-mks-board.png', 'wiring-canable.svg'):
    shutil.copyfile(REPO/'docs/user/assets'/name, target/'assets'/name)
  shutil.copyfile(REPO/'tools/carrot_b1/starter-photo.png', target/'assets/s3-starter-photo.png')
  for name in ('indicator-adapters', 'canable', 's3-can', 'peak-can'):
    text = (REPO/'docs/user/ko'/f'{name}.md').read_text(encoding='utf8')
    text = text.replace('../assets/', 'assets/').replace('../../../tools/carrot_b1/starter-photo.png', 'assets/s3-starter-photo.png')
    for page in ('indicator-adapters', 'canable', 's3-can', 'peak-can'):
      text = text.replace(']('+page+'.md)', ']('+('START' if page=='indicator-adapters' else page)+'.html)')
    text = text.replace('../../../tools/', 'https://github.com/ajouatom/openpilot/blob/carrot-wip/tools/')
    text = text.replace('../en/', 'https://github.com/ajouatom/openpilot/blob/carrot-wip/docs/user/en/')
    body = markdown.markdown(text, extensions=['tables','fenced_code'])
    title = html.escape(text.splitlines()[0].lstrip('# '))
    document = f'<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title><style>{STYLE}</style><main><nav><a href="START.html">지원 장치 목록</a><a href="canable.html">CANable</a><a href="s3-can.html">S3 BLE</a><a href="peak-can.html">PEAK</a></nav>{body}</main></html>'
    (target/('START.html' if name=='indicator-adapters' else name+'.html')).write_text(document, encoding='utf8')


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--exe', type=Path, required=True)
  p.add_argument('--dfu', type=Path, required=True)
  p.add_argument('--serial-wheel', type=Path, required=True)
  p.add_argument('--output', type=Path, required=True)
  a = p.parse_args()
  check(a.dfu, DFU_SHA);check(a.serial_wheel, SERIAL_SHA)
  a.output.mkdir(parents=True, exist_ok=True)
  for platform in ('Windows','Jetson'):
    directory = a.output/('Carrot-CANable-v2-'+platform)
    directory.mkdir(exist_ok=False)
    guide(directory)
    for name in ('install_canable.py','canable2-carrot-b1-slcan-2.bin','firmware.json','README.md','LICENSE.md'):
      shutil.copyfile(ROOT/name,directory/name)
    with zipfile.ZipFile(a.serial_wheel) as wheel:
      for name in wheel.namelist():
        if name.startswith('serial/') and not name.endswith('/') or name.endswith('/LICENSE.txt'):
          output = directory/'vendor'/name
          output.parent.mkdir(parents=True,exist_ok=True)
          output.write_bytes(wheel.read(name))
    if platform == 'Windows':
      shutil.copyfile(a.exe,directory/'Install-CANable.exe')
      with tarfile.open(a.dfu) as archive:
        for name in ('win64/dfu-util.exe','win64/libusb-1.0.dll','COPYING','README','README-bin.txt','AUTHORS'):
          data = archive.extractfile('dfu-util-0.11-binaries/'+name).read()
          output = directory/'dfu-util'/Path(name).name
          output.parent.mkdir(exist_ok=True)
          output.write_bytes(data)
      (directory/'dfu-util/SOURCE.txt').write_text('Unmodified dfu-util 0.11 official distribution.\nSource: https://dfu-util.sourceforge.net/releases/dfu-util-0.11.tar.gz\nBinaries: https://dfu-util.sourceforge.net/releases/dfu-util-0.11-binaries.tar.xz\nlibusb: https://libusb.info/\n')
    with zipfile.ZipFile(directory.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
      for item in sorted(directory.rglob('*')):
        if item.is_file():archive.write(item,item.relative_to(directory))
    print(directory.with_suffix('.zip'))


if __name__ == '__main__':main()
