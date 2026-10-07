"""Make one self-contained Windows ZIP for the offline automatic-update patch."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from build_windows_installer import PYTHON_SHA, PYTHON_URL
from offline_boot_patch import validate_index


def digest(path):
  with path.open('rb') as stream:
    return hashlib.file_digest(stream, 'sha256').hexdigest()


def build(patch, python_zip, output):
  source = Path(__file__).resolve().parent
  index = json.loads((patch / 'boot-patch.json').read_text())
  first, _ = validate_index(index)
  if digest(python_zip) != PYTHON_SHA:
    raise ValueError('Portable Python release mismatch')
  if digest(patch / 'carrot-boot-update.zip') != index['payload_sha256']:
    raise ValueError('Boot payload mismatch')
  release = {'version': 'v0.4.1-boot-update-patch-preview',
                 'image_bytes': first['image_bytes'], 'patch_sha256': digest(patch / 'boot-patch.json'),
                 'payload_sha256': index['payload_sha256'], 'payload_bytes': index['payload_bytes'],
                 'host_source': index['host_source'], 'python_sha256': PYTHON_SHA, 'python_url': PYTHON_URL,
                 'physical_patch_tested': False, 'physical_boot_tested': False,
                 'validation': 'Preview: desktop tests and original/portable image overlay checks; physical patch/boot pending'}
  command = ('@echo off\r\nchcp 65001 >nul\r\n' +
             'if not exist "%~dp0support\\boot_update_patch.ps1" (\r\n' +
             'echo 먼저 모두 압축을 풀어 주세요.\r\necho Extract all files first.\r\npause >nul\r\nexit /b 1\r\n)\r\n' +
             'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0support\\boot_update_patch.ps1"\r\n' +
             'set "RESULT=%ERRORLEVEL%"\r\necho 아무 키나 누르면 닫습니다.\r\necho Press any key to close.\r\n' +
             'pause >nul\r\nexit /b %RESULT%\r\n')
  output.parent.mkdir(parents=True, exist_ok=True)
  temporary = output.with_suffix('.zip.partial')
  prefix = 'CarrotJetsonPatch/'
  with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED) as package:
    for name in ('boot-patch.json', 'carrot-boot-update.zip'):
      package.write(patch / name, prefix + 'support/' + name)
    for name in ('offline_boot_patch.py', 'offline_hotfix.py', 'apply_offline_hotfix_windows.ps1'):
      body = (source / name).read_text(encoding='utf-8-sig')
      package.writestr(prefix + 'support/' + name, body.encode('utf-8-sig' if name.endswith('.ps1') else 'utf-8'))
    for name in ('boot_update_patch.ps1', 'messages.ps1', 'disks.ps1'):
      package.writestr(prefix + 'support/' + name,
                       (source / 'windows_installer' / name).read_text(encoding='utf-8-sig').encode('utf-8-sig'))
    with zipfile.ZipFile(python_zip) as runtime:
      for entry in runtime.infolist():
        if entry.is_dir():
          continue
        if '/' in entry.filename or '\\' in entry.filename:
          raise ValueError('Unexpected portable Python layout')
        package.writestr(prefix + 'support/python/' + entry.filename, runtime.read(entry))
    package.writestr(prefix + 'support/boot-patch-release.json', json.dumps(release, indent=2) + '\n')
    package.writestr(prefix + 'Jetson_자동업데이트패치.cmd', command.encode('utf-8'))
    package.write(source / 'windows_installer/boot_update_patch.html', prefix + '패치안내.html')
    package.write(source.parents[1] / 'LICENSE', prefix + 'support/LICENSE-carrot.txt')
  temporary.replace(output)
  with zipfile.ZipFile(output) as package:
    if package.testzip() is not None:
      raise ValueError('Package CRC failure')
  release.update(zip_bytes=output.stat().st_size, zip_sha256=digest(output))
  output.with_name('release.json').write_text(json.dumps(release, indent=2) + '\n')
  output.with_name('SHA256SUMS').write_text(f'{release["zip_sha256"]}  {output.name}\n')
  print(json.dumps(release), flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('patch', type=Path)
  parser.add_argument('python_zip', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  build(args.patch, args.python_zip, args.output)
