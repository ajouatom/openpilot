"""Bundle a small add-on ZIP; reuse the existing installer's portable Python."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from offline_hotfix import load_manifest, validate


def build(patch_dir, output):
  source = Path(__file__).parent
  metadata = json.loads((patch_dir / 'sd-nvme-release.json').read_text())
  manifest = load_manifest(patch_dir / 'sd-nvme-patch.json.gz', metadata['patch_sha256'])
  validate(manifest)
  if (metadata['physical_boot_tested'] is not False or manifest['format'] != 2
      or metadata['patched_image_sha256'] != manifest['patched_image_sha256']):
    raise ValueError('Expected unpromoted R2 portable patch')
  script = ('@echo off\r\nchcp 65001 >nul\r\n'
            'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0support\\sd_nvme_patch.ps1"\r\n'
            'set "RESULT=%ERRORLEVEL%"\r\necho.\r\necho 아무 키나 누르면 닫습니다.\r\necho Press any key to close.\r\npause >nul\r\nexit /b %RESULT%\r\n')
  temporary = output.with_suffix('.zip.partial')
  output.parent.mkdir(parents=True, exist_ok=True)
  with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    for name in ('sd-nvme-patch.json.gz', 'sd-nvme-release.json'):
      archive.write(patch_dir / name, 'support/' + name)
    for name in ('offline_hotfix.py', 'apply_offline_hotfix_windows.ps1'):
      body = (source / name).read_text(encoding='utf-8-sig')
      archive.writestr('support/' + name, body.encode('utf-8-sig' if name.endswith('.ps1') else 'utf-8'))
    for name in ('sd_nvme_patch.ps1', 'messages.ps1', 'disks.ps1'):
      archive.writestr('support/' + name, (source / 'windows_installer' / name).read_text(encoding='utf-8-sig').encode('utf-8-sig'))
    archive.writestr('03_SD_SSD공용패치.cmd', script.encode('utf-8'))
    archive.write(source / 'windows_installer/sd_nvme_patch.html', 'SD_SSD_패치안내.html')
  temporary.replace(output)
  with zipfile.ZipFile(output) as archive:
    if archive.testzip() is not None:
      raise ValueError('Package CRC failure')
  metadata.update(package_type='PATCH_ONLY_REQUIRES_EXISTING_R2_INSTALLER', zip_bytes=output.stat().st_size,
                  zip_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
  output.with_name('release.json').write_text(json.dumps(metadata, indent=2) + '\n')
  output.with_name('SHA256SUMS').write_text(f'{metadata["zip_sha256"]}  {output.name}\n')
  print(json.dumps(metadata))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('patch_dir', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  build(args.patch_dir, args.output)
