"""Publisher: bundle a verified image and portable tools, with optional legacy patch."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

from offline_hotfix import validate

IMAGE_SHA = '5a1e7a3ba6156c621d8a01412f062b6c16ecb8ef2274b82b6acdaadf4b19a516'
COMPRESSED_SHA = '61fce013d1fb9db548084ca4d2e9a3ea9d697717b1470725f0fec39830fc3285'
PATCH_SHA = '1ba794698dd136e6fc089891a5711ca4fcbba826335dc960e4f2b41465ea539a'
PYTHON_SHA = 'd297e5ff019966817ad8502465176139f2d3d840fa4ed84b13bed399a6ab1f15'
PYTHON_URL = 'https://www.python.org/ftp/python/3.14.7/python-3.14.7-embed-amd64.zip'


def digest(path):
  with path.open('rb') as stream:
    return hashlib.file_digest(stream, 'sha256').hexdigest()


def patched_digest(image, manifest):
  patches = validate(manifest)
  original, result = hashlib.sha256(), hashlib.sha256()
  position = 0
  with image.open('rb') as stream:
    while block := stream.read(8 << 20):
      original.update(block)
      changed = bytearray(block)
      for offset, before, after in patches:
        low, high = max(position, offset), min(position + len(block), offset + len(before))
        if low < high:
          if block[low-position:high-position] != before[low-offset:high-offset]:
            raise ValueError('Patch bytes do not match original image')
          changed[low-position:high-position] = after[low-offset:high-offset]
      result.update(changed)
      position += len(block)
  if original.hexdigest() != IMAGE_SHA:
    raise ValueError('Wrong base image')
  return result.hexdigest()


def integrated_release(image, compressed, metadata, validation=None, owner_boot=None):
  if validation is not None and owner_boot is not None:
    raise ValueError('Choose measured validation or owner-reported boot, not both')
  value = json.loads(metadata.read_text(encoding='utf-8'))
  if (value.get('storage_format') != 1 or value.get('data_partition') != 17
      or value.get('image_bytes') != 40 * (1 << 30)
      or not re.fullmatch('[0-9a-f]{40}', value.get('source_commit', ''))
      or value.get('state') != 'PROTECTED_CANDIDATE_NOT_BOOT_TESTED'):
    raise ValueError('Expected audited protected candidate metadata')
  for path, size, sha in ((image, value['image_bytes'], value['image_sha256']),
                          (compressed, value['compressed_bytes'], value['compressed_sha256'])):
    if path.stat().st_size != size or digest(path) != sha:
      raise ValueError('Protected candidate file/hash mismatch')
  release = dict(version='v0.4.0-storage-candidate', preparation='integrated',
              image_bytes=value['image_bytes'], image_sha256=value['image_sha256'],
              prepared_sha256=value['image_sha256'], compressed_bytes=value['compressed_bytes'],
              compressed_sha256=value['compressed_sha256'], source_commit=value['source_commit'],
              storage_format=1, validation='PRIVATE CANDIDATE: physical boot and power-loss tests pending')
  if owner_boot is not None:
    evidence = json.loads(owner_boot.read_text(encoding='utf-8'))
    if (evidence.get('image_sha256') != value['image_sha256']
        or evidence.get('card_readback') is not True
        or evidence.get('owner_reported_boot') is not True
        or evidence.get('publication_requested') is not True):
      raise ValueError('Owner preview requires exact-image readback, reported boot and publication request')
    # A user report is useful evidence but does not stand in for instrumented tests.
    release.update(version='v0.4.0-boot-preview',
                   validation='Full PC card readback passed; owner reports successful boot/operation. Instrumented device checks pending.',
                   owner_reported_boot=True, validation_basis='owner_report',
                   validation_checks=dict(card_readback=True, protected_boot=False, model_ready=False,
                                          wifi_reconnect=False, ssh_persistence=False, data_update=False),
                   power_cut_cycles=0)
  elif validation is not None:
    evidence = json.loads(validation.read_text(encoding='utf-8'))
    checks = ('card_readback', 'protected_boot', 'model_ready', 'wifi_reconnect', 'ssh_persistence', 'data_update')
    if (evidence.get('image_sha256') != value['image_sha256']
        or any(evidence.get(name) is not True for name in checks)
        or type(evidence.get('power_cut_cycles')) is not int or evidence['power_cut_cycles'] < 0):
      raise ValueError('Physical validation must identify this exact image and completed checks')
    release.update(version='v0.4.0-protected-preview',
                   validation='Parked reference-device validation; see validation_checks for scope',
                   validation_checks={name: True for name in checks}, power_cut_cycles=evidence['power_cut_cycles'])
  return release


def candidate_guide(text, release):
  """Keep the bilingual layout; never link a private candidate to the old download."""
  if release['version'] in ('v0.4.0-protected-preview', 'v0.4.0-boot-preview'):
    button = (f'<a class="download" href="https://upload.shind0.synology.me/downloads/jetson/{release["version"]}/carrot-jetson-windows.zip">'
              f'설치파일 받기<span class="en" lang="en">Download installer · {release["compressed_bytes"] / 1e9:.1f} GB</span></a>')
    if release['version'] == 'v0.4.0-boot-preview':
      ko = '공개 시험판: 카드 전체 검증 완료·사용자 정상 동작 확인. 자동 업데이트 전환·Wi-Fi 복구·전원 차단 시험은 별도 확인 전입니다.'
      en = 'Public preview: full card verification passed; owner reports normal operation. Update activation, Wi-Fi recovery and power-cut tests remain pending.'
    else:
      ko = '시험판: 기준 장치의 주차 상태에서 기록·부팅·업데이트를 확인했습니다. 주행 시험은 별도입니다.'
      en = 'Preview: card writing, boot and updates tested on a parked reference device. Driving is a separate test.'
    text = text.replace('시험판: 실제 카드 기록·첫 부팅 시험은 남아 있습니다.', ko)
    text = text.replace('Preview: physical card writing and first-boot testing are pending.', en)
  else:
    button = '<div class="download">시험용 설치파일<span class="en" lang="en">Private test installer · not released</span></div>'
  text = re.sub(r'<a class="download".*?</a>', button, text)
  # Download + extracted compressed payload + raw image + workspace margin.
  gb = 5 * ((release['image_bytes'] + 2 * release['compressed_bytes'] + 5 * (1 << 30) + 4999999999) // 5000000000)
  text = text.replace('약 45GB', f'약 {gb}GB').replace('About 45 GB', f'About {gb} GB')
  text = text.replace('약 15~60분', '약 30~90분').replace('About 15–60 min', 'About 30–90 min')
  text = text.replace('이미지 검사·압축 해제·USB-C 수정까지', '수정사항이 포함된 이미지 검사·압축 해제·최종 검증까지')
  return text.replace('Image checks, extraction and the USB-C fix', 'Image checks, extraction and final verification')


def build(image, compressed, patch, python_zip, output, candidate_json=None, validation_json=None, owner_boot_json=None):
  if (validation_json is not None or owner_boot_json is not None) and candidate_json is None:
    raise ValueError('Physical validation is only supported for an integrated image')
  inputs = [(python_zip, PYTHON_SHA)]
  if candidate_json is None:
    inputs += [(compressed, COMPRESSED_SHA), (patch, PATCH_SHA)]
  elif patch is not None:
    raise ValueError('Integrated images cannot also apply a legacy patch')
  for path, expected in inputs:
    if digest(path) != expected:
      raise ValueError(f'Wrong publisher input: {path.name}')
  tools = Path(__file__).resolve().parent
  release = integrated_release(image, compressed, candidate_json, validation_json, owner_boot_json) if candidate_json else dict(version='v0.3.2-windows-preview', image_bytes=image.stat().st_size, image_sha256=IMAGE_SHA,
                 compressed_bytes=compressed.stat().st_size, compressed_sha256=COMPRESSED_SHA,
                 prepared_sha256=patched_digest(image, json.loads(patch.read_bytes())), patch_sha256=PATCH_SHA,
                 python_url=PYTHON_URL, python_sha256=PYTHON_SHA,
                 validation='PC preparation tested; physical card writing and first boot pending')
  release.update(python_url=PYTHON_URL, python_sha256=PYTHON_SHA)
  temporary = output.with_suffix('.zip.partial')
  output.parent.mkdir(parents=True, exist_ok=True)
  prefix = 'CarrotJetson/'
  with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as package:
    def add(name, data):
      package.writestr(prefix + name, data)
    package.write(compressed, prefix + 'support/carrot-jetson.img.zst')
    if patch is not None:
      package.write(patch, prefix + 'support/offline-usbc.json')
    for name in ('offline_hotfix.py', 'write_sd_windows.ps1'):
      data = (tools / name).read_text(encoding='utf-8-sig')
      add('support/' + name, data.encode('utf-8-sig' if name.endswith('.ps1') else 'utf-8'))
    for name in ('prepare.py', 'launcher.ps1', 'disks.ps1', 'messages.ps1'):
      data = (tools / 'windows_installer' / name).read_text(encoding='utf-8-sig')
      add('support/' + name, data.encode('utf-8-sig' if name.endswith('.ps1') else 'utf-8'))
    with zipfile.ZipFile(python_zip) as runtime:
      for entry in runtime.infolist():
        if entry.is_dir():
          continue
        if '/' in entry.filename or '\\' in entry.filename:
          raise ValueError('Unexpected portable Python layout')
        add('support/python/' + entry.filename, runtime.read(entry))
    add('support/release.json', json.dumps(release, ensure_ascii=False, indent=2).encode('utf-8'))
    for stage, filename in [('Prepare', '01_설치준비.cmd'), ('Install', '02_SD카드설치.cmd')]:
      script = ('@echo off\r\nchcp 65001 >nul\r\n'
                'if not exist "%~dp0support\\launcher.ps1" (\r\n'
                '  echo 먼저 모두 압축을 풀어 주세요.\r\n  echo Extract all files before running.\r\n  echo 아무 키나 누르면 닫습니다.\r\necho Press any key to close.\r\n  pause >nul\r\n  exit /b 1\r\n)\r\n'
                'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0support\\launcher.ps1" -Stage ' + stage + '\r\n'
                'set "RESULT=%ERRORLEVEL%"\r\necho 아무 키나 누르면 닫습니다.\r\necho Press any key to close.\r\npause >nul\r\nexit /b %RESULT%\r\n')
      add(filename, script.encode('utf-8'))
    for name in ('먼저읽기.txt', '설치안내.html'):
      content = (tools / 'windows_installer' / name).read_bytes()
      if candidate_json and name.endswith('.html'):
        content = candidate_guide(content.decode('utf-8'), release).encode('utf-8')
      add(name, content)
    add('support/SOURCE.txt', b'https://github.com/ajouatom/carrot-jetson\nhttps://www.python.org/downloads/release/python-3147/\n')
    add('support/LICENSE-carrot.txt', (tools.parents[1] / 'LICENSE').read_bytes())
  temporary.replace(output)
  release['zip_bytes'] = output.stat().st_size
  release['zip_sha256'] = digest(output)
  output.with_name('release.json').write_text(json.dumps(release, indent=2)+'\n', encoding='utf-8')
  output.with_name('SHA256SUMS').write_text(f'{release["zip_sha256"]}  {output.name}\n', encoding='ascii')
  print(json.dumps(release), flush=True)


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  for name in ('image', 'compressed', 'python-zip', 'output'):
    p.add_argument('--'+name, type=Path, required=True)
  mode = p.add_mutually_exclusive_group(required=True)
  mode.add_argument('--patch', type=Path)
  mode.add_argument('--candidate-json', type=Path)
  evidence = p.add_mutually_exclusive_group()
  evidence.add_argument('--validation-json', type=Path, help='Completed instrumented checks for this exact integrated image')
  evidence.add_argument('--owner-boot-json', type=Path, help='Explicit publication request and owner-reported boot; unmeasured checks remain pending')
  a = p.parse_args()
  build(a.image, a.compressed, a.patch, a.python_zip, a.output, a.candidate_json, a.validation_json, a.owner_boot_json)
