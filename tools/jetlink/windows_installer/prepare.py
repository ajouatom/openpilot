"""Prepare a pinned image using the private, bundled Python (no pip required)."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_hotfix import apply


def digest(path):
  sha = hashlib.sha256()
  with path.open('rb') as stream:
    for block in iter(lambda: stream.read(8 << 20), b''):
      sha.update(block)
  return sha.hexdigest()


def check(path, size, sha):
  if path.stat().st_size != size or digest(path) != sha:
    raise RuntimeError(f'파일 검증 실패 / File verification failed: {path.name}. 설치파일을 다시 받아 압축을 풀어 주세요. / Download and extract again.')


def prepare(root, decompress=None):
  root = Path(root).resolve()
  release = json.loads((root / 'support/release.json').read_text(encoding='utf-8'))
  target = root / 'prepared.img'
  if target.exists():
    print('기존 준비 파일 검사 중 / Checking the prepared image. Please wait.', flush=True)
    check(target, release['image_bytes'], release['prepared_sha256'])
    print('준비 완료! 02_SD카드설치.cmd를 실행하세요. / Ready! Run 02 to install to the SD card.', flush=True)
    return
  if shutil.disk_usage(root).free < release['image_bytes'] + (1 << 30):
    raise RuntimeError('여유 공간 부족. 28GB 이상 비우고 다시 실행하세요. / Not enough space. Free at least 28 GB and retry.')
  source = root / 'support/carrot-jetson.img.zst'
  print('1/4 원본 이미지 검사 / Checking the original image.', flush=True)
  check(source, release['compressed_bytes'], release['compressed_sha256'])
  patch_file = root / 'support/offline-usbc.json'
  if digest(patch_file) != release['patch_sha256']:
    raise RuntimeError('USB-C 수정 파일 검증 실패. 설치파일을 다시 받으세요. / USB-C patch verification failed. Download again.')
  partial = root / 'prepared.img.partial'
  print('2/4 이미지 압축 해제 / Extracting the image. Do not close this window.', flush=True)
  if decompress is None:
    from compression.zstd import open as decompress
  sha = hashlib.sha256()
  total = 0
  next_report = 1 << 30
  with decompress(source, 'rb') as reader, partial.open('wb') as writer:
    while block := reader.read(8 << 20):
      total += len(block)
      if total > release['image_bytes']:
        raise RuntimeError('이미지 크기가 올바르지 않습니다. / Invalid image size.')
      writer.write(block)
      sha.update(block)
      if total >= next_report:
        print(f'  {total * 100 // release["image_bytes"]}%', flush=True)
        next_report += 1 << 30
    writer.flush()
    os.fsync(writer.fileno())
  if total != release['image_bytes'] or sha.hexdigest() != release['image_sha256']:
    raise RuntimeError('압축 해제 이미지 검증 실패. 01을 다시 실행하세요. / Extracted image verification failed. Retry 01.')
  print('3/4 USB-C 연결 수정 자동 적용 / Applying the USB-C connection fix.', flush=True)
  with partial.open('r+b', buffering=0) as stream:
    apply(stream, json.loads(patch_file.read_bytes()), sync=lambda: os.fsync(stream.fileno()), report=lambda *_: None)
  print('4/4 최종 이미지 검사 / Verifying the final installation image.', flush=True)
  check(partial, release['image_bytes'], release['prepared_sha256'])
  partial.replace(target)
  print('준비 완료! 02_SD카드설치.cmd를 실행하세요. / Ready! Run 02 to install to the SD card.', flush=True)


if __name__ == '__main__':
  try:
    prepare(Path(__file__).resolve().parent.parent)
  except Exception as error:
    print(f'준비 실패 / Preparation failed: {error}', file=sys.stderr, flush=True)
    sys.exit(1)
