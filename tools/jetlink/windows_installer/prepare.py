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
    raise RuntimeError(f'파일 검증 실패: {path.name}. ZIP을 다시 내려받아 압축을 풀어 주세요.')


def prepare(root, decompress=None):
  root = Path(root).resolve()
  release = json.loads((root / 'support/release.json').read_text(encoding='utf-8'))
  target = root / 'prepared.img'
  if target.exists():
    print('기존 준비 파일을 검사합니다. 잠시 기다려 주세요.', flush=True)
    check(target, release['image_bytes'], release['prepared_sha256'])
    print('준비 완료! 02_SD카드설치.cmd를 실행하세요.', flush=True)
    return
  if shutil.disk_usage(root).free < release['image_bytes'] + (1 << 30):
    raise RuntimeError('이 드라이브에 여유 공간이 부족합니다. 최소 26GB를 비우고 다시 실행하세요.')
  source = root / 'support/carrot-jetson.img.zst'
  print('1/4 포함된 원본 이미지를 검사합니다.', flush=True)
  check(source, release['compressed_bytes'], release['compressed_sha256'])
  patch_file = root / 'support/offline-usbc.json'
  if digest(patch_file) != release['patch_sha256']:
    raise RuntimeError('USB-C 수정 파일 검증 실패. ZIP을 다시 내려받아 주세요.')
  partial = root / 'prepared.img.partial'
  print('2/4 이미지 압축을 풉니다. 창을 닫지 마세요.', flush=True)
  if decompress is None:
    from compression.zstd import open as decompress
  sha = hashlib.sha256()
  total = 0
  next_report = 1 << 30
  with decompress(source, 'rb') as reader, partial.open('wb') as writer:
    while block := reader.read(8 << 20):
      total += len(block)
      if total > release['image_bytes']:
        raise RuntimeError('이미지 크기가 올바르지 않습니다.')
      writer.write(block)
      sha.update(block)
      if total >= next_report:
        print(f'  {total * 100 // release["image_bytes"]}%', flush=True)
        next_report += 1 << 30
    writer.flush()
    os.fsync(writer.fileno())
  if total != release['image_bytes'] or sha.hexdigest() != release['image_sha256']:
    raise RuntimeError('압축 해제 이미지 검증 실패. 01을 다시 실행하세요.')
  print('3/4 USB-C 연결 수정을 자동으로 적용합니다.', flush=True)
  with partial.open('r+b', buffering=0) as stream:
    apply(stream, json.loads(patch_file.read_bytes()), sync=lambda: os.fsync(stream.fileno()), report=lambda *_: None)
  print('4/4 설치할 최종 이미지를 검사합니다.', flush=True)
  check(partial, release['image_bytes'], release['prepared_sha256'])
  partial.replace(target)
  print('준비 완료! 02_SD카드설치.cmd를 실행하세요.', flush=True)


if __name__ == '__main__':
  try:
    prepare(Path(__file__).resolve().parent.parent)
  except Exception as error:
    print(f'준비를 완료하지 못했습니다: {error}', file=sys.stderr, flush=True)
    sys.exit(1)
