"""Build the pinned MD v3 Mountain Dew NAS package from verified upstream downloads."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import shutil
import tarfile

MODEL_SHA = 'd4c7a3fcfc95813826ca871a80bea18df833aacb2eb6f5333e25a01e65b1a422'
MODEL_SIZE = 799941965
SOURCE = 'e86b58dc7b9fb499dd84ac6dbe514a03fa2bc428'
TINYGRAD = '02e698160eed1751738212144a7ea2cfb160b3f1'
SOURCE_TAR_SHA = '2824b933fbf48df672cbb18c8462bd7f9d637a065f4c510ae3d07dedc01ffcb9'
CHECKPOINT = 'f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/1365126a-e71e-4778-be00-8ab92dc82c09/12864'


def digest(path):
  with path.open('rb') as stream:
    return hashlib.file_digest(stream, 'sha256').hexdigest()


def build(model, source_tar, output):
  if model.stat().st_size != MODEL_SIZE or digest(model) != MODEL_SHA:
    raise ValueError('MDM model identity mismatch')
  if digest(source_tar) != SOURCE_TAR_SHA:
    raise ValueError('pinned tinygrad source archive mismatch')
  output.mkdir(parents=True, exist_ok=True)
  runtime = output / 'precompiled-runtime.tar.gz'
  with source_tar.open('rb') as source, tarfile.open(fileobj=source, mode='r:gz') as upstream:
    with runtime.open('wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', filename='', mtime=0) as compressed:
      with tarfile.open(fileobj=compressed, mode='w') as archive:
        for member in sorted(upstream.getmembers(), key=lambda m: m.name):
          name = member.name.partition('/')[2]
          if not member.isfile() or not (name.startswith(('tinygrad/', 'examples/openpilot/')) or name == 'LICENSE'):
            continue
          if '..' in Path(name).parts:
            raise ValueError('invalid source archive path')
          data = upstream.extractfile(member).read()
          info = tarfile.TarInfo(name)
          info.size, info.mode, info.mtime = len(data), 0o644, 0
          archive.addfile(info, io.BytesIO(data))
  manifest = {'model_id': 'comma-pr39087-md-v3-e86b58dc-d4c7a3fc', 'filename': 'big_driving_tinygrad.pkl',
              'size': MODEL_SIZE, 'sha256': MODEL_SHA, 'url': 'big_driving_tinygrad.pkl'}
  catalog = {'protocol': 1, 'format': 'comma-generic-onnx', 'serialization': 'persistent-buffer-v1',
             'gpu_arch': 'gfx1200', 'frame_skip': 4, 'camera_resolutions': [[1928, 1208], [1344, 760]],
             'model_sha256': MODEL_SHA, 'model_checkpoint': CHECKPOINT, 'source_commit': SOURCE, 'tinygrad_commit': TINYGRAD,
             'pickle': {'url': manifest['url'], 'size': MODEL_SIZE, 'sha256': MODEL_SHA},
             'runtime': {'url': runtime.name, 'size': runtime.stat().st_size, 'sha256': digest(runtime)}}
  for name, value in [('manifest.json', manifest), ('precompiled.json', catalog)]:
    (output / name).write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
  shutil.copyfile(model, output / manifest['filename'])
  print(json.dumps(catalog, indent=2))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('model', type=Path)
  parser.add_argument('source_tar', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  build(args.model, args.source_tar, args.output)
