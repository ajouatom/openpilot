"""Build the pinned Light speed 2 NAS package from verified upstream downloads."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import shutil
import tarfile

MODEL_SHA = 'b4dead3ee091d66ab002a6b8e716c26f4f79e3c931e4be5f48e017ba92f6e960'
MODEL_SIZE = 767228907
SOURCE = 'b92fc4828664f8767bce5e92f1630ed319cbe509'
TINYGRAD = 'ec91b74b5b6b0fc924b72d3f27689679ca2438b9'
SOURCE_TAR_SHA = 'c29c0343a113e4f051087143968d05eb99d52f9a7875ba65f1d274361facc29d'
CHECKPOINT = 'f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/ea2f516f-f758-42be-a245-913bb818a64f'


def digest(path):
  with path.open('rb') as stream:
    return hashlib.file_digest(stream, 'sha256').hexdigest()


def build(model, source_tar, output):
  if model.stat().st_size != MODEL_SIZE or digest(model) != MODEL_SHA:
    raise ValueError('Light speed 2 model identity mismatch')
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
  manifest = {'model_id': 'comma-light-speed-2-b92fc482-b4dead3e', 'filename': 'big_driving_tinygrad.pkl',
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
