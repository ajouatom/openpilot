"""Build the pinned navigation driving model NAS package from verified upstream downloads."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import shutil
import tarfile

MODEL_SHA = '20cae5d01d0c59352fa6bf23cb0bf2a9c6c7257f6b9997652f6ef64669e194f8'
MODEL_SIZE = 822465506
SOURCE = 'a03bb1d4e80050cae0007780ecb8eb7cf06d0cb9'
TINYGRAD = 'd3f09c9bbd542fbfbe68c8569a1173550c408969'
SOURCE_TAR_SHA = 'bcb4dbf8538879a7d3f2dc1968bca2b9c63ac2cc81489c54640a05564149464f'
CHECKPOINT = 'cf8f6120-49ee-48f8-b2d7-d663a4708271/56320/80eedcdd-f97b-46d0-9b94-780685292acf/12864'
ONNX_SHA = '1d18c42ea4cd95c8bffab4635844b78b8e2d4e25b9de44a9c5ac2e0961371a2e'
ONNX_SIZE = 814591106


def digest(path):
  with path.open('rb') as stream:
    return hashlib.file_digest(stream, 'sha256').hexdigest()


def build(model, source_tar, output, onnx):
  if model.stat().st_size != MODEL_SIZE or digest(model) != MODEL_SHA:
    raise ValueError('navigation model identity mismatch')
  if onnx.stat().st_size != ONNX_SIZE or digest(onnx) != ONNX_SHA:
    raise ValueError('navigation ONNX identity mismatch')
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
  manifest = {'model_id': 'comma-nav-rl-a03bb1d4-20cae5d0', 'filename': 'big_driving_tinygrad.pkl',
              'size': MODEL_SIZE, 'sha256': MODEL_SHA, 'url': 'big_driving_tinygrad.pkl'}
  catalog = {'protocol': 1, 'format': 'comma-generic-onnx', 'serialization': 'persistent-buffer-v1',
             'gpu_arch': 'gfx1200', 'frame_skip': 4, 'camera_resolutions': [[1928, 1208], [1344, 760]],
             'model_sha256': MODEL_SHA, 'model_checkpoint': CHECKPOINT, 'source_commit': SOURCE, 'tinygrad_commit': TINYGRAD,
             'pickle': {'url': manifest['url'], 'size': MODEL_SIZE, 'sha256': MODEL_SHA},
             'runtime': {'url': runtime.name, 'size': runtime.stat().st_size, 'sha256': digest(runtime)}}
  for name, value in [('manifest.json', manifest), ('precompiled.json', catalog)]:
    (output / name).write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
  shutil.copyfile(model, output / manifest['filename'])
  # Archive for future Jetson integration; this does not change Jetson selection.
  onnx_manifest = {'model_id': 'comma-nav-rl-80eedcdd-onnx', 'filename': 'big_driving_supercombo.onnx',
                   'size': ONNX_SIZE, 'sha256': ONNX_SHA, 'url': 'big_driving_supercombo.onnx',
                   'model_checkpoint': CHECKPOINT, 'source_commit': SOURCE}
  shutil.copyfile(onnx, output / onnx_manifest['filename'])
  (output / 'onnx-manifest.json').write_text(json.dumps(onnx_manifest, indent=2) + '\n', encoding='utf-8')
  print(json.dumps(catalog, indent=2))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('model', type=Path)
  parser.add_argument('source_tar', type=Path)
  parser.add_argument('output', type=Path)
  parser.add_argument('--onnx', type=Path, required=True)
  args = parser.parse_args()
  build(args.model, args.source_tar, args.output, args.onnx)
