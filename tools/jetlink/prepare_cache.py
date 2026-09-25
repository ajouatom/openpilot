#!/usr/bin/env python3
"""Import a previously validated TensorRT plan into Jetlink's native cache."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def digest(path):
  h = hashlib.sha256()
  with path.open('rb') as f:
    for part in iter(lambda: f.read(4 << 20), b''):
      h.update(part)
  return h.hexdigest()


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--onnx', type=Path, required=True)
  p.add_argument('--plan', type=Path, required=True)
  p.add_argument('--spec', type=Path, required=True)
  p.add_argument('--cache', type=Path, required=True)
  args = p.parse_args()
  from jetlink.server.cache import EngineCache
  from jetlink.server.backends.trt import TrtBackend
  from jetlink.spec import spec_from_onnx
  spec = json.loads(args.spec.read_text())
  assert args.onnx.stat().st_size == spec['nbytes'] and digest(args.onnx) == spec['sha256'], 'ONNX identity mismatch'
  assert spec_from_onnx(str(args.onnx), spec['frame_skip']).to_dict() == spec, 'model contract mismatch'
  meta = json.loads(args.plan.with_suffix('.json').read_text())
  backend = TrtBackend()
  assert meta['source_sha256'] == spec['sha256'], 'engine source mismatch'
  assert meta['trt_version'] == backend.runtime_version and meta['device'] == backend.describe()['device'], 'engine runtime mismatch'
  cache = EngineCache(args.cache, backend)
  entry = cache.entry(spec['sha256'])
  for source, target in ((args.plan, entry.path), (args.onnx, cache.model_path(spec['sha256']))):
    temporary = target.with_suffix(target.suffix + '.tmp')
    shutil.copyfile(source, temporary)
    os.replace(temporary, target)
  meta.update(spec=spec, imported_plan_sha256=digest(args.plan))
  temporary = entry.meta_path.with_suffix('.json.tmp')
  temporary.write_text(json.dumps(meta, indent=2))
  os.replace(temporary, entry.meta_path)
  cache.remember_loaded(spec['sha256'], spec['frame_skip'])
  print(json.dumps({'cache': str(args.cache), 'engine': str(entry.path), 'model': spec['sha256']}))


if __name__ == '__main__':
  main()
