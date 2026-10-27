"""Pin a committed plaintext host bundle/model for locally enabled SHA-256 USB delivery."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

from usb_release import RUNTIME, validate


def identity(path, url):
  sha = hashlib.sha256()
  with path.open('rb') as source:
    while data := source.read(1 << 20):
      sha.update(data)
  return {'sha256': sha.hexdigest(), 'size': path.stat().st_size, 'url': url}


def prepare(bundle, model, profile, bundle_url, model_url, output):
  """Publish plaintext metadata and expected hashes; no cryptographic keys."""
  with tarfile.open(bundle) as archive:
    commit = archive.extractfile('SOURCE_COMMIT').read().decode().strip()
    spec = json.load(archive.extractfile('openpilot/selfdrive/modeld/jetlink/jetson_model.json'))
    compatibility = json.load(archive.extractfile('tools/jetlink/runtime_compatibility.json'))
  if compatibility['protected_storage_format'] != 1:
    raise ValueError('Bundle lacks protected-storage compatibility')
  value = {'format': 2, 'delivery': 'usb', 'integrity': 'sha256', 'profile': profile, 'source_commit': commit,
           'runtime': RUNTIME, 'storage_format': 1, 'bundle': identity(bundle, bundle_url), 'model': identity(model, model_url)}
  validate(value)
  if (spec['sha256'], spec['nbytes']) != (value['model']['sha256'], value['model']['size']):
    raise ValueError('ONNX does not match bundled model pin')
  output.mkdir(parents=True, exist_ok=True)
  (output / 'usb_host_sha256_release.json').write_text(json.dumps(value, indent=2) + '\n')
  return value


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  for name in ('bundle', 'model', 'output'):
    parser.add_argument('--' + name, type=Path, required=True)
  for name in ('profile', 'bundle-url', 'model-url'):
    parser.add_argument('--' + name, required=True)
  args = parser.parse_args()
  prepare(args.bundle, args.model, args.profile, args.bundle_url, args.model_url, args.output)
