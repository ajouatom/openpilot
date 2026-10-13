"""Provision the pinned Jetlink ONNX from its dedicated NAS directory, before boot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from model_pin import read_spec
import tempfile
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_URL = 'https://upload.shind0.synology.me/models/carrot-jetlink-cinque-v2/manifest.json'


def digest(path):
  value = hashlib.sha256()
  with path.open('rb') as source:
    for block in iter(lambda: source.read(4 << 20), b''):
      value.update(block)
  return value.hexdigest()


def provision(output, manifest_url=None):
  spec = read_spec(ROOT)
  if manifest_url is None:
    if (ROOT / 'openpilot/selfdrive/modeld/jetlink/jetson_model.json').is_file():
      raise ValueError('Experimental Jetson deployment requires --manifest-url for the pinned model release')
    manifest_url = MANIFEST_URL
  if urllib.parse.urlsplit(manifest_url).scheme != 'https':
    raise ValueError('Model manifest requires HTTPS')
  with urllib.request.urlopen(manifest_url, timeout=30) as response:
    manifest = json.loads(response.read(65537))
  if manifest.get('sha256') != spec['sha256'] or manifest.get('size') != spec['nbytes']:
    raise ValueError('NAS manifest does not match this branch model')
  url = urllib.parse.urljoin(manifest_url, manifest['url'])
  expected_url = urllib.parse.urljoin(manifest_url, 'big_driving_supercombo.onnx')
  if url != expected_url:
    raise ValueError('Unexpected model download URL')
  if output.exists():
    if output.stat().st_size != spec['nbytes'] or digest(output) != spec['sha256']:
      raise ValueError('Existing destination contains a different model; choose another path')
    print('Verified pinned model:', output)
    return
  output.parent.mkdir(parents=True, exist_ok=True)
  fd, temporary_name = tempfile.mkstemp(dir=output.parent, prefix=output.name + '.part-')
  temporary = Path(temporary_name)
  value = hashlib.sha256()
  size = 0
  try:
    with os.fdopen(fd, 'wb') as target, urllib.request.urlopen(url, timeout=60) as response:
      for block in iter(lambda: response.read(4 << 20), b''):
        size += len(block)
        if size > spec['nbytes']:
          raise ValueError('Oversized model download')
        target.write(block)
        value.update(block)
    if size != spec['nbytes'] or value.hexdigest() != spec['sha256']:
      raise ValueError('Downloaded model identity mismatch')
    if output.exists():
      raise FileExistsError('Destination appeared during download')
    os.replace(temporary, output)
  finally:
    temporary.unlink(missing_ok=True)
  print('Downloaded and verified pinned model:', output)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--manifest-url', help='HTTPS manifest for this branch model (required for experimental models)')
  args = parser.parse_args()
  provision(args.output, args.manifest_url)
