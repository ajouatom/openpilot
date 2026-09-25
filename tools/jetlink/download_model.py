"""Provision the pinned Jetlink ONNX from its dedicated NAS directory, before boot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
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


def provision(output):
  spec = json.loads((ROOT / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
  with urllib.request.urlopen(MANIFEST_URL, timeout=30) as response:
    manifest = json.loads(response.read(65537))
  if manifest.get('sha256') != spec['sha256'] or manifest.get('size') != spec['nbytes']:
    raise ValueError('NAS manifest does not match this branch model')
  url = urllib.parse.urljoin(MANIFEST_URL, manifest['url'])
  expected_url = urllib.parse.urljoin(MANIFEST_URL, 'big_driving_supercombo.onnx')
  if url != expected_url:
    raise ValueError('Unexpected model download URL')
  if output.exists():
    if output.stat().st_size != spec['nbytes'] or digest(output) != spec['sha256']:
      raise ValueError('Existing destination contains a different model; choose another path')
    print('Verified existing Cinque v2:', output)
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
  print('Downloaded and verified Cinque v2:', output)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  provision(parser.parse_args().output)
