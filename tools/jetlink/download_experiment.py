"""Fetch this branch's exact upstream ONNX into a PC preparation directory."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import urllib.parse
import urllib.request

from model_pin import read_spec
from install_local_experiment import digest as file_digest

LFS = 'https://huggingface.co/commaai/openpilot-lfs.git/info/lfs/objects/batch'


def download(output):
  spec = read_spec()
  expected, size = spec['sha256'], spec['nbytes']
  output = Path(output)
  if output.is_file():
    digest = file_digest(output)
    if output.stat().st_size == size and digest == expected:
      print('VERIFIED', expected, flush=True)
      return
    raise ValueError('Existing destination is a different model')
  body = json.dumps({'operation': 'download', 'transfers': ['basic'],
                     'objects': [{'oid': expected, 'size': size}]}).encode()
  request = urllib.request.Request(LFS, data=body, headers={
    'Content-Type': 'application/vnd.git-lfs+json', 'Accept': 'application/vnd.git-lfs+json'})
  with urllib.request.urlopen(request, timeout=45) as response:
    item, = json.loads(response.read(65536))['objects']
  if item.get('oid') != expected or item.get('size') != size or 'error' in item:
    raise ValueError('Upstream LFS object identity mismatch')
  action = item['actions']['download']
  if urllib.parse.urlsplit(action['href']).scheme != 'https':
    raise ValueError('LFS download requires HTTPS')
  output.parent.mkdir(parents=True, exist_ok=True)
  temporary = output.with_suffix('.part')
  digest, count, reported = hashlib.sha256(), 0, 0
  request = urllib.request.Request(action['href'], headers=action.get('header', {}))
  try:
    with urllib.request.urlopen(request, timeout=60) as response, temporary.open('wb') as target:
      if response.status != 200 or urllib.parse.urlsplit(response.url).scheme != 'https':
        raise ValueError('Unexpected model response')
      while block := response.read(4 << 20):
        count += len(block)
        if count > size:
          raise ValueError('Oversized model download')
        target.write(block)
        digest.update(block)
        if count - reported >= 64 << 20:
          print('MODEL_DOWNLOAD', count, '/', size, flush=True)
          reported = count
      target.flush()
      os.fsync(target.fileno())
    if count != size or digest.hexdigest() != expected:
      raise ValueError('Model size or SHA-256 mismatch')
    os.replace(temporary, output)
    print('VERIFIED', expected, count, flush=True)
  finally:
    temporary.unlink(missing_ok=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  download(parser.parse_args().output)
