"""Create an immutable signed NAS update manifest from a committed host bundle."""
import argparse
import base64
import json
from pathlib import Path
import tarfile

from Crypto.PublicKey import ECC
from Crypto.Signature import eddsa
from update_host import digest, validate_manifest


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bundle', type=Path, required=True)
  parser.add_argument('--key', type=Path, required=True)
  parser.add_argument('--model-url', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  with tarfile.open(args.bundle) as archive:
    commit = archive.extractfile('SOURCE_COMMIT').read().decode().strip()
    spec = json.load(archive.extractfile('openpilot/selfdrive/modeld/jetlink/cinque_v2.json'))
  manifest = {'format': 1, 'source_commit': commit,
              'runtime': {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'},
              'bundle': {'url': f'https://upload.shind0.synology.me/models/jetlink-host-{commit}/precompiled-runtime.tar.gz',
                         'sha256': digest(args.bundle), 'size': args.bundle.stat().st_size},
              'model': {'url': args.model_url, 'sha256': spec['sha256'], 'size': spec['nbytes']}}
  validate_manifest(manifest)
  key = ECC.import_key(args.key.read_text())
  expected = ECC.import_key(Path(__file__).with_name('release-signing-public.pem').read_text())
  if key.public_key() != expected:
    raise ValueError('Signing key does not match the installed updater trust root')
  payload = json.dumps(manifest, sort_keys=True, separators=(',', ':')).encode()
  manifest['signature'] = base64.b64encode(eddsa.new(key, 'rfc8032').sign(payload)).decode()
  args.output.write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
  main()
