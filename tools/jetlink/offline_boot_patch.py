"""Select an exactly verified R2 variant, then use the existing guarded patcher."""
import argparse
import base64
import json
import os
from pathlib import Path
import re
import sys

# Windows embedded Python isolates sys.path from the script's directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_hotfix import apply, load_manifest, normalized_hash, read_exact, validate


def validate_index(index):
  if index.get('format') != 'carrot-offline-boot-v1' or len(index.get('profiles', [])) != 2:
    raise ValueError('Unknown boot patch format')
  if (not re.fullmatch('[0-9a-f]{64}', index.get('payload_sha256', '')) or
      not isinstance(index.get('payload_bytes'), int) or not 0 < index['payload_bytes'] < 1 << 20):
    raise ValueError('Invalid boot payload identity')
  first = index['profiles'][0]
  first_patches = validate(first)
  identities = set()
  for profile in index['profiles']:
    patches = validate(profile)
    if (any(profile[k] != first[k] for k in ('root_offset', 'root_bytes', 'image_bytes', 'partition_guard')) or
        [(o, old) for o, old, _ in patches] != [(o, old) for o, old, _ in first_patches]):
      raise ValueError('Variant normalization differs')
    if not re.fullmatch('[0-9a-f]{64}', profile.get('root_sha256', '')):
      raise ValueError('Invalid APP identity')
    identities.add(profile['root_sha256'])
  if len(identities) != len(index['profiles']):
    raise ValueError('Ambiguous boot patch profiles')
  return first, first_patches


def select(stream, index, report=print):
  first, patches = validate_index(index)
  guard = first['partition_guard']
  stream.seek(guard['offset'])
  if read_exact(stream, 512) != base64.b64decode(guard['data'], validate=True):
    raise RuntimeError('Unsupported partition layout; nothing written')
  actual = normalized_hash(stream, first['root_offset'], first['root_bytes'], patches, report)
  matches = [p for p in index['profiles'] if p['root_sha256'] == actual]
  if len(matches) != 1:
    raise RuntimeError('Unsupported or modified installation; nothing written. Do not format or bypass this check.')
  report('INSTALLATION ' + matches[0]['release'])
  return matches[0]


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--manifest', required=True)
  parser.add_argument('--manifest-sha256', required=True)
  parser.add_argument('--target')
  parser.add_argument('--verify-only', action='store_true')
  parser.add_argument('--inspect', action='store_true')
  args = parser.parse_args()
  index = load_manifest(args.manifest, args.manifest_sha256)
  first, _ = validate_index(index)
  if args.inspect:
    print(json.dumps({key: first[key] for key in ('root_offset', 'root_bytes', 'image_bytes')}))
    return
  if not args.target:
    parser.error('--target is required')
  with open(args.target, 'rb' if args.verify_only else 'r+b', buffering=0) as stream:
    profile = select(stream, index)
    def sync():
      stream.flush()
      os.fsync(stream.fileno())
    apply(stream, profile, verify_only=args.verify_only, sync=sync)


if __name__ == '__main__':
  main()
