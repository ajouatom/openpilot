"""Apply a release-specific, data-only patch to a freshly flashed, unmounted SD.

No filesystem writer is implemented here. A publisher resolves the existing
file's allocated sectors; the complete normalized root partition must match
the pristine release before any write. This also permits interrupted retries.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path


def read_exact(stream, size):
  result = bytearray()
  while len(result) < size:
    part = stream.read(size - len(result))
    if not part:
      raise RuntimeError('Short read; no verification is possible')
    result.extend(part)
  return bytes(result)


def validate(manifest):
  if manifest['format'] != 1:
    raise ValueError('Unknown patch format')
  start, length = manifest['root_offset'], manifest['root_bytes']
  if start < 512 or start % 512 or length <= 0 or length % 512:
    raise ValueError('Invalid root partition bounds')
  previous = start
  patches = []
  for item in manifest['patches']:
    offset = item['offset']
    old, new = (base64.b64decode(item[key], validate=True) for key in ('before', 'after'))
    if (offset < previous or offset % 512 or not old or len(old) % 512 or len(old) != len(new)
        or offset + len(old) > start + length):
      raise ValueError('Invalid or overlapping patch extent')
    if hashlib.sha256(old).hexdigest() != item['before_sha256'] or hashlib.sha256(new).hexdigest() != item['after_sha256']:
      raise ValueError('Patch data checksum mismatch')
    previous = offset + len(old)
    patches.append((offset, old, new))
  if not patches or sum(len(p[1]) for p in patches) > 65536:
    raise ValueError('Unexpected patch size')
  return patches


def normalized_hash(stream, start, length, patches, report=print):
  stream.seek(start)
  digest = hashlib.sha256()
  position, next_report = start, start + (1 << 30)
  while position < start + length:
    block = bytearray(read_exact(stream, min(4 << 20, start + length - position)))
    for offset, old, _ in patches:
      low, high = max(offset, position), min(offset + len(old), position + len(block))
      if low < high:
        block[low-position:high-position] = old[low-offset:high-offset]
    digest.update(block)
    position += len(block)
    if position >= next_report:
      report(f'CHECK {position-start} / {length}')
      next_report += 1 << 30
  return digest.hexdigest()


def apply(stream, manifest, *, verify_only=False, sync=None, report=print):
  patches = validate(manifest)
  start, length = manifest['root_offset'], manifest['root_bytes']
  # Check the GPT entry, including type/unique ID, bounds and name. Windows may
  # relocate the backup GPT or modify FAT; neither changes the root entry.
  guard = manifest['partition_guard']
  stream.seek(guard['offset'])
  expected = base64.b64decode(guard['data'], validate=True)
  if len(expected) != 512 or read_exact(stream, 512) != expected:
    raise RuntimeError('Partition identity differs from the supported release')
  if normalized_hash(stream, start, length, patches, report) != manifest['root_sha256']:
    raise RuntimeError('Not the pristine supported root filesystem; nothing was written. Booted cards require the online installer.')
  changes = []
  for offset, old, new in patches:
    stream.seek(offset)
    actual = read_exact(stream, len(new))
    if actual != new:
      changes.append((offset, actual, new))
  if verify_only:
    report('VERIFIED_ALREADY_APPLIED' if not changes else 'VERIFIED_READY_TO_APPLY')
    return not changes
  if not changes:
    report('ALREADY_APPLIED')
    return True
  flush = sync or stream.flush
  try:
    for offset, _, new in changes:
      stream.seek(offset)
      if stream.write(new) != len(new):
        raise RuntimeError('Short write')
    flush()
    for offset, _, new in changes:
      stream.seek(offset)
      if read_exact(stream, len(new)) != new:
        raise RuntimeError('Hotfix readback mismatch')
  except Exception:
    # Best effort; a power loss/disconnection can prevent rollback. Re-running
    # this exact package repairs its sectors after the full normalized check.
    try:
      for offset, before, _ in changes:
        stream.seek(offset)
        stream.write(before)
      flush()
    except Exception as error:
      report(f'ROLLBACK_INCOMPLETE: {error}; reconnect and run this package again')
    raise
  report(f'APPLIED_AND_READ_BACK {sum(len(p[2]) for p in changes)} bytes')
  return True


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--target', required=True, help='unmounted disk, or disposable test image')
  parser.add_argument('--manifest', required=True)
  parser.add_argument('--manifest-sha256', required=True)
  parser.add_argument('--verify-only', action='store_true')
  args = parser.parse_args()
  data = Path(args.manifest).read_bytes()
  if hashlib.sha256(data).hexdigest() != args.manifest_sha256:
    raise RuntimeError('Release manifest checksum mismatch')
  manifest = json.loads(data)
  with open(args.target, 'rb' if args.verify_only else 'r+b', buffering=0) as stream:
    def sync():
      stream.flush()
      os.fsync(stream.fileno())
    apply(stream, manifest, verify_only=args.verify_only, sync=sync)


if __name__ == '__main__':
  main()
