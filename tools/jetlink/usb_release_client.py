"""Comma-side hash-pinned asset download and stop-and-wait boot-gate USB upload."""
import hashlib
import json
import os
from pathlib import Path
import urllib.request

from tools.jetlink.usb_release import ACK, BEGIN, BLOCK, CHECK, CHUNK, FINISH, HEADER, KINDS, NAMES, validate  # noqa: TID251


def asset(item, cache, allowed):
  cache.mkdir(parents=True, exist_ok=True)
  path = cache / item['sha256']
  def matches():
    if not path.is_file() or path.stat().st_size != item['size']:
      return False
    digest = hashlib.sha256()
    with path.open('rb') as stream:
      while data := stream.read(1 << 20):
        if not allowed():
          raise ConnectionError('USB update requires offroad and USB connection')
        digest.update(data)
    return digest.hexdigest() == item['sha256']
  if matches():
    return path
  partial = path.with_suffix('.part')
  try:
    if not allowed():
      raise ConnectionError('USB update requires offroad and USB connection')
    with urllib.request.urlopen(item['url'], timeout=30) as source, partial.open('wb') as output:
      if not source.url.startswith('https://'):
        raise ValueError('Insecure asset redirect')
      count, digest = 0, hashlib.sha256()
      while data := source.read(BLOCK):
        if not allowed():
          raise ConnectionError('USB update interrupted by loss of offroad permission or USB')
        count += len(data)
        if count > item['size']:
          raise ValueError('Oversized USB update asset')
        digest.update(data)
        output.write(data)
      output.flush()
      os.fsync(output.fileno())
    if count != item['size'] or digest.hexdigest() != item['sha256']:
      raise ValueError('USB update download hash mismatch')
    os.replace(partial, path)
    return path
  finally:
    partial.unlink(missing_ok=True)


def exchange(client, kind, payload, allowed, raw=False, timeout=30):
  if not allowed():
    raise ConnectionError('USB update permission or connection unavailable')
  seq = client._next_seq()
  client.t.send(kind, seq, [payload if raw else json.dumps(payload).encode()], timeout=timeout)
  response = client.t.recv(timeout=timeout)
  if response.msg_type != ACK or response.seq != seq or len(response.payload) > 65536:
    raise ValueError('Unexpected USB update acknowledgement')
  value = json.loads(bytes(response.payload))
  if not isinstance(value, dict):
    raise ValueError('Invalid USB update acknowledgement')
  if value.get('ok') is not True:
    stage = {BEGIN: 'begin', CHECK: 'check', CHUNK: 'transfer', FINISH: 'verify'}[kind]
    raise ValueError(f'Jetson rejected USB update [{stage}]: ' + str(value.get('error', 'invalid reply')))
  return value


def available(client, manifest, connected):
  """Read-only check; a matching current commit alone is not registration."""
  validate(manifest)
  return exchange(client, CHECK, manifest, connected, timeout=120).get('complete') is True


def upload(client, manifest, cache, allowed, progress):
  validate(manifest)
  def request(kind, payload, **kwargs):
    if not allowed():
      raise ConnectionError('USB update requires offroad and USB connection')
    return exchange(client, kind, payload, allowed, **kwargs)
  # An already registered package needs no network. BEGIN cannot activate it.
  progress('checking_usb', None)
  result = request(BEGIN, manifest, timeout=120)
  if result.get('complete') is True:
    return
  total, sent = sum(manifest[k]['size'] for k in KINDS), 0
  for index, kind in enumerate(KINDS):
    progress('downloading', sent / total)
    path = asset(manifest[kind], Path(cache), allowed)
    offset = 0
    with path.open('rb') as source:
      while data := source.read(BLOCK):
        result = request(CHUNK, HEADER.pack(index, offset) + data, raw=True)
        offset += len(data)
        sent += len(data)
        if result.get('index') != index or result.get('offset') != offset:
          raise ValueError('USB acknowledgement offset mismatch')
        progress('transferring', sent / total)
    if offset != manifest[kind]['size']:
      raise ValueError('Cached USB asset changed during transfer: ' + NAMES[index])
  progress('verifying', 1.)
  if request(FINISH, {}, timeout=120).get('complete') is not True:
    raise ValueError('USB update registration incomplete')
