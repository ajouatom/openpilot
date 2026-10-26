"""Small catalog cache requests, used only when comma is actually offroad/idle."""
import json
import time
from openpilot.common import big_model_catalog as C

CAPABILITY = 'carrot_model_library_v1'
REQUEST, RESPONSE = 0x4040, 0x4041


def host_request(host, request, root):
  if not isinstance(request, dict) or set(request) != {'delete'} or not isinstance(request['delete'], list):
    raise ValueError('invalid model library request')
  specs = {C.model_spec(key)['sha256'] for key in C.catalog()}
  requested = request['delete']
  if len(requested) > len(specs) or any(sha not in specs for sha in requested):
    raise ValueError('unknown model identity')
  from openpilot.common.jetson_model import read_spec
  pinned = read_spec(root)['sha256']
  with host.lock:
    if host.job is not None and host.job.state == 'building':
      raise ValueError('model preparation in progress')
    protected = {pinned, host.loaded.sha256 if host.loaded else None}
    deleted = []
    for sha in requested:
      if sha in protected:
        continue
      entry = host.cache.entry(sha)
      targets = [host.cache.model_path(sha), entry.path, entry.meta_path]
      for path in targets:
        if path.is_symlink() or not path.resolve().is_relative_to(host.cache.root.resolve()):
          raise ValueError('invalid cache path')
      for path in targets:
        path.unlink(missing_ok=True)
      deleted.append(sha)
    inventory = {}
    for sha in specs:
      entry = host.cache.entry(sha)
      inventory[sha] = sum(p.stat().st_size for p in (host.cache.model_path(sha), entry.path, entry.meta_path) if p.is_file())
  return {'models': inventory, 'deleted': deleted}


class Client:
  def __init__(self, peer):
    self.enabled = peer.get(CAPABILITY) is True
    self.last = 0.

  def idle(self, client, params):
    if not self.enabled or time.monotonic() - self.last < 10:
      return
    self.last = time.monotonic()
    if not params.get_bool('IsOffroad') or params.get_bool('IsOnroad'):
      return
    with C.library_lock():
      self._exchange(client)

  def _exchange(self, client):
    from openpilot.selfdrive.modeld.model_library import protected_ids
    queue_path = C.storage() / 'jetson-delete.json'
    queue = C.read_json(queue_path).get('models', [])
    protected = {C.model_spec(key)['sha256'] for key in protected_ids()}
    retained = {C.model_spec(key)['sha256'] for key in C.available()}
    retired = {key for key, entry in C.catalog().items() if entry.get('retired') is True}
    requested = {C.model_spec(key)['sha256'] for key in queue if key in retired} - protected - retained
    seq = client._next_seq()
    client.t.send_json(REQUEST, seq, {'delete': sorted(requested)})
    result = json.loads(bytes(client._expect(RESPONSE, seq, 3.).payload))
    if not isinstance(result, dict) or not isinstance(result.get('models'), dict):
      return
    C.atomic_json(C.storage() / 'jetson-library.json', dict(result, updated=time.monotonic(), boot=C.boot_id()))
    # Re-read so requests submitted while the transaction ran are not lost.
    current = C.read_json(queue_path).get('models', [])
    done = set(result.get('deleted', []))
    remaining = [key for key in current if key in retired and C.model_spec(key)['sha256'] not in done | retained]
    if remaining != current:
      C.atomic_json(queue_path, {'models': remaining})
