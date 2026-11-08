"""Restore existing local permissions after an address change; never a condition for account connection."""
import hashlib
import hmac
import ipaddress
import re
from urllib.parse import urlsplit

TOKEN = re.compile(r'[A-Za-z0-9_-]{43}')
HEX = re.compile(r'[a-f0-9]{64}')


def local_origin(value):
  try:
    url = urlsplit(value)
    address = ipaddress.IPv4Address(url.hostname)
    return (url.scheme == 'http' and url.port == 7000 and url.netloc == f'{address}:7000'
            and not url.path and not url.query and not url.fragment and address.is_private and not address.is_loopback)
  except (ValueError, TypeError, AttributeError):
    return False


def proof_id(digest):
  return hashlib.sha256(digest.encode()).hexdigest()


class AssistantConnection:
  def __init__(self, stores):
    # role -> (permission store, optional callback to cancel pending work)
    self.stores = stores

  def proof(self, origin, previous, nonce, identifiers):
    if not local_origin(origin) or not local_origin(previous) or not isinstance(nonce, str) or not TOKEN.fullmatch(nonce):
      raise ValueError('Invalid device proof request')
    if not isinstance(identifiers, dict) or len(identifiers) > 4:
      raise ValueError('Invalid proof identifiers')
    replies = {}
    for role, identity in identifiers.items():
      if role not in self.stores or not isinstance(identity, str) or not HEX.fullmatch(identity):
        continue
      store, _ = self.stores[role]
      for digest, bound in store.grants.items():
        if bound in (previous, origin) and hmac.compare_digest(proof_id(digest), identity):
          # Only the original phone and this device know SHA256(the bearer token).
          # Sending its hash again as an identifier does not disclose that secret.
          message = f'{nonce}\n{previous}\n{origin}'
          replies[role] = hmac.new(digest.encode(), message.encode(), hashlib.sha256).hexdigest()
    return {'proofs': replies}

  def move(self, origin, previous, credentials):
    if not local_origin(origin) or not local_origin(previous) or not isinstance(credentials, dict) or not 1 <= len(credentials) <= 4:
      raise ValueError('Invalid connection move')
    # Validate every supplied credential before modifying any store. No new scopes.
    matched = []
    for role, token in credentials.items():
      if role not in self.stores or not isinstance(token, str) or not TOKEN.fullmatch(token):
        raise PermissionError('Invalid existing permission')
      store, cancel = self.stores[role]
      digest = store._digest(token)
      bound = store.grants.get(digest)
      if bound not in (previous, origin):
        raise PermissionError('Permission belongs to another device or was revoked')
      matched.append((role, store, cancel, digest, bound))
    for _role, store, cancel, digest, bound in matched:
      # Cleanup also runs on retries after a partially completed move.
      store.proposals = {k: v for k, v in store.proposals.items() if v['owner'] != digest}
      store.results = {k: v for k, v in store.results.items() if k[0] != digest}
      if cancel:
        cancel(digest)
      if bound != origin:
        store._save({**store.grants, digest: origin})
    return {'moved': [row[0] for row in matched]}
