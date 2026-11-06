import hashlib
import hmac
import pytest

from openpilot.selfdrive.carrot.server.services.assistant_connection import AssistantConnection, local_origin, proof_id
from openpilot.selfdrive.carrot.server.services.assistant_settings import AssistantSettings

OLD = 'http://10.225.84.19:7000'
NEW = 'http://192.168.0.178:7000'


@pytest.fixture
def pair(tmp_path):
  stores = {role: (AssistantSettings(tmp_path / (role + '.json'), dict, lambda _: 0, lambda *_: None, lambda: True, lambda *_: None), None)
            for role in ('settings', 'drive', 'tools')}
  tokens = {role: row[0].enable(OLD) for role, row in stores.items()}
  return AssistantConnection(stores), tokens


def test_proof_requires_matching_device_and_challenge(pair):
  service, tokens = pair
  ids = {role: proof_id(AssistantSettings._digest(token)) for role, token in tokens.items()}
  reply = service.proof(NEW, OLD, 'n' * 43, ids)['proofs']
  assert set(reply) == set(tokens)
  for role, token in tokens.items():
    assert reply[role] == hmac.new(AssistantSettings._digest(token).encode(), f'{"n" * 43}\n{OLD}\n{NEW}'.encode(), hashlib.sha256).hexdigest()
  assert service.proof(NEW, OLD, 'x' * 43, ids) != {'proofs': reply}
  assert service.proof(NEW, 'http://192.168.1.2:7000', 'n' * 43, ids) == {'proofs': {}}
  assert service.proof(NEW, OLD, 'n' * 43, {'settings': 'f' * 64}) == {'proofs': {}}
  assert proof_id(AssistantSettings._digest('a' * 43)) == 'eb64db39233b0ababf39976672ee3877bde4d18a79cbb8ae1dd2c0da96659bfb'


def test_move_preserves_grants_but_rejects_old_origin_and_clears_pending(pair):
  service, tokens = pair
  called = []
  store = service.stores['drive'][0]
  owner = store._digest(tokens['drive'])
  store.proposals['pending'] = {'owner': owner}
  service.stores['drive'] = (store, called.append)
  assert set(service.move(NEW, OLD, tokens)['moved']) == set(tokens)
  assert not store.proposals
  assert called == [owner]
  for role, token in tokens.items():
    current = service.stores[role][0]
    current.authorize(token, NEW)
    with pytest.raises(PermissionError):
      current.authorize(token, OLD)
  service.move(NEW, OLD, tokens)  # failed response is safe to retry
  assert called == [owner, owner]
  service.move(OLD, NEW, tokens)  # hotspot -> office -> hotspot
  store.authorize(tokens['drive'], OLD)


def test_no_partial_grant_expansion_or_revoked_credential_restore(pair):
  service, tokens = pair
  with pytest.raises(PermissionError):
    service.move(NEW, OLD, {**tokens, 'drive': 'z' * 43})
  for role, token in tokens.items():
    service.stores[role][0].authorize(token, OLD)
  service.stores['settings'][0].revoke(tokens['settings'], OLD)
  with pytest.raises(PermissionError):
    service.move(NEW, OLD, tokens)
  service.move(NEW, OLD, {'tools': tokens['tools']})
  service.stores['drive'][0].authorize(tokens['drive'], OLD)


@pytest.mark.parametrize('origin', ['http://evil.test:7000', 'http://8.8.8.8:7000', OLD + '/', OLD + '?x', 'http://user@10.225.84.19:7000', 'https://10.225.84.19:7000'])
def test_reject_nonlocal_origins(origin):
  assert not local_origin(origin)
