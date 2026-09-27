import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import update_host as update


def manifest():
  return {'format': 1, 'source_commit': 'a' * 40,
          'runtime': {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'},
          'model': {'url': 'https://upload.shind0.synology.me/models/test/big_driving_supercombo.onnx', 'sha256': 'b' * 64, 'size': 3},
          'bundle': {'url': 'https://upload.shind0.synology.me/models/test/precompiled-runtime.tar.gz', 'sha256': 'c' * 64, 'size': 3}}


def test_reject_os_upgrade_or_untrusted_host():
  value = manifest()
  update.validate_manifest(value)
  value['runtime']['l4t'] = '999'
  with pytest.raises(ValueError):
    update.validate_manifest(value)
  for url in ('http://upload.shind0.synology.me/models/x', 'https://evil.invalid/models/x',
              'https://upload.shind0.synology.me.evil.invalid/models/x', 'file:///etc/passwd'):
    with pytest.raises(ValueError):
      update.checked_url(url)


def test_download_corruption_does_not_replace_current_file(tmp_path, monkeypatch):
  class Response(io.BytesIO):
    url = 'https://upload.shind0.synology.me/models/x'
  monkeypatch.setattr(update.urllib.request, 'urlopen', lambda *a, **k: Response(b'bad'))
  target = tmp_path / 'model.onnx'
  target.write_bytes(b'old')
  with pytest.raises(ValueError, match='checksum'):
    update.fetch(Response.url, target, '0' * 64, 3)
  assert target.read_bytes() == b'old'
  assert not list(tmp_path.glob('*.part'))


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux service/symlink semantics')
def test_failed_candidate_preserves_running_release_and_model(tmp_path, monkeypatch):
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  monkeypatch.setattr(update, 'verify_signature', lambda _: None)
  old = tmp_path / 'releases' / ('d' * 40)
  old.mkdir(parents=True)
  candidate = old.with_name('a' * 40)
  candidate.mkdir()
  (tmp_path / 'current').symlink_to(old)
  (tmp_path / 'cache').mkdir()
  remembered = tmp_path / 'cache/last-loaded.json'
  remembered.write_text('old model')
  update.atomic_json(tmp_path / 'updates/pending.json', manifest())
  def probe(release):
    remembered.write_text('candidate model')
    raise subprocess.CalledProcessError(1, ['synthetic-probe'])
  monkeypatch.setattr(update, 'probe_release', probe)
  monkeypatch.setattr(update.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=3))
  update.activate()
  assert (tmp_path / 'current').resolve() == old
  assert remembered.read_text() == 'old model'
  assert not (tmp_path / 'updates/pending.json').exists()
  assert json.loads((tmp_path / 'updates/status.json').read_text())['state'] == 'rejected'


def test_cannot_activate_while_inference_is_running(tmp_path, monkeypatch):
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  update.atomic_json(tmp_path / 'updates/pending.json', manifest())
  monkeypatch.setattr(update.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0))
  with pytest.raises(RuntimeError, match='stopped'):
    update.activate()


def test_no_telemetry_does_not_authorize_automatic_download(monkeypatch):
  import hud_protocol
  monkeypatch.setattr(hud_protocol, 'read_snapshot', lambda: None)
  monkeypatch.setattr(update, 'stage', lambda: pytest.fail('must not download'))
  update.automatic_stage()


def test_unsigned_release_is_rejected():
  with pytest.raises(ValueError):
    update.verify_signature(manifest())


def test_signature_detects_manifest_tampering(monkeypatch):
  import base64
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  key = ECC.generate(curve='Ed25519')
  original = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda p, *a, **k:
                      key.public_key().export_key(format='PEM') if p.name == 'release-signing-public.pem' else original(p, *a, **k))
  value = manifest()
  payload = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
  value['signature'] = base64.b64encode(eddsa.new(key, 'rfc8032').sign(payload)).decode()
  update.verify_signature(value)
  value['model']['size'] += 1
  with pytest.raises(ValueError):
    update.verify_signature(value)


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux service/symlink semantics')
def test_interrupted_activation_recovers_before_any_new_probe(tmp_path, monkeypatch):
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  old, candidate = tmp_path / 'releases/old', tmp_path / 'releases/new'
  old.mkdir(parents=True)
  candidate.mkdir()
  (tmp_path / 'current').symlink_to(candidate)
  (tmp_path / 'cache').mkdir()
  (tmp_path / 'cache/last-loaded.json').write_text('new model')
  update.atomic_json(tmp_path / 'updates/transaction.json', {'release': str(old), 'last_loaded': 'old model'})
  update.atomic_json(tmp_path / 'updates/pending.json', manifest())
  monkeypatch.setattr(update.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=3))
  update.activate()
  assert (tmp_path / 'current').resolve() == old
  assert (tmp_path / 'cache/last-loaded.json').read_text() == 'old model'
  assert not (tmp_path / 'updates/transaction.json').exists()
  assert not (tmp_path / 'updates/pending.json').exists()
  assert json.loads((tmp_path / 'updates/status.json').read_text())['state'] == 'recovered'


def test_probe_timeout_kills_and_reaps_entire_child_group(monkeypatch):
  calls = []
  class Process:
    pid = 12345
    def wait(self, timeout=None):
      calls.append(('wait', timeout))
      if timeout is not None:
        raise subprocess.TimeoutExpired('synthetic', timeout)
      return -9
  def popen(command, **kwargs):
    assert kwargs['start_new_session'] is True
    return Process()
  monkeypatch.setattr(update.subprocess, 'Popen', popen)
  monkeypatch.setattr(update.signal, 'SIGKILL', 9, raising=False)
  monkeypatch.setattr(update.os, 'killpg', lambda pid, sig: calls.append(('kill', pid)), raising=False)
  with pytest.raises(subprocess.TimeoutExpired):
    update.probe_release(Path('/synthetic/release'))
  assert calls == [('wait', 960), ('kill', 12345), ('wait', None)]
