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
  def run(argv, **kwargs):
    if argv[0] == 'systemctl':
      return SimpleNamespace(returncode=3)
    remembered.write_text('candidate model')
    raise subprocess.CalledProcessError(1, argv)
  monkeypatch.setattr(update.subprocess, 'run', run)
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
