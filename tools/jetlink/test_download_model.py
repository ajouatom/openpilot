import hashlib
import io
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import download_model as d


@pytest.mark.parametrize('mode', ['valid', 'bad_manifest', 'bad_model', 'existing'])
def test_provision_verifies_identity_and_preserves_existing_files(tmp_path, monkeypatch, mode):
  content = b'verified test model'
  digest = hashlib.sha256(content).hexdigest()
  spec = tmp_path / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json'
  spec.parent.mkdir(parents=True)
  spec.write_text(json.dumps({'sha256': digest, 'nbytes': len(content)}))
  monkeypatch.setattr(d, 'ROOT', tmp_path)
  manifest = {'sha256': digest, 'size': len(content), 'url': 'big_driving_supercombo.onnx'}
  if mode == 'bad_manifest':
    manifest['sha256'] = 'wrong'
  def get(url, **kwargs):
    return io.BytesIO(json.dumps(manifest).encode() if url == d.MANIFEST_URL else
                      (b'wrong model' if mode == 'bad_model' else content))
  monkeypatch.setattr(d.urllib.request, 'urlopen', get)
  output = tmp_path / 'model.onnx'
  if mode == 'existing':
    output.write_bytes(b'existing user model')
  if mode == 'valid':
    d.provision(output)
    assert output.read_bytes() == content
  else:
    with pytest.raises(ValueError):
      d.provision(output)
    if mode == 'existing':
      assert output.read_bytes() == b'existing user model'
    else:
      assert not output.exists()
  assert not list(tmp_path.glob('model.onnx.part-*'))
