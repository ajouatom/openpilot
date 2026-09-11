import copy
import hashlib
import io
import json
import tarfile
from types import SimpleNamespace
from urllib.error import HTTPError

import pytest

from openpilot.selfdrive.modeld import precompiled_model as pm


class Response(io.BytesIO):
  def __init__(self, data, status=200, headers=None):
    super().__init__(data)
    self.status, self.headers = status, headers or {}


def catalog():
  runtime = io.BytesIO()
  with tarfile.open(fileobj=runtime, mode='w:gz') as tar:
    for name in ['model_runtime.py', 'tinygrad/__init__.py']:
      entry = tarfile.TarInfo(name)
      entry.size = 1
      tar.addfile(entry, io.BytesIO(b'\n'))
  data = {'pickle': b'compiled-pickle', 'runtime': runtime.getvalue()}
  value = {'protocol': 1, 'format': 'comma-run-model', 'gpu_arch': 'gfx1200', 'frame_skip': 4,
           'camera_resolutions': [[1928, 1208], [1344, 760]], 'onnx_sha256': 'a' * 64, 'model_checkpoint': 'checkpoint'}
  for name, content in data.items():
    value[name] = {'url': name, 'sha256': hashlib.sha256(content).hexdigest(), 'size': len(content)}
  return value, data


def test_install_and_reuse_verified_artifacts_without_network(tmp_path, monkeypatch):
  value, data = catalog()
  model = SimpleNamespace(sha256='a' * 64, url='https://nas.example/models/v2/model.onnx')
  def fetch(request, **kwargs):
    name = request.full_url.rsplit('/', 1)[-1]
    return Response(json.dumps(value).encode() if name == 'precompiled.json' else data[name])
  monkeypatch.setattr(pm, 'urlopen', fetch)
  path = pm.ensure_precompiled(model, tmp_path)
  assert path.read_bytes() == data['pickle']
  assert pm.installed(model, tmp_path) == path
  monkeypatch.setattr(pm, 'urlopen', lambda *a, **kw: pytest.fail('valid cache should work offline'))
  assert pm.ensure_precompiled(model, tmp_path) == path
  pm.reject(path)
  assert pm.installed(model, tmp_path) is None


@pytest.mark.parametrize('field,value', [('onnx_sha256', 'b' * 64), ('protocol', 2), ('gpu_arch', 'gfx1100'),
                                       ('frame_skip', 2), ('camera_resolutions', [[1, 2]])])
def test_incompatible_catalog_is_rejected(field, value):
  metadata, _ = catalog()
  metadata[field] = value
  with pytest.raises(ValueError):
    pm.validate_catalog(metadata, 'a' * 64, 'https://nas.example/precompiled.json')


def test_catalog_cannot_redirect_execution_to_another_origin():
  metadata, _ = catalog()
  metadata['runtime']['url'] = 'https://other.example/runtime.tar.gz'
  with pytest.raises(ValueError):
    pm.validate_catalog(metadata, 'a' * 64, 'https://nas.example/precompiled.json')


def test_missing_catalog_leaves_local_model_untouched(tmp_path, monkeypatch):
  model = SimpleNamespace(sha256='a' * 64, url='https://nas.example/model.onnx')
  local = tmp_path / 'old-model.pkl'
  local.write_bytes(b'working local model')
  def missing(*args, **kwargs):
    raise HTTPError(model.url, 404, 'missing', {}, None)
  monkeypatch.setattr(pm, 'urlopen', missing)
  with pytest.raises(HTTPError):
    pm.ensure_precompiled(model, tmp_path)
  assert local.read_bytes() == b'working local model'
  assert pm.installed(model, tmp_path) is None


def test_download_resumes_and_rejects_corruption(tmp_path, monkeypatch):
  value, data = catalog()
  artifact = copy.deepcopy(value['pickle'])
  artifact['url'] = 'https://nas.example/model.pkl'
  target = tmp_path / 'model.pkl'
  target.with_suffix('.pkl.part').write_bytes(data['pickle'][:4])
  def resume(request, **kwargs):
    assert request.get_header('Range') == 'bytes=4-'
    return Response(data['pickle'][4:], 206, {'Content-Range': f'bytes 4-{len(data["pickle"])-1}/{len(data["pickle"])}'})
  monkeypatch.setattr(pm, 'urlopen', resume)
  pm.download(artifact, target)
  assert target.read_bytes() == data['pickle']
  target.unlink()
  monkeypatch.setattr(pm, 'urlopen', lambda *a, **kw: Response(b'x' * artifact['size']))
  with pytest.raises(ValueError, match='hash mismatch'):
    pm.download(artifact, target)
  assert not target.exists()
  assert not target.with_suffix('.pkl.part').exists()


def test_wrong_resume_range_is_rejected(tmp_path, monkeypatch):
  value, _ = catalog()
  artifact = value['pickle'] | {'url': 'https://nas.example/model.pkl'}
  target = tmp_path / 'model.pkl'
  target.with_suffix('.pkl.part').write_bytes(b'com')
  monkeypatch.setattr(pm, 'urlopen', lambda *a, **kw: Response(b'piled', 206, {'Content-Range': 'bytes 0-4/5'}))
  with pytest.raises(OSError, match='resume'):
    pm.download(artifact, target)
