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


BOOTLOADER_TIMEOUT = 'TimeoutError: BL not ready. Timed out after 10000 ms, condition not met: 0 != 2147483648\n'


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


def test_generic_catalog_is_bound_to_the_pickle_hash():
  value, _ = catalog()
  value.update(format='comma-generic-onnx', model_sha256=value['pickle']['sha256'])
  del value['onnx_sha256']
  pm.validate_catalog(copy.deepcopy(value), value['model_sha256'], 'https://nas.example/precompiled.json')
  value['pickle']['sha256'] = 'b' * 64
  with pytest.raises(ValueError, match='selected model hash'):
    pm.validate_catalog(value, value['model_sha256'], 'https://nas.example/precompiled.json')


def test_generic_install_reuses_verified_download(tmp_path, monkeypatch):
  from openpilot.selfdrive.modeld.big_model import BigModelManifest, model_path
  value, data = catalog()
  value.update(format='comma-generic-onnx', model_sha256=value['pickle']['sha256'])
  # The new runtime carries upstream's warp compiler in place of the old fused module.
  runtime = io.BytesIO()
  with tarfile.open(fileobj=runtime, mode='w:gz') as tar:
    for name in ('examples/openpilot/compile_warp.py', 'tinygrad/__init__.py'):
      entry = tarfile.TarInfo(name)
      entry.size = 1
      tar.addfile(entry, io.BytesIO(b'\n'))
  data['runtime'] = runtime.getvalue()
  value['runtime'].update(size=len(data['runtime']), sha256=hashlib.sha256(data['runtime']).hexdigest())
  model = BigModelManifest('v3', 'big_driving_tinygrad.pkl', len(data['pickle']), value['model_sha256'],
                           'https://nas.example/v3/big_driving_tinygrad.pkl')
  model_path(model, tmp_path).write_bytes(data['pickle'])
  def fetch(request, **kwargs):
    name = request.full_url.rsplit('/', 1)[-1]
    assert name != 'pickle', 'the verified source must be reused'
    return Response(json.dumps(value).encode() if name == 'precompiled.json' else data[name])
  monkeypatch.setattr(pm, 'urlopen', fetch)
  path = pm.ensure_precompiled(model, tmp_path)
  assert path.read_bytes() == data['pickle']
  assert pm.installed(model, tmp_path) == path


@pytest.mark.parametrize('change,allowed', [
  ({}, True), ({'boot_id': 'this-boot'}, False), ({'boot_id': ''}, False),
  ({'pickle_sha256': 'b' * 64}, False), ({'phase': 'inference'}, False),
  ({'error': 'ValueError: checkpoint mismatch'}, False), ({'rejected': False}, False),
  ({'error': 'OSError: Input/Output Error reading model.pkl'}, False),
])
@pytest.mark.parametrize('error', ['RuntimeError: bulk OUT 0x02 failed: Input/Output Error', BOOTLOADER_TIMEOUT,
                                 'RuntimeError: libusb_control_transfer: No such device (it may have been disconnected)'])
def test_old_usb_rejection_requires_matching_previous_boot_evidence(tmp_path, monkeypatch, change, allowed, error):
  from openpilot.selfdrive.modeld import egpu_worker_progress
  value, data = catalog()
  model = SimpleNamespace(sha256='a' * 64, url='https://nas.example/models/v2/model.onnx')
  def fetch(request, **kwargs):
    name = request.full_url.rsplit('/', 1)[-1]
    return Response(json.dumps(value).encode() if name == 'precompiled.json' else data[name])
  monkeypatch.setattr(pm, 'urlopen', fetch)
  monkeypatch.setattr(egpu_worker_progress, 'boot_identity', lambda: 'this-boot')
  path = pm.ensure_precompiled(model, tmp_path)
  failure = {'rejected': True, 'pickle_sha256': value['pickle']['sha256'], 'phase': 'load',
             'boot_id': 'previous-boot', 'error': error} | change
  (path.parent / 'last_failure.json').write_text(json.dumps(failure))
  pm.reject(path)
  # A stale receipt must not bypass a new smoke test after recovery.
  (path.parent / 'boot_validation.json').write_text('{"key": "old"}')
  # Recovered artifacts must still be rehashed/repaired before accepting them.
  path.write_bytes(b'corrupt')
  (path.parent / 'runtime.tar.gz').write_bytes(b'corrupt')
  result = pm.ensure_precompiled(model, tmp_path)
  assert (result == path) == allowed
  assert (path.parent / 'rejected').exists() == (not allowed)
  assert json.loads((path.parent / 'last_failure.json').read_text()) == failure
  if allowed:
    assert path.read_bytes() == data['pickle']
    assert (path.parent / 'runtime.tar.gz').read_bytes() == data['runtime']
    assert not (path.parent / 'boot_validation.json').exists()
  else:
    assert pm.installed(model, tmp_path) is None


@pytest.mark.parametrize('contents', [None, 'invalid', '[]', '{}'])
def test_usb_rejection_is_not_recovered_without_diagnostics(tmp_path, contents):
  if contents is not None:
    (tmp_path / 'last_failure.json').write_text(contents)
  assert not pm.retry_usb_rejection(tmp_path, 'a' * 64)


@pytest.mark.parametrize('phase,allowed', [('load', True), ('boot_validation', True), ('inference', False)])
def test_bootloader_timeout_is_retryable_only_during_startup(tmp_path, phase, allowed):
  path = tmp_path / 'model.pkl'
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  (tmp_path / 'boot_validation.json').write_text('{"key":"old"}')
  assert pm.record_failure(path, RuntimeError(BOOTLOADER_TIMEOUT), phase) == (not allowed)
  assert (tmp_path / 'rejected').exists() == (not allowed)
  assert not (tmp_path / 'boot_validation.json').exists()


@pytest.mark.parametrize('phase', ['load', 'boot_validation', 'inference'])
@pytest.mark.parametrize('operation', ['libusb_control_transfer', 'libusb_bulk_transfer', 'libusb_open'])
@pytest.mark.parametrize('reason', ['No such device (it may have been disconnected)', 'Input/Output Error', 'Operation timed out'])
def test_transport_failure_never_blacklists_model(tmp_path, phase, operation, reason):
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  error = RuntimeError(f'Traceback (most recent call last):\n  worker operation\nRuntimeError: {operation}: {reason}\n')
  assert not pm.record_failure(tmp_path / 'model.pkl', error, phase)
  assert not (tmp_path / 'rejected').exists()
  assert json.loads((tmp_path / 'last_failure.json').read_text())['error'] == str(error)


@pytest.mark.parametrize('error', ['OSError: Input/Output Error reading model.pkl',
                                 'RuntimeError: libusb_control_transfer: Invalid parameter',
                                 'RuntimeError: libusb_control_transfer: Input/Output Error\nValueError: checkpoint mismatch',
                                 'ValueError: quoted libusb_control_transfer: Input/Output Error'])
def test_transport_match_does_not_hide_model_errors(error):
  assert not pm.gpu_transport_failure(error)
