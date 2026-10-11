import hashlib
import errno
import io
import json
import socket
import ssl
import tarfile
from urllib.error import HTTPError, URLError

import pytest

from openpilot.selfdrive.modeld import big_model as bm, precompiled_model as pm
from openpilot.selfdrive.modeld.big_model_status import BigModelStatusReporter, delivery_badge, read_big_model_status


@pytest.fixture
def package(tmp_path, monkeypatch):
  data = b'verified pickle'
  sha = hashlib.sha256(data).hexdigest()
  model = bm.BigModelManifest('test-v3', 'model.pkl', len(data), sha, 'https://test.example/model.pkl')
  bm.model_path(model, tmp_path).write_bytes(data)
  bm._write_state(model, model, tmp_path)
  monkeypatch.setattr(bm, 'fetch_manifest', lambda *a: model)
  archive = io.BytesIO()
  with tarfile.open(fileobj=archive, mode='w:gz') as tar:
    for name in ('examples/openpilot/compile_warp.py', 'tinygrad/__init__.py'):
      entry = tarfile.TarInfo(name)
      entry.size = 1
      tar.addfile(entry, io.BytesIO(b'\n'))
  runtime = archive.getvalue()
  catalog = {'protocol': 1, 'format': 'comma-generic-onnx', 'gpu_arch': 'gfx1200', 'frame_skip': 4,
             'camera_resolutions': [[1928, 1208], [1344, 760]], 'model_sha256': sha,
             'pickle': {'url': 'model.pkl', 'size': len(data), 'sha256': sha},
             'runtime': {'url': 'runtime.tar.gz', 'size': len(runtime), 'sha256': hashlib.sha256(runtime).hexdigest()}}
  return model, catalog, runtime


class Response(io.BytesIO):
  status = 200


def test_existing_model_dns_then_clock_error_recovers_complete_runtime(package, tmp_path, monkeypatch):
  model, catalog, runtime = package
  calls, sleeps, snapshots = [], [], []
  reporter = BigModelStatusReporter(tmp_path)
  def sleep(seconds):
    sleeps.append(seconds)
    snapshots.append(read_big_model_status(tmp_path))
  monkeypatch.setattr(bm.time, 'sleep', sleep)
  def fetch(request, **kwargs):
    url = request.full_url
    calls.append(url)
    if len(calls) == 1:
      raise URLError(socket.gaierror(-3, 'Temporary failure in name resolution'))
    if len(calls) == 2:
      raise URLError(ssl.SSLCertVerificationError(1, 'certificate is not yet valid'))
    assert not url.endswith('model.pkl'), 'reuse the existing verified source'
    return Response(json.dumps(catalog).encode() if url.endswith('precompiled.json') else runtime)
  monkeypatch.setattr(pm, 'urlopen', fetch)
  assert bm.deliver_model(bm.DEFAULT_MANIFEST_URL, tmp_path, reporter, retry_network=True)
  assert sleeps == [30, 30]
  assert [v['error_code'] for v in snapshots] == ['dns', 'clock']
  assert all(v['state'] == 'waiting_for_network' and v['retry_in_seconds'] == 30 for v in snapshots)
  assert pm.installed(model, tmp_path).is_file()
  assert read_big_model_status(tmp_path)['state'] == 'installed'


@pytest.mark.parametrize('error,code', [(HTTPError('https://test', 404, 'missing', {}, None), 'download'),
                                      (ssl.SSLCertVerificationError(1, 'hostname mismatch'), 'certificate'),
                                      (ValueError('hash mismatch'), 'install')])
def test_permanent_failures_preserve_checks_and_do_not_retry(package, tmp_path, monkeypatch, error, code):
  def fail(*a, **kw):
    raise error
  monkeypatch.setattr(pm, 'ensure_precompiled', fail)
  monkeypatch.setattr(bm.time, 'sleep', lambda *a: pytest.fail('permanent failure must not loop'))
  assert not bm.deliver_model(bm.DEFAULT_MANIFEST_URL, tmp_path, BigModelStatusReporter(tmp_path), retry_network=True)
  status = read_big_model_status(tmp_path)
  assert (status['state'], status['error_code'], status['retry_in_seconds']) == ('error', code, 0)


def test_rejected_artifact_is_not_silently_unblocked(package, tmp_path, monkeypatch):
  monkeypatch.setattr(pm, 'ensure_precompiled', lambda *a: None)
  assert not bm.deliver_model(bm.DEFAULT_MANIFEST_URL, tmp_path, BigModelStatusReporter(tmp_path), retry_network=True)
  assert read_big_model_status(tmp_path)['error_code'] == 'rejected'


def test_installed_package_needs_no_network_or_repeat_hashing(package, tmp_path, monkeypatch):
  monkeypatch.setattr(pm, 'installed', lambda *a: tmp_path / 'model.pkl')
  monkeypatch.setattr(pm, 'ensure_precompiled', lambda *a: pytest.fail('must reuse installed package'))
  assert bm.deliver_model(bm.DEFAULT_MANIFEST_URL, tmp_path, BigModelStatusReporter(tmp_path), retry_network=True)
  assert read_big_model_status(tmp_path)['state'] == 'compiled'


@pytest.mark.parametrize('status,text', [({'state': 'waiting_for_network', 'error_code': 'dns'}, 'eGPU NET'),
                                        ({'state': 'waiting_for_network', 'error_code': 'clock'}, 'eGPU CLOCK'),
                                        ({'state': 'installing'}, 'eGPU SETUP'),
                                        ({'state': 'installed'}, 'eGPU NEXT'),
                                        ({'state': 'error', 'error_code': 'rejected'}, 'eGPU FILE')])
def test_delivery_badge_explains_current_phase(status, text):
  assert delivery_badge(status)[0] == text


@pytest.mark.parametrize('error,expected', [(TimeoutError('slow'), ('network', True)),
                                          (OSError(errno.ENOSPC, 'disk full'), ('storage', False)),
                                          (HTTPError('https://test', 503, 'busy', {}, None), ('server', True))])
def test_delivery_error_classification(error, expected):
  assert bm.delivery_error(error) == expected


@pytest.mark.parametrize('kind', ['source', 'runtime'])
def test_interrupted_download_retains_verified_resume_path(tmp_path, monkeypatch, kind):
  from openpilot.selfdrive.modeld.tests.test_big_model import FakeResponse
  data = b'0123456789'
  model = bm.BigModelManifest('resume', 'model.pkl', len(data), hashlib.sha256(data).hexdigest(), 'https://test/model.pkl')
  path = bm.model_path(model, tmp_path)
  calls = []
  def fetch(request, **kwargs):
    calls.append(request)
    if len(calls) == 1:
      return FakeResponse(data[:4])
    assert request.headers['Range'] == 'bytes=4-'
    return FakeResponse(data[4:], status=206, headers={'Content-Range': 'bytes 4-9/10'})
  monkeypatch.setattr(bm if kind == 'source' else pm, 'urlopen', fetch)
  def download():
    if kind == 'source':
      bm._download_model(model, tmp_path)
    else:
      pm.download({'size': len(data), 'sha256': model.sha256, 'url': model.url}, path)
  with pytest.raises(ConnectionError):
    download()
  assert not path.exists()
  assert path.with_suffix('.pkl.part').read_bytes() == data[:4]
  download()
  assert path.read_bytes() == data
