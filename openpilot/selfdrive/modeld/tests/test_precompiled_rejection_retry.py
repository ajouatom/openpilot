"""Boot boundary and integrity checks for persisted numerical failures."""
import json
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld import egpu_worker_progress, precompiled_model as pm
from openpilot.selfdrive.modeld.tests.test_precompiled_model import Response, catalog


@pytest.mark.parametrize('error,phase,old_boot,allowed', [
  ('ValueError: invalid precompiled model output', 'inference', True, True),
  ('non-finite model output', 'inference', True, True),
  ('NonFiniteModelOutput: invalid numbers', 'inference', True, True),
  ('RepeatedInvalidModelOutput: two consecutive invalid eGPU outputs', 'inference', True, True),
  ('ValueError: invalid precompiled model output', 'inference', False, False),
  ('ValueError: invalid precompiled model output', 'boot_validation', True, False),
  ('ValueError: invalid precompiled model output: count=1, expected=2', 'inference', True, False),
  ('ValueError: checkpoint mismatch', 'inference', True, False),
  ('NonFiniteModelOutput: invalid\nValueError: checksum mismatch', 'inference', True, False),
])
def test_reboot_retry_rehashes_artifacts_and_requires_new_smoke_test(tmp_path, monkeypatch, capsys, error, phase, old_boot, allowed):
  value, data = catalog()
  model = SimpleNamespace(sha256='a' * 64, url='https://nas.example/models/v2/model.onnx')
  def fetch(request, **kwargs):
    name = request.full_url.rsplit('/', 1)[-1]
    return Response(json.dumps(value).encode() if name == 'precompiled.json' else data[name])
  monkeypatch.setattr(pm, 'urlopen', fetch)
  monkeypatch.setattr(egpu_worker_progress, 'boot_identity', lambda: 'this-boot')
  path = pm.ensure_precompiled(model, tmp_path)
  failure = {'rejected': True, 'pickle_sha256': value['pickle']['sha256'], 'phase': phase,
             'boot_id': 'previous-boot' if old_boot else 'this-boot', 'error': error}
  (path.parent / 'last_failure.json').write_text(json.dumps(failure))
  pm.reject(path)
  receipt = path.parent / 'boot_validation.json'
  receipt.write_text('{"key":"old"}')
  path.write_bytes(b'corrupt')
  (path.parent / 'runtime.tar.gz').write_bytes(b'corrupt')
  result = pm.ensure_precompiled(model, tmp_path)
  events = [json.loads(line.removeprefix('[eGPU] ')) for line in capsys.readouterr().err.splitlines()]
  assert events[-1]['event'] == ('artifact_retry' if allowed else 'artifact_blocked')
  if not allowed:
    assert events[-1]['retry'] is False and events[-1]['error'] == error.splitlines()[-1]
  assert (result == path) == allowed
  assert (path.parent / 'rejected').exists() == (not allowed)
  assert json.loads((path.parent / 'last_failure.json').read_text()) == failure
  if allowed:
    assert path.read_bytes() == data['pickle']
    assert (path.parent / 'runtime.tar.gz').read_bytes() == data['runtime']
    assert not receipt.exists()
  else:
    assert pm.installed(model, tmp_path) is None
