import asyncio
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld.big_model_status import write_big_model_status


@pytest.fixture
def feature(monkeypatch, tmp_path):
  spec = importlib.util.spec_from_file_location(
    "openpilot.selfdrive.carrot.server.features.egpu_model_test",
    Path(__file__).parents[1] / "features/egpu_model.py",
  )
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  monkeypatch.setattr(module, "model_cache_dir", lambda: tmp_path)
  monkeypatch.setattr(module, "active_manifest", lambda: SimpleNamespace(model_id="old-model", sha256="a" * 64, size=100))
  monkeypatch.setattr(module, "active_model_compiled", lambda: True)
  return module


def payload(feature, engaged=False):
  return feature.build_status_payload(SimpleNamespace(get_bool=lambda key: key == "UsbGpuHardwareSeen" or (key == "IsEngaged" and engaged)))


@pytest.mark.parametrize('state,expected', [('waiting', 'recovering'), ('recovering', 'recovering'),
                                         ('ready', 'recovery_ready'), ('stopped', 'recovery_stopped')])
def test_live_recovery_status_overrides_old_failure_without_offering_reboot(feature, monkeypatch, state, expected):
  monkeypatch.setattr(feature, 'read_egpu_recovery', lambda: {'state': state, 'attempts': 1})
  params = SimpleNamespace(get_bool=lambda key: key in ('UsbGpuHardwareSeen', 'UsbGpuStartupFailed'))
  result = feature.build_status_payload(params)
  assert result['state'] == expected and result['error_code'] is None
  assert not result['can_restart'] and not result['active']


def test_active_gpu_wins_over_recovery_status(feature, monkeypatch):
  monkeypatch.setattr(feature, 'read_egpu_recovery', lambda: {'state': 'ready'})
  params = SimpleNamespace(get_bool=lambda key: key in ('UsbGpuHardwareSeen', 'UsbGpuActive'))
  result = feature.build_status_payload(params)
  assert result['active'] and result['recovery'] == {}


@pytest.mark.parametrize("state", ["checking", "downloading", "verifying", "compiling", "error"])
@pytest.mark.parametrize("sha", ["a" * 64, "b" * 64])
def test_update_phase_is_not_hidden_by_compiled_model(feature, tmp_path, state, sha):
  write_big_model_status(tmp_path, state, model_id="updating-model", sha256=sha,
                         downloaded_bytes=0, total_bytes=200, detail="current update")
  result = payload(feature)
  assert result["state"] == state
  assert result["model_id"] == "updating-model"
  assert result["sha256"] == sha
  assert result["compiled"] is False
  assert result["downloaded_bytes"] == 0
  assert result["total_bytes"] == 200
  assert result["progress"] == 0
  assert result["detail"] == "current update"
  assert result["can_restart"] is (state == "error" and sha == "a" * 64)


def test_downloading_model_becomes_ready_then_compiled(feature, monkeypatch, tmp_path):
  new = SimpleNamespace(model_id="new-model", sha256="b" * 64, size=200)
  write_big_model_status(tmp_path, "downloading", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=50, total_bytes=200)
  result = payload(feature)
  assert (result["state"], result["progress"], result["compiled"], result["can_restart"]) == ("downloading", 25, False, False)

  # Verification finishes before the downloader replaces its status record.
  monkeypatch.setattr(feature, "active_manifest", lambda: new)
  monkeypatch.setattr(feature, "active_model_compiled", lambda: False)
  write_big_model_status(tmp_path, "verifying", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=200, total_bytes=200)
  assert payload(feature)["state"] == "verifying"
  write_big_model_status(tmp_path, "ready", model_id=new.model_id, sha256=new.sha256,
                         downloaded_bytes=200, total_bytes=200)
  result = payload(feature)
  assert (result["state"], result["progress"], result["compiled"], result["can_restart"]) == ("ready", 100, False, True)
  assert payload(feature, engaged=True)["can_restart"] is False
  write_big_model_status(tmp_path, "waiting_for_ignition", model_id=new.model_id, sha256=new.sha256)
  assert payload(feature)["state"] == "waiting_for_ignition"
  write_big_model_status(tmp_path, "compiling", model_id=new.model_id, sha256=new.sha256)
  assert payload(feature)["can_restart"] is False
  monkeypatch.setattr(feature, "active_model_compiled", lambda: True)
  assert payload(feature)["state"] == "compiling"
  write_big_model_status(tmp_path, "compiled", model_id=new.model_id, sha256=new.sha256)
  result = payload(feature)
  assert (result["state"], result["compiled"], result["can_restart"]) == ("compiled", True, False)


@pytest.mark.parametrize("state", ["ready", "waiting_for_ignition", "compiled"])
def test_other_models_status_cannot_offer_compilation(feature, tmp_path, state):
  write_big_model_status(tmp_path, state, model_id="new-model", sha256="b" * 64)
  result = payload(feature)
  assert result["state"] == "checking"
  assert result["compiled"] is False
  assert result["can_restart"] is False
  assert result["downloaded_bytes"] == 0
  assert result["total_bytes"] == 0


def test_missing_compiled_artifact_downgrades_saved_compiled_status(feature, monkeypatch, tmp_path):
  write_big_model_status(tmp_path, "compiled", model_id="old-model", sha256="a" * 64)
  monkeypatch.setattr(feature, "active_model_compiled", lambda: False)
  result = payload(feature)
  assert (result["state"], result["compiled"], result["can_restart"]) == ("ready", False, True)


def test_unidentified_update_does_not_borrow_old_model_details(feature, tmp_path):
  write_big_model_status(tmp_path, "checking")
  result = payload(feature)
  assert result["state"] == "checking"
  assert result["model_id"] is None
  assert result["sha256"] is None
  assert result["total_bytes"] == 0
  assert result["compiled"] is False


def test_no_status_uses_verified_active_model(feature):
  result = payload(feature)
  assert result["model_id"] == "old-model"
  assert result["state"] == "compiled"
  assert result["compiled"] is True
  assert result["progress"] == 100


def test_card_hidden_without_egpu_history(feature):
  assert feature.build_status_payload(SimpleNamespace(get_bool=lambda _key: False)) == {"ok": True, "available": False}


@pytest.mark.parametrize('state', ['waiting_for_network', 'installing'])
def test_auto_delivery_explains_failure_without_requesting_reboot(feature, tmp_path, state):
  write_big_model_status(tmp_path, state, sha256='a' * 64, error_code='dns', detail='resolver unavailable',
                         retry_count=2, retry_in_seconds=30)
  result = payload(feature)
  assert result['state'] == state
  assert result['error_code'] == 'dns' and result['retry_in_seconds'] == 30
  assert not result['can_restart']


def test_installed_runtime_is_not_claimed_to_be_running(feature, tmp_path):
  write_big_model_status(tmp_path, 'installed', sha256='a' * 64)
  result = payload(feature)
  assert result['state'] == 'installed' and result['compiled']
  assert not result['active'] and not result['can_restart']


def test_runtime_failure_overrides_stale_successful_installation(feature, tmp_path):
  write_big_model_status(tmp_path, 'compiled', sha256='a' * 64)
  result = feature.build_status_payload(SimpleNamespace(get_bool=lambda k: k in {'UsbGpuHardwareSeen', 'UsbGpuStartupFailed'}))
  assert result['state'] == 'error' and result['error_code'] == 'runtime'


def test_current_failure_explained_but_never_overrides_active_gpu(feature, monkeypatch):
  monkeypatch.setattr(feature, 'current_failure', lambda *args: {'error_code': 'timeout', 'worker': {'stage': 'output_read'}})
  flags = {'UsbGpuHardwareSeen', 'UsbGpuStartupFailed'}
  params = SimpleNamespace(get_bool=lambda key: key in flags)
  result = feature.build_status_payload(params)
  assert result['error_code'] == 'timeout' and result['detail'] == 'Last worker stage: output_read'
  flags.add('UsbGpuActive')
  result = feature.build_status_payload(params)
  assert result['active'] and result['error_code'] != 'timeout'


@pytest.mark.parametrize('marker', [False, True])
def test_rejected_model_never_offers_ineffective_compile_restart(feature, tmp_path, marker):
  write_big_model_status(tmp_path, 'error', sha256='a' * 64, error_code='rejected' if not marker else 'runtime')
  if marker:
    folder = tmp_path / 'precompiled' / ('a' * 64)
    folder.mkdir(parents=True)
    (folder / 'rejected').write_text('a' * 64)
  assert not payload(feature)['can_restart']


def test_compile_restart_rejects_blocked_model_without_reboot(feature, monkeypatch, tmp_path):
  writes = []
  params = SimpleNamespace(get_bool=lambda key: key == 'UsbGpuHardwareSeen', put_bool=lambda *args: writes.append(args))
  monkeypatch.setattr(feature, 'HAS_PARAMS', True)
  monkeypatch.setattr(feature, 'Params', lambda: params)
  write_big_model_status(tmp_path, 'error', sha256='a' * 64, error_code='rejected')
  assert asyncio.run(feature.api_compile_restart(None)).status == 409
  assert not writes


def test_diagnostic_download_preserves_original_failure_after_reboot(feature, tmp_path):
  folder = tmp_path / 'precompiled' / ('a' * 64)
  folder.mkdir(parents=True)
  (folder / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'b' * 64}}))
  failure = {'pickle_sha256': 'b' * 64, 'boot_id': 'yesterday', 'rejected': True,
             'phase': 'inference', 'error': 'ValueError: invalid precompiled model output'}
  (folder / 'last_failure.json').write_text(json.dumps(failure | {'unrelated_private_data': 'do not export'}))
  response = asyncio.run(feature.api_diagnostics(None))
  assert response.status == 200 and response.headers['Cache-Control'] == 'no-store'
  assert 'attachment' in response.headers['Content-Disposition']
  assert json.loads(response.text)['failure'] == failure
  (folder / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'c' * 64}}))
  assert asyncio.run(feature.api_diagnostics(None)).status == 404

def test_library_only_allows_curated_selection_even_offroad(feature, monkeypatch, tmp_path):
  from openpilot.common import big_model_catalog as C
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  monkeypatch.setattr(C, 'boot_id', lambda: 'same-boot')
  C.freeze_boot()
  monkeypatch.setattr(feature, 'HAS_PARAMS', True)
  monkeypatch.setattr(feature, 'Params', lambda: SimpleNamespace(get_bool=lambda key: key == 'IsOnroad'))
  class Request:
    def __init__(self, action, model): self.body = {'action': action, 'model': model}
    async def json(self): return self.body
  response = asyncio.run(feature.api_library(Request('select', 'flip-flop')))
  assert response.status == 200
  value = json.loads(response.body)['library']
  assert value['selected'] == 'flip-flop' and value['effective'] == 'n16' and value['reboot_required']
  assert asyncio.run(feature.api_library(Request('delete', 'md-v3'))).status == 409
  monkeypatch.setattr(feature, 'Params', lambda: SimpleNamespace(get_bool=lambda key: key == 'IsOffroad'))
  assert asyncio.run(feature.api_library(Request('delete', 'n16'))).status == 409
  assert asyncio.run(feature.api_library(Request('delete', 'md-v3'))).status == 409
  assert asyncio.run(feature.api_library(Request('restore', 'md-v3'))).status == 409
  assert asyncio.run(feature.api_library(Request('select', '../../anything'))).status == 409


def test_library_maintenance_runs_off_request_thread_and_stops(feature, monkeypatch):
  from aiohttp import web
  calls = []
  monkeypatch.setattr(feature, 'maintain_library_once', lambda: calls.append('run'))
  app = web.Application()
  feature.register(app)
  assert feature.library_maintenance in app.cleanup_ctx
  async def run():
    context = feature.library_maintenance(app)
    await anext(context)
    for _ in range(100):
      if calls:
        break
      await asyncio.sleep(.01)
    assert calls == ['run']
    await context.aclose()
    await asyncio.sleep(.01)
    assert calls == ['run']
  asyncio.run(run())


def test_library_retired_entry_cannot_be_selected(feature, monkeypatch, tmp_path):
  from openpilot.common import big_model_catalog as C
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  models = C.catalog()
  models['md-v3']['retired'] = True
  monkeypatch.setattr(C, 'catalog', lambda: models)
  class Request:
    async def json(self): return {'action': 'select', 'model': 'md-v3'}
  assert asyncio.run(feature.api_library(Request())).status == 409
  assert not (tmp_path / 'selection.json').exists()
