"""A new checkout must never start a different cached external model."""
import hashlib
import sys
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld import big_model as bm, helpers, precompiled_model


def cached_model(directory, model_id, data):
  model = bm.BigModelManifest(model_id, 'model.pkl', len(data), hashlib.sha256(data).hexdigest(),
                              f'https://test.example/{model_id}.pkl')
  bm.model_path(model, directory).write_bytes(data)
  return model


@pytest.fixture
def selection(tmp_path, monkeypatch):
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  monkeypatch.delenv('CARROT_BIG_MODEL_MANIFEST', raising=False)
  monkeypatch.delenv('CARROT_BIG_MODEL_STARTUP_FAILED', raising=False)
  old = cached_model(tmp_path, 'old', b'old model')
  new = cached_model(tmp_path, 'new', b'new model')
  bm._write_state(old, None, tmp_path)
  monkeypatch.setattr(bm, 'fetch_manifest', lambda *args: new)
  return old, new


def test_branch_switch_rejects_installed_old_model_then_selects_new(selection, tmp_path, monkeypatch):
  old, new = selection
  calls = []
  def installed(model, *args):
    calls.append(model.sha256)
    return tmp_path / model.model_id / 'model.pkl'
  monkeypatch.setattr(precompiled_model, 'installed', installed)
  assert bm.active_manifest() == old  # Storage metadata alone is not execution permission.
  assert bm.selected_manifest() is None
  assert helpers.usbgpu_compiled_path() is None
  assert not bm.active_model_compiled()
  assert calls == []
  # Cache already contains the desired verified source; delivery selects it.
  bm.ensure_big_model(cache_dir=tmp_path)
  assert bm.selected_manifest() == new
  assert helpers.usbgpu_compiled_path() == tmp_path / 'new' / 'model.pkl'
  assert calls == [new.sha256]
  assert bm.read_state()['previous'] == old


def test_missing_new_runtime_never_selects_previous(selection, tmp_path, monkeypatch):
  old, new = selection
  bm._write_state(new, old, tmp_path)
  monkeypatch.setattr(precompiled_model, 'installed', lambda *args: None)
  assert helpers.usbgpu_compiled_path() is None


def test_permanent_startup_failure_keeps_internal_for_this_boot(selection, tmp_path, monkeypatch):
  old, new = selection
  bm._write_state(new, old, tmp_path)
  monkeypatch.setenv('CARROT_BIG_MODEL_STARTUP_FAILED', '1')
  monkeypatch.setattr(precompiled_model, 'installed', lambda *args: pytest.fail('startup failed'))
  assert bm.selected_manifest() is None
  assert helpers.usbgpu_compiled_path() is None


def test_explicit_catalog_selection_requires_delivery_provenance(selection, tmp_path, monkeypatch):
  _, new = selection
  catalog = 'https://test.example/experiment/manifest.json'
  monkeypatch.setenv('CARROT_BIG_MODEL_MANIFEST', catalog)
  bm._write_state(new, None, tmp_path)
  assert bm.selected_manifest() is None
  # Even when bytes are already cached, establish which catalog selected them.
  assert bm.ensure_big_model(catalog, tmp_path)[1] is False
  assert bm.selected_manifest() == new
  monkeypatch.setenv('CARROT_BIG_MODEL_MANIFEST', 'https://test.example/other/manifest.json')
  assert bm.selected_manifest() is None


@pytest.mark.parametrize('success', [True, False])
def test_startup_cli_waits_for_delivery_and_propagates_failure(selection, monkeypatch, success):
  events = []
  class Spinner:
    def __enter__(self):
      events.append('open')
      return self
    def __exit__(self, *args):
      events.append('close')
    def update(self, text):
      events.append(text)
  def deliver(url, cache_dir, reporter, *, retry_network):
    assert url == bm.DEFAULT_MANIFEST_URL
    assert retry_network
    reporter.update('waiting_for_network', error_code='dns', retry_in_seconds=30)
    reporter.download_progress(selection[1], 5, 10)
    events.append('delivery finished')
    return success
  monkeypatch.setitem(sys.modules, 'openpilot.common.spinner', SimpleNamespace(Spinner=Spinner))
  monkeypatch.setattr(bm, 'deliver_model', deliver)
  monkeypatch.setattr(sys, 'argv', ['big_model', '--prepare-for-startup', '--retry-network'])
  assert bm.main() == (0 if success else 1)
  assert events[0] == 'open'
  assert 'Waiting for network' in events[1]
  assert events[-2:] == ['delivery finished', 'close']


def test_cli_active_sha_cannot_advertise_wrong_branch_model(selection, monkeypatch, capsys):
  monkeypatch.setattr(sys, 'argv', ['big_model', '--active-sha'])
  assert bm.main() == 0
  assert capsys.readouterr().out == '\n'
