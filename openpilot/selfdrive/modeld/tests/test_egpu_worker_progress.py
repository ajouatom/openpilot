import json
import struct

import pytest

from openpilot.selfdrive.modeld import egpu_worker_progress as progress


def test_snapshot_captures_last_call_and_bounded_thread_state(tmp_path):
  path = tmp_path / 'progress'
  worker = progress.WorkerProgress(path)
  try:
    worker.frame = 17
    worker.mark('output_read')
    snapshot = progress.worker_snapshot(path, 123, tmp_path)
    task = tmp_path / '123' / 'task' / str(snapshot['thread_id'])
    task.mkdir(parents=True)
    (task / 'schedstat').write_text('100 200 3')
    (task / 'wchan').write_text('x' * 3000)
    snapshot = progress.worker_snapshot(path, 123, tmp_path)
    assert snapshot['stage'] == 'output_read' and snapshot['frame'] == 17
    assert snapshot['stage_age_ms'] >= 0 and snapshot['thread_cpu_seconds'] >= 0
    assert snapshot['schedstat'] == '100 200 3' and len(snapshot['wchan']) == 2048
    # A record interrupted while publishing must not be presented as evidence.
    struct.pack_into('<Q', worker.memory, 0, 7)
    assert 'stage' not in progress.worker_snapshot(path, 123, tmp_path)
  finally:
    worker.close()


def test_missing_diagnostics_do_not_prevent_inference(tmp_path):
  path = tmp_path / 'missing' / 'progress'
  worker = progress.WorkerProgress(path)
  worker.mark('model_call')
  worker.close()
  assert progress.worker_snapshot(path, 123, tmp_path) == {'pid': 123}


@pytest.mark.parametrize('error,code', [
  ('RuntimeError: PCIe link not up (LTSSM=0x00)', 'pcie'),
  ('precompiled eGPU worker timed out', 'timeout'),
  ('LIBUSB_ERROR_IO', 'usb'), ('bulk OUT failed', 'usb'),
  ('Input/output error', 'usb'), ('unexpected exit', 'runtime'),
])
def test_classify_observed_failure_without_claiming_hardware_cause(error, code):
  assert progress.runtime_failure_code(error) == code


@pytest.mark.parametrize('boot,sha,worker,expected', [
  ('current', 'a' * 64, {'stage': 'output_read'}, 'pcie'),
  ('previous', 'a' * 64, {}, None), ('current', 'b' * 64, {}, None),
  ('current', 'a' * 64, 'malformed', 'pcie'),
])
def test_only_current_boot_and_model_can_supply_failure_details(tmp_path, monkeypatch, boot, sha, worker, expected):
  monkeypatch.setattr(progress, 'boot_identity', lambda: 'current')
  folder = tmp_path / 'precompiled' / ('a' * 64)
  folder.mkdir(parents=True)
  (folder / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  (folder / 'last_failure.json').write_text(json.dumps({
    'boot_id': boot, 'pickle_sha256': sha, 'worker': worker, 'error': 'PCIe link not up',
  }))
  result = progress.current_failure(tmp_path, 'a' * 64)
  assert result.get('error_code') == expected
  assert isinstance(result.get('worker', {}), dict)


def test_legacy_onnx_directory_uses_installed_pickle_identity(tmp_path, monkeypatch):
  monkeypatch.setattr(progress, 'boot_identity', lambda: 'current')
  folder = tmp_path / 'precompiled' / ('a' * 64)
  folder.mkdir(parents=True)
  (folder / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'b' * 64}}))
  (folder / 'last_failure.json').write_text(json.dumps({
    'boot_id': 'current', 'pickle_sha256': 'b' * 64, 'error': 'PCIe link not up',
  }))
  assert progress.current_failure(tmp_path, 'a' * 64)['error_code'] == 'pcie'


def test_bad_json_is_ignored(tmp_path, monkeypatch):
  monkeypatch.setattr(progress, 'boot_identity', lambda: 'current')
  folder = tmp_path / 'precompiled' / ('a' * 64)
  folder.mkdir(parents=True)
  (folder / 'last_failure.json').write_text('[')
  assert progress.current_failure(tmp_path, 'a' * 64) == {}
  assert progress.current_failure(tmp_path, '../bad') == {}
