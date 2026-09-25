import json

import pytest

from openpilot.common import jetlink_status as status


@pytest.mark.parametrize(('peer', 'expected'), (
  ({'carrot_host': 'jetson'}, 'jetSON'), ({'carrot_host': 'mac'}, 'MAC'),
  ({'backend': 'trt', 'device': 'Orin-sm87'}, 'jetSON'),
  ({'backend': 'ort', 'device': 'coreml-Apple-M1'}, 'MAC'),
  ({'backend': 'trt', 'device': 'RTX-sm89'}, 'Jetlink'),
  ({'carrot_host': 'untrusted text'}, 'Jetlink'), ({'carrot_host': []}, 'Jetlink'), (None, 'Jetlink'),
))
def test_host_identity_compatibility(peer, expected):
  assert status.host_label(peer) == expected


def test_active_ready_and_expired_host_status(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 10.)
  link.write_text(json.dumps({'updated': 9.9, 'state': 'ready', 'peer': {'carrot_host': 'mac'}}))
  assert status.badge() == ('MAC READY', 'ready')
  model.write_text(json.dumps({'updated': 9.9, 'active': True}))
  assert status.badge() == ('MAC', 'active')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 14.)
  assert status.badge() is None
  link.write_text('invalid')
  model.write_text('[]')
  assert status.badge() is None


def test_remote_badge_disappears_with_expired_snapshot(monkeypatch):
  import hud
  params = hud.DisplayParams()
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (10., {'external_compute_label': 'jetSON'}))
  assert params.external_compute_label() == 'jetSON'
  monkeypatch.setattr(hud, 'read_snapshot', lambda: None)
  assert params.external_compute_label() == ''
