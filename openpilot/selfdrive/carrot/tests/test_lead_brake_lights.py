import json
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot import lead_brake_lights as runtime, signal_mode
from tools.signal_analysis.lead_brake_detector import LeadBrakeDetector


def context():
  lead = dict(status=True, radar=True, radarTrackId=3, dRel=8., yRel=0.)
  vision = dict(prob=.99, x=[9.52], y=[0.])
  car = dict(yawRate=0., steeringAngleDeg=0., vEgo=0.)
  calibration = dict(calStatus='calibrated', height=[1.286], rpyCalib=[0., 0., 0.])
  return lead, vision, car, calibration


def lamp_image():
  rgb = np.zeros((760, 1344, 3), np.uint8)
  rgb[342:348, 660:684, 0] = 255
  return rgb


def test_confirmation_and_missing_lamp_is_unknown_not_off():
  detector = LeadBrakeDetector()
  for t in [1., 1.1]:
    assert detector.process(lamp_image(), t, *context())['state'] == 'unknown'
  assert detector.process(lamp_image(), 1.2, *context())['state'] == 'candidate'
  assert detector.process(np.zeros((760, 1344, 3), np.uint8), 1.25, *context())['state'] == 'unknown'


@pytest.mark.parametrize('kind', ['turn', 'vision', 'lead', 'calibration', 'nan'])
def test_rejects_unreliable_context(kind):
  lead, vision, car, calibration = context()
  if kind == 'turn':car['yawRate'] = .2
  if kind == 'vision':vision['y'] = [2.]
  if kind == 'lead':lead['status'] = False
  if kind == 'calibration':calibration['calStatus'] = 'uncalibrated'
  if kind == 'nan':lead['dRel'] = float('nan')
  detector = LeadBrakeDetector()
  for t in [1., 1.1, 1.2]:
    assert detector.process(lamp_image(), t, lead, vision, car, calibration)['state'] == 'unknown'


def test_neighbor_and_low_tail_lamps_do_not_confirm():
  rgb = np.zeros((760, 1344, 3), np.uint8)
  rgb[342:348, 900:924, 0] = 255
  rgb[410:416, 660:684, 0] = 255
  detector = LeadBrakeDetector()
  for t in [1., 1.1, 1.2]:
    assert detector.process(rgb, t, *context())['state'] == 'unknown'


def test_repeated_frame_and_replacement_lead_do_not_inherit_confirmation():
  detector = LeadBrakeDetector()
  for _ in range(10):
    assert detector.process(lamp_image(), 1., *context())['state'] == 'unknown'
  for t in [1.1, 1.2]:detector.process(lamp_image(), t, *context())
  lead, *rest = context()
  lead['radarTrackId'] = 4
  assert detector.process(lamp_image(), 1.3, lead, *rest)['state'] == 'unknown'


def test_long_gap_requires_new_confirmation():
  detector = LeadBrakeDetector()
  for t in [1., 1.1, 1.2]:detector.process(lamp_image(), t, *context())
  assert detector.process(lamp_image(), 1.6, *context())['state'] == 'unknown'


def test_mode_zero_does_not_read_brake_parameter(monkeypatch):
  monkeypatch.setattr(signal_mode, 'signal_mode', lambda: 0)
  monkeypatch.setattr(runtime, '_params', SimpleNamespace(get_bool=lambda key: pytest.fail('OFF read')))
  assert not runtime.requested()


def test_off_runtime_does_not_create_subscriber_or_write(monkeypatch, tmp_path):
  monkeypatch.setattr(runtime, 'requested', lambda: False)
  monkeypatch.setattr(runtime, 'PATH', tmp_path/'result.json')
  worker = runtime.BrakeRuntime()
  worker.update(None, 1, 1., 's', 1.)
  assert worker.sm is None and worker.detector is None and not list(tmp_path.iterdir())


def test_busy_signal_skips_detector_and_publishes_unknown(monkeypatch, tmp_path):
  monkeypatch.setattr(runtime, 'requested', lambda: True)
  monkeypatch.setattr(runtime.time, 'monotonic', lambda: 2.)
  monkeypatch.setattr(runtime, 'PATH', tmp_path/'result.json')
  worker = runtime.BrakeRuntime()
  worker.update(None, 3, 1.99, 's', 1.9)
  result = json.loads(runtime.PATH.read_text())
  assert worker.sm is None and result['state'] == 'unknown'
  assert result['reason'] == 'signal_budget' and not result['control_permission']


def test_display_stale_and_wrong_camera_never_holds_candidate():
  record = dict(version=1, frame_id=20, timestamp=1., control_permission=False,
                state='candidate', box=[600, 340, 640, 350])
  assert runtime.display_result(record, 1.1, (20, 1.05, 1344, 760, False)) is record
  assert runtime.display_result(record, 1.4, (27, 1.35, 1344, 760, False)) is None
  assert runtime.display_result(record, 1.1, (20, 1.05, 1344, 760, True)) is None


def test_parameter_defaults_off_and_has_single_setting_and_group_reference():
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  settings = (root/'selfdrive/carrot_settings.json').read_text(encoding='utf8')
  assert settings.count('"LeadBrakeLights"') == 2
  assert '{"LeadBrakeLights", {PERSISTENT, BOOL, "0"}}' in (root/'common/params_keys.h').read_text()
