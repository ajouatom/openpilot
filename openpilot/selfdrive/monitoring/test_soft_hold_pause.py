from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.monitoring import dm2d
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.policy import AlertLevel
from openpilot.selfdrive.monitoring.test_monitoring import make_msg


class Messages(dict):
  healthy = True

  def __init__(self):
    super().__init__(carState=car.CarState.new_message(canValid=True, gearShifter='drive',
                                                     standstill=True, softHoldActive=1),
                     selfdriveState=log.SelfdriveState.new_message(enabled=True, active=True))
    self.logMonoTime = dict.fromkeys(self, int(100e9))

  def all_checks(self, _):
    return self.healthy


@pytest.mark.parametrize('hold', [1, 2])
def test_stationary_soft_hold_pauses_without_changing_engagement(hold):
  sm = Messages(); sm['carState'].softHoldActive = hold
  assert dm2d.soft_hold_pause_eligible(sm, 100.)
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = False
  assert dm2d.soft_hold_pause_eligible(sm, 100.)


@pytest.mark.parametrize('fault', ['released', 'moving', 'raw_moving', 'nan', 'raw_nan', 'not_stopped',
  'gas', 'park', 'reverse', 'neutral', 'unknown', 'can_invalid', 'unhealthy', 'stale_car', 'stale_state', 'future', 'demo'])
def test_pause_does_not_survive_release_motion_or_missing_evidence(fault):
  sm = Messages(); cs = sm['carState']
  if fault == 'released': cs.softHoldActive = 0
  elif fault == 'moving': cs.vEgo = .1
  elif fault == 'raw_moving': cs.vEgoRaw = .1
  elif fault == 'nan': cs.vEgo = float('nan')
  elif fault == 'raw_nan': cs.vEgoRaw = float('nan')
  elif fault == 'not_stopped': cs.standstill = False
  elif fault == 'gas': cs.gasPressed = True
  elif fault in ('park', 'reverse', 'neutral', 'unknown'): cs.gearShifter = fault
  elif fault == 'can_invalid': cs.canValid = False
  elif fault == 'unhealthy': sm.healthy = False
  elif fault == 'stale_car': sm.logMonoTime['carState'] = int(99.7e9)
  elif fault == 'stale_state': sm.logMonoTime['selfdriveState'] = int(99.7e9)
  elif fault == 'future': sm.logMonoTime['carState'] = int(101e9)
  assert not dm2d.soft_hold_pause_eligible(sm, 100., demo=fault == 'demo')


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('always_on', [False, True])
def test_dispatcher_freezes_assessment_and_resumes_next_frame(monkeypatch, experimental, camera, always_on):
  sm = Messages(); clock = [100.]; packets = []; snapshots = []
  dm = DriverMonitoring2(experimental=experimental, always_on=always_on)
  dm.awareness = dm.last_vision_awareness = dm.last_wheeltouch_awareness = .7
  dm.alert_level = AlertLevel.one
  dm.alert_3_cnt = dm.no_response_cnt = 1
  dm.too_distracted = True  # a hold must not erase the existing saved lockout
  camera_step = Mock(wraps=dm.run_step); fallback_step = Mock(wraps=dm.run_without_camera)
  monkeypatch.setattr(dm, 'run_step', camera_step)
  monkeypatch.setattr(dm, 'run_without_camera', fallback_step)
  model = log.ModelDataV2.new_message()
  model.orientationRate.z = [0.] * 33
  model.laneLineProbs = [0., 1., 1., 0.]
  model.meta.disengagePredictions.brakeDisengageProbs = [0.]
  sm.update(modelV2=model.as_reader(), radarState=log.RadarState.new_message(),
            driverStateV2=make_msg(True, distracted=True),
            liveCalibration=log.LiveCalibrationData.new_message(rpyCalib=[0., 0., 0.]))
  sm.updated = {'carParams': False, 'radarState': True, 'driverStateV2': True}

  def update(_):
    clock[0] += .05
    sm.logMonoTime = dict.fromkeys(sm, int(clock[0] * 1e9))
    if len(packets) >= 120:
      sm['carState'].softHoldActive = 0
      sm['carState'].standstill = False
      sm['carState'].vEgo = sm['carState'].vEgoRaw = 20.

  def send(_, packet):
    packets.append(packet.to_dict())
    snapshots.append((dm.awareness, dm.alert_3_cnt, dm.no_response_cnt, dm.too_distracted,
                      camera_step.call_count, fallback_step.call_count))

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame == 122: raise Done

  sm.update = update
  monkeypatch.delenv('REPLAY', raising=False)
  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **k: sm)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: NS(send=send))
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **k: None)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **k: [])
  monkeypatch.setattr(dm2d, 'DriverMonitoring2', lambda **k: dm)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: NS(read=lambda **k: None))
  monkeypatch.setattr(dm2d, 'CameraAvailability', lambda: NS(update=lambda *a: camera))
  monkeypatch.setattr(dm2d, 'camera_sample_usable', lambda *a, **k: camera)
  monkeypatch.setattr(dm2d, 'DmRatekeeper', lambda *a, **k: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  params = NS(get_bool=lambda key: key == 'DriverMonitoringEnabled' or key == 'AlwaysOnDM' and always_on,
              get_int=lambda _: int(experimental), put_bool=Mock(), put_bool_nonblocking=Mock())
  with pytest.raises(Done): dm2d.run_dm2(params, experimental)
  for packet, snapshot in zip(packets[:120], snapshots[:120], strict=True):
    state = packet['driverMonitoringState']
    assert packet['valid'] and state['dm2Disabled'] and state['alertLevel'] == 'none'
    assert not state['noResponseForceDecel'] and not state['alwaysOnLockout']
    assert not state['visionPolicyState']['isDistracted']
    assert snapshot == (.7, 1, 1, True, 0, 0)
  for packet in packets[120:]:
    assert not packet['driverMonitoringState']['dm2Disabled']
    assert packet['driverMonitoringState']['lockout']  # original lockout survives
  assert camera_step.call_count == (2 if camera else 0)
  assert fallback_step.call_count == (0 if camera else 2)
  params.put_bool.assert_not_called()
  params.put_bool_nonblocking.assert_not_called()
