import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.monitoring import dm2d
from openpilot.selfdrive.monitoring.test_monitoring import make_msg


@pytest.mark.parametrize('experimental', [False, True])
def test_missing_or_failed_camera_still_publishes_valid_interaction_monitoring(monkeypatch, experimental):
  clock = [100.0]
  packets = []

  class Params:
    def get_bool(self, _):
      return False

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0}

    def update(self, timeout):
      assert timeout == 50
      clock[0] += .05

    def all_checks(self, services):
      return 'driverStateV2' not in services

  cs = car.CarState.new_message(vEgo=20, canValid=True, gearShifter='drive')
  state = State(carState=cs, selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=log.ModelDataV2.new_message())

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 310:
        raise Done

  class Publisher:
    def send(self, service, packet):
      assert service == 'driverMonitoringState'
      packets.append(packet.to_dict())

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **kw: state, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **kw: None, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **kw: [], raising=False)
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(Params(), experimental)
  assert len(packets) == 310 and all(p['valid'] for p in packets)
  assert all(p['driverMonitoringState']['cameraUnavailable'] for p in packets)
  state = packets[-1]['driverMonitoringState']
  assert state['alertLevel'] == 'one'
  assert state['dm2WheelTimeoutFactor'] == 1  # no verified empty-road coverage


def test_malformed_camera_outputs_cannot_reuse_previous_attention():
  model = log.ModelDataV2.new_message()
  model.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  calibration = log.LiveCalibrationData.new_message(rpyCalib=[0., 0., 0.])

  class State(dict):
    def all_checks(self, _):
      return True

  state = State(driverStateV2=make_msg(True), liveCalibration=calibration, modelV2=model)
  assert dm2d.camera_sample_usable(state, False)
  state['driverStateV2'].leftDriverData.faceOrientation = []
  assert not dm2d.camera_sample_usable(state, False)
  state['driverStateV2'] = make_msg(True)
  state['driverStateV2'].leftDriverData.faceOrientation = [float('nan'), 0]
  assert not dm2d.camera_sample_usable(state, False)
  state['driverStateV2'] = make_msg(True)
  state['driverStateV2'].leftDriverData.sleepProb = float('nan')
  assert not dm2d.camera_sample_usable(state, False)


@pytest.mark.parametrize('experimental', [False, True])
def test_live_camera_bt_event_defers_only_experimental_monitoring(monkeypatch, experimental):
  clock, packets = [100.0], []

  class Params:
    def get_bool(self, _):
      return False

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': True}
    logMonoTime = {'driverStateV2': 0}

    def update(self, _):
      clock[0] += .05
      self.logMonoTime['driverStateV2'] = int(clock[0] * 1e9)

    def all_checks(self, _):
      return True

  model = log.ModelDataV2.new_message()
  model.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  state = State(carState=car.CarState.new_message(vEgo=20, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True), modelV2=model,
                radarState=log.RadarState.new_message(), liveCalibration=log.LiveCalibrationData.new_message(rpyCalib=[0., 0., 0.]),
                driverStateV2=make_msg(True, distracted=True))

  class Bluetooth:
    sent = False

    def read(self, allowed, now):
      if allowed and not self.sent and now >= 103:
        self.sent = True
        return 'none'  # an unmapped real BT button still counts
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 1220:
        raise Done

  class Publisher:
    def send(self, _, packet):
      packets.append((clock[0], packet.to_dict()))

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **kw: state)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **kw: None, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **kw: [], raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(Params(), experimental)
  steady = [p['driverMonitoringState'] for t, p in packets if 104 < t < 147]
  if experimental:
    assert all(p['alertLevel'] == 'none' and p['dm2InteractionGraceRemaining'] > 0 for p in steady)
    assert packets[-1][1]['driverMonitoringState']['alertLevel'] == 'one'
  else:
    assert all(p['dm2InteractionGraceRemaining'] == 0 for p in steady)
    assert packets[-1][1]['driverMonitoringState']['alertLevel'] == 'three'
