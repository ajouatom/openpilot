from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.monitoring import dm2d
from openpilot.selfdrive.monitoring.test_monitoring import make_msg


def checked_submaster(state, replay=False):
  def create(services, *, poll=None, frequency=None):
    # Preserve the real IPC API contract even in the simulated daemon loop.
    assert frequency is None or poll is None
    expected_poll = 'modelV2' if replay else 'driverStateV2'
    assert poll == expected_poll and poll in services
    return state
  return create


def test_replay_dm_clock_uses_route_timeline(monkeypatch):
  state = SimpleNamespace(logMonoTime={
    'driverStateV2': int(12.05e9), 'modelV2': int(12.0e9),
    'carState': int(12.04e9), 'selfdriveState': int(11.99e9),
  })
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: 100.0)

  assert dm2d.dm_clock(state, replay=True) == 12.05
  assert dm2d.input_packet_fresh(dm2d.dm_clock(state, replay=True), 11.9)

  assert dm2d.dm_clock(state, replay=False) == 100.0
  assert dm2d.input_packet_fresh(100.0, 99.9)
  assert not dm2d.input_packet_fresh(100.0, 12.0)
  assert not dm2d.input_packet_fresh(100.0, float('nan'))


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('touch_signal', ['none', 'held', 'stale'])
@pytest.mark.parametrize('model_ready', [False, True])
def test_missing_or_failed_camera_still_publishes_valid_interaction_monitoring(monkeypatch, experimental, touch_signal, model_ready):
  clock = [100.0]
  packets = []

  class Params:
    def get_bool(self, name):
      return name == "DriverMonitoringEnabled"

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0}

    def update(self, timeout):
      assert timeout == 50
      clock[0] += .05
      if touch_signal == 'held':
        self['carState'].steeringTouch.sampleMonoTime = int(clock[0] * 1e9)

    def all_checks(self, services):
      return 'driverStateV2' not in services

  cs = car.CarState.new_message(vEgo=20, canValid=True, gearShifter='drive')
  if touch_signal != 'none':
    cs.steeringTouch = {'available': True, 'valid': True, 'touched': True, 'sampleMonoTime': int(100e9)}
  model = log.ModelDataV2.new_message()
  if model_ready:
    model.orientationRate.z = [0.] * 33
    model.laneLineProbs = [0., 1., 1., 0.]
  state = State(carState=cs, selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=model.as_reader())

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

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', checked_submaster(state), raising=False)
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
  assert state['alertLevel'] == ('none' if touch_signal == 'held' else 'one')
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


def test_replay_camera_dependencies_expire_on_route_timeline():
  model = log.ModelDataV2.new_message()
  model.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  calibration = log.LiveCalibrationData.new_message(rpyCalib=[0., 0., 0.])

  class State(dict):
    logMonoTime = {
      'driverStateV2': int(100e9),
      'modelV2': int(100e9),
      'liveCalibration': int(100e9),
    }

    def all_checks(self, _):
      return True

  state = State(driverStateV2=make_msg(True), liveCalibration=calibration, modelV2=model)
  assert dm2d.camera_sample_usable(state, False, replay_now=100.0)

  state.logMonoTime['modelV2'] = int(99.49e9)
  assert not dm2d.camera_sample_usable(state, False, replay_now=100.0)
  assert dm2d.camera_sample_usable(state, False, demo=True, replay_now=100.0)

  state.logMonoTime['modelV2'] = int(100e9)
  state.logMonoTime['liveCalibration'] = int(97.49e9)
  assert not dm2d.camera_sample_usable(state, False, replay_now=100.0)

  state.logMonoTime['liveCalibration'] = int(100e9)
  state.logMonoTime['driverStateV2'] = int(99.49e9)
  assert not dm2d.camera_sample_usable(state, False, replay_now=100.0)


def test_disabled_state_packet_is_valid_and_neutral():
  state = dm2d.disabled_state_packet(rhd=True, experimental=True).to_dict()
  assert state['valid']
  dm_state = state['driverMonitoringState']
  assert dm_state['dm2Disabled'] and dm_state['dm2Experimental'] and dm_state['isRHD']
  assert dm_state['alertLevel'] == 'none' and dm_state['activePolicy'] == 'wheeltouch'
  assert not dm_state['lockout'] and not dm_state['alwaysOnLockout'] and not dm_state['noResponseForceDecel']
  assert not dm_state['cameraUnavailable']
  assert dm_state['visionPolicyState']['awarenessPercent'] == 100
  assert dm_state['wheeltouchPolicyState']['awarenessPercent'] == 100


def test_disabled_driver_view_keeps_face_preview_without_enforcement(monkeypatch):
  clock, packets = [100.0], []

  class Params:
    def get_bool(self, name):
      return name == "IsDriverViewEnabled"

    def get_int(self, _):
      return 0

  class State(dict):
    updated = {'carParams': False, 'radarState': False, 'driverStateV2': True}
    logMonoTime = {'driverStateV2': int(clock[0] * 1e9)}

    def update(self, _):
      clock[0] += .05
      self.logMonoTime['driverStateV2'] = int(clock[0] * 1e9)

    def all_checks(self, _services):
      return True

  state = State(carState=car.CarState.new_message(vEgo=0, canValid=True, gearShifter='park'),
                selfdriveState=log.SelfdriveState.new_message(enabled=False),
                driverStateV2=make_msg(True, distracted=True))

  class Bluetooth:
    def read(self, **_kwargs):
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 45:
        raise Done

  class Publisher:
    def send(self, _, packet):
      packets.append(packet.to_dict())

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', checked_submaster(state))
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **kw: None, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **kw: [], raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(Params(), experimental=False)

  preview = packets[-1]['driverMonitoringState']
  assert preview['dm2Disabled'] and preview['visionPolicyState']['faceDetected']
  assert preview['activePolicy'] == 'vision' and preview['alertLevel'] == 'none'
  assert not preview['lockout'] and not preview['noResponseForceDecel']
  assert preview['visionPolicyState']['awarenessPercent'] == 100
  assert not preview['visionPolicyState']['isDistracted']
  assert not any(preview['visionPolicyState']['distractedTypes'].values())


def test_three_physical_cancel_presses_retry_persistent_disable_until_observed(monkeypatch):
  clock, packets = [100.0], []

  class Params:
    def __init__(self):
      self.values = {"DriverMonitoringEnabled": True}
      self.writes = []

    def get_bool(self, name):
      return self.values.get(name, False)

    def get_int(self, _):
      return 0

    def put_bool_nonblocking(self, name, value):
      self.writes.append((name, value))
      # Model an asynchronous first write that is not yet observable. The
      # daemon must keep DM locally disabled and retry at its settings cadence.
      if len(self.writes) == 2:
        self.values[name] = value

  params = Params()

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0}

    def update(self, _):
      clock[0] += .05

    def all_checks(self, services):
      return 'driverStateV2' not in services

  state = State(carState=car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=log.ModelDataV2.new_message())
  edges = iter((True, False, True, False, True, False))

  def drain_sock(sock, **_kwargs):
    if sock != 'carState':
      return []
    try:
      pressed = next(edges)
    except StopIteration:
      return []
    sample = car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive')
    sample.buttonEvents = [{'type': 'cancel', 'pressed': pressed}]
    return [SimpleNamespace(valid=True, logMonoTime=int(clock[0] * 1e9), carState=sample)]

  class Bluetooth:
    def read(self, **_kwargs):
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 43:
        raise Done

  class Publisher:
    def send(self, _, packet):
      packets.append(packet.to_dict())

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **kw: state)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda service, **kw: service, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', drain_sock, raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(params, experimental=False)

  assert params.writes == [("DriverMonitoringEnabled", False)] * 2
  assert params.values["DriverMonitoringEnabled"] is False
  assert any(not packet['driverMonitoringState']['dm2Disabled'] for packet in packets[:4])
  assert all(packet['valid'] and packet['driverMonitoringState']['dm2Disabled'] for packet in packets[4:])


def test_automatic_cancel_echo_racing_between_socket_drains_is_not_counted(monkeypatch):
  clock, packets = [100.0], []

  class Params:
    writes = []

    def get_bool(self, name):
      return name == "DriverMonitoringEnabled"

    def get_int(self, _):
      return 0

    def put_bool_nonblocking(self, name, value):
      self.writes.append((name, value))

  params = Params()

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0}

    def update(self, _):
      clock[0] += .05

    def all_checks(self, services):
      return 'driverStateV2' not in services

  state = State(carState=car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=log.ModelDataV2.new_message())
  edges = iter((True, False, True, False, True, False))
  pending_request = [False]

  def drain_sock(sock, **_kwargs):
    if sock == 'carControl':
      if not pending_request[0]:
        return []
      pending_request[0] = False
      control = SimpleNamespace(cruiseControl=SimpleNamespace(cancel=True))
      # card actuates alive packets regardless of this validity bit.
      return [SimpleNamespace(valid=False, logMonoTime=int((clock[0] - .01) * 1e9), carControl=control)]

    try:
      pressed = next(edges)
    except StopIteration:
      return []
    sample = car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive')
    sample.buttonEvents = [{'type': 'cancel', 'pressed': pressed}]
    pending_request[0] = pressed
    return [SimpleNamespace(valid=True, logMonoTime=int(clock[0] * 1e9), carState=sample)]

  class Bluetooth:
    def read(self, **_kwargs):
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 8:
        raise Done

  class Publisher:
    def send(self, _, packet):
      packets.append(packet.to_dict())

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **kw: state)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda service, **kw: service, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', drain_sock, raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(params, experimental=False)

  assert params.writes == []
  assert all(not packet['driverMonitoringState']['dm2Disabled'] for packet in packets)
  assert all(not packet['driverMonitoringState']['wheeltouchPolicyState']['driverInteracting'] for packet in packets)


def test_replay_route_time_gap_resets_cancel_sequence(monkeypatch):
  route_times = iter((10.0, 10.05, 10.40, 10.45, 10.50, 10.55, 10.60))
  current_time = [0.0]

  class Params:
    writes = []

    def get_bool(self, name):
      return name == "DriverMonitoringEnabled"

    def get_int(self, _):
      return 0

    def put_bool_nonblocking(self, name, value):
      self.writes.append((name, value))

  params = Params()

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0, 'modelV2': 0}

    def update(self, _):
      current_time[0] = next(route_times)
      self.logMonoTime['modelV2'] = int(current_time[0] * 1e9)

    def all_checks(self, services):
      return 'driverStateV2' not in services

  state = State(carState=car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=log.ModelDataV2.new_message())
  edges = iter((True, False, True, False, True, False))

  def drain_sock(sock, **_kwargs):
    if sock != 'carState':
      return []
    try:
      pressed = next(edges)
    except StopIteration:
      return []
    sample = car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive')
    sample.buttonEvents = [{'type': 'cancel', 'pressed': pressed}]
    return [SimpleNamespace(valid=True, logMonoTime=int(current_time[0] * 1e9), carState=sample)]

  class Bluetooth:
    def read(self, **_kwargs):
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 7:
        raise Done

  class Publisher:
    def send(self, *_args):
      pass

  monkeypatch.setenv('REPLAY', '1')
  monkeypatch.setattr(dm2d.messaging, 'SubMaster', checked_submaster(state, replay=True))
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda service, **kw: service, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', drain_sock, raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  with pytest.raises(Done):
    dm2d.run_dm2(params, experimental=False)

  assert params.writes == []


def test_manual_setting_reenables_monitoring_with_fresh_state(monkeypatch):
  clock, packets = [100.0], []

  class Params:
    def __init__(self):
      self.values = {"DriverMonitoringEnabled": False}

    def get_bool(self, name):
      return self.values.get(name, False)

    def get_int(self, _):
      return 0

  params = Params()

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
    logMonoTime = {'driverStateV2': 0}
    frames = 0

    def update(self, _):
      self.frames += 1
      clock[0] += .05
      if self.frames == 41:
        params.values["DriverMonitoringEnabled"] = True

    def all_checks(self, services):
      return 'driverStateV2' not in services

  state = State(carState=car.CarState.new_message(vEgo=10, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True),
                radarState=log.RadarState.new_message(), modelV2=log.ModelDataV2.new_message())

  class Bluetooth:
    def read(self, **_kwargs):
      return None

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame >= 43:
        raise Done

  class Publisher:
    def send(self, _, packet):
      packets.append(packet.to_dict())

  created = []
  original_factory = dm2d.new_driver_monitor

  def factory(*args):
    monitor = original_factory(*args)
    created.append(monitor)
    return monitor

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **kw: state)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: Publisher(), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **kw: None, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **kw: [], raising=False)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: Bluetooth())
  monkeypatch.setattr(dm2d, 'new_driver_monitor', factory)
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **kw: Rate())
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])
  with pytest.raises(Done):
    dm2d.run_dm2(params, experimental=False)

  assert len(created) == 2
  assert all(packet['driverMonitoringState']['dm2Disabled'] for packet in packets[:40])
  assert all(not packet['driverMonitoringState']['dm2Disabled'] for packet in packets[40:])


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('source', ['bt', 'touch'])
def test_live_camera_response_defers_only_experimental_monitoring(monkeypatch, experimental, source):
  clock, packets = [100.0], []

  class Params:
    def get_bool(self, name):
      return name == "DriverMonitoringEnabled"

  class State(dict):
    updated = {'carParams': False, 'radarState': True, 'driverStateV2': True}
    logMonoTime = {'driverStateV2': 0}

    def update(self, _):
      clock[0] += .05
      self.logMonoTime['driverStateV2'] = int(clock[0] * 1e9)
      if source == 'touch':
        self['carState'].steeringTouch = {'available': True, 'valid': True, 'touched': clock[0] >= 103,
                                         'sampleMonoTime': int(clock[0] * 1e9)}

    def all_checks(self, _):
      return True

  model = log.ModelDataV2.new_message()
  model.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  # Production SubMaster exposes read-only Cap'n Proto arrays, not Python
  # lists. Populate the road fields so short-circuiting cannot hide API errors.
  model.orientationRate.z = [0.] * 33
  model.laneLineProbs = [0., 1., 1., 0.]
  state = State(carState=car.CarState.new_message(vEgo=20, canValid=True, gearShifter='drive'),
                selfdriveState=log.SelfdriveState.new_message(enabled=True), modelV2=model.as_reader(),
                radarState=log.RadarState.new_message(), liveCalibration=log.LiveCalibrationData.new_message(rpyCalib=[0., 0., 0.]),
                driverStateV2=make_msg(True, distracted=True))

  class Bluetooth:
    sent = False

    def read(self, allowed, now):
      if source == 'bt' and allowed and not self.sent and now >= 103:
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

  monkeypatch.setattr(dm2d.messaging, 'SubMaster', checked_submaster(state))
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
