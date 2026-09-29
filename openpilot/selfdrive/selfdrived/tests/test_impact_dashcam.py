import ast
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.common.impact_dashcam import FEEDBACK_KEY, NOTICE_KEY, REBOOT_KEY, ImpactDashcam
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.selfdrived.impact_detector import IMPACT_ACCEL, ImpactDetector


class Memory(dict):
  def __init__(self):
    super().__init__()
    self.writes = []

  def put(self, key, value):
    self[key] = value
    self.writes.append((key, value))

  def put_bool(self, key, value):
    self.put(key, value)

  def get_bool(self, key):
    return bool(self.get(key, False))

  def remove(self, key):
    self.pop(key, None)


def sample(detector, t, accel=(0, 0, 0), *, orientation=(0, 0, 0), calibration=(0, 0, 0), **kwargs):
  device = rot_from_euler(calibration) @ np.array(accel) + rot_from_euler(orientation).T @ [0, 0, -9.81]
  return detector.update(now=t, timestamp=t, sensor=-device[::-1], orientation=orientation,
                         calibration=calibration, valid=kwargs.get('valid', True))


@pytest.mark.parametrize('accel', [(IMPACT_ACCEL, 0, 0), (-IMPACT_ACCEL, 0, 0), (0, IMPACT_ACCEL, 0),
                                   (0, -IMPACT_ACCEL, 0), (11, 11, 0)])
def test_two_new_horizontal_samples(accel):
  detector = ImpactDetector()
  assert not sample(detector, 1, accel)[0]
  assert not sample(detector, 1, accel)[0]  # one packet seen twice
  assert sample(detector, 1.01, accel)[0]


@pytest.mark.parametrize('accel', [(0, 0, 0), (-10, 0, 0), (0, 0, 30), (IMPACT_ACCEL - 0.01, 0, 0)])
def test_gravity_aeb_and_vertical_bumps_do_not_trigger(accel):
  detector = ImpactDetector()
  for t in np.arange(1, 1.2, .01):
    assert not sample(detector, t, accel)[0]


def test_mount_and_road_tilt_compensation():
  detector = ImpactDetector()
  orientation, calibration = (0.15, -.2, .5), (.02, .1, -.05)
  for t in (1, 1.01):
    trigger, quiet, accel = sample(detector, t, orientation=orientation, calibration=calibration)
    assert not trigger and quiet
    assert np.linalg.norm(accel) < 1e-10
  for t in (1.02, 1.03):
    result = sample(detector, t, (16, 0, 0), orientation=orientation, calibration=calibration)
  assert result[0]
  assert result[2] == pytest.approx([16, 0, 0])


def test_isolated_spike_gaps_invalid_and_stale_samples():
  detector = ImpactDetector()
  assert not sample(detector, 1, (16, 0, 0))[0]
  assert not sample(detector, 1.01)[0]
  assert not sample(detector, 1.02, (16, 0, 0))[0]
  assert not sample(detector, 1.06, (16, 0, 0))[0]
  assert not sample(detector, 1.07, (16, 0, 0), valid=False)[0]
  assert not sample(detector, 1.08, (16, 0, 0))[0]
  for ts, sensor in [(1, [0, 0, 16]), (2.1, [0, 0, 16]), (2, [float('nan'), 0, 0]), (2, [])]:
    assert detector.update(now=2, timestamp=ts, sensor=sensor, orientation=[0, 0, 0],
                           calibration=[0, 0, 0], valid=True)[:2] == (False, False)


def pending():
  params, memory = Memory(), Memory()
  state = ImpactDashcam(params, memory)
  state.update(now=1, allowed=True, trigger=True, quiet=False)
  assert state.pending
  return state, params, memory


def feedback(state, memory, now, **kwargs):
  memory.put(FEEDBACK_KEY, json.dumps({'token': state.token, 'visible': now, **kwargs}))
  state.update(now=now, allowed=True, trigger=False, quiet=True)


def test_full_visible_countdown_then_durable_off_before_reboot_only_once():
  state, params, memory = pending()
  for index in range(100):
    feedback(state, memory, 1.1 + index / 10)
    assert not params.get_bool('DoReboot')
  feedback(state, memory, 11.1)
  assert params.writes == [('OpenpilotEnabledToggle', False), (REBOOT_KEY, True), ('DoReboot', True)]
  state.update(now=12, allowed=True, trigger=True, quiet=False)
  assert len(params.writes) == 3


def test_touch_at_deadline_cancels_without_changing_settings_and_quiet_rearms():
  state, params, memory = pending()
  for index in range(100):
    feedback(state, memory, 1.1 + index / 10)
  feedback(state, memory, 11.1, cancel=True)
  assert not state.pending and not params.writes and not memory.get(NOTICE_KEY)
  state.update(now=11.2, allowed=True, trigger=True, quiet=False)
  assert not state.pending  # same impulse cannot re-trigger
  state.update(now=12, allowed=True, trigger=False, quiet=True)
  state.update(now=13.01, allowed=True, trigger=False, quiet=True)
  state.update(now=13.02, allowed=True, trigger=True, quiet=False)
  assert state.pending


@pytest.mark.parametrize('mode', ['unseen', 'frozen', 'hidden', 'offroad', 'old_token', 'malformed', 'future'])
def test_no_reboot_without_visible_notice(mode):
  state, params, memory = pending()
  if mode in ('frozen', 'hidden', 'offroad'):
    feedback(state, memory, 1.1)
  if mode == 'old_token':
    memory.put(FEEDBACK_KEY, json.dumps({'token': 'old', 'visible': 12}))
  elif mode == 'malformed':
    memory.put(FEEDBACK_KEY, 'invalid json')
  elif mode == 'future':
    memory.put(FEEDBACK_KEY, json.dumps({'token': state.token, 'visible': 99}))
  state.update(now=12, allowed=mode != 'offroad', trigger=False, quiet=False)
  assert not state.pending and not params.writes


def test_ui_return_after_gap_must_not_skip_unseen_time():
  state, params, memory = pending()
  feedback(state, memory, 1.1)
  feedback(state, memory, 12)
  assert not params.writes


def test_sensor_failure_after_detection_does_not_erase_notice():
  state, params, memory = pending()
  memory.put(FEEDBACK_KEY, json.dumps({'token': state.token, 'visible': 1.1}))
  state.update(now=1.1, allowed=True, trigger=False, quiet=False)
  assert state.pending and not params.writes


def test_settings_write_failure_cannot_request_reboot():
  state, params, memory = pending()
  for index in range(100):
    feedback(state, memory, 1.1 + index / 10)
  def fail(*_args):
    raise OSError('disk full')
  params.put_bool = fail
  with pytest.raises(OSError):
    feedback(state, memory, 11.1)
  assert not state.committed and not params.get_bool('DoReboot')


def test_silent_native_write_failure_requires_off_readback():
  state, params, memory = pending()
  params['OpenpilotEnabledToggle'] = True
  params.put_bool = lambda *_args: None
  for index in range(100):
    feedback(state, memory, 1.1 + index / 10)
  with pytest.raises(OSError, match='OpenpilotEnabledToggle'):
    feedback(state, memory, 11.1)
  assert not state.committed and not params.get_bool('DoReboot')


def test_notice_has_sound_but_cannot_override_takeover_alert():
  from openpilot.selfdrive.selfdrived.events import EVENTS, ET
  notice = EVENTS[log.OnroadEvent.EventName.impactDetected][ET.PERMANENT]
  takeover = EVENTS[log.OnroadEvent.EventName.impactDashcamReboot][ET.IMMEDIATE_DISABLE]
  assert notice.audible_alert == car.CarControl.HUDControl.AudibleAlert.prompt
  assert notice.priority < takeover.priority


def test_always_lateral_is_gated_before_actuator_calculation():
  path = Path(__file__).parents[2] / 'controls/controlsd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  guard = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
               and ast.unparse(n.test) == "self.params.get_bool('ImpactDashcamReboot')")
  cc = car.CarControl.new_message()
  cc.latActive = True
  ns = {'CC': cc}
  exec(compile(ast.fix_missing_locations(ast.Module(body=guard.body, type_ignores=[])), str(path), 'exec'), ns)
  assert not cc.enabled and not cc.latActive and not cc.longActive


def extract_function(path, cls, method, namespace):
  tree = ast.parse(path.read_text(encoding='utf-8'))
  node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
  node = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == method)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(path), 'exec'), namespace)
  return namespace[method]


@pytest.mark.parametrize('blocked', ['', 'replay', 'simulation', 'notCar', 'passive', 'offroad', 'uncalibrated', 'stalePose'])
def test_runtime_sensor_wiring_and_exclusions(blocked):
  now = [1.0]
  clock = SimpleNamespace(monotonic=lambda: now[0])
  sensor = log.SensorEventData.new_message()
  sensor.source = log.SensorEventData.SensorSource.lsm6ds3
  sensor.init('acceleration').v = [9.81, 0, -16]
  pose = log.LivePose.new_message()
  pose.orientationNED.valid = True
  pose.inputsOK = pose.sensorsOK = True
  calibration = log.LiveCalibrationData.new_message()
  calibration.rpyCalib = [0, 0, 0]
  calibration.calStatus = 'uncalibrated' if blocked == 'uncalibrated' else 'calibrated'
  class Messages(dict):
    updated = {'accelerometer': True}
    alive = valid = {'accelerometer': True}
    def all_checks(self, services):
      return True
  sm = Messages(accelerometer=sensor, livePose=pose, liveCalibration=calibration,
                deviceState=SimpleNamespace(started=blocked != 'offroad'))
  state = SimpleNamespace(CP=SimpleNamespace(notCar=blocked == 'notCar', passive=blocked == 'passive'),
    sm=sm, initialized=True, impact_detector=ImpactDetector(), impact_dashcam=ImpactDashcam(Memory(), Memory()))
  method = extract_function(Path(__file__).parents[1] / 'selfdrived.py', 'SelfdriveD', 'update_impact_dashcam',
    {'time': clock, 'log': log, 'REPLAY': blocked == 'replay', 'SIMULATION': blocked == 'simulation',
     'cloudlog': SimpleNamespace(warning=lambda *_: None, exception=lambda *_: None)})
  for t in (1.0, 1.01):
    now[0] = t
    sensor.timestamp = int(t * 1e9)
    pose.timestamp = int((t - 0.3 if blocked == 'stalePose' else t) * 1e9)
    method(state, SimpleNamespace(aEgo=0))
  assert state.impact_dashcam.pending == (blocked == '')


def test_selfdrive_step_disables_and_blocks_reentry():
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.state import StateMachine
  state_machine = StateMachine()
  state_machine.state = log.SelfdriveState.OpenpilotState.enabled
  events = Events()
  def update_events(_cs):
    events.clear()
    events.add(log.OnroadEvent.EventName.buttonEnable)
  drive = SimpleNamespace(data_sample=lambda: object(), update_impact_dashcam=lambda cs: None,
    update_events=update_events, impact_dashcam=SimpleNamespace(pending=False, committed=True), events=events,
    CP=SimpleNamespace(passive=False), initialized=True, state_machine=state_machine,
    update_alerts=lambda cs: None, update_system_ready_alert=lambda cs: None, publish_selfdriveState=lambda cs: None)
  step = extract_function(Path(__file__).parents[1] / 'selfdrived.py', 'SelfdriveD', 'step', {'EventName': log.OnroadEvent.EventName})
  step(drive)
  assert not drive.enabled and not drive.active
  step(drive)
  assert not drive.enabled and not drive.active


def test_final_can_boundary_neutralizes_queued_control():
  path = Path(__file__).parents[2] / 'car/card.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  guard = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
               and ast.unparse(n.test) == "self.params.get_bool('ImpactDashcamReboot')")
  cc = car.CarControl.new_message()
  cc.enabled = cc.latActive = cc.longActive = True
  cc.actuators.accel = 2.0
  cc.actuators.torque = 0.8
  cc.cruiseControl.resume = cc.cruiseControl.override = True
  ns = {'self': SimpleNamespace(params={REBOOT_KEY: True}), 'CC': cc.as_reader(),
        'CS': SimpleNamespace(cruiseState=SimpleNamespace(enabled=True)), 'car': car}
  exec(compile(ast.fix_missing_locations(ast.Module(body=guard.body, type_ignores=[])), str(path), 'exec'), ns)
  result = ns['CC']
  assert not result.enabled and not result.latActive and not result.longActive
  assert result.actuators.accel == result.actuators.torque == 0
  assert result.cruiseControl.cancel and not result.cruiseControl.resume and not result.cruiseControl.override
