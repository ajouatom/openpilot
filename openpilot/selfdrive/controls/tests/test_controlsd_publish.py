import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.carrot.carrot_man_input import get_carrot_man
from openpilot.selfdrive.monitoring.dm_alerts import driver_monitoring_hud_alert
from opendbc.car.hyundai.stop_object import signal_stop_object_requested
from opendbc.car.hyundai.values import HyundaiFlags


class Messages(dict):
  def __init__(self):
    super().__init__({
      'carState': car.CarState.new_message(canValid=True, gearShifter='drive'),
      'carOutput': car.CarOutput.new_message(),
      'longitudinalPlan': log.LongitudinalPlan.new_message(shouldStop=True, xState=3),
      'radarState': log.RadarState.new_message(),
      'carrotMan': new_message('carrotMan').carrotMan,
      'selfdriveState': log.SelfdriveState.new_message(active=True),
      'modelV2': log.ModelDataV2.new_message(),
      'driverMonitoringState': log.DriverMonitoringState.new_message(),
      'driverAssistance': new_message('driverAssistance').driverAssistance,
    })
    self.frame = 100
    self.recv_frame = dict.fromkeys(self, self.frame)
    self.logMonoTime = dict.fromkeys(self, 0)
    self.valid = dict.fromkeys(self, True)
    self.seen = dict.fromkeys(self, False)

  def all_checks(self, services):
    return all(self.valid[service] for service in services)


def new_message(service):
  message = log.Event.new_message()
  message.init(service)
  return message


def publish_fixture(brand, flags, enabled):
  # Execute the entire production method with real cereal messages. Only live
  # IPC, settings and the physical blinker reader are replaced.
  path = Path(__file__).parents[1] / 'controlsd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  controls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Controls')
  publish = next(node for node in controls.body if isinstance(node, ast.FunctionDef) and node.name == 'publish')
  namespace = {
    'car': car, 'log': log, 'CV': CV, 'DT_CTRL': DT_CTRL,
    'State': log.SelfdriveState.OpenpilotState,
    'get_carrot_man': get_carrot_man,
    'driver_monitoring_hud_alert': driver_monitoring_hud_alert,
    'signal_stop_object_requested': signal_stop_object_requested,
    'messaging': NS(new_message=new_message),
  }
  exec(compile(ast.Module(body=[publish], type_ignores=[]), str(path), 'exec'), namespace)
  cp = car.CarParams.new_message(brand=brand, flags=flags, openpilotLongitudinalControl=True)
  cp.lateralTuning.init('pid')
  sent = {}
  control = NS(CP=cp, sm=Messages(), curvature=0.0, desired_curvature=0.0, calibrated_pose=None,
               physical_blinker_hold=NS(read=lambda valid: 0), is_vw_meb=False,
               params=NS(get_int=lambda key: 0), lanefull_mode_enabled=False,
               LoC=NS(long_control_state='off', pid=NS(p=0.0, i=0.0, f=0.0)),
               pm=NS(send=lambda service, message: sent.update({service: message})))
  cc = car.CarControl.new_message(enabled=enabled, longActive=enabled)
  lac_log = log.ControlsState.LateralPIDState.new_message()
  return control, sent, lambda: namespace['publish'](control, cc, lac_log)


@pytest.mark.parametrize('brand,flags', [
  ('hyundai', int(HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC)),
  ('hyundai', 0),
  ('toyota', 0),
])
@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('service,age,valid', [
  ('longitudinalPlan', 0, True),
  ('longitudinalPlan', 20, True),
  ('longitudinalPlan', 21, True),
  ('longitudinalPlan', -1, True),
  ('longitudinalPlan', 0, False),
  ('radarState', 20, True),
  ('radarState', 21, True),
  ('radarState', -1, True),
  ('radarState', 0, False),
])
def test_complete_publish_checks_message_age_and_publishes_both_services(brand, flags, enabled, service, age, valid):
  control, sent, publish = publish_fixture(brand, flags, enabled)
  control.sm.recv_frame[service] -= age
  control.sm.valid[service] = valid
  publish()
  assert set(sent) == {'controlsState', 'carControl'}
  assert all(message.valid for message in sent.values())
  expected = brand == 'hyundai' and flags != 0 and enabled and valid and 0 <= age <= 20
  assert sent['carControl'].carControl.hudControl.signalStopObject == expected
