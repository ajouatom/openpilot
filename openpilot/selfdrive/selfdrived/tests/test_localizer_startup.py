import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.basedir import BASEDIR


def localizer_check():
  # Exercise the real event conditions without importing vehicle-only processes.
  tree = ast.parse((Path(BASEDIR) / 'openpilot/selfdrive/selfdrived/selfdrived.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_events')
  block = next(n for n in method.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'not self.CP.notCar')
  module = ast.parse('def check(self, cal_status, TESTING_CLOSET=False, SIMULATION=False, REPLAY=False): pass')
  module.body[0].body = [block]
  namespace = {'log': log, 'EventName': SimpleNamespace(posenetInvalid='posenet', locationdTemporaryError='location',
                                                     paramsdTemporaryError='params')}
  exec(compile(ast.fix_missing_locations(module), '<localizer startup>', 'exec'), namespace)
  return namespace['check']


@pytest.mark.parametrize('pose_seen, params_seen, expected', [
  (False, False, set()), (True, False, {'posenet', 'location'}),
  (False, True, {'params'}), (True, True, {'posenet', 'location', 'params'}),
])
def test_only_received_localizer_messages_raise_alerts(pose_seen, params_seen, expected):
  class Messages(dict):
    seen = {'livePose': pose_seen, 'liveParameters': params_seen}

  sm = Messages(livePose=SimpleNamespace(posenetOK=False, inputsOK=False), liveParameters=SimpleNamespace(valid=False))
  state = SimpleNamespace(CP=SimpleNamespace(notCar=False), sm=sm, events=set())
  localizer_check()(state, log.LiveCalibrationData.Status.calibrated)
  assert state.events == expected
  sm['livePose'].posenetOK = sm['livePose'].inputsOK = sm['liveParameters'].valid = True
  state.events.clear()
  localizer_check()(state, log.LiveCalibrationData.Status.calibrated)
  assert not state.events


@pytest.mark.parametrize('not_car, calibrated, closet, simulation, replay, expected', [
  (True, True, False, False, False, set()),
  (False, False, False, False, False, {'posenet', 'location'}),
  (False, True, True, False, False, {'posenet', 'location'}),
  (False, True, False, True, False, {'posenet', 'location'}),
  (False, True, False, True, True, {'posenet', 'location', 'params'}),
])
def test_existing_localizer_alert_exemptions(not_car, calibrated, closet, simulation, replay, expected):
  class Messages(dict):
    seen = {'livePose': True, 'liveParameters': True}

  sm = Messages(livePose=SimpleNamespace(posenetOK=False, inputsOK=False), liveParameters=SimpleNamespace(valid=False))
  state = SimpleNamespace(CP=SimpleNamespace(notCar=not_car), sm=sm, events=set())
  status = log.LiveCalibrationData.Status.calibrated if calibrated else log.LiveCalibrationData.Status.uncalibrated
  localizer_check()(state, status, closet, simulation, replay)
  assert state.events == expected
