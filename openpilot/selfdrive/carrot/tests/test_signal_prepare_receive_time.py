from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare
from openpilot.selfdrive.carrot.signal_stop_prepare_runtime import SignalPrepareRuntime
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import context, geometry, observation


def runtime_fixture(monkeypatch, receipt_offset=0., main_age=0., receive_delay=.002):
  import time
  from openpilot.selfdrive.carrot import signal_mode

  clock = [10.]
  class SM(dict):
    def update(self, timeout):
      clock[0] += receive_delay
      self.recv_time = dict.fromkeys(self, clock[0]+receipt_offset)

  extra = SM(liveCalibration=NS(calStatus='calibrated', rpyCalib=[0., 0., 0.]),
             roadCameraState=NS(sensor='os04c10'), deviceState=NS(deviceType='mici'))
  extra.seen = extra.valid = extra.alive = dict.fromkeys(extra, True)
  extra.recv_time = dict.fromkeys(extra, 10.)
  g = geometry()
  model = NS(roadEdges=g['edges'], roadEdgeStds=g['edge_stds'], laneLines=g['lane_lines'],
             laneLineProbs=g['lane_probs'], laneLineStds=g['lane_stds'])
  pose = NS(timestamp=10*10**9, inputsOK=True, posenetOK=True, sensorsOK=True,
            orientationNED=NS(valid=True),
            angularVelocityDevice=NS(valid=True, x=0., y=0., z=0., xStd=.001, yStd=.001, zStd=.001))
  sm = SM(livePose=pose, modelV2=model, carState=NS(), radarState=NS(), selfdriveState=NS(), carControl=NS())
  sm.seen = sm.valid = sm.alive = dict.fromkeys(sm, True)
  sm.recv_time = dict.fromkeys(sm, 10.-main_age)
  monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(signal_mode, 'signal_mode', lambda: 2)
  monkeypatch.setattr(SignalAssistRuntime, 'read', lambda *a: (
    observation(clock[0]), {'now': clock[0], 'enabled': True, 'valid': True, 'drive': True}))
  runtime = SignalPrepareRuntime.__new__(SignalPrepareRuntime)
  runtime.mode, runtime.mode_since = None, -float('inf')
  runtime.enabled, runtime.extra, runtime.prepare = True, extra, SignalStopPrepare()
  return runtime, sm, clock


def test_newly_received_camera_data_does_not_reset_confirmation(monkeypatch):
  runtime, sm, clock = runtime_fixture(monkeypatch)
  decisions = []
  for i in range(17):
    clock[0] = 10.+i*.05
    sm.recv_time = dict.fromkeys(sm, clock[0])
    sm['livePose'].timestamp = round(clock[0]*1e9)
    obs, ctx = runtime.read(sm)
    assert ctx['valid'] and ctx['now'] == pytest.approx(clock[0])
    args = context()
    args.update(enabled=ctx['enabled'], valid=ctx['valid'], drive=ctx['drive'],
                turning=ctx['turning'], geometry=ctx['geometry'], alignment=ctx['alignment'])
    decisions.append(runtime.prepare.update(ctx['now'], obs, **args))
  assert decisions[-1].active
  assert sum(d.entered for d in decisions) == 1


@pytest.mark.parametrize('receipt_offset,main_age,receive_delay', [
  (.01, 0., .002),   # Truly future receipt still fails.
  (-3., 0., .002),   # Stale extra camera/calibration data still fails.
  (0., .249, .01),   # Main inputs can expire while receiving extra data.
  (0., 0., .3),     # A long receive also expires pose and main input freshness.
])
def test_resampling_time_does_not_relax_freshness(monkeypatch, receipt_offset, main_age, receive_delay):
  runtime, sm, _ = runtime_fixture(monkeypatch, receipt_offset, main_age, receive_delay)
  assert not runtime.read(sm)[1]['valid']
