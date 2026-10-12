"""Separate default-OFF transport for experimental moving red preparation."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime, requested, MAX_BYTES
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, housing_range, housing_alignment, night_lamp_support

ENABLE_PATH = Path('/data/signal-color-shadow/stop_prepare_enabled')
OBSERVATION_PATH = Path('/dev/shm/carrot_signal_prepare_observation.json')


def publish_prepare_observation(record, directory=ENABLE_PATH.parent, path=OBSERVATION_PATH):
  if not requested(Path(directory)/ENABLE_PATH.name):
    return
  # Dedicated opt-in can observe the comparison engine without enabling the old
  # latched moving-red assist or giving the display any control permission.
  obj = dict(version=1, stream='road', size=[1344, 760], session=record['session'],
             frame_id=record['frame_id'], timestamp=record['timestamp_eof']/1e9,
             tracks=record['tracks'][:20])
  data = json.dumps(obj, allow_nan=False)
  if len(data.encode()) > MAX_BYTES:
    raise ValueError('prepare observation too large')
  path = Path(path); tmp = path.with_suffix('.tmp')
  tmp.write_text(data); tmp.replace(path)


class SignalPrepareRuntime(SignalAssistRuntime):
  def __init__(self):
    super().__init__(ENABLE_PATH, OBSERVATION_PATH)
    self.prepare = SignalStopPrepare()
    self.extra = None
    self.last_prepare_log = None
    self.mode = None
    self.mode_since = -math.inf

  def read(self, sm, now=None):
    obs, context = super().read(sm, now)
    from openpilot.selfdrive.carrot.signal_mode import signal_mode
    mode = signal_mode()
    if mode != self.mode:
      self.mode, self.mode_since = mode, context['now']
      self.prepare.reset('mode_change')
    context['actuate'] = mode == 2
    if obs is not None and obs.get('timestamp', -math.inf) < self.mode_since:
      obs = None
    context.update(geometry=lambda box: None, alignment=None, night_geometry=None, turning=True)
    if not self.enabled:
      context['valid'] = False
      return obs, context
    if self.extra is None:
      from openpilot.cereal import messaging
      self.extra = messaging.SubMaster(['liveCalibration', 'roadCameraState', 'deviceState'])
    try:
      from openpilot.common.transformations.camera import DEVICE_CAMERAS
      from openpilot.common.transformations.orientation import rot_from_euler
      self.extra.update(0)
      if now is None:
        # SubMaster timestamps receipts inside update(). Comparing them with
        # the earlier transport-read time rejects newly arrived data as future
        # data and repeatedly clears the preparation confirmation history.
        context['now'] = time.monotonic()
        services = ('carState', 'modelV2', 'radarState', 'selfdriveState', 'carControl')
        context['valid'] = context['valid'] and all(
          0 <= context['now']-sm.recv_time[s] <= .25 for s in services)
      now = context['now']; extra = self.extra
      ok = all(extra.seen[k] and extra.valid[k] and extra.alive[k]
               and 0 <= now-extra.recv_time[k] <= age
               for k, age in [('liveCalibration', 2.), ('roadCameraState', .25), ('deviceState', 2.)])
      calib = extra['liveCalibration']
      config = DEVICE_CAMERAS.get((str(extra['deviceState'].deviceType), str(extra['roadCameraState'].sensor)))
      # Current observer coordinates match OS04C10 road frames only.
      ok = ok and config is not None and config.fcam.size == (1344, 760) and str(calib.calStatus) == 'calibrated'
      rpy = list(calib.rpyCalib)
      pose = sm['livePose']
      ok = (ok and len(rpy) == 3 and np.isfinite(rpy).all()
            and sm.seen['livePose'] and sm.valid['livePose'] and sm.alive['livePose']
            and 0 <= now-sm.recv_time['livePose'] <= .2 and 0 <= now-pose.timestamp/1e9 <= .2
            and pose.inputsOK and pose.posenetOK and pose.sensorsOK
            and pose.angularVelocityDevice.valid and pose.orientationNED.valid)
      if not ok:
        context['valid'] = False
        return obs, context
      angular = pose.angularVelocityDevice
      omega = np.array([angular.x, angular.y, angular.z])
      std = np.array([angular.xStd, angular.yStd, angular.zStd])
      if omega.shape != (3,) or std.shape != (3,) or not np.isfinite(omega).all() or not np.isfinite(std).all() or np.any(std < 0):
        context['valid'] = False
        return obs, context
      yaw = math.degrees((rot_from_euler(rpy).T @ omega)[2])
      context['turning'] = abs(yaw) > 1.5 or math.degrees(np.linalg.norm(std)) > 2.
      model = sm['modelV2']
      context['geometry'] = lambda box: housing_range(
        box, config.fcam.focal_length, rpy, model.roadEdges, model.roadEdgeStds,
        model.laneLines, model.laneLineProbs, model.laneLineStds)
      context['alignment'] = lambda box, distance: housing_alignment(
        box, distance, config.fcam.focal_length, rpy, model.roadEdges, model.roadEdgeStds,
        model.laneLines, model.laneLineProbs, model.laneLineStds)
      context['night_geometry'] = lambda box: night_lamp_support(
        box, config.fcam.focal_length, rpy, model.roadEdges, model.roadEdgeStds,
        model.laneLines, model.laneLineProbs, model.laneLineStds)
    except (AttributeError, KeyError, TypeError, ValueError, IndexError):
      context['valid'] = False
    return obs, context

  def record_prepare(self, decision):
    if not self.enabled and self.last_prepare_log is None:
      return
    value = asdict(decision)
    value.update(mode=self.mode, control_active=decision.active and self.mode == 2)
    if value != self.last_prepare_log:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('signalStopPrepare', **value)
      self.last_prepare_log = value
