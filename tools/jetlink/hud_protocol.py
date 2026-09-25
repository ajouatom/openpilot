"""Read-only display snapshots over a negotiated Jetlink extension."""
import base64
import json
from pathlib import Path
import struct
import subprocess
import sys
import time

HUD_MESSAGE = 0x4000
HUD_CAPABILITY = 'carrot_hud_v1'
MAX_HUD_BYTES = 512 * 1024
HUD_PACKET = Path('/dev/shm/carrot-jetlink-hud.packet')
HEADER = struct.Struct('<d')
SERVICES = ('carState', 'carParams', 'modelV2', 'radarState', 'liveTracks', 'longitudinalPlan',
            'lateralPlan', 'controlsState', 'selfdriveState', 'carControl', 'carOutput', 'deviceState',
            'roadCameraState', 'cameraOdometry', 'liveCalibration', 'livePose', 'drivingModelData',
            'liveDelay', 'liveParameters', 'liveTorqueParameters', 'navInstruction', 'navInstructionCarrot',
            'navRoute', 'carrotMan', 'carrotNavi', 'wideRoadCameraState')
PARAMS = ('ClusterHud', 'ClusterHudDebug', 'ClusterHudBrightness', 'ClusterHudOrientation',
          'ClusterHudMirror', 'ClusterHudTheme', 'ClusterHudRadarInfo', 'ClusterHudRadarDisplay',
          'ClusterHudRadarSourceColor', 'ClusterHudCameraViewMode', 'ClusterHudPanelLayout',
          'ClusterHudScreenMode', 'ShowPlotMode', 'LanguageSetting', 'IsMetric', 'ShowDateTime',
          'IsOnroad', 'CarParams', 'CalibrationParams', 'CustomSteerRatio', 'SteerActuatorDelay',
          'LateralAccelerationFactor', 'UsbGpuActive')


class Publisher:
  """Keep cereal decoding, JSON and preview work outside the USB owner's GIL."""
  def __init__(self):
    import os
    self.path = Path(f'/dev/shm/carrot-jetlink-display-{os.getpid()}.packet')
    self.last_sent = 0.
    self.process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('hud_publisher.py')), str(self.path)],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

  def packet(self):
    try:
      with self.path.open('rb') as f:
        data = f.read(96 * 1024 + HEADER.size + 1)
      if not HEADER.size < len(data) <= 96 * 1024 + HEADER.size:
        return None
      stamp, = HEADER.unpack_from(data)
      if stamp == self.last_sent or not 0 <= time.monotonic() - stamp < .3:
        return None
      self.last_sent = stamp
      return data[HEADER.size:]
    except (OSError, struct.error):
      return None

  def close(self):
    self.process.terminate()
    try:
      self.process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=1)
    self.path.unlink(missing_ok=True)
    self.path.with_suffix('.tmp').unlink(missing_ok=True)


class SnapshotBuilder:
  def __init__(self):
    from openpilot.cereal import messaging
    from openpilot.common.params import Params
    self.sm = messaging.SubMaster(list(SERVICES))
    self.params = Params()
    self.cached = {}
    self.settings = {}
    self.next_params = self.next_send = 0.
    from hud_camera import CameraPublisher
    self.camera = CameraPublisher()

  def close(self):
    self.camera.close()

  def packet(self):
    from openpilot.cereal import log
    now = time.monotonic()
    if now < self.next_send:
      return None
    self.next_send = now + .1 if now - self.next_send > .2 else self.next_send + .1
    self.sm.update(0)
    for name in SERVICES:
      if self.sm.updated[name]:
        event = log.Event.new_message(logMonoTime=self.sm.logMonoTime[name], valid=self.sm.valid[name])
        setattr(event, name, self.sm[name])
        self.cached[name] = base64.b64encode(event.to_bytes()).decode()
    if now >= self.next_params:
      self.settings = {}
      for key in PARAMS:
        try:
          value = self.params.get(key)
          if value is not None:
            if isinstance(value, bool):
              value = b'1' if value else b'0'
            self.settings[key] = base64.b64encode(value if isinstance(value, bytes) else str(value).encode()).decode()
        except Exception:
          pass
      self.next_params = now + 1
    record = {'version': 1, 'sent': now, 'events': self.cached, 'params': self.settings,
              'received': self.sm.recv_time, 'mono': self.sm.logMonoTime,
              'valid': self.sm.valid, 'alive': self.sm.alive, 'cameras': self.camera.latest}
    data = json.dumps(record, separators=(',', ':')).encode()
    if len(data) > 96 * 1024:
      record['cameras'] = {}
      data = json.dumps(record, separators=(',', ':')).encode()
    if len(data) > 96 * 1024:
      record['events'] = {k:v for k,v in self.cached.items() if k != 'navRoute'}
      data = json.dumps(record, separators=(',', ':')).encode()
    if len(data) > 96 * 1024:
      raise ValueError('HUD snapshot exceeds bounded USB allocation')
    return data


_cached_snapshot = None
_next_snapshot_read = 0.


def read_snapshot():
  global _cached_snapshot, _next_snapshot_read
  now = time.monotonic()
  if now < _next_snapshot_read:
    return _cached_snapshot
  _next_snapshot_read = now + .02
  _cached_snapshot = None
  try:
    with HUD_PACKET.open('rb') as f:
      data = f.read(MAX_HUD_BYTES + HEADER.size + 1)
    if len(data) > MAX_HUD_BYTES + HEADER.size:
      return None
    received, = HEADER.unpack_from(data)
    if not 0 <= time.monotonic() - received < .5:
      return None
    value = json.loads(data[HEADER.size:])
    if value.get('version') == 1:
      _cached_snapshot = received, value
      return _cached_snapshot
  except (OSError, ValueError, struct.error):
    pass
  return None
