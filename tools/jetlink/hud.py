"""Run the existing Carrot renderer using read-only, USB-forwarded cereal data."""
import base64
import json
import os
from pathlib import Path
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from hud_protocol import read_snapshot, SERVICES


def require_usb_display(scan, expected):
  product = scan(expected)
  if product is None:
    raise RuntimeError('Jetson USB display disappeared before opening; retrying')
  return product


def wait_for_usb_display(scan, expected, sleep=time.sleep):
  announced = False
  while scan(expected) is None:
    if not announced:
      print('Waiting for Jetson USB display', flush=True)
      announced = True
    sleep(1.)


class DisplayParams:
  """Only this renderer sees this adapter; no vehicle Params can be written."""
  def __init__(self, *args, **kwargs):
    self.next_read = 0.
    self.values = {}

  def get(self, key, *args, **kwargs):
    now = time.monotonic()
    if now >= self.next_read:
      record = read_snapshot()
      # Missing telemetry is not an ignition-off command. Keep the last
      # settings until a fresh C4 snapshot explicitly changes them; SubMaster
      # still invalidates all vehicle data immediately on snapshot loss.
      if record is not None:
        self.values = record[1]['params']
      self.next_read = now + .1
    value = self.values.get(key)
    return base64.b64decode(value) if value is not None else None

  def get_bool(self, key, *args, **kwargs):
    return self.get(key) in (b'1', b'True')

  def external_compute_label(self):
    snapshot = read_snapshot()
    label = snapshot[1].get('external_compute_label', '') if snapshot else ''
    return label if label in ('jetSON', 'MAC', 'Jetlink') else ''

  def get_int(self, key, *args, **kwargs):
    try:
      return int(self.get(key) or 0)
    except ValueError:
      return 0

  def get_float(self, key, *args, **kwargs):
    try:
      return float(self.get(key) or 0)
    except ValueError:
      return 0.

  def put_bool_nonblocking(self, key, value):
    if key == 'ClusterHudConnected':
      Path('/dev/shm/carrot-jetlink-hud-connected').write_text(str(bool(value)))


class RemoteSubMaster:
  def __init__(self, services):
    from openpilot.cereal import log
    self.log = log
    self.services = services
    self.data = {}
    self.updated = dict.fromkeys(services, False)
    self.valid = dict.fromkeys(services, False)
    self.alive = dict.fromkeys(services, False)
    self.recv_time = dict.fromkeys(services, 0.)
    self.logMonoTime = dict.fromkeys(services, 0)
    self.generations = {}
    self.last_received = 0.
    for name in services:
      self.data[name] = log.Event.new_message(**{name: {}}) if name != 'liveTracks' else log.Event.new_message(liveTracks={})

  def __getitem__(self, key):
    return getattr(self.data[key], key)

  def update(self, timeout=0):
    self.updated = dict.fromkeys(self.services, False)
    snapshot = read_snapshot()
    if snapshot is None:
      self.valid = dict.fromkeys(self.services, False)
      self.alive = dict.fromkeys(self.services, False)
      return
    received, value = snapshot
    if received == self.last_received:
      return
    self.last_received = received
    shift = received - value['sent']
    for name in self.services:
      if name not in SERVICES:
        continue
      self.valid[name] = bool(value['valid'].get(name, False))
      self.alive[name] = bool(value['alive'].get(name, False))
      self.recv_time[name] = value['received'].get(name, 0.) + shift
      mono = value['mono'].get(name, 0)
      self.logMonoTime[name] = int(mono + shift * 1e9)
      if mono == self.generations.get(name) or name not in value['events']:
        continue
      raw = base64.b64decode(value['events'][name])
      with self.log.Event.from_bytes(raw) as reader:
        if reader.which() != name:
          raise ValueError('display service identity mismatch')
        self.data[name] = reader.as_builder()
      self.generations[name] = mono
      self.updated[name] = True


def main():
  if '--help' not in sys.argv:
    settings = DisplayParams()
    while not (settings.get_int('ClusterHud') == 1 and
               (settings.get_bool('IsOnroad') or settings.get_int('ClusterHudDebug') >= 1)):
      time.sleep(.2)
  from openpilot.cereal import log
  params_module = types.ModuleType('openpilot.common.params')
  params_module.Params = DisplayParams
  sys.modules[params_module.__name__] = params_module
  messaging = types.ModuleType('openpilot.cereal.messaging')
  messaging.SubMaster = RemoteSubMaster
  from hud_navi import RemoteMediaSocket
  def sub_sock(service, **kwargs):
    if service != 'carrotNaviMedia':
      raise ValueError(f'unsupported display subscription: {service}')
    return RemoteMediaSocket()
  messaging.sub_sock = sub_sock
  messaging.drain_sock = lambda sock: sock.drain()
  def from_bytes(data, schema=log.Event):
    with schema.from_bytes(data) as reader:
      return reader.as_builder()
  messaging.log_from_bytes = from_bytes
  sys.modules[messaging.__name__] = messaging
  import cluster_live_camera
  from hud_camera import RemoteRoadCamera
  cluster_live_camera.LiveRoadCamera = RemoteRoadCamera
  import cluster_system_monitor
  from hud_stats import VehicleSystemStats, VehicleCpuOverlay
  cluster_system_monitor.SystemStatsSampler = VehicleSystemStats
  cluster_system_monitor.ClusterProcessCoreUsageSampler = VehicleCpuOverlay
  import cluster_navi_source
  display_metrics = {}
  class RemoteNaviSource(cluster_navi_source.NaviIpcMediaSource):
    def update(self, navi_live):
      dashboard = super().update(navi_live)
      frame = next((f for f in dashboard.media if f.key == 'render:map_main' and f.present), None)
      display_metrics['map'] = {'sequence': frame.sequence if frame else None,
                                'width': frame.width if frame else None,
                                'height': frame.height if frame else None,
                                'age_ms': dashboard.map_frame_age_ms,
                                'stalled': dashboard.map_stream_stalled}
      return dashboard
  cluster_navi_source.NaviIpcMediaSource = RemoteNaviSource
  vehicle_stats = VehicleSystemStats()
  import main as cluster
  if '--help' not in sys.argv:
    scan = cluster.find_supported_usb_product
    wait_for_usb_display(scan, cluster.product_id_for_hud_mode(1))
    # The common PC renderer falls back to an invisible window if the USB
    # panel is absent. A headless Jetson must wait/retry, including the race
    # where it disappears between the preflight scan and opening the device.
    cluster.find_supported_usb_product = lambda expected: require_usb_display(scan, expected)
  class Display(cluster.TuringUsbDisplay):
    next_status = 0.

    def send_h264_chunk(self, *args, **kwargs):
      result = super().send_h264_chunk(*args, **kwargs)
      now = time.monotonic()
      if now >= self.next_status:
        path = Path('/dev/shm/carrot-jetlink-hud-status.json')
        tmp = path.with_suffix('.tmp')
        stats = vehicle_stats.sample()
        tmp.write_text(json.dumps({'updated': now, **display_metrics,
                                  'vehicle_cpu': stats.cpu_core_percents,
                                  'vehicle_memory_percent': stats.memory_used_percent}))
        os.replace(tmp, path)
        self.next_status = now + 1
      return result
  cluster.TuringUsbDisplay = Display
  # Jetson has no video encoder block. Keep inference CPU/GPU headroom and
  # use the existing software encoder at the normal 10 FPS cluster rate.
  sys.argv = [__file__, '--input', 'live', '--output', 'usb', '--fps', '10',
              '--usb-codec', 'h264', '--usb-h264-backend', 'ffmpeg',
              '--usb-h264-ffmpeg-encoder', 'libx264', '--cluster-hud-mode', '1', *sys.argv[1:]]
  cluster.main()


if __name__ == '__main__':
  main()
