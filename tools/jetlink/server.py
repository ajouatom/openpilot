"""Pinned upstream server with an optional, read-only Carrot HUD extension."""
import os
import json
from pathlib import Path
import sys
import time

if __name__ == '__main__':
  if Path('/etc/nv_tegra_release').is_file():
    from install_updates import migrate_running_release
    migrate_running_release()
  from boot_update import wait_for_runtime
  wait_for_runtime()

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hud_protocol import HUD_CAPABILITY, HUD_MESSAGE, MAX_HUD_BYTES, HUD_PACKET, HEADER
from hud_navi import CAPABILITY as NAVI_CAPABILITY, MESSAGE as NAVI_MESSAGE, HostForwarder
from hud_navi import PUMP_CAPABILITY
from host_reader import ReadAheadTransport
from host_health import health_worker
from wifi_protocol import CAPABILITY as WIFI_CAPABILITY, MESSAGE as WIFI_MESSAGE, receive as receive_wifi
from jetlink import protocol as P
from jetlink.server.session import Session
from jetlink.server import main as server


class DisplayTelemetry:
  def __init__(self, original):
    self.original = original
    self.health = health_worker() if Path('/etc/nv_tegra_release').is_file() else None

  def read(self):
    connected = False
    try:
      status = json.loads(Path('/dev/shm/carrot-jetlink-hud-status.json').read_text())
      connected = 0 <= time.monotonic() - status['updated'] < 2
    except (OSError, ValueError, KeyError, TypeError):
      pass
    return {**self.original.read(), 'carrot_hud_connected': connected,
            **({'carrot_health': self.health.read()} if self.health else {})}


class CarrotSession(Session):
  def _send(self, msg_type, seq, parts=(), flags=0):
    if msg_type == P.Msg.INFER_RESP and self.telemetry.health is not None:
      status = P.unpack_infer_resp(parts[0])[1]
      if status != P.Status.OK:
        self.telemetry.health.record_error('Inference', f'result status {status}')
    return super()._send(msg_type, seq, parts, flags)

  def _error(self, seq, error, detail=''):
    if self.telemetry.health is not None:
      self.telemetry.health.record_error(error, detail)
    return super()._error(seq, error, detail)

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    if sys.platform == 'linux':
      self.t = ReadAheadTransport(self.t)
    self.telemetry = DisplayTelemetry(self.telemetry)
    self.navi = HostForwarder() if sys.platform == 'linux' else None

  def _send_json(self, msg_type, seq, obj, flags=0):
    if msg_type == P.Msg.ENGINE_RESP and obj.get('state') == 'failed' and self.telemetry.health is not None:
      self.telemetry.health.record_error('Engine', obj.get('detail', 'failed'))
    if msg_type == P.Msg.HELLO_RESP:
      host = 'mac' if sys.platform == 'darwin' else (
        'jetson' if Path('/etc/nv_tegra_release').is_file() else 'unknown')
      obj = {**obj, HUD_CAPABILITY: sys.platform == 'linux', NAVI_CAPABILITY: sys.platform == 'linux',
             PUMP_CAPABILITY: sys.platform == 'linux', WIFI_CAPABILITY: host == 'jetson', 'carrot_host': host}
      if host == 'jetson':
        obj['carrot_source_commit'] = (ROOT / 'SOURCE_COMMIT').read_text().strip()
        obj['carrot_boot_update_installed'] = Path('/opt/carrot-jetlink/boot-update-required').is_file()
    return super()._send_json(msg_type, seq, obj, flags)

  def handle(self, msg):
    if msg.msg_type == WIFI_MESSAGE:
      if msg.seq > self.last_seq:
        self.last_seq = msg.seq
        receive_wifi(msg.payload)
      return
    if msg.msg_type == NAVI_MESSAGE:
      if msg.seq > self.last_seq:
        self.last_seq = msg.seq
        if self.navi is not None:
          self.navi.send(msg.payload)
      return
    if msg.msg_type != HUD_MESSAGE:
      return super().handle(msg)
    if msg.seq <= self.last_seq:
      return
    self.last_seq = msg.seq
    if not 0 < len(msg.payload) <= MAX_HUD_BYTES:
      return  # Display data must never change or invalidate a model result.
    temporary = HUD_PACKET.with_suffix('.tmp')
    try:
      with temporary.open('wb') as f:
        f.write(HEADER.pack(time.monotonic()))
        f.write(msg.payload)
      os.replace(temporary, HUD_PACKET)
    except OSError:
      pass  # An unavailable display does not stop inference.

  def close(self):
    if self.navi is not None:
      self.navi.close()
    if isinstance(self.t, ReadAheadTransport):
      self.t.close()
    super().close()


if __name__ == '__main__':
  if Path('/etc/nv_tegra_release').is_file():
    health_worker()
  server.Session = CarrotSession
  raise SystemExit(server.main())
