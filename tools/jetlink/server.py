"""Pinned upstream server with an optional, read-only Carrot HUD extension."""
import os
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hud_protocol import HUD_CAPABILITY, HUD_MESSAGE, MAX_HUD_BYTES, HUD_PACKET, HEADER
from jetlink import protocol as P
from jetlink.server.session import Session
from jetlink.server import main as server


class DisplayTelemetry:
  def __init__(self, original):
    self.original = original

  def read(self):
    connected = False
    try:
      status = json.loads(Path('/dev/shm/carrot-jetlink-hud-status.json').read_text())
      connected = 0 <= time.monotonic() - status['updated'] < 2
    except (OSError, ValueError, KeyError, TypeError):
      pass
    return {**self.original.read(), 'carrot_hud_connected': connected}


class CarrotSession(Session):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.telemetry = DisplayTelemetry(self.telemetry)

  def _send_json(self, msg_type, seq, obj, flags=0):
    if msg_type == P.Msg.HELLO_RESP:
      host = 'mac' if sys.platform == 'darwin' else (
        'jetson' if Path('/etc/nv_tegra_release').is_file() else 'unknown')
      obj = {**obj, HUD_CAPABILITY: sys.platform == 'linux', 'carrot_host': host}
    return super()._send_json(msg_type, seq, obj, flags)

  def handle(self, msg):
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


if __name__ == '__main__':
  server.Session = CarrotSession
  raise SystemExit(server.main())
