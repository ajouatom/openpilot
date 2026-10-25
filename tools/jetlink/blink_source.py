"""Low-priority source for explicit Bluetooth physical-indicator mappings."""
import json
from pathlib import Path
import select
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from blink_protocol import PACKAGE_CONFIG, encode, finite, fresh


def vehicle_ready(sm, now, fingerprint):
  """Manual indicators work while driving/engaged; unknown or stale CAN never does."""
  valid = all(sm.valid[k] and sm.alive[k] and
              0 <= now-sm.recv_time[k] < (.75 if k == 'pandaStates' else .35) for k in sm.services)
  c = sm['carState']
  return (fingerprint == 'HYUNDAI_IONIQ_5_PE' and valid and c.canValid and finite(c.vEgo) and str(c.gearShifter) != 'unknown' and
          len(sm['pandaStates']) > 0)


class Events:
  def __init__(self, started):
    self.started = started

  def read(self, journal, settings, status, learning, cancelled, now):
    if not fresh(status.get('time'), now):
      return []
    result = []
    for event in journal.get('events', [])[-64:]:
      if not isinstance(event, dict) or not fresh(event.get('time'), now) or event['time'] < self.started:
        continue
      mac, action = event.get('address'), event.get('action')
      device = settings.get('devices', {}).get(mac, {})
      if (device.get('enabled') is not True or mac not in status.get('grabbed', []) or
          action not in ('blinkerLeft', 'blinkerRight') or action not in device.get('mapping', {}).values() or
          event.get('repeat') or event['time'] <= cancelled.get(mac, 0) or
          (learning.get('address') == mac and learning.get('until', 0) > now)):
        continue
      result.append({'id': event['id'], 'time': event['time'], 'direction': 'left' if action=='blinkerLeft' else 'right'})
    return result[-4:]


def main(fd):
  from openpilot.cereal import messaging
  from openpilot.selfdrive.carrot.bluetooth.model import RUNTIME, config, read_json
  from openpilot.common.params import Params
  from openpilot.cereal import car
  with car.CarParams.from_bytes(Params().get('CarParams')) as cp:
    fingerprint = cp.carFingerprint
  if json.loads(PACKAGE_CONFIG.read_text()).get('fingerprint') != fingerprint:
    return
  sock = socket.socket(fileno=fd)
  sock.setblocking(False)
  sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState', 'pandaStates'])
  events = Events(time.monotonic())
  while True:
    now = time.monotonic()
    if select.select([sock], [], [], 0)[0] and not sock.recv(1):
      return
    sm.update(0)
    ok = vehicle_ready(sm, now, fingerprint)
    commands = events.read(read_json(RUNTIME/'blinker.json', {}), config(), read_json(RUNTIME/'status.json', {}),
                           read_json(RUNTIME/'learn.json', {}), read_json(RUNTIME/'cancelled.json', {}), now)
    try:
      sock.send(encode({'stamp': now, 'ok': bool(ok), 'events': commands if ok else []}))
    except BlockingIOError:
      pass  # Never buffer commands for delivery after congestion.
    time.sleep(max(0, now+.05-time.monotonic()))


if __name__ == '__main__':
  main(int(sys.argv[1]))
