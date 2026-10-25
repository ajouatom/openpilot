"""Low-priority source for explicit Bluetooth physical-indicator mappings."""
import json
from pathlib import Path
import select
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from blink_protocol import PACKAGE_CONFIG, LIMIT, encode, finite, fresh
from blink_policy import Policy, AutomaticSignals


def vehicle_ready(sm, now, fingerprint):
  """Manual indicators work while driving/engaged; unknown or stale CAN never does."""
  valid = all(sm.valid[k] and sm.alive[k] and
              0 <= now-sm.recv_time[k] < (.75 if k == 'pandaStates' else .35) for k in ('carState', 'carControl', 'selfdriveState', 'pandaStates'))
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
      result.append({'id': event['id'], 'time': event['time'], 'direction': 'left' if action=='blinkerLeft' else 'right', 'device':mac})
    return result[-4:]


def main(fd):
  from openpilot.cereal import messaging
  from openpilot.selfdrive.carrot.bluetooth.model import RUNTIME, config, read_json, atomic_json
  from openpilot.selfdrive.carrot.bluetooth.physical import HOLD_PATH
  from openpilot.common.params import Params
  from openpilot.cereal import car
  with car.CarParams.from_bytes(Params().get('CarParams')) as cp:
    fingerprint = cp.carFingerprint
  if json.loads(PACKAGE_CONFIG.read_text()).get('fingerprint') != fingerprint:
    return
  sock = socket.socket(fileno=fd)
  sock.setblocking(False)
  sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState', 'pandaStates', 'modelV2', 'carrotMan'])
  events = Events(time.monotonic())
  policy, params = Policy(), Params()
  automatic = AutomaticSignals()
  feedback, activated_id = {}, None
  release_until = 0.
  while True:
    now = time.monotonic()
    for _ in range(8):
      if not select.select([sock], [], [], 0)[0]:
        break
      raw = sock.recv(LIMIT+1)
      if not raw:
        return
      try:
        value = json.loads(raw) if len(raw) <= LIMIT else None
        if isinstance(value, dict):
          feedback = value
      except ValueError:
        feedback = {}
    sm.update(0)
    now = time.monotonic()  # SubMaster assigns receive timestamps during update.
    ok = vehicle_ready(sm, now, fingerprint)
    status = read_json(RUNTIME/'status.json', {})
    settings, learning, cancelled = config(), read_json(RUNTIME/'learn.json', {}), read_json(RUNTIME/'cancelled.json', {})
    commands = events.read(read_json(RUNTIME/'blinker.json', {}), settings, status, learning, cancelled, now)
    remote = False
    if policy.hold and policy.hold['source'] == 'bluetooth' and fresh(status.get('time'), now):
      mac = policy.hold.get('device')
      device = settings.get('devices', {}).get(mac, {})
      action = 'blinkerLeft' if policy.hold['direction'] == 'left' else 'blinkerRight'
      remote = (mac in status.get('grabbed', []) and device.get('enabled') is True and
                action in device.get('mapping', {}).values() and policy.hold['time'] > cancelled.get(mac, 0) and
                not (learning.get('address') == mac and learning.get('until', 0) > now))
    cs = sm['carState']
    mode = params.get_int('AutoBlinkerControl')
    mode = mode if mode in (0, 1, 2) else 0
    auto_fresh = all(k in sm.services and sm.valid[k] and sm.alive[k] and fresh(sm.recv_time[k], now, .35)
                     for k in ('modelV2', 'carrotMan'))
    auto_valid = auto_fresh and ok and sm['carControl'].latActive and str(cs.gearShifter) == 'drive'
    auto = None
    if auto_fresh:
      meta, ss = sm['modelV2'].meta, sm['selfdriveState']
      auto = automatic.update(now, mode, auto_valid,
                              ss.atcTurnStartMonoTime / 1e9, str(ss.atcTurnDirection), sm['carrotMan'].atcType,
                              str(meta.desire), str(meta.autoTurnSignal) if meta.autoTurnSignalLaneChange else None)
    else:
      automatic.update(now, mode, False, 0., 'none', '', 'none')
    commands, hold = policy.update(now, mode, ok, commands, cs.vEgo, cs.steeringAngleDeg,
                                  str(cs.gearShifter), remote, auto, auto_valid)
    hold = policy.acknowledged(now, activated_id, fresh(feedback.get('stamp'), now, .35) and feedback.get('state') == 'driver_override')
    acknowledged = (hold and fresh(feedback.get('stamp'), now, .35) and feedback.get('accepted_id') == hold['id'] and
                    feedback.get('state') in ('ready', 'waiting', 'waiting_previous_signal'))
    if not acknowledged:
      if activated_id is not None:
        release_until = now+.8
      activated_id = None
    elif ((hold['direction'] == 'left' and cs.leftBlinker and not cs.rightBlinker) or
          (hold['direction'] == 'right' and cs.rightBlinker and not cs.leftBlinker)):
      activated_id = hold['id']
    if feedback.get('state') == 'driver_override':
      release_until = 0.
    if mode:
      atomic_json(HOLD_PATH, {'stamp':now, 'mode':mode, 'fingerprint':fingerprint,
                             'direction':hold['direction'] if hold and activated_id == hold['id'] else None,
                             'override':activated_id is not None or now < release_until or (policy.suppressed and release_until > 0)})
    devices = [mac for mac in status.get('grabbed', []) if fresh(status.get('time'), now) and
               settings.get('devices', {}).get(mac, {}).get('enabled') is True and
               not (learning.get('address') == mac and learning.get('until', 0) > now)]
    try:
      sock.send(encode({'stamp': now, 'ok': bool(ok), 'events': commands, 'hold': hold, 'devices':devices}))
    except BlockingIOError:
      pass  # Never buffer commands for delivery after congestion.
    time.sleep(max(0, now+.05-time.monotonic()))


if __name__ == '__main__':
  main(int(sys.argv[1]))
