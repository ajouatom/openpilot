"""Exclusive evdev reader. No commands are accepted from HTTP or raw sockets."""
import fcntl
import os
from pathlib import Path
import select
import struct
import subprocess
import time
import uuid

from openpilot.cereal import messaging
from openpilot.selfdrive.carrot.bluetooth.model import RUNTIME, Decoder, address, atomic_json, config, read_json

EVENT = struct.Struct('@llHHi')
EVIOCGRAB = 0x40044590


def devices():
  result = {}
  for node in Path('/sys/class/input').glob('event*'):
    try:
      if (node / 'device/id/bustype').read_text().strip() != '0005':
        continue
      mac = address((node / 'device/uniq').read_text().strip())
      result[str(Path('/dev/input') / node.name)] = (mac, (node / 'device/name').read_text().strip())
    except (OSError, ValueError):
      continue
  return result


def open_input(path):
  try:
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
  except PermissionError:
    # Only a kernel-enumerated Bluetooth event node reaches this function.
    node = Path(path)
    if node.parent != Path('/dev/input') or not node.name.startswith('event') or not node.is_char_device():
      raise
    subprocess.run(['sudo', '-n', 'chgrp', 'gpio', str(node)], check=True, timeout=3, capture_output=True)
    subprocess.run(['sudo', '-n', 'chmod', 'g+rw', str(node)], check=True, timeout=3, capture_output=True)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
  try:
    fcntl.ioctl(fd, EVIOCGRAB, 1)
    fcntl.ioctl(fd, 0x400445A0, struct.pack('i', 1))  # EVIOCSCLOCKID: CLOCK_MONOTONIC
    # Do not execute key/gesture fragments buffered before ownership began.
    while True:
      try:
        if not os.read(fd, EVENT.size * 64):
          break
      except BlockingIOError:
        break
    return fd
  except BaseException:
    os.close(fd)
    raise


def main():
  RUNTIME.mkdir(parents=True, exist_ok=True)
  lock = (RUNTIME / 'reader.lock').open('w')
  fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  sm = messaging.SubMaster(['carState', 'deviceState', 'selfdriveState'])
  opened = {}
  settings = config()
  last_reload = last_status = 0.0
  last_event = None
  errors = {}
  last_fire = {}
  learning = {}
  session = uuid.uuid4().hex
  sequence = 0
  try:
    while True:
      now = time.monotonic()
      sm.update(0)
      started = sm.alive['deviceState'] and sm['deviceState'].started
      car_ok = sm.alive['carState'] and sm.valid['carState'] and sm['carState'].canValid
      stationary = (sm.alive['deviceState'] and not sm['deviceState'].started) or (
        car_ok and abs(sm['carState'].vEgo) < 0.1 and sm.alive['selfdriveState'] and not sm['selfdriveState'].enabled)
      if now - last_reload >= 0.25:
        last_reload = now
        updated = config()
        learning = read_json(RUNTIME / 'learn.json', {}) or {}
        if not isinstance(learning, dict) or not isinstance(learning.get('until', 0), (int, float)) or learning.get('until', 0) < now:
          learning = {}
        available = devices()
        for path in list(opened):
          fd, mac, decoder = opened[path]
          old = settings['devices'].get(mac)
          new = updated['devices'].get(mac)
          wanted = new and (new['enabled'] or learning.get('address') == mac)
          if path not in available or not wanted or old != new:
            os.close(fd)
            del opened[path]
        settings = updated
        errors = {}
        for path, (mac, _name) in available.items():
          device = settings['devices'].get(mac)
          if path in opened or not device or not (device['enabled'] or learning.get('address') == mac):
            continue
          try:
            opened[path] = (open_input(path), mac, Decoder(device['profile']))
          except (OSError, subprocess.SubprocessError) as exc:
            errors[mac] = str(exc)
      ready, _, _ = select.select([entry[0] for entry in opened.values()], [], [], 0.01)
      for path, (fd, mac, decoder) in list(opened.items()):
        if fd not in ready:
          continue
        try:
          data = os.read(fd, EVENT.size * 128)
          if not data or len(data) % EVENT.size:
            raise OSError('HID device disconnected or incomplete event')
        except BlockingIOError:
          continue
        except OSError:
          os.close(fd)
          del opened[path]
          continue
        now = time.monotonic()
        for _sec, _usec, kind, code, value in EVENT.iter_unpack(data):
          for token in decoder.feed(kind, code, value, now):
            device = settings['devices'][mac]
            action = device['mapping'].get(token, 'none')
            testing = learning.get('address') == mac
            fresh = 0 <= now - (_sec + _usec / 1e6) < 0.4
            reason = 'test' if testing else 'inactive'
            emitted = False
            # Key repeats and queued events never become acceleration repeats.
            if fresh and not testing and device['enabled'] and started and car_ok and now - last_fire.get(mac, 0) >= 0.18:
              if action != 'none':
                sequence += 1
                channel = 'lane' if action in ('laneLeft', 'laneRight') else 'cruise'
                atomic_json(RUNTIME / f'{channel}.json', {'id': f'{session}:{sequence}', 'time': now,
                                                        'action': action, 'address': mac})
                last_fire[mac] = now
                emitted, reason = True, 'sent'
            last_event = {'id': uuid.uuid4().hex, 'time': now, 'address': mac, 'button': token,
                          'action': action, 'emitted': emitted, 'reason': reason}
      if now - last_status >= 0.2:
        last_status = now
        atomic_json(RUNTIME / 'status.json', {'time': now, 'stationary': bool(stationary), 'started': bool(started),
                    'grabbed': sorted({entry[1] for entry in opened.values()}), 'errors': errors,
                    'last_event': last_event, 'learning': learning})
  finally:
    for fd, _, _ in opened.values():
      os.close(fd)
    atomic_json(RUNTIME / 'status.json', {'time': time.monotonic(), 'stationary': False, 'grabbed': [], 'stopped': True})


if __name__ == '__main__':
  main()
