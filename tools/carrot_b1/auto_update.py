"""Install the B1 image bundled in this checkout when the enrolled board is parked."""
import asyncio
import hashlib
from pathlib import Path
import re
import time

from openpilot.selfdrive.carrot.bluetooth.b1_monitor import ENROLLMENT, META, OTA_LOCK, maintenance_active, parked
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json
from .ble_update import OTA, install

IMAGE = Path(__file__).with_name('CarrotB1.bin')
RECEIPT = ENROLLMENT.with_name('b1-update.json')
TRACE = 'db340a96-804c-4ead-af37-8a0e6af754a1'
TAG = rb'carrot-b1-release=([1-9][0-9]*)'


def release_id(image):
  # ESP32-S3 image header, first segment app descriptor and appended SHA-256.
  tags = re.findall(TAG, image)
  if (len(image) < 256 or image[0] != 0xe9 or image[12:14] != b'\x09\x00' or
      image[32:36] != bytes.fromhex('3254cdab') or hashlib.sha256(image[:-32]).digest() != image[-32:] or
      len(tags) != 1 or not any(image[176:208])):
    raise ValueError('invalid bundled B1 release image')
  return int(tags[0]), image[176:208].hex()


def decision(report, target, record, mac, now):
  release, elf = target
  tag = re.search(TAG, report)
  build = re.search(rb'\belf=([0-9a-f]{64})\b', report)
  if tag or build:
    if not tag or not build:
      raise ValueError('incomplete board build identity')
    version = int(tag[1])
    if version > release:
      return 'newer'
    if version == release:
      return 'current' if build[1].decode() == elf else 'version conflict'
  if record.get('mac') == mac and record.get('elf') == elf:
    if not tag and record.get('status') in ('uploaded', 'current'):
      return 'waiting for build identity'
    if -600 < now-record.get('time', 0) < 600:
      return 'retry later'
  return 'install'


async def check(monitor, image, target, mac, session):
  import fcntl
  from .comma_update import Client, ParkGuard
  guard = ParkGuard()
  for _ in range(12):
    guard.sm.update(0)
    await asyncio.sleep(.05)

  def safe():
    guard.check()
    if monitor.session != session or monitor.owned != mac:
      raise RuntimeError('board session changed')
    if read_json(ENROLLMENT, {}).get('address') != mac:
      raise RuntimeError('registration changed')

  safe()
  registration = read_json(ENROLLMENT, {})
  key = bytes.fromhex(registration['key'])
  attrs = dict(monitor.attrs or {})
  if not {META, OTA}.issubset(attrs):
    raise ValueError('OTA service unavailable')
  client = Client(monitor.client, attrs)
  with OTA_LOCK.open('ab') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    await asyncio.sleep(.75)
    safe()
    report = await client.read_gatt_char(TRACE) if TRACE in attrs else b''
    previous = read_json(RECEIPT, {})
    timestamp = time.time()  # noqa: TID251 -- retry receipts survive comma reboots
    action = decision(report, target, previous, mac, timestamp)
    def record(state):
      atomic_json(RECEIPT, {'mac':mac, 'elf':target[1], 'release':target[0], 'time':timestamp, 'status':state})
    if action == 'current':
      if (previous.get('mac'), previous.get('elf'), previous.get('status')) != (mac, target[1], 'current'):
        record('current')
    if action != 'install':
      return action
    # Original boards without TRACE only understand the quiet-bus manifest.
    # A rejection never triggers an immediate second transfer.
    record('attempt')
    monitor.firmware_state = 'updating'
    await install(client, image, key, parked_check=safe, quiet_bus=TRACE not in attrs)
    record('uploaded')
    return 'waiting for reboot'


async def run(monitor):
  image = target = checked = None
  while True:
    await asyncio.sleep(5)
    session = (monitor.owned, monitor.session)
    if (not all(session) or session == checked or time.monotonic()-monitor.proven_at >= 1 or
        not parked() or maintenance_active() or monitor.state.split(' · ', 1)[0] not in ('ready','missing')):
      continue
    try:
      if image is None:
        candidate = await asyncio.to_thread(IMAGE.read_bytes)
        target = release_id(candidate)
        image = candidate
      result = await check(monitor, image, target, *session)
      monitor.firmware_state = result
      if result in ('current','newer','version conflict'):
        checked = session
      else:
        await asyncio.sleep(25)
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
      monitor.firmware_state = str(exc)[:100]
      await asyncio.sleep(25)
    except Exception as exc:  # BlueZ can disappear at any point during a reboot.
      monitor.firmware_state = type(exc).__name__
      await asyncio.sleep(25)
