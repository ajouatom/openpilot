"""Update the registered B1 through the comma's existing BlueZ connection."""
import argparse
import fcntl
import asyncio
import time
from pathlib import Path

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.carrot.bluetooth.b1_monitor import ENROLLMENT, GATT, SERVICE, OTA_LOCK
from openpilot.selfdrive.carrot.bluetooth.bluez import Bluez, DEVICE, properties
from openpilot.selfdrive.carrot.bluetooth.model import address, read_json
from .ble_update import META, OTA, install


class ParkGuard:
  def __init__(self):
    self.sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState'])
    self.params = Params()

  def check(self):
    self.sm.update(0)
    if not self.params.get_bool('IsOnroad') and self.params.get_bool('IsOffroad'):
      return
    now = time.monotonic()
    sm = self.sm
    if not all(sm.valid[k] and sm.alive[k] and 0 <= now - sm.recv_time[k] < .5 for k in sm.services):
      raise RuntimeError('Cannot verify fresh vehicle state')
    cs, cc, ss = (sm[k] for k in ('carState', 'carControl', 'selfdriveState'))
    if not (cs.canValid and cs.standstill and str(cs.gearShifter) == 'park' and abs(cs.vEgo) < .01 and
            not (cc.enabled or cc.latActive or cc.longActive or ss.enabled or ss.active)):
      raise RuntimeError('Update requires Park, standstill and disengaged controls')


class Client:
  def __init__(self, bus, attrs):
    self.bus, self.attrs = bus, attrs

  async def request(self, uuid, method, signature, args):
    for _ in range(40):
      try:
        return await self.bus.call(self.attrs[uuid], GATT, method, signature, args, reply_timeout=6)
      except Exception as exc:
        retry_read = method == 'ReadValue' and 'ATT error: 0x0e' in str(exc)
        if not retry_read and 'InProgress' not in str(exc) and 'In Progress' not in str(exc):
          raise
        await asyncio.sleep(.025)
    raise TimeoutError('BLE adapter is busy')

  async def read_gatt_char(self, uuid):
    return bytes((await self.request(uuid, 'ReadValue', 'a{sv}', ({},)))[0])

  async def write_gatt_char(self, uuid, data, *, response):
    await self.request(uuid, 'WriteValue', 'aya{sv}', (data, {'type': ('s', 'request')}))


async def update(image, quiet_bus):
  registration = read_json(ENROLLMENT, {})
  mac = address(registration.get('address'))
  key = bytes.fromhex(registration['key'])
  guard = ParkGuard()
  for _ in range(15):
    guard.sm.update(100)
  guard.check()
  bus = Bluez()
  try:
    objects = await bus.objects()
    attrs = {}
    for path, interfaces in objects.items():
      char = properties(interfaces.get(GATT, {}))
      service = properties(objects.get(char.get('Service'), {}).get('org.bluez.GattService1', {}))
      device = properties(objects.get(service.get('Device'), {}).get(DEVICE, {}))
      if service.get('UUID') == SERVICE and device.get('Address') == mac and device.get('Connected'):
        attrs[char['UUID']] = path
    if not {META, OTA}.issubset(attrs):
      raise RuntimeError('The enrolled board must be connected with its OTA service resolved')
    with OTA_LOCK.open('ab') as lock:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      await asyncio.sleep(.75)  # Drain a pending monitor exchange and expire its control lease.
      await install(Client(bus, attrs), image, key, parked_check=guard.check, quiet_bus=quiet_bus)
  finally:
    await bus.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('image', type=Path)
  parser.add_argument('--quiet-bus', action='store_true', help='Use the original manifest for older firmware, with CAN silent')
  args = parser.parse_args()
  asyncio.run(update(args.image.read_bytes(), args.quiet_bus))


if __name__ == '__main__':
  main()
