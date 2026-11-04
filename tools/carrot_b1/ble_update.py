"""Update an enrolled S3 from a BLE-capable PC. Keep the CAN harness disconnected."""
import argparse
import asyncio
from getpass import getpass
import hashlib
import hmac
from pathlib import Path
import struct

META = 'db340a92-804c-4ead-af37-8a0e6af754a1'
OTA = 'db340a94-804c-4ead-af37-8a0e6af754a1'


async def install(client, image, key, *, parked_check=None, quiet_bus=False, transfer_check=None):
  if parked_check is not None and transfer_check is not None:
    raise ValueError("Specify only one transfer guard")
  if transfer_check is None:
    transfer_check = parked_check
  if not 256 <= len(image) <= 0x300000 or image[0] != 0xe9 or len(key) != 32:
    raise ValueError('Expected an ESP32-S3 app image and a 32-byte board key')
  nonce = struct.unpack_from('<I', bytes(await client.read_gatt_char(META)))[0]
  if transfer_check:
    transfer_check()
  manifest = struct.pack('<BII', 4 if transfer_check and not quiet_bus else 1, nonce, len(image)) + hashlib.sha256(image).digest()

  async def transfer(packet, state, offset):
    if transfer_check:
      transfer_check()
    await client.write_gatt_char(OTA, packet, response=True)
    for _ in range(80):
      raw = bytes(await client.read_gatt_char(OTA))
      if len(raw) == 5:
        actual, written = struct.unpack('<BI', raw)
        if actual == 3:
          raise ValueError('Update rejected: check board key, image and quiet CAN bus')
        if (actual, written) == (state, offset):
          return
      await asyncio.sleep(.015)
    raise TimeoutError('S3 did not acknowledge the flash offset')

  await transfer(manifest + hmac.digest(key, manifest, 'sha256'), 1, 0)
  last_percent = 0
  for offset in range(0, len(image), 480):
    chunk = image[offset:offset+480]
    await transfer(struct.pack('<BI', 2, offset) + chunk, 1, offset+len(chunk))
    percent = (offset+len(chunk))*100//len(image)
    if percent >= last_percent+10:
      print(f'{percent}% written', flush=True)
      last_percent = percent
  await transfer(b'\x03', 2, len(image))


async def main():
  from bleak import BleakClient, BleakScanner
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('address')
  parser.add_argument('image', type=Path)
  args = parser.parse_args()
  key = bytes.fromhex(getpass('USB enrollment key (64 hex digits): ').strip())
  device = await BleakScanner.find_device_by_address(args.address, timeout=15)
  if not device:
    raise RuntimeError('S3 not found')
  async with BleakClient(device) as client:
    await install(client, args.image.read_bytes(), key)
  print('Image verified on the S3; restarting into the updated slot.')


if __name__ == '__main__':
  asyncio.run(main())
