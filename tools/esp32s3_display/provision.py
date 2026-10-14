#!/usr/bin/env python3
"""Register over UART using C4's active Wi-Fi, or import the phone setup file."""
import argparse
import getpass
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from cluster_wifi_protocol import load_config, save_config


def current_wifi() -> tuple[str, str]:
  def nmcli(*args: str) -> str:
    result = subprocess.run(['nmcli', *args], capture_output=True, text=True, timeout=10)
    if result.returncode:
      raise RuntimeError('Cannot read the active Wi-Fi profile; use --ssid for manual registration')
    return result.stdout.rstrip('\n')
  connections = nmcli('-t', '-f', 'UUID,TYPE', 'connection', 'show', '--active').splitlines()
  profiles = [line.split(':')[0] for line in connections if line.endswith((':802-11-wireless', ':wifi'))]
  if len(profiles) != 1:
    raise RuntimeError('Exactly one active Wi-Fi connection is required; use --ssid otherwise')
  profile = profiles[0]
  ssid = nmcli('--escape', 'no', '-g', '802-11-wireless.ssid', 'connection', 'show', 'uuid', profile)
  password = nmcli('--show-secrets', '--escape', 'no', '-g', '802-11-wireless-security.psk', 'connection', 'show', 'uuid', profile)
  if not password:
    raise RuntimeError('Active Wi-Fi has no readable WPA password; use --ssid for manual registration')
  return ssid, password


def validate_wifi(ssid: str, password: str) -> None:
  if not 1 <= len(ssid.encode()) <= 32 or not 8 <= len(password.encode()) <= 63 or '\0' in ssid + password:
    raise ValueError('SSID must be 1..32 UTF-8 bytes and WPA2 password 8..63 bytes')


def uart_request(port, command: str, *, timeout: float = 8) -> dict:
  port.write((command + '\n').encode())
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    line = port.readline()
    if line.startswith(b'S3JSON '):
      try:
        reply = json.loads(line[7:])
      except ValueError:
        continue
      if reply.get('error'):
        raise RuntimeError('S3 rejected registration; check Wi-Fi input and storage')
      return reply
  raise TimeoutError('S3 registration firmware did not reply on the UART port')


def register(port_name: str, ssid: str, password: str) -> dict:
  import serial
  validate_wifi(ssid, password)
  port = serial.Serial()
  port.port, port.baudrate, port.timeout = port_name, 115200, .25
  port.dtr = port.rts = False
  port.open()
  try:
    port.reset_input_buffer()
    hello = uart_request(port, 'S3HELLO')
    if hello.get('version') != 1 or len(bytes.fromhex(hello.get('device', ''))) != 6:
      raise RuntimeError('The selected port is not an S3 display bridge')
    reply = uart_request(port, 'S3CONFIG ' + json.dumps({'ssid': ssid, 'password': password}, ensure_ascii=True))
    if reply.get('saved') is not True:
      raise RuntimeError('S3 did not confirm persistent storage')
    pairing = uart_request(port, 'S3PAIR')
    if pairing.get('device') != hello['device']:
      raise RuntimeError('S3 registration identity changed')
    return pairing
  finally:
    port.close()


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  source = parser.add_mutually_exclusive_group(required=True)
  source.add_argument('--port', help='UART USB port, e.g. /dev/ttyUSB0 or COM14')
  source.add_argument('--import-pairing', type=Path, help='Registration JSON downloaded from the phone setup page')
  parser.add_argument('--ssid', help='Manual Wi-Fi name; password is prompted without echo')
  parser.add_argument('--output', type=Path, help='Default: /data/carrot/cluster_wifi.json')
  args = parser.parse_args()
  if args.import_pairing:
    config = load_config(args.import_pairing)
    if config is None:
      parser.error('Registration file does not exist')
  else:
    ssid, password = (args.ssid, getpass.getpass('Wi-Fi password: ')) if args.ssid else current_wifi()
    config = register(args.port, ssid, password)
  save_config(config, args.output)
  print('S3 registered. Enable the 9.2-inch external HUD; C4 and S3 must use the same 2.4 GHz network.')


if __name__ == '__main__':
  try:
    main()
  except (RuntimeError, ValueError, OSError, TimeoutError) as exc:
    raise SystemExit(str(exc)) from None
