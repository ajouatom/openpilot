"""Small, allowlisted boot diagnostics. Never include profiles, keys or journals."""
import ipaddress
import json
from pathlib import Path
import subprocess
import time

STAGE = Path('/run/carrot-boot-stage.json')
UNITS = ('carrot-protected-storage', 'carrot-image-setup', 'carrot-jetlink-usbc-host',
         'nv', 'nvpmodel', 'carrot-jetlink-performance', 'NetworkManager', 'ssh',
         'carrot-jetlink-wifi', 'carrot-jetlink', 'carrot-jetlink-xorg', 'carrot-jetlink-hud')


def read_json(path):
  try:
    with Path(path).open() as stream:
      value = json.loads(stream.read(32769))
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def stage(name, error=None):
  value = {'stage': name, 'updated': time.monotonic()}
  if error:
    value['error'] = type(error).__name__
  temporary = STAGE.with_suffix('.tmp')
  temporary.write_text(json.dumps(value))
  temporary.replace(STAGE)


def collect():
  units = {}
  try:
    result = subprocess.run(['systemctl', 'show', '--property=Id,ActiveState,SubState,Result',
                             *[name + '.service' for name in UNITS]],
                            capture_output=True, text=True, timeout=3)
    for section in result.stdout.strip().split('\n\n'):
      fields = dict(line.split('=', 1) for line in section.splitlines() if '=' in line)
      name = fields.get('Id', '').removesuffix('.service')
      if name in UNITS:
        units[name] = {key: fields.get(key, 'unknown') for key in ('ActiveState', 'SubState', 'Result')}
  except (OSError, subprocess.SubprocessError):
    pass
  addresses = []
  try:
    result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'scope', 'global'],
                            capture_output=True, text=True, timeout=2, check=True)
    for interface in json.loads(result.stdout):
      for entry in interface.get('addr_info', []):
        address = ipaddress.ip_address(entry.get('local', ''))
        if not address.is_link_local and not address.is_loopback:
          addresses.append(str(address))
  except (OSError, ValueError, subprocess.SubprocessError):
    pass
  network = read_json('/dev/shm/carrot-jetlink-network-status.json')
  if not 0 <= time.monotonic() - network.get('updated', -100) < 6:
    network = {}
  temperatures = []
  for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
    try:
      temperature = int(path.read_text()) / 1000
      if -20 <= temperature <= 150:
        temperatures.append(temperature)
    except (OSError, ValueError, TypeError):
      pass
  return {'stage': read_json(STAGE), 'storage': read_json('/run/carrot-storage.json'),
          'units': units, 'addresses': addresses[:4], 'wifi': network.get('state', 'waiting'),
          'uptime': int(time.monotonic()), 'temp_c': max(temperatures, default=None)}


def persist_result(directory, value):
  """Called once at the end of storage boot, on the already guarded setup mount."""
  from persistent_state import durable_write
  # Restrict persisted content even if a caller accidentally supplies raw logs.
  record = {'format': 1, 'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'stage': str(value.get('stage', 'unknown'))[:48],
            'error': str(value.get('error', ''))[:80]}
  durable_write(Path(directory) / 'BOOT-STATUS.json', json.dumps(record).encode(), 0o600)
