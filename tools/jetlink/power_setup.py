"""Select the installed NVIDIA Super profile before inference, once per boot.

Only NVIDIA's active board configuration supplies a mode ID. This does not
flash firmware, invent frequency limits, or reboot a running vehicle.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import time

STATUS = Path('/run/carrot-jetlink-power.json')


def profiles(text):
  text = '\n'.join(line.split('#', 1)[0] for line in text.splitlines())
  result = []
  for header in re.findall(r'<\s*POWER_MODEL\s+([^>]+)>', text):
    fields = dict(re.findall(r'(ID|NAME)\s*=\s*([^\s>]+)', header))
    if not fields.get('ID', '').isdigit() or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', fields.get('NAME', '')):
      raise ValueError('Invalid NVIDIA power profile')
    result.append({'id': int(fields['ID']), 'name': fields['NAME']})
  if len(result) > 32 or len({p['id'] for p in result}) != len(result):
    raise ValueError('Ambiguous NVIDIA power profiles')
  return result


def command(*args):
  # NVIDIA can prompt for reboot when a GPU context already exists. Decline
  # explicitly and report it; never let a service wait for terminal input.
  return subprocess.run(list(args), input='NO\n', capture_output=True, text=True, timeout=8)


def selected():
  result = command('/usr/sbin/nvpmodel', '-q')
  match = re.search(r'NV Power Mode:\s*([^\r\n]+)\s*\n\s*(\d+)\s*$', result.stdout)
  if result.returncode or not match:
    raise RuntimeError('Power mode query failed')
  return {'name': match[1].strip(), 'id': int(match[2])}


def gpu_clocks(root):
  result = []
  for device in sorted((root / 'sys/class/devfreq').glob('*')):
    if any(name in device.name for name in ('gpu', 'ga10b', 'gv11b')):
      result.append({'device': device.name, **{
        name: int((device / filename).read_text()) / 1e6
        for name, filename in (('clock_mhz', 'cur_freq'), ('max_mhz', 'max_freq'))}})
  return result


def apply(root=Path('/')):
  value = {'version': 1, 'updated': time.monotonic(), 'state': 'checking'}
  try:
    value['board'] = (root / 'proc/device-tree/model').read_text().replace('\x00', '').strip()[:120]
    value['l4t'] = (root / 'etc/nv_tegra_release').read_text().splitlines()[0][:160]
    value['compatible'] = (root / 'proc/device-tree/compatible').read_text().replace('\x00', ' ')[:240]
    if 'tegra234' not in value['compatible'] or 'Orin Nano' not in value['board']:
      return {**value, 'state': 'unsupported_board'}
    config = root / 'etc/nvpmodel.conf'
    value['config'] = config.resolve().name
    value['profiles'] = profiles(config.read_text())
    candidates = [p for p in value['profiles'] if p['name'] == 'MAXN_SUPER']
    value['before'] = selected()
    if len(candidates) != 1:
      if re.search(r'p3767-000[345](?:\s|$)', value['compatible']):
        # nvpower.sh deliberately relinks Super profiles to the regular profile
        # when the running device tree identifies a non-Super firmware variant.
        value['firmware_action'] = 'Install matching NVIDIA Orin Nano Super boot firmware'
        return {**value, 'state': 'super_firmware_required'}
      return {**value, 'state': 'super_profile_missing'}
    target, = candidates
    value['target'] = target
    if value['before'] != target:
      result = command('/usr/sbin/nvpmodel', '-m', str(target['id']))
      output = (result.stdout + result.stderr).lower()
      if 'reboot required' in output or 'reboot is required' in output:
        return {**value, 'state': 'reboot_required'}
      if result.returncode or 'nvpm error' in output:
        return {**value, 'state': 'mode_apply_failed', 'returncode': result.returncode}
    value['selected'] = selected()
    if value['selected'] != target:
      return {**value, 'state': 'mode_verify_failed'}
    result = command('/usr/bin/jetson_clocks')
    if result.returncode:
      return {**value, 'state': 'clock_apply_failed', 'returncode': result.returncode}
    value['gpu'] = gpu_clocks(root)
    # MAXN_SUPER on the supported Orin Nano raises the GPU ceiling to 1020MHz.
    # A mode label with the old 624.75MHz ceiling is not a successful setup.
    if not value['gpu'] or any(g['max_mhz'] < 1000 for g in value['gpu']):
      return {**value, 'state': 'firmware_clock_limit'}
    return {**value, 'state': 'ready'}
  except (OSError, ValueError, IndexError, RuntimeError, subprocess.SubprocessError) as exc:
    return {**value, 'state': 'setup_failed', 'error': type(exc).__name__}


def read_status(path=STATUS):
  try:
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) and value.get('version') == 1 else {}
  except (OSError, ValueError):
    return {}


def run_once(root=Path('/')):
  import fcntl
  status = root / STATUS.relative_to('/')
  boot_id = (root / 'proc/sys/kernel/random/boot_id').read_text().strip()
  with status.with_suffix('.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    previous = read_status(status)
    if previous.get('boot_id') == boot_id:
      return previous
    value = {**apply(root), 'boot_id': boot_id}
    temporary = status.with_suffix('.new')
    temporary.write_text(json.dumps(value))
    temporary.chmod(0o644)
    temporary.replace(status)
    print(json.dumps({'event': 'jetsonPowerSetup', **value}), flush=True)
    return value


def configure(root, source):
  """Image installation establishes ordering before any NVIDIA display use."""
  root, source = Path(root), Path(source)
  destination = root / 'usr/lib/carrot-jetlink-power'
  destination.mkdir(parents=True, exist_ok=True)
  (destination / 'power_setup.py').write_bytes((source / 'power_setup.py').read_bytes())
  unit = root / 'etc/systemd/system/carrot-jetlink-performance.service'
  unit.parent.mkdir(parents=True, exist_ok=True)
  unit.write_text('''[Unit]
Description=Select NVIDIA MAXN_SUPER and configure inference clocks
After=nvpmodel.service
Requires=nvpmodel.service
Before=carrot-jetlink.service carrot-jetlink-xorg.service carrot-jetlink-hud.service carrot-jetlink-update-apply.service
[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/lib/carrot-jetlink-power/power_setup.py
RemainAfterExit=yes
TimeoutStartSec=40
[Install]
WantedBy=multi-user.target
''')
  for name in ('carrot-jetlink', 'carrot-jetlink-xorg', 'carrot-jetlink-hud', 'carrot-jetlink-update-apply'):
    folder = unit.parent / (name + '.service.d')
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'power.conf').write_text('[Unit]\nRequires=carrot-jetlink-performance.service\nAfter=carrot-jetlink-performance.service\n')
  link = unit.parent / 'multi-user.target.wants' / unit.name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + unit.name)


if __name__ == '__main__':
  if os.geteuid() != 0:
    raise PermissionError('Power setup requires the root startup service')
  run_once()
