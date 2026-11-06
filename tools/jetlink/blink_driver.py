"""Prepare optional USB CAN access at startup, never in an inference session."""
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

ROOT = Path(__file__).parent
CANABLE_RULE = ''.join('SUBSYSTEM=="tty", ATTRS{idVendor}=="'+vid+'", ATTRS{idProduct}=="'+pid+'", GROUP="plugdev", MODE="0660"\n'
                       for vid, pid in (('16d0', '117e'), ('ad50', '60c4')))


def prepare_canable():
  # Protected Jetson images rebuild /etc on boot. Reinstall this narrow rule
  # before discovery so both cold-plug and later hot-plug use the service group.
  rule = Path('/etc/udev/rules.d/71-carrot-canable.rules')
  if not rule.exists() or rule.read_text() != CANABLE_RULE:
    with tempfile.TemporaryDirectory(prefix='carrot-canable-') as work:
      source = Path(work)/rule.name
      source.write_text(CANABLE_RULE)
      subprocess.run(['sudo', '-n', 'install', '-m', '0644', str(source), str(rule)], check=True, timeout=5)
    subprocess.run(['sudo', '-n', 'udevadm', 'control', '--reload-rules'], check=True, timeout=5)
    subprocess.run(['sudo', '-n', 'udevadm', 'trigger', '--subsystem-match=tty'], check=True, timeout=5)


def prepare():
  profile = ROOT/'blinker_profile.json'
  if not profile.is_file():
    return False
  config = json.loads(profile.read_text())
  if config.get('enabled') is not True or config.get('driver_kernel') != platform.release():
    return False
  prepare_canable()
  if Path('/sys/module/peak_usb').is_dir():
    return True
  headers = Path('/lib/modules')/platform.release()/'build'
  if not headers.is_dir():
    raise RuntimeError('PEAK driver requires matching installed kernel headers')
  source = ROOT/'peak_usb'
  manifest = json.loads((source/'sources.json').read_text())
  with tempfile.TemporaryDirectory(prefix='carrot-peak-') as work:
    for name, item in manifest.items():
      if Path(name).name != name:
        raise ValueError('Invalid driver source name')
      data = (source/name).read_bytes()
      if hashlib.sha256(data).hexdigest() != item['sha256']:
        raise ValueError('PEAK source hash mismatch')
      (Path(work)/name).write_bytes(data)
    subprocess.run(['nice','-n','10','make','-s','-C',str(headers),'M='+work,
                    'CONFIG_CAN_PEAK_USB=m','-j1','modules'], check=True, timeout=120,
                   stdout=subprocess.DEVNULL)
    module = Path(work)/'peak_usb.ko'
    vermagic = subprocess.check_output(['modinfo','-F','vermagic',str(module)], text=True).split()[0]
    if vermagic != platform.release():
      raise ValueError('PEAK module kernel mismatch')
    subprocess.run(['sudo','-n','modprobe','can_dev'], check=True, timeout=5)
    subprocess.run(['sudo','-n','insmod',str(module)], check=True, timeout=5)
  return Path('/sys/module/peak_usb').is_dir()


def prepare_optional():
  try:
    return prepare()
  except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
    print('Optional USB CAN preparation failed: '+str(exc), file=sys.stderr, flush=True)
    return False


if __name__ == '__main__':
  raise SystemExit(0 if prepare_optional() else 1)
