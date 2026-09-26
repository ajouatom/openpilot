"""Remove only NVIDIA's fixed utmp timestamp delay; preserve boot services.

NVIDIA describes this sleep as a workaround for `last reboot -F` timestamps:
https://forums.developer.nvidia.com/t/why-systemd-update-utmp-service-need-sleep-2s/363767
The reboot history timestamp may be less accurate. This does not skip filesystem,
power-mode, driver, model-readiness or vehicle-join checks.
"""
import argparse
import os
from pathlib import Path
import subprocess

DIRECTORY = Path('/etc/systemd/system/systemd-update-utmp.service.d')
OVERRIDE = '[Service]\nExecStartPre=\n'


def validate_pre_commands(text):
  commands = [line.strip() for line in text.splitlines() if line.strip().startswith('ExecStartPre=')]
  if commands != ['ExecStartPre=/bin/sleep 2']:
    raise ValueError('Unexpected pre-start commands; inspect rather than overriding them')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--restore', action='store_true')
  args = parser.parse_args()
  if os.geteuid() != 0 or not Path('/etc/nv_tegra_release').is_file():
    raise RuntimeError('Root on a prepared Jetson is required')
  target = DIRECTORY/'zz-carrot-no-timestamp-delay.conf'
  if args.restore:
    if target.exists() and target.read_text() != OVERRIDE:
      raise RuntimeError('Override changed; refusing to remove it')
    target.unlink(missing_ok=True)
  elif not target.exists():
    text = subprocess.check_output(['systemctl', 'cat', 'systemd-update-utmp.service'], text=True)
    validate_pre_commands(text)
    target.write_text(OVERRIDE)
  elif target.read_text() != OVERRIDE:
    raise RuntimeError('Existing override has unexpected contents')
  subprocess.run(['systemctl', 'daemon-reload'], check=True)
  print('utmp timestamp delay restored' if args.restore else 'utmp timestamp delay disabled for subsequent boots')


if __name__ == '__main__':
  main()
