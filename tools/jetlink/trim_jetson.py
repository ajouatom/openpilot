"""Preview/apply removal of desktop applications on a dedicated Jetlink appliance."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

PACKAGES = {'snapd', 'docker.io', 'docker-ce', 'docker-ce-cli', 'docker-buildx-plugin',
            'docker-compose-plugin', 'containerd', 'containerd.io', 'runc', 'cups',
            'cups-daemon', 'cups-browsed', 'lpr', 'lprng', 'bluez', 'modemmanager',
            'gdm3', 'ubuntu-desktop', 'ubuntu-desktop-minimal', 'gnome-shell', 'gnome-session-bin',
            'nomachine', 'gnome-software', 'gnome-software-common', 'gnome-control-center',
            'gnome-control-center-data', 'gnome-initial-setup', 'gnome-calendar', 'gnome-contacts',
            'gnome-maps', 'gnome-weather', 'gnome-todo', 'gnome-todo-common', 'gnome-mahjongg',
            'gnome-mines', 'gnome-sudoku', 'aisleriot', 'deja-dup', 'simple-scan', 'seahorse',
            'cheese', 'cheese-common', 'nautilus', 'nautilus-data', 'yelp', 'eog', 'evince'}
PREFIXES = ('libreoffice-', 'thunderbird', 'rhythmbox', 'shotwell', 'transmission-',
            'remmina', 'totem', 'printer-driver-')
DEPENDENTS = {'gnome-bluetooth', 'gnome-shell-extension-appindicator', 'gnome-shell-extension-desktop-icons-ng',
              'gnome-shell-extension-ubuntu-dock', 'gnome-software-plugin-snap', 'gnome-startup-applications',
              'nautilus-share', 'ubuntu-session', 'gnome-software-plugin-flatpak',
              'gnome-user-docs', 'ubuntu-docs', 'ubuntu-settings', 'libcheese-gtk25', 'libcheese8', 'python3-uno'}
RUNTIME = {'openssh-server', 'network-manager', 'wpasupplicant', 'xserver-xorg-core', 'xauth',
           'ffmpeg', 'libusb-1.0-0', 'python3', 'python3-venv'}
SNAP_ORDER = ('chromium', 'cups', 'gnome-46-2404', 'mesa-2404', 'gtk-common-themes',
              'bare', 'core24', 'core22', 'snapd')


def installed():
  raw = subprocess.check_output(['dpkg-query', '-W', '-f=${binary:Package}\t${db:Status-Status}\t${Version}\n'], text=True)
  return {name.split(':')[0]: version for name, status, version in (line.split('\t') for line in raw.splitlines())
          if status in ('installed', 'half-configured', 'unpacked', 'half-installed')}


def main():
  os.environ['LC_ALL'] = 'C'
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--apply', action='store_true', help='Remove the reviewed dedicated-appliance apps; never autoremove dependencies')
  args = p.parse_args()
  if not Path('/etc/nv_tegra_release').is_file():
    raise RuntimeError('This profile is for Jetson only')
  current = installed()
  selected = sorted(name for name in current if name in PACKAGES or name.startswith(PREFIXES))
  command = ['apt-get', 'purge', '--no-auto-remove', '--no-install-recommends', *selected]
  plan = subprocess.check_output([command[0], '-s', *command[1:]], text=True)
  removed = set(re.findall(r'^(?:Remv|Purg) (\S+)', plan, re.MULTILINE))
  unexpected = removed - set(selected) - DEPENDENTS
  if unexpected:
    raise RuntimeError(f'Inspect unexpected removals before proceeding: {sorted(unexpected)}')
  if set(selected) - removed:
    raise RuntimeError('apt did not produce the expected removal plan')
  snap_names = set()
  if 'snapd' in current:
    snap_names = {line.split()[0] for line in subprocess.check_output(['snap', 'list'], text=True).splitlines()[1:]}
    if snap_names - set(SNAP_ORDER):
      raise RuntimeError(f'Inspect other installed Snap applications first: {sorted(snap_names - set(SNAP_ORDER))}')
  print(json.dumps({'selected': selected, 'actual_removals': sorted(removed),
                    'snap_removals': sorted(snap_names)}, indent=2), flush=True)
  if not args.apply or not selected:
    return
  if os.geteuid() != 0:
    raise RuntimeError('Root required for --apply')
  if 'docker-ce' in current or 'docker.io' in current:
    containers = subprocess.check_output(['docker', 'ps', '-aq'], text=True).strip()
    if containers:
      raise RuntimeError('Docker containers exist; this removal profile is not applicable')
  if not Path('/etc/systemd/system/carrot-jetlink-xorg.service').exists():
    raise RuntimeError('Install and validate the independent HUD X server first')
  evidence = Path('/var/lib/carrot-jetlink/trim-before.json')
  evidence.parent.mkdir(parents=True, exist_ok=True)
  if not evidence.exists():
    evidence.write_text(json.dumps({'installed': current, 'plan': plan}, indent=2))
  keep = sorted(name for name in current if name in RUNTIME or name.startswith(('nvidia-', 'cuda-', 'libnvinfer', 'libcudnn', 'tensorrt')))
  subprocess.run(['apt-mark', 'manual', *keep], check=True)
  if 'snapd' in current:
    # Retain snap's automatic removal snapshots; do not erase the user's app data.
    for name in SNAP_ORDER:
      if name in snap_names:
        subprocess.run(['snap', 'remove', name], check=True)
    snapshots = Path('/var/lib/snapd/snapshots')
    if snapshots.exists():
      shutil.copytree(snapshots, evidence.parent/'snap-snapshots', dirs_exist_ok=True)
  env = {**os.environ, 'DEBIAN_FRONTEND': 'noninteractive'}
  subprocess.run([*command[:1], '-y', *command[1:]], env=env, check=True)
  subprocess.run(['systemctl', 'daemon-reload'], check=True)
  print('Dedicated-appliance application cleanup complete; runtime dependencies retained', flush=True)


if __name__ == '__main__':
  main()
