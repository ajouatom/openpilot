"""One-time local enrollment; run against stopped live or recovery-mounted DATA."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from install_local_experiment import atomic_copy
from local_release import owned

FILES = ('finalize_sd_image.py', 'local_release.py', 'usb_release.py', 'boot_update.py', 'update_host.py')
SOURCE = Path(__file__).resolve().parent


def install(runtime, source=SOURCE):
  if os.geteuid() != 0:
    raise PermissionError('Enroll from the local root/recovery console')
  runtime, source = Path(runtime).resolve(), Path(source).resolve()
  owned(runtime, directory=True)
  updater = runtime / 'updater'
  owned(updater, directory=True)
  for marker in ('boot-update-required', 'protected-runtime.json'):
    owned(runtime / marker)
  if runtime == Path('/opt/carrot-jetlink').resolve():
    for unit in ('carrot-jetlink', 'carrot-jetlink-hud', 'carrot-jetlink-boot-gate', 'carrot-jetlink-update-stage'):
      if subprocess.run(['systemctl', 'is-active', '--quiet', unit]).returncode == 0:
        raise RuntimeError('Stop Jetson inference/update services or use recovery mode')
  official = updater / 'release-signing-public.pem'
  owned(official)
  if official.read_bytes() != (source / official.name).read_bytes():
    raise ValueError('Official trust root differs; refusing enrollment')
  from usb_release import SHA256_POLICY, SHA256_POLICY_VALUE
  policy = updater / SHA256_POLICY
  if policy.exists() or policy.is_symlink():
    owned(policy)
  for name in FILES:
    if not (source / name).is_file():
      raise ValueError('Incomplete receiver source')
    target = updater / name
    if target.exists() or target.is_symlink():
      owned(target)
  backup = updater / 'before-usb-experiments'
  backup.mkdir(mode=0o700, exist_ok=True)
  owned(backup, directory=True)
  for name in FILES:
    target = updater / name
    if target.exists():
      atomic_copy(target, backup / name)
    atomic_copy(source / name, target)
  # Publish the locally selected policy last, after all receiver files exist.
  # USB packets cannot enable it; official key bytes are never replaced.
  temporary = policy.with_suffix('.new')
  temporary.write_text(json.dumps(SHA256_POLICY_VALUE, sort_keys=True) + '\n')
  temporary.chmod(0o644)
  os.replace(temporary, policy)
  os.sync()
  print('USB SHA-256 receiver enabled; official key and current release preserved.')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--runtime', type=Path, required=True)
  parser.add_argument('--sha256', action='store_true', required=True,
                      help='Explicitly authorize physical USB installation using SHA-256/length, without author signatures')
  args = parser.parse_args()
  install(args.runtime)
