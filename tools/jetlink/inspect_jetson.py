"""Read-only Jetson provisioning report; no installs, restarts or GPU execution."""
import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SERVICES = ('carrot-jetlink', 'carrot-jetlink-hud', 'carrot-jetlink-xorg', 'carrot-jetlink-performance')


def read(path):
  try:
    return path.read_text().strip().replace('\x00', '')
  except OSError:
    return ''


def command(*args):
  try:
    result = subprocess.run(args, capture_output=True, text=True, timeout=10)
    return {'returncode': result.returncode, 'output': result.stdout.strip()}
  except (OSError, subprocess.TimeoutExpired) as exc:
    return {'returncode': -1, 'output': str(exc)}


def inspect(runtime, verify_model=False):
  errors = []
  spec = json.loads((ROOT / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
  release = read(Path('/etc/nv_tegra_release'))
  board = read(Path('/proc/device-tree/model'))
  if platform.system() != 'Linux' or platform.machine() != 'aarch64' or not release:
    errors.append('Run on the target Jetson with its JetPack system Python or runtime venv.')
  commit = read(runtime / 'carrot/SOURCE_COMMIT')
  if not re.fullmatch('[0-9a-f]{40}', commit):
    errors.append('Missing or invalid carrot/SOURCE_COMMIT; use the committed host bundle.')
  packages = {}
  for package in ('numpy', 'onnx', 'cuda-python', 'libusb1', 'tensorrt'):
    try:
      packages[package] = importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
      packages[package] = None
      if package != 'tensorrt':
        errors.append(f'Required inference package is missing from this Python: {package}')
  if importlib.util.find_spec('tensorrt') is None:
    errors.append('TensorRT is unavailable in this Python; use JetPack and a system-site-packages venv.')
  if not (runtime / 'venv/bin/python').is_file():
    errors.append('Runtime Python venv is missing.')
  cache = runtime / 'cache'
  try:
    last = json.loads((cache / 'last-loaded.json').read_text())
  except (OSError, ValueError):
    last = {}
  if not isinstance(last, dict) or any(last.get(k) != v for k, v in
      {'sha256': spec['sha256'], 'frame_skip': spec['frame_skip'], 'backend': 'trt'}.items()):
    errors.append('Preload record does not select the pinned TensorRT model.')
  engines = []
  for metadata in (cache / 'engines').glob(spec['sha256'][:16] + '.*.json'):
    try:
      data = json.loads(metadata.read_text())
      plan = metadata.with_suffix('.plan')
      matching_runtime = packages['tensorrt'] is None or data.get('trt_version') == packages['tensorrt']
      if matching_runtime and data.get('spec') == spec and plan.is_file() and plan.stat().st_size > 0:
        engines.append({'file': plan.name, 'trt_version': data.get('trt_version'), 'device': data.get('device')})
    except (OSError, ValueError, AttributeError):
      continue
  if not engines:
    errors.append('No TensorRT plan with the exact pinned model contract was found.')
  model = cache / 'models' / (spec['sha256'][:16] + '.onnx')
  model_check = 'not hashed (use --verify-model)'
  if not model.is_file() or model.stat().st_size != spec['nbytes']:
    errors.append('Cached original ONNX is missing or has the wrong size.')
  elif verify_model:
    sha = hashlib.sha256()
    with model.open('rb') as source:
      for block in iter(lambda: source.read(4 << 20), b''):
        sha.update(block)
    model_check = sha.hexdigest()
    if model_check != spec['sha256']:
      errors.append('Cached original ONNX checksum mismatch.')
  existing = runtime
  while not existing.exists() and existing != existing.parent:
    existing = existing.parent
  return {
    'scope': 'Static inventory only; does not establish engine load, USB health or inference timing.',
    'static_checks_ok': not errors, 'errors': errors, 'source_commit': commit,
    'board': board, 'jetpack_l4t': release, 'python': sys.executable, 'packages': packages,
    'free_bytes': shutil.disk_usage(existing).free, 'engines': engines, 'model_sha256': model_check,
    'power_mode': command('/usr/sbin/nvpmodel', '-q'),
    'services': {name: command('systemctl', 'is-active', name + '.service') for name in SERVICES},
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--runtime', type=Path, required=True)
  parser.add_argument('--verify-model', action='store_true')
  args = parser.parse_args()
  result = inspect(args.runtime.resolve(), args.verify_model)
  print(json.dumps(result, indent=2))
  return 0 if result['static_checks_ok'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
