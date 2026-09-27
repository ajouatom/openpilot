"""Verified NAS runtime/model updates; activation only with both services stopped.

The timer stages while C4 explicitly reports offroad. The boot service validates
the staged engine before switching sources. A failed probe keeps the old release.
No OS, user identity, network credentials or vehicle settings are updated.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import urllib.parse
import urllib.request

ROOT = Path('/opt/carrot-jetlink')
DEFAULT_MANIFEST = 'https://upload.shind0.synology.me/models/jetlink-host-stable/manifest.json'


def verify_signature(manifest):
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  value = {k: v for k, v in manifest.items() if k != 'signature'}
  payload = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
  key = ECC.import_key(Path(__file__).with_name('release-signing-public.pem').read_text())
  eddsa.new(key, 'rfc8032').verify(payload, base64.b64decode(manifest.get('signature', ''), validate=True))


def digest(path):
  value = hashlib.sha256()
  with path.open('rb') as source:
    for block in iter(lambda: source.read(4 << 20), b''):
      value.update(block)
  return value.hexdigest()


def atomic_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix('.tmp')
  with temporary.open('w') as target:
    json.dump(value, target)
    target.flush()
    os.fsync(target.fileno())
  os.replace(temporary, path)


def checked_url(url):
  parsed = urllib.parse.urlsplit(url)
  if parsed.scheme != 'https' or parsed.netloc != 'upload.shind0.synology.me' or not parsed.path.startswith('/models/'):
    raise ValueError('Updates require the configured HTTPS NAS model host')
  return url


def fetch(url, path, sha256, size):
  checked_url(url)
  if not re.fullmatch('[0-9a-f]{64}', sha256) or not isinstance(size, int) or not 0 < size <= 4 << 30:
    raise ValueError('Invalid download identity')
  if path.exists() and path.stat().st_size == size and digest(path) == sha256:
    return
  temporary = path.with_suffix('.part')
  try:
    with urllib.request.urlopen(url, timeout=45) as response, temporary.open('wb') as output:
      checked_url(response.url)
      count = 0
      for block in iter(lambda: response.read(1 << 20), b''):
        count += len(block)
        if count > size:
          raise ValueError('Oversized download')
        output.write(block)
      output.flush()
      os.fsync(output.fileno())
    if count != size or digest(temporary) != sha256:
      raise ValueError('Update checksum/length mismatch')
    os.replace(temporary, path)
  finally:
    temporary.unlink(missing_ok=True)


def validate_manifest(manifest):
  if manifest.get('format') != 1 or not re.fullmatch('[0-9a-f]{40}', str(manifest.get('source_commit', ''))):
    raise ValueError('Unsupported release manifest')
  if manifest.get('runtime') != {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'}:
    raise ValueError('This updater cannot change the OS/runtime ABI')
  if not re.fullmatch('[0-9a-f]{64}', str(manifest.get('model', {}).get('sha256', ''))):
    raise ValueError('Model identity required')
  checked_url(manifest['bundle']['url'])
  checked_url(manifest['model']['url'])


def probe_release(release):
  command = ['runuser', '-u', 'jetlink', '--', str(ROOT / 'venv/bin/python'),
             str(release / 'tools/jetlink/probe_release.py'), str(ROOT / 'cache')]
  process = subprocess.Popen(command, start_new_session=True)
  try:
    code = process.wait(timeout=960)
  except BaseException:
    # Killing only runuser can leave its GPU-building Python child alive.
    # Reap the entire isolated probe group before the old server may start.
    try:
      os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
    process.wait()
    raise
  if code:
    raise subprocess.CalledProcessError(code, command)


def stage(manifest_url=DEFAULT_MANIFEST):
  from finalize_sd_image import extract_bundle
  checked_url(manifest_url)
  with urllib.request.urlopen(manifest_url, timeout=30) as response:
    checked_url(response.url)
    manifest = json.loads(response.read(65537))
  validate_manifest(manifest)
  verify_signature(manifest)
  commit = manifest['source_commit']
  if (ROOT / 'current/SOURCE_COMMIT').read_text().strip() == commit:
    return
  # Installation dependencies and kernel changes need a separately tested image.
  l4t = subprocess.check_output(['dpkg-query', '-W', '-f=${Version}', 'nvidia-l4t-core'], text=True)
  if not l4t.startswith('36.4.7-'):
    raise ValueError('Installed L4T does not match the release')
  runtime = subprocess.check_output([str(ROOT / 'venv/bin/python'), '-c', 'import tensorrt; print(tensorrt.__version__)'], text=True).strip()
  if runtime != manifest['runtime']['tensorrt'] or os.uname().machine != 'aarch64':
    raise ValueError('Installed TensorRT/architecture does not match')
  updates = ROOT / 'updates'
  updates.mkdir(exist_ok=True)
  bundle = updates / (commit + '.tar.gz')
  item = manifest['bundle']
  fetch(item['url'], bundle, item['sha256'], item['size'])
  release = ROOT / 'releases' / commit
  if not release.exists():
    with tempfile.TemporaryDirectory(dir=ROOT / 'releases', prefix='update-') as temporary:
      candidate = Path(temporary)
      extract_bundle(bundle, candidate)
      if (candidate / 'SOURCE_COMMIT').read_text().strip() != commit:
        raise ValueError('Bundle source identity mismatch')
      spec = json.loads((candidate / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
      if (spec['sha256'], spec['nbytes']) != (manifest['model']['sha256'], manifest['model']['size']):
        raise ValueError('Bundle model contract differs from manifest')
      (candidate / '.bundle-sha256').write_text(item['sha256'])
      candidate.chmod(0o755)
      os.rename(candidate, release)
  elif not (release / '.bundle-sha256').exists() or (release / '.bundle-sha256').read_text() != item['sha256']:
    raise ValueError('Existing release has a different identity')
  model = manifest['model']
  destination = ROOT / 'cache/models' / (model['sha256'][:16] + '.onnx')
  fetch(model['url'], destination, model['sha256'], model['size'])
  # Cache files are consumed and managed by the unprivileged inference service.
  import pwd
  owner = pwd.getpwnam('jetlink')
  os.chown(destination, owner.pw_uid, owner.pw_gid)
  atomic_json(updates / 'pending.json', manifest)
  atomic_json(updates / 'status.json', {'state': 'staged', 'source_commit': commit, 'updated': time.time()})
  os.sync()


def activate():
  pending = ROOT / 'updates/pending.json'
  transaction = ROOT / 'updates/transaction.json'
  if not pending.exists() and not transaction.exists():
    return
  for name in ('carrot-jetlink', 'carrot-jetlink-hud'):
    if subprocess.run(['systemctl', 'is-active', '--quiet', name]).returncode == 0:
      raise RuntimeError('Activation requires stopped inference and HUD services; use the next boot')
  if transaction.exists():
    saved = json.loads(transaction.read_text())
    old = Path(saved['release'])
    if old.is_symlink() or old.resolve().parent != (ROOT / 'releases').resolve() or not old.is_dir():
      raise ValueError('Invalid recovery release')
    link = ROOT / 'current.recovery'
    link.unlink(missing_ok=True)
    link.symlink_to(old)
    os.replace(link, ROOT / 'current')
    if saved['last_loaded'] is not None:
      (ROOT / 'cache/last-loaded.json').write_text(saved['last_loaded'])
    if pending.exists():
      os.replace(pending, pending.with_name('interrupted.json'))
    transaction.unlink()
    atomic_json(ROOT / 'updates/status.json', {'state': 'recovered', 'updated': time.time()})
    os.sync()
    return
  manifest = json.loads(pending.read_text())
  validate_manifest(manifest)
  verify_signature(manifest)
  release = ROOT / 'releases' / manifest['source_commit']
  if release.is_symlink() or release.resolve().parent != (ROOT / 'releases').resolve():
    raise ValueError('Invalid release directory')
  previous = (ROOT / 'current').resolve()
  remembered = ROOT / 'cache/last-loaded.json'
  previous_model = remembered.read_bytes() if remembered.exists() else None
  saved = {'release': str(previous), 'last_loaded': previous_model.decode() if previous_model else None}
  atomic_json(transaction, saved)
  os.sync()
  try:
    # No USB/control connection: load/build and execute synthetic tensors in a
    # separate process before the new sources are reachable by the vehicle.
    probe_release(release)
    link = ROOT / 'current.next'
    link.unlink(missing_ok=True)
    link.symlink_to(release)
    atomic_json(ROOT / 'updates/previous.json', saved)
    os.replace(link, ROOT / 'current')
    atomic_json(ROOT / 'updates/status.json', {'state': 'applied', 'source_commit': manifest['source_commit'], 'updated': time.time()})
    pending.unlink()
    os.sync()
    transaction.unlink()
    os.sync()
  except Exception as error:
    if (ROOT / 'current').resolve() != previous:
      link = ROOT / 'current.rollback'
      link.unlink(missing_ok=True)
      link.symlink_to(previous)
      os.replace(link, ROOT / 'current')
    if previous_model is not None:
      remembered.write_bytes(previous_model)
    atomic_json(ROOT / 'updates/status.json', {'state': 'rejected', 'error': str(error)[:240], 'updated': time.time()})
    if pending.exists():
      os.replace(pending, pending.with_name('rejected.json'))
    transaction.unlink(missing_ok=True)
    print('Candidate rejected; retaining previous release:', error, flush=True)


def automatic_stage():
  from hud_protocol import read_snapshot
  snapshot = read_snapshot()
  if snapshot is None:
    return  # Missing telemetry never means parked/offroad.
  import base64
  if base64.b64decode(snapshot[1].get('params', {}).get('IsOnroad', '')) != b'0':
    return
  stage()


def main():
  import fcntl
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=['stage', 'activate', 'automatic'])
  parser.add_argument('--manifest', default=DEFAULT_MANIFEST)
  args = parser.parse_args()
  if os.geteuid() != 0:
    raise PermissionError('Run through the installed update service (root)')
  with (ROOT / 'update.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if args.action == 'activate':
      activate()
    elif args.action == 'automatic':
      automatic_stage()
    else:
      stage(args.manifest)


if __name__ == '__main__':
  main()
