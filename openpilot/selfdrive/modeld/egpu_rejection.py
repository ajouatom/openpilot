"""Visible quarantine and a one-shot, next-boot verification request.

Web requests never remove rejection/validation markers or start GPU work.
The normal installer consumes a request before rehashing and boot validation.
"""
import hashlib
import json
import os
from pathlib import Path

from openpilot.selfdrive.modeld.egpu_events import error_summary, log_event
from openpilot.selfdrive.modeld.egpu_worker_progress import boot_identity

REQUEST = 'retry_request.json'


def _read(path):
  with path.open('rb') as f:
    data = f.read(65537)
  if len(data) > 65536:
    raise ValueError('oversized eGPU diagnostic')
  return data


def _binding(root):
  manifest = json.loads(_read(root / 'installed.json'))
  raw = _read(root / 'last_failure.json')
  failure = json.loads(raw)
  sha = manifest['pickle']['sha256']
  if _read(root / 'rejected').decode() != sha or failure['pickle_sha256'] != sha or failure.get('rejected') is not True:
    raise ValueError('rejection identity does not match')
  binding = {'pickle_sha256': sha, 'runtime_sha256': manifest['runtime']['sha256'],
             'failure_sha256': hashlib.sha256(raw).hexdigest()}
  return binding, manifest, failure


def rejection_status(root: Path) -> dict:
  if not (root / 'rejected').exists():
    return {'blocked': False}
  result = {'blocked': True, 'retry_pending': False, 'can_request': False, 'reason': 'diagnostics_missing'}
  try:
    binding, manifest, failure = _binding(root)
    from openpilot.selfdrive.modeld.precompiled_model import gpu_restart_failure
    automatic = (isinstance(failure.get('boot_id'), str) and bool(failure['boot_id']) and
                 failure.get('phase') in ('load', 'boot_validation', 'inference') and
                 gpu_restart_failure(failure.get('error', ''), manifest))
    result.update(can_request=bool(boot_identity()), phase=failure.get('phase'),
                  error=error_summary(failure.get('error', '')),
                  reason='device_restart_required' if automatic else 'verification_required')
    try:
      request = json.loads(_read(root / REQUEST))
      result['retry_pending'] = request.get('binding') == binding and bool(request.get('boot_id'))
    except (OSError, ValueError, TypeError, AttributeError):
      pass
  except (OSError, ValueError, KeyError, TypeError, AttributeError):
    pass
  return result


def request_retry(root: Path) -> None:
  binding, _, _ = _binding(root)
  boot = boot_identity()
  if not boot:
    raise ValueError('current boot identity unavailable')
  temporary = root / (REQUEST + '.tmp')
  temporary.write_text(json.dumps({'binding': binding, 'boot_id': boot}))
  os.replace(temporary, root / REQUEST)
  log_event('artifact_retry_requested', model=binding['pickle_sha256'], boot_id=boot,
            next='full reboot; hashes and startup smoke test required')


def consume_retry_request(root: Path, catalog: dict) -> bool:
  try:
    request = json.loads(_read(root / REQUEST))
    binding, _, _ = _binding(root)
    boot = boot_identity()
    if not (boot and request.get('boot_id') and request['boot_id'] != boot and request.get('binding') == binding and
            catalog['pickle']['sha256'] == binding['pickle_sha256'] and catalog['runtime']['sha256'] == binding['runtime_sha256']):
      return False
    # Consume BEFORE any download/GPU work, so a failed attempt is not replayed.
    (root / REQUEST).unlink()
    return True
  except (OSError, ValueError, KeyError, TypeError, AttributeError):
    return False
