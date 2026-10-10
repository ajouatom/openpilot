"""Bounded, read-only eGPU evidence appended by tmux capture workers (stdlib only)."""
from __future__ import annotations

import json
from collections import deque
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time

LIMIT = 64 * 1024
PARAM_KEYS = ('UsbGpuPresent', 'UsbGpuHardwareSeen', 'UsbGpuCompiled', 'UsbGpuLoading',
              'UsbGpuActive', 'UsbGpuStartupFailed', 'GitCommit', 'GitBranch')
MARKER = '\n===== EGPU DIAGNOSTICS v1 =====\n'


def redact(text: str) -> str:
  # Never include authorization material from exception strings or downloader logs.
  lines = ['[credential line omitted]' if re.search(r'token|password|authorization|cookie|secret', line, re.I) else line
           for line in text.splitlines()]
  return re.sub(r'https?://[^\s\"\'<>]+', '[URL omitted]', '\n'.join(lines))


def read_text(path: Path, limit: int = LIMIT, *, tail: bool = False) -> str:
  try:
    with path.open('rb') as f:
      if tail:
        f.seek(max(0, path.stat().st_size - limit))
      data = f.read(limit)
    return data.decode('utf-8', errors='replace')
  except OSError as exc:
    return f'[unavailable: {type(exc).__name__}]'


def read_json(path: Path) -> dict:
  try:
    value = json.loads(read_text(path))
    return value if isinstance(value, dict) else {'read_error': 'not an object'}
  except ValueError:
    return {'read_error': 'missing, unreadable, oversized or invalid JSON'}


def select(value: dict, keys: tuple[str, ...]) -> dict:
  return {k: value[k][-8192:] if isinstance(value[k], str) else value[k]
          for k in keys if k in value and isinstance(value[k], (str, int, float, bool, type(None)))}


def command_tail(argv: list[str], *, cwd: Path | None = None, pattern: str | None = None) -> str:
  # File-backed output bounds RAM even when the kernel ring buffer is large.
  try:
    with tempfile.TemporaryFile() as output:
      result = subprocess.run(argv, cwd=cwd, stdout=output, stderr=subprocess.STDOUT, timeout=2, check=False)
      if pattern is None:
        output.seek(max(0, output.tell() - LIMIT))
        data = output.read(LIMIT).decode('utf-8', errors='replace')
      else:
        # Camera/other kernel traffic can bury the relevant USB events beyond
        # the raw tail. Filter the complete file first, with bounded memory.
        output.seek(0)
        matched = deque(maxlen=200)
        regex = re.compile(pattern, re.I)
        while line := output.readline(8192):
          text = line.decode('utf-8', errors='replace')
          if regex.search(text):
            matched.append(text)
        data = ''.join(matched)[-LIMIT:]
      return f'exit={result.returncode}\n' + data
  except (OSError, subprocess.TimeoutExpired) as exc:
    return f'[unavailable: {type(exc).__name__}]'


def collect_report(cache: Path, params: Path, usb_root: Path, repo: Path, update_log: Path) -> dict:
  report = {'schema_version': 1, 'captured_unix': time.time(), 'captured_monotonic': time.monotonic(),  # noqa: TID251 - correlate persisted failures and clock errors
            'params': {k: read_text(params / k, 256).strip() for k in PARAM_KEYS},
            'git_head': command_tail(['git', 'rev-parse', 'HEAD'], cwd=repo),
            'git_changed_files': command_tail(['git', 'diff', 'HEAD', '--name-only'], cwd=repo),
            'status': select(read_json(cache / 'status.json'),
                             ('state', 'detail', 'error_code', 'retry_count', 'retry_in_seconds', 'sha256',
                              'downloaded_bytes', 'total_bytes', 'updated_at', 'read_error'))}
  state = read_json(cache / 'state.json')
  active = state.get('active')
  report['active_model'] = (select(active, ('model_id', 'sha256', 'filename', 'size'))
                            if isinstance(active, dict) else state.get('read_error', 'no active model'))
  # Include bounded historical artifacts too: active state can be missing after a failed update.
  roots = sorted((cache / 'precompiled').glob('*'))
  roots = [p for p in roots if re.fullmatch(r'[0-9a-f]{64}', p.name) and p.is_dir()]
  active_sha = active.get('sha256') if isinstance(active, dict) else None
  if isinstance(active, dict) and re.fullmatch(r'[0-9a-f]{64}', str(active_sha)):
    filename = active.get('filename', '')
    if isinstance(filename, str) and re.fullmatch(r'[A-Za-z0-9._-]+\.(onnx|pkl)', filename):
      name = Path(filename)
      source = cache / f'{name.stem}-{active_sha[:16]}{name.suffix}'
      report['active_source'] = {'exists': source.is_file(), 'bytes': source.stat().st_size if source.is_file() else None,
                                 'expected_bytes': active.get('size')}
  roots.sort(key=lambda p: p.name != active_sha)
  artifacts = []
  for root in roots[:4]:
    installed = read_json(root / 'installed.json')
    runtime_name = installed.get('runtime_directory', '')
    runtime_safe = isinstance(runtime_name, str) and re.fullmatch(r'runtime-[0-9a-f]{16}', runtime_name)
    entry = 'examples/openpilot/compile_warp.py' if installed.get('format') == 'comma-generic-onnx' else 'model_runtime.py'
    model_file = root / 'model.pkl'
    failure = read_json(root / 'last_failure.json')
    failure_report = select(failure, ('time', 'phase', 'rejected', 'pickle_sha256', 'error', 'boot_id', 'read_error'))
    if isinstance(failure.get('worker'), dict):
      failure_report['worker'] = select(failure['worker'], ('pid', 'stage', 'frame', 'stage_age_ms', 'thread_cpu_seconds',
                                                         'thread_id', 'stat', 'schedstat', 'wchan'))
    artifacts.append({'model_sha256': root.name, 'is_active': root.name == active_sha,
                      'model_exists': model_file.is_file(), 'model_bytes': model_file.stat().st_size if model_file.is_file() else None,
                      'pickle': select(installed.get('pickle', {}) if isinstance(installed.get('pickle'), dict) else {}, ('sha256', 'size')),
                      'catalog': select(installed, ('format', 'gpu_arch', 'runtime_directory', 'read_error')),
                      'runtime_entry_exists': bool(runtime_safe and (root / runtime_name / entry).is_file()),
                      'runtime_tinygrad_exists': bool(runtime_safe and (root / runtime_name / 'tinygrad/__init__.py').is_file()),
                      'rejected': (root / 'rejected').exists(), 'rejected_hash': read_text(root / 'rejected', 128).strip(),
                      'last_failure': failure_report,
                      'boot_validation_exists': (root / 'boot_validation.json').is_file()})
  report['artifacts'] = artifacts
  report['usb_devices'] = []
  for device in sorted(usb_root.glob('*')):
    vendor, product = read_text(device / 'idVendor', 16).strip().lower(), read_text(device / 'idProduct', 16).strip().lower()
    if (vendor, product) in (('add1', '0001'), ('3801', '0001')):
      report['usb_devices'].append({'port': device.name, 'vendor': vendor, 'product': product,
                                    **{k: read_text(device / k, 128).strip() for k in
                                       ('speed', 'authorized', 'power/runtime_status', 'power/control')}})
  kernel = command_tail(['dmesg'], pattern=r'usb|pcie|xhci|over.?current|voltage|unavailable|permitted|denied')
  report['kernel_usb_tail'] = '\n'.join(line for line in kernel.splitlines()
                                       if re.search(r'usb|pcie|xhci|over.?current|voltage|unavailable|permitted|denied|^exit=', line, re.I))[-12000:]
  report['download_error_tail'] = '\n'.join(line for line in read_text(update_log, tail=True).splitlines()
                                           if re.search(r'error|failed|unavailable|reject|precompiled|certificate|download|unavailable', line, re.I))[-8000:]
  report['limits'] = 'No model hashing, GPU initialization, reset or network probe. Missing evidence does not prove hardware health.'
  return report


def append_report(tmux_path: str | Path) -> bool:
  """Persist evidence and attach it to the existing upload; never block capture on failure."""
  path = Path(tmux_path)
  try:
    cache = Path(os.environ.get('CARROT_BIG_MODEL_DIR', '/data/media/0/carrot/models'))
    report = collect_report(cache, Path('/data/params/d'), Path('/sys/bus/usb/devices'),
                            Path(__file__).resolve().parents[2], Path('/tmp/big_model_update.log'))
    payload = json.dumps(_redact_values(report), ensure_ascii=True, indent=2)
    with path.open('a', encoding='utf-8') as f:
      f.write(MARKER + payload + '\n===== END EGPU DIAGNOSTICS =====\n')
    destination = path.with_name('egpu_diagnostics.json')
    fd, temporary = tempfile.mkstemp(prefix='.egpu-', dir=path.parent)
    try:
      with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(payload + '\n')
      os.replace(temporary, destination)
    finally:
      Path(temporary).unlink(missing_ok=True)
    return True
  except Exception as exc:
    try:
      with path.open('a', encoding='utf-8') as f:
        f.write(f'{MARKER}collection failed: {type(exc).__name__}\n')
    except OSError:
      pass
    return False


def _redact_values(value):
  if isinstance(value, str):
    return redact(value)
  if isinstance(value, dict):
    return {k: _redact_values(v) for k, v in value.items()}
  if isinstance(value, list):
    return [_redact_values(v) for v in value]
  return value


if __name__ == '__main__':
  import sys
  append_report(sys.argv[1])
