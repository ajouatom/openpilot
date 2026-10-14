"""Crash notification in manager; capture/upload in the independent tombstoned.

The supervisor only sends bounded nonblocking local datagrams. Files, tmux,
Params, DNS, session creation and HTTP retries belong to the diagnostic process.
"""
import faulthandler
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
from collections import deque


SOCKET_PATH = '/dev/shm/carrot-process-crash.sock'
REPORT_DIR = '/data/carrot/process_crashes'
MAX_PENDING = 32
MAX_REPORTS = 20
MAX_TEXT = 256 * 1024
RETRY_SECONDS = 60


def enable_fault_handler():
  # Keep stderr open for the lifetime of the child. Fatal signals cannot run
  # Python exception handlers; faulthandler prints the available Python stacks.
  try:
    faulthandler.enable(all_threads=True)
  except (OSError, RuntimeError, ValueError):
    pass


class CrashSender:
  def __init__(self, address=SOCKET_PATH):
    self.address = address
    self.pending = deque(maxlen=MAX_PENDING)

  def report(self, name, pid, exitcode):
    if not pid or exitcode in (None, 0):
      return
    event = {'id': f'{os.getpid()}-{pid}-{time.monotonic_ns()}', 'name': name,
             'pid': pid, 'exitcode': exitcode, 'time': time.time(), 'manager_pid': os.getpid(),  # noqa: TID251 - incident wall time
             'git_branch': os.environ.get('GIT_BRANCH', ''), 'git_commit': os.environ.get('GIT_COMMIT', ''),
             'dongle_id': os.environ.get('DONGLE_ID', ''), 'pane': os.environ.get('TMUX_PANE', '')}
    self.pending.append(json.dumps(event).encode())
    self.flush()

  def flush(self):
    if not self.pending:
      return
    try:
      with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
        sock.setblocking(False)
        while self.pending:
          sock.sendto(self.pending[0], self.address)
          self.pending.popleft()
    except (OSError, AttributeError):
      # Collector may be starting/restarting; retry on the next manager tick.
      pass


_sender = CrashSender()


def report_process_exit(name, pid, exitcode):
  try:
    _sender.report(name, pid, exitcode)
  except Exception:
    # Diagnostics must never prevent reaping/restarting a managed process.
    pass


def flush_crash_reports():
  try:
    _sender.flush()
  except Exception:
    pass


def capture_tmux(event):
  command = ['tmux', 'capture-pane', '-p', '-J', '-S', '-1000']
  if event.get('pane'):
    command += ['-t', event['pane']]
  try:
    result = subprocess.run(command, capture_output=True, timeout=3, check=True)
    return result.stdout[-MAX_TEXT:].decode('utf-8', errors='replace')
  except (OSError, subprocess.SubprocessError) as e:
    return f'Tmux capture unavailable: {type(e).__name__}. Exit metadata retained.\n'


def device_payload(event):
  from openpilot.common.params import Params
  params = Params()
  keys = {'car_name': 'CarName', 'git_branch': 'GitBranch', 'git_commit': 'GitCommit',
          'git_commit_date': 'GitCommitDate', 'github_id': 'GithubUsername',
          'git_remote': 'GitRemote', 'dongle_id': 'DongleId', 'device_serial': 'HardwareSerial'}
  payload = {'tmux_why': 'process_crash'}
  for field, key in keys.items():
    value = event.get(field) or params.get(key) or ''
    payload[field] = value.decode('utf-8', errors='replace') if isinstance(value, bytes) else str(value)
  return payload


def upload_report(target, payload, path):
  # Reuse the two existing tmux destinations; no new endpoint or settings dump.
  import requests
  from openpilot.selfdrive.carrot.web_upload import (
    carrot_logs_web_target, create_web_upload_session_sync, post_tmux_web, tmux_web_target, web_upload_settings,
  )
  if os.environ.get('FAKEUPLOAD') == '1':
    return False
  if target == 'dsm':
    from openpilot.selfdrive.carrot.server.services.web_settings import read_web_settings
    try:
      settings = read_web_settings()
    except Exception:
      settings = {}
    base, token = web_upload_settings(settings)
    token = token or create_web_upload_session_sync(base, payload, requests.post, 'tmux')
    url, headers = tmux_web_target(settings, token)
  else:
    url, headers = carrot_logs_web_target()
  response = post_tmux_web(url, headers, payload, str(path), post=requests.post)
  return 200 <= response.status_code < 300


class CrashCollector:
  def __init__(self, root=REPORT_DIR, address=SOCKET_PATH, capture=capture_tmux, payload=device_payload, upload=upload_report):
    self.root, self.address = Path(root), address
    self.capture, self.payload, self.upload = capture, payload, upload
    self.stop_event = threading.Event()
    self.lock = threading.Lock()
    self.threads = []
    self.retry_at = {}
    self.sock = None

  @staticmethod
  def write_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data), encoding='utf-8')
    temporary.replace(path)

  def record(self, event):
    # Socket input controls neither a path nor a shell command.
    if not re.fullmatch(r'[0-9]{1,20}-[0-9]{1,20}-[0-9]{1,20}', str(event.get('id', ''))):
      raise ValueError('invalid crash id')
    if not re.fullmatch(r'[A-Za-z0-9_]{1,100}', str(event.get('name', ''))):
      raise ValueError('invalid process name')
    if not isinstance(event.get('pid'), int) or not isinstance(event.get('exitcode'), int):
      raise ValueError('invalid process status')
    if event['exitcode'] < 0:
      try:
        event['signal'] = signal.Signals(-event['exitcode']).name
      except ValueError:
        event['signal'] = 'unknown'
    event['captured_at'] = time.time()  # noqa: TID251 - compare capture and incident wall time
    # Capture first, before any Params read or upload can wait on I/O.
    text = self.capture(event).encode('utf-8')[-MAX_TEXT:].decode('utf-8', errors='replace')
    try:
      payload = self.payload(event)
    except Exception:
      payload = {k: event.get(k, '') for k in ('git_branch', 'git_commit', 'dongle_id')}
      payload['tmux_why'] = 'process_crash'
    path = self.root / event['id']
    with self.lock:
      self.root.mkdir(parents=True, exist_ok=True)
      if path.with_suffix('.json').exists():
        return
      path.with_suffix('.txt').write_text('PROCESS_EXIT ' + json.dumps(event) + '\n\n' + text, encoding='utf-8')
      self.write_json(path.with_suffix('.json'), {'event': event, 'payload': payload, 'sent': []})
      reports = sorted(self.root.glob('*.json'), key=lambda p: p.stat().st_mtime_ns)
      for old in reports[:-MAX_REPORTS]:
        old.with_suffix('.txt').unlink(missing_ok=True)
        old.unlink(missing_ok=True)
        self.retry_at.pop(old.name, None)

  def upload_pending(self, now=None):
    now = time.monotonic() if now is None else now
    with self.lock:
      paths = list(self.root.glob('*.json'))
    for path in paths:
      if self.stop_event.is_set() or now < self.retry_at.get(path.name, 0):
        continue
      try:
        with self.lock:
          report = json.loads(path.read_text(encoding='utf-8'))
        for target in ('dsm', 'carrot_logs'):
          if target in report['sent']:
            continue
          try:
            delivered = self.upload(target, report['payload'], path.with_suffix('.txt'))
          except Exception:
            delivered = False
          if delivered:
            with self.lock:
              if path.exists():
                report['sent'].append(target)
                self.write_json(path, report)
      except (OSError, ValueError, KeyError):
        pass
      with self.lock:
        if path.exists():
          self.retry_at[path.name] = now + RETRY_SECONDS

  def _receive(self):
    while not self.stop_event.is_set():
      try:
        event = json.loads(self.sock.recv(8192))
        if isinstance(event, dict):
          self.record(event)
      except TimeoutError:
        continue
      except Exception as e:
        if not self.stop_event.is_set():
          print(f'[process_crash] capture failed: {type(e).__name__}', flush=True)

  def _upload(self):
    while not self.stop_event.is_set():
      try:
        self.upload_pending()
      except Exception as e:
        print(f'[process_crash] upload retry: {type(e).__name__}', flush=True)
      self.stop_event.wait(1)

  def start(self):
    self.root.mkdir(parents=True, exist_ok=True)
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
      Path(self.address).unlink(missing_ok=True)
      self.sock.bind(self.address)
      os.chmod(self.address, 0o600)
      self.sock.settimeout(0.5)
    except Exception:
      self.sock.close()
      raise
    for target in (self._receive, self._upload):
      thread = threading.Thread(target=target, daemon=True)
      thread.start()
      self.threads.append(thread)
    return self

  def close(self):
    self.stop_event.set()
    if self.sock is not None:
      self.sock.close()
    for thread in self.threads:
      thread.join(1)


def start_crash_reporting():
  from openpilot.system.hardware import PC
  if not PC:
    try:
      return CrashCollector().start()
    except Exception as e:
      print(f'[process_crash] collector unavailable: {type(e).__name__}', flush=True)
  return None
