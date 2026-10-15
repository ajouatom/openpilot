"""Bounded background eGPU recovery while the internal model keeps publishing.

No model load, USB wait, probe, or worker join runs on the frame thread.
The sole candidate uses AMD only, never the internal QCOM GPU. A local health
watchdog cancels the attempt if internal publications stop or become late.
"""
import json
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace

from openpilot.common.egpu_recovery_status import BUDGET, STATUS, write_status
from openpilot.selfdrive.modeld.egpu_events import error_summary, log_event
from openpilot.selfdrive.modeld.model_join import may_join
from openpilot.selfdrive.modeld.precompiled_model import gpu_bootloader_timeout, gpu_transport_failure, gpu_usb_link_failure, gpu_worker_busy

DELAYS = (10., 30., 60.)
HEALTH_SECONDS = 3.
MAX_PUBLICATION_GAP = .1
MAX_INTERNAL_WORK = .05
READY_SECONDS = 30.
TASK_SECONDS = 150.


class ReadyExpired(RuntimeError):
  pass


def background_scheduling():
  import os
  if hasattr(os, 'sched_setscheduler'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.sched_setaffinity(0, {0, 1, 2, 3})
    os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 10)


def retryable(error, manifest):
  # Integrity, contract, numerical and unknown driver errors remain rejected.
  return (gpu_transport_failure(str(error)) or gpu_usb_link_failure(str(error), manifest) or
          (isinstance(error, TimeoutError) and str(error) == 'precompiled eGPU worker timed out'))


def retryable_preparation(error, manifest):
  return (retryable(error, manifest) or isinstance(error, ReadyExpired) or
          gpu_worker_busy(str(error)) or
          gpu_bootloader_timeout(str(error), 'load') or
          re.search(r'^(?:RuntimeError: )?PCIe link not up \(LTSSM=0x[0-9a-f]+\)\Z', str(error).rstrip(), re.MULTILINE) is not None)


def prepare_candidate(path, width, height, cancel, manifest):
  import numpy as np
  from openpilot.selfdrive.modeld.precompiled_runner import PrecompiledModelState
  if json.loads((path.parent / 'installed.json').read_text()) != manifest or (path.parent / 'rejected').exists():
    raise ValueError('eGPU recovery artifact changed or rejected')
  available = next(int(line.split()[1]) * 1024 for line in Path('/proc/meminfo').read_text().splitlines()
                   if line.startswith('MemAvailable:'))
  if available < max(2 * 1024**3, 2 * path.stat().st_size + 512 * 1024**2):
    raise RuntimeError('insufficient memory for background eGPU recovery')
  model = PrecompiledModelState(width, height, path, recovery_cancel=cancel)
  try:
    frames = {name: SimpleNamespace(data=np.zeros(model.frame_size, np.uint8)) for name in ('img', 'big_img')}
    transforms = {name: np.eye(3, dtype=np.float32) for name in frames}
    inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1., 0.], np.float32),
              'action_t': np.array([.2, .2], np.float32)}
    for index in range(5):
      if cancel.is_set():
        raise InterruptedError('eGPU recovery cancelled')
      started = time.monotonic()
      output = model.run(frames, transforms, inputs, False)
      if output is None or (index >= 2 and time.monotonic() - started > .05):
        raise RuntimeError('eGPU recovery probe was missing or late')
      cancel.wait(.05)
    # Synthetic probes must not become the live model's recurrent history.
    model.reset_after_probe()
    return model
  except BaseException:
    model.close()
    raise


class RecoveryTask:
  def __init__(self, prepare, healthy, clock=time.monotonic):
    self.clock, self.healthy = clock, healthy
    self.started_at = clock()
    self.cancel = threading.Event()
    self.done = threading.Event()
    self.lock = threading.Lock()
    self.model = None
    self.error = None
    self.claimed = False
    self.ready_at = None
    self.thread = threading.Thread(target=self._work, args=(prepare,), name='egpu-recovery', daemon=True)
    self.thread.start()

  def _watch(self):
    while not self.done.wait(.02):
      if self.claimed:
        return
      if not self.healthy():
        self.error = RuntimeError('internal model publication delayed')
        self.cancel.set()
        return

  def _work(self, prepare):
    model = None
    try:
      # Threads inherit modeld's realtime scheduling/affinity. Drop that BEFORE
      # spawning the GPU worker; keep the preparation off control/model cores.
      background_scheduling()
      threading.Thread(target=self._watch, name='egpu-recovery-health', daemon=True).start()
      model = prepare(self.cancel)
      with self.lock:
        if not self.cancel.is_set():
          self.model, self.ready_at = model, self.clock()
      while not self.cancel.wait(.05):
        if self.claimed:
          model = None  # ownership transferred to modeld
          return
        if self.ready_at is None or self.clock() - self.ready_at >= READY_SECONDS:
          self.error = ReadyExpired('eGPU ready wait expired')
          break
        if model.process.poll() is not None:
          self.error = RuntimeError('eGPU prepared worker exited')
          break
    except Exception as exc:
      self.error = exc
    finally:
      with self.lock:
        self.model = None
        if self.claimed:
          model = None
      try:
        if model is not None:
          model.close()
      except Exception as exc:
        self.error = exc
      finally:
        self.done.set()

  def take(self):
    with self.lock:
      if (self.model is None or self.cancel.is_set() or not self.healthy() or
          self.clock() - self.ready_at >= READY_SECONDS or self.model.process.poll() is not None):
        return None
      model, self.model = self.model, None
      self.claimed = True
      # After transfer only live modeld owns cancellation and shared buffers.
      model.recovery_cancel = None
      return model


class EgpuRecovery:
  def __init__(self, path, width, height, *, clock=time.monotonic, prepare=None,
               task_factory=RecoveryTask, status_path=STATUS, budget_path=BUDGET):
    self.path, self.clock = Path(path), clock
    self.manifest = json.loads((self.path.parent / 'installed.json').read_text())
    self.prepare = prepare or (lambda cancel: prepare_candidate(self.path, width, height, cancel, self.manifest))
    self.task_factory, self.status_path, self.budget_path = task_factory, status_path, budget_path
    self.task = None
    self.state, self.reason = 'idle', ''
    self.healthy_since = self.last_publication = None
    self.next_attempt = 0.
    self.next_status = 0.
    self.attempts = 0
    self.exhausted = False
    self.last_wait_reason, self.next_wait_log = None, 0.
    try:
      value = json.loads(budget_path.read_text())
      self.attempts, self.exhausted = int(value['attempts']), bool(value['stopped'])
      if not 0 <= self.attempts <= len(DELAYS) or not isinstance(value['stopped'], bool):
        self.exhausted = True
    except FileNotFoundError:
      pass
    except (OSError, ValueError, TypeError, KeyError):
      self.exhausted = True  # Never reset a damaged boot budget permissively.

  def _save_budget(self):
    write_status({'attempts': self.attempts, 'stopped': self.exhausted}, self.budget_path)

  def _event(self, event, **fields):
    log_event(event, model=self.path.parent.name, **{'attempt': self.attempts, 'limit': len(DELAYS), **fields})

  def _scheduled(self):
    self._event('retry_scheduled', attempt=self.attempts + 1,
                delay_s=DELAYS[self.attempts], requires='USB present and healthy internal output for 3s',
                active_model='internal')

  def failed(self, error):
    self._event('fallback', error=error_summary(error), active_model='internal',
                retryable=retryable(error, self.manifest),
                artifact_blocked=(self.path.parent / 'rejected').exists())
    self.healthy_since = self.last_publication = None
    if self.exhausted or self.attempts >= len(DELAYS) or not retryable(error, self.manifest):
      self.stop('not_retryable' if not retryable(error, self.manifest) else 'attempts_exhausted')
    elif (self.path.parent / 'rejected').exists() or self.manifest.get('format') != 'comma-generic-onnx':
      self.stop('not_retryable')
    else:
      self.state, self.reason = 'waiting', ''
      self.next_attempt = self.clock() + DELAYS[self.attempts]
      self._scheduled()

  def stop(self, reason):
    if self.state == 'stopped':
      return
    self.state, self.reason, self.exhausted = 'stopped', reason, True
    if self.task is not None:
      self.task.cancel.set()
    self._save_budget()
    self._event('retry_stopped', reason=reason, active_model='internal', retry=False)

  def healthy(self):
    return self.last_publication is not None and 0 <= self.clock() - self.last_publication <= MAX_PUBLICATION_GAP

  def observe(self, published, work_seconds, frame_dropped):
    now = self.clock()
    good = (published and 0 <= work_seconds <= MAX_INTERNAL_WORK and not frame_dropped and
            (self.last_publication is None or 0 <= now - self.last_publication <= MAX_PUBLICATION_GAP))
    self.last_publication = now if published else None
    if not good:
      self.healthy_since = None
      if self.task is not None:
        self.stop('internal_delayed')
    elif self.healthy_since is None:
      self.healthy_since = now

  def tick(self, sm, present):
    now = self.clock()
    if self.task is not None:
      if now - self.task.started_at >= TASK_SECONDS:
        self.stop('cleanup_timeout')
      if not present and not self.task.cancel.is_set():
        self._event('retry_cancelled', reason='USB absent', active_model='internal')
        self.task.cancel.set()
      if self.task.done.is_set():
        error, self.task = self.task.error, None
        self._event('retry_failed', error=error_summary(error) if error is not None else 'worker ended without handover',
                    active_model='internal')
        if self.state != 'stopped':
          if self.attempts >= len(DELAYS) or (error is not None and not retryable_preparation(error, self.manifest)):
            self.stop('attempts_exhausted' if self.attempts >= len(DELAYS) else 'probe_failed')
          else:
            self.state = 'waiting'
            self.next_attempt = now + DELAYS[self.attempts]
            self._scheduled()
      elif self.task.cancel.is_set():
        self.state = 'stopped' if self.exhausted else 'recovering'
      elif self.task.ready_at is not None:
        if self.state != 'ready':
          self._event('retry_ready', result='probe passed; waiting for handover permission', active_model='internal')
        self.state = 'ready'
        valid = {k: sm.valid[k] and sm.alive[k] for k in ('carState', 'selfdriveState', 'carControl')}
        if present and may_join(now, sm, valid, sm.recv_time):
          model = self.task.take()
          if model is not None:
            self.task, self.state = None, 'idle'
            self._event('retry_selected', result='prepared worker handed to modeld')
            self.publish(force=True)
            return model
    if (self.state == 'waiting' and self.task is None and present and self.healthy() and
        self.healthy_since is not None and now - self.healthy_since >= HEALTH_SECONDS and now >= self.next_attempt):
      # Persist before launching. modeld restarts cannot create an infinite loop.
      self.attempts += 1
      self._save_budget()
      self.state = 'recovering'
      self._event('retry_started', active_model='internal')
      self.task = self.task_factory(self.prepare, self.healthy)
      self.task.started_at = now
    if self.state == 'waiting':
      reason = ('USB absent' if not present else 'internal output not yet healthy for 3s'
                if not self.healthy() or self.healthy_since is None or now - self.healthy_since < HEALTH_SECONDS
                else 'backoff delay')
      if reason != self.last_wait_reason and now >= self.next_wait_log:
        self._event('retry_waiting', reason=reason, retry_in_s=max(0, round(self.next_attempt - now)))
        self.last_wait_reason, self.next_wait_log = reason, now + 5.
    self.publish()
    return None

  def publish(self, force=False):
    now = self.clock()
    if force or now >= self.next_status:
      write_status({'state': self.state, 'reason': self.reason, 'attempts': self.attempts,
                    'limit': len(DELAYS), 'updated': now,
                    'retry_in_seconds': max(0, round(self.next_attempt - now))}, self.status_path)
      self.next_status = now + 1
