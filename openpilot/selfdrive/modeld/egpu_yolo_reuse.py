"""Manually leased observation using the driving owner's existing eGPU queue."""
import json
import math
import os

from openpilot.selfdrive.modeld.egpu_yolo import YoloRuntime, artifact_path, camera_time


def session_path():
  return artifact_path().parent / 'reuse_session.json'


def leased(value, now, *, enabled=True):
  expiry = value.get('expires', 0)
  return (isinstance(expiry, (float, int)) and math.isfinite(expiry) and 0 < expiry-now <= 5
          and value.get('mode', 'stationary') in ('stationary', 'road_observation')
          and value.get('prepared') is True and (not enabled or value.get('enabled') is True))


def read_session_details(*, enabled=True):
  try:
    value = json.loads(session_path().read_text())
    if isinstance(value, dict) and value.get('automatic'):
      from openpilot.selfdrive.modeld.egpu_yolo_auto import boot_id, configured
      if not configured() or value.get('boot_id') != boot_id():
        return None
    return value if isinstance(value, dict) and leased(value, camera_time(), enabled=enabled) else None
  except (OSError, ValueError, TypeError):
    return None


def read_session(*, enabled=True, mode=None):
  value = read_session_details(enabled=enabled)
  return value is not None and (mode is None or value.get('mode', 'stationary') == mode)


def stationary_permitted(*, fresh, started, parked, standstill, speed, enabled, lat_active, long_active,
                         primary_seconds, dropped):
  return (fresh and started and parked and standstill and math.isfinite(speed) and abs(speed) < .01
          and not enabled and not lat_active and not long_active
          and math.isfinite(primary_seconds) and 0 < primary_seconds < .06 and not dropped)


def observation_state_permitted(*, fresh, started, gear, speed):
  # This gate authorizes optional compute only, never an actuator command.
  return (fresh and started and gear in ('park', 'drive', 'neutral', 'reverse', 'sport', 'low', 'brake', 'eco', 'manumatic')
          and math.isfinite(speed))


def observation_permitted(*, primary_seconds, dropped, **state):
  return (observation_state_permitted(**state) and math.isfinite(primary_seconds) and 0 < primary_seconds < .06 and not dropped)


def frame_permitted(*, mode='stationary', gear, **state):
  if mode == 'road_observation':
    return observation_permitted(gear=gear, **{k: state[k] for k in ('fresh', 'started', 'speed', 'primary_seconds', 'dropped')})
  return mode == 'stationary' and stationary_permitted(parked=gear == 'park', **state)


class ReuseRuntime(YoloRuntime):
  @classmethod
  def load(cls, input_queue):
    directory = artifact_path().parent
    target = directory / 'egpu2-reuse/yolo_reuse.pkl'
    session = read_session_details(enabled=False)
    from openpilot.selfdrive.modeld.egpu_yolo_auto import configured
    if session is None and configured():
      # Prepare with the normal driving model startup, even if the supervisor
      # has not started yet. A fresh owner-bound lease is still needed to run.
      session = {'mode': 'road_observation', 'automatic': True}
    if session is None or not target.is_file() or (directory/'qcom_enabled').exists():
      return None
    from openpilot.selfdrive.modeld.helpers import load_oob
    with target.open('rb') as stream:
      bundle = load_oob(stream)
    if bundle.get('input_rebinding_passed') is not True:
      raise ValueError('reuse artifact must prove changing input allocation and pixels')
    if bundle.get('output_dtype') not in ('float32', 'float16'):
      raise ValueError('CPU delivery expects compact FP32 or FP16 output')
    if not bundle.get('native') or (bundle['width'], bundle['height']) != (input_queue.shape[-1]*2, input_queue.shape[-2]*2):
      raise ValueError('reuse artifact must retain native driving input resolution')
    # This is a compiled artifact replay on the driving owner's startup thread.
    # No ONNX runner or compiler is invoked on live camera frames.
    runtime = cls(input_queue, bundle)
    from openpilot.selfdrive.modeld.egpu_yolo_budget import RevalidatingBudget
    runtime.budget = RevalidatingBudget(runtime.budget)
    # Lock mode at startup; changing a lease cannot promote a stationary owner.
    runtime.mode = session.get('mode', 'stationary')
    runtime.automatic = session.get('automatic') is True
    runtime.generation = -1
    from openpilot.selfdrive.modeld.egpu_yolo_postprocess import OutputWorker
    runtime.output = OutputWorker(recover=runtime.automatic)
    from openpilot.common.swaglog import cloudlog
    cloudlog.info('resident YOLO prepared: %s, required %.3f ms', bundle['variant'], runtime.budget.estimate*1000+.001*1000)
    return runtime

  def infer(self):
    from openpilot.selfdrive.modeld.egpu_yolo_usb import run_yolo, read_yolo
    start = camera_time()
    raw = run_yolo(self.run, self.queue)
    submitted = camera_time()
    values = read_yolo(raw)
    copied = camera_time()
    self.phases = (submitted-start, copied-submitted, 0.)
    return values

  def after_publish(self, pm, frame_id, sof_ns, eof_ns, received, driving_published, dropped, camera,
                    transform, camera_size, next_frame_ready, inference_started, inference_ended, *, permitted):
    self.budget.observe(frame_id, sof_ns/1e9, received, dropped)
    mode = getattr(self, 'mode', 'stationary')
    if not permitted or not read_session(enabled=False, mode=mode):
      return
    enabled = read_session(mode=mode)
    if getattr(self, 'automatic', False):
      lease = read_session_details(enabled=False)
      if not lease or lease.get('automatic') is not True or lease.get('owner_pid') != os.getpid():
        return
      enabled = lease.get('enabled') is True
      if not self.output.ready():
        return
      if enabled and lease.get('generation', -1) > self.generation and self.budget.settled >= 20:
        self.generation = lease['generation']
        # A completed deadline miss can recover. Keep the increased reservation
        # and cumulative overrun count; never clear a failed GPU call's latch.
        if self.budget.disabled_reason == 'overrun':
          self.budget.disabled_reason = ''
    pending = next_frame_ready() if enabled else False
    start = camera_time()
    # Consider every completed primary frame. The camera/deadline/overrun
    # checks still decide whether this frame has room for optional GPU work.
    # Lamp observation needs a stable few frames, not every driving frame.
    # Bound its duty cycle while retaining all existing deadline/fault guards.
    interval = .2 if self.model_id == 'signal-v33-observe-s260911' else 0.
    reason = self.budget.admit(start, pending, min_interval=interval) if enabled else 'paused'
    values = None
    if reason == 'run':
      try:
        values = self.infer()
      except Exception:
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception('optional resident YOLO failed; disabling this session')
        self.budget.disabled_reason = reason = 'error'
      self.last_execution = camera_time()-start
    elif start-self.last_publish < 1:
      return
    metadata = {
      'frameId': frame_id, 'timestampSof': sof_ns, 'timestampEof': eof_ns,
      'modelId': self.model_id, 'camera': camera, 'state': reason,
      'executionTime': self.last_execution, 'budgetTime': max(0, self.budget.deadline-start),
      'drivingPublishTime': int(driving_published*1e9), 'drivingLatency': max(0, driving_published-eof_ns/1e9),
      'inputReadyTime': int(received*1e9), 'inferenceStartTime': int(inference_started*1e9),
      'inferenceEndTime': int(inference_ended*1e9), 'deadlineTime': int(max(0, self.budget.deadline)*1e9),
      'requiredTime': self.budget.estimate+.001, 'runs': self.budget.runs+int(reason == 'run'),
      'skipped': self.budget.skipped, 'overruns': self.budget.overruns,
      'cameraWidth': camera_size[0], 'cameraHeight': camera_size[1], 'detections': [],
      'submitTime': self.phases[0], 'readbackTime': self.phases[1], 'postprocessTime': self.phases[2],
    }
    # Only compact output crosses this bounded nonblocking CPU socket. The
    # primary owner immediately returns to receiving the next driving frame.
    self.output.send((metadata, values, transform, camera_size, self.names, start))
    self.last_publish = camera_time()
    if reason == 'run':
      self.budget.finish(start, self.last_publish)
