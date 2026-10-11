"""Opt-in road-camera observer; control consumes it only with separate assist opt-in."""
import hashlib
import json
import os
from pathlib import Path
import time

from openpilot.selfdrive.modeld.signal_color_shadow import DIRECTORY, configure_worker_scheduling, requested

REFERENCE_SIZE = (1344, 760)
MAX_INPUT_AGE_MS = 150
MAX_RESULT_AGE_MS = 200


def tracking_requested(directory=DIRECTORY):
  try:
    return (Path(directory) / 'tracking_enabled').read_text().strip() == '1'
  except OSError:
    return False


def comparison_requested(directory=DIRECTORY):
  try:
    return (Path(directory) / 'daytime_comparison_enabled').read_text().strip() == '1'
  except OSError:
    return False


def new_trackers(factory, comparison):
  legacy = factory()
  daytime = factory(daytime_cores=True) if comparison else None
  return legacy, daytime


def publish_control_observation(result, frame_id, timestamp, session, *, comparison, publisher):
  # Comparison sessions never publish either engine to the control transport,
  # even if assist_enabled is accidentally re-enabled separately.
  if not comparison:
    publisher(result, frame_id, timestamp, session)


def next_delay(wall, cpu):
  # At most 20 Hz and a target <=50% of one CPU, without catching up.
  return max(0., .05 - wall, cpu)


def copy_nv12_rgb(frame):
  import cv2
  import numpy as np
  width, height, stride, offset = int(frame.width), int(frame.height), int(frame.stride), int(frame.uv_offset)
  if (width, height) != REFERENCE_SIZE or stride < width or stride % 2 or offset < stride * height:
    raise ValueError('unsupported tracking camera geometry')
  raw = np.frombuffer(frame.data, np.uint8)
  end = offset + stride * (height // 2)
  if raw.size < end:
    raise ValueError('truncated tracking camera buffer')
  packed = np.empty((height * 3 // 2, width), np.uint8)
  packed[:height] = raw[:height * stride].reshape(height, stride)[:, :width]
  packed[height:] = raw[offset:end].reshape(height // 2, stride)[:, :width]
  return cv2.cvtColor(packed, cv2.COLOR_YUV2RGB_NV12)


def result_fields(result, age_ms):
  fresh = 0 <= age_ms <= MAX_RESULT_AGE_MS
  return {'prediction': result['state'] if fresh else 'unknown', 'fresh': fresh,
          'reason': result['reason'] if fresh else 'stale_result', 'control_permission': False}


def run(directory=DIRECTORY, duration=None):
  import fcntl
  directory = Path(directory)
  if not requested(directory) or not tracking_requested(directory):
    return
  with (directory / 'worker.lock').open('w') as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return
    configure_worker_scheduling()
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
      os.environ[name] = '1'
    import cv2
    from tools.signal_analysis import signal_tracker  # noqa: TID251 - repository-root experiment, not openpilot/tools
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    from openpilot.common.swaglog import cloudlog
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    configure_worker_scheduling()
    source_sha = hashlib.sha256(Path(signal_tracker.__file__).read_bytes()).hexdigest()
    from openpilot.selfdrive.carrot.signal_assist_runtime import publish_observation
    comparison = comparison_requested(directory)
    def new_tracker():
      return (*new_trackers(signal_tracker.SignalTracker, comparison), f'{os.getpid()}:{time.monotonic_ns()}')
    tracker, daytime_tracker, session_id = new_tracker()
    client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    start = time.monotonic()
    cloudlog.event('signalTrackingShadowLoaded', mode='tracking_observation', algorithm_sha256=source_sha,
                   runtime=cv2.__version__, pid=os.getpid(), max_hz=20, target_cpu_duty=.5,
                   stream='road', reference_width=1344, reference_height=760, control_permission=False,
                   assist_transport=not comparison, daytime_comparison=comparison)
    previous_id = None
    last_file = 0.
    while (requested(directory) and tracking_requested(directory) and comparison_requested(directory) == comparison
           and (duration is None or time.monotonic() - start < duration)):
      if not client.is_connected() and not client.connect(False):
        tracker, daytime_tracker, session_id = new_tracker()
        time.sleep(.2)
        continue
      frame = client.recv(timeout_ms=100)
      if frame is None:
        tracker, daytime_tracker, session_id = new_tracker()
        continue
      frame_id, eof = int(client.frame_id), int(client.timestamp_eof)
      if frame_id == previous_id:
        time.sleep(.005)
        continue
      gap = None if previous_id is None else frame_id - previous_id
      if gap is not None and gap <= 0:
        tracker, daytime_tracker, session_id = new_tracker()
      previous_id = frame_id
      work_start, cpu_start = time.monotonic(), time.process_time()
      input_age = (time.monotonic_ns() - eof) / 1e6
      if not 0 <= input_age <= MAX_INPUT_AGE_MS:
        tracker, daytime_tracker, session_id = new_tracker()
        cloudlog.event('signalTrackingShadowSkipped', frame_id=frame_id, timestamp_eof=eof,
                       reason='stale_input', input_age_ms=input_age)
        time.sleep(.05)
        continue
      rgb = copy_nv12_rgb(frame)
      if int(frame.frame_id) != frame_id:
        tracker, daytime_tracker, session_id = new_tracker()
        cloudlog.event('signalTrackingShadowSkipped', frame_id=frame_id, timestamp_eof=eof,
                       reason='buffer_reused_during_copy')
        time.sleep(.05)
        continue
      result = tracker.process(rgb, eof / 1e9)
      age = (time.monotonic_ns() - eof) / 1e6
      record = {'mode': 'tracking_observation', 'algorithm_sha256': source_sha, 'stream': 'road',
                'frame_id': frame_id, 'frame_gap': gap, 'timestamp_eof': eof,
                'recorded_monotonic_ns': time.monotonic_ns(), 'reference_width': 1344, 'reference_height': 760,
                'input_age_ms': input_age, 'result_age_ms': age,
                'work_ms': (time.monotonic() - work_start) * 1000,
                'cpu_ms': (time.process_time() - cpu_start) * 1000,
                'tracks': result['tracks'], 'daytime_comparison': comparison,
                'assist_transport': not comparison, 'session': session_id, **result_fields(result, age)}
      try:
        publish_control_observation(result, frame_id, eof / 1e9, session_id,
                                    comparison=comparison, publisher=publish_observation)
      except (OSError, ValueError):
        cloudlog.exception('signalObservationPublishFailed')
      cloudlog.event('signalTrackingShadow', **record)
      comparison_record = None
      if daytime_tracker is not None:
        candidate_start, candidate_cpu = time.monotonic(), time.process_time()
        candidate = daytime_tracker.process(rgb, eof / 1e9)
        candidate_age = (time.monotonic_ns() - eof) / 1e6
        comparison_record = dict(record, mode='daytime_comparison', tracks=candidate['tracks'],
                                 recorded_monotonic_ns=time.monotonic_ns(), result_age_ms=candidate_age,
                                 work_ms=(time.monotonic() - candidate_start) * 1000,
                                 cpu_ms=(time.process_time() - candidate_cpu) * 1000,
                                 pair_work_ms=(time.monotonic() - work_start) * 1000,
                                 baseline_prediction=record['prediction'],
                                 **result_fields(candidate, candidate_age))
        cloudlog.event('signalTrackingDaytimeShadow', **comparison_record)
      if time.monotonic() - last_file >= 1:
        temp = directory / 'tracking_latest.tmp'
        temp.write_text(json.dumps(record))
        temp.replace(directory / 'tracking_latest.json')
        if comparison_record is not None:
          temp = directory / 'daytime_comparison_latest.tmp'
          temp.write_text(json.dumps(comparison_record))
          temp.replace(directory / 'daytime_comparison_latest.json')
        last_file = time.monotonic()
      # A late result is unusable by the consumer's unchanged 200 ms age gate.
      # Do not erase object identity merely because computation ran late: the
      # next camera timestamp still has to pass the tracker's bounded gap and
      # geometric matching. Reconnects, bad input and reordered frames reset it.
      time.sleep(next_delay(time.monotonic() - work_start, time.process_time() - cpu_start))
