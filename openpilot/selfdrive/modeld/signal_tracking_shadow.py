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
    def new_tracker():
      return signal_tracker.SignalTracker(), f'{os.getpid()}:{time.monotonic_ns()}'
    tracker, session_id = new_tracker()
    client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    start = time.monotonic()
    cloudlog.event('signalTrackingShadowLoaded', mode='tracking_observation', algorithm_sha256=source_sha,
                   runtime=cv2.__version__, pid=os.getpid(), max_hz=20, target_cpu_duty=.5,
                   stream='road', reference_width=1344, reference_height=760, control_permission=False, assist_transport=True)
    previous_id = None
    last_file = 0.
    while requested(directory) and tracking_requested(directory) and (duration is None or time.monotonic() - start < duration):
      if not client.is_connected() and not client.connect(False):
        tracker, session_id = new_tracker()
        time.sleep(.2)
        continue
      frame = client.recv(timeout_ms=100)
      if frame is None:
        tracker, session_id = new_tracker()
        continue
      frame_id, eof = int(client.frame_id), int(client.timestamp_eof)
      if frame_id == previous_id:
        time.sleep(.005)
        continue
      gap = None if previous_id is None else frame_id - previous_id
      if gap is not None and gap <= 0:
        tracker, session_id = new_tracker()
      previous_id = frame_id
      work_start, cpu_start = time.monotonic(), time.process_time()
      input_age = (time.monotonic_ns() - eof) / 1e6
      if not 0 <= input_age <= MAX_INPUT_AGE_MS:
        tracker, session_id = new_tracker()
        cloudlog.event('signalTrackingShadowSkipped', frame_id=frame_id, timestamp_eof=eof,
                       reason='stale_input', input_age_ms=input_age)
        time.sleep(.05)
        continue
      rgb = copy_nv12_rgb(frame)
      if int(frame.frame_id) != frame_id:
        tracker, session_id = new_tracker()
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
                'tracks': result['tracks'], **result_fields(result, age)}
      try:
        publish_observation(result, frame_id, eof / 1e9, session_id)
      except (OSError, ValueError):
        cloudlog.exception('signalObservationPublishFailed')
      cloudlog.event('signalTrackingShadow', **record)
      if time.monotonic() - last_file >= 1:
        temp = directory / 'tracking_latest.tmp'
        temp.write_text(json.dumps(record))
        temp.replace(directory / 'tracking_latest.json')
        last_file = time.monotonic()
      if not record['fresh']:
        tracker, session_id = new_tracker()
      time.sleep(next_delay(time.monotonic() - work_start, time.process_time() - cpu_start))
