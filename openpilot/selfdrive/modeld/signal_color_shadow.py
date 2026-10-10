"""Opt-in camera signal comparison. Only logMessage output; never controls.

The learned classifier does NOT identify the applicable lane's traffic light.
Its intentionally unchanged blind search can select brake lamps/signs. Preserve
the selected region, score, frame identity and timing for subsequent review.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

DIRECTORY = Path('/data/signal-color-shadow')
MODEL_SHA256 = 'c091fad2b08d9a59e0e061413ec62ec181a77f371791d6f78e6a5bd61b148d06'
REFERENCE_SIZE = (1344, 760)
CLASSES = ('background', 'red', 'green')
MAX_AGE_MS = 1500


def requested(directory=DIRECTORY):
  try:
    return (Path(directory) / 'enabled').read_text().strip() == '1'
  except OSError:
    return False


def validate_artifact(directory=DIRECTORY):
  directory = Path(directory)
  meta = json.loads((directory / 'installed.json').read_text())
  if (meta.get('version') != 1 or meta.get('mode') != 'camera_comparison_only' or
      meta.get('model_sha256') != MODEL_SHA256 or meta.get('classes') != list(CLASSES)):
    raise ValueError('invalid signal-color comparison manifest')
  path = directory / 'signal_color_pool.onnx'
  if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
    raise ValueError('signal-color ONNX checksum mismatch')
  return path, meta


def window_boxes():
  width, height = REFERENCE_SIZE
  return [(x, y, x + w, y + h) for w, h in ((96, 48), (144, 72), (216, 108))
          for y in range(0, int(height * .6) - h + 1, h // 2)
          for x in range(int(width * .05), int(width * .95) - w + 1, w // 2)]


def rgb_from_nv12(data, width, height, stride, uv_offset):
  """Copy active padded NV12 planes, BT.601 limited-range -> RGB.

  This is independent of modeld warp/calibration and does not mutate IPC data.
  Raw camera versus encoded-video color differences require separate validation.
  """
  import numpy as np
  from PIL import Image
  if (width <= 0 or height <= 0 or width % 2 or height % 2 or stride < width or
      stride % 2 or uv_offset < stride * height):
    raise ValueError('invalid NV12 dimensions')
  raw = np.frombuffer(data, np.uint8)
  end = uv_offset + stride * (height // 2)
  if raw.size < end:
    raise ValueError('truncated NV12 buffer')
  y = raw[:height * stride].reshape(height, stride)[:, :width]
  uv = raw[uv_offset:end].reshape(height // 2, stride)[:, :width]
  u = uv[:, 0::2].repeat(2, axis=0).repeat(2, axis=1)
  v = uv[:, 1::2].repeat(2, axis=0).repeat(2, axis=1)
  # Use Pillow's native matrix loop rather than allocating many full-resolution
  # int32 intermediates on the little CPU cores. RGB here is just the 3-byte
  # input container; its channels hold Y, U, V until the matrix is applied.
  packed = Image.fromarray(np.stack((y, u, v), axis=2))
  matrix = (298 / 256, 0, 409 / 256, -(298 * 16 + 409 * 128) / 256,
            298 / 256, -100 / 256, -208 / 256, (-298 * 16 + 308 * 128) / 256,
            298 / 256, 516 / 256, 0, -(298 * 16 + 516 * 128) / 256)
  return np.asarray(packed.convert('RGB', matrix))


def select_candidates(probabilities, boxes, count=3):
  import numpy as np
  p = np.asarray(probabilities)
  if (p.shape != (len(boxes), 3) or not np.isfinite(p).all() or (p < 0).any() or
      (p > 1).any() or not np.allclose(p.sum(axis=1), 1, atol=1e-4)):
    raise ValueError('malformed color probabilities')
  scores = p[:, 1:].max(axis=1)
  order = np.argsort(-scores, kind='stable')
  selected = []
  for index in order:
    box = boxes[int(index)]
    # Preserve the strongest-window decision. Suppress overlapping runners-up
    # only in the diagnostic top-three list, never in the primary result.
    overlaps = False
    for old in selected:
      other = old['box_xyxy']
      area = max(0, min(box[2], other[2]) - max(box[0], other[0])) * max(0, min(box[3], other[3]) - max(box[1], other[1]))
      union = (box[2] - box[0]) * (box[3] - box[1]) + (other[2] - other[0]) * (other[3] - other[1]) - area
      overlaps |= area / union > .3
    if overlaps:
      continue
    color = CLASSES[1 + int(p[index, 1:].argmax())]
    selected.append({'box_xyxy': list(box), 'color': color, 'score': float(scores[index]),
                     'probabilities': p[index].astype(float).tolist()})
    if len(selected) >= count:
      break
  return selected


def scan_image(rgb_image, session):
  import numpy as np
  from PIL import Image
  image = Image.fromarray(rgb_image).resize(REFERENCE_SIZE, Image.Resampling.BILINEAR)
  boxes = window_boxes()
  predictions = []
  for start in range(0, len(boxes), 32):
    group = boxes[start:start + 32]
    # Transfer all resized patches to NumPy once per batch. Each patch is still
    # resized independently, preserving the recorded-image experiment exactly.
    atlas = Image.new('RGB', (96, 32 * len(group)))
    for index, box in enumerate(group):
      atlas.paste(image.crop(box).resize((96, 32), Image.Resampling.BILINEAR), (0, index * 32))
    batch = np.asarray(atlas, np.float32).reshape(len(group), 32, 96, 3).transpose(0, 3, 1, 2) / 255
    predictions.append(session.run(None, {'rgb': batch})[0])
  return select_candidates(np.concatenate(predictions), boxes)


def result_fields(candidates, age_ms):
  if not candidates or age_ms < 0 or age_ms > MAX_AGE_MS:
    return {'prediction': 'unknown', 'usable': False, 'reason': 'stale_frame'}
  strongest = candidates[0]
  accepted = strongest['score'] >= .8
  return {'prediction': strongest['color'] if accepted else 'unknown', 'usable': accepted,
          'reason': 'score_threshold' if not accepted else 'comparison_only_no_lane_association'}


def next_delay(work_wall, work_cpu):
  # At most 1 Hz, with <=25% of one CPU average process duty after startup.
  # A slow process never catches up on queued frames.
  return max(.05, 1.0 - work_wall, 3 * work_cpu)


def configure_worker_scheduling():
  # The manager's spawn launcher can create logging/IPC threads before main().
  # Linux affinity/nice are per-thread; include those existing threads too.
  allowed = os.sched_getaffinity(0) & {0, 1, 2, 3}
  if not allowed:
    raise RuntimeError('no permitted little CPU for signal observer')
  for task in Path('/proc/self/task').iterdir():
    try:
      tid = int(task.name)
      os.sched_setaffinity(tid, allowed)
      os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
      os.setpriority(os.PRIO_PROCESS, tid, 19)
    except ProcessLookupError:
      pass


def run(directory=DIRECTORY, duration=None):
  import sys
  import fcntl
  directory = Path(directory)
  if not requested(directory):
    return
  directory.mkdir(parents=True, exist_ok=True)
  with (directory / 'worker.lock').open('w') as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return
    configure_worker_scheduling()
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
      os.environ[name] = '1'
    sys.path.insert(0, '/data/signal-model-shadow/runtime')
    import onnxruntime as ort
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    from openpilot.common.swaglog import cloudlog
    path, meta = validate_artifact(directory)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.add_session_config_entry('session.intra_op.allow_spinning', '0')
    session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
    client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    start = time.monotonic()
    cloudlog.event('signalColorShadowLoaded', **meta, runtime=ort.__version__, pid=os.getpid(),
                   max_hz=1, target_cpu_duty=.25, stream='road', nv12='bt601_limited',
                   reference_width=1344, reference_height=760, candidates=667)
    previous_id = None
    while requested(directory) and (duration is None or time.monotonic() - start < duration):
      if not client.is_connected() and not client.connect(False):
        time.sleep(1)
        continue
      frame = client.recv(timeout_ms=100)
      if frame is None:
        time.sleep(.1)
        continue
      frame_id, eof = int(client.frame_id), int(client.timestamp_eof)
      if frame_id == previous_id:
        time.sleep(.05)
        continue
      previous_id = frame_id
      work_start, cpu_start = time.monotonic(), time.process_time()
      input_age = (time.monotonic_ns() - eof) / 1e6
      if input_age < 0 or input_age > 300:
        cloudlog.event('signalColorShadowSkipped', frame_id=frame_id, timestamp_eof=eof,
                       reason='stale_input', input_age_ms=input_age)
        time.sleep(1)
        continue
      rgb = rgb_from_nv12(frame.data, frame.width, frame.height, frame.stride, frame.uv_offset)
      if int(frame.frame_id) != frame_id:
        cloudlog.event('signalColorShadowSkipped', frame_id=frame_id, timestamp_eof=eof,
                       reason='buffer_reused_during_copy')
        time.sleep(1)
        continue
      candidates = scan_image(rgb, session)
      elapsed, cpu = time.monotonic() - work_start, time.process_time() - cpu_start
      age = (time.monotonic_ns() - eof) / 1e6
      record = {'mode': 'camera_comparison_only', 'model_sha256': MODEL_SHA256, 'stream': 'road',
                'frame_id': frame_id, 'timestamp_eof': eof, 'recorded_monotonic_ns': time.monotonic_ns(),
                'source_width': int(frame.width), 'source_height': int(frame.height),
                'reference_width': 1344, 'reference_height': 760, 'input_age_ms': input_age,
                'result_age_ms': age, 'work_ms': elapsed * 1000, 'cpu_ms': cpu * 1000,
                'next_delay_s': next_delay(elapsed, cpu), 'candidates': candidates,
                **result_fields(candidates, age)}
      cloudlog.event('signalColorShadow', **record)
      temporary = directory / 'latest.tmp'
      temporary.write_text(json.dumps(record))
      temporary.replace(directory / 'latest.json')
      time.sleep(next_delay(elapsed, cpu))


def main():
  # Manager calls main without CLI arguments. Latch ordinary failures until a
  # disable/ignition restart; manager otherwise re-launches exited processes.
  try:
    run()
  except Exception:
    from openpilot.common.swaglog import cloudlog
    cloudlog.exception('signalColorShadowDisabled; original control unchanged')
  while requested():
    time.sleep(1)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Read-only live camera signal comparison')
  parser.add_argument('mode', choices=('on', 'off', 'status', 'run'))
  parser.add_argument('--seconds', type=float)
  args = parser.parse_args()
  if args.mode in ('on', 'off'):
    if args.mode == 'on':
      validate_artifact()
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    temp = DIRECTORY / 'enabled.tmp'
    temp.write_text('1' if args.mode == 'on' else '0')
    temp.replace(DIRECTORY / 'enabled')
  if args.mode == 'run':
    run(duration=args.seconds)
  else:
    print(json.dumps({'requested': requested(), 'control': 'original',
                      'latest': json.loads((DIRECTORY / 'latest.json').read_text()) if (DIRECTORY / 'latest.json').exists() else None}))
