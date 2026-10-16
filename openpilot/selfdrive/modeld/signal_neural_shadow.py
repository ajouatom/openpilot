"""Opt-in neural observer with a separately enabled stopped-car veto transport."""
import hashlib
import json
import os
from pathlib import Path
import time

from openpilot.selfdrive.modeld.signal_color_shadow import DIRECTORY, requested, configure_worker_scheduling


def neural_requested(directory=DIRECTORY):
  try:
    return (Path(directory) / 'neural_enabled').read_text().strip() == '1'
  except OSError:
    return False


def backend_requested(directory=DIRECTORY):
  try:
    backend = (Path(directory) / 'neural_backend').read_text().strip()
  except FileNotFoundError:
    backend = 'cpu'
  if backend not in ('cpu', 'qcom', 'auto'):
    raise ValueError('invalid neural backend selection')
  return backend


def run_selected(directory, backend):
  if backend == 'qcom':
    if any(not (Path(directory) / 'neural' / (name + '.qcom' + suffix)).is_file()
           for name in ('detector', 'classifier') for suffix in ('.pkl', '.json')):
      raise RuntimeError('auto neural mode requires locally validated QCOM caches')
    run(directory, backend_override='qcom')
  else:
    from openpilot.selfdrive.modeld.signal_tracking_shadow import run as run_tracking
    run_tracking(directory, comparison_override=True)


def automatic_backend(params):
  from openpilot.selfdrive.carrot.signal_mode import read_mode
  if read_mode(params) == 2:
    # Preparation requires the measured-housing CPU evidence contract.
    return 'tracking_cpu'
  # The DM model also runs on the internal GPU with an external driving model.
  # Parked A/B/A showed ~20 -> 10 Hz DM even at QCOM priority 15. Camera failure
  # must never grant GPU access: that would keep the overloaded DM in fallback.
  dm_needed = (params.get_bool('DriverMonitoringEnabled') and
               not params.get_bool('DriverMonitoringSessionDisabled')) or params.get_bool('IsDriverViewEnabled')
  return 'qcom' if params.get_bool('UsbGpuActive') and not dm_needed else 'tracking_cpu'


def run_auto(directory=DIRECTORY, duration=None):
  """Keep QCOM neural work off the GPU needed by the driving or DM model.

  A process boundary releases all GPU buffers on a mode change. The CPU mode
  deliberately uses the existing lightweight camera tracker, not CPU YOLOX.
  """
  import multiprocessing
  import signal
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  params = Params()
  selection = backend_requested(directory)
  child, selected = None, None
  start = time.monotonic()

  def stop_child():
    nonlocal child
    if child is not None:
      if child.is_alive():
        child.terminate()
      child.join(2)
      if child.is_alive():
        child.kill()
        child.join(2)
      if child.is_alive():
        raise RuntimeError('signal backend did not stop')
      child.close()
      child = None

  def terminate(_signum, _frame):
    raise SystemExit(0)

  previous = signal.signal(signal.SIGTERM, terminate)
  try:
    while (requested(directory) and neural_requested(directory) and backend_requested(directory) == selection
           and (duration is None or time.monotonic()-start < duration)):
      mode = automatic_backend(params)
      if mode != selected:
        stop_child()
        child = multiprocessing.get_context('spawn').Process(target=run_selected, args=(directory, mode), daemon=True)
        child.start()
        selected = mode
        cloudlog.event('signalAutoBackend', backend=mode, pid=child.pid, control_permission=False)
      elif child.exitcode is not None:
        raise RuntimeError(f'signal backend {selected} exited: {child.exitcode}')
      time.sleep(.1)
  finally:
    try:
      stop_child()
    finally:
      signal.signal(signal.SIGTERM, previous)


def display_tracks(tracks):
  result = []
  for t in tracks[:20]:
    # Mixed red/arrow labels retain red for visualization. This does not decide
    # which direction or lane is allowed. Yellow remains explicitly unknown.
    labels = t['label'].split(',')
    raw = ('red' if 'red' in labels else 'green' if labels == ['green'] else 'unknown') if t['score'] >= .5 else 'unknown'
    result.append(dict(id=t['id'], box=t['box'], age=0, state='unknown',
                       evidence=dict(raw=raw), label=t['label'], score=t['score'],
                       source=t['source'], neural_age=t['neural_age']))
  return result


def run(directory=DIRECTORY, duration=None, backend_override=None):
  import sys
  directory = Path(directory)
  if not requested(directory) or not neural_requested(directory):
    return
  selection = backend_requested(directory)
  if selection in ('auto', 'qcom') and backend_override is None:
    return run_auto(directory, duration)
  import fcntl
  with (directory / 'worker.lock').open('w') as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return
    configure_worker_scheduling()
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
      os.environ[name] = '1'
    sys.path.insert(0, '/data/signal-model-shadow/runtime')
    import cv2
    import numpy as np
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    from openpilot.common.swaglog import cloudlog
    from tools.signal_analysis import neural_signal
    from openpilot.selfdrive.modeld.signal_tracking_shadow import copy_nv12_rgb, next_delay, MAX_INPUT_AGE_MS
    from openpilot.selfdrive.carrot.signal_display import display_requested, publish_display
    from openpilot.selfdrive.carrot.signal_start_guard import publish_start_observation
    from openpilot.selfdrive.carrot.signal_assist_runtime import requested as flag_requested
    from openpilot.selfdrive.carrot.lead_brake_lights import BrakeRuntime
    brake_observer = BrakeRuntime()
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    backend = backend_override or selection
    model_dir = directory / 'neural'
    network = neural_signal.NeuralSignal(model_dir / 'detector.onnx', model_dir / 'classifier.onnx', backend=backend)
    # Compile/capture before subscribing: startup cannot create a camera backlog.
    for _ in range(3):
      network.detector.run(None, {'images': np.zeros((1, 3, 416, 416), np.float32)})
      network.classifier.run(None, {'input': np.zeros((1, 3, 224, 224), np.float32)})
    configure_worker_scheduling()
    source_sha = hashlib.sha256(Path(neural_signal.__file__).read_bytes()).hexdigest()
    def reset():
      return neural_signal.NeuralSignalSequence(network), f'neural:{os.getpid()}:{time.monotonic_ns()}'
    sequence, session = reset()
    client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    start = time.monotonic()
    cloudlog.event('signalNeuralShadowLoaded', backend=backend, algorithm_sha256=source_sha,
                   detector_sha256=network.DETECTOR_SHA, classifier_sha256=network.CLASSIFIER_SHA,
                   pid=os.getpid(), control_permission=False, assist_transport=False)
    previous_id, last_file = None, 0.
    from openpilot.selfdrive.carrot.signal_mode import signal_mode
    selected_mode = signal_mode() if directory == DIRECTORY else None
    while (requested(directory) and neural_requested(directory) and backend_requested(directory) == selection
           and (selected_mode is None or signal_mode() == selected_mode)
           and (duration is None or time.monotonic()-start < duration)):
      if not client.is_connected() and not client.connect(False):
        sequence, session = reset(); time.sleep(.2); continue
      frame = client.recv(timeout_ms=100)
      if frame is None:
        sequence, session = reset(); continue
      fid, eof = int(client.frame_id), int(client.timestamp_eof)
      if fid == previous_id:
        time.sleep(.005); continue
      if previous_id is not None and fid < previous_id:
        sequence, session = reset()
      gap = None if previous_id is None else fid-previous_id
      previous_id = fid
      begin, cpu = time.monotonic(), time.process_time()
      age = (time.monotonic_ns()-eof)/1e6
      if not 0 <= age <= MAX_INPUT_AGE_MS:
        sequence, session = reset(); time.sleep(.05); continue
      rgb = copy_nv12_rgb(frame)
      if int(frame.frame_id) != fid:
        sequence, session = reset(); continue
      result = sequence.process(rgb, eof/1e9)
      record = dict(mode='neural_display_only', neural=True, backend=backend, algorithm_sha256=source_sha,
                    stream='road', reference_width=1344, reference_height=760, frame_id=fid,
                    frame_gap=gap, timestamp_eof=eof, session=session, detected=result['detected'],
                    input_age_ms=age, result_age_ms=(time.monotonic_ns()-eof)/1e6,
                    recorded_monotonic_ns=time.monotonic_ns(), work_ms=(time.monotonic()-begin)*1000,
                    cpu_ms=(time.process_time()-cpu)*1000, tracks=display_tracks(result['tracks']),
                    start_guard_transport=flag_requested(directory / 'start_guard_enabled'),
                    control_permission=False, assist_transport=False)
      cloudlog.event('signalNeuralShadow', **record)
      try:
        publish_start_observation(record, directory)
      except (OSError, ValueError, TypeError, KeyError):
        cloudlog.exception('signalStartObservationPublishFailed')
      if display_requested(directory / 'display_enabled'):
        publish_display(record)
      if time.monotonic()-last_file >= 1:
        tmp = directory / 'neural_latest.tmp'
        tmp.write_text(json.dumps(record, allow_nan=False)); tmp.replace(directory / 'neural_latest.json')
        last_file = time.monotonic()
      try:
        brake_observer.update(rgb, fid, eof/1e9, session, begin, cloudlog.event)
      except Exception:
        cloudlog.exception('leadBrakeLightsFailed')
      time.sleep(next_delay(time.monotonic()-begin, time.process_time()-cpu))
