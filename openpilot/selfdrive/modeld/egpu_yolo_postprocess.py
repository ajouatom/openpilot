"""Bounded CPU-only delivery for the driving owner's resident YOLO output."""
import os
import errno
import pickle
import socket
import subprocess
import sys
import threading
import time


def cpu_worker_scheduler():
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  try:
    os.sched_setaffinity(0, {4})
  except OSError as exc:
    # Offroad power saving may offline the entire big CPU cluster. A decoder
    # test/restart must remain CPU-only and usable until that cluster returns.
    if exc.errno != errno.EINVAL:
      raise
    os.sched_setaffinity(0, {0, 1, 2, 3})


class OutputWorker:
  def __init__(self, *, recover=False):
    self.recover = recover
    self.lock = threading.Lock()
    self._spawn()
    if recover:
      threading.Thread(target=self._watch, name='yolo-cpu-watch', daemon=True).start()

  def _spawn(self):
    self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    self.socket.setblocking(False)
    self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 131072)
    try:
      self.process = subprocess.Popen([sys.executable, '-m', __name__, str(child.fileno())], pass_fds=(child.fileno(),),
                                      stdin=subprocess.DEVNULL)
    finally:
      child.close()

  def _watch(self):
    # Only the CPU decoder is replaced. Never fork/reload the GPU owner from a
    # recovery attempt, and never make the primary thread wait for this lock.
    cpu_worker_scheduler()
    failures = 0
    healthy_since = time.monotonic()
    while True:
      time.sleep(.5)
      if self.process.poll() is None:
        if time.monotonic()-healthy_since >= 60:
          failures = 0
        continue
      time.sleep((2, 5, 10, 30)[min(failures, 3)])
      with self.lock:
        self.socket.close()
        try:
          self._spawn()
        except OSError:
          pass
      failures += 1
      healthy_since = time.monotonic()

  def send(self, packet):
    if not self.lock.acquire(blocking=False):
      return False
    try:
      return self._send(packet)
    finally:
      self.lock.release()

  def ready(self):
    return not self.lock.locked() and self.process.poll() is None

  def _send(self, packet):
    if self.process.poll() is not None:
      if self.recover:
        return False
      raise RuntimeError('CPU-only YOLO output worker exited')
    data = pickle.dumps(packet, protocol=5)
    if len(data) > 131072:
      raise ValueError('YOLO output packet exceeds bound')
    try:
      return self.socket.send(data) == len(data)
    except BlockingIOError:
      return False
    except (BrokenPipeError, ConnectionResetError):
      if self.recover:
        return False
      raise


def project_detections(detections, transform, model_size, camera_size):
  """Batch corner projection for crowded frames; retain the small-frame path."""
  import numpy as np
  from openpilot.selfdrive.modeld.egpu_yolo import camera_detections
  if len(detections) < 2:
    return camera_detections(detections, transform, model_size, camera_size)
  if np.shape(transform) != (3, 3) or not np.isfinite(transform).all() or min(*model_size, *camera_size) <= 0:
    return []
  mw, mh = model_size
  boxes = np.array([[d[k] for k in ('x1', 'y1', 'x2', 'y2')] for d in detections])
  corners = np.ones((len(detections), 4, 3))
  corners[:, :, 0] = boxes[:, [0, 2, 2, 0]]*mw
  corners[:, :, 1] = boxes[:, [1, 1, 3, 3]]*mh
  corners = corners @ transform.T
  valid = np.isfinite(corners).all(axis=(1, 2)) & (corners[:, :, 2] > 1e-6).all(axis=1)
  indices = np.flatnonzero(valid)
  normalized = corners[indices, :, :2]/corners[indices, :, 2:]/np.asarray(camera_size)
  return [{**detections[index], 'cameraPoints': points.flatten().tolist()} for index, points in zip(indices, normalized, strict=True)
          if np.max(np.abs(points)) <= 10]


def decode_packet(packet):
  import numpy as np
  from openpilot.selfdrive.modeld.egpu_yolo import decode_detections, camera_time
  metadata, values, transform, size, names, started = packet
  if values is None:
    return metadata
  if values.shape != (1, 6, 2688) or values.dtype not in (np.float32, np.float16):
    raise ValueError('native compact FP32 or FP16 output required')
  cpu_start = camera_time()
  # Keep transport compact, but run filtering/NMS in FP32 on this CPU worker.
  signal_model = metadata.get('modelId') == 'signal-v33-observe-s260911' and list(names) == ['red_visible', 'green_visible']
  detections = decode_detections(values.astype(np.float32, copy=False), 512, 256,
                                 confidence=.25 if signal_model else .35, compact=True)
  for detection in detections:
    detection['label'] = names[detection['classId']]
  metadata['detections'] = project_detections(detections, np.asarray(transform), (512, 256), size)
  metadata['postprocessTime'] = camera_time()-cpu_start
  metadata['executionTime'] = camera_time()-started
  return metadata


def main():
  # Reset inherited FIFO/CPU-7 policy before importing NumPy or cereal. This
  # process never imports tinygrad, opens an image buffer or owns a GPU device.
  cpu_worker_scheduler()
  from openpilot.cereal import messaging
  from openpilot.selfdrive.modeld.egpu_yolo import camera_time
  from openpilot.selfdrive.modeld.egpu_yolo_reuse import read_session
  stream = socket.socket(fileno=int(sys.argv[1]))
  pm = messaging.PubMaster(['carrotYolo'])
  while True:
    data = stream.recv(131073)
    if not data:
      return
    # Keep only the newest complete packet when CPU processing falls behind.
    while True:
      try:
        newer = stream.recv(131073, socket.MSG_DONTWAIT)
      except BlockingIOError:
        break
      if not newer:
        return
      data = newer
    if len(data) > 131072 or not read_session(enabled=False):
      continue
    # Only this owner's inherited private socket can supply pickle data.
    packet = pickle.loads(data)
    if camera_time()-packet[-1] > .25:
      continue
    try:
      metadata = decode_packet(packet)
    except Exception:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception('CPU YOLO postprocessing failed')
      return
    if camera_time()-packet[-1] > .25 or not read_session(enabled=False):
      continue
    msg = messaging.new_message('carrotYolo')
    msg.valid = metadata['state'] == 'run'
    msg.carrotYolo = metadata
    pm.send('carrotYolo', msg)


if __name__ == '__main__':
  main()
