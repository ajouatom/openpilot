"""Opt-in CPU lamp observations, independent of the driving GPU owner."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from openpilot.selfdrive.modeld.egpu_yolo import camera_detections, camera_time, decode_detections

MODEL_ID = 'signal-v33-observe-int8-s260911'
MODEL_SHA256 = '297684217729517c1bd9118d3ec12c966418d5a0d61b31961242fdd8cfebecd7'
INTERVAL = 1 / 3


def directory():
  return Path(os.getenv('EGPU_YOLO_DIR', '/data/egpu_yolo'))


def configured():
  root = directory()
  try:
    config = json.loads((root/'signal_cpu.json').read_text())
    auto = json.loads((root/'auto_enabled.json').read_text()) if (root/'auto_enabled.json').exists() else {}
    return (config.get('enabled') is True and config.get('model_id') == MODEL_ID
            and auto.get('enabled') is not True and not (root/'qcom_enabled').exists()
            and (root/'signal-v36-cpu-int8/model.onnx').is_file()
            and (root/'signal-v35-observation/ort-deps/onnxruntime').is_dir())
  except (OSError, ValueError, TypeError, AttributeError):
    return False


def road_rgb(frame, *, width, height, stride, uv_offset, transform):
  """CPU reconstruction of the calibrated native driving road crop."""
  matrix = np.asarray(transform, dtype=np.float32)
  if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or min(width, height) <= 0 or stride < width:
    raise ValueError('invalid road geometry')
  values = np.asarray(frame, dtype=np.uint8)
  if values.ndim != 1 or values.size < uv_offset + stride*(height//2):
    raise ValueError('incomplete NV12 frame')

  def warp(plane, mat, w, h):
    x, y = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    denom = mat[2, 0]*x + mat[2, 1]*y + mat[2, 2]
    if not np.isfinite(denom).all() or (denom <= 1e-6).any():
      raise ValueError('invalid road projection depth')
    sx = np.rint((mat[0, 0]*x+mat[0, 1]*y+mat[0, 2])/denom).astype(int).clip(0, plane.shape[1]-1)
    sy = np.rint((mat[1, 0]*x+mat[1, 1]*y+mat[1, 2])/denom).astype(int).clip(0, plane.shape[0]-1)
    return plane[sy, sx]

  y = warp(values[:stride*height].reshape(height, stride)[:, :width], matrix, 512, 256).astype(np.float32)/255
  uv = values[uv_offset:uv_offset+stride*(height//2)].reshape(height//2, stride)[:, :width]
  uv_matrix = matrix*np.array([[1, 1, .5], [1, 1, .5], [2, 2, 1]], dtype=np.float32)
  u, v = [warp(uv[:, channel::2], uv_matrix, 256, 128).repeat(2, 0).repeat(2, 1).astype(np.float32)/255-.5
          for channel in (0, 1)]
  return np.stack([y+1.402*v, y-.344*u-.714*v, y+1.772*u]).clip(0, 1)[None].astype(np.float32)


def main():
  if not configured():
    return
  # No USB GPU, QCOM context, or driving-owner queue is opened by this process.
  os.sched_setaffinity(0, {4, 5})
  os.environ['OMP_NUM_THREADS'] = '1'
  os.environ['OPENBLAS_NUM_THREADS'] = '1'
  sys.path.insert(0, str(directory()/'signal-v35-observation/ort-deps'))
  import onnxruntime as ort
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.common.transformations.model import get_warp_matrix
  from msgq.visionipc import VisionIpcClient, VisionStreamType

  source = directory()/'signal-v36-cpu-int8/model.onnx'
  if hashlib.sha256(source.read_bytes()).hexdigest() != MODEL_SHA256:
    raise ValueError('signal model checksum mismatch')
  options = ort.SessionOptions()
  options.intra_op_num_threads, options.inter_op_num_threads = 2, 1
  options.add_session_config_entry('session.intra_op.allow_spinning', '0')
  session = ort.InferenceSession(str(source), sess_options=options, providers=['CPUExecutionProvider'])
  if session.get_inputs()[0].shape != [1, 3, 256, 512]:
    raise ValueError('fixed road input required')
  params = Params()
  services = ['carState', 'selfdriveState', 'deviceState', 'modelV2', 'roadCameraState', 'liveCalibration']
  sm = messaging.SubMaster(services, frequency=3)
  pm = messaging.PubMaster(['carrotYolo'])
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
  if not client.connect(True):
    return
  runs = skipped = 0
  last_frame = -1

  def healthy():
    sm.update(0)
    now = time.monotonic()
    return (sm.all_alive(services) and sm.all_valid(services) and sm['deviceState'].started
            and all(now-sm.recv_time[s] < .3 for s in ['carState', 'selfdriveState', 'modelV2', 'roadCameraState'])
            and math.isfinite(sm['modelV2'].modelExecutionTime) and 0 < sm['modelV2'].modelExecutionTime < .06
            and math.isfinite(sm['modelV2'].frameDropPerc) and sm['modelV2'].frameDropPerc <= 1)

  while configured():
    start = time.monotonic()
    values = {'modelId': MODEL_ID, 'camera': 'road', 'state': 'paused', 'runs': runs, 'skipped': skipped, 'detections': []}
    if healthy():
      buffer = client.recv(timeout_ms=100)
      if buffer is not None and client.frame_id != last_frame:
        last_frame = client.frame_id
        eof, sof = client.timestamp_eof, client.timestamp_sof
        if 0 <= camera_time()-eof/1e9 <= .1:
          calibration = np.array(sm['liveCalibration'].rpyCalib, dtype=np.float32)
          if str(sm['liveCalibration'].calStatus) == 'calibrated':
            calibration[2] -= np.radians(params.get_float('CameraYawTrimDeg')*.01)
          camera = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
          transform = get_warp_matrix(calibration, camera.fcam.intrinsics, False).astype(np.float32)
          image = road_rgb(np.frombuffer(buffer.data, dtype=np.uint8).copy(), width=buffer.width, height=buffer.height,
                           stride=buffer.stride, uv_offset=buffer.uv_offset, transform=transform)
          infer_start = time.monotonic()
          raw, = session.run(None, {session.get_inputs()[0].name: image})
          elapsed = time.monotonic()-infer_start
          if healthy() and configured() and 0 <= camera_time()-eof/1e9 <= .35:
            detections = decode_detections(raw, 512, 256, confidence=.25)
            for d in detections:
              d['label'] = ('red_visible', 'green_visible')[d['classId']]
            runs += 1
            values.update(state='run', runs=runs, frameId=last_frame, timestampSof=sof, timestampEof=eof,
                          executionTime=elapsed, cameraWidth=buffer.width, cameraHeight=buffer.height,
                          detections=camera_detections(detections, transform, (512, 256), (buffer.width, buffer.height)))
    if values['state'] != 'run':
      skipped += 1
      values['skipped'] = skipped
    msg = messaging.new_message('carrotYolo')
    msg.valid = values['state'] == 'run'
    msg.carrotYolo = values
    pm.send('carrotYolo', msg)
    time.sleep(max(0, INTERVAL-(time.monotonic()-start)))


if __name__ == '__main__':
  main()
