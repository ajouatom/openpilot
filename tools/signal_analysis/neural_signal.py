"""Neural signal/crop inference shared by offline tests and opt-in visualization.

Reference artifacts and interpretation limits: docs/signal_detection_strategy_20261011.md.
Boxes and lamp classes do not establish which signal applies to the ego vehicle.
"""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from tools.signal_analysis.signal_tracker import overlap


def copy_nv12_crop_rgb(frame, rect):
  """Copy only an even-aligned rectangle of padded NV12; retain original coordinates.

  Caller owns frame synchronization and must check for buffer reuse after copying.
  This does not expose camerad's full-resolution placeholder or alter its output.
  """
  width, height, stride, offset = map(int, (frame.width, frame.height, frame.stride, frame.uv_offset))
  x1, y1, x2, y2 = rect
  if (any(type(v) is not int or v % 2 for v in rect) or not 0 <= x1 < x2 <= width
      or not 0 <= y1 < y2 <= height or width % 2 or height % 2 or stride < width
      or stride % 2 or offset < stride * height):
    raise ValueError('invalid NV12 crop geometry')
  raw = np.frombuffer(frame.data, np.uint8)
  end = offset + stride * (height // 2)
  if raw.size < end:
    raise ValueError('truncated NV12 buffer')
  w, h = x2-x1, y2-y1
  packed = np.empty((h*3//2, w), np.uint8)
  packed[:h] = raw[:stride*height].reshape(height, stride)[y1:y2, x1:x2]
  packed[h:] = raw[offset:end].reshape(height//2, stride)[y1//2:y2//2, x1:x2]
  return cv2.cvtColor(packed, cv2.COLOR_YUV2RGB_NV12)


class QcomSession:
  """Explicit QCOM execution with fixed-shape JIT and synchronous output copies.

  Own this in a dedicated process: tinygrad's default device is process-global.
  No automatic CPU fallback can conceal a failed GPU measurement.
  """
  def __init__(self, path):
    import os
    os.environ.update(QCOM_FLAGS)
    from tinygrad import Tensor, Device, TinyJit
    from tinygrad.nn.onnx import OnnxRunner
    if Device.DEFAULT != 'QCOM':
      raise RuntimeError('QCOM session requires a dedicated QCOM process')
    self.Tensor = Tensor
    self.inputs = None
    compiled = Path(path).with_suffix('.qcom.pkl')
    if compiled.exists():
      import pickle
      metadata = json.loads(compiled.with_suffix('.json').read_text())
      if (metadata.get('version') != 1 or metadata.get('model_sha256') != hashlib.sha256(Path(path).read_bytes()).hexdigest()
          or metadata.get('runtime') != qcom_runtime_identity()
          or metadata.get('compiled_sha256') != hashlib.sha256(compiled.read_bytes()).hexdigest()):
        raise ValueError('compiled signal QCOM identity mismatch')
      # Only locally compiled, identity-checked artifacts; never download a pickle.
      with compiled.open('rb') as f:
        self.input_names, self.forward = pickle.load(f)
      return
    self.runner = OnnxRunner(str(path))
    self.input_names = tuple(self.runner.graph_inputs)
    def forward(*values):
      outputs = self.runner(dict(zip(self.input_names, values)))
      return tuple(outputs[n].realize() for n in self.runner.graph_outputs)
    self.forward = TinyJit(forward, prune=True)

  def run(self, output_names, values):
    if output_names is not None or set(values) != set(self.input_names):
      raise ValueError('unexpected QCOM input/output contract')
    arrays = [np.ascontiguousarray(values[n], dtype=np.float32) for n in self.input_names]
    if self.inputs is None:
      self.inputs = [self.Tensor(a, device='QCOM').realize() for a in arrays]
    else:
      for tensor, array in zip(self.inputs, arrays):
        if tensor.shape != array.shape:
          raise ValueError('QCOM input shape changed')
        tensor.assign(array).realize()
    return [output.numpy() for output in self.forward(*self.inputs)]


QCOM_FLAGS = dict(DEV='QCOM', IMAGE='1', FLOAT16='0', NOLOCALS='1', JIT_BATCH_SIZE='0', OPENPILOT_HACKS='1')


def qcom_runtime_identity():
  """Bind local compiled code to its runtime tree, Python ABI and GPU."""
  import platform
  import subprocess
  import sys
  import tinygrad
  repository = Path(tinygrad.__file__).resolve().parents[2]
  subprocess.run(['git', '-C', str(repository), 'diff', '--quiet', 'HEAD', '--', 'tinygrad_repo'], check=True)
  tree = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', 'HEAD:tinygrad_repo'], text=True).strip()
  chip = Path('/sys/class/kgsl/kgsl-3d0/gpu_model')
  return dict(tree=tree, python=sys.implementation.cache_tag, machine=platform.machine(),
              gpu=chip.read_text().strip() if chip.exists() else 'local-qcom', flags=QCOM_FLAGS)


class NeuralSignal:
  """Pinned pretrained ONNX; explicit CPU or QCOM backend and artifact paths."""
  DETECTOR_SHA = '1ad633066a1195006f4709f8fa07800dd65a74a814b3efb4c99bcc5a1a7962f6'
  CLASSIFIER_SHA = '455b71b3b20d3a96aa0e49f32714ba50421f668a2f9b9907c30b1346ac8a3703'
  LABELS = ('green', 'left,red', 'left,red,straight', 'red', 'red,right', 'red,straight',
            'unknown', 'yellow', 'red,up_left', 'red,right,straight', 'red,up_right')

  def __init__(self, detector, classifier, backend='cpu'):
    if backend not in ('cpu', 'qcom'):
      raise ValueError('unsupported neural backend')
    self.backend = backend
    if backend == 'cpu':
      import onnxruntime as ort
      options = ort.SessionOptions()
      options.intra_op_num_threads = options.inter_op_num_threads = 1
      options.add_session_config_entry('session.intra_op.allow_spinning', '0')
    sessions = []
    for path, expected in [(detector, self.DETECTOR_SHA), (classifier, self.CLASSIFIER_SHA)]:
      if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
        raise ValueError('reference ONNX hash mismatch')
      sessions.append(QcomSession(path) if backend == 'qcom' else
                      ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider']))
    self.detector, self.classifier = sessions
    self.grid = np.array([(x, y, s) for s in (8, 16, 32) for y in range(416//s) for x in range(416//s)], np.float32)

  def detect(self, rgb, rect=None):
    h, w = rgb.shape[:2]
    x1, y1, x2, y2 = rect or (0, 0, w, h)
    crop = rgb[y1:y2, x1:x2]
    h, w = crop.shape[:2]
    scale = min(416/w, 416/h)
    tensor = np.full((416, 416, 3), 114, np.float32)
    resized = cv2.resize(crop[..., ::-1], (int(w*scale), int(h*scale)))
    tensor[:resized.shape[0], :resized.shape[1]] = resized
    p = self.detector.run(None, {'images': tensor.transpose(2, 0, 1)[None].copy()})[0][0]
    score = p[:, 4] * p[:, 6]
    indices = np.flatnonzero(score > .3)
    q, grid = p[indices], self.grid[indices]
    centers = (q[:, :2] + grid[:, :2]) * grid[:, 2:]
    sizes = np.exp(q[:, 2:4]) * grid[:, 2:]
    boxes = np.concatenate((centers-sizes/2, centers+sizes/2), axis=1)/scale + [x1, y1, x1, y1]
    kept = []
    for j in np.argsort(-score[indices]):
      box = np.clip(boxes[j], [0, 0, 0, 0], [rgb.shape[1], rgb.shape[0]]*2).tolist()
      if not all(np.isfinite(box)) or box[0] >= box[2] or box[1] >= box[3]:
        continue
      if all(overlap(box, old['box']) < .65 for old in kept):
        kept.append(dict(box=box, score=float(score[indices[j]])))
    return kept

  def classify(self, rgb, box):
    h, w = rgb.shape[:2]
    x1, y1, x2, y2 = map(int, box)
    crop = rgb[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
    if not crop.size:
      return dict(label='unknown', score=0.)
    h, w = crop.shape[:2]
    scale = min(224/w, 224/h)
    resized = cv2.resize(crop, (max(1, round(w*scale)), max(1, round(h*scale))))
    tensor = np.zeros((224, 224, 3), np.float32)
    tensor[:resized.shape[0], :resized.shape[1]] = resized
    tensor = (tensor-np.array([123.675, 116.28, 103.53], np.float32))/np.array([58.395, 57.12, 57.375], np.float32)
    probs = self.classifier.run(None, {'input': tensor.transpose(2, 0, 1)[None].copy()})[0][0]
    index = int(probs.argmax())
    return dict(label=self.LABELS[index], score=float(probs[index]))


class NeuralSignalSequence:
  """Causal offline prototype: periodic ROI detection, per-frame housing tracking.

  Tracking uses surrounding structure, excludes changing lamps, and expires
  without neural reacquisition. Classifier unknown stays unknown; no color hold.
  """
  def __init__(self, neural, period=.2):
    self.neural, self.period = neural, period
    self.last_detection = float('-inf')
    self.previous_time = None
    self.tracks = []
    self.next_id = 1

  @staticmethod
  def template(gray, box):
    h, w = gray.shape
    a, b, c, d = map(int, box)
    x1, y1, x2, y2 = max(0, a-6), max(0, b-6), min(w, c+6), min(h, d+6)
    patch = gray[y1:y2, x1:x2].copy()
    mask = np.ones(patch.shape, np.uint8)*255
    mask[max(0, b-y1):min(y2-y1, d-y1), max(0, a-x1):min(x2-x1, c-x1)] = 0
    if patch.size == 0 or np.count_nonzero(mask) < 20 or np.std(patch[mask > 0]) < 2:
      return None
    return patch, mask, (x1, y1)

  def process(self, rgb, timestamp):
    if self.previous_time is not None and not 0 < timestamp-self.previous_time <= .25:
      self.tracks = []; self.last_detection = float('-inf')
    self.previous_time = timestamp
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    h, w = gray.shape
    tracked = []
    for old in self.tracks:
      if timestamp-old['detected_at'] > .5 or old['template'] is None:
        continue
      patch, mask, (px, py) = old['template']
      ph, pw = patch.shape
      x1, y1, x2, y2 = max(0, px-12), max(0, py-12), min(w, px+pw+12), min(h, py+ph+12)
      response = cv2.matchTemplate(gray[y1:y2, x1:x2], patch, cv2.TM_CCORR_NORMED, mask=mask)
      response = np.where(np.isfinite(response), response, -1.)
      _, quality, _, (dx, dy) = cv2.minMaxLoc(response)
      if quality < .90:
        continue
      dx, dy = x1+dx-px, y1+dy-py
      box = [v+shift for v, shift in zip(old['box'], (dx, dy, dx, dy))]
      if not (0 <= box[0] < box[2] <= w and 0 <= box[1] < box[3] <= h):
        continue
      tracked.append(dict(old, box=box, source='tracked'))
    # Search more often after losing all objects; never make a stale color fresh.
    interval = self.period if tracked else min(self.period, .1)
    detected = timestamp - self.last_detection >= interval - 1e-9
    if detected:
      self.last_detection = timestamp
      proposals = self.neural.detect(rgb, (int(.2*w), int(.04*h), int(.85*w), int(.6*h)))
      used = set(); merged = []
      for p in proposals:
        candidates = [t for t in tracked if t['id'] not in used and overlap(t['box'], p['box']) >= .2]
        if candidates:
          old = max(candidates, key=lambda t: overlap(t['box'], p['box'])); tid = old['id']; used.add(tid)
        else:
          tid = self.next_id; self.next_id += 1
        merged.append(dict(p, id=tid, detected_at=timestamp, source='detected'))
      tracked = merged + [t for t in tracked if t['id'] not in used and all(overlap(t['box'], p['box']) < .2 for p in proposals)]
    self.tracks = tracked[:20]
    result = []
    for t in self.tracks:
      t['template'] = self.template(gray, t['box'])
      color = self.neural.classify(rgb, t['box'])
      result.append(dict(id=t['id'], box=t['box'], source=t['source'], neural_age=timestamp-t['detected_at'], **color))
    return dict(timestamp=timestamp, detected=detected, tracks=result)
