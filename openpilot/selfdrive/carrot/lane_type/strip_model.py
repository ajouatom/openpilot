"""Frozen StripNet on the 4.5-20 m row-aligned YCbCr strip (display-only lane marking recogniser).

The strip is built exactly as in training (tools/lane_profile/build_dataset.py): profile_scanner geometry
and sampling, uint8, rows area-resized to 128. Its lateral width comes from the manifest's ``strip``
({half_m, cols, rows}; absent keys = DEFAULT_STRIP, the v4 +-1.0 m, 128 columns, 128 rows); the ONNX input shape must match it.
Inference is OpenCV DNN on the CPU (one thread), the same backend as the existing lane classifier.
Below the confidence threshold a head is UNKNOWN; dashed+yellow is not a learned class and is UNKNOWN for both heads.

Manifest v4 (``outputs`` == OUTPUTS_V4, ONNX outputs ``color`` (N, 3) and ``type`` (N, 5) logits; or OUTPUTS_V4_4, the
4-type head without an 'unknown' class, ``type`` (N, 4)) is mapped onto the two-class result the strip filter reads
(StripModel.decide4, through StripModel.decide). ``mirror_right``: marking_daemon flips the right strip (see MIRROR_AXIS).

The model is assets/lane_strip_v4.{onnx,json} (MODEL_NAME), the only StripNet model (DaBin 10-26: the v1-v3 lane_strip model,
the model choice setting and its fallback were removed). load() returns None when it is missing or invalid; marking_daemon then
publishes nothing (the existing lane display stays) and logs a warning.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from openpilot.selfdrive.carrot.lane_type.profile_scanner import sample, strip_geometry

ASSETS = Path(__file__).with_name('assets')
MODEL_NAME = 'lane_strip_v4'  # the StripNet model (file stem in ASSETS)
HALF_M, ROWS, COLUMNS = 1., 128, 128
DEFAULT_STRIP = {'half_m': HALF_M, 'cols': COLUMNS, 'rows': ROWS}  # the v4 strip (a manifest key absent from 'strip')
TYPES, COLORS = ('dashed', 'solid'), ('white', 'yellow')  # the two-class result decide() makes (strip filter / display)
# v4 contract (shared with tools/lane_profile training): near = the ego-side line of a double marking.
TYPES_V4, COLORS_V4 = ('dashed', 'solid', 'dbl_near', 'dbl_far', 'unknown'), ('white', 'yellow', 'blue')
OUTPUTS_V4 = {'color': list(COLORS_V4), 'type': list(TYPES_V4)}
# 4-type v4 (2026-10-26, 'unknown is not a class'): the type head learns dashed / solid / dbl_near / dbl_far only and abstains
# (reason 'unknown', as a 5-type unknown argmax) when its largest type probability is below the manifest threshold.
TYPES_V4_4 = TYPES_V4[:4]
OUTPUTS_V4_4 = {'color': list(COLORS_V4), 'type': list(TYPES_V4_4)}
# HOLD_COLOR (filterab1026 F3, replays only; False in the product): an abstaining decide4 result also carries 'pYellowHeld'
# (the colour softmax P(yellow)) so marking_daemon can update the colour while the type abstains.
# HOLD_BY_FAMILY (4-type head; DaBin 10-26 filterab1026 F4, the product default): abstain when
# max(P(dashed)+P(dbl_near), P(solid)+P(dbl_far)) < threshold; False: abstain when the largest single class is below it.
# It acts on 4-type manifests only: no effect on the current product asset (lane_strip_v4 03e5f5153701, round1108b UB s0, 5 types).
HOLD_COLOR = False
HOLD_BY_FAMILY = True
# profile_scanner.strip_geometry: column 0 = line y - half_m, the last column = y + half_m (image left -> right), so the
# ego side is the HIGH columns of the left strip and the LOW columns of the right strip. With mirror_right the right
# strip is flipped on its column axis (the last one of 3 x rows x cols), putting its ego side on the high columns too.
MIRROR_AXIS = -1


def strip_of(manifest):
  """Strip geometry {half_m, cols, rows} of a manifest; a missing 'strip' (or key) keeps DEFAULT_STRIP."""
  given = manifest.get('strip') or {}
  if not isinstance(given, dict) or set(given) - set(DEFAULT_STRIP):
    raise ValueError(f'lane strip manifest: invalid strip {given!r}')
  out = {**DEFAULT_STRIP, **given}
  half_m, cols, rows = out['half_m'], out['cols'], out['rows']
  if (isinstance(half_m, bool) or not isinstance(half_m, (int, float)) or not 0 < half_m <= 3 or
      any(isinstance(v, bool) or not isinstance(v, int) or not 0 < v <= 1024 for v in (cols, rows))):
    raise ValueError(f'lane strip manifest: invalid strip {given!r}')
  return {'half_m': float(half_m), 'cols': cols, 'rows': rows}


def _fields(buf):
  """(field number, wire type, value) of one protobuf message (value: int or bytes)."""
  i, n = 0, len(buf)
  def varint():
    nonlocal i
    v = shift = 0
    while True:
      b = buf[i]
      i += 1
      v |= (b & 0x7f) << shift
      shift += 7
      if b < 0x80:
        return v
  while i < n:
    key = varint()
    field, wire = key >> 3, key & 7
    if wire == 0:
      yield field, wire, varint()
    elif wire == 2:
      size = varint()
      yield field, wire, buf[i:i + size]
      i += size
    elif wire in (1, 5):
      i += 8 if wire == 1 else 4
    else:
      raise ValueError('lane strip model: unreadable ONNX')


def onnx_input_shape(data, name, field=11):
  """Declared dims of graph input ``name`` (None for a symbolic dim), or None when the input is absent.
  field=12: a graph output instead (GraphProto.output)."""
  data = memoryview(data)
  for f, w, graph in _fields(data):
    if f != 7 or w != 2:  # ModelProto.graph
      continue
    for g, gw, vi in _fields(graph):
      if g != field or gw != 2:  # GraphProto.input (11) / output (12)
        continue
      fields = list(_fields(vi))
      if bytes(next((v for k, kw, v in fields if k == 1 and kw == 2), b'')).decode() != name:
        continue
      dims = []
      for k, kw, tp in fields:
        if k == 2 and kw == 2:  # ValueInfoProto.type -> TypeProto.tensor_type -> shape -> dim
          for t, tw, tensor in _fields(tp):
            for s, sw, shape in (_fields(tensor) if t == 1 and tw == 2 else ()):
              for d, dw, dim in (_fields(shape) if s == 2 and sw == 2 else ()):
                if d == 1 and dw == 2:
                  dims.append(next((int(v) for x, xw, v in _fields(dim) if x == 1 and xw == 0), None))
      return dims
  return None


def onnx_output_shape(data, name):
  """Declared dims of graph output ``name`` (None for a symbolic dim), or None when the output is absent."""
  return onnx_input_shape(data, name, field=12)


def softmax(x):
  e = np.exp(np.asarray(x, np.float64) - np.max(x))
  return e/e.sum()


def strip_input(y_plane, uv_plane, line, rpy, k, half_m=HALF_M, cols=COLUMNS, rows=ROWS):
  """3 x rows x cols float32 raw YCbCr (default 3 x 128 x 128 over +-1.0 m), or None when the boundary geometry is
  unavailable. half_m / cols / rows: the model manifest's strip (StripModel.strip)."""
  import cv2
  geo = strip_geometry(line, rpy, k, y_plane.shape[1], y_plane.shape[0], half_m=half_m, cols=cols)
  if geo is None:
    return None
  lum, cb, cr = sample(y_plane, uv_plane, geo)
  ycc = np.clip(np.stack((lum, cb, cr), -1), 0, 255).astype(np.uint8)
  ycc = cv2.resize(ycc, (cols, rows), interpolation=cv2.INTER_AREA if len(ycc) > rows else cv2.INTER_LINEAR)
  return ycc.transpose(2, 0, 1).astype(np.float32)


_DEFAULT_STRIP_INPUT = strip_input


def batch_strip_inputs(y_plane, uv_plane, lines, rpy, k, geometries, extractor):
  """Width-major strips. Preserve the scalar sampler hook used by cached replays and alternate samplers."""
  if strip_input is not _DEFAULT_STRIP_INPUT:
    groups = []
    for line in lines:
      group = []
      for geometry in geometries:
        value = strip_input(y_plane, uv_plane, line, rpy, k, **geometry)
        if value is None:
          group = [None]*len(geometries)
          break
        group.append(value)
      groups.append(group)
    return [group[j] for j in range(len(geometries)) for group in groups]
  widths = sorted({(g.get('half_m', HALF_M), g.get('cols', COLUMNS), g.get('rows', ROWS)) for g in geometries})
  strips = extractor.extract(y_plane, uv_plane, lines, rpy, k, widths)
  return [strips[widths.index((g.get('half_m', HALF_M), g.get('cols', COLUMNS), g.get('rows', ROWS)))*len(lines) + i]
          for g in geometries for i in range(len(lines))]


class StripModel:
  def __init__(self, directory=ASSETS, threshold=None, name=MODEL_NAME):
    """``name``: the file stem of the manifest / ONNX pair in ``directory`` (MODEL_NAME)."""
    import cv2
    directory = Path(directory)
    manifest = json.loads((directory/f'{name}.json').read_text(encoding='utf-8'))
    if not isinstance(manifest, dict):  # valid JSON but not an object ([], null, 3): a contract error (load() returns None)
      raise ValueError(f'lane strip manifest {name}.json: top level is {type(manifest).__name__}, expected an object')
    data = (directory/f'{name}.onnx').read_bytes()
    self.name, self.version = name, manifest.get('version')
    outputs = manifest.get('outputs')
    if hashlib.sha256(data).hexdigest() != manifest['model_sha256'] or outputs not in (OUTPUTS_V4, OUTPUTS_V4_4):
      raise ValueError('lane strip model artifact contract mismatch')
    self.v4 = True  # the only supported manifest (kept for replays that read it)
    self.mirror_right = manifest.get('mirror_right', False)
    if not isinstance(self.mirror_right, bool):
      raise ValueError(f"lane strip manifest: mirror_right {self.mirror_right!r} needs a bool")
    for head, classes in outputs.items():
      dims = onnx_output_shape(data, head)
      if dims is None or len(dims) != 2 or dims[1] != len(classes):
        expects = f'[N, {len(classes)}] for {classes}'
        raise ValueError(f"lane strip ONNX output {head!r} shape {dims} does not match the manifest v4 outputs (expects {expects})")
    self.heads = ['type', 'color']
    self.strip = strip_of(manifest)
    want = [3, self.strip['rows'], self.strip['cols']]
    geometry = f"strip +-{self.strip['half_m']:g} m / {self.strip['cols']} cols / {self.strip['rows']} rows"
    if manifest['input']['shape'] != want:
      raise ValueError(f"lane strip manifest input shape {manifest['input']['shape']} does not match its {geometry} (expects {want})")
    dims = onnx_input_shape(data, manifest['input']['name'])
    if dims is None or len(dims) != 4 or dims[1:] != want:
      expects = ', '.join(map(str, want))
      raise ValueError(f"lane strip ONNX input {manifest['input']['name']!r} shape {dims} does not match the manifest {geometry} (expects [N, {expects}])")
    self.threshold = float(manifest['threshold'] if threshold is None else threshold)
    self.model_hash = manifest['model_sha256']
    self.net = cv2.dnn.readNetFromONNX(np.frombuffer(data, dtype=np.uint8))
    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

  def info(self):
    """What the daemon logs and publishes for the active model (packet 'model')."""
    return {'name': self.name, 'version': self.version, 'sha256': self.model_hash[:12], 'v4': self.v4}

  def forward(self, strips):
    """The network's (type, colour) logit rows for a non-empty list of strips."""
    self.net.setInput(np.ascontiguousarray(np.stack(strips)))
    return self.net.forward(self.heads)

  def infer(self, strips):
    """strips: list of 3 x rows x cols float32 (self.strip) -> list of result dicts (same keys as result.unknown())."""
    if not strips:
      return []
    types, colors = self.forward(strips)
    return [self.decide4(t, c) for t, c in zip(types, colors, strict=True)]

  def decide(self, type_p, color_p, yellow_dashed_learned=False):
    """yellow_dashed_learned (v4): dashed+yellow is a legal Korean marking (centre-line passing zone, roadside parking line)
    and is passed through; v1-v3 (default) report it as unknown."""
    out = {'reason': 'strip_model'}
    for head, p, classes in (('type', type_p, TYPES), ('color', color_p, COLORS)):
      i = int(np.argmax(p))
      confident = float(p[i]) >= self.threshold
      out[head] = classes[i] if confident else 'unknown'
      out[head + 'Confidence'] = float(p[i])
      out[head + 'Candidate'] = classes[i]
    if out['type'] == 'dashed' and out['color'] == 'yellow' and not yellow_dashed_learned:
      out.update(type='unknown', color='unknown', reason='yellow_dashed_not_learned')
    return out

  def decide4(self, type_logits, color_logits):
    """v4 logits -> the two-class result decide() makes, plus the v4 extras (typeClass, colorClass, double, blue, pUnknown).

    Type: an 'unknown' argmax abstains (reason 'unknown': no probabilities, both heads unknown; marking_daemon passes it like
    'lane_prob_dip'); otherwise P(dashed) = (P(dashed) + P(dbl_near)) / (1 - P(unknown)) (near = ego side -> dashed display,
    far -> solid). Colour: P(yellow) of the 3-way softmax; blue is shown as white (P(white) = 1 - P(yellow)). Yellow + dashed
    is passed through (type dashed, colour yellow, reason strip_model), unlike v1-v3.

    4-type logits (OUTPUTS_V4_4, no unknown class): P(unknown) = 0; the side abstains exactly like an unknown argmax when the
    largest of the 4 type probabilities is below the threshold; otherwise P(dashed) = P(dashed) + P(dbl_near)."""
    pt, pc = softmax(type_logits), softmax(color_logits)
    if len(pt) == len(TYPES_V4_4):
      t, c = int(np.argmax(pt)), int(np.argmax(pc))
      extras = {'typeClass': TYPES_V4_4[t], 'colorClass': COLORS_V4[c], 'double': TYPES_V4_4[t] in ('dbl_near', 'dbl_far'),
                'blue': COLORS_V4[c] == 'blue', 'pUnknown': 0.}
      family = max(float(pt[0] + pt[2]), float(pt[1] + pt[3]))
      if (family if HOLD_BY_FAMILY else float(pt[t])) < self.threshold:
        if HOLD_COLOR:
          extras['pYellowHeld'] = float(pc[1])
        return {'reason': 'unknown', 'type': 'unknown', 'typeConfidence': 0., 'typeCandidate': 'unknown',
                'color': 'unknown', 'colorConfidence': 0., 'colorCandidate': 'unknown', **extras}
      dashed = min(1., float(pt[0] + pt[2]))
      yellow = float(pc[1])
      out = self.decide(np.array([dashed, 1. - dashed]), np.array([1. - yellow, yellow]), yellow_dashed_learned=True)
      out.update(extras)
      return out
    t, c = int(np.argmax(pt)), int(np.argmax(pc))
    u = float(pt[TYPES_V4.index('unknown')])
    extras = {'typeClass': TYPES_V4[t], 'colorClass': COLORS_V4[c], 'double': TYPES_V4[t] in ('dbl_near', 'dbl_far'),
              'blue': COLORS_V4[c] == 'blue', 'pUnknown': u}
    if TYPES_V4[t] == 'unknown':
      if HOLD_COLOR:
        extras['pYellowHeld'] = float(pc[1])
      return {'reason': 'unknown', 'type': 'unknown', 'typeConfidence': 0., 'typeCandidate': 'unknown',
              'color': 'unknown', 'colorConfidence': 0., 'colorCandidate': 'unknown', **extras}
    dashed = min(1., float(pt[0] + pt[2])/(1. - u))
    yellow = float(pc[1])
    out = self.decide(np.array([dashed, 1. - dashed]), np.array([1. - yellow, yellow]), yellow_dashed_learned=True)
    out.update(extras)
    return out


def load(directory=None, warn=None):
  """The StripNet model (MODEL_NAME in ``directory``, None: ASSETS looked up at call time), or None when its files are missing,
  unreadable or fail the manifest / ONNX contract (``warn(message)`` is called first). There is no fallback model."""
  import cv2
  directory = ASSETS if directory is None else directory
  try:
    return StripModel(directory)
  except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
    if warn is not None:
      warn(f'lane strip model {MODEL_NAME} not loaded, lane types are not shown: {type(error).__name__}: {error}')
    return None
