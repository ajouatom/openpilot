"""Native camera warp and a guarded switch between local and Jetlink inference."""
import json
import os
from pathlib import Path
import time

import numpy as np

from openpilot.selfdrive.modeld.jetlink.link import Client, SPEC, FAULT, state

MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')


def may_join(now, messages, valid, received):
  required = ('carState', 'selfdriveState', 'carControl')
  if not all(valid.get(k, False) and 0 <= now - received.get(k, -1e6) < .25 for k in required):
    return False
  cs, sd, cc = (messages[k] for k in required)
  return cs.standstill and abs(cs.vEgo) < .01 and not (sd.enabled or cc.latActive or cc.longActive)


class Warp:
  def __init__(self, width, height):
    from tinygrad import Tensor, TinyJit, Device
    from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_warp
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    if Device.DEFAULT != 'QCOM':
      raise RuntimeError('Jetlink camera adapter requires the native QCOM device')
    self.size = get_nv12_info(width, height)[3]
    self.transforms = {k: np.eye(3, dtype=np.float32) for k in ('tfm', 'big_tfm')}
    self.inputs = {k: Tensor(v, device='NPY').realize() for k, v in self.transforms.items()}
    self.run_warp = TinyJit(make_warp(NV12Frame(width, height, *get_nv12_info(width, height)), 512, 256, SPEC.frame_skip))
    dummy = {k: Tensor(np.zeros(self.size, np.uint8), device='QCOM').realize() for k in ('frame', 'big_frame')}
    for _ in range(3):
      result = self.run_warp(**self.inputs, **dummy).numpy()
      if result.shape != SPEC.warped_shape or result.dtype != np.uint8:
        raise ValueError('unexpected camera warp contract')
    from openpilot.system.hardware import HARDWARE
    from openpilot.selfdrive.modeld.jetlink.warp import validated_warp
    self.run_warp = validated_warp(self.run_warp, NV12Frame(width, height, *get_nv12_info(width, height)),
                                   self.transforms, self.inputs, SPEC.frame_skip, HARDWARE.get_device_type())
    self.blobs = {}
    self.timings = (0., 0., 0.)

  def __call__(self, bufs, transforms):
    from tinygrad import Tensor
    started = time.monotonic()
    frames = {}
    for key, arg in (('img', 'frame'), ('big_img', 'big_frame')):
      source = np.frombuffer(bufs[key].data, np.uint8)
      if source.size != self.size:
        raise ValueError('camera layout changed')
      cache_key = (key, source.ctypes.data)
      if cache_key not in self.blobs:
        if len(self.blobs) >= 64:
          self.blobs.clear()
        self.blobs[cache_key] = Tensor.from_blob(source.ctypes.data, (self.size,), dtype='uint8', device='QCOM')
      frames[arg] = self.blobs[cache_key]
    self.transforms['tfm'][:] = transforms['img']
    self.transforms['big_tfm'][:] = transforms['big_img']
    prepared = time.monotonic()
    result = self.run_warp(**self.inputs, **frames)
    submitted = time.monotonic()
    output = result.numpy()
    self.timings = (prepared - started, submitted - prepared, time.monotonic() - submitted)
    return output


class JoiningModel:
  def __init__(self, small, width, height):
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.system.hardware import HARDWARE
    self.phase_enabled = HARDWARE.get_device_type() == 'mici'
    self.source_sof = 0
    self.small = small
    self.warp = Warp(width, height)
    self.parser = Parser()
    self.client = None
    self.active = False
    self.usbgpu = False
    self.vision_input_names = ['img', 'big_img']
    self.packed = np.zeros(SPEC.packed_nelem, np.float32)
    self.views = {name: a.reshape(shape) for (name, shape), a in zip(
      SPEC.packed_shapes.items(), np.split(self.packed, np.cumsum(SPEC.packed_sizes[:-1])), strict=True)}
    self.prev_desire = np.zeros(8, np.float32)
    self.frame = 0
    self.join_allowed = False
    self.next_join = 0.
    self.next_status = 0.
    self.ready = False
    self.reset = True
    self.error = ''
    self.small_runs = 0
    self.last_slow_log = 0.

  def update(self, sm, metadata):
    self.source_sof = metadata.timestamp_sof if self.phase_enabled else 0
    now = time.monotonic()
    self.join_allowed = may_join(now, {k: sm[k] for k in ('carState', 'selfdriveState', 'carControl')},
                                {k: sm.valid[k] and sm.alive[k] for k in ('carState', 'selfdriveState', 'carControl')}, sm.recv_time)
    if now >= self.next_status:
      self.ready = state().get('state') == 'ready'
      record = dict(active=self.active, ready=self.ready, model='Cinque v2', error=self.error, updated=now)
      tmp = MODEL_STATUS.with_suffix('.tmp')
      tmp.write_text(json.dumps(record))
      os.replace(tmp, MODEL_STATUS)
      self.next_status = now + 1

  def _reset_small(self):
    # Captured JITs keep buffer identities: clear in place, never replace queues.
    self.small.prev_desire[:] = 0
    for array in self.small.npy.values():
      array[:] = 0
    for name, tensor in self.small.input_queues.items():
      if name.endswith('_q'):
        tensor._buffer().copyin(memoryview(bytearray(tensor.numel() * tensor.dtype.itemsize)))

  def run(self, bufs, transforms, inputs, prepare_only):
    from openpilot.common.swaglog import cloudlog
    if self.client is None and self.small_runs >= 3 and self.ready and self.join_allowed and time.monotonic() >= self.next_join:
      try:
        self.client = Client()
        self.packed[:] = 0
        self.prev_desire[:] = 0
        self.reset = True
      except Exception as exc:
        self.error = str(exc)
        self.next_join = time.monotonic() + 5
    if self.client is not None:
      try:
        desire = inputs['desire_pulse'].copy()
        desire[0] = 0
        self.views['desire'][:] = np.where(desire - self.prev_desire > .99, desire, 0)
        self.prev_desire[:] = desire
        for name in ('traffic_convention', 'action_t'):
          self.views[name][:] = inputs[name]
        self.frame = (self.frame + 1) & 0xFFFFFFFF
        started = time.monotonic()
        images = self.warp(bufs, transforms)
        warped = time.monotonic()
        result = self.client.infer(images, self.packed, self.frame, self.reset, source_sof=self.source_sof)
        replied = time.monotonic()
        self.reset = False
        self.views['prev_feat'][:] = result[SPEC.output_slices['hidden_state']]
        if not self.active:
          cloudlog.warning('Jetlink active: Cinque v2 %s', SPEC.sha256)
          self.active = True
          self.error = ''
          self.next_status = 0
        # As in the generic eGPU runtime, run the full recurrent graph for each
        # received pair. Camera gaps still propagate unchanged into pose validity.
        parsed = self.parser.parse_outputs({k: result[np.newaxis, v] for k, v in SPEC.output_slices.items()})
        if os.getenv('SEND_RAW_PRED'):
          parsed['raw_pred'] = result.copy()
        finished = time.monotonic()
        if finished - started > .05 and finished - self.last_slow_log >= 1:
          self.last_slow_log = finished
          cloudlog.warning('Jetlink slow frame %d: warp %.2f (prepare/submit/read %.2f/%.2f/%.2f), '
                           'roundtrip %.2f, parse %.2f ms; server gpu/queue/total %.2f/%.2f/%.2f ms',
                           self.frame, (warped-started)*1000, *(v*1000 for v in self.warp.timings),
                           (replied-warped)*1000, (finished-replied)*1000, *(v/1000 for v in self.client.timings))
        return parsed
      except Exception as exc:
        cloudlog.exception('Jetlink failed; restoring local model')
        self.error = str(exc)
        self.client.close()
        self.client = None
        if self.active:
          # Existing commIssue event requests disengagement even if local
          # inference recovers immediately; never silently switch under control.
          FAULT.write_text(str(time.monotonic()))
          self._reset_small()
        self.active = False
        self.next_join = time.monotonic() + 5
        self.next_status = 0
    result = self.small.run(bufs, transforms, inputs, prepare_only)
    if result is not None:
      # Loading the PKL alone leaves its first execution cold (~0.8 s on C4).
      # Execute the native fallback before allowing an external-model join.
      self.small_runs += 1
    return result
