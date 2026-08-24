#!/usr/bin/env python3
"""Carrot model selector — custom modeld entry point.

Runs whenever `/data/models` contains a valid custom model set.  Self-contained
so upstream `selfdrive/modeld/modeld.py` stays untouched and free to track
carrot-wip changes independently.  Supports both legacy 2-model (vision +
policy) and the newer 3-model (vision + on_policy + off_policy) layouts.
"""
import json
import os
from pathlib import Path

# Custom-model root.  Fixed; if this is not a valid model set, modeld_runner
# falls back to the built-in modeld and this module is never imported.
MODELS_DIR = Path('/data/models')

# Match the backend-selection behaviour of upstream tinygrad_helpers so the
# custom model is compiled with the flags it was built against.
_COMPILED_FLAGS = MODELS_DIR / 'tg_compiled_flags.json'
if _COMPILED_FLAGS.is_file():
  try:
    with open(_COMPILED_FLAGS) as _f:
      os.environ['DEV'] = str(json.load(_f)['DEV'])
  except Exception:
    pass

USBGPU = "USBGPU" in os.environ
if USBGPU:
  os.environ['DEV'] = 'AMD'
  os.environ['AMD_IFACE'] = 'USB'
from tinygrad.tensor import Tensor
import time
import pickle
import numpy as np
import openpilot.cereal.messaging as messaging
from openpilot.cereal import car, log
from openpilot.cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, fill_driving_model_data, PublishState
from openpilot.selfdrive.modeld.helpers import select_vision_streams
from openpilot.common.file_chunker import read_file_chunked
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

# Use our own parser that supports 3-model (off_policy) and metadata-driven
# conditional parsing; upstream carrot-wip parser is 2-model only.
from openpilot.carrot.model_selector.carrot_parse_model_outputs import Parser


PROCESS_NAME = "carrot.model_selector.carrot_modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

DEFAULT_MODEL_DIR = Path('/data/openpilot/openpilot/selfdrive/modeld/models')


def validate_model_files(base: Path) -> bool:
  """vision pair required; on_policy or policy pair satisfies the policy slot."""
  for fname in ('driving_vision_tinygrad.pkl', 'driving_vision_metadata.pkl'):
    fp = base / fname
    if not fp.exists() or fp.stat().st_size == 0:
      cloudlog.warning(f"carrot_modeld: missing or empty {fp}")
      return False
  has_on = ((base / 'driving_on_policy_tinygrad.pkl').exists() and
            (base / 'driving_on_policy_metadata.pkl').exists())
  has_policy = ((base / 'driving_policy_tinygrad.pkl').exists() and
                (base / 'driving_policy_metadata.pkl').exists())
  if not (has_on or has_policy):
    cloudlog.warning(f"carrot_modeld: no policy model in {base}")
    return False
  return True


def get_model_paths():
  """Resolve pkl paths, preferring on_policy naming then policy, and
  optionally loading the off_policy pair when present.
  """
  base = MODELS_DIR if validate_model_files(MODELS_DIR) else DEFAULT_MODEL_DIR
  cloudlog.warning(f"carrot_modeld: using model dir {base}")

  on_pkl = base / 'driving_on_policy_tinygrad.pkl'
  on_meta = base / 'driving_on_policy_metadata.pkl'
  if on_pkl.exists() and on_meta.exists():
    policy_pkl, policy_meta = on_pkl, on_meta
  else:
    policy_pkl = base / 'driving_policy_tinygrad.pkl'
    policy_meta = base / 'driving_policy_metadata.pkl'

  paths = {
    'vision_pkl': base / 'driving_vision_tinygrad.pkl',
    'vision_meta': base / 'driving_vision_metadata.pkl',
    'policy_pkl': policy_pkl,
    'policy_meta': policy_meta,
    'models_dir': base,
  }
  off_pkl = base / 'driving_off_policy_tinygrad.pkl'
  off_meta = base / 'driving_off_policy_metadata.pkl'
  if off_pkl.exists() and off_meta.exists():
    paths['off_policy_pkl'] = off_pkl
    paths['off_policy_meta'] = off_meta
  return paths


LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3
RECOVERY_POWER = 1.0  # planplus lane-recovery gain

IMG_QUEUE_SHAPE = (6*(ModelConstants.MODEL_RUN_FREQ//ModelConstants.MODEL_CONTEXT_FREQ + 1), 128, 256)
assert IMG_QUEUE_SHAPE[0] == 30


def img_queue_shape(img_shape: tuple[int, ...]) -> tuple[int, int, int]:
    """Return legacy standalone-warp queue shape for a model image input.

    Metadata image inputs are shaped (batch, stacked_channels, h/2, w/2),
    typically (1, 12, 128, 256).  The legacy warp pkl keeps a rolling channel
    buffer shaped (queued_channels, h/2, w/2); derive it from metadata so custom
    models with non-default image sizes don't reuse the old 512x256 assumption.
    """
    if len(img_shape) != 4:
      raise ValueError(f"unexpected image input shape: {img_shape}")
    n_channels = img_shape[1] // ModelConstants.N_FRAMES
    queued_channels = (ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ + (ModelConstants.N_FRAMES - 1)) * n_channels
    return queued_channels, img_shape[2], img_shape[3]


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float, lat_smooth_seconds: float, vEgoStopping: float) -> log.ModelDataV2.Action:
    # op11: on_policy가 (curv_unscaled, accel)을 직접 산출.
    # plan 기반 산출이 불가능한 경우 폴백.
    if 'action' in model_output:
      desired_curv_unscaled, desired_accel = model_output['action'][0]
      desired_curvature = float(desired_curv_unscaled) / 100.0
      desired_accel = float(desired_accel)
      should_stop = (v_ego < 0.3 and desired_accel < 0.1)

      desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
      if v_ego > MIN_LAT_CONTROL_SPEED:
        desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, lat_smooth_seconds)
      else:
        desired_curvature = prev_action.desiredCurvature

      # op11에는 desired_velocity 산출이 없으므로 plan 기반으로 추가 계산 (있을 때만)
      desired_velocity_now = prev_action.desiredVelocity
      if 'plan' in model_output:
        plan = model_output['plan'][0]
        _, _, _, desired_velocity_now = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0],
                                                            plan[:, Plan.ACCELERATION][:, 0],
                                                            ModelConstants.T_IDXS,
                                                            action_t=long_action_t,
                                                            vEgoStopping=vEgoStopping)
        desired_velocity_now = smooth_value(desired_velocity_now, prev_action.desiredVelocity, LONG_SMOOTH_SECONDS)

      return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                    desiredAcceleration=float(desired_accel),
                                    shouldStop=bool(should_stop),
                                    desiredVelocity=float(desired_velocity_now))

    # op7/legacy: plan 에서 curvature/accel 산출
    plan = model_output['plan'][0]
    desired_accel, should_stop, _, desired_velocity_now = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                     plan[:,Plan.ACCELERATION][:,0],
                                                     ModelConstants.T_IDXS,
                                                     action_t=long_action_t,
                                                     vEgoStopping=vEgoStopping)
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
    desired_velocity_now = smooth_value(desired_velocity_now, prev_action.desiredVelocity, LONG_SMOOTH_SECONDS)

    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
    if v_ego > MIN_LAT_CONTROL_SPEED:
      desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, lat_smooth_seconds)
    else:
      desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                  desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(should_stop),
                                  desiredVelocity=float(desired_velocity_now))

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

class InputQueues:
  def __init__ (self, model_fps, env_fps, n_frames_input):
    assert env_fps % model_fps == 0
    assert env_fps >= model_fps
    self.model_fps = model_fps
    self.env_fps = env_fps
    self.n_frames_input = n_frames_input

    self.dtypes = {}
    self.shapes = {}
    self.q = {}

  def update_dtypes_and_shapes(self, input_dtypes, input_shapes) -> None:
    self.dtypes.update(input_dtypes)
    if self.env_fps == self.model_fps:
      self.shapes.update(input_shapes)
    else:
      for k in input_shapes:
        shape = list(input_shapes[k])
        if 'img' in k:
          n_channels = shape[1] // self.n_frames_input
          shape[1] = (self.env_fps // self.model_fps + (self.n_frames_input - 1)) * n_channels
        else:
          shape[1] = (self.env_fps // self.model_fps) * shape[1]
        self.shapes[k] = tuple(shape)

  def reset(self) -> None:
    self.q = {k: np.zeros(self.shapes[k], dtype=self.dtypes[k]) for k in self.dtypes.keys()}

  def enqueue(self, inputs:dict[str, np.ndarray]) -> None:
    for k in inputs.keys():
      if inputs[k].dtype != self.dtypes[k]:
        raise ValueError(f'supplied input <{k}({inputs[k].dtype})> has wrong dtype, expected {self.dtypes[k]}')
      input_shape = list(self.shapes[k])
      input_shape[1] = -1
      single_input = inputs[k].reshape(tuple(input_shape))
      sz = single_input.shape[1]
      self.q[k][:,:-sz] = self.q[k][:,sz:]
      self.q[k][:,-sz:] = single_input

  def get(self, *names) -> dict[str, np.ndarray]:
    if self.env_fps == self.model_fps:
      return {k: self.q[k] for k in names}
    else:
      out = {}
      for k in names:
        shape = self.shapes[k]
        if 'img' in k:
          n_channels = shape[1] // (self.env_fps // self.model_fps + (self.n_frames_input - 1))
          out[k] = np.concatenate([self.q[k][:, s:s+n_channels] for s in np.linspace(0, shape[1] - n_channels, self.n_frames_input, dtype=int)], axis=1)
        elif 'pulse' in k:
          # any pulse within interval counts
          out[k] = self.q[k].reshape((shape[0], shape[1] * self.model_fps // self.env_fps, self.env_fps // self.model_fps, -1)).max(axis=2)
        else:
          idxs = np.arange(-1, -shape[1], -self.env_fps // self.model_fps)[::-1]
          out[k] = self.q[k][:, idxs]
      return out

class ModelState:
  inputs: dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  @property
  def desire_key(self) -> str:
    """Find the desire input key dynamically (e.g. 'desire' or 'desire_pulse')."""
    return next(key for key in self.numpy_inputs if key.startswith('desire'))

  def __init__(self):
    paths = get_model_paths()
    self._models_dir = paths['models_dir']

    with open(paths['vision_meta'], 'rb') as f:
      vision_metadata = pickle.load(f)
      self.vision_input_shapes = vision_metadata['input_shapes']
      self.vision_input_names = list(self.vision_input_shapes.keys())
      self.vision_output_slices = vision_metadata['output_slices']
      vision_output_size = vision_metadata['output_shapes']['outputs'][1]

    with open(paths['policy_meta'], 'rb') as f:
      policy_metadata = pickle.load(f)
      self.policy_input_shapes = policy_metadata['input_shapes']
      self.policy_output_slices = policy_metadata['output_slices']
      policy_output_size = policy_metadata['output_shapes']['outputs'][1]

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

    # policy inputs
    self.numpy_inputs = {k: np.zeros(self.policy_input_shapes[k], dtype=np.float32) for k in self.policy_input_shapes}
    cloudlog.info(f"carrot_modeld: desire key = {self.desire_key}")
    self.full_input_queues = InputQueues(ModelConstants.MODEL_CONTEXT_FREQ, ModelConstants.MODEL_RUN_FREQ, ModelConstants.N_FRAMES)
    for k in [self.desire_key, 'features_buffer']:
      self.full_input_queues.update_dtypes_and_shapes({k: self.numpy_inputs[k].dtype}, {k: self.numpy_inputs[k].shape})
    self.full_input_queues.reset()

    self.img_queues = {k: Tensor.zeros(img_queue_shape(self.vision_input_shapes[k]), dtype='uint8').contiguous().realize()
                       for k in self.vision_input_names}
    self.full_frames : dict[str, Tensor] = {}
    self._blob_cache : dict[tuple[str, int], Tensor] = {}
    self.transforms_np = {k: np.zeros((3,3), dtype=np.float32) for k in self.img_queues}
    self.transforms = {k: Tensor(v, device='NPY').realize() for k, v in self.transforms_np.items()}
    self.vision_output = np.zeros(vision_output_size, dtype=np.float32)
    self.policy_inputs = {k: Tensor(v, device='NPY').realize() for k,v in self.numpy_inputs.items()}
    self.policy_output = np.zeros(policy_output_size, dtype=np.float32)

    # Optional off-policy model (3-model architecture).
    self.has_off_policy = 'off_policy_pkl' in paths
    if self.has_off_policy:
      with open(paths['off_policy_meta'], 'rb') as f:
        off_meta = pickle.load(f)
        self.off_policy_input_shapes = off_meta['input_shapes']
        self.off_policy_output_slices = off_meta['output_slices']
        off_output_size = off_meta['output_shapes']['outputs'][1]
      self.off_policy_output = np.zeros(off_output_size, dtype=np.float32)
      cloudlog.warning(f"carrot_modeld: 3-model arch, off_policy output_size={off_output_size}")

    self.parser = Parser()
    self.frame_buf_params : dict[str, tuple[int, int, int, int]] = {}
    self.update_imgs = None
    self.vision_run = pickle.loads(read_file_chunked(str(paths['vision_pkl'])))
    self.policy_run = pickle.loads(read_file_chunked(str(paths['policy_pkl'])))
    if self.has_off_policy:
      self.off_policy_run = pickle.loads(read_file_chunked(str(paths['off_policy_pkl'])))

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}
    return parsed_model_outputs

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
                inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    desire_in = inputs[self.desire_key]
    desire_in[0] = 0
    new_desire = np.where(desire_in - self.prev_desire > .99, desire_in, 0)
    self.prev_desire[:] = desire_in
    if self.update_imgs is None:
      for key in bufs.keys():
        w, h = bufs[key].width, bufs[key].height
        self.frame_buf_params[key] = get_nv12_info(w, h)
      warp_name = f'warp_{w}x{h}_tinygrad.pkl'
      warp_path = self._models_dir / warp_name
      if not warp_path.exists():
        warp_path = DEFAULT_MODEL_DIR / warp_name
      with open(warp_path, "rb") as f:
        self.update_imgs = pickle.load(f)

    for key in bufs.keys():
      ptr = np.frombuffer(bufs[key].data, dtype=np.uint8).ctypes.data
      yuv_size = self.frame_buf_params[key][3]
      # There is a ringbuffer of imgs, just cache tensors pointing to all of them
      cache_key = (key, ptr)
      if cache_key not in self._blob_cache:
        self._blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype='uint8')
      self.full_frames[key] = self._blob_cache[cache_key]
    for key in bufs.keys():
      self.transforms_np[key][:,:] = transforms[key][:,:]

    out = self.update_imgs(self.img_queues['img'], self.full_frames['img'], self.transforms['img'],
                           self.img_queues['big_img'], self.full_frames['big_img'], self.transforms['big_img'])
    vision_inputs = {'img': out[0], 'big_img': out[1]}

    if prepare_only:
      return None

    self.vision_output = self.vision_run(**vision_inputs).contiguous().realize().uop.base.buffer.numpy().flatten()
    vision_outputs_dict = self.parser.parse_vision_outputs(self.slice_outputs(self.vision_output, self.vision_output_slices))

    self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], self.desire_key: new_desire})
    for k in [self.desire_key, 'features_buffer']:
      self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
    if 'traffic_convention' in self.numpy_inputs:
      self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']
    # op11 추가 입력: action_t (lat/long action_t 시간 지평), prev_action (이전 출력)
    # 모델 메타데이터에 입력 슬롯이 있을 때만 채움 → op7 모델은 영향 없음
    if 'action_t' in self.numpy_inputs:
      action_t_in = inputs.get('action_t')
      if action_t_in is not None:
        self.numpy_inputs['action_t'][:] = action_t_in
      else:
        self.numpy_inputs['action_t'][:] = 0
    if 'prev_action' in self.numpy_inputs:
      prev_action_in = inputs.get('prev_action')
      if prev_action_in is not None:
        self.numpy_inputs['prev_action'][:] = prev_action_in
      else:
        self.numpy_inputs['prev_action'][:] = 0

    # off-policy inference (if present): shares policy_inputs, contributes
    # environment-aware outputs (lane_lines / road_edges / lead / plan).
    # op11: plan only in off_policy.  op7: both off & on may emit plan; merge
    # order below lets on-policy override off-policy where keys overlap.
    off_policy_outputs_dict: dict[str, np.ndarray] = {}
    if self.has_off_policy:
      self.off_policy_output = self.off_policy_run(**self.policy_inputs).contiguous().realize().uop.base.buffer.numpy().flatten()
      off_policy_outputs_dict = self.parser.parse_off_policy_outputs(
        self.slice_outputs(self.off_policy_output, self.off_policy_output_slices))

    self.policy_output = self.policy_run(**self.policy_inputs).contiguous().realize().uop.base.buffer.numpy().flatten()

    # eGPU numerics can degrade silently, so mirror upstream modeld.py and refuse to
    # publish a non-finite frame instead of feeding garbage downstream.
    if USBGPU:
      for name, raw in (('vision', self.vision_output),
                        ('off_policy', self.off_policy_output if self.has_off_policy else None),
                        ('policy', self.policy_output)):
        if raw is not None and not np.all(np.isfinite(raw)):
          raise RuntimeError(f"eGPU model output is not finite ({name})")

    policy_outputs_dict = self.parser.parse_policy_outputs(self.slice_outputs(self.policy_output, self.policy_output_slices))

    # Merge order: vision → off_policy → policy.  On-policy overrides shared
    # keys (e.g. plan, lane_lines) when present.
    combined_outputs_dict = {**vision_outputs_dict, **off_policy_outputs_dict, **policy_outputs_dict}

    # planplus recovery: add extra plan-delta on top of the base plan.
    if 'planplus' in combined_outputs_dict and 'plan' in combined_outputs_dict:
      combined_outputs_dict['plan'] = combined_outputs_dict['plan'] + RECOVERY_POWER * combined_outputs_dict['planplus']
    elif 'planplus' in combined_outputs_dict and 'plan' not in combined_outputs_dict:
      combined_outputs_dict['plan'] = combined_outputs_dict['planplus']
      if 'planplus_stds' in combined_outputs_dict:
        combined_outputs_dict['plan_stds'] = combined_outputs_dict['planplus_stds']

    if SEND_RAW_PRED:
      combined_outputs_dict['raw_pred'] = np.concatenate([self.vision_output.copy(), self.policy_output.copy()])

    return combined_outputs_dict


def main(demo=False):
  cloudlog.warning("modeld init")

  params = Params()
  use_wide_camera = bool(params.get("UseWideCamera", return_default=True))

  if not USBGPU:
    # USB GPU currently saturates a core so can't do this yet,
    # also need to move the aux USB interrupts for good timings
    config_realtime_process(7, 54)

  st = time.monotonic()
  cloudlog.warning("loading model")
  model = ModelState()
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    vipc_client_main_stream, use_extra_client = select_vision_streams(
      available_streams,
      VisionStreamType.VISION_STREAM_ROAD,
      VisionStreamType.VISION_STREAM_WIDE_ROAD,
      use_wide_camera,
    )
    if vipc_client_main_stream is not None:
      break
    time.sleep(.1)

  main_wide_camera = vipc_client_main_stream == VisionStreamType.VISION_STREAM_WIDE_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, use_wide_camera: {use_wide_camera}, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  # messaging
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay", "carrotMan", "radarState"])

  publish_state = PublishState()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  lat_delay = CP.steerActuatorDelay + .2 + LAT_SMOOTH_SECONDS
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()

  frame = 0
  custom_lat_delay = 0.0
  lat_smooth_seconds = LAT_SMOOTH_SECONDS
  vEgoStopping = params.get_float("VEgoStopping") * 0.01
  camera_yaw_trim_deg = params.get_float("CameraYawTrimDeg") * 0.01
  while True:
    frame += 1
    if frame % 100 == 0:
      custom_lat_delay = params.get_float("SteerActuatorDelay") * 0.01
      lat_smooth_seconds = params.get_float("LatSmoothSec") * 0.01
      long_delay = params.get_float("LongActuatorDelay")*0.01
      vEgoStopping = params.get_float("VEgoStopping") * 0.01
      camera_yaw_trim_deg = params.get_float("CameraYawTrimDeg") * 0.01

    if custom_lat_delay > 0.0:
      lat_delay = custom_lat_delay + lat_smooth_seconds
    else:
      lat_delay = sm["liveDelay"].lateralDelay + lat_smooth_seconds

    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                         extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    #lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)

      calib_done = sm["liveCalibration"].calStatus == log.LiveCalibrationData.Status.calibrated
      applied_yaw_trim_deg = camera_yaw_trim_deg if calib_done else 0.0

      if applied_yaw_trim_deg != 0.0:
        device_from_calib_euler[2] -= np.radians(applied_yaw_trim_deg)

      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False).astype(np.float32)
      # 신 modeld.py 와 동일: 와이드캠이 없는 구성에서는 big_img 도 fcam 프레임이므로
      # fcam 내참행렬로 워프해야 기하학적으로 맞다.
      has_wide_camera = use_extra_client or main_wide_camera
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if has_wide_camera else dc.fcam.intrinsics, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
    action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    inputs:dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
      'action_t': np.array([lat_action_t, long_action_t], dtype=np.float32),
      'prev_action': np.array([prev_action.desiredCurvature * max(1.0, v_ego)**2, prev_action.desiredAcceleration], dtype=np.float32),
    }

    mt1 = time.perf_counter()
    model_output = model.run(bufs, transforms, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')

      action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego, lat_smooth_seconds, vEgoStopping)
      prev_action = action
      fill_model_msg(modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], modelv2_send.modelV2, sm['carControl'].latActive, lane_change_prob, sm['carrotMan'], sm['radarState'])
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      modelv2_send.modelV2.meta.desireLog = DH.desireLog #carrot
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      modelv2_send.modelV2.meta.laneWidthLeft = float(DH.left.lane_width)
      modelv2_send.modelV2.meta.laneWidthRight = float(DH.right.lane_width)
      modelv2_send.modelV2.meta.distanceToRoadEdgeLeft = float(DH.left.dist_to_edge)
      modelv2_send.modelV2.meta.distanceToRoadEdgeRight = float(DH.right.dist_to_edge)
      modelv2_send.modelV2.meta.desire = DH.desire
      modelv2_send.modelV2.meta.laneChangeProb = DH.lane_change_ll_prob
      modelv2_send.modelV2.meta.modelTurnSpeed = float(DH.model_turn_speed)
      modelv2_send.modelV2.meta.laneChangeAvailableLeft = DH.lane_change_available_left
      modelv2_send.modelV2.meta.laneChangeAvailableRight = DH.lane_change_available_right
      mt3 = time.perf_counter()
      drivingdata_send.drivingModelData.modelExecutionTime = mt3 - mt1

      fill_driving_model_data(drivingdata_send, modelv2_send)
      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
