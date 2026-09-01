#!/usr/bin/env python3
import os
os.environ['GMMU'] = '0' # for usbgpu fast loading, noop for qcom
from tinygrad.tensor import Tensor
import time
import threading
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
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, WARP_INPUTS, POLICY_INPUTS
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, fill_pose_msg, PublishState
from openpilot.common.file_chunker import open_file_chunked
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.helpers import (get_tg_input_devices, load_oob, modeld_pkl_path,
                                                refresh_usbgpu_device_cache, select_vision_streams, usbgpu_compiled_path,
                                                usbgpu_pcie_not_ready, usbgpu_present, wait_for_usbgpu_present)

PROCESS_NAME = "openpilot.selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')
SIMULATION = os.getenv('SIMULATION') == '1'

LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3
USBGPU_MODEL_LOAD_TIMEOUT = 40
USBGPU_DISCOVERY_GRACE_SECONDS = 5.0
USBGPU_DISCOVERY_POLL_INTERVAL = 0.1
USBGPU_INIT_ATTEMPTS = 6
USBGPU_INIT_RETRY_INTERVAL = 2.0
USBGPU_TMUX_ERROR_REASON = "egpu_error"


def queue_usbgpu_error_tmux(params: Params, context: str) -> bool:
  """Request one tmux capture without replacing another pending diagnostic."""
  try:
    pending_reason = params.get("CarrotException")
    if isinstance(pending_reason, bytes):
      pending_reason = pending_reason.decode("utf-8", errors="ignore")
    if pending_reason in (None, ""):
      params.put("CarrotException", USBGPU_TMUX_ERROR_REASON)
      cloudlog.warning(f"queued {USBGPU_TMUX_ERROR_REASON} tmux capture: {context}")
      return True
    if pending_reason == USBGPU_TMUX_ERROR_REASON:
      return True
    cloudlog.warning(
      f"did not replace pending CarrotException={pending_reason!r} with "
      f"{USBGPU_TMUX_ERROR_REASON}: {context}"
    )
  except Exception:
    cloudlog.exception(f"failed to queue {USBGPU_TMUX_ERROR_REASON} tmux capture: {context}")
  return False


def get_lat_smooth_seconds_dynamic(model_output: dict[str, np.ndarray],
                                   base_lat_smooth_seconds: float) -> tuple[float, float, float]:
  if base_lat_smooth_seconds <= 0.0:
    return 0.0, 0.0, 0.0

  try:
    y_std_1s = float(model_output['plan_stds'][0, 10, Plan.POSITION, 1])
  except Exception:
    y_std_1s = 0.0

  extra_smooth_seconds = float(np.interp(y_std_1s, [0.15, 0.25], [0.0, base_lat_smooth_seconds * 2]))

  dynamic_lat_smooth_seconds = float(np.clip(base_lat_smooth_seconds + extra_smooth_seconds, 0.0, 0.60))

  return dynamic_lat_smooth_seconds, y_std_1s, extra_smooth_seconds


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float, lat_smooth_seconds: float, vEgoStopping: float) -> log.ModelDataV2.Action:
  plan = model_output['plan'][0]
  desired_accel, should_stop, _, desired_velocity_now = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                   plan[:,Plan.ACCELERATION][:,0],
                                                   ModelConstants.T_IDXS,
                                                   action_t=long_action_t,
                                                   vEgoStopping=vEgoStopping)
  if 'action' not in model_output:
    #plan = model_output['plan'][0]
    #desired_accel, should_stop = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
    #                                                 plan[:,Plan.ACCELERATION][:,0],
    #                                                 ModelConstants.T_IDXS,
    #                                                 action_t=long_action_t)
    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
  else:
    desired_accel = model_output['action'][0,1]
    desired_curvature = model_output['action'][0,0] / (max(1.0, v_ego))**2
    #should_stop = (v_ego < 0.3 and desired_accel < 0.1)
  desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
  desired_velocity_now = smooth_value(desired_velocity_now, prev_action.desiredVelocity, LONG_SMOOTH_SECONDS)

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


class ModelState:
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, usbgpu: bool, pkl_path=None):
    input_devices = get_tg_input_devices(PROCESS_NAME, usbgpu)
    self.WARP_DEV, self.QUEUE_DEV = input_devices['WARP_DEV'], input_devices['QUEUE_DEV']
    jits = load_oob(open_file_chunked(pkl_path or modeld_pkl_path(usbgpu)))
    metadata = jits['metadata']
    self.input_shapes = metadata['input_shapes']
    self.vision_input_names = [k for k in self.input_shapes if 'img' in k]
    self.output_slices = metadata['output_slices']

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.usbgpu = usbgpu

    self.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    self.input_queues, self.npy = make_input_queues(self.input_shapes, self.frame_skip, device=self.QUEUE_DEV)
    self.full_frames: dict[str, Tensor] = {}
    self._blob_cache: dict[int, Tensor] = {}
    self.parser = Parser()
    self.frame_buf_params = {k: get_nv12_info(cam_w, cam_h) for k in ('img', 'big_img')}
    self.run_policy = jits['run_policy']
    self.warp = jits[(cam_w,cam_h)]

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}
    return parsed_model_outputs

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    for key in bufs.keys():
      stride, y_height, uv_height, yuv_size = self.frame_buf_params[key]
      frame_data = np.frombuffer(bufs[key].data, dtype=np.uint8)
      if self.WARP_DEV.split(':', 1)[0] in ('CUDA', 'NV'):
        # VisionIPC buffers are host memory on discrete GPUs. from_blob would
        # incorrectly treat their address as a GPU pointer and fault.
        uv_offset = stride * y_height
        if len(frame_data) != yuv_size or bufs[key].stride != stride or bufs[key].uv_offset != uv_offset:
          # PC camera sources may provide tightly packed NV12. Convert them to
          # the Venus layout expected by the compiled model warp.
          src_stride = bufs[key].stride
          src_uv_offset = bufs[key].uv_offset
          packed_frame = np.zeros(yuv_size, dtype=np.uint8)
          src_y_height = min(bufs[key].height, src_uv_offset // src_stride)
          src_uv_height = min(bufs[key].height // 2, max(0, len(frame_data) - src_uv_offset) // src_stride)
          copy_width = min(bufs[key].width, src_stride, stride)
          src_y = frame_data[:src_stride * src_y_height].reshape(src_y_height, src_stride)
          src_uv = frame_data[src_uv_offset:src_uv_offset + src_stride * src_uv_height].reshape(src_uv_height, src_stride)
          dst_y = packed_frame[:uv_offset].reshape(y_height, stride)
          dst_uv = packed_frame[uv_offset:uv_offset + stride * uv_height].reshape(uv_height, stride)
          dst_y[:src_y_height, :copy_width] = src_y[:, :copy_width]
          dst_uv[:src_uv_height, :copy_width] = src_uv[:, :copy_width]
          frame_data = packed_frame
        self.full_frames[key] = Tensor(frame_data, device=self.WARP_DEV).realize()
      else:
        ptr = frame_data.ctypes.data
        # Integrated GPUs can access the VisionIPC ringbuffer directly.
        cache_key = (key, ptr)
        if cache_key not in self._blob_cache:
          self._blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype='uint8', device=self.WARP_DEV)
        self.full_frames[key] = self._blob_cache[cache_key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']
    self.npy['tfm'][:,:] = transforms['img'][:,:]
    self.npy['big_tfm'][:,:] = transforms['big_img'][:,:]

    warped = self.warp(**{k: self.input_queues[k] for k in WARP_INPUTS}, frame=self.full_frames['img'], big_frame=self.full_frames['big_img'])

    if prepare_only:
      return None

    outs, = self.run_policy(
      **{k: self.input_queues[k] for k in POLICY_INPUTS if k in self.input_queues}, warped=warped
    )
    model_output = outs.numpy()[0]
    if self.usbgpu and not np.all(np.isfinite(model_output)):
      raise RuntimeError("eGPU model output is not finite")
    outputs_dict = self.parser.parse_outputs(self.slice_outputs(model_output, self.output_slices))
    self.npy['prev_feat'][:] = model_output[self.output_slices['hidden_state']]

    if SEND_RAW_PRED:
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict


def main(demo=False):
  cloudlog.warning("modeld init")

  params = Params()
  usbgpu_pkl_path = usbgpu_compiled_path()
  _compiled = usbgpu_pkl_path is not None
  _hardware_seen = params.get_bool("UsbGpuHardwareSeen")
  _present = usbgpu_present()
  if not _present and _compiled and _hardware_seen:
    cloudlog.warning(
      f"eGPU USB not present at modeld start; waiting up to {USBGPU_DISCOVERY_GRACE_SECONDS:.1f}s for SuperSpeed enumeration"
    )
    _present = wait_for_usbgpu_present(USBGPU_DISCOVERY_GRACE_SECONDS, USBGPU_DISCOVERY_POLL_INTERVAL)
    if _present:
      cloudlog.warning("eGPU USB enumerated during startup grace period")
    else:
      cloudlog.warning("eGPU USB still absent after startup grace period; using internal model")
  _startup_failed = params.get_bool("UsbGpuStartupFailed")
  USBGPU = _present and _compiled and not _startup_failed
  cloudlog.warning(f"usbgpu present: {_present}, compiled: {_compiled}, startup_failed: {_startup_failed}, requested: {USBGPU}")
  params.put_bool("UsbGpuPresent", _present)
  if _present or _compiled:
    params.put_bool("UsbGpuHardwareSeen", True)
  params.put_bool("UsbGpuCompiled", _compiled)
  params.put_bool("UsbGpuLoading", USBGPU)
  params.put_bool("UsbGpuActive", False)
  use_wide_camera = bool(params.get("UseWideCamera", return_default=True))

  config_realtime_process(7, 54)

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

  st = time.monotonic()
  cloudlog.warning("loading model")
  model = None
  usbgpu_model_loaded = False
  if USBGPU:
    usbgpu_model = None

    def load_usbgpu_model():
      nonlocal usbgpu_model
      for attempt in range(1, USBGPU_INIT_ATTEMPTS + 1):
        try:
          usbgpu_model = ModelState(vipc_client_main.width, vipc_client_main.height, True, usbgpu_pkl_path)
          return
        except Exception as exc:
          if usbgpu_pcie_not_ready(exc) and attempt < USBGPU_INIT_ATTEMPTS:
            cloudlog.warning(f"eGPU PCIe link not ready; retrying ({attempt}/{USBGPU_INIT_ATTEMPTS}): {exc!r}")
            time.sleep(USBGPU_INIT_RETRY_INTERVAL)
            refresh_usbgpu_device_cache()
            continue
          cloudlog.exception("eGPU model load failed")
          queue_usbgpu_error_tmux(params, "model load failed")
          return

    loader = threading.Thread(target=load_usbgpu_model, name="usbgpu-model-loader", daemon=True)
    loader.start()
    loader.join(USBGPU_MODEL_LOAD_TIMEOUT)
    if loader.is_alive():
      cloudlog.error(f"eGPU model load timed out after {USBGPU_MODEL_LOAD_TIMEOUT}s")
      queue_usbgpu_error_tmux(params, "model load timed out")
      params.put_bool("UsbGpuStartupFailed", True)
      params.put_bool("UsbGpuLoading", False)
      # A Python thread cannot be stopped safely. Terminate modeld so the
      # process restart releases every tinygrad/libusb resource, then use the
      # internal model for the rest of this ignition cycle.
      raise RuntimeError("eGPU model loader did not terminate")
    model = usbgpu_model
    if model is None:
      params.put_bool("UsbGpuStartupFailed", True)
    usbgpu_model_loaded = model is not None
    params.put_bool("UsbGpuActive", usbgpu_model_loaded)

  # Keep the internal-GPU model ready so a USB disconnect or runtime error does
  # not take modeld down while driving.
  small_model = ModelState(vipc_client_main.width, vipc_client_main.height, False) if model is None or USBGPU else None
  if model is None:
    model = small_model
  # Loading is not complete until the first model result is published. The
  # first eGPU execution can spend several seconds initializing queues/kernels
  # after the PKL has loaded; clearing this here causes a false commIssue while
  # modelV2 and its downstream services are still waiting for their first data.
  usbgpu_startup_pending = usbgpu_model_loaded
  if not usbgpu_startup_pending:
    params.put_bool("UsbGpuLoading", False)
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # messaging
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay", "carrotMan", "radarState"])

  publish_state = PublishState()
  params = Params()

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
  lat_delay_dynamic = lat_smooth_seconds
  while True:
    frame += 1
    if frame % 100 == 0:
      custom_lat_delay = params.get_float("SteerActuatorDelay") * 0.01
      lat_smooth_seconds = params.get_float("LatSmoothSec") * 0.01
      long_delay = params.get_float("LongActuatorDelay")*0.01
      vEgoStopping = params.get_float("VEgoStopping") * 0.01
      camera_yaw_trim_deg = params.get_float("CameraYawTrimDeg") * 0.01
      # eGPU power follows ignition on the vehicle. Keep UI state current when
      # the shared USB hub is connected or removed after modeld starts.
      usbgpu_present_now = usbgpu_present()
      params.put_bool_nonblocking("UsbGpuPresent", usbgpu_present_now)
      if usbgpu_present_now:
        params.put_bool_nonblocking("UsbGpuHardwareSeen", True)
      params.put_bool_nonblocking("UsbGpuCompiled", usbgpu_compiled_path() is not None)

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
    lat_action_t = lat_delay_dynamic + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    inputs: dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
      'action_t': np.array([lat_action_t, long_action_t], dtype=np.float32),
    }

    mt1 = time.perf_counter()
    try:
      model_output = model.run(bufs, transforms, inputs, prepare_only)
    except Exception:
      if not params.get_bool("UsbGpuActive") or small_model is None:
        raise
      cloudlog.exception("eGPU model failed, falling back to internal GPU")
      queue_usbgpu_error_tmux(params, "runtime model execution failed")
      params.put_bool("UsbGpuActive", False)
      params.put_bool("UsbGpuStartupFailed", True)
      params.put_bool("UsbGpuLoading", False)
      usbgpu_startup_pending = False
      model = small_model
      run_count = 0
      # Run the already-loaded internal model for this same camera frame. A
      # missing modelV2 frame during fallback can otherwise cascade into a
      # misleading communication/CAN error while selfdrived waits for modeld.
      model_output = model.run(bufs, transforms, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')

      lat_smooth_seconds_dynamic, y_std_1s, lat_smooth_extra = get_lat_smooth_seconds_dynamic(
          model_output,
          lat_smooth_seconds,
        )
      if custom_lat_delay > 0.0:
        lat_delay_dynamic = custom_lat_delay + lat_smooth_seconds_dynamic
      else:
        lat_delay_dynamic = sm["liveDelay"].lateralDelay + lat_smooth_seconds_dynamic

      action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego, lat_smooth_seconds_dynamic, vEgoStopping)
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
      # Slow PC inference can exceed locationd's rewind window. The simulator
      # pose represents current simulated motion, so timestamp it at publish.
      pose_timestamp_eof = time.monotonic_ns() if SIMULATION else meta_main.timestamp_eof
      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, pose_timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
      if usbgpu_startup_pending:
        # Clear only after all first-frame outputs are on the bus so selfdrived
        # cannot observe "ready" before modelV2 and its dependants can run.
        params.put_bool("UsbGpuLoading", False)
        usbgpu_startup_pending = False
        cloudlog.warning("eGPU first model output published; startup complete")
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
