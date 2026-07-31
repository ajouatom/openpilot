import time
import numpy as np
import pyray as rl
from openpilot.cereal import log, messaging
from msgq.visionipc import VisionStreamType
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.carrot_param_cache import BorderParamSnapshot, TimedSnapshotCache, read_border_params
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_draw import draw_text_ui_style
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

OpState = log.SelfdriveState.OpenpilotState
CALIBRATED = log.LiveCalibrationData.Status.calibrated
ROAD_CAM = VisionStreamType.VISION_STREAM_ROAD
WIDE_CAM = VisionStreamType.VISION_STREAM_WIDE_ROAD
DEFAULT_DEVICE_CAMERA = DEVICE_CAMERAS["tici", "ar0231"]

BORDER_COLORS = {
  UIStatus.DISENGAGED: rl.Color(0x12, 0x28, 0x39, 0xFF),  # Blue for disengaged state
  UIStatus.OVERRIDE: rl.Color(0x89, 0x92, 0x8D, 0xFF),  # Gray for override state
  UIStatus.ENGAGED: rl.Color(0x16, 0x7F, 0x40, 0xFF),  # Green for engaged state
}

WIDE_CAM_MAX_SPEED = 10.0  # m/s (22 mph)
ROAD_CAM_MIN_SPEED = 15.0  # m/s (34 mph)
INF_POINT = np.array([1000.0, 0.0, 0.0])


class AugmentedRoadView(CameraView):
  def __init__(self, stream_type: VisionStreamType = VisionStreamType.VISION_STREAM_ROAD):
    super().__init__("camerad", stream_type)
    self._set_placeholder_color(BORDER_COLORS[UIStatus.DISENGAGED])

    self.device_camera: DeviceCameraConfig | None = None
    self.view_from_calib = view_frame_from_device_frame.copy()
    self.view_from_wide_calib = view_frame_from_device_frame.copy()

    self._matrix_cache_key = (0, 0.0, 0.0, stream_type)
    self._cached_matrix: np.ndarray | None = None
    self._content_rect = rl.Rectangle()
    self._suppress_camera_for_cluster = False

    self.model_renderer = ModelRenderer()
    self._hud_renderer = HudRenderer()
    self.alert_renderer = AlertRenderer()
    self.driver_state_renderer = DriverStateRenderer()

    # debug
    self._pm = messaging.PubMaster(['uiDebug'])
    # uiDebug.plotMode용 ShowPlotMode 캐시 — 파일 읽기라 매 프레임 금지, ~2초 스로틀
    self._plot_mode = 0
    self._plot_mode_next_t = 0.0
    self._border_params = TimedSnapshotCache(
      BorderParamSnapshot(),
      lambda: read_border_params(ui_state.params, ui_state.params_memory),
    )

  def _refresh_plot_mode(self, now: float) -> None:
    if now < self._plot_mode_next_t:
      return
    self._plot_mode_next_t = now + 2.0
    try:
      self._plot_mode = min(max(ui_state.params.get_int("ShowPlotMode"), 0), 255)
    except Exception:
      self._plot_mode = 0

  def set_cluster_hud_connected(self, connected: bool, show_camera: bool = False) -> None:
    self._suppress_camera_for_cluster = connected and not show_camera

  def _render(self, rect):
    # plotMode 갱신(가끔 파일 읽기)은 drawTime 창 밖에서 — total을 오염시키지 않는다
    self._refresh_plot_mode(time.monotonic())
    # Only render when system is started to avoid invalid data access
    start_draw = time.monotonic()
    if not ui_state.started:
      return
    # 구간별 계측(계측 전용) — 렌더 호출 순서는 그대로, 각 구간 전후 monotonic만 잰다.
    # scissor begin/end는 raylib 배치 flush 지점이라 특정 구간에 귀속시키지 않는다 —
    # total과 구간 합의 차이(미귀속)로 남는다. extras = carrot 테두리(+텍스트)
    cam_ms = model_ms = ds_ms = hud_ms = alert_ms = extras_ms = 0.0

    if not self._suppress_camera_for_cluster:
      self._switch_stream_if_needed(ui_state.sm)
      self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      rect.x + UI_BORDER_SIZE,
      rect.y + UI_BORDER_SIZE,
      rect.width - 2 * UI_BORDER_SIZE,
      rect.height - 2 * UI_BORDER_SIZE,
    )

    # Enable scissor mode to clip all rendering within content rectangle boundaries
    # This creates a rendering viewport that prevents graphics from drawing outside the border
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    _t = time.monotonic()
    if self._suppress_camera_for_cluster:
      rl.draw_rectangle_rec(self._content_rect, rl.BLACK)
    else:
      # Render the base camera view
      super()._render(rect)
    cam_ms = (time.monotonic() - _t) * 1000.0

    if not self._suppress_camera_for_cluster:
      # Draw the model overlay only with the camera view
      _t = time.monotonic()
      self.model_renderer.render(self._content_rect)
      model_ms = (time.monotonic() - _t) * 1000.0
    _t = time.monotonic()
    self._hud_renderer.render(self._content_rect)  # plot 활성 시 plot 비용도 hud 구간에 포함
    hud_ms = (time.monotonic() - _t) * 1000.0
    _t = time.monotonic()
    self.alert_renderer.render(self._content_rect)
    alert_ms = (time.monotonic() - _t) * 1000.0
    _t = time.monotonic()
    self.driver_state_renderer.render(self._content_rect)
    ds_ms = (time.monotonic() - _t) * 1000.0

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds

    # End clipping region
    rl.end_scissor_mode()

    # Draw colored border based on driving state
    _t = time.monotonic()
    self._draw_border_carrot(rect)
    extras_ms = (time.monotonic() - _t) * 1000.0

    # publish uiDebug
    msg = messaging.new_message('uiDebug', valid=True)
    ud = msg.uiDebug
    ud.drawTimeMillis = (time.monotonic() - start_draw) * 1000
    ud.cameraTimeMillis = cam_ms
    ud.modelTimeMillis = model_ms
    ud.driverStateTimeMillis = ds_ms
    ud.hudTimeMillis = hud_ms
    ud.alertTimeMillis = alert_ms
    ud.extrasTimeMillis = extras_ms
    ud.plotMode = self._plot_mode
    ud.recording = gui_app.is_recording()
    self._pm.send('uiDebug', msg)

  def _handle_mouse_press(self, _):
    if not self._hud_renderer.user_interacting() and self._click_callback is not None:
      self._click_callback()

  def _handle_mouse_release(self, _):
    # We only call click callback on press if not interacting with HUD
    pass

  def _draw_border(self, rect: rl.Rectangle):
    rl.draw_rectangle_lines_ex(rect, UI_BORDER_SIZE, rl.BLACK)
    border_roundness = 0.12
    border_color = BORDER_COLORS.get(ui_state.status, BORDER_COLORS[UIStatus.DISENGAGED])
    border_rect = rl.Rectangle(rect.x + UI_BORDER_SIZE, rect.y + UI_BORDER_SIZE,
                               rect.width - 2 * UI_BORDER_SIZE, rect.height - 2 * UI_BORDER_SIZE)
    rl.draw_rectangle_rounded_lines_ex(border_rect, border_roundness, 10, UI_BORDER_SIZE, border_color)

  def _switch_stream_if_needed(self, sm):
    if sm['selfdriveState'].experimentalMode and WIDE_CAM in self.available_streams:
      v_ego = sm['carState'].vEgo
      if v_ego < WIDE_CAM_MAX_SPEED:
        target = WIDE_CAM
      elif v_ego > ROAD_CAM_MIN_SPEED:
        target = ROAD_CAM
      else:
        # Hysteresis zone - keep current stream
        target = self.stream_type
    else:
      target = ROAD_CAM

    if self.stream_type != target:
      self.switch_stream(target)

  def _update_calibration(self):
    # Update device camera if not already set
    sm = ui_state.sm
    if not self.device_camera and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      self.device_camera = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]

    # Check if live calibration data is available and valid
    if not (sm.updated["liveCalibration"] and sm.valid['liveCalibration']):
      return

    calib = sm['liveCalibration']
    if len(calib.rpyCalib) != 3 or calib.calStatus != CALIBRATED:
      return

    # Update view_from_calib matrix
    device_from_calib = rot_from_euler(calib.rpyCalib)
    self.view_from_calib = view_frame_from_device_frame @ device_from_calib

    # Update wide calibration if available
    if hasattr(calib, 'wideFromDeviceEuler') and len(calib.wideFromDeviceEuler) == 3:
      wide_from_device = rot_from_euler(calib.wideFromDeviceEuler)
      self.view_from_wide_calib = view_frame_from_device_frame @ wide_from_device @ device_from_calib

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    # Check if we can use cached matrix
    cache_key = (
      ui_state.sm.recv_frame['liveCalibration'],
      self._content_rect.width,
      self._content_rect.height,
      self.stream_type
    )
    if cache_key == self._matrix_cache_key and self._cached_matrix is not None:
      return self._cached_matrix

    # Get camera configuration
    device_camera = self.device_camera or DEFAULT_DEVICE_CAMERA
    is_wide_camera = self.stream_type == WIDE_CAM
    intrinsic = device_camera.ecam.intrinsics if is_wide_camera else device_camera.fcam.intrinsics
    calibration = self.view_from_wide_calib if is_wide_camera else self.view_from_calib
    zoom = 2.0 if is_wide_camera else 1.1

    # Calculate transforms for vanishing point
    calib_transform = intrinsic @ calibration
    kep = calib_transform @ INF_POINT

    # Calculate center points and dimensions
    x, y = self._content_rect.x, self._content_rect.y
    w, h = self._content_rect.width, self._content_rect.height
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]

    # Calculate max allowed offsets with margins
    margin = 5
    max_x_offset = cx * zoom - w / 2 - margin
    max_y_offset = cy * zoom - h / 2 - margin

    # Calculate and clamp offsets to prevent out-of-bounds issues
    try:
      if abs(kep[2]) > 1e-6:
        x_offset = np.clip((kep[0] / kep[2] - cx) * zoom, -max_x_offset, max_x_offset)
        y_offset = np.clip((kep[1] / kep[2] - cy) * zoom, -max_y_offset, max_y_offset)
      else:
        x_offset, y_offset = 0, 0
    except (ZeroDivisionError, OverflowError):
      x_offset, y_offset = 0, 0

    # Cache the computed transformation matrix to avoid recalculations
    self._matrix_cache_key = cache_key
    self._cached_matrix = np.array([
      [zoom * 2 * cx / w, 0, -x_offset / w * 2],
      [0, zoom * 2 * cy / h, -y_offset / h * 2],
      [0, 0, 1.0]
    ])

    video_transform = np.array([
      [zoom, 0.0, (w / 2 + x - x_offset) - (cx * zoom)],
      [0.0, zoom, (h / 2 + y - y_offset) - (cy * zoom)],
      [0.0, 0.0, 1.0]
    ])
    self.model_renderer.set_transform(video_transform @ calib_transform)

    return self._cached_matrix

  def _get_border_color(self, status: UIStatus) -> rl.Color:
    return BORDER_COLORS.get(status, BORDER_COLORS[UIStatus.DISENGAGED])


  def _draw_border_carrot(self, rect: rl.Rectangle):
    sm = ui_state.sm
    if not sm.alive["carState"]:
      self._draw_border(rect)
      return

    border_params = self._border_params.refresh(time.monotonic())
    car_state = sm["carState"]

    x = float(rect.x)
    y = float(rect.y)
    w = float(rect.width)
    h = float(rect.height)

    mid_y = y + h / 2.0
    center_gap = 200.0
    gap_half = center_gap / 2.0

    thickness = float(UI_BORDER_SIZE)
    roundness = 0.18
    segments = 10

    # ---------- colors ----------
    if car_state.steeringPressed:
      top_color = self._get_border_color(UIStatus.OVERRIDE)
    elif ui_state.lat_active:
      top_color = self._get_border_color(UIStatus.ENGAGED)
    else:
      top_color = self._get_border_color(UIStatus.DISENGAGED)

    bottom_color = self._get_border_color(ui_state.status)

    left_blink = bool(car_state.leftBlinker)
    right_blink = bool(car_state.rightBlinker)

    # ---------- geometry ----------
    top_h = max(0.0, mid_y - gap_half - y)
    bottom_y = mid_y + gap_half
    bottom_h = max(0.0, y + h - bottom_y)

    # blinker box
    blink_w = UI_BORDER_SIZE
    blink_h = center_gap
    blink_y = mid_y - blink_h / 2.0

    left_blink_rect = rl.Rectangle(
      x,
      blink_y,
      blink_w,
      blink_h
    )
    right_blink_rect = rl.Rectangle(
      x + w - blink_w,
      blink_y,
      blink_w,
      blink_h
    )

    # ---------- top inverted U ----------
    # top horizontal
    rl.draw_rectangle(
      int(x),
      int(y),
      int(w),
      int(thickness),
      top_color
    )

    # top left vertical
    rl.draw_rectangle(
      int(x),
      int(y),
      int(thickness),
      int(top_h),
      top_color
    )

    # top right vertical
    rl.draw_rectangle(
      int(x + w - thickness),
      int(y),
      int(thickness),
      int(top_h),
      top_color
    )

    # ---------- bottom U ----------
    # bottom left vertical
    rl.draw_rectangle(
      int(x),
      int(bottom_y),
      int(thickness),
      int(bottom_h),
      bottom_color
    )

    # bottom right vertical
    rl.draw_rectangle(
      int(x + w - thickness),
      int(bottom_y),
      int(thickness),
      int(bottom_h),
      bottom_color
    )

    # bottom horizontal
    rl.draw_rectangle(
      int(x),
      int(y + h - thickness),
      int(w),
      int(thickness),
      bottom_color
    )

    # ---------- blinkers ----------
    rl.draw_rectangle_rounded(
      left_blink_rect,
      roundness,
      segments,
      rl.ORANGE if left_blink else rl.BLACK
    )
    rl.draw_rectangle_rounded_lines_ex(
      left_blink_rect,
      roundness,
      segments,
      1.0,
      top_color
    )

    rl.draw_rectangle_rounded(
      right_blink_rect,
      roundness,
      segments,
      rl.ORANGE if right_blink else rl.BLACK
    )
    rl.draw_rectangle_rounded_lines_ex(
      right_blink_rect,
      roundness,
      segments,
      1.0,
      top_color
    )

    # ---------- text ----------
    text_margin = 30.0
    font_size = 30.0
    line_margin = 2.0

    top = str(car_state.logCarrot)
    top_left = ""
    top_right = ""
    bottom = ""
    bottom_left = ""
    bottom_right = ""

    top_left = border_params.top_left
    try:
      if sm.alive["carParams"] and sm["carParams"].openpilotLongitudinalControl:
        top_left = border_params.top_left_op_long
    except Exception:
      pass

    try:
      top_right_parts = []

      if sm.alive["liveDelay"]:
        live_delay = sm["liveDelay"]
        top_right_parts.append(
          f"LD[{live_delay.calPerc:.0f}%,{live_delay.lateralDelay:.2f}]"
        )

      if sm.alive["liveTorqueParameters"]:
        ltp = sm["liveTorqueParameters"]
        live_valid = "ON" if ltp.liveValid else "OFF"
        top_right_parts.append(
          f"LT[{ltp.calPerc:.0f}%,{live_valid}]({ltp.latAccelFactorFiltered:.2f}/{ltp.frictionCoefficientFiltered:.2f})"
        )

      if sm.alive["liveParameters"]:
        lp = sm["liveParameters"]
        top_right_parts.append(f"SR({lp.steerRatio:.1f},{border_params.custom_sr:.1f})")

      top_right = ", ".join(top_right_parts)
    except Exception:
      print("Error accessing live debug data for top right text")
      top_right = ""

    if sm.alive["lateralPlan"]:
      lat_plan = sm["lateralPlan"]
      bottom = str(lat_plan.latDebugText)

    bottom_left = border_params.bottom_left
    bottom_right = border_params.bottom_right

    # text positions
    top_text_y = y + line_margin
    bottom_text_y = bottom_y + bottom_h - font_size - 2
    draw_text_ui_style(top, x + w / 2.0, top_text_y, font_size, rl.WHITE,
                       align="center_top", y_offset=0.0)
    draw_text_ui_style(top_left, x + text_margin, top_text_y, font_size, rl.WHITE,
                       align="left_top", y_offset=0.0)
    draw_text_ui_style(top_right, x + w - text_margin, top_text_y, font_size, rl.WHITE,
                       align="right_top", y_offset=0.0)

    draw_text_ui_style(bottom, x + w / 2.0, bottom_text_y, font_size, rl.WHITE,
                       align="center_top", y_offset=0.0)
    draw_text_ui_style(bottom_left, x + text_margin, bottom_text_y, font_size, rl.WHITE,
                       align="left_top", y_offset=0.0)
    draw_text_ui_style(bottom_right, x + w - text_margin, bottom_text_y, font_size, rl.WHITE,
                       align="right_top", y_offset=0.0)

if __name__ == "__main__":
  gui_app.init_window("OnRoad Camera View")
  road_camera_view = AugmentedRoadView(ROAD_CAM)
  gui_app.push_widget(road_camera_view)
  print("***press space to switch camera view***")
  try:
    for _ in gui_app.render():
      ui_state.update()
      if rl.is_key_released(rl.KeyboardKey.KEY_SPACE):
        if WIDE_CAM in road_camera_view.available_streams:
          stream = ROAD_CAM if road_camera_view.stream_type == WIDE_CAM else WIDE_CAM
          road_camera_view.switch_stream(stream)
  finally:
    road_camera_view.close()
