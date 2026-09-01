import json
import time
import numpy as np
import pyray as rl
from dataclasses import dataclass
from typing import Optional
from openpilot.common.constants import CV
from openpilot.selfdrive.carrot.deceleration_source import (
  deceleration_source_presentation,
  external_navigation_connected,
  navigation_status_presentation,
)
# from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar # 아이콘에 토크 적용: 토크바 미사용
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.hardware.usbgpu import usbgpu_badge_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.text_draw import draw_text_ui_style
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.cereal import log
from openpilot.common.params import Params
from datetime import datetime
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds
DEFAULT_MAX_LAT_ACCEL = 3.0  # m/s^2
CRUISE_SPEED_ANIMATION_START = 120
CRUISE_SPEED_ANIMATION_MAX = 100
CRUISE_SPEED_ANIMATION_STEP = 12
CRUISE_SPEED_ANIMATION_START_SIZE = 96
CRUISE_SPEED_ANIMATION_TARGET_SIZE = 40

@dataclass(frozen=True)
class SetSpeedOverrideState:
  active: bool
  speed_kph: float
  label: str
  speed_color_mode: int # 0: white, 1: eco green, 2: orange, 3: vehicle-navigation blue, 4: external-navigation green
  force_persist: bool


class SetSpeedOverride:

  def compute(self, sm, set_speed_kph: float) -> SetSpeedOverrideState:
    # 1) eco (highest)
    cruise_target = None
    try:
      cruise_target = float(sm['longitudinalPlan'].cruiseTarget)
    except Exception:
      cruise_target = None

    if cruise_target is not None and cruise_target > (set_speed_kph + 0.5):
      return SetSpeedOverrideState(
        active=True,
        speed_kph=cruise_target,
        label="eco",
        speed_color_mode=1,
        force_persist=True,   # eco 조건 유지되는 동안 계속 표시
      )

    # 2) apply_speed (desiredSpeed/source)
    desired_speed = None
    desired_source = ""
    try:
      desired_speed = float(sm['carrotMan'].desiredSpeed)
      desired_source = str(sm['carrotMan'].desiredSource or "")
    except Exception:
      desired_speed = None
      desired_source = ""

    if desired_speed is not None and 0 < desired_speed < 200 and desired_speed < set_speed_kph:
      label, speed_color_mode = deceleration_source_presentation(desired_source)
      return SetSpeedOverrideState(
        active=True,
        speed_kph=desired_speed,
        label=label,
        speed_color_mode=speed_color_mode,
        force_persist=True,   # 조건 유지되는 동안 계속 표시
      )

    # 3) default
    return SetSpeedOverrideState(
      active=False,
      speed_kph=set_speed_kph,
      label=tr("MAX"),
      speed_color_mode=0,
      force_persist=False,
    )

@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self._debug_speed_panel = False
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    # self._torque_bar = TorqueBar() # 아이콘에 토크 적용: 토크바 미사용
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps) # 아이콘에 토크 적용: LowPassFilter

    # 휠 당근 휠로 변경
    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/carrot_wheel.png', 50, 50) # 당근 휠
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/carrot_wheel_critical.png', 50, 50) # 당근 휠 위험
    self._txt_wheel_lane: rl.Texture = gui_app.texture('icons_mici/carrot_wheel_lane.png', 100, 50) # 당근 레인모드
    self._txt_wheel_cap: rl.Texture = gui_app.texture('icons_mici/carrot_wheel_cap.png', 50, 50) # 당근 휠 중앙 당근맨

    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 44, 44)

    # Bottom-left speed panel background
    self._txt_speed_bg: rl.Texture = gui_app.texture('images/speed_bg.png', 307, 115)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_override = SetSpeedOverride()
    self._debug_traffic_light = False

    self._cruise_speed_text_last = ""
    self._cruise_speed_animation_text = ""
    self._cruise_speed_animation_time = -1

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self._engaged and self.is_cruise_set:
      display_set_speed = self.set_speed * (1.0 if ui_state.is_metric else KM_TO_MILE)
      cruise_text = str(int(round(display_set_speed)))
    else:
      cruise_text = "--"
    self._update_cruise_speed_animation(cruise_text)

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    # 토크 상태 계산 (휠 아이콘 크기 조절용) from TorqueBar()
    if controls_state.lateralControlState.which() == 'angleState':
      live_parameters = sm['liveParameters']
      car_control = sm['carControl']

      actual_lateral_accel = controls_state.curvature * car_state.vEgo ** 2
      desired_lateral_accel = controls_state.desiredCurvature * car_state.vEgo ** 2
      accel_diff = (desired_lateral_accel - actual_lateral_accel)

      roll_compensation = live_parameters.roll * ACCELERATION_DUE_TO_GRAVITY * np.interp(car_state.vEgo, [5, 15], [0.0, 1.0])
      lateral_acceleration = actual_lateral_accel - roll_compensation
      max_lateral_acceleration = ui_state.CP.maxLateralAccel if ui_state.CP else DEFAULT_MAX_LAT_ACCEL

      if car_control.latActive:
        self._torque_filter.update(float(np.clip((lateral_acceleration + accel_diff) / max_lateral_acceleration, -1.0, 1.0)))
      else:
        self._torque_filter.update(0.0)
    else:
      self._torque_filter.update(float(-sm['carOutput'].actuatorsOutput.torque))

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    # self._torque_bar.render(rect) # 아이콘에 토크 적용: 토크바 미사용

    # bottom-left panel (speed_bg)
    self._draw_set_speed(rect)

    self._draw_steering_wheel(rect)

    self._draw_egpu_badge(rect)

    self._draw_cruise_speed_animation(rect)

  def _draw_egpu_badge(self, rect: rl.Rectangle) -> None:
    # Keep runtime state visible while the shared USB hub re-enumerates; a
    # transient missing sysfs sample must not hide loading or failure details.
    if not (ui_state.usbgpu_present or ui_state.usbgpu_active or
            ui_state.usbgpu_loading or ui_state.usbgpu_startup_failed):
      return

    state = usbgpu_badge_state(ui_state.usbgpu_compiled, ui_state.usbgpu_loading,
                               ui_state.usbgpu_active, ui_state.usbgpu_startup_failed,
                               ui_state.usbgpu_compile_pending)
    text = "eGPU REBOOT" if state == "compile_pending" else "eGPU"
    font_size = 22
    text_size = measure_text_cached(self._font_semi_bold, text, font_size)
    pad_x, pad_y = 10, 5
    badge_w = text_size.x + pad_x * 2
    badge = rl.Rectangle(
      rect.x + rect.width - badge_w - 24,
      rect.y + 12,
      badge_w,
      text_size.y + pad_y * 2,
    )
    color = {
      "active": rl.Color(0, 255, 0, 230),
      "loading": rl.Color(255, 255, 0, 230),
      "error": rl.Color(255, 0, 0, 230),
      "compile_pending": rl.Color(255, 165, 0, 230),
      "not_compiled": rl.Color(255, 165, 0, 230),
      "ready": rl.Color(255, 255, 255, 210),
    }[state]
    rl.draw_rectangle_rounded(badge, 0.35, 8, rl.Color(0, 0, 0, 150))
    rl.draw_rectangle_rounded_lines_ex(badge, 0.35, 8, 2, color)
    rl.draw_text_ex(
      self._font_semi_bold,
      text,
      rl.Vector2(badge.x + pad_x, badge.y + pad_y),
      font_size,
      0,
      color,
    )

  def _update_cruise_speed_animation(self, cruise_text: str) -> None:
    if self._cruise_speed_text_last == cruise_text:
      return

    self._cruise_speed_text_last = cruise_text
    if cruise_text == "--":
      return

    self._cruise_speed_animation_text = cruise_text
    self._cruise_speed_animation_time = CRUISE_SPEED_ANIMATION_START

  def _draw_cruise_speed_animation(self, rect: rl.Rectangle) -> None:
    if self._cruise_speed_animation_time <= 0 or not self._cruise_speed_animation_text:
      return

    self._cruise_speed_animation_time -= CRUISE_SPEED_ANIMATION_STEP
    animation_time = self._cruise_speed_animation_time
    interpolation_time = min(animation_time, CRUISE_SPEED_ANIMATION_MAX)

    panel_x = rect.x + 10.0
    panel_y = rect.y + rect.height - self._txt_speed_bg.height - 10.0
    start_x = rect.x + rect.width * 0.5
    start_y = rect.y + rect.height * 0.45
    target_x = panel_x + self._txt_speed_bg.width * 0.76
    target_y = panel_y + self._txt_speed_bg.height * 0.33

    x = int((start_x * interpolation_time + target_x * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
            CRUISE_SPEED_ANIMATION_MAX)
    y = int((start_y * interpolation_time + target_y * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
            CRUISE_SPEED_ANIMATION_MAX)
    font_size = int((CRUISE_SPEED_ANIMATION_START_SIZE * interpolation_time +
                     CRUISE_SPEED_ANIMATION_TARGET_SIZE * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
                    CRUISE_SPEED_ANIMATION_MAX)

    if animation_time >= CRUISE_SPEED_ANIMATION_MAX:
      border_width = 3.0
      shadow_offset = 3.0
    else:
      border_width = 1.0
      shadow_offset = 0.0

    draw_text_ui_style(
      self._cruise_speed_animation_text,
      x,
      y,
      font_size,
      rl.Color(0, 255, 0, 230),
      font=self._font_display,
      border_width=border_width,
      shadow_offset=shadow_offset,
      align="center",
      y_offset=0.0,
    )

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    # Always visible (no hide). We keep filters but drive them to stable values.
    self._wheel_alpha_filter.update(255 * 0.95)
    self._wheel_y_filter.update(0)

    # pos (TOP-left)
    margin_x = 18
    margin_y = 18
    pos_x = int(rect.x + margin_x + wheel_txt.width / 2)
    pos_y = int(rect.y + margin_y + wheel_txt.height / 2 + self._wheel_y_filter.x)

    self._draw_steering_wheel_icon(wheel_txt, pos_x, pos_y)
    self._draw_wheel_side_info(wheel_txt, pos_x, pos_y)


  def _draw_steering_wheel_icon(self, wheel_txt, pos_x: int, pos_y: int) -> None:
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    torque_val = abs(self._torque_filter.x)
    # 토크가 0.5 넘어가면 휠 아이콘을 서서히 1.5배까지 키우기
    scale = float(np.interp(torque_val, [0.5, 1.0], [1.0, 1.5]))
    scaled_width = wheel_txt.width * scale
    scaled_height = wheel_txt.height * scale

    turn_intent_margin = 25 * scale
    self._turn_intent.render(rl.Rectangle(
      pos_x - scaled_width / 2 - turn_intent_margin,
      pos_y - scaled_height / 2 - turn_intent_margin,
      scaled_width + turn_intent_margin * 2,
      scaled_height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, scaled_width, scaled_height)
    origin = (scaled_width / 2, scaled_height / 2)

    if ui_state.lat_active:
      # 토크 정도에 따라 녹색 -> 주황색 블렌딩
      green_color = rl.Color(0, 255, 0, int(self._wheel_alpha_filter.x))
      orange_color = rl.Color(255, 115, 0, int(self._wheel_alpha_filter.x))
      blend_factor = float(np.clip((torque_val - 0.75) * 4.0, 0.0, 1.0))
      wheel_color = blend_colors(green_color, orange_color, blend_factor)
    else:
      wheel_color = rl.Color(230, 230, 230, int(self._wheel_alpha_filter.x))

    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, wheel_color)
    # 당근맨은 틴팅 없이 덧대서 그리기
    rl.draw_texture_pro(self._txt_wheel_cap, src_rect, dest_rect, origin, rotation, rl.WHITE)

    if self._show_wheel_critical:
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)
    # 속도패널 디버깅 모드거나 레인모드일 때 차선 이미지 추가
    elif self._debug_speed_panel or bool(ui_state.sm['controlsState'].activeLaneLine):
      LANE_TOP_OFFSET = 3
      lane_pos_x = pos_x - self._txt_wheel_lane.width / 2
      lane_pos_y = pos_y - self._txt_wheel_lane.height / 2 - LANE_TOP_OFFSET
      rl.draw_texture_ex(self._txt_wheel_lane, rl.Vector2(lane_pos_x, lane_pos_y), 0.0, 1.0, wheel_color)


  def _get_cpu_temp_text(self) -> str:
    try:
      ds = ui_state.sm['deviceState']
      cpu_temps = getattr(ds, 'cpuTempC', None)

      if cpu_temps is not None and len(cpu_temps) > 0:
        valid_temps = [float(t) for t in cpu_temps]
        if len(valid_temps) > 0:
          cpu_temp = sum(valid_temps) / float(len(valid_temps))
          return f"CPU: {cpu_temp:.0f}"
    except Exception:
      pass

    return "CPU: --"


  def _draw_wheel_side_info(self, wheel_txt, pos_x: int, pos_y: int) -> None:
    now = datetime.now()

    try:
      show_date_time = int(ui_state.show_date_time)
    except Exception:
      show_date_time = 1

    try:
      show_debug_ui = int(ui_state.show_debug_ui)
    except Exception:
      show_debug_ui = 0

    time_font = int(wheel_txt.height * 1.1)
    small_dt_font = max(18, int(time_font * 0.62))   # date+time 2줄용
    side_font = max(18, int(time_font * 0.33))

    time_x = pos_x + wheel_txt.width / 2 + 15

    # --------------------------------------------------------------------------
    # Date / Time
    # show_date_time: 0=hide, 1=date+time, 2=time only, 3=date only
    # --------------------------------------------------------------------------
    time_block_right = time_x

    if show_date_time != 0:
      weekdays_ko = ["일", "월", "화", "수", "목", "금", "토"]
      # Python weekday(): 월=0 ... 일=6 이라서 C tm_wday 스타일로 변환
      weekday = weekdays_ko[(now.weekday() + 1) % 7]

      time_text = now.strftime("%H:%M")
      date_text = now.strftime(f"%m-%d({weekday})")

      if show_date_time == 1:
        # 시간 + 날짜: 시간은 조금 크게, 날짜는 조금 작게
        time_font = int(wheel_txt.height * 1.05)
        date_font = max(18, int(time_font * 0.58))

        time_size = measure_text_cached(self._font_display, time_text, time_font)
        date_size = measure_text_cached(self._font_display, date_text, date_font)

        line_gap = max(2, int(time_font * 0.02))
        total_h = time_size.y + line_gap + date_size.y
        base_y = pos_y - total_h / 2

        block_w = max(time_size.x, date_size.x)

        draw_time_x = time_x + (block_w - time_size.x) / 2
        date_x = time_x + (block_w - date_size.x) / 2

        time_y = base_y
        date_y = time_y + time_size.y + line_gap

        draw_text_ui_style(
          time_text, draw_time_x, time_y, time_font,
          rl.Color(255, 255, 255, 235),
          font=self._font_display,
          border_width=1.0,
          shadow_offset=3.0,
          align="left_top",
          y_offset=0.0,
        )

        draw_text_ui_style(
          date_text, date_x, date_y, date_font,
          rl.Color(255, 255, 255, 220),
          font=self._font_display,
          border_width=1.0,
          shadow_offset=3.0,
          align="left_top",
          y_offset=0.0,
        )

        time_block_right = time_x + block_w

      elif show_date_time == 2:
        # 시간만: 크게
        text_font = int(wheel_txt.height * 1.1)
        time_size = measure_text_cached(self._font_display, time_text, text_font)
        time_y = pos_y - time_size.y / 2

        draw_text_ui_style(
          time_text, time_x, time_y, text_font,
          rl.Color(255, 255, 255, 235),
          font=self._font_display,
          border_width=1.0,
          shadow_offset=3.0,
          align="left_top",
          y_offset=0.0,
        )

        time_block_right = time_x + time_size.x

      elif show_date_time == 3:
        # 날짜만: 년도 없이 요일 포함
        text_font = int(wheel_txt.height * 0.72)
        date_size = measure_text_cached(self._font_display, date_text, text_font)
        date_y = pos_y - date_size.y / 2

        draw_text_ui_style(
          date_text, time_x, date_y, text_font,
          rl.Color(255, 255, 255, 220),
          font=self._font_display,
          border_width=1.0,
          shadow_offset=3.0,
          align="left_top",
          y_offset=0.0,
        )

        time_block_right = time_x + date_size.x


    # --------------------------------------------------------------------------
    # Traffic Light (always higher priority than debug UI)
    # --------------------------------------------------------------------------
    traffic_x = int(time_block_right + 12)
    traffic_y = int(pos_y)

    if self._draw_traffic_light_info(traffic_x, traffic_y):
      return

    # --------------------------------------------------------------------------
    # Debug UI
    # --------------------------------------------------------------------------
    if show_debug_ui == 0:
      return

    info_x = time_block_right + 25

    cpu_text = self._get_cpu_temp_text()

    try:
      steer_ratio = float(ui_state.sm['liveParameters'].steerRatio)
      sr_text = f"SR: {steer_ratio:.1f}"
    except Exception:
      sr_text = "SR: --.-"

    try:
      road_name = ui_state.sm['carrotMan'].szPosRoadName
      if not road_name:
        road_name = ""
    except Exception:
      road_name = ""

    cpu_size = measure_text_cached(self._font_medium, cpu_text, side_font)
    sr_size = measure_text_cached(self._font_medium, sr_text, side_font)
    road_size = measure_text_cached(self._font_medium, road_name, side_font) if road_name else rl.Vector2(0, 0)

    line_gap = max(4, int(side_font * 0.15))

    total_h = cpu_size.y + line_gap + sr_size.y
    if road_name:
      total_h += line_gap + road_size.y

    base_y = pos_y - total_h / 2

    cpu_y = base_y
    sr_y = cpu_y + cpu_size.y + line_gap
    road_y = sr_y + sr_size.y + line_gap

    draw_text_ui_style(cpu_text, info_x, cpu_y, side_font, rl.Color(255, 255, 255, 210), font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)

    draw_text_ui_style(sr_text, info_x, sr_y, side_font, rl.Color(255, 255, 255, 210), font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)

    if road_name:
      draw_text_ui_style(road_name, info_x, road_y, side_font, rl.Color(255, 255, 255, 210), font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)


  def _get_gear_text(self) -> str:
    sm = ui_state.sm

    try:
      car_state = sm["carState"]
      gear = car_state.gearShifter
    except Exception:
      return "R"

    # cereal enum → 문자열 변환
    try:
      gear_name = str(gear).split('.')[-1]
    except Exception:
      gear_name = str(gear)

    # DRIVE 처리
    if "DRIVE" in gear_name.upper():
      try:
        step = int(car_state.gearStep)
        if step > 0:
          return str(step)
        else:
          return "D"
      except Exception:
        return "D"

    if "PARK" in gear_name.upper():
      return "P"

    if "REVERSE" in gear_name.upper():
      return "R"

    if "NEUTRAL" in gear_name.upper():
      return "N"

    if "SPORT" in gear_name.upper():
      return "S"

    if "LOW" in gear_name.upper():
      return "L"

    if "BRAKE" in gear_name.upper():
      return "B"

    if "ECO" in gear_name.upper():
      return "E"

    if "UNKNOWN" in gear_name.upper():
      return "U"

    return "M"

  def _get_cruise_gap(self) -> int:
    try:
      personality = Params().get_int("LongitudinalPersonality")
      gap = int(personality) + 1
    except Exception:
      gap = 8

    return gap

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """
    Bottom-left speed panel (like your 3rd image)
    - Background: images/speed_bg.png
    - Overlays: current speed, set speed, traffic light, cruise gap (1~4), gear (D/P/R/N)
    """
    ov = self._set_speed_override.compute(ui_state.sm, float(self.set_speed))

    # ----- panel placement (bottom-left) -----
    bg = self._txt_speed_bg
    panel_w = bg.width
    panel_h = bg.height

    margin_x = 10
    margin_y = 10
    panel_x = int(rect.x + margin_x)
    panel_y = int(rect.y + rect.height - panel_h - margin_y)

    # draw background
    rl.draw_texture(bg, panel_x, panel_y, rl.WHITE)

    # ----- current speed (big, left) -----
    if self._debug_speed_panel:
      cur_speed_int = 123
    else:
      cur_speed_int = int(round(self.speed))

    cur_text = str(cur_speed_int)

    cur_font = 80
    cur_size = measure_text_cached(self._font_display, cur_text, cur_font)
    cur_x = panel_x + 18

    cur_y = int(panel_y + panel_h * 0.48 - cur_size.y * 0.5) - 2

    draw_text_ui_style(cur_text, cur_x, cur_y, cur_font, rl.WHITE, font=self._font_display, border_width=2.0, shadow_offset=3.0, align="left_top", y_offset=0.0)

    mode_text, mode_color = self._get_driving_mode_text_and_color()
    if self._debug_speed_panel:
      mode_text = "safe"
      mode_color = rl.Color(0, 255, 0, 230)

    if mode_text:
      mode_font = 25
      mode_size = measure_text_cached(self._font_semi_bold, mode_text, mode_font)

      mode_x = panel_x + 5
      mode_y = int(panel_y + panel_h * 0.05 - mode_size.y * 0.5 - 15)

      draw_text_ui_style(mode_text, mode_x, mode_y, mode_font, mode_color, font=self._font_display, border_width=1.0, shadow_offset=3.0, align="left_top", y_offset=0.0)

    # ----- set speed (center, smaller) -----
    show_set = self._engaged and self.is_cruise_set
    if True: #show_set or self._debug_speed_panel:
      if show_set:
        set_speed = self.set_speed
        if not ui_state.is_metric:
          set_speed *= KM_TO_MILE
        set_text = str(int(round(set_speed)))
      else:
        set_text = "--"

      set_color = rl.Color(0, 255, 0, 230)

      if self._debug_speed_panel:
        set_text = str(123)

      set_font = 40
      set_size = measure_text_cached(self._font_display, set_text, set_font)
      set_x = int(panel_x + panel_w * 0.76 - set_size.x * 0.5)
      set_y = int(panel_y + panel_h * 0.33 - set_size.y * 0.5)
      draw_text_ui_style(set_text, set_x, set_y, set_font, set_color, font=self._font_display, border_width=1.0, shadow_offset=3.0, align="left_top", y_offset=0.0)
      if ov.active:
        set_speed = ov.speed_kph
        if not ui_state.is_metric:
          set_speed *= KM_TO_MILE
        set_text = str(int(round(set_speed)))
        set_label_text = ov.label

        if ov.speed_color_mode == 1:      # eco
          set_color = rl.Color(0, 255, 0, 230)
        elif ov.speed_color_mode == 2:    # apply
          set_color = rl.Color(255, 165, 0, 230)
        elif ov.speed_color_mode == 3:    # vehicle navigation CAN
          set_color = rl.Color(199, 125, 255, 230)
        elif ov.speed_color_mode == 4:    # external navigation
          set_color = rl.Color(244, 172, 54, 230)
        else:
          set_color = rl.Color(0, 255, 0, 230)   # your sample is green

        if self._debug_speed_panel:
          set_text = str(111)
          set_color = rl.Color(255, 165, 0, 230)
          set_label_text = "vturn"

        set_font = 40
        set_size = measure_text_cached(self._font_display, set_text, set_font)
        set_x = int(panel_x + panel_w * 0.90 - set_size.x * 0.5 + 50)
        set_y = int(panel_y + panel_h * 0.25 - set_size.y * 0.5)
        draw_text_ui_style(set_text, set_x, set_y, set_font, set_color, font=self._font_display, border_width=1.0, shadow_offset=3.0, align="left_top", y_offset=0.0)
        set_font = 30
        set_size = measure_text_cached(self._font_display, set_label_text, set_font)
        set_x = int(panel_x + panel_w * 0.90 - set_size.x * 0.5 + 50)
        set_y = int(panel_y + panel_h * 0.10 - set_size.y * 0.5 - 20)
        draw_text_ui_style(set_label_text, set_x, set_y, set_font, set_color, font=self._font_display, border_width=1.0, shadow_offset=3.0, align="left_top", y_offset=0.0)

    # ----- cruise gap (small circle + number, bottom-mid-right) -----
    gap = self._get_cruise_gap()
    gap_center_x = int(panel_x + panel_w * 0.90)
    gap_center_y = int(panel_y + panel_h * 0.82)
    #rl.draw_circle_lines(gap_center_x, gap_center_y, 16, rl.WHITE)

    gap_text = str(gap)
    gap_font = 28
    gap_size = measure_text_cached(self._font_semi_bold, gap_text, gap_font)
    draw_text_ui_style(gap_text, gap_center_x, gap_center_y, gap_font, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=3.0, align="center", y_offset=0.0)

    # Navigation availability is independent of speed-control state. Vehicle
    # CAN candidates also change activeCarrot, so it cannot identify an
    # external navigation connection.
    sm = ui_state.sm
    carrot_man = sm['carrotMan']
    vehicle_navi_available = bool(getattr(carrot_man, "vehicleNaviAvailable", False))
    try:
      carrot_navi_connected = bool(
        sm.alive['carrotNavi']
        and sm.valid['carrotNavi']
        and getattr(sm['carrotNavi'], "connected", False)
      )
    except Exception:
      carrot_navi_connected = False
    external_navi_connected = external_navigation_connected(
      getattr(carrot_man, "remote", ""), carrot_navi_connected,
    )
    navi_status = navigation_status_presentation(vehicle_navi_available, external_navi_connected)
    if navi_status is not None:
      navi_label, navi_color_mode = navi_status
      x = int(panel_x + panel_w * 0.60 - 26)
      y = int(panel_y + panel_h * 0.82)
      navi_color = rl.Color(199, 125, 255, 230) if navi_color_mode == 3 else rl.Color(244, 172, 54, 230)
      draw_text_ui_style(navi_label, x, y, 26, navi_color, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)


    # ----- gear (right side box with letter) -----
    gear = self._get_gear_text()
    box_w = 44
    box_h = 54
    box_x = int(panel_x + panel_w - box_w - 14 + 70)
    box_y = int(panel_y + panel_h * 0.50)

    # Fill (dark) + border (green)
    rl.draw_rectangle_rounded(rl.Rectangle(box_x, box_y, box_w, box_h), 0.2, 8, rl.Color(0, 0, 0, 120))
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(box_x, box_y, box_w, box_h), 0.2, 8, 3, rl.Color(0, 255, 0, 230))

    gear_font = 44
    gear_size = measure_text_cached(self._font_display, gear, gear_font)
    rl.draw_text_ex(
      self._font_display,
      gear,
      rl.Vector2(box_x + (box_w - gear_size.x) * 0.5, box_y + (box_h - gear_size.y) * 0.5),
      gear_font,
      0,
      rl.WHITE,
    )

    # 기존 레인모드/레인리스 출력 코드 제거
    """
    if self._debug_speed_panel:
      active_lane_line = True
    else:
      active_lane_line = bool(ui_state.sm['controlsState'].activeLaneLine)

    line1 = "lane"
    line2 = "mode" if active_lane_line else "less"

    lane_font = 26  # 원하면 22~30 사이로 조절
    lane_color = rl.Color(255, 255, 255, 220)  # 흰색

    lane_x = box_x + box_w + 80
    lane_y1 = box_y + 2
    lane_y2 = box_y + 2 + lane_font + 2

    # 오른쪽 정렬(gear box 옆에 딱 붙게)
    s1 = measure_text_cached(self._font_semi_bold, line1, lane_font)
    s2 = measure_text_cached(self._font_semi_bold, line2, lane_font)

    draw_text_ui_style(line1, lane_x - s1.x, lane_y1, lane_font, lane_color, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)
    draw_text_ui_style(line2, lane_x - s2.x, lane_y2, lane_font, lane_color, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)
    """

  def _get_driving_mode_text_and_color(self) -> tuple[str, rl.Color]:
    carState = ui_state.sm["carState"]
    if carState.brakeHoldActive:
      return tr("brake hold"), rl.Color(255, 0, 0, 230)
    elif carState.softHoldActive:
      return tr("soft hold"), rl.Color(255, 165, 0, 230)
    elif carState.carrotCruise:
      return tr("carrot"), rl.Color(0, 255, 0, 230)

    try:
      mode_val = int(ui_state.sm["longitudinalPlan"].myDrivingMode)
    except Exception:
      return "", rl.Color(255, 255, 255, 200)

    if mode_val == 1:   # eco
      return tr("eco"), rl.Color(0, 255, 0, 200)
    if mode_val == 2:   # safe
      return tr("safe"), rl.Color(255, 165, 0, 200)
    if mode_val == 3:   # normal
      return tr("norm"), rl.Color(255, 255, 255, 200)
    if mode_val == 4:   # high
      return tr("high"), rl.Color(255, 0, 0, 200)

    return "", rl.Color(255, 255, 255, 200)


  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)


  def _get_traffic_light_info(self):
    # debug demo
    if self._debug_traffic_light:
      demo_list = [
        {"lamp": "red", "remain": 13, "ts": time.monotonic()},
        {"lamp": "green", "remain": 8, "ts": time.monotonic()},
        {"lamp": "left", "remain": 7, "ts": time.monotonic()},
        {"lamp": "right", "remain": 5, "ts": time.monotonic()},
        {"lamp": "uturn", "remain": 4, "ts": time.monotonic()},
      ]
      idx = int(time.monotonic() // 2) % len(demo_list)
      return demo_list[idx]

    try:
      raw = ui_state.params_memory.get("TrafficLight", encoding="utf-8")
      if not raw:
        return None

      d = json.loads(raw)
      lamp = str(d.get("lamp", "")).strip()
      remain = int(d.get("remain", 0))
      ts = float(d.get("ts", 0.0))

      if lamp not in ("red", "green", "left", "right", "uturn"):
        return None

      if remain <= 0:
        return None

      # 1초마다 들어온다고 했으니, 2.5초 정도 지나면 stale로 보고 숨김
      if ts > 0.0 and (time.monotonic() - ts) > 2.5:
        return None

      return {
        "lamp": lamp,
        "remain": remain,
      }
    except Exception:
      return None

  def _draw_traffic_light_lamp(self, lamp: str, cx: int, cy: int, size: int) -> None:
    if lamp == "red":
      rl.draw_circle(cx, cy, size, rl.Color(255, 70, 70, 245))
      rl.draw_circle_lines(cx, cy, size, rl.Color(255, 255, 255, 220))
      return

    if lamp == "green":
      rl.draw_circle(cx, cy, size, rl.Color(0, 220, 80, 245))
      rl.draw_circle_lines(cx, cy, size, rl.Color(255, 255, 255, 220))
      return

    if lamp == "left":
      txt = "<-"
      color = rl.Color(0, 255, 100, 240)
    elif lamp == "right":
      txt = "->"
      color = rl.Color(0, 255, 100, 240)
    elif lamp == "uturn":
      txt = "U"
      color = rl.Color(255, 220, 80, 240)
    else:
      return

    font_size = int(size * 2.0)
    text_size = measure_text_cached(self._font_display, txt, font_size)
    draw_text_ui_style(txt, cx, cy, font_size, color, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="center", y_offset=0.0)

  def _draw_traffic_light_info(self, pos_x: int, pos_y: int) -> bool:
    info = self._get_traffic_light_info()
    if not info:
      return False

    lamp = info["lamp"]
    remain = str(info["remain"])

    lamp_size = 24
    remain_font = 28
    gap = 5

    remain_size = measure_text_cached(self._font_semi_bold, remain, remain_font)

    lamp_cx = pos_x + lamp_size
    lamp_cy = int(pos_y)

    self._draw_traffic_light_lamp(lamp, lamp_cx, lamp_cy, lamp_size)

    text_x = lamp_cx + lamp_size + gap
    text_y = int(pos_y - remain_size.y / 2)

    draw_text_ui_style(remain, text_x, text_y, remain_font, rl.Color(255, 255, 255, 235), font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)

    return True
