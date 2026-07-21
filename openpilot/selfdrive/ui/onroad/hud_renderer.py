import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.text_draw import draw_text_ui_style
from openpilot.system.ui.widgets import Widget

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'
CRUISE_SPEED_ANIMATION_START = 120
CRUISE_SPEED_ANIMATION_MAX = 100
CRUISE_SPEED_ANIMATION_STEP = 12
CRUISE_SPEED_ANIMATION_START_SIZE = 300
HUD_PARAM_REFRESH_INTERVAL = 1.0
WEEKDAYS_KO = ("일", "월", "화", "수", "목", "금", "토")


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  CARROT_GREEN = rl.Color(0, 203, 0, 255)
  BLACK_90 = rl.Color(0, 0, 0, 90)
  RED_180 = rl.Color(255, 0, 0, 180)
  GREEN_190 = rl.Color(0, 255, 0, 190)
  GREEN_200 = rl.Color(0, 255, 0, 200)
  GREEN_210 = rl.Color(0, 255, 0, 210)
  BLUE_210 = rl.Color(0, 120, 255, 210)
  RED_200 = rl.Color(255, 0, 0, 200)
  RED_210 = rl.Color(255, 0, 0, 210)
  YELLOW_210 = rl.Color(255, 255, 0, 210)
  WHITE_210 = rl.Color(255, 255, 255, 210)
  WHITE_220 = rl.Color(255, 255, 255, 220)
  TPMS_LOW = rl.Color(255, 90, 90, 220)
  ORANGE_200 = rl.Color(255, 165, 0, 200)
  ORANGE_230 = rl.Color(255, 165, 0, 230)
  RED_SOLID = rl.Color(255, 0, 0, 255)


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()

@dataclass(frozen=True)
class SetSpeedOverrideState:
  active: bool
  speed_kph: float
  label: str
  speed_color_mode: int # 0: white, 1: green, 2: orange
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
      label = desired_source.strip() or "apply"
      label = label[:8]  # 너무 길면 UI 깨짐 방지 (원하면 길이 조절)
      return SetSpeedOverrideState(
        active=True,
        speed_kph=desired_speed,
        label=label,
        speed_color_mode=2,
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


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.is_cruise_set = False
    self.is_cruise_available = True
    self.set_speed = SET_SPEED_NA
    self.speed = 0.0
    self.v_ego_cluster_seen = False

    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._font_display = gui_app.font(FontWeight.DISPLAY)

    self._exp_button = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    self._txt_speed_bg = gui_app.texture('images/speed_bg.png')

    # traffic light icon들 이름은 실제 프로젝트 리소스 이름에 맞춰 수정 가능
    self._traffic_red_icon = gui_app.texture('images/traffic_red.png')
    self._traffic_green_icon = gui_app.texture('images/traffic_green.png')

    self._ic_turn_l = gui_app.texture('images/turn_l.png')
    self._ic_turn_r = gui_app.texture('images/turn_r.png')
    self._ic_lane_change_l = gui_app.texture('images/lane_change_l.png')
    self._ic_lane_change_r = gui_app.texture('images/lane_change_r.png')
    self._ic_turn_u = gui_app.texture('images/turn_u.png')

    self._set_speed_override = SetSpeedOverride()
    self._debug_speed_panel = False
    self._engaged = False

    # c3-wip cruise-speed animation: center popup -> left Carrot HUD target.
    self._cruise_speed_text_last = ""
    self._cruise_speed_animation_text = ""
    self._cruise_speed_animation_time = -1

    self._blink_timer = 0
    self._disp_timer = 0

    self._cpu_temp = 0.0
    self._cpu_usage = 0.0
    self._memory_usage = 0
    self._free_space = 0.0
    self._voltage = 0.0
    self._device_info_loaded = False
    self._device_info_recv_frames = (-1, -1)
    self._cpu_temp_text = "0°C"
    self._memory_usage_text = "0%"
    self._disk_usage_text = "100%"
    self._voltage_text = "0.0V"
    self._plot_renderer = None
    self._round_box_rect = rl.Rectangle(0.0, 0.0, 0.0, 0.0)

    self._hud_params_next_refresh_time = 0.0
    self._show_device_state = 0
    self._show_date_time = 0
    self._show_plot_mode = 0
    self._longitudinal_personality = 7

    self._date_time_minute_key: tuple[int, int, int, int, int] | None = None
    self._date_time_text = ""
    self._date_text = ""

  def _refresh_hud_params(self, now: float) -> None:
    if now < self._hud_params_next_refresh_time:
      return

    try:
      show_device_state = ui_state.params.get_int("ShowDeviceState")
      show_date_time = ui_state.params.get_int("ShowDateTime")
      show_plot_mode = ui_state.params.get_int("ShowPlotMode")
    except Exception:
      # Keep the last complete snapshot and retry on the next frame.
      return

    personality_read_failed = False
    try:
      longitudinal_personality = ui_state.params.get_int("LongitudinalPersonality")
    except Exception:
      # Preserve the legacy fallback (gap 8) and retry on the next frame.
      longitudinal_personality = 7
      personality_read_failed = True

    self._show_device_state = show_device_state
    self._show_date_time = show_date_time
    self._show_plot_mode = show_plot_mode
    self._longitudinal_personality = longitudinal_personality
    self._hud_params_next_refresh_time = now if personality_read_failed else now + HUD_PARAM_REFRESH_INTERVAL

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    device_info_recv_frames = (sm.recv_frame["deviceState"], sm.recv_frame["peripheralState"])
    if not self._device_info_loaded or device_info_recv_frames != self._device_info_recv_frames:
      self._update_device_info()
      if self._device_info_loaded:
        self._device_info_recv_frames = device_info_recv_frames

    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    #if self.is_cruise_set and not ui_state.is_metric:
    #  self.set_speed *= KM_TO_MILE

    self._engaged = sm['selfdriveState'].enabled

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    self._refresh_hud_params(time.monotonic())

    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed_carrot(rect)

    #self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    if self._plot_renderer is None:
      self._plot_renderer = PlotRenderer()
    self._plot_renderer.draw(rect, self._font_display, self._show_plot_mode)

    self._draw_date_time(rect)
    self._draw_tpms_top_right(rect)
    self._draw_cruise_speed_animation(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

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

  def _draw_round_box(self, x, y, w, h, fill_color,
                      line_color=None,
                      roundness=0.25,
                      segments=8,
                      line_thickness=2):
    rect = self._round_box_rect
    rect.x = float(x)
    rect.y = float(y)
    rect.width = float(w)
    rect.height = float(h)
    rl.draw_rectangle_rounded(rect, roundness, segments, fill_color)
    if line_color is not None and line_thickness > 0:
      rl.draw_rectangle_rounded_lines_ex(rect, roundness, segments, float(line_thickness), line_color)


  def _draw_texture_rect(self, tex, x, y, w, h, tint=rl.WHITE):
    if tex is None:
      return
    rl.draw_texture_pro(
      tex,
      rl.Rectangle(0, 0, float(tex.width), float(tex.height)),
      rl.Rectangle(float(x), float(y), float(w), float(h)),
      rl.Vector2(0, 0),
      0.0,
      tint,
    )

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
    return int(self._longitudinal_personality) + 1

  def _get_driving_mode_text_and_color(self) -> tuple[str, rl.Color]:
    try:
      mode_val = int(ui_state.sm["longitudinalPlan"].myDrivingMode)
    except Exception:
      return "", COLORS.WHITE_TRANSLUCENT

    if mode_val == 1:   # eco
      return tr("eco"), COLORS.GREEN_200
    if mode_val == 2:   # safe
      return tr("safe"), COLORS.ORANGE_200
    if mode_val == 3:   # normal
      return tr("norm"), COLORS.WHITE_TRANSLUCENT
    if mode_val == 4:   # high
      return tr("high"), COLORS.RED_200

    return "", COLORS.WHITE_TRANSLUCENT

  def _update_device_info(self):
    sm = ui_state.sm
    loaded = True

    self._cpu_temp = 0.0
    self._cpu_usage = 0.0
    self._memory_usage = 0
    self._free_space = 0.0
    self._voltage = 0.0
    #self._plot_renderer = None

    try:
      device_state = sm["deviceState"]
      self._free_space = float(device_state.freeSpacePercent)
      self._memory_usage = int(device_state.memoryUsagePercent)

      try:
        cpu_temps = list(device_state.cpuTempC)
        if len(cpu_temps) > 0:
          self._cpu_temp = sum(cpu_temps) / len(cpu_temps)
      except Exception:
        loaded = False

      try:
        cpu_usages = [float(v) for v in device_state.cpuUsagePercent if float(v) > 0]
        if len(cpu_usages) > 0:
          self._cpu_usage = sum(cpu_usages) / len(cpu_usages)
      except Exception:
        loaded = False
    except Exception:
      loaded = False

    try:
      peripheral_state = sm["peripheralState"]
      self._voltage = float(peripheral_state.voltage) / 1000.0
    except Exception:
      loaded = False

    self._cpu_temp_text = f"{self._cpu_temp:.0f}°C"
    self._memory_usage_text = f"{self._memory_usage}%"
    self._disk_usage_text = f"{100 - self._free_space:.0f}%"
    self._voltage_text = f"{self._voltage:.1f}V"
    self._device_info_loaded = loaded

  def _get_active_carrot(self) -> int:
    try:
      return int(ui_state.sm["carrotMan"].activeCarrot)
    except Exception:
      return 0


  def _get_nav_path_vertex_count(self) -> int:
    try:
      return int(ui_state.sm["carrotMan"].navPathVertexCount)
    except Exception:
      return 0


  def _get_traffic_state(self) -> int:
    try:
      return int(ui_state.sm["carrotMan"].trafficState)
    except Exception:
      return 0


  def _get_traffic_state_carrot(self) -> int:
    try:
      return int(ui_state.sm["carrotMan"].trafficStateCarrot)
    except Exception:
      return 0


  def _get_speed_limit_info(self) -> tuple[int, int, int]:
    """
    return:
      x_spd_limit, x_sign_type, road_limit_speed
    """
    try:
      cm = ui_state.sm["carrotMan"]
    except Exception:
      return 0, 0, 0

    try:
      x_spd_limit = int(cm.xSpdLimit)
    except Exception:
      x_spd_limit = 0

    try:
      x_sign_type = int(cm.xSignType)
    except Exception:
      x_sign_type = 0

    try:
      road_limit_speed = int(cm.nRoadLimitSpeed)
    except Exception:
      road_limit_speed = 0

    return x_spd_limit, x_sign_type, road_limit_speed


  def _gps_has_fix(self) -> bool:
    sm = ui_state.sm

    try:
      return bool(sm["gpsLocationExternal"].hasFix)
    except Exception:
      pass

    try:
      return bool(sm["gpsLocation"].hasFix)
    except Exception:
      pass

    return False

  def _draw_carrot_traffic_light(self, bx: int, by: int):
    traffic_state = self._get_traffic_state()
    traffic_state_carrot = self._get_traffic_state_carrot()

    icon_size = 64
    red_light = traffic_state == 1
    green_light = traffic_state == 2

    icon_red = icon_size
    icon_green = icon_size

    if traffic_state_carrot == 1:
      red_light = True
      icon_red = int(icon_red * 1.5)
    elif traffic_state_carrot == 2:
      green_light = True
      icon_green = int(icon_green * 1.5)

    x = bx
    y = by + 270

    if red_light:
      self._draw_texture_rect(self._traffic_red_icon, x - icon_red / 2, y - icon_red / 2, icon_red, icon_red)
    elif green_light:
      self._draw_texture_rect(self._traffic_green_icon, x - icon_green / 2, y - icon_green / 2, icon_green, icon_green)

  def _draw_carrot_speed_panel(self, bx: int, by: int):
    sm = ui_state.sm
    ov = self._set_speed_override.compute(sm, float(self.set_speed))

    self._draw_texture_rect(self._txt_speed_bg, bx - 100, by - 60, 350, 150)

    cur_speed_int = 123 if self._debug_speed_panel else int(round(self.speed))
    cur_text = str(cur_speed_int)

    draw_text_ui_style(
      cur_text, bx, by + 50, 120, rl.WHITE,
      font=self._font_display,
      border_width=3.0,
      shadow_offset=8.0,
      align="center_bottom",
    )

    if self._engaged and self.is_cruise_set:
      set_speed = float(self.set_speed)
      if not ui_state.is_metric:
        set_speed *= KM_TO_MILE
      cruise_text = str(int(round(set_speed)))
    else:
      cruise_text = "--"

    self._update_cruise_speed_animation(cruise_text)

    draw_text_ui_style(
      cruise_text, bx + 170, by + 15, 60, COLORS.CARROT_GREEN,
      font=self._font_display,
      border_width=1.0,
      shadow_offset=5.0,
      align="center_bottom",
    )

    if ov.active:
      ov_speed = float(ov.speed_kph)
      if not ui_state.is_metric:
        ov_speed *= KM_TO_MILE
      ov_text = str(int(round(ov_speed)))
      ov_label = ov.label

      if ov.speed_color_mode == 1:
        ov_color = rl.GREEN
      elif ov.speed_color_mode == 2:
        ov_color = COLORS.ORANGE_230
      else:
        ov_color = rl.GREEN

      if self._debug_speed_panel:
        ov_text = "111"
        ov_label = "vturn"

      draw_text_ui_style(
        ov_text, bx + 250, by - 50 + 5, 50, ov_color,
        font=self._font_display,
        border_width=1.0,
        shadow_offset=5.0,
        align="center_bottom",
      )

      draw_text_ui_style(
        ov_label, bx + 250, by - 100, 30, ov_color,
        font=self._font_display,
        border_width=1.0,
        shadow_offset=5.0,
        align="center_bottom",
      )

  def _update_cruise_speed_animation(self, cruise_text: str) -> None:
    if self._cruise_speed_text_last != cruise_text:
      self._cruise_speed_text_last = cruise_text
      if cruise_text == "--":
        return
      self._cruise_speed_animation_text = cruise_text
      self._cruise_speed_animation_time = CRUISE_SPEED_ANIMATION_START

  def _draw_cruise_speed_animation(self, rect: rl.Rectangle) -> None:
    if self._cruise_speed_animation_time <= 0 or not self._cruise_speed_animation_text:
      return

    # c3-wip's integer state machine, tuned for a smaller and quicker BIG popup.
    # At 20 Hz this renders for 10 frames and starts shrinking almost at once.
    self._cruise_speed_animation_time -= CRUISE_SPEED_ANIMATION_STEP
    animation_time = self._cruise_speed_animation_time
    interpolation_time = min(animation_time, CRUISE_SPEED_ANIMATION_MAX)

    start_x = rect.x + rect.width / 2.0
    start_y = rect.y + rect.height - 400.0
    target_x = rect.x + 140.0 + 170.0
    target_y = rect.y + rect.height - 230.0 + 15.0
    target_size = 60

    x = int((start_x * interpolation_time + target_x * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
            CRUISE_SPEED_ANIMATION_MAX)
    y = int((start_y * interpolation_time + target_y * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
            CRUISE_SPEED_ANIMATION_MAX)
    font_size = int((CRUISE_SPEED_ANIMATION_START_SIZE * interpolation_time +
                     target_size * (CRUISE_SPEED_ANIMATION_MAX - interpolation_time)) /
                    CRUISE_SPEED_ANIMATION_MAX)

    if animation_time >= CRUISE_SPEED_ANIMATION_MAX:
      border_width = 9.0
      shadow_offset = 8.0
    else:
      border_width = 3.0
      shadow_offset = 0.0

    draw_text_ui_style(
      self._cruise_speed_animation_text,
      x,
      y,
      font_size,
      COLORS.CARROT_GREEN,
      font=self._font_display,
      border_width=border_width,
      shadow_offset=shadow_offset,
      align="center_bottom",
    )

  def _draw_carrot_lower_status(self, bx: int, by: int):
    mode_text, mode_color = self._get_driving_mode_text_and_color()
    if self._debug_speed_panel:
      mode_text = "safe"
      mode_color = COLORS.ORANGE_230

    # driving mode
    if mode_text:
      dx = bx - 50
      dy = by + 175

      self._draw_round_box(
        dx - 55, dy - 38, 110, 48,
        mode_color,
        line_color=rl.WHITE,
        roundness=0.25,
        segments=8,
        line_thickness=2,
      )

      draw_text_ui_style(
        mode_text, dx, dy - 2, 32, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )

      if self._gps_has_fix():
        draw_text_ui_style(
          "GPS", dx, dy - 45, 30, rl.GREEN,
          font=self._font_display,
          border_width=2.0,
          shadow_offset=4.0,
          align="center_bottom",
        )

    # gap number
    gap = self._get_cruise_gap()
    draw_text_ui_style(
      str(gap), bx + 220, by + 77, 40, rl.WHITE,
      font=self._font_display,
      border_width=2.0,
      shadow_offset=4.0,
      align="center_bottom",
    )

    # gap bars
    dx = bx + 270
    dy = by + 185
    ddy = 80.0 / 4.0
    for i in range(max(0, min(gap, 4))):
      self._draw_round_box(
        dx,
        dy - ddy * (i + 1) + 2,
        70,
        ddy - 2,
        COLORS.GREEN_210,
        line_color=rl.WHITE,
        roundness=0.12,
        segments=4,
        line_thickness=2,
      )

    # gear
    gear = self._get_gear_text()
    gx = bx + 305
    gy = by + 60

    self._draw_round_box(
      gx - 35, gy - 70, 70, 80,
      COLORS.GREEN_210,
      line_color=rl.WHITE,
      roundness=0.20,
      segments=8,
      line_thickness=3,
    )

    draw_text_ui_style(
      gear, gx, gy + 5, 70, rl.WHITE,
      font=self._font_display,
      border_width=2.0,
      shadow_offset=4.0,
      align="center_bottom",
    )

    # active carrot
    active_carrot = self._get_active_carrot()
    dx = bx + 200
    dy = by + 175

    if active_carrot >= 2:
      self._draw_round_box(
        dx - 55, dy - 38, 110, 48,
        rl.GREEN,
        line_color=rl.WHITE,
        roundness=0.25,
        segments=8,
        line_thickness=2,
      )
      draw_text_ui_style(
        "APN", dx, dy, 40, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )
    elif active_carrot >= 1:
      self._draw_round_box(
        dx - 55, dy - 38, 110, 48,
        COLORS.BLUE_210,
        line_color=rl.WHITE,
        roundness=0.25,
        segments=8,
        line_thickness=2,
      )
      draw_text_ui_style(
        "APM", dx, dy, 40, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )

    if self._get_nav_path_vertex_count() > 1:
      draw_text_ui_style(
        "ROUTE", dx, dy - 45, 30, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )

  def _draw_carrot_speed_limit_box(self, bx: int, by: int, speed_limit_info: tuple[int, int, int]):
    x_spd_limit, x_sign_type, road_limit_speed = speed_limit_info

    dx = bx + 75
    dy = by + 175

    disp_speed = 0
    limit_color = COLORS.GREEN_210
    label = "LIMIT"

    if x_spd_limit > 0 and x_sign_type != 22:
      disp_speed = int(x_spd_limit if ui_state.is_metric else (x_spd_limit * KM_TO_MILE + 0.5))
      label = "CAM"
      if self._blink_timer <= 8:
        limit_color = COLORS.RED_210
      else:
        limit_color = COLORS.YELLOW_210
    else:
      disp_speed = int(road_limit_speed if ui_state.is_metric else (road_limit_speed * KM_TO_MILE + 0.5))
      if self.speed > disp_speed + 2:
        limit_color = COLORS.RED_210
      else:
        limit_color = COLORS.WHITE_210

    draw_text_ui_style(
      label, dx, dy - 45, 30, rl.WHITE,
      font=self._font_display,
      border_width=2.0,
      shadow_offset=4.0,
      align="center_bottom",
    )

    self._draw_round_box(
      dx - 55, dy - 38, 110, 48,
      limit_color,
      line_color=rl.WHITE,
      roundness=0.25,
      segments=8,
      line_thickness=2,
    )

    draw_text_ui_style(
      str(disp_speed), dx, dy, 40, rl.WHITE,
      font=self._font_display,
      border_width=2.0,
      shadow_offset=4.0,
      align="center_bottom",
    )

  def _draw_carrot_main_background(self, bx: int, by: int, speed_limit_info: tuple[int, int, int]):
    x_spd_limit, x_sign_type, _ = speed_limit_info
    cam_detected = x_spd_limit > 0 and x_sign_type not in (22, 4)

    stroke_color = rl.WHITE
    if cam_detected and self._blink_timer > 8:
      bg_color = COLORS.RED_180
    else:
      bg_color = COLORS.BLACK_90

    if self._show_device_state > 0:
      self._draw_round_box(
        bx - 120, by - 270, 475, 495,
        bg_color,
        line_color=stroke_color,
        roundness=30.0 / 495.0,
        segments=12,
        line_thickness=2,
      )
    else:
      self._draw_round_box(
        bx - 120, by - 130, 475, 355,
        bg_color,
        line_color=stroke_color,
        roundness=30.0 / 355.0,
        segments=12,
        line_thickness=2,
      )

  def _draw_carrot_device_state(self, bx: int, by: int):
    if self._show_device_state <= 0:
      return

    dx = bx - 35
    dy = by - 200
    ok_color = COLORS.GREEN_190

    # CPU
    cpu_fill = COLORS.RED_SOLID if (self._cpu_temp > 80 and self._blink_timer <= 8) else ok_color
    self._draw_round_box(dx - 65, dy - 38, 130, 90, cpu_fill, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)
    draw_text_ui_style("CPU", dx, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")
    draw_text_ui_style(self._cpu_temp_text, dx, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")

    # MEM
    dx2 = dx + 150
    mem_fill = COLORS.RED_SOLID if (self._memory_usage > 85 and self._blink_timer <= 8) else ok_color
    self._draw_round_box(dx2 - 65, dy - 38, 130, 90, mem_fill, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)
    draw_text_ui_style("MEM", dx2, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")
    draw_text_ui_style(self._memory_usage_text, dx2, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")

    # DISK / VOLT
    dx3 = dx2 + 150
    self._draw_round_box(dx3 - 65, dy - 38, 130, 90, ok_color, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)

    if self._disp_timer < 32:
      draw_text_ui_style("DISK", dx3, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")
      draw_text_ui_style(self._disk_usage_text, dx3, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")
    else:
      draw_text_ui_style("VOLT", dx3, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")
      draw_text_ui_style(self._voltage_text, dx3, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=4.0, align="center_bottom")

  def _draw_date_time(self, rect: rl.Rectangle) -> None:
    show_datetime = self._show_date_time
    if show_datetime <= 0:
      return

    self._refresh_date_time_text(time.localtime())

    x = int(rect.x + 170)
    y = int(rect.y + 120)

    if show_datetime in (1, 2):
      draw_text_ui_style(
        self._date_time_text, x, y, 100, rl.WHITE,
        font=self._font_display,
        border_width=3.0,
        shadow_offset=8.0,
        align="center_bottom",
      )

    if show_datetime in (1, 3):
      draw_text_ui_style(
        self._date_text, x, y + 70, 60, rl.WHITE,
        font=self._font_display,
        border_width=3.0,
        shadow_offset=8.0,
        align="center_bottom",
      )

  def _refresh_date_time_text(self, now: time.struct_time) -> None:
    minute_key = (now.tm_year, now.tm_yday, now.tm_hour, now.tm_min, now.tm_isdst)
    if minute_key == self._date_time_minute_key:
      return

    weekday = WEEKDAYS_KO[(now.tm_wday + 1) % 7]
    self._date_time_text = time.strftime("%H:%M", now)
    self._date_text = f"{time.strftime('%m-%d', now)}({weekday})"
    self._date_time_minute_key = minute_key

  def _get_tpms_color(self, tpms: float) -> rl.Color:
    if tpms < 5 or tpms > 60:
      return COLORS.WHITE_220
    if tpms < 31:
      return COLORS.TPMS_LOW
    return COLORS.WHITE_220

  def _get_tpms_text(self, tpms: float) -> str:
    if tpms < 5 or tpms > 60:
      return '  -'
    return f'{round(tpms):.0f}'

  def _draw_tpms_top_right(self, rect: rl.Rectangle) -> None:
    show_tpms = 1 #ui_state.params.get_int('ShowTpms')
    if show_tpms not in (1, 3):
      return

    try:
      tpms = ui_state.sm['carState'].tpms
      fl = float(tpms.fl)
      fr = float(tpms.fr)
      rl_v = float(tpms.rl)
      rr = float(tpms.rr)
    except Exception:
      return

    bx = rect.x + rect.width - 125
    by = rect.y + 130
    dw = 80

    draw_text_ui_style(
      self._get_tpms_text(fl), bx - dw, by - 55, 40, self._get_tpms_color(fl),
      font=self._font_display, border_width=1.0, shadow_offset=4.0, align='center_bottom',
    )
    draw_text_ui_style(
      self._get_tpms_text(fr), bx + dw, by - 55, 40, self._get_tpms_color(fr),
      font=self._font_display, border_width=1.0, shadow_offset=4.0, align='center_bottom',
    )
    draw_text_ui_style(
      self._get_tpms_text(rl_v), bx - dw, by + 70, 40, self._get_tpms_color(rl_v),
      font=self._font_display, border_width=1.0, shadow_offset=4.0, align='center_bottom',
    )
    draw_text_ui_style(
      self._get_tpms_text(rr), bx + dw, by + 70, 40, self._get_tpms_color(rr),
      font=self._font_display, border_width=1.0, shadow_offset=4.0, align='center_bottom',
    )

  def _get_turn_info_hud_data(self) -> dict:
    try:
      cm = ui_state.sm["carrotMan"]
    except Exception:
      return {
        "active_carrot": 0,
        "x_turn_info": 0,
        "x_dist_to_turn": 0,
        "n_go_pos_dist": 0,
        "n_go_pos_time": 0,
        "atc_type": "",
        "sdi_descr": "",
        "road_name": "",
        "tbt_main_text": "",
      }

    try:
      active_carrot = int(cm.activeCarrot)
    except Exception:
      active_carrot = 0

    try:
      x_turn_info = int(cm.xTurnInfo)
    except Exception:
      x_turn_info = 0

    try:
      x_dist_to_turn = int(cm.xDistToTurn)
    except Exception:
      x_dist_to_turn = 0

    try:
      n_go_pos_dist = int(cm.nGoPosDist)
    except Exception:
      n_go_pos_dist = 0

    try:
      n_go_pos_time = int(cm.nGoPosTime)
    except Exception:
      n_go_pos_time = 0

    try:
      atc_type = str(cm.atcType or "")
    except Exception:
      atc_type = ""

    try:
      sdi_descr = str(cm.szSdiDescr or "")
    except Exception:
      sdi_descr = ""

    try:
      road_name = str(cm.szPosRoadName or "")
    except Exception:
      road_name = ""

    try:
      tbt_main_text = str(cm.szTBTMainText or "")
    except Exception:
      tbt_main_text = ""

    return {
      "active_carrot": active_carrot,
      "x_turn_info": x_turn_info,
      "x_dist_to_turn": x_dist_to_turn,
      "n_go_pos_dist": n_go_pos_dist,
      "n_go_pos_time": n_go_pos_time,
      "atc_type": atc_type,
      "sdi_descr": sdi_descr,
      "road_name": road_name,
      "tbt_main_text": tbt_main_text,
    }

  def _format_turn_distance_text(self, dist_m: int) -> str:
    if dist_m <= 0:
      return ""

    if ui_state.is_metric:
      if dist_m < 1000:
        return f"{dist_m} m"
      return f"{dist_m / 1000.0:.1f} km"
    else:
      if dist_m < 1609:
        return f"{int(dist_m * 3.28084)} ft"
      return f"{dist_m / 1609.344:.1f} mi"

  def _format_eta_text(self, remain_sec: int) -> str:
    if remain_sec <= 0:
      return ""

    # Arrival time is wall-clock based; monotonic time cannot be converted to local time.
    eta_tm = time.localtime(time.time() + remain_sec)  # noqa: TID251
    remain_min = remain_sec / 60.0
    return f"도착: {remain_min:.1f}분({eta_tm.tm_hour:02d}:{eta_tm.tm_min:02d})"

  def _format_go_pos_distance_text(self, dist_m: int) -> str:
    if dist_m <= 0:
      return ""

    if ui_state.is_metric:
      return f"{dist_m / 1000.0:.1f}km"
    else:
      return f"{dist_m / 1000.0 * KM_TO_MILE:.1f}mile"

  def _draw_text_left_bottom(self, text: str, x: float, y: float, size: int, color, font=None, border_width: float = 2.0, shadow_offset: float = 4.0):
    if not text:
      return

    draw_text_ui_style(
      text, x, y, size, color,
      font=font or self._font_display,
      border_width=border_width,
      shadow_offset=shadow_offset,
      align="left_bottom",
    )

  def _draw_turn_icon(self, turn_info: int, bx: int, by: int, icon_size: int = 140):
    if turn_info == 1:
      self._draw_texture_rect(self._ic_turn_l, bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size)
    elif turn_info == 2:
      self._draw_texture_rect(self._ic_turn_r, bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size)
    elif turn_info == 3:
      self._draw_texture_rect(self._ic_lane_change_l, bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size)
    elif turn_info == 4:
      self._draw_texture_rect(self._ic_lane_change_r, bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size)
    elif turn_info == 7:
      self._draw_texture_rect(self._ic_turn_u, bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size)
    elif turn_info == 6:
      draw_text_ui_style(
        "TG", bx, by + 20, 35, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )
    elif turn_info == 8:
      draw_text_ui_style(
        "목적지", bx, by + 20, 35, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )
    else:
      draw_text_ui_style(
        f"감속:{turn_info}", bx, by + 20, 35, rl.WHITE,
        font=self._font_display,
        border_width=2.0,
        shadow_offset=4.0,
        align="center_bottom",
      )

  def _draw_turn_info_hud(self, rect: rl.Rectangle):
    if rect.width < 1200:
      return

    info = self._get_turn_info_hud_data()
    n_go_pos_dist = info["n_go_pos_dist"]
    n_go_pos_time = info["n_go_pos_time"]

    if not (n_go_pos_dist > 0 and n_go_pos_time > 0):
      return

    tbt_x = int(rect.x + rect.width - 800)
    tbt_y = int(rect.y + rect.height - 250)

    self._draw_round_box(
      tbt_x,
      tbt_y - 60,
      790,
      300,
      rl.Color(0, 0, 0, 120),
      line_color=rl.WHITE,
      roundness=30.0 / 300.0,
      segments=12,
      line_thickness=2,
    )

    if info["tbt_main_text"]:
      self._draw_text_left_bottom(
        info["tbt_main_text"], tbt_x + 20, tbt_y - 15, 40, rl.WHITE,
        font=self._font_bold, border_width=2.0, shadow_offset=4.0,
      )

    x_turn_info = info["x_turn_info"]
    x_dist_to_turn = info["x_dist_to_turn"]

    if x_turn_info > 0:
      bx = tbt_x + 100
      by = tbt_y + 85

      if info["atc_type"]:
        fill_color = rl.Color(0, 255, 0, 100) if "prepare" in info["atc_type"] else rl.GREEN
        self._draw_round_box(
          bx - 80, by - 90, 160, 230,
          fill_color,
          line_color=rl.BLACK,
          roundness=15.0 / 230.0,
          segments=8,
          line_thickness=1,
        )

      self._draw_turn_icon(x_turn_info, bx, by, 140)

      dist_text = self._format_turn_distance_text(x_dist_to_turn)
      if dist_text:
        draw_text_ui_style(
          dist_text,
          bx, by + 120, 40, rl.WHITE,
          font=self._font_bold,
          border_width=2.0,
          shadow_offset=4.0,
          align="center_bottom",
        )

    if info["sdi_descr"]:
      label_x = tbt_x + 200
      label_y = tbt_y + 200
      size = measure_text_cached(self._font_bold, info["sdi_descr"], 40)
      box_h = max(48, int(size.y + 13))
      self._draw_round_box(
        label_x - 10,
        label_y - int(size.y) - 2,
        int(size.x) + 20,
        box_h,
        rl.GREEN,
        roundness=10.0 / box_h,
        segments=8,
        line_thickness=0,
      )
      self._draw_text_left_bottom(
        info["sdi_descr"], label_x, label_y, 40, rl.WHITE,
        font=self._font_bold, border_width=1.5, shadow_offset=3.0,
      )
    elif info["road_name"]:
      self._draw_text_left_bottom(
        info["road_name"], tbt_x + 200, tbt_y + 200, 40, rl.WHITE,
        font=self._font_bold, border_width=1.5, shadow_offset=3.0,
      )

    eta_text = self._format_eta_text(n_go_pos_time)
    if eta_text:
      self._draw_text_left_bottom(
        eta_text, tbt_x + 190, tbt_y + 80, 50, rl.WHITE,
        font=self._font_bold, border_width=2.0, shadow_offset=4.0,
      )

    go_dist_text = self._format_go_pos_distance_text(n_go_pos_dist)
    if go_dist_text:
      self._draw_text_left_bottom(
        go_dist_text, tbt_x + 310, tbt_y + 130, 50, rl.WHITE,
        font=self._font_bold, border_width=2.0, shadow_offset=4.0,
      )

  def _draw_set_speed_carrot(self, rect: rl.Rectangle) -> None:
    self._blink_timer = (self._blink_timer + 1) % 16
    self._disp_timer = (self._disp_timer + 1) % 64

    # C drawHud anchor
    bx = int(rect.x + 140)
    by = int(rect.y + rect.height - 230)

    speed_limit_info = self._get_speed_limit_info()
    self._draw_carrot_main_background(bx, by, speed_limit_info)
    self._draw_carrot_traffic_light(bx, by)
    self._draw_carrot_speed_panel(bx, by)

    self._draw_carrot_lower_status(bx, by)
    self._draw_carrot_speed_limit_box(bx, by, speed_limit_info)
    self._draw_carrot_device_state(bx, by)

    self._draw_turn_info_hud(rect)



class PlotRenderer:
  PLOT_MAX = 400

  def __init__(self):
    self._plot_size = 0
    self._plot_index = 0
    self._plot_queue = [[0.0] * self.PLOT_MAX for _ in range(3)]
    self._plot_min = 0.0
    self._plot_max = 0.0
    self._plot_x = 350.0
    self._plot_width = 1000.0
    self._plot_y = 40.0
    self._plot_height = 300.0
    self._plot_dx = 2.0
    self._show_plot_mode_prev = -1

  def _clear(self):
    self._plot_size = 0
    self._plot_index = 0
    self._plot_min = 0.0
    self._plot_max = 0.0
    self._plot_queue = [[0.0] * self.PLOT_MAX for _ in range(3)]

  def _make_plot_data(self, sm, show_plot_mode: int):
    car_state = sm['carState']
    lp = sm['longitudinalPlan']
    car_control = sm['carControl']
    controls_state = sm['controlsState']

    a_ego = float(car_state.aEgo)
    v_ego = float(car_state.vEgo)

    accel = 0.0
    try:
      accel = float(lp.accels[0])
    except Exception:
      pass

    speeds_0 = 0.0
    try:
      speeds_0 = float(lp.speeds[0])
    except Exception:
      pass

    accel_out = 0.0
    try:
      accel_out = float(car_control.actuators.accel)
    except Exception:
      pass

    if show_plot_mode in (0, 1):
      return [a_ego, accel, accel_out], '1.Accel (Y:a_ego, G:a_target, O:a_out)'

    if show_plot_mode == 2:
      return [speeds_0, v_ego, a_ego], '2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)'

    if show_plot_mode == 3:
      pos_32 = 0.0
      vel_32 = 0.0
      vel_0 = 0.0
      try:
        pos_32 = float(sm['modelV2'].position.x[32])
      except Exception:
        pass
      try:
        vel_32 = float(sm['modelV2'].velocity.x[32])
      except Exception:
        pass
      try:
        vel_0 = float(sm['modelV2'].velocity.x[0])
      except Exception:
        pass
      return [pos_32, vel_32, vel_0], '3.Model(Y:pos_32, G:vel_32, O:vel_0)'

    if show_plot_mode == 4:
      a_lead_k = 0.0
      v_rel = 0.0
      try:
        a_lead_k = float(sm['radarState'].leadOne.aLeadK)
      except Exception:
        pass
      try:
        v_rel = float(sm['radarState'].leadOne.vRel)
      except Exception:
        pass
      return [accel, a_lead_k, v_rel], '4.Lead(Y:accel, G:a_lead, O:v_rel)'

    if show_plot_mode == 5:
      a_lead = 0.0
      j_lead = 0.0
      try:
        a_lead = float(sm['radarState'].leadOne.aLead)
      except Exception:
        pass
      try:
        j_lead = float(sm['radarState'].leadOne.jLead)
      except Exception:
        pass
      return [a_ego, a_lead, j_lead], '5.Lead(Y:a_ego, G:a_lead, O:j_lead)'

    if show_plot_mode == 6:
      actual_lat_accel = 0.0
      desired_lat_accel = 0.0
      output = 0.0
      try:
        actual_lat_accel = float(controls_state.lateralControlState.torqueState.actualLateralAccel) * 10.0
      except Exception:
        pass
      try:
        desired_lat_accel = float(controls_state.lateralControlState.torqueState.desiredLateralAccel) * 10.0
      except Exception:
        pass
      try:
        output = float(controls_state.lateralControlState.torqueState.output) * 10.0
      except Exception:
        pass
      return [actual_lat_accel, desired_lat_accel, output], '6.Steer(Y:actual, G:desire, O:output)'

    if show_plot_mode == 7:
      actual_angle = float(car_state.steeringAngleDeg)
      target_angle = 0.0
      angle_offset = 0.0
      try:
        target_angle = float(car_control.actuators.steeringAngleDeg)
      except Exception:
        pass
      try:
        angle_offset = float(sm['liveParameters'].angleOffsetDeg) * 10.0
      except Exception:
        pass
      return [actual_angle, target_angle, angle_offset], '7.SteerA (Y:Actual, G:Target, O:Offset*10)'

    if show_plot_mode == 8:
      curvature = 0.0
      try:
        curvature = float(car_control.actuators.curvature) * 10000.0
      except Exception:
        pass
      return [curvature, curvature, curvature], '8.Curvature (*10000)'

    return [0.0, 0.0, 0.0], 'no data'

  def _update_plot_queue(self, plot_data):
    self._plot_index = (self._plot_index + 1) % self.PLOT_MAX

    for i in range(3):
      self._plot_queue[i][self._plot_index] = float(plot_data[i])

    if self._plot_size < self.PLOT_MAX:
      self._plot_size += 1

    self._plot_min = float('inf')
    self._plot_max = float('-inf')
    for i in range(3):
      values = self._plot_queue[i][:self._plot_size] if self._plot_size < self.PLOT_MAX else self._plot_queue[i]
      self._plot_min = min(self._plot_min, min(values))
      self._plot_max = max(self._plot_max, max(values))

    if self._plot_min == float('inf'):
      self._plot_min = -2.0
    if self._plot_max == float('-inf'):
      self._plot_max = 2.0

    if self._plot_min > -2.0:
      self._plot_min = -2.0
    if self._plot_max < 2.0:
      self._plot_max = 2.0

  def _draw_plotting(self, index: int, x_base: float, y_base: float, color, font):
    if self._plot_size <= 0:
      return

    plot_range = self._plot_max - self._plot_min
    plot_ratio = self._plot_height if plot_range < 1.0 else (self._plot_height / plot_range)

    prev = None
    latest_x = None
    latest_y = None
    latest_value = 0.0

    for i in range(self._plot_size):
      data = self._plot_queue[index][(self._plot_index - i + self.PLOT_MAX) % self.PLOT_MAX]
      plot_y = y_base + self._plot_height - (data - self._plot_min) * plot_ratio
      plot_x = x_base + (self._plot_size - i) * self._plot_dx

      pt = rl.Vector2(plot_x, plot_y)
      if prev is not None:
        rl.draw_line_ex(prev, pt, 3.0, color)
      else:
        latest_x = plot_x
        latest_y = plot_y
        latest_value = data
      prev = pt

    if latest_x is not None and latest_y is not None:
      draw_text_ui_style(
        f'{latest_value:.2f}', latest_x + 50, latest_y + (40 if index > 0 else 0), 40, color,
        font=font, border_width=2.0, shadow_offset=4.0, align='center_bottom',
      )

  def draw(self, rect: rl.Rectangle, font, show_plot_mode: int) -> None:
    if show_plot_mode == 0:
      return
    try:
      if not ui_state.sm.alive['carState'] or not ui_state.sm.alive['longitudinalPlan']:
        return
    except Exception:
      return

    if show_plot_mode != self._show_plot_mode_prev:
      self._clear()
      self._show_plot_mode_prev = show_plot_mode

    try:
      plot_data, title = self._make_plot_data(ui_state.sm, show_plot_mode)
    except Exception:
      return

    self._update_plot_queue(plot_data)

    if rect.width < 1200:
      return

    x_base = rect.x + self._plot_x
    y_base = rect.y + self._plot_y
    colors = [rl.YELLOW, rl.GREEN, rl.Color(255, 165, 0, 255)]

    for i in range(3):
      self._draw_plotting(i, x_base, y_base, colors[i], font)

    draw_text_ui_style(
      title, x_base + 400, y_base - 20, 25, rl.WHITE,
      font=font, border_width=2.0, shadow_offset=4.0, align='center_bottom',
    )
