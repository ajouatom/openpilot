import math
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'


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

    self._set_speed_override = SetSpeedOverride()
    self._debug_speed_panel = False
    self._engaged = False

    self._blink_timer = 0
    self._disp_timer = 0

    self._cpu_temp = 0.0
    self._cpu_usage = 0.0
    self._memory_usage = 0
    self._free_space = 0.0
    self._voltage = 0.0

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

  def _draw_text_raw(self, font, text, x, y, font_size, color):
    rl.draw_text_ex(
      font,
      text,
      rl.Vector2(float(x), float(y)),
      float(font_size),
      0,
      color,
    )


  def _draw_text_ui_style(self,
                          text: str,
                          x: float,
                          y: float,
                          font_size: float,
                          color: rl.Color,
                          font=None,
                          border_width: float = 3.0,
                          shadow_offset: float = 0.0,
                          border_color: rl.Color = rl.BLACK,
                          shadow_color: rl.Color = rl.BLACK,
                          align: str = "center_bottom",
                          y_offset: float = 6.0) -> None:
    if not text:
      return

    if font is None:
      font = self._font_display

    text_size = measure_text_cached(font, text, font_size)

    draw_x = x
    draw_y = y + y_offset

    if align == "center_bottom":
      draw_x = x - text_size.x * 0.5
      draw_y = (y + y_offset) - text_size.y
    elif align == "center_top":
      draw_x = x - text_size.x * 0.5
      draw_y = y + y_offset
    elif align == "left_top":
      draw_x = x
      draw_y = y + y_offset

    if border_width > 0.0:
      for deg in range(0, 360, 45):
        rad = math.radians(deg)
        ox = border_width * math.cos(rad)
        oy = border_width * math.sin(rad)
        self._draw_text_raw(font, text, draw_x + ox, draw_y + oy, font_size, border_color)

    if shadow_offset != 0.0:
      self._draw_text_raw(font, text, draw_x + shadow_offset, draw_y + shadow_offset, font_size, shadow_color)

    self._draw_text_raw(font, text, draw_x, draw_y, font_size, color)


  def _draw_round_box(self, x, y, w, h, fill_color,
                      line_color=None,
                      roundness=0.25,
                      segments=8,
                      line_thickness=2):
    rect = rl.Rectangle(float(x), float(y), float(w), float(h))
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
    try:
      personality = ui_state.params.get_int("LongitudinalPersonality")
      gap = int(personality) + 1
    except Exception:
      gap = 8

    return gap

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

  def _update_device_info(self):
    sm = ui_state.sm

    self._cpu_temp = 0.0
    self._cpu_usage = 0.0
    self._memory_usage = 0
    self._free_space = 0.0
    self._voltage = 0.0

    try:
      device_state = sm["deviceState"]
      self._free_space = float(device_state.freeSpacePercent)
      self._memory_usage = int(device_state.memoryUsagePercent)

      try:
        cpu_temps = list(device_state.cpuTempC)
        if len(cpu_temps) > 0:
          self._cpu_temp = sum(cpu_temps) / len(cpu_temps)
      except Exception:
        pass

      try:
        cpu_usages = [float(v) for v in device_state.cpuUsagePercent if float(v) > 0]
        if len(cpu_usages) > 0:
          self._cpu_usage = sum(cpu_usages) / len(cpu_usages)
      except Exception:
        pass
    except Exception:
      pass

    try:
      peripheral_state = sm["peripheralState"]
      self._voltage = float(peripheral_state.voltage) / 1000.0
    except Exception:
      pass

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


  def _get_speed_limit_info(self):
    """
    return:
      x_spd_limit, x_sign_type, road_limit_speed
    """
    try:
      cm = ui_state.sm["carrotMan"]
      x_spd_limit = int(cm.xSpdLimit)
    except Exception:
      x_spd_limit = 0

    try:
      cm = ui_state.sm["carrotMan"]
      x_sign_type = int(cm.xSignType)
    except Exception:
      x_sign_type = 0

    try:
      cm = ui_state.sm["carrotMan"]
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

    self._draw_text_ui_style(
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

    self._draw_text_ui_style(
      cruise_text, bx + 170, by + 15 + 5, 60, rl.GREEN,
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
        ov_color = rl.Color(255, 165, 0, 230)
      else:
        ov_color = rl.GREEN

      if self._debug_speed_panel:
        ov_text = "111"
        ov_label = "vturn"

      self._draw_text_ui_style(
        ov_text, bx + 250, by - 50 + 5, 50, ov_color,
        font=self._font_display,
        border_width=1.0,
        shadow_offset=5.0,
        align="center_bottom",
      )

      self._draw_text_ui_style(
        ov_label, bx + 250, by - 100, 30, ov_color,
        font=self._font_display,
        border_width=1.0,
        shadow_offset=5.0,
        align="center_bottom",
      )

  def _draw_carrot_lower_status(self, bx: int, by: int):
    mode_text, mode_color = self._get_driving_mode_text_and_color()
    if self._debug_speed_panel:
      mode_text = "safe"
      mode_color = rl.Color(255, 165, 0, 230)

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

      self._draw_text_ui_style(
        mode_text, dx, dy - 2, 32, rl.WHITE,
        font=self._font_display,
        border_width=0.0,
        shadow_offset=0.0,
        align="center_bottom",
      )

      if self._gps_has_fix():
        self._draw_text_ui_style(
          "GPS", dx, dy - 45, 30, rl.GREEN,
          font=self._font_display,
          border_width=0.0,
          shadow_offset=0.0,
          align="center_bottom",
        )

    # gap number
    gap = self._get_cruise_gap()
    self._draw_text_ui_style(
      str(gap), bx + 220, by + 77, 40, rl.WHITE,
      font=self._font_display,
      border_width=0.0,
      shadow_offset=0.0,
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
        rl.Color(0, 255, 0, 210),
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
      rl.Color(0, 255, 0, 210),
      line_color=rl.WHITE,
      roundness=0.20,
      segments=8,
      line_thickness=3,
    )

    self._draw_text_ui_style(
      gear, gx, gy + 5, 70, rl.WHITE,
      font=self._font_display,
      border_width=0.0,
      shadow_offset=0.0,
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
      self._draw_text_ui_style(
        "APN", dx, dy, 40, rl.WHITE,
        font=self._font_display,
        border_width=0.0,
        shadow_offset=0.0,
        align="center_bottom",
      )
    elif active_carrot >= 1:
      self._draw_round_box(
        dx - 55, dy - 38, 110, 48,
        rl.Color(0, 120, 255, 210),
        line_color=rl.WHITE,
        roundness=0.25,
        segments=8,
        line_thickness=2,
      )
      self._draw_text_ui_style(
        "APM", dx, dy, 40, rl.WHITE,
        font=self._font_display,
        border_width=0.0,
        shadow_offset=0.0,
        align="center_bottom",
      )

    if self._get_nav_path_vertex_count() > 1:
      self._draw_text_ui_style(
        "ROUTE", dx, dy - 45, 30, rl.WHITE,
        font=self._font_display,
        border_width=0.0,
        shadow_offset=0.0,
        align="center_bottom",
      )

  def _draw_carrot_speed_limit_box(self, bx: int, by: int):
    x_spd_limit, x_sign_type, road_limit_speed = self._get_speed_limit_info()

    dx = bx + 75
    dy = by + 175

    disp_speed = 0
    limit_color = rl.Color(0, 255, 0, 210)
    label = "LIMIT"

    if x_spd_limit > 0 and x_sign_type != 22:
      disp_speed = int(x_spd_limit if ui_state.is_metric else (x_spd_limit * KM_TO_MILE + 0.5))
      label = "CAM"
      if self._blink_timer <= 8:
        limit_color = rl.Color(255, 0, 0, 210)
      else:
        limit_color = rl.Color(255, 255, 0, 210)
    else:
      disp_speed = int(road_limit_speed if ui_state.is_metric else (road_limit_speed * KM_TO_MILE + 0.5))
      if self.speed > disp_speed + 2:
        limit_color = rl.Color(255, 0, 0, 210)
      else:
        limit_color = rl.Color(255, 255, 255, 210)

    self._draw_text_ui_style(
      label, dx, dy - 45, 30, rl.WHITE,
      font=self._font_display,
      border_width=0.0,
      shadow_offset=0.0,
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

    self._draw_text_ui_style(
      str(disp_speed), dx, dy, 40, rl.WHITE,
      font=self._font_display,
      border_width=0.0,
      shadow_offset=0.0,
      align="center_bottom",
    )

  def _draw_carrot_main_background(self, bx: int, by: int):
    show_device_state = True #ui_state.params.get_int("ShowDeviceState")

    x_spd_limit, x_sign_type, _ = self._get_speed_limit_info()
    cam_detected = x_spd_limit > 0 and x_sign_type not in (22, 4)

    stroke_color = rl.WHITE
    if cam_detected and self._blink_timer > 8:
      bg_color = rl.Color(255, 0, 0, 180)
    else:
      bg_color = rl.Color(0, 0, 0, 90)

    if show_device_state > 0:
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
    show_device_state = True #ui_state.params.get_int("ShowDeviceState")
    if show_device_state <= 0:
      return

    self._update_device_info()

    dx = bx - 35
    dy = by - 200
    ok_color = rl.Color(0, 255, 0, 190)

    # CPU
    cpu_fill = rl.Color(255, 0, 0, 255) if (self._cpu_temp > 80 and self._blink_timer <= 8) else ok_color
    self._draw_round_box(dx - 65, dy - 38, 130, 90, cpu_fill, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)
    self._draw_text_ui_style("CPU", dx, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")
    self._draw_text_ui_style(f"{self._cpu_temp:.0f}°C", dx, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")

    # MEM
    dx2 = dx + 150
    mem_fill = rl.Color(255, 0, 0, 255) if (self._memory_usage > 85 and self._blink_timer <= 8) else ok_color
    self._draw_round_box(dx2 - 65, dy - 38, 130, 90, mem_fill, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)
    self._draw_text_ui_style("MEM", dx2, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")
    self._draw_text_ui_style(f"{self._memory_usage}%", dx2, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")

    # DISK / VOLT
    dx3 = dx2 + 150
    self._draw_round_box(dx3 - 65, dy - 38, 130, 90, ok_color, line_color=rl.WHITE, roundness=0.16, segments=8, line_thickness=2)

    if self._disp_timer < 32:
      self._draw_text_ui_style("DISK", dx3, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")
      self._draw_text_ui_style(f"{100 - self._free_space:.0f}%", dx3, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")
    else:
      self._draw_text_ui_style("VOLT", dx3, dy - 5, 25, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")
      self._draw_text_ui_style(f"{self._voltage:.1f}V", dx3, dy + 40, 40, rl.WHITE, font=self._font_display, border_width=0.0, shadow_offset=0.0, align="center_bottom")

  def _draw_set_speed_carrot(self, rect: rl.Rectangle) -> None:
    self._blink_timer = (self._blink_timer + 1) % 16
    self._disp_timer = (self._disp_timer + 1) % 64

    # C drawHud anchor
    bx = int(rect.x + 140)
    by = int(rect.y + rect.height - 230)

    self._draw_carrot_main_background(bx, by)
    self._draw_carrot_traffic_light(bx, by)
    self._draw_carrot_speed_panel(bx, by)
    self._draw_carrot_lower_status(bx, by)
    self._draw_carrot_speed_limit_box(bx, by)
    self._draw_carrot_device_state(bx, by)
  
