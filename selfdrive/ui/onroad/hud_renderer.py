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
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    self._carrot_scale = 1.5
    self._txt_speed_bg: rl.Texture = gui_app.texture('images/speed_bg.png', 307 * self._carrot_scale, 115 * self._carrot_scale)
    self._set_speed_override = SetSpeedOverride()
    self._debug_speed_panel = False
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)
    self._engaged: bool = False


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

  def _draw_text_with_outline(self, text, pos, font_size,
                              text_color,
                              outline_color=rl.BLACK,
                              thickness=2):
    x, y = pos.x, pos.y
    for dx in range(-thickness, thickness + 1):
      for dy in range(-thickness, thickness + 1):
        if dx == 0 and dy == 0:
          continue
        rl.draw_text_ex(
          self._font_display,
          text,
          rl.Vector2(x + dx, y + dy),
          font_size,
          0,
          outline_color
        )

    # main text
    rl.draw_text_ex(
      self._font_display,
      text,
      rl.Vector2(x, y),
      font_size,
      0,
      text_color
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

  def _draw_set_speed_carrot(self, rect: rl.Rectangle) -> None:
    """
    Bottom-left speed panel
    - speed_bg image is drawn at 1.5x scale
    - fonts, offsets, and box sizes are also scaled by 1.5x
    - positions are recomputed for the enlarged panel
    """
    ov = self._set_speed_override.compute(ui_state.sm, float(self.set_speed))

    scale = 1.5

    # ----- panel placement -----
    bg = self._txt_speed_bg
    src_w = bg.width
    src_h = bg.height

    panel_w = int(src_w * scale)
    panel_h = int(src_h * scale)

    margin_x = int(10 * scale)
    margin_y = int(10 * scale)

    panel_x = int(rect.x + margin_x)
    panel_y = int(rect.y + rect.height - panel_h - margin_y)

    # scaled background draw
    rl.draw_texture_pro(
      bg,
      rl.Rectangle(0, 0, src_w, src_h),
      rl.Rectangle(panel_x, panel_y, panel_w, panel_h),
      rl.Vector2(0, 0),
      0.0,
      rl.WHITE,
    )

    outline_1 = max(1, int(round(1 * scale)))
    outline_2 = max(1, int(round(2 * scale)))

    # ----- current speed (big, left) -----
    if self._debug_speed_panel:
      cur_speed_int = 123
    else:
      cur_speed_int = int(round(self.speed))

    cur_text = str(cur_speed_int)
    cur_font = int(80 * scale)
    cur_size = measure_text_cached(self._font_display, cur_text, cur_font)

    cur_x = int(panel_x + 18 * scale)
    cur_y = int(panel_y + panel_h * 0.48 - cur_size.y * 0.5 - 2 * scale)

    self._draw_text_with_outline(
      cur_text,
      rl.Vector2(cur_x, cur_y),
      cur_font,
      rl.WHITE,
      rl.BLACK,
      thickness=outline_2,
    )

    # ----- driving mode text (top-left) -----
    mode_text, mode_color = self._get_driving_mode_text_and_color()
    if self._debug_speed_panel:
      mode_text = "safe"
      mode_color = rl.Color(0, 255, 0, 230)

    if mode_text:
      mode_font = int(25 * scale)
      mode_size = measure_text_cached(self._font_semi_bold, mode_text, mode_font)

      mode_x = int(panel_x + 5 * scale)
      mode_y = int(panel_y + panel_h * 0.05 - mode_size.y * 0.5 - 15 * scale)

      self._draw_text_with_outline(
        mode_text,
        rl.Vector2(mode_x, mode_y),
        mode_font,
        mode_color,
        rl.BLACK,
        thickness=outline_1,
      )

    # ----- set speed (center area) -----
    show_set = self._engaged and self.is_cruise_set

    if show_set:
      set_speed = self.set_speed
      if not ui_state.is_metric:
        set_speed *= KM_TO_MILE
      set_text = str(int(round(set_speed)))
    else:
      set_text = "--"

    set_color = rl.Color(0, 255, 0, 230)

    if self._debug_speed_panel:
      set_text = "123"

    set_font = int(40 * scale)
    set_size = measure_text_cached(self._font_display, set_text, set_font)

    set_x = int(panel_x + panel_w * 0.76 - set_size.x * 0.5)
    set_y = int(panel_y + panel_h * 0.33 - set_size.y * 0.5)

    self._draw_text_with_outline(
      set_text,
      rl.Vector2(set_x, set_y),
      set_font,
      set_color,
      rl.BLACK,
      thickness=outline_1,
    )

    # ----- override speed / label (upper-right) -----
    if ov.active:
      ov_speed = ov.speed_kph
      if not ui_state.is_metric:
        ov_speed *= KM_TO_MILE
      ov_text = str(int(round(ov_speed)))
      ov_label_text = ov.label

      if ov.speed_color_mode == 1:
        ov_color = rl.Color(0, 255, 0, 230)      # eco
      elif ov.speed_color_mode == 2:
        ov_color = rl.Color(255, 165, 0, 230)    # apply
      else:
        ov_color = rl.Color(0, 255, 0, 230)

      if self._debug_speed_panel:
        ov_text = "111"
        ov_label_text = "vturn"
        ov_color = rl.Color(255, 165, 0, 230)

      ov_font = int(40 * scale)
      ov_size = measure_text_cached(self._font_display, ov_text, ov_font)
      ov_x = int(panel_x + panel_w * 0.90 - ov_size.x * 0.5 + 50 * scale)
      ov_y = int(panel_y + panel_h * 0.25 - ov_size.y * 0.5)

      self._draw_text_with_outline(
        ov_text,
        rl.Vector2(ov_x, ov_y),
        ov_font,
        ov_color,
        rl.BLACK,
        thickness=outline_1,
      )

      label_font = int(30 * scale)
      label_size = measure_text_cached(self._font_display, ov_label_text, label_font)
      label_x = int(panel_x + panel_w * 0.90 - label_size.x * 0.5 + 50 * scale)
      label_y = int(panel_y + panel_h * 0.10 - label_size.y * 0.5 - 20 * scale)

      self._draw_text_with_outline(
        ov_label_text,
        rl.Vector2(label_x, label_y),
        label_font,
        ov_color,
        rl.BLACK,
        thickness=outline_1,
      )

    # ----- cruise gap -----
    gap = self._get_cruise_gap()
    gap_text = str(gap)
    gap_font = int(28 * scale)
    gap_size = measure_text_cached(self._font_semi_bold, gap_text, gap_font)

    gap_center_x = int(panel_x + panel_w * 0.90)
    gap_center_y = int(panel_y + panel_h * 0.82)

    self._draw_text_with_outline(
      gap_text,
      rl.Vector2(gap_center_x - gap_size.x * 0.5, gap_center_y - gap_size.y * 0.5),
      gap_font,
      rl.WHITE,
      rl.BLACK,
      thickness=outline_1,
    )

    # ----- active carrot -----
    sm = ui_state.sm
    active_carrot = sm["carrotMan"].activeCarrot
    if active_carrot >= 2:
      nav_font = int(26 * scale)
      nav_x = int(panel_x + panel_w * 0.60)
      nav_y = int(panel_y + panel_h * 0.82)

      self._draw_text_with_outline(
        "NAV",
        rl.Vector2(nav_x, nav_y),
        nav_font,
        rl.GREEN,
        rl.BLACK,
        thickness=outline_1,
      )

    # ----- gear box -----
    gear = self._get_gear_text()

    box_w = int(44 * scale)
    box_h = int(54 * scale)
    box_x = int(panel_x + panel_w - box_w - 14 * scale + 70 * scale)
    box_y = int(panel_y + panel_h * 0.50)

    box_rect = rl.Rectangle(box_x, box_y, box_w, box_h)

    rl.draw_rectangle_rounded(
      box_rect,
      0.2,
      8,
      rl.Color(0, 0, 0, 120),
    )
    rl.draw_rectangle_rounded_lines_ex(
      box_rect,
      0.2,
      8,
      max(1, int(3 * scale)),
      rl.Color(0, 255, 0, 230),
    )

    gear_font = int(44 * scale)
    gear_size = measure_text_cached(self._font_display, gear, gear_font)

    self._draw_text_with_outline(
      gear,
      rl.Vector2(
        box_x + (box_w - gear_size.x) * 0.5,
        box_y + (box_h - gear_size.y) * 0.5,
      ),
      gear_font,
      rl.WHITE,
      rl.BLACK,
      thickness=outline_1,
    )
