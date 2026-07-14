import colorsys
import numpy as np
import pyray as rl
from openpilot.cereal import messaging, car
from dataclasses import dataclass, field
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_draw import draw_text_ui_style
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from typing import Optional, Any

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]

LANE_LINE_COLORS = {
  UIStatus.DISENGAGED: rl.Color(200, 200, 200, 255),
  UIStatus.OVERRIDE: rl.Color(255, 255, 255, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 64, 255),
}


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  fill_alpha: int = 0
  rect: list[tuple[float, float]] = field(default_factory=list)   # 4 corners (screen space)
  color: Optional[Any] = None

@dataclass
class RadarInfoItem:
  x: float = 0.0
  y: float = 0.0
  w: float = 0.0
  h: float = 0.0
  text: str = ""
  color: Optional[Any] = None
  is_star: bool = False
  
class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    self._acceleration_x_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._acceleration_x_filter2 = FirstOrderFilter(0.0, 1, 1 / gui_app.target_fps)

    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._ll_color_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None
    
    self._lead_pt_filt = [None, None]
    self._radar_info_items: list[RadarInfoItem] = []
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)
    self._tire_trajectory = False
    self._tire_trajectory_param_time = -1.0

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm
    self._refresh_tire_trajectory_param()

    self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

    # Check if data is up-to-date
    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode

    live_calib = sm['liveCalibration']
    self._path_offset_z = live_calib.height[0] if live_calib.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads_carrot(radar_state, path_x_array)
        
      if ui_state.show_radar_info > 0 and radar_state is not None:
        self._update_radar_info(radar_state, path_x_array)
      else:
        self._radar_info_items = []
        
      self._transform_dirty = False

    # Draw elements (hide when disengaged)
    self._draw_lane_lines()
    if ui_state.status != UIStatus.DISENGAGED:
      #self._draw_lane_lines()
      self._draw_path(sm)
      self._draw_tire_trajectory(sm)

    if render_lead_indicator and radar_state:
      self._draw_lead_indicator()
    if ui_state.show_radar_info > 0:
      self._draw_radar_info()
      
  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)
          
  def _update_leads_carrot(self, radar_state, path_x_array):
    """Carrot: draw leadOne as outline rectangle."""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]

    lead_one = radar_state.leadOne

    def _filter_pt(slot: int, pt: tuple[float, float], alpha: float = 0.2):
      prev = self._lead_pt_filt[slot]
      if prev is None:
        self._lead_pt_filt[slot] = (float(pt[0]), float(pt[1]))
        return float(pt[0]), float(pt[1])

      x = prev[0] + (float(pt[0]) - prev[0]) * alpha
      y = prev[1] + (float(pt[1]) - prev[1]) * alpha
      self._lead_pt_filt[slot] = (x, y)
      return x, y

    if lead_one and lead_one.status:
      d_rel = float(lead_one.dRel)
      y_rel = float(lead_one.yRel)

      idx = self._get_path_length_idx(path_x_array, d_rel)
      z = float(self._path.raw_points[idx, 2]) if idx < len(self._path.raw_points) else 0.0

      pt_left = self._map_to_screen(d_rel, -y_rel - 1.2, z + self._path_offset_z)
      pt_right = self._map_to_screen(d_rel, -y_rel + 1.2, z + self._path_offset_z)

      if pt_left and pt_right:
        # 중심은 좌/우 평균으로 잡아야 함
        center_x = (pt_left[0] + pt_right[0]) * 0.5
        center_y = (pt_left[1] + pt_right[1]) * 0.5

        # 폭은 좌우 거리 기준, clamp는 유지
        path_width = float(np.clip(abs(pt_right[0] - pt_left[0]), 60.0, 400.0))

        # 중심 필터
        pt_x, pt_y = _filter_pt(0, (center_x, center_y), alpha=0.2)

        rect_x_min = self._rect.x
        rect_x_max = self._rect.x + self._rect.width
        rect_y_min = self._rect.y
        rect_y_max = self._rect.y + self._rect.height

        half_w = path_width * 0.5
        rect_h = path_width * 0.8

        left = float(np.clip(pt_x - half_w, rect_x_min, rect_x_max))
        right = float(np.clip(pt_x + half_w, rect_x_min, rect_x_max))
        bottom_y = float(np.clip(pt_y, rect_y_min, rect_y_max))
        top = float(np.clip(pt_y - rect_h, rect_y_min, rect_y_max))

        if not bool(lead_one.radar):
          c = rl.Color(0, 120, 255, 255)   # BLUE
        else:
          track_id = int(getattr(lead_one, "radarTrackId", 0))
          if track_id in (0, 1):
            c = rl.Color(201, 34, 49, 255) # RED
          else:
            c = rl.Color(255, 115, 0, 255) # ORANGE

        self._lead_vehicles[0] = LeadVehicle(
          rect=[(left, top), (right, top), (right, bottom_y), (left, bottom_y)],
          color=c,
        )
      else:
        self._lead_pt_filt[0] = None
        
  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    line_width_factor = 0.12
    for i, lane_line in enumerate(self._lane_lines):
      if i in (1, 2):
        line_width_factor = 0.16
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, line_width_factor * self._lane_line_probs[i], 0.0, max_idx
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, line_width_factor, 0.0, max_idx)

    # Update path using raw points
    if lead and lead.status:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    soon_acceleration = self._acceleration_x[len(self._acceleration_x) // 4] if len(self._acceleration_x) > 0 else 0
    self._acceleration_x_filter.update(soon_acceleration)
    self._acceleration_x_filter2.update(soon_acceleration)

    # make path width wider/thinner when initially braking/accelerating
    if self._experimental_mode and False:
      high_pass_acceleration = self._acceleration_x_filter.x - self._acceleration_x_filter2.x
      y_off = np.interp(high_pass_acceleration, [-1, 0, 1], [0.9 * 2, 0.9, 0.9 / 2])
    else:
      y_off = 0.9

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, y_off, self._path_offset_z, max_idx, allow_invert=False
    )

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient.colors = segment_colors
    self._exp_gradient.stops = gradient_stops

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 1
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    alpha = np.clip(prob, 0.0, 0.7)
    if adjacent:
      _base_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED if not ui_state.lat_active else UIStatus.ENGAGED])
      color = rl.Color(_base_color.r, _base_color.g, _base_color.b, int(alpha * 255))

      # turn adjacent lls orange if torque is high
      torque = self._torque_filter.x
      high_torque = abs(torque) > 0.6
      if high_torque and (left == (torque > 0)):
        color = blend_colors(
          color,
          rl.Color(255, 115, 0, int(alpha * 255)),  # orange
          np.interp(abs(torque), [0.6, 0.8], [0.0, 1.0])
        )
    else:
      color = rl.Color(255, 255, 255, int(alpha * 255))

    if ui_state.status == UIStatus.DISENGAGED and not ui_state.lat_active:
      color = rl.Color(0, 0, 0, int(alpha * 255))

    return color

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    """Two closest lines should be green (lane line or road edges)"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      # if closest lane lines are not confident, make road edges green
      color = self._get_ll_color(float(1.0 - self._road_edge_stds[i]), float(self._lane_line_probs[i + 1]) < 0.25, i == 0)
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _refresh_tire_trajectory_param(self) -> None:
    now = rl.get_time()
    if now - self._tire_trajectory_param_time < 1.0:
      return
    self._tire_trajectory = ui_state.params.get_bool("CarrotTireTrajectory")
    self._tire_trajectory_param_time = now

  @staticmethod
  def _tire_gradient_color(danger_ratio: float) -> rl.Color:
    if danger_ratio <= 0.10:
      red, green = 0.0, 180.0
    elif danger_ratio <= 0.60:
      factor = (danger_ratio - 0.10) / 0.50
      red, green = factor * 255.0, 180.0
    elif danger_ratio <= 1.00:
      factor = (danger_ratio - 0.60) / 0.40
      red, green = 255.0, 180.0 - factor * 180.0
    else:
      red, green = 255.0, 0.0
    return rl.Color(int(red), int(green), 0, 255)

  @staticmethod
  def _draw_tire_edge_band(
    left_points: np.ndarray,
    right_points: np.ndarray,
    is_left: bool,
    color: rl.Color,
  ) -> None:
    n_edge = min(len(left_points), len(right_points))
    if n_edge < 2:
      return

    rl.rl_disable_texture()
    rl.rl_begin(rl.RL_QUADS)
    try:
      for i in range(n_edge - 1):
        lx0, ly0 = map(float, left_points[i])
        rx0, ry0 = map(float, right_points[i])
        lx1, ly1 = map(float, left_points[i + 1])
        rx1, ry1 = map(float, right_points[i + 1])
        outer_y0 = ly0 if is_left else ry0
        outer_y1 = ly1 if is_left else ry1
        subdivisions = max(1, int(abs(outer_y1 - outer_y0) / 5.0))

        for subdivision in range(subdivisions):
          t0 = subdivision / subdivisions
          t1 = (subdivision + 1) / subdivisions
          slx_b = lx0 + (lx1 - lx0) * t0
          sly_b = ly0 + (ly1 - ly0) * t0
          srx_b = rx0 + (rx1 - rx0) * t0
          sry_b = ry0 + (ry1 - ry0) * t0
          slx_t = lx0 + (lx1 - lx0) * t1
          sly_t = ly0 + (ly1 - ly0) * t1
          srx_t = rx0 + (rx1 - rx0) * t1
          sry_t = ry0 + (ry1 - ry0) * t1

          band_width_bottom = abs(slx_b - srx_b) / 6.0
          band_width_top = abs(slx_t - srx_t) / 6.0
          outer_x_bottom = slx_b if is_left else srx_b
          outer_y_bottom = sly_b if is_left else sry_b
          outer_x_top = slx_t if is_left else srx_t
          outer_y_top = sly_t if is_left else sry_t
          center_bottom = (slx_b + srx_b) * 0.5
          direction = 1.0 if center_bottom > outer_x_bottom else -1.0
          inner_x_bottom = outer_x_bottom + direction * band_width_bottom
          inner_x_top = outer_x_top + direction * band_width_top

          edge_dx = outer_x_top - outer_x_bottom
          edge_dy = outer_y_top - outer_y_bottom
          if edge_dx * edge_dx + edge_dy * edge_dy < 0.25:
            continue

          rl.rl_color4ub(color.r, color.g, color.b, color.a)
          rl.rl_vertex2f(outer_x_bottom, outer_y_bottom)
          rl.rl_color4ub(color.r, color.g, color.b, 0)
          rl.rl_vertex2f(inner_x_bottom, outer_y_bottom)
          rl.rl_color4ub(color.r, color.g, color.b, 0)
          rl.rl_vertex2f(inner_x_top, outer_y_top)
          rl.rl_color4ub(color.r, color.g, color.b, color.a)
          rl.rl_vertex2f(outer_x_top, outer_y_top)
    finally:
      rl.rl_end()

  def _draw_tire_trajectory(self, sm) -> None:
    if not self._tire_trajectory or not sm.alive['modelV2'] or not sm.valid['modelV2']:
      return

    projected_points = self._path.projected_points
    side_length = len(projected_points) // 2
    if side_length < 2:
      return

    left_points = projected_points[:side_length]
    right_points = projected_points[side_length:side_length * 2][::-1]

    model = sm['modelV2']
    lane_lines = model.laneLines
    lane_line_probs = model.laneLineProbs
    if len(lane_lines) < 3 or len(lane_line_probs) < 3 or len(lane_lines[1].y) == 0 or len(lane_lines[2].y) == 0:
      return

    left_m = float(lane_lines[1].y[0])
    right_m = -float(lane_lines[2].y[0])
    if lane_line_probs[1] < 0.3 and lane_line_probs[2] > 0.3:
      left_m = 3.0 - right_m
    elif lane_line_probs[2] < 0.3 and lane_line_probs[1] > 0.3:
      right_m = 3.0 - left_m

    total_width = left_m + right_m
    if total_width < 0.5:
      total_width = 3.0
    offset_m = (left_m - right_m) / 2.0
    danger_ratio = abs(offset_m) / max(total_width / 2.0 - 0.9, 0.1)

    lane_warning = abs(offset_m) >= 0.5
    warning_pulse = lane_warning and int(rl.get_time() * 1000) % 800 < 400
    edge_green = rl.Color(0, 200, 0, 255)
    if lane_warning:
      warning_color = rl.Color(255, 0, 0, 255) if warning_pulse else rl.Color(255, 100, 0, 200)
      left_color = warning_color if offset_m > 0.0 else edge_green
      right_color = warning_color if offset_m < 0.0 else edge_green
    elif offset_m > 0.02:
      left_color = self._tire_gradient_color(danger_ratio)
      right_color = edge_green
    elif offset_m < -0.02:
      left_color = edge_green
      right_color = self._tire_gradient_color(danger_ratio)
    else:
      left_color = right_color = edge_green

    self._draw_tire_edge_band(left_points, right_points, True, left_color)
    self._draw_tire_edge_band(left_points, right_points, False, right_color)

    if danger_ratio < 1.5:
      center_x = int(self._rect.x + self._rect.width / 2.0)
      text_y = int(np.clip(
        float(left_points[0][1]) - 40.0,
        self._rect.y + 26.0,
        self._rect.y + self._rect.height - 44.0,
      ))
      direction_label = "L" if offset_m > 0.02 else ("R" if offset_m < -0.02 else "C")
      draw_text_ui_style(
        direction_label, center_x, text_y, 24, rl.WHITE,
        font=self._font_display, border_width=1.0, shadow_offset=0.0, align="center",
      )
      draw_text_ui_style(
        f"{abs(offset_m):.2f}", center_x, text_y + 22, 18, rl.WHITE,
        font=self._font_display, border_width=1.0, shadow_offset=0.0, align="center",
      )

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if self._experimental_mode:
      # Draw with acceleration coloring
      if ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      elif len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )

      if ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      else:
        draw_polygon(self._rect, self._path.projected_points, gradient=gradient)

  def _draw_lead_indicator_old(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

  def _draw_lead_indicator(self):
    # Carrot: draw outline rectangles only (no fill)
    thickness = 4.0  # 원하는 두께 (float)

    for lead in self._lead_vehicles:
      if not lead.rect or lead.color is None:
        continue

      pts = lead.rect
      c = lead.color

      rl.draw_line_ex(pts[0], pts[1], thickness, c)
      rl.draw_line_ex(pts[1], pts[2], thickness, c)
      rl.draw_line_ex(pts[2], pts[3], thickness, c)
      rl.draw_line_ex(pts[3], pts[0], thickness, c)
    
  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_height: float) -> int:
    """Get the index corresponding to the given path height"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_height)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]
    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  @staticmethod
  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]

  def _get_radar_info_color(self, lead, v_sum: float):
    radar = bool(getattr(lead, "radar", False))
    model_prob = float(getattr(lead, "modelProb", 0.0))

    if not radar:
      return rl.Color(0, 0, 255, 220)      # BLUE
    elif abs(model_prob - 0.01) < 0.001:
      return rl.Color(0, 203, 0, 220)       # GREEN
    elif v_sum > 0.0:
      return rl.Color(255, 175, 3, 220)      # ORANGE
    else:
      return rl.Color(255, 0, 0, 220)      # RED

  def _update_radar_info(self, radar_state, path_x_array):
    self._radar_info_items = []

    if self._lane_lines[2].raw_points.shape[0] == 0:
      return

    leads_groups = [
      radar_state.leadsLeft,
      radar_state.leadsRight,
      radar_state.leadsCenter,
    ]

    rect_x_min = self._rect.x
    rect_x_max = self._rect.x + self._rect.width
    rect_y_min = self._rect.y
    rect_y_max = self._rect.y + self._rect.height

    for leads in leads_groups:
      for l in leads:
        d_rel = float(getattr(l, "dRel", 0.0))
        y_rel = float(getattr(l, "yRel", 0.0))

        if d_rel <= 2.5:
          continue

        idx = self._get_path_length_idx(self._lane_lines[2].raw_points[:, 0], d_rel)
        if idx >= len(self._lane_lines[2].raw_points):
          continue

        lane_z = float(self._lane_lines[2].raw_points[idx, 2]) - 0.61
        pt = self._map_to_screen(d_rel, -y_rel, lane_z)
        if not pt:
          continue

        x, y = float(pt[0]), float(pt[1])

        v = float(getattr(l, "vLeadK", 0.0))
        v_lat = float(getattr(l, "vLat", 0.0))
        v_abs = float(np.sqrt(v * v + v_lat * v_lat))
        v_sum = v_abs if v >= 0.0 else -v_abs

        # 정지는 기존처럼 "*"
        if v_abs <= 3.0:
          self._radar_info_items.append(
            RadarInfoItem(
              x=x,
              y=y,
              w=18.0,
              h=18.0,
              text="*",
              color=rl.Color(255, 255, 255, 230),
              is_star=True,
            )
          )
          continue

        speed_val = v_sum * (3.6 if ui_state.is_metric else 2.2369363)
        text = f"{speed_val:.0f}"

        font_size = 22
        pad_x = 6
        pad_y = 2

        text_w = rl.measure_text(text, font_size)
        box_w = float(text_w + pad_x * 2)
        box_h = float(font_size + pad_y * 2)

        box_x = float(np.clip(x - box_w * 0.5, rect_x_min, rect_x_max - box_w))
        box_y = float(np.clip(y - box_h * 0.5, rect_y_min, rect_y_max - box_h))

        color = self._get_radar_info_color(l, v_sum)

        self._radar_info_items.append(
          RadarInfoItem(
            x=box_x,
            y=box_y,
            w=float(box_w),
            h=float(box_h),
            text=text,
            color=color,
            is_star=False,
          )
        )


  def _draw_radar_info(self):
    if not self._radar_info_items:
      return

    font_size = 22

    for item in self._radar_info_items:
      if item.color is None:
        continue

      if item.is_star:
        tw = rl.measure_text(item.text, font_size)
        draw_text_ui_style(item.text, item.x, item.y, font_size, item.color, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="center", y_offset=0.0)
        continue

      # 박스
      rl.draw_rectangle_rounded(
        rl.Rectangle(item.x, item.y, item.w, item.h),
        0.28,
        8,
        item.color,
      )

      # 텍스트 중앙정렬
      tw = rl.measure_text(item.text, font_size)
      tx = int(item.x + (item.w - tw) / 2)
      ty = int(item.y + (item.h - font_size) / 2 - 1)

      #rl.draw_text(
      #  item.text,
      #  tx,
      #  ty,
      #  font_size,
      #  rl.WHITE,
      #)

      draw_text_ui_style(item.text, tx, ty, font_size, rl.WHITE, font=self._font_display, border_width=1.0, shadow_offset=8.0, align="left_top", y_offset=0.0)

