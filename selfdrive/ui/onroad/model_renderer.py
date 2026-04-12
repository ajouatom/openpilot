import math
import colorsys
import numpy as np
import pyray as rl
from cereal import messaging, car
from dataclasses import dataclass, field
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

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


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  fill_alpha: int = 0

@dataclass
class RadarLeadInfo:
  x: float = 0.0
  y: float = 0.0
  ax: float = 0.0
  ay: float = 0.0
  d_rel: float = 0.0
  y_rel: float = 0.0
  v_rel: float = 0.0
  v_lat: float = 0.0
  v_sum: float = 0.0
  radar: bool = False
  model_prob: float = 0.0
  has_future_point: bool = False
  
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

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

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

    self._init_carrot()

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

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
        self._update_leads(radar_state, path_x_array)
      self._transform_dirty = False

    # Draw elements
    #self._draw_lane_lines()
    #self._draw_path(sm)

    #if render_lead_indicator and radar_state:
    #  self._draw_lead_indicator()
    self._draw_path_carrot(sm)
    self._draw_lane_lines_carrot(sm)
    self._draw_radar_info_carrot(sm)

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

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    for i, lane_line in enumerate(self._lane_lines):
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, 0.025 * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, 0.025, 0.0, max_idx, max_distance)

    # Update path using raw points
    if lead and lead.status:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, 0.9, self._path_offset_z, max_idx, max_distance, allow_invert=False
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
    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=segment_colors,
      stops=gradient_stops,
    )

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
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      alpha = np.clip(self._lane_line_probs[i], 0.0, 0.7)
      color = rl.Color(255, 255, 255, int(alpha * 255))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      color = rl.Color(255, 0, 0, int(alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if self._experimental_mode:
      # Draw with acceleration coloring
      if len(self._exp_gradient.colors) > 1:
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
      draw_polygon(self._rect, self._path.projected_points, gradient=gradient)

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_distance: float) -> int:
    """Get the index corresponding to the given path distance"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_distance)[0]
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

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, max_distance: float, allow_invert: bool = True, y_shift: float = 0.0) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]

    # Interpolate around max_idx so path end is smooth (max_distance is always >= p0.x)
    if 0 < max_idx < line.shape[0] - 1:
      p0 = line[max_idx]
      p1 = line[max_idx + 1]
      x0, x1 = p0[0], p1[0]
      interp_y = np.interp(max_distance, [x0, x1], [p0[1], p1[1]])
      interp_z = np.interp(max_distance, [x0, x1], [p0[2], p1[2]])
      interp_point = np.array([max_distance, interp_y, interp_z], dtype=points.dtype)
      points = np.concatenate((points, interp_point[None, :]), axis=0)

    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off + y_shift, z_off], [0, y_off + y_shift, z_off]], dtype=np.float32)
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


  def _init_carrot(self):
    self._carrot_path_mode = 13
    self._carrot_path_color = 14
    self._carrot_path_mode_lane = 13
    self._carrot_path_color_lane = 14
    self._carrot_path_color_cruise_off = 14
    self._carrot_show_lane_info = 1
    self._carrot_show_radar_info = 0
    self._carrot_radar_lat_factor = 1.0

    self._carrot_path_fx = 0.0
    self._carrot_path_fy = 0.0
    self._carrot_path_fwidth = 0.0
    self._carrot_path_x = 0
    self._carrot_path_y = 0
    self._carrot_path_width = 0
    self._carrot_path_alpha = 0.85

    self._carrot_radar_leads = []
    self._carrot_tf_left = None
    self._carrot_tf_right = None
    self._carrot_tf_distance = 0.0
    self._carrot_t_follow = 0.0

    self._carrot_lead_one_left = None
    self._carrot_lead_one_right = None
    self._carrot_lead_two_left = None
    self._carrot_lead_two_right = None
    self._carrot_lead_two_xl = 0.0
    self._carrot_lead_two_xr = 0.0
    self._carrot_lead_two_y = 0.0
    self._carrot_lead_two_status = 0
    self._carrot_radar_track_id = -1
    self._carrot_radar_dist = 0.0
    self._carrot_vision_dist = 0.0
    self._carrot_lead_status = False
    
    self._carrot_path_draw_seq = 0.0
    self._carrot_path_forward = True
    self._carrot_use_lane_line_speed_apply = 0

    self._carrot_show_path_width = 1.0

  def _refresh_carrot_params(self):
    self._carrot_show_lane_info = ui_state.params.get_int("ShowLaneInfo")
    self._carrot_show_radar_info = ui_state.params.get_int("ShowRadarInfo")
    self._carrot_radar_lat_factor = ui_state.params.get_float("RadarLatFactor") / 100.0

    self._carrot_path_mode = ui_state.params.get_int("ShowPathMode")
    self._carrot_path_color = ui_state.params.get_int("ShowPathColor")
    self._carrot_path_mode_lane = ui_state.params.get_int("ShowPathModeLane")
    self._carrot_path_color_lane = ui_state.params.get_int("ShowPathColorLane")
    self._carrot_path_color_cruise_off = ui_state.params.get_int("ShowPathColorCruiseOff")

  def _draw_path_carrot(self, sm):
    if not self._path.raw_points.size:
      return

    self._refresh_carrot_params()
    self._update_path_end_carrot(sm)

    car_state = sm['carState']
    controls_state = sm['controlsState']
    selfdrive_state = sm['selfdriveState']
    longitudinal_plan = sm['longitudinalPlan']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state is not None else None

    show_path_mode = self._carrot_path_mode
    show_path_color = self._carrot_path_color

    active_lane_line = controls_state.activeLaneLine
    if active_lane_line:
      show_path_mode = self._carrot_path_mode_lane
      show_path_color = self._carrot_path_color_lane

    long_active = selfdrive_state.enabled
    if not long_active:
      show_path_color = self._carrot_path_color_cruise_off

    brake_valid = car_state.brakeLights
    accel = longitudinal_plan.accels[0] if len(longitudinal_plan.accels) > 0 else 0.0

    if show_path_color >= 20:
      if long_active:
        show_path_color = 13
        if lead_one is not None and lead_one.status:
          if abs(accel) < 0.5:
            show_path_color = 12
          elif accel >= 0.5:
            show_path_color = 11
          else:
            show_path_color = 10
      else:
        show_path_color = 19

    self._carrot_path_draw_seq += 0.12
    if self._carrot_path_draw_seq > 10000.0:
      self._carrot_path_draw_seq = 0.0

    if show_path_mode == 0:
      track_vertices = self._build_path_polygon_mode0_carrot(sm)
      if track_vertices.size == 0:
        return
      fill_color = self._get_path_color_carrot(show_path_color)
      draw_polygon(self._rect, track_vertices, fill_color)
      if show_path_color >= 10 or brake_valid:
        self._draw_polygon_outline_carrot(
          track_vertices,
          rl.Color(255, 0, 0, 255) if brake_valid else rl.Color(255, 255, 255, 255),
          2.0,
        )
    elif 13 <= show_path_mode <= 15:
      self._draw_special_modes_carrot(sm, show_path_mode, show_path_color, brake_valid)
    elif show_path_mode >= 9:
      self._draw_complex_path_carrot(sm, show_path_mode, show_path_color, brake_valid)
    else:
      self._draw_animated_path_carrot(sm, show_path_mode, show_path_color, brake_valid)

    self._draw_path_end_overlay_carrot(sm)

  def _draw_lane_lines_carrot(self, sm):
    if self._carrot_show_lane_info < 1:
      return

    car_state = sm['carState']
    left_lane_line = car_state.leftLaneLine
    right_lane_line = car_state.rightLaneLine

    path_x_array = self._path.raw_points[:, 0]
    if path_x_array.size == 0:
      return

    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    lane_max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      alpha = 220 if self._lane_line_probs[i] > 0.3 else 0
      stroke_width = 0.0

      if i == 1:
        if left_lane_line >= 20:
          color = rl.Color(218, 202, 37, alpha)
          stroke_width = 1.0
        else:
          color = rl.Color(255, 255, 255, alpha)
      elif i == 2:
        if right_lane_line >= 20:
          color = rl.Color(218, 202, 37, alpha)
        else:
          color = rl.Color(255, 255, 255, alpha)
      else:
        color = rl.Color(255, 255, 255, alpha)

      draw_polygon(self._rect, lane_line.projected_points, color)

      if stroke_width > 0.0:
        self._draw_polygon_outline_carrot(lane_line.projected_points, color, stroke_width)

      if i == 1 and (left_lane_line % 10 == 4):
        double_points = self._map_line_to_polygon(
          self._lane_lines[i].raw_points,
          0.05,
          0.0,
          lane_max_idx,
          max_distance,
          allow_invert=True,
          y_shift=-0.3,
        )
        if double_points.size != 0:
          draw_polygon(self._rect, double_points, color)
          if stroke_width > 0.0:
            self._draw_polygon_outline_carrot(double_points, color, stroke_width)

    if self._carrot_show_lane_info > 1:
      self._draw_road_edges_carrot()


  def _draw_road_edges_carrot(self):
    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      temp_f = float(np.clip(self._road_edge_stds[i] / 2.0, 0.0, 1.0))
      color = rl.Color(
        int((1.0 - temp_f) * 255.0),
        0,
        int(temp_f * 255.0),
        255,
      )
      draw_polygon(self._rect, road_edge.projected_points, color)


  def _update_path_end_carrot(self, sm):
    if self._path.raw_points.shape[0] == 0:
      return

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state is not None else None
    lead_two = radar_state.leadTwo if radar_state is not None else None
    longitudinal_plan = sm['longitudinalPlan']
    selfdrive_state = sm['selfdriveState']
    car_state = sm['carState']

    line = self._path.raw_points
    max_distance = np.clip(line[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)

    idx = self._get_path_length_idx(line[:, 0], max_distance)
    y = line[idx, 1]
    z = line[idx, 2]

    leads_v3 = model.leadsV3
    if len(leads_v3) > 0 and leads_v3[0].prob > 0.5:
      self._carrot_vision_dist = leads_v3[0].x[0] - 1.52
    else:
      self._carrot_vision_dist = 0.0

    self._carrot_lead_status = False
    self._carrot_radar_track_id = -1
    self._carrot_radar_dist = 0.0

    if lead_one is not None and lead_one.status:
      lead_idx = self._get_path_length_idx(line[:, 0], lead_one.dRel)
      z = line[lead_idx, 2]
      max_distance = lead_one.dRel
      y = -lead_one.yRel
      self._carrot_radar_track_id = lead_one.radarTrackId
      self._carrot_radar_dist = lead_one.dRel if lead_one.radar else 0.0
      self._carrot_lead_status = True

    self._carrot_lead_one_left = self._map_to_screen(max_distance, y - 1.2, z + 1.22)
    self._carrot_lead_one_right = self._map_to_screen(max_distance, y + 1.2, z + 1.22)

    if self._carrot_lead_one_left is not None and self._carrot_lead_one_right is not None:
      lex = self._carrot_lead_one_left[0]
      ley = self._carrot_lead_one_left[1]
      rex = self._carrot_lead_one_right[0]
      rey = self._carrot_lead_one_right[1]

      path_width = rex - lex
      path_x = (lex + rex) / 2.0
      path_y = (ley + rey) / 2.0

      path_x = float(np.clip(path_x, 350.0, self._rect.width - 350.0))
      path_y = float(np.clip(path_y, 200.0, self._rect.height - 80.0))

      self._carrot_path_fx = self._carrot_path_fx * self._carrot_path_alpha + path_x * (1.0 - self._carrot_path_alpha)
      self._carrot_path_fy = self._carrot_path_fy * self._carrot_path_alpha + path_y * (1.0 - self._carrot_path_alpha)

      if path_width < 120.0:
        path_width = 120.0
      elif path_width > 800.0:
        path_width = 800.0

      self._carrot_path_fwidth = self._carrot_path_fwidth * self._carrot_path_alpha + path_width * (1.0 - self._carrot_path_alpha)

      self._carrot_path_x = int(self._carrot_path_fx)
      self._carrot_path_y = int(self._carrot_path_fy)
      self._carrot_path_width = int(self._carrot_path_fwidth)

    self._carrot_t_follow = longitudinal_plan.tFollow
    self._carrot_tf_distance = longitudinal_plan.desiredDistance

    if self._carrot_tf_distance > 0.0:
      tf_idx = self._get_path_length_idx(line[:, 0], self._carrot_tf_distance)
      tf_y = line[tf_idx, 1]
      tf_z = line[tf_idx, 2]
      self._carrot_tf_left = self._map_to_screen(self._carrot_tf_distance, tf_y - 1.0, tf_z + 1.22)
      self._carrot_tf_right = self._map_to_screen(self._carrot_tf_distance, tf_y + 1.0, tf_z + 1.22)
    else:
      self._carrot_tf_left = None
      self._carrot_tf_right = None

    if lead_two is not None and self._carrot_lead_status and lead_two.radar and lead_two.dRel > lead_one.dRel + 3.0:
      lead_two_idx = self._get_path_length_idx(line[:, 0], lead_two.dRel)
      lead_two_z = line[lead_two_idx, 2]
      lead_two_y = -lead_two.yRel

      self._carrot_lead_two_left = self._map_to_screen(lead_two.dRel, lead_two_y - 1.2, lead_two_z + 1.22)
      self._carrot_lead_two_right = self._map_to_screen(lead_two.dRel, lead_two_y + 1.2, lead_two_z + 1.22)

      if self._carrot_lead_two_left is not None and self._carrot_lead_two_right is not None:
        if self._carrot_lead_two_status > 0:
          self._carrot_lead_two_xl = self._carrot_lead_two_xl * 0.8 + self._carrot_lead_two_left[0] * 0.2
          self._carrot_lead_two_xr = self._carrot_lead_two_xr * 0.8 + self._carrot_lead_two_right[0] * 0.2
          self._carrot_lead_two_y = self._carrot_lead_two_y * 0.8 + self._carrot_lead_two_left[1] * 0.2
        else:
          self._carrot_lead_two_xl = self._carrot_lead_two_left[0]
          self._carrot_lead_two_xr = self._carrot_lead_two_right[0]
          self._carrot_lead_two_y = self._carrot_lead_two_left[1]

        if longitudinal_plan.longitudinalPlanSource == longitudinal_plan.LongitudinalPlanSource.lead1:
          self._carrot_lead_two_status = 2
        else:
          self._carrot_lead_two_status = 1
    else:
      self._carrot_lead_two_status = 0


  def _draw_path_end_overlay_carrot(self, sm):
    if not self._carrot_lead_status:
      return

    lead_is_scc = self._carrot_radar_track_id < 1
    radar_detected = self._carrot_radar_track_id >= 0
    rcolor = rl.Color(255, 0, 0, 255) if lead_is_scc else rl.Color(255, 175, 3, 255)

    if self._carrot_lead_two_status > 0:
      radar_stroke = rl.Color(218, 111, 37, 255)
      path_width2 = int(self._carrot_lead_two_xr - self._carrot_lead_two_xl)
      fill_color = rl.Color(255, 0, 0, 50) if self._carrot_lead_two_status == 2 else rl.Color(0, 0, 0, 20)
      self._draw_rect_fill_outline_carrot(
        self._carrot_lead_two_xl - 10,
        self._carrot_lead_two_y - path_width2 * 0.8,
        path_width2 + 20,
        path_width2 * 0.8,
        fill_color,
        radar_stroke,
        3.0,
      )

    radar_stroke = rcolor if radar_detected else rl.Color(0, 0, 255, 255)
    self._draw_rect_fill_outline_carrot(
      self._carrot_path_x - self._carrot_path_width / 2 - 10,
      self._carrot_path_y - self._carrot_path_width * 0.8,
      self._carrot_path_width + 20,
      self._carrot_path_width * 0.8,
      rl.Color(0, 0, 0, 20),
      radar_stroke,
      3.0,
    )

    if self._carrot_tf_left is not None and self._carrot_tf_right is not None:
      self._draw_line_segment_carrot(self._carrot_tf_left, self._carrot_tf_right, rl.Color(255, 255, 255, 255), 3.0)
      tf_text = f"{self._carrot_tf_distance:.1f}({self._carrot_t_follow:.2f})"
      self._draw_text_carrot(int(self._carrot_tf_right[0]), int(self._carrot_tf_right[1]), tf_text, 25, rl.Color(255, 255, 255, 255))

  def _interp_carrot(self, x: float, xp, fp) -> float:
    return float(np.interp(x, xp, fp))


  def _build_path_polygon_mode0_carrot(self, sm) -> np.ndarray:
    line = self._path.raw_points
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    max_distance = np.clip(line[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    if radar_state is not None and radar_state.leadOne.status:
      lead_d = radar_state.leadOne.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(line[:, 0], max_distance)
    return self._map_line_to_polygon(
      line,
      0.9 * self._carrot_show_path_width,
      self._path_offset_z,
      max_idx,
      max_distance,
      allow_invert=False,
    )


  def _build_path_polygon_lineardepth_carrot(
      self,
      line: np.ndarray,
      width_apply: float,
      z_off_start: float,
      z_off_end: float,
      max_idx: int,
      allow_invert: bool = True,
  ) -> np.ndarray:
    if line.shape[0] == 0 or max_idx < 0:
      return np.empty((0, 2), dtype=np.float32)

    points = line[:max_idx + 1]
    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    left_pts = []
    right_pts = []

    for i in range(points.shape[0]):
      x = float(points[i, 0])
      y = float(points[i, 1])
      z = float(points[i, 2])

      z_off = self._interp_carrot(x, [0.0, 100.0], [z_off_start, z_off_end])
      y_off = self._interp_carrot(z_off, [-3.0, 0.0, 3.0], [1.5, 0.5, 1.5]) * width_apply

      left = self._map_to_screen(x, y - y_off, z + z_off)
      right = self._map_to_screen(x, y + y_off, z + z_off)
      if left is None or right is None:
        continue

      if not allow_invert and len(left_pts) > 0 and left[1] > left_pts[-1][1]:
        continue

      left_pts.append(left)
      right_pts.insert(0, right)

    if len(left_pts) == 0 or len(right_pts) == 0:
      return np.empty((0, 2), dtype=np.float32)

    return np.array(left_pts + right_pts, dtype=np.float32)


  def _build_path_polygon_dist_carrot(
      self,
      line: np.ndarray,
      width_apply: float,
      z_off_start: float,
      z_off_end: float,
      max_dist: float,
      allow_invert: bool = True,
  ) -> np.ndarray:
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    line_x = line[:, 0].astype(np.float32).copy()
    line_y = line[:, 1].astype(np.float32).copy()
    line_z = line[:, 2].astype(np.float32).copy()

    for i in range(1, line_x.shape[0]):
      if line_x[i] < line_x[i - 1]:
        line_x[i] = line_x[i - 1]

    idxs = np.arange(line_x.shape[0], dtype=np.float32)

    left_pts = []
    right_pts = []

    dist = 2.0
    exit_flag = False
    while not exit_flag:
      if dist >= max_dist:
        dist = max_dist
        exit_flag = True

      z_off = self._interp_carrot(dist, [0.0, 100.0], [z_off_start, z_off_end])
      y_off = self._interp_carrot(z_off, [-3.0, 0.0, 3.0], [1.5, 0.5, 1.5]) * width_apply

      idx = self._interp_carrot(dist, line_x, idxs)
      if idx >= line_x.shape[0] - 1:
        idx = line_x.shape[0] - 1

      y = self._interp_carrot(idx, idxs, line_y)
      z = self._interp_carrot(idx, idxs, line_z)

      left = self._map_to_screen(dist, y - y_off, z + z_off)
      right = self._map_to_screen(dist, y + y_off, z + z_off)
      if left is not None and right is not None:
        if not allow_invert and len(left_pts) > 0 and left[1] > left_pts[-1][1]:
          dist = dist + dist * 0.15
          continue

        left_pts.append(left)
        right_pts.insert(0, right)

      dist = dist + dist * 0.15

    if len(left_pts) == 0 or len(right_pts) == 0:
      return np.empty((0, 2), dtype=np.float32)

    return np.array(left_pts + right_pts, dtype=np.float32)

  def _draw_special_modes_carrot(self, sm, mode: int, color_idx: int, brake_valid: bool):
    line = self._path.raw_points
    if line.shape[0] == 0:
      return

    max_distance = np.clip(line[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(line[:, 0], max_distance)

    fill_color = self._get_path_color_carrot(color_idx)
    stroke_color = rl.Color(255, 0, 0, 255) if brake_valid else rl.Color(255, 255, 255, 255)
    stroke_width = 2.0 if (color_idx >= 10 or brake_valid) else 0.0

    if mode == 13:
      track_vertices = self._build_path_polygon_lineardepth_carrot(
        line,
        1.0 * self._carrot_show_path_width,
        self._path_offset_z,
        self._path_offset_z,
        max_idx,
        allow_invert=False,
      )
    elif mode == 14:
      track_vertices = self._build_path_polygon_lineardepth_carrot(
        line,
        1.0 * self._carrot_show_path_width,
        self._path_offset_z + 0.15,
        self._path_offset_z - 0.35,
        max_idx,
        allow_invert=False,
      )
    else:  # mode == 15
      track_vertices = self._build_path_polygon_dist_carrot(
        line,
        1.0 * self._carrot_show_path_width,
        self._path_offset_z + 0.20,
        self._path_offset_z - 0.50,
        max_distance,
        allow_invert=False,
      )

    if track_vertices.size == 0:
      return

    draw_polygon(self._rect, track_vertices, fill_color)
    if stroke_width > 0.0:
      self._draw_polygon_outline_carrot(track_vertices, stroke_color, stroke_width)


  def _draw_complex_path_carrot(self, sm, mode: int, color_idx: int, brake_valid: bool):
    line = self._path.raw_points
    if line.shape[0] == 0:
      return

    max_distance = np.clip(line[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    fill_color = self._get_path_color_carrot(color_idx)
    stroke_color = rl.Color(255, 0, 0, 255) if brake_valid else rl.Color(255, 255, 255, 255)
    stroke_width = 2.0 if (color_idx >= 10 or brake_valid) else 0.0

    if mode == 9:
      track_vertices = self._build_path_polygon_dist_carrot(
        line, 1.00 * self._carrot_show_path_width,
        self._path_offset_z, self._path_offset_z,
        max_distance, allow_invert=False
      )
    elif mode == 10:
      track_vertices = self._build_path_polygon_dist_carrot(
        line, 0.85 * self._carrot_show_path_width,
        self._path_offset_z + 0.10, self._path_offset_z - 0.15,
        max_distance, allow_invert=False
      )
    elif mode == 11:
      track_vertices = self._build_path_polygon_dist_carrot(
        line, 1.15 * self._carrot_show_path_width,
        self._path_offset_z + 0.20, self._path_offset_z - 0.30,
        max_distance, allow_invert=False
      )
    else:
      track_vertices = self._build_path_polygon_dist_carrot(
        line, 1.00 * self._carrot_show_path_width,
        self._path_offset_z + 0.25, self._path_offset_z - 0.45,
        max_distance, allow_invert=False
      )

    if track_vertices.size == 0:
      return

    draw_polygon(self._rect, track_vertices, fill_color)
    if stroke_width > 0.0:
      self._draw_polygon_outline_carrot(track_vertices, stroke_color, stroke_width)


  def _draw_animated_path_carrot(self, sm, mode: int, color_idx: int, brake_valid: bool):
    line = self._path.raw_points
    if line.shape[0] == 0:
      return

    max_distance = np.clip(line[-1, 0], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    fill_color = self._get_path_color_carrot(color_idx)
    stroke_color = rl.Color(255, 0, 0, 255) if brake_valid else rl.Color(255, 255, 255, 255)
    stroke_width = 2.0 if (color_idx >= 10 or brake_valid) else 0.0

    phase = (math.sin(self._carrot_path_draw_seq) + 1.0) * 0.5

    if mode == 1:
      width_apply = (0.65 + 0.35 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z
      z1 = self._path_offset_z
    elif mode == 2:
      width_apply = (0.55 + 0.55 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.10
      z1 = self._path_offset_z - 0.15
    elif mode == 3:
      width_apply = (0.70 + 0.40 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.20
      z1 = self._path_offset_z - 0.25
    elif mode == 4:
      width_apply = (0.80 + 0.45 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.25
      z1 = self._path_offset_z - 0.35
    elif mode == 5:
      width_apply = (0.90 + 0.25 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.05
      z1 = self._path_offset_z - 0.25
    elif mode == 6:
      width_apply = (1.00 + 0.20 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.15
      z1 = self._path_offset_z - 0.40
    elif mode == 7:
      width_apply = (0.75 + 0.50 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.30
      z1 = self._path_offset_z - 0.45
    else:
      width_apply = (0.65 + 0.35 * phase) * self._carrot_show_path_width
      z0 = self._path_offset_z + 0.10
      z1 = self._path_offset_z - 0.20

    track_vertices = self._build_path_polygon_dist_carrot(
      line,
      width_apply,
      z0,
      z1,
      max_distance,
      allow_invert=False,
    )

    if track_vertices.size == 0:
      return

    draw_polygon(self._rect, track_vertices, fill_color)
    if stroke_width > 0.0:
      self._draw_polygon_outline_carrot(track_vertices, stroke_color, stroke_width)
      
  def _draw_radar_info_carrot(self, sm):
    if self._carrot_show_radar_info <= 0:
      return

    if not sm.valid['radarState']:
      return

    radar_state = sm['radarState']
    lane_lines = sm['modelV2'].laneLines
    if len(lane_lines) < 3:
      return

    lane_line = lane_lines[2]

    for rs in (radar_state.leadsLeft, radar_state.leadsRight, radar_state.leadsCenter):
      for lead in rs:
        d_rel = lead.dRel
        if d_rel <= 2.5:
          continue

        idx = self._get_path_length_idx(np.array(lane_line.x, dtype=np.float32), d_rel)
        if idx >= len(lane_line.z):
          continue

        z = lane_line.z[idx] - 0.61
        side = self._map_to_screen(d_rel, -lead.yRel, z)
        if side is None:
          continue

        x = side[0]
        y = side[1]
        v = lead.vLeadK
        v_lat = lead.vLat
        y_rel = lead.yRel
        radar = lead.radar
        model_prob = lead.modelProb

        v_abs = math.sqrt(v * v + v_lat * v_lat)
        v_sum = v_abs if v >= 0.0 else -v_abs

        if v_abs > 3.0:
          t = self._carrot_radar_lat_factor
          a_d_rel = d_rel + v * t
          if a_d_rel < 2.0:
            a_d_rel = 2.0
          a_y_rel = y_rel + v_lat * t

          a_side = None
          if abs(v) > 3.0:
            a_side = self._map_to_screen(a_d_rel, -a_y_rel, z)

          line_color = rl.Color(0, 203, 0, 255) if v_sum > 0.0 else rl.Color(255, 0, 0, 255)

          if a_side is not None:
            self._draw_line_segment_carrot(side, a_side, line_color, 3.0)
            rl.draw_circle(int(a_side[0]), int(a_side[1]), 10.0, line_color)

          speed_text = f"{(v_sum * 3.6) if ui_state.scene.is_metric else (v_sum * 2.2369363):.0f}"

          if not radar:
            box_color = rl.Color(0, 0, 255, 255)
          elif model_prob == 0.01:
            box_color = rl.Color(0, 203, 0, 255)
          elif v_sum > 0.0:
            box_color = rl.Color(255, 175, 3, 255)
          else:
            box_color = rl.Color(255, 0, 0, 255)

          self._draw_text_box_carrot(int(x), int(y), speed_text, 40, box_color)

          if self._carrot_show_radar_info >= 2:
            self._draw_text_carrot(int(x), int(y - 40), f"{y_rel:.1f}", 30, rl.Color(255, 255, 255, 255))
            dist_text = f"{d_rel:.1f}" if ui_state.scene.is_metric else f"{(d_rel * 0.000621371):.1f}"
            self._draw_text_carrot(int(x), int(y + 30), dist_text, 30, rl.Color(255, 255, 255, 255))

        elif self._carrot_show_radar_info >= 3:
          self._draw_text_carrot(int(x), int(y), "*", 40, rl.Color(255, 255, 255, 255))


  def _get_path_color_carrot(self, show_path_color: int) -> rl.Color:
    color_id = show_path_color % 10

    if color_id == 0:
      return rl.Color(255, 0, 0, 120)
    if color_id == 1:
      return rl.Color(255, 153, 0, 120)
    if color_id == 2:
      return rl.Color(218, 202, 37, 120)
    if color_id == 3:
      return rl.Color(0, 203, 0, 120)
    if color_id == 4:
      return rl.Color(0, 0, 255, 120)
    if color_id == 5:
      return rl.Color(0, 0, 128, 120)
    if color_id == 6:
      return rl.Color(139, 0, 255, 120)
    if color_id == 7:
      return rl.Color(218, 111, 37, 120)
    if color_id == 8:
      return rl.Color(255, 255, 255, 120)
    return rl.Color(0, 0, 0, 120)


  def _draw_polygon_outline_carrot(self, points: np.ndarray, color: rl.Color, thickness: float):
    if points.shape[0] < 2:
      return

    for i in range(points.shape[0] - 1):
      p0 = rl.Vector2(float(points[i][0]), float(points[i][1]))
      p1 = rl.Vector2(float(points[i + 1][0]), float(points[i + 1][1]))
      rl.draw_line_ex(p0, p1, thickness, color)

    p0 = rl.Vector2(float(points[-1][0]), float(points[-1][1]))
    p1 = rl.Vector2(float(points[0][0]), float(points[0][1]))
    rl.draw_line_ex(p0, p1, thickness, color)


  def _draw_line_segment_carrot(self, p0, p1, color: rl.Color, thickness: float):
    v0 = rl.Vector2(float(p0[0]), float(p0[1]))
    v1 = rl.Vector2(float(p1[0]), float(p1[1]))
    rl.draw_line_ex(v0, v1, thickness, color)


  def _draw_rect_fill_outline_carrot(self, x: float, y: float, w: float, h: float, fill_color: rl.Color, stroke_color: rl.Color, stroke_width: float):
    rl.draw_rectangle_rounded(
      rl.Rectangle(float(x), float(y), float(w), float(h)),
      0.15,
      12,
      fill_color,
    )
    rl.draw_rectangle_rounded_lines_ex(
      rl.Rectangle(float(x), float(y), float(w), float(h)),
      0.15,
      12,
      stroke_width,
      stroke_color,
    )


  def _draw_text_box_carrot(self, x: int, y: int, text: str, font_size: int, box_color: rl.Color):
    w = max(40, int(len(text) * font_size * 0.9))
    h = 42
    self._draw_rect_fill_outline_carrot(
      x - w / 2,
      y - 35,
      w,
      h,
      box_color,
      box_color,
      0.0,
    )
    self._draw_text_carrot(x, y, text, font_size, rl.Color(255, 255, 255, 255))


  def _draw_text_carrot(self, x: int, y: int, text: str, font_size: int, color: rl.Color):
    rl.draw_text(text, int(x - len(text) * font_size * 0.25), int(y - font_size * 0.5), font_size, color)
