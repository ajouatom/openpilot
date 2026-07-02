import math
from collections import deque
from typing import List, Tuple

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.selfdrive.ui.ui_state import ui_state


PLOT_MAX = 300


def _safe_get(obj, path: str, default=0.0):
  """Safely get nested attributes, e.g. path='a.b.c'."""
  cur = obj
  for key in path.split("."):
    if cur is None:
      return default
    try:
      cur = getattr(cur, key)
    except Exception:
      return default
  return cur if cur is not None else default


def _safe_arr_get(arr, idx: int, default=0.0):
  try:
    return arr[idx]
  except Exception:
    return default


class _SlidingMinMax:
  """Monotonic queue min/max over time-indexed samples."""
  def __init__(self):
    self._minq = deque()  # (t, v) increasing v
    self._maxq = deque()  # (t, v) decreasing v

  def push(self, t: int, v: float):
    while self._minq and self._minq[-1][1] > v:
      self._minq.pop()
    self._minq.append((t, v))

    while self._maxq and self._maxq[-1][1] < v:
      self._maxq.pop()
    self._maxq.append((t, v))

  def expire_older_than(self, t_oldest: int):
    while self._minq and self._minq[0][0] < t_oldest:
      self._minq.popleft()
    while self._maxq and self._maxq[0][0] < t_oldest:
      self._maxq.popleft()

  def current_min(self) -> float:
    return self._minq[0][1] if self._minq else 0.0

  def current_max(self) -> float:
    return self._maxq[0][1] if self._maxq else 0.0


class DebugPlot(Widget):
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))

    self.params = Params()

    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    # plot state
    self.plot_size = 0
    self.plot_index = -1
    self.sample_t = -1

    self.plot_queue: List[List[float]] = [[0.0] * PLOT_MAX for _ in range(3)]
    self.mm: List[_SlidingMinMax] = [_SlidingMinMax(), _SlidingMinMax(), _SlidingMinMax()]

    self.plot_min = 0.0
    self.plot_max = 0.0

    # layout (will be overridden responsively in _render)
    self.plot_x = 350.0
    self.plot_y = 40.0
    self.plot_height = 300.0
    self.plot_dx = 2.0

    self.show_plot_mode_prev = -1

    # === 20Hz sampling ===
    self.sample_hz = 20.0
    self.sample_dt = 1.0 / self.sample_hz
    self._last_sample_time = None  # type: float | None

  def show_event(self):
    super().show_event()

  def hide_event(self):
    super().hide_event()

  def _reset_plot(self):
    self.plot_size = 0
    self.plot_index = -1
    self.sample_t = -1
    self.plot_min = 0.0
    self.plot_max = 0.0
    for i in range(3):
      self.plot_queue[i] = [0.0] * PLOT_MAX
      self.mm[i] = _SlidingMinMax()

    # reset sampling clock too
    self._last_sample_time = None

  def _make_plot_data(self, sm, show_plot_mode: int):
    data = [0.0, 0.0, 0.0]
    title = "no data"

    required = ["carState", "longitudinalPlan", "carControl", "controlsState", "modelV2", "radarState", "liveParameters"]
    if not all(sm.alive[ch] for ch in required):
      return data, title

    cs = sm["carState"]
    lp = sm["longitudinalPlan"]
    cc = sm["carControl"]
    controls_state = sm["controlsState"]
    model = sm["modelV2"]
    radar = sm["radarState"]
    live_params = sm["liveParameters"]

    a_ego = cs.aEgo
    v_ego = cs.vEgo
    accel = lp.accels[0]
    speed_0 = lp.speeds[0]
    accel_out = cc.actuators.accel

    torque_state = None
    if controls_state is not None:
      lat_state = _safe_get(controls_state, "lateralControlState", None)
      torque_state = _safe_get(lat_state, "torqueState", None) if lat_state is not None else None

    # 0: yellow, 1: green, 2: orange
    if show_plot_mode in (0, 1):
      data[0] = a_ego
      data[1] = accel
      data[2] = accel_out
      title = "1.Accel (Y:a_ego, G:a_target, O:a_out)"

    elif show_plot_mode == 2:
      data[0] = speed_0
      data[1] = v_ego
      data[2] = a_ego
      title = "2.Speed/Accel (Y:speed_0, G:v_ego, O:a_ego)"

    elif show_plot_mode == 3 and model is not None:
      pos = _safe_get(model, "position", None)
      vel = _safe_get(model, "velocity", None)
      pos_x = _safe_get(pos, "x", []) if pos is not None else []
      vel_x = _safe_get(vel, "x", []) if vel is not None else []
      data[0] = float(_safe_arr_get(pos_x, 32, 0.0))
      data[1] = float(_safe_arr_get(vel_x, 32, 0.0))
      data[2] = float(_safe_arr_get(vel_x, 0, 0.0))
      title = "3.Model (Y:pos_32, G:vel_32, O:vel_0)"

    elif show_plot_mode == 4 and radar is not None:
      data[0] = accel
      data[1] = float(_safe_get(radar, "aLeadK", 0.0))
      data[2] = float(_safe_get(radar, "vRel", 0.0))
      title = "4.Lead (Y:accel, G:a_leadK, O:v_rel)"

    elif show_plot_mode == 5 and radar is not None:
      data[0] = a_ego
      data[1] = float(_safe_get(radar, "aLead", 0.0))
      data[2] = float(_safe_get(radar, "jLead", 0.0))
      title = "5.Lead (Y:a_ego, G:a_lead, O:j_lead)"

    elif show_plot_mode == 6 and torque_state is not None:
      data[0] = float(_safe_get(torque_state, "actualLateralAccel", 0.0)) * 10.0
      data[1] = float(_safe_get(torque_state, "desiredLateralAccel", 0.0)) * 10.0
      data[2] = float(_safe_get(torque_state, "output", 0.0)) * 10.0
      title = "6.Steer (Y:actual, G:desired, O:output) *10"

    elif show_plot_mode == 7 and cc is not None:
      data[0] = float(_safe_get(cs, "steeringAngleDeg", 0.0))
      data[1] = float(_safe_get(_safe_get(cc, "actuators", cc), "steeringAngleDeg", 0.0))
      data[2] = float(_safe_get(live_params, "angleOffsetDeg", 0.0)) * 10.0
      title = "7.SteerA (Y:Actual, G:Target, O:Offset*10)"

    elif show_plot_mode == 8 and cc is not None:
      curv = float(_safe_get(_safe_get(cc, "actuators", cc), "curvature", 0.0)) * 10000.0
      data[0] = curv
      data[1] = curv
      data[2] = curv
      title = "8.Curvature (x10000)"

    return data, title

  def _update_plot_queue(self, plot_data: List[float]):
    self.sample_t += 1
    self.plot_index = (self.plot_index + 1) % PLOT_MAX

    if self.plot_size < PLOT_MAX:
      self.plot_size += 1
    oldest_t = self.sample_t - (self.plot_size - 1)

    for i in range(3):
      val = float(plot_data[i])
      self.plot_queue[i][self.plot_index] = val
      self.mm[i].push(self.sample_t, val)
      self.mm[i].expire_older_than(oldest_t)

    mn = min(self.mm[i].current_min() for i in range(3))
    mx = max(self.mm[i].current_max() for i in range(3))

    if mn > -2.0:
      mn = -2.0
    if mx < 2.0:
      mx = 2.0

    self.plot_min = mn
    self.plot_max = mx

  def _get_series_value(self, series_idx: int, k_back: int) -> float:
    """k_back=0 -> newest, k_back=plot_size-1 -> oldest."""
    idx = (self.plot_index - k_back) % PLOT_MAX
    return self.plot_queue[series_idx][idx]

  def _draw_series(self, rect: rl.Rectangle, series_idx: int, color: rl.Color, stroke: int = 3):
    if self.plot_size < 2:
      return

    pr = self.plot_max - self.plot_min
    ratio = self.plot_height if pr < 1e-6 else (self.plot_height / pr)

    prev_x = None
    prev_y = None

    for i in range(self.plot_size):
      k_back = (self.plot_size - 1) - i  # oldest -> newest
      val = self._get_series_value(series_idx, k_back)
      x = self.plot_x + i * self.plot_dx
      y = self.plot_y + self.plot_height - (val - self.plot_min) * ratio

      if prev_x is not None:
        if stroke <= 1:
          rl.draw_line(int(prev_x), int(prev_y), int(x), int(y), color)
        else:
          for o in range(-(stroke // 2), stroke // 2 + 1):
            rl.draw_line(int(prev_x), int(prev_y) + o, int(x), int(y) + o, color)
      prev_x, prev_y = x, y

    last_val = self._get_series_value(series_idx, 0)
    label = f"{last_val:.2f}"

    font_size = 30
    pad = 6

    x = int(prev_x + 12)
    y = int(prev_y + (30 if series_idx > 0 else 0))

    left = int(rect.x) + pad
    right = int(rect.x + rect.width) - pad
    top = int(rect.y) + pad
    bottom = int(rect.y + rect.height) - pad

    text_w = int(rl.measure_text(label, font_size))
    text_h = font_size

    if x + text_w > right:
      x = max(left, right - text_w)

    if x < left:
      x = left

    if y + text_h > bottom:
      y = max(top, bottom - text_h)
    if y < top:
      y = top

    rl.draw_text(label, x, y, font_size, color)

  def _render(self, rect: rl.Rectangle):
    show_plot_mode = int(self.params.get_int("ShowPlotMode"))
    if show_plot_mode == 0:
      return

    sm = ui_state.sm
    if not (sm.alive.get("carState", False) and sm.alive.get("longitudinalPlan", False)):
      return

    # reset when mode changes
    if show_plot_mode != self.show_plot_mode_prev:
      self._reset_plot()
      self.show_plot_mode_prev = show_plot_mode

    # === full-area layout (use entire rect) ===
    rx = float(rect.x)
    ry = float(rect.y)
    W = int(rect.width)
    H = int(rect.height)

    title_h = 46 

    plot_w = max(1, W)
    self.plot_x = rx + 0.0
    self.plot_y = ry + float(title_h)
    self.plot_height = float(max(60, H - title_h))  

    self.plot_dx = float(max(1.0, plot_w / max(1, (PLOT_MAX - 1))))

    # background (transparent)
    rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, 0))

    # === 20Hz gate ===
    now = float(rl.get_time())
    if self._last_sample_time is None:
      do_sample = True
    else:
      do_sample = (now - self._last_sample_time) >= self.sample_dt

    if do_sample:
      if self._last_sample_time is None:
        self._last_sample_time = now
      else:
        steps = int((now - self._last_sample_time) / self.sample_dt)
        steps = max(1, min(steps, 5))
        self._last_sample_time += steps * self.sample_dt

      plot_data, title = self._make_plot_data(sm, show_plot_mode)
      self._update_plot_queue(plot_data)
    else:
      plot_data, title = self._make_plot_data(sm, show_plot_mode)

    # draw grid (only within rect)
    grid_color = rl.Color(60, 60, 60, 120)
    step = 100
    x0 = int(rx)
    y0 = int(ry)
    x1 = int(rx + rect.width)
    y1 = int(ry + rect.height)

    # vertical grid
    gx = x0 - (x0 % step)
    while gx <= x1:
      rl.draw_line(gx, y0, gx, y1, grid_color)
      gx += step

    # horizontal grid
    gy = y0 - (y0 % step)
    while gy <= y1:
      rl.draw_line(x0, gy, x1, gy, grid_color)
      gy += step

    rl.draw_rectangle(x0, y0, x1 - x0, y1 - y0, rl.Color(0, 0, 0, 70))

    rl.draw_text_ex(
      self._font_display,
      title,
      rl.Vector2(x0 + 10, y0 + 8),
      26,
      0,
      rl.WHITE
    )

    rl.draw_text(
      f"min={self.plot_min:.2f}  max={self.plot_max:.2f}",
      x0 + 10,
      y0 + 30,
      22,
      rl.Color(200, 200, 200, 255),
    )

    # series colors: yellow, green, orange
    colors = [
      rl.Color(255, 220, 0, 255),
      rl.Color(0, 255, 0, 255),
      rl.Color(255, 165, 0, 255),
    ]

    for i in range(3):
      self._draw_series(rect, i, colors[i], stroke=3)