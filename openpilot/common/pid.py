import numpy as np
from numbers import Number

class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  @property
  def error_integral(self):
    return self.i/self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_rate

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = np.clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f

    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control


class MultiplicativeUnwindPID:
  # infiniteCable2 포팅: LatControlCurvature(VW MEB 곡률 폐루프)에서 사용.
  # override(운전자 개입) 시 적분항을 곱셈적으로 풀어주는(unwind) PID.
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100, min_cmd=1e-10, ki_red_time=1.0):
    if isinstance(k_p, Number):
      k_p = [[0], [k_p]]
    if isinstance(k_i, Number):
      k_i = [[0], [k_i]]
    if isinstance(k_d, Number):
      k_d = [[0], [k_d]]
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = float(k_f)
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit
    self.rate = float(rate)
    self.i_dt = 1.0 / rate
    self.min_cmd = abs(min_cmd)
    self.ki_red_time = float(ki_red_time)
    self.override_prev = False
    self.i_unwind_factor = 1.0
    self.speed = 0.0
    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def _calc_unwind_factor(self, override):
    if not override or self.override_prev:
      return
    if self.ki_red_time <= 0.0:
      self.i_unwind_factor = 1.0
      return
    if abs(self.i) <= self.min_cmd:
      self.i_unwind_factor = 0.0
      return
    steps = max(int(self.ki_red_time * self.rate), 1)
    factor = (self.min_cmd / abs(self.i)) ** (1.0 / steps)
    self.i_unwind_factor = min(factor, 1.0)

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self._calc_unwind_factor(override)
      self.i *= self.i_unwind_factor
      if abs(self.i) < self.min_cmd:
        self.i = 0.0
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_dt

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = np.clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f

    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    self.override_prev = override
    return self.control
