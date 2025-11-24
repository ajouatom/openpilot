import numpy as np
from collections import deque

class FirstOrderFilter:
  # first order filter
  def __init__(self, x0, rc, dt, initialized=True):
    self.x = x0
    self.dt = dt
    self.update_alpha(rc)
    self.initialized = initialized

  def update_alpha(self, rc):
    self.alpha = self.dt / (rc + self.dt)

  def update(self, x):
    if self.initialized:
      self.x = (1. - self.alpha) * self.x + self.alpha * x
    else:
      self.initialized = True
      self.x = x
    return self.x


class BounceFilter(FirstOrderFilter):
  def __init__(self, x0, rc, dt, initialized=True, bounce=2):
    self.velocity = FirstOrderFilter(0.0, 0.15, dt)
    self.bounce = bounce
    super().__init__(x0, rc, dt, initialized)

  def update(self, x):
    super().update(x)
    scale = self.dt / (1.0 / 60.0)  # tuned at 60 fps
    self.velocity.x += (x - self.x) * self.bounce * scale * self.dt
    self.velocity.update(0.0)
    if abs(self.velocity.x) < 1e-5:
      self.velocity.x = 0.0
    self.x += self.velocity.x
    return self.x

class MyMovingAverage:
  def __init__(self, window_size, value=None):
    self.window_size = window_size
    if (value is not None):
      self.values = deque([value] * window_size, maxlen=window_size)
      self.sum = value * window_size
      self.result = value
    else:
      self.values = deque(maxlen=window_size)
      self.sum = 0
      self.result = 0

  def set(self, value):
    self.values.clear()
    self.values.append(value)
    self.sum = value
    self.result = value
    return value

  def set_all(self, value):
    self.values = deque([value] * self.window_size, maxlen=self.window_size)
    self.sum = value * self.window_size
    self.result = value
    return value

  def process(self, value, median=False):
    self.values.append(value)
    self.sum = sum(self.values)  
    self.result = float(np.median(self.values)) if median else float(self.sum) / len(self.values)
    return self.result
