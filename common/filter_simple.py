import numpy as np

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

class StreamingMovingAverage:
  def __init__(self, window_size):
    self.window_size = window_size
    self.values = []
    self.sum = 0
    self.result = 0

  def set(self, value):
    #for i in range(len(self.values)):
    #  self.values[i] = value
    #self.sum = value * len(self.values)
    self.values.clear()
    self.values.append(value)
    self.sum = value
    self.result = value
    return value

  def process(self, value, median = False):
    self.values.append(value)
    self.sum += value
    if len(self.values) > self.window_size:
      self.sum -= self.values.pop(0)
    self.result = float(np.median(self.values)) if median else float(self.sum) / len(self.values)
    return self.result
