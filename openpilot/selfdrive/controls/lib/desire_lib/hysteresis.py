from openpilot.common.realtime import DT_MDL

class ExistCounter:
  def __init__(self, sustain_sec: float = 0.2):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(sustain_sec / DT_MDL)

  def update(self, exist_flag: bool):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0
      if self.true_count >= self.threshold:
        self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0
      if self.false_count >= self.threshold:
        self.counter = min(self.counter - 1, -1)
    return self.counter
