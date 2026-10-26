class PalisadeCgwTest:
  """Parked, one-shot CGW1 status experiment; lamp actuation is unverified."""

  def __init__(self):
    self.armed = False
    self.direction = 0
    self.started = 0
    self.last_tx = 0

  def update(self, selection, now, raw, timestamps, parked, inactive):
    if selection == 0:
      self.armed = True
      self.direction = 0
      return []

    if self.armed:
      self.armed = False
      self.direction = selection if selection in (1, 2) else 0
      self.started = now
      self.last_tx = 0

    fresh = len(timestamps) == 3 and all(0 < ts <= now and now - ts <= 250_000_000 for ts in timestamps)
    # Stop for stock lamps, hazard or either physical turn switch.
    neutral = len(raw) == 8 and not (raw[2] & 0x18 or raw[7] & 0xC0 or raw[4] & 0x06 or raw[5] & 0x06)
    if (selection != self.direction or not fresh or not neutral or not parked or not inactive
        or not 0 <= now - self.started < 3_000_000_000):
      self.direction = 0
    if not self.direction or self.last_tx and now - self.last_tx < 50_000_000:
      return []

    data = bytearray(raw)
    data[2] |= 0x08 if self.direction == 1 else 0
    data[7] |= 0x40 if self.direction == 2 else 0
    self.last_tx = now
    return [(0x541, bytes(data), 0)]
