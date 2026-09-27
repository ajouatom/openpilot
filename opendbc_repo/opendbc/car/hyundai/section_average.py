"""Section-camera average-speed allowance for stock-navigation CAN (joongyu01/speedcam).

A section stays capped at its limit (the existing behaviour) until the driver
unlocks it with one cruise + press or gas tap. Once unlocked, time banked below
the limit (e.g. in congestion) may be spent up to the cruise set speed:
B = elapsed_time - distance / limit, and B >= 0 keeps the running average at or
below the limit whatever the remaining length is. The allowance spends B over a
fixed horizon and falls back to the limit cap as B runs out. A long press of
cruise - locks the section again; leaving the section resets it.
"""

SECTION_END_DEBOUNCE_S = 3.0
SPEND_HORIZON_M = 1000.0
MIN_BANK_S = 0.5
MAX_ALLOWANCE_KPH = 250.0


class SectionAverage:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.unlocked = False
    self.limit_kph = 0.0
    self.start_time = 0.0
    self.start_distance = 0.0
    self.inactive_since = None

  def update(self, section_active, limit_kph, now, total_distance):
    if section_active and limit_kph > 0:
      self.inactive_since = None
      if not self.active or limit_kph != self.limit_kph:
        self.reset()
        self.active = True
        self.limit_kph = limit_kph
        self.start_time = now
        self.start_distance = total_distance
    elif self.active:
      # 0x4B4 can drop for a frame; a real section end or exit lasts longer.
      if self.inactive_since is None:
        self.inactive_since = now
      elif now - self.inactive_since >= SECTION_END_DEBOUNCE_S:
        self.reset()

  def unlock(self):
    if self.active:
      self.unlocked = True

  def lock(self):
    self.unlocked = False

  def _limit_ms(self):
    return max(1.0, self.limit_kph / 3.6)

  def bank_seconds(self, now, total_distance):
    if not self.active:
      return 0.0
    return (now - self.start_time) - (total_distance - self.start_distance) / self._limit_ms()

  def allowance_kph(self, now, total_distance):
    """Speed (km/h, actual) that spends the bank over SPEND_HORIZON_M; 0 when locked or without bank."""
    if not self.unlocked:
      return 0.0
    bank = self.bank_seconds(now, total_distance)
    if bank <= MIN_BANK_S:
      return 0.0
    remaining_time = SPEND_HORIZON_M / self._limit_ms() - bank
    if remaining_time <= SPEND_HORIZON_M / (MAX_ALLOWANCE_KPH / 3.6):
      return MAX_ALLOWANCE_KPH
    return min(MAX_ALLOWANCE_KPH, SPEND_HORIZON_M / remaining_time * 3.6)
