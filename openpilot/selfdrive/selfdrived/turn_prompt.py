from enum import Enum

# log.Desire turnLeft / turnRight: the turn model is driving the car.
TURN_DESIRES = (1, 2)

TURN_ENTER_S = 0.2    # a one-frame desire flicker must not say "Turn"
TURN_RELEASE_S = 0.5  # short gaps inside one turn must not say "Straight", "Turn"


class TurnPrompt(Enum):
  TURN = "turn"
  STRAIGHT = "straight"


class TurnPromptTracker:
  """Say "Turn" once when the turn desire has held for TURN_ENTER_S and "Straight" once when it has been gone
  for TURN_RELEASE_S. Only while lateral control is active; losing it ends the turn silently."""

  def __init__(self):
    self.announced = False
    self.on_since: float | None = None
    self.off_since: float | None = None

  def update(self, turning: bool, lat_active: bool, now: float) -> TurnPrompt | None:
    if not lat_active:
      self.__init__()
      return None

    if turning:
      self.off_since = None
      if self.on_since is None:
        self.on_since = now
      if not self.announced and now - self.on_since >= TURN_ENTER_S:
        self.announced = True
        return TurnPrompt.TURN
      return None

    self.on_since = None
    if not self.announced:
      return None
    if self.off_since is None:
      self.off_since = now
    if now - self.off_since >= TURN_RELEASE_S:
      self.announced = False
      self.off_since = None
      return TurnPrompt.STRAIGHT
    return None
