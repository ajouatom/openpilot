from dataclasses import dataclass
from .constants import BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT

@dataclass
class BlinkerOutput:
  driver_blinker_state: int
  driver_blinker_changed: bool
  driver_desire_enabled: bool

  atc_blinker_state: int
  atc_desire_enabled: bool

  blinker_state: int
  desire_enabled: bool


class BlinkerManager:
  def __init__(self):
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0

    self.atc_type = ""
    self.atc_active = 0  # 0: 없음, 1: ATC 동작, 2: 충돌
    self.blinker_ignore = False

  def tick(self):
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)

  def update_driver(self, carstate, laneChangeNeedTorque: int):
    st = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    changed = st != self.driver_blinker_state
    self.driver_blinker_state = st

    enabled = st in (BLINKER_LEFT, BLINKER_RIGHT)
    if laneChangeNeedTorque < 0:
      enabled = False
    return st, changed, enabled

  def update_atc(self, carrotMan, driver_blinker_state: int):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state

    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / carrotMan.DT_MDL) if hasattr(carrotMan, "DT_MDL") else 0
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state

    elif atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False

    elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active = 1

    else:
      self.atc_active = 0

    # 운전자와 충돌하면 ATC 무효
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)

    # ignore 처리
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 타입 바뀌면 1프레임 무시(안정화)
    if self.atc_type != atc_type:
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled, atc_type

  def run(self, carstate, carrotMan, laneChangeNeedTorque: int):
    self.tick()

    driver_st, driver_changed, driver_enabled = self.update_driver(carstate, laneChangeNeedTorque)
    atc_st, atc_enabled, _ = self.update_atc(carrotMan, driver_st)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    return BlinkerOutput(
      driver_blinker_state=driver_st,
      driver_blinker_changed=driver_changed,
      driver_desire_enabled=driver_enabled,
      atc_blinker_state=atc_st,
      atc_desire_enabled=atc_enabled,
      blinker_state=blinker_state,
      desire_enabled=desire_enabled,
    )
