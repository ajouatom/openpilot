# PFEIFER - AOL

# Acknowledgements:
# Huge thanks to the following forks, much of my initial implementation used these as a reference:
#   * Sunnypilot - https://github.com/sunnyhaibin/sunnypilot
#   * Ghostpilot - https://github.com/spektor56/ghostpilot
#   * Alexander Sato's fork - https://github.com/AlexandreSato/openpilot/tree/personal3
#
# Another huge thanks goes out to Frogai. Working with him on Toyota support lead to a pretty generic method to support
# nearly every brand. https://github.com/frogAi/FrogPilot
#
# A huge thanks also goes out to nworby for his continued attention to detail and diligence that has lead to many
# improvements to the safety of this implementation

import time

from cereal import log, car
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED
from panda import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.controls.lib.events import EngagementAlert, AudibleAlert
from openpilot.selfdrive.controls.lib.events import ET
from openpilot.common.realtime import DT_CTRL

State = log.ControlsState.OpenpilotState
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)

params = Params()
mem_params = Params("/dev/shm/params")
"""
A speed optimization, Params with a base path in /dev/shm/params.

/dev/shm is a memory mapped folder and does not persistently store across
reboots. The advantage to using a memory mapped folder is that it should be
very fast and consistent for both writes and reads.

If using the real filesystem placing data can, in extreme circumstances,
take over 1 second. put_bool_nonblocking helps with this by creating a
thread but this can create race conditions. If we need to block but don't
care about persistence across reboots the memory mapped location should
avoid random lag.
"""

SOFT_DISABLE_TIME = 3  # seconds

class AlwaysOnLateral:
  def __init__(self, controls):
    self.CI = controls.CI
    self.AM = controls.AM
    self.sm = controls.sm
    self.controls = controls

    # UI Toggles
    self.disengage_lat_on_brake: bool = False
    self.disengage_lat_on_blinker: bool = False

    # Car State
    self.braking: bool = False
    self.blinkers_active: bool = False
    self.standstill: bool = False
    self.steer_fault: bool = False
    self.invalid_gear: bool = False
    self.cruise_available: bool = False
    self.cruise_enabled: bool = False
    self.blinker_low_speed = False

    # OP/AOL states
    self.cruise_previously_engaged: bool = False
    self.last_lat_allowed: bool = False
    self.op_active: bool = False
    self.prev_lat_active: bool = False
    self.calibrated: bool = False
    self.start_time = time.time() # seconds

    # This is used to disable aol in the event that openpilot has received a
    # disable event. aol will only be re-enabled once openpilot has been
    # re-engaged.
    self.disabled: bool = False
    self.soft_disabling: bool = False
    self.soft_disable_timer: float = 0.0

    params.put_bool("AlwaysOnLateralEnabledLock", True) # locks toggle when onroad
    # sets the default blinker low speed limit to 25 mph if not already set
    AlwaysOnLateral.set_default_lat_blinker_low_speed_limit(11.176)

    # we only need to check these once since we are locking these toggles while onroad
    self.enabled = params.get_bool('AlwaysOnLateralEnabled')
    self.main_enables = params.get_bool('AlwaysOnLateralMainEnables')
    self.disengage_lat_on_brake = params.get_bool('DisengageLatOnBrake')
    self.disengage_lat_on_blinker = params.get_bool('DisengageLatOnBlinker')
    self.disengage_lat_on_low_speed_blinker = params.get_bool('DisengageLatOnLowSpeedBlinker')
    self.blinker_low_speed_limit = float(params.get('LatBlinkerLowSpeedLimit').decode()) # m/s

  @staticmethod
  def set_default_lat_blinker_low_speed_limit(limit: float) -> None:
    """
    sets a default low speed blinker limit
    """
    try:
      float(params.get("LatBlinkerLowSpeedLimit").decode())
    except:
      params.put("LatBlinkerLowSpeedLimit", str(limit))

  def handle_audible_alert(self, lateral_allowed: bool):
    """
    Plays the engagement/disengagement sound when lateral allowed is toggled
    """
    if self.last_lat_allowed != lateral_allowed:
      alert = None
      if lateral_allowed:
        alert = EngagementAlert(AudibleAlert.engage)
      else:
        alert = EngagementAlert(AudibleAlert.disengage)
      self.AM.add_many(self.sm.frame, [alert])

  def update_cruise_previously_engaged(self):
    """
    This checks if openpilot has been fully engaged at least once after cruise was turned on. This is tracked because
    some cars will give cruise faults if we send lateral commands immediately upon turning cruise main on.
    """
    if not self.cruise_available:
      self.cruise_previously_engaged = False
    self.cruise_previously_engaged |= self.op_active

  def check_aol_state(self) -> bool:
    """
    Checks if we meet the panda lateral allowed requirements
    """
    if not self.enabled:
      return False

    if self.disabled:
      return False

    # When using cruise state to check if aol is enabled we check if cruise is available (on).
    # If we do not enable on main then we check if openpilot has previously been fully engaged.
    if self.main_enables:
      return self.cruise_available
    return self.cruise_previously_engaged and self.cruise_available

  def update(self, car_state, op_state, car_params):
    panda_states = self.sm['pandaStates']
    self.op_active = op_state in ACTIVE_STATES

    self.cruise_available = car_state.cruiseState.available
    self.cruise_enabled = car_state.cruiseState.enabled

    lateral_allowed = self.check_aol_state()
    self.handle_audible_alert(lateral_allowed)

    # Clear disabled state if OP is enabled or we have toggled aol off
    # This allows rengagement by either engaging op or toggling aol off and back on
    if op_state == State.enabled or not lateral_allowed:
      self.disabled = False

    self.update_cruise_previously_engaged()

    self.braking = car_state.brakePressed or car_state.regenBraking
    self.blinkers_active = car_state.leftBlinker or car_state.rightBlinker
    self.standstill = car_state.vEgo <= max(car_params.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or car_state.standstill
    self.blinkers_low_speed = car_state.vEgo <= self.blinker_low_speed_limit
    self.steer_fault = car_state.steerFaultTemporary or car_state.steerFaultPermanent
    self.invalid_gear = car_state.gearShifter not in [car.CarState.GearShifter.drive, car.CarState.GearShifter.sport, car.CarState.GearShifter.low, car.CarState.GearShifter.eco]
    self.calibrated = self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

    # Check events to see if we should disable or soft disable
    self.check_event_state()

    self.last_lat_allowed = lateral_allowed

  @property
  def _lat_active(self):

    # Do not allow lateral if both aol and op are disengaged
    if not self.check_aol_state():
      return False

    # Do not allow lateral for first few seconds to ensure op has properly obtained car state
    if time.time() - self.start_time < 5:
      return False

    # If car is in a gear that does not move forward do not engage lateral
    if self.invalid_gear:
      return False

    # If there is a steer fault lat is not available
    if self.steer_fault:
      return False

    # If OP is not calibrated lat is not available
    if not self.calibrated:
      return False

    # Disable lateral when vehicle is stopped to prevent "twitching" steering wheel
    if self.standstill:
      return False

    # If DisengageLatOnBrake is enabled we disable lat while braking
    if self.disengage_lat_on_brake and self.braking:
      return False

    # If DisengageLatOnBlinker is enabled we disable lat while blinkers are on
    if self.disengage_lat_on_blinker and self.blinkers_active and not self.op_active:
      return False

    # If DisengageLatOnLowSpeedBlinker is enabled we disable lat while blinkers are on and we are below the blinker low speed limit
    if self.disengage_lat_on_low_speed_blinker and (self.blinkers_active and self.blinkers_low_speed) and not self.op_active:
      return False

    # Lat is active if we pass all previous tests for disengagement
    return True

  def check_event_state(self):
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)
    if not self.lat_active:
      return

    if self.disabled:
      return

    if self.controls.events.contains(ET.IMMEDIATE_DISABLE):
      self.disabled = True
      self.soft_disabling = False
      self.controls.state = State.disabled
      # only alert if OP is not active as it will alert itself
      if not self.op_active:
        self.controls.current_alert_types.append(ET.IMMEDIATE_DISABLE)
    elif self.soft_disabling:
      if not self.controls.events.contains(ET.SOFT_DISABLE):
        # no more soft disabling condition, so go back to ENABLED
        self.soft_disabling = False

      elif self.soft_disable_timer > 0:
        self.controls.current_alert_types.append(ET.SOFT_DISABLE)
        # only alert if OP is not active as it will alert itself
        if not self.op_active:
          self.controls.current_alert_types.append(ET.SOFT_DISABLE)

      elif self.soft_disable_timer <= 0:
        self.disabled = True
        self.soft_disabling = False
        self.controls.state = State.disabled
    elif self.controls.events.contains(ET.SOFT_DISABLE):
      self.soft_disabling = True
      self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
      # only alert if OP is not active as it will alert itself
      if not self.op_active:
        self.controls.current_alert_types.append(ET.SOFT_DISABLE)

    # Ensure we send warnings when only lat is active
    if self.lat_active:
      self.controls.current_alert_types.append(ET.WARNING)

  @property
  def lat_active(self):
    lat_active = self._lat_active

    # push lat active to the ui
    if self.prev_lat_active != lat_active:
      mem_params.put_bool("LateralActive", lat_active)

    self.prev_lat_active = lat_active
    return lat_active

  @staticmethod
  def alternative_experience():
    experience = 0
    if params.get_bool('AlwaysOnLateralEnabled'):
      experience = ALTERNATIVE_EXPERIENCE.ENABLE_ALWAYS_ON_LATERAL
    return experience
