"""A low-speed signal-stop CAN object, never a radar/planner observation."""

import math

from opendbc.car.hyundai.stopping import ENTRY_SPEED
from opendbc.car.hyundai.values import HyundaiFlags

STOP_OBJECT_DISTANCE = 3.0


def has_lead(radar):
  return radar is not None and any(bool(getattr(getattr(radar, name, None), "status", False)) for name in ("leadOne", "leadTwo"))


def signal_stop_object_requested(cp, cs, control, plan, radar, *, inputs_ready):
  scope = HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC
  return bool(
    inputs_ready
    and cp.brand == "hyundai"
    and (cp.flags & scope) == scope
    and cp.openpilotLongitudinalControl
    and control.enabled
    and control.longActive
    and cs.canValid
    and not cs.canTimeout
    and str(cs.gearShifter) == "drive"
    and not (cs.gasPressed or cs.brakePressed or cs.parkingBrake or cs.brakeHoldActive)
    and plan.shouldStop
    and plan.xState in (3, 5)
    and not plan.hasLead
    and not has_lead(radar)
  )


def apply_signal_stop_object(values, cs, hud, *, stopping_controller):
  """Return whether the final, already-interlocked packet contains the object."""
  out = cs.out
  speed = out.vEgo
  wheels = out.wheelSpeeds
  speeds = (speed, out.vEgoRaw, wheels.fl, wheels.fr, wheels.rl, wheels.rr)
  if (
    stopping_controller is None
    or not getattr(hud, "signalStopObject", False)
    or values["ACCMode"] != 1
    or values["StopReq"] != 1
    or not out.canValid
    or getattr(out, "canTimeout", False)
    or str(out.gearShifter) != "drive"
    or out.gasPressed
    or out.brakePressed
    or out.brakeHoldActive
    or out.parkingBrake
    or not all(math.isfinite(value) for value in speeds)
    or max(abs(value) for value in speeds) > ENTRY_SPEED
    or has_lead(getattr(cs, "radarState", None))
  ):
    return False
  # A stationary object has zero absolute speed, not zero relative speed while
  # ego creeps. This is intentionally local to the outgoing SCC object fields.
  values.update(ACC_ObjDist=STOP_OBJECT_DISTANCE, ACC_ObjLatPos=0.0, ACC_ObjRelSpd=-speed, HUD_LEAD_INFO=2)
  return True
