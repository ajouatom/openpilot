import math
from dataclasses import dataclass


LATERAL_SERVICES = ('modelV2', 'liveParameters', 'livePose', 'selfdriveState', 'onroadEvents')


def service_ready(sm, service):
  # A default SubMaster message is not an input, even during startup grace.
  return sm.seen[service] and sm.all_checks([service])


def live_parameters_ready(sm):
  lp = sm['liveParameters']
  return (service_ready(sm, 'liveParameters') and lp.valid and lp.sensorValid and lp.posenetValid and
          all(math.isfinite(x) for x in (lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetDeg, lp.roll)) and
          lp.steerRatio > 1.0 and lp.stiffnessFactor > 0.0)


@dataclass(frozen=True)
class NominalLateralParameters:
  steerRatio: float
  stiffnessFactor: float = 1.0
  angleOffsetDeg: float = 0.0
  roll: float = 0.0


def lateral_vehicle_parameters(sm, CP):
  # Keep nominal geometry while live inputs are absent/invalid. This fallback
  # is for inactive calculations; it does not grant permission to steer.
  return sm['liveParameters'] if live_parameters_ready(sm) else NominalLateralParameters(CP.steerRatio)


def lateral_inputs_ready(sm, CS):
  if not all(service_ready(sm, s) for s in LATERAL_SERVICES) or not live_parameters_ready(sm):
    return False
  if not CS.canValid or CS.steerFaultTemporary or CS.steerFaultPermanent:
    return False
  if not all(math.isfinite(x) for x in (CS.vEgo, CS.steeringAngleDeg, CS.steeringRateDeg, CS.steeringTorque)):
    return False
  if any(e.name == 'selfdriveInitializing' for e in sm['onroadEvents']):
    return False
  pose = sm['livePose']
  if not (pose.inputsOK and pose.posenetOK and pose.sensorsOK):
    return False
  if not math.isfinite(sm['modelV2'].action.desiredCurvature):
    return False
  # The lane planner is an additional input when it requests lane-line use.
  # A missing planner cannot supply this flag; model-only control still needs
  # all of the common lateral inputs above.
  if sm['lateralPlan'].useLaneLines:
    plan = sm['lateralPlan']
    if not service_ready(sm, 'lateralPlan') or not plan.mpcSolutionValid:
      return False
    if not (len(plan.psis) == len(plan.curvatures) == len(plan.distances) == 17):
      return False
    if not all(math.isfinite(x) for values in (plan.psis, plan.curvatures, plan.distances) for x in values):
      return False
  return True
