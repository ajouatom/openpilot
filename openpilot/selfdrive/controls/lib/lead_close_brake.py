# Earlier braking when already inside the follow distance and closing on a lead (StopGoAssist >= 1).
# Logs 10-24: at 0.5 s gap the MPC let the gap shrink 10 -> 4 m while the lead slowed gently, then braked hard.
# Replays of 35 lead braking events (02_logs/measurements.md §25): min gap +0.5~0.8 m, never smaller, accel unchanged.
import numpy as np

FADE_KPH = (45., 65.)  # full below, none above: highway following is left as is unless the gap is short
# 11-08 21:25: 85 km/h at a 0.5 s gap, lead braking -0.5, no extra at all; gap fell to 0.52 s and the driver braked
TIME_GAP = 0.9  # s: actual dRel / vEgo below which the speed fade is ignored
HIGHWAY_CLOSING_SPEED = 1.0  # m/s: 0.3 fired in 38 % of steady 0.5 s highway following (replay), 1.0 in 2.7 %
TF_EXTRA = 0.3  # s
OBSTACLE_COST_MAX = 4.0  # x X_EGO_OBSTACLE_COST at full extra
BRAKING_A_CHANGE_COST = 50.  # instead of A_CHANGE_COST 200 while the lead brakes
CLOSING_SPEED = 0.3  # m/s faster than the lead
LEAD_ACCEL_MAX = 0.3  # not while the lead pulls away
LEAD_BRAKING_ACCEL = -0.5
RISE_RATE, FALL_RATE = 0.6, 0.1  # s of time gap per s, as carrot t_follow ramps


def speed_fade(v_ego):
  return float(np.interp(v_ego * 3.6, FADE_KPH, [1., 0.]))


def tf_extra_request(v_ego, lead, desired_distance):
  closing = (lead.status and lead.dRel < desired_distance and lead.vLead - v_ego < -CLOSING_SPEED
             and lead.aLeadK <= LEAD_ACCEL_MAX)
  if not closing:
    return 0.
  # radar leads only: a one-frame vision lead met this in the 11-08 22:21:59 cut-in
  short_gap = lead.radar and lead.dRel < TIME_GAP * v_ego and lead.vLead - v_ego < -HIGHWAY_CLOSING_SPEED
  return TF_EXTRA if short_gap else TF_EXTRA * speed_fade(v_ego)


# not keyed to the lead: after a lead change the extra still decays over up to 3 s (more braking, never less)
def ramp_extra(extra, request, dt):
  return min(request, extra + RISE_RATE * dt) if request > extra else max(request, extra - FALL_RATE * dt)


def obstacle_cost_factor(extra):
  # follows the ramped extra so the weight never switches abruptly
  return 1. + (OBSTACLE_COST_MAX - 1.) * extra / TF_EXTRA


def a_change_cost_factor(v_ego, lead, base_cost):
  if not (lead.status and lead.aLeadK < LEAD_BRAKING_ACCEL):
    return 1.
  return 1. - speed_fade(v_ego) * (1. - BRAKING_A_CHANGE_COST / base_cost)
