# Earlier braking when already inside the follow distance and closing on a lead (LeadCloseBrake, default off).
# Logs 10-24: at 0.5 s gap the MPC let the gap shrink 10 -> 4 m while the lead slowed gently, then braked hard.
# Replays of 35 lead braking events (02_logs/measurements.md §25): min gap +0.5~0.8 m, never smaller, accel unchanged.
import numpy as np

FADE_KPH = (45., 65.)  # full below, none above: highway following is left as is
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
  return TF_EXTRA * speed_fade(v_ego) if closing else 0.


def ramp_extra(extra, request, dt):
  return min(request, extra + RISE_RATE * dt) if request > extra else max(request, extra - FALL_RATE * dt)


def obstacle_cost_factor(extra):
  # follows the ramped extra so the weight never switches abruptly
  return 1. + (OBSTACLE_COST_MAX - 1.) * extra / TF_EXTRA


def a_change_cost_factor(v_ego, lead, base_cost):
  if not (lead.status and lead.aLeadK < LEAD_BRAKING_ACCEL):
    return 1.
  return 1. - speed_fade(v_ego) * (1. - BRAKING_A_CHANGE_COST / base_cost)
