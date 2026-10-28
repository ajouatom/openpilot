# Low-speed stop & go corrections on the planner's output accel (StopGoAssist: 1 stop guard, 2 also departure go, 3 also
# soft stop; every level also enables lead_close_brake in the MPC).
# Replays of 17 stops / 25 departures behind a lead (02_logs/measurements.md §22-23): the MPC overshoots the stop
# distance by ~0.9 m (too weak braking below 3 m/s) and trails an accelerating lead after standstill.
# 10-28 drive (§28): the car keeps the decel it has when StopReq starts (-1.2 -> a jolt), and a creeping lead (aLeadK 0.2)
# left us waiting 1 s at standstill because the gap was under the stop distance.

LOW_SPEED = 3.0  # m/s

STOP_GUARD_LEAD_SPEED = 0.3  # lead treated as stopped
STOP_GUARD_MIN_ACCEL = -1.5
STOP_GUARD_MIN_GAP = 0.2

# Soft stop: below SOFT_SPEED the decel eases linearly to SOFT_END_ACCEL at SOFT_END_SPEED (StopReq starts at <=0.7 m/s),
# like the stock stop. Behind a stopped lead it may roll past the stop distance (user 10-28: stopping closer is fine, the
# lead is not moving), but never eases below what still stops SOFT_MIN_GAP from the lead.
SOFT_SPEED = 2.0
SOFT_END_SPEED = 0.5
SOFT_END_ACCEL = -0.5
SOFT_MIN_GAP = 2.5  # m from the lead

DEPART_WINDOW = 3.0  # s after standstill
DEPART_ACCEL = 1.2
DEPART_LEAD_DV = 0.3  # lead faster than ego
DEPART_LEAD_ACCEL = 0.1  # lead accelerating at all (0.5 left a creeping lead waiting 1 s, 10-28 405 s)
DEPART_MIN_GAP = 2.5  # m; the lead is pulling away, so the stop distance target is not waited for


def _stopped_lead(a_target, v_ego, lead, top_speed):
  return 0.1 < v_ego < top_speed and a_target <= 0.0 and lead.status and abs(lead.vLead) < STOP_GUARD_LEAD_SPEED


def _stop_accel(v_ego, lead, stop_distance):
  return -v_ego ** 2 / (2.0 * max(lead.dRel - stop_distance, STOP_GUARD_MIN_GAP))


def stop_guard(a_target, v_ego, lead, stop_distance):
  # Only strengthens braking the planner already wants: brake at least as hard as a constant decel stop at stop_distance.
  if not _stopped_lead(a_target, v_ego, lead, LOW_SPEED):
    return a_target
  return min(a_target, max(STOP_GUARD_MIN_ACCEL, _stop_accel(v_ego, lead, stop_distance)))


def soft_stop(a_target, v_ego, lead):
  if not _stopped_lead(a_target, v_ego, lead, SOFT_SPEED) or lead.dRel - SOFT_MIN_GAP <= STOP_GUARD_MIN_GAP:
    return a_target
  floor = SOFT_END_ACCEL + (STOP_GUARD_MIN_ACCEL - SOFT_END_ACCEL) * \
    min(max((v_ego - SOFT_END_SPEED) / (SOFT_SPEED - SOFT_END_SPEED), 0.0), 1.0)
  # Only eases off, never past what still stops SOFT_MIN_GAP from the lead (strengthening is the guard's job)
  return max(a_target, min(floor, _stop_accel(v_ego, lead, SOFT_MIN_GAP)))


def departure_go(v_ego, lead, since_standstill, track_stable):
  # A stable radar lead pulling away right after standstill: go even inside the stop distance.
  return (since_standstill <= DEPART_WINDOW and v_ego < LOW_SPEED and track_stable and lead.status and lead.radar
          and lead.vLead - v_ego > DEPART_LEAD_DV and lead.aLeadK > DEPART_LEAD_ACCEL and lead.dRel > DEPART_MIN_GAP)
