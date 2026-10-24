# Low-speed stop & go corrections on the planner's output accel (StopGoAssist: 1 stop guard, 2 stop guard + departure boost;
# both levels also enable lead_close_brake in the MPC).
# Replays of 17 stops / 25 departures behind a lead (02_logs/measurements.md §22-23): the MPC overshoots the stop
# distance by ~0.9 m (too weak braking below 3 m/s) and trails an accelerating lead after standstill.

LOW_SPEED = 3.0  # m/s

STOP_GUARD_LEAD_SPEED = 0.3  # lead treated as stopped
STOP_GUARD_MIN_ACCEL = -1.5
STOP_GUARD_MIN_GAP = 0.2

DEPART_WINDOW = 3.0  # s after standstill
DEPART_ACCEL = 1.2
DEPART_LEAD_DV = 0.3  # lead faster than ego
DEPART_LEAD_ACCEL = 0.5  # lead still accelerating: skips leads that creep and stop again
DEPART_GAP_MARGIN = 0.8  # below stop distance


def stop_guard(a_target, v_ego, lead, stop_distance):
  # Only strengthens braking the planner already wants: brake at least as hard as a constant decel stop at stop_distance.
  if not (0.1 < v_ego < LOW_SPEED) or a_target > 0.0 or not lead.status or abs(lead.vLead) >= STOP_GUARD_LEAD_SPEED:
    return a_target
  a_req = -v_ego ** 2 / (2.0 * max(lead.dRel - stop_distance, STOP_GUARD_MIN_GAP))
  return min(a_target, max(STOP_GUARD_MIN_ACCEL, a_req))


def departure_boost(a_target, v_ego, lead, stop_distance, since_standstill, accel_max, track_stable):
  # Only raises acceleration the planner already wants, toward a stable radar lead that is pulling away.
  if (since_standstill > DEPART_WINDOW or v_ego >= LOW_SPEED or a_target <= 0.0 or not track_stable
      or not (lead.status and lead.radar) or lead.vLead - v_ego <= DEPART_LEAD_DV or lead.aLeadK <= DEPART_LEAD_ACCEL
      or lead.dRel <= stop_distance - DEPART_GAP_MARGIN):
    return a_target
  return max(a_target, min(DEPART_ACCEL, accel_max))
