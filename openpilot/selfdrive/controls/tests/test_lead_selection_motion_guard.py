from openpilot.selfdrive.controls.radard import LeadSelectionMotionGuard


def update(guard: LeadSelectionMotionGuard, v_lead: float, a_lead: float, j_lead: float,
           track_id: int = 37):
  return guard.update(True, True, track_id, v_lead, a_lead, j_lead)


def test_new_lead_rejects_uncorroborated_motion_transient():
  guard = LeadSelectionMotionGuard()

  assert update(guard, 7.181, -3.203, -5.050) == (0.0, 0.0, False)
  assert update(guard, 7.174, -3.343, -3.990) == (0.0, 0.0, False)
  assert update(guard, 7.232, -1.820, 1.406) == (0.0, 0.0, False)

  a_lead, j_lead, accepted = update(guard, 7.153, -0.913, 4.373)
  assert accepted
  assert a_lead == -0.913
  assert j_lead == 4.373


def test_new_braking_lead_is_accepted_on_second_frame_when_velocity_agrees():
  guard = LeadSelectionMotionGuard()

  assert update(guard, 20.0, -2.0, -4.0) == (0.0, 0.0, False)
  a_lead, j_lead, accepted = update(guard, 19.9, -2.0, -3.0)

  assert accepted
  assert a_lead == -2.0
  assert j_lead == 0.0


def test_lead_handoff_restarts_motion_guard():
  guard = LeadSelectionMotionGuard()
  for v_lead in (20.0, 19.9, 19.8, 19.7):
    update(guard, v_lead, -2.0, -1.0)

  assert update(guard, 18.0, -3.0, -5.0, track_id=52) == (0.0, 0.0, False)
