from openpilot.selfdrive.selfdrived.turn_prompt import TURN_ENTER_S, TURN_RELEASE_S, TurnPrompt, TurnPromptTracker

DT = 0.01


def _run(tracker, turning, lat_active, t0, duration):
  prompts = []
  t = t0
  while t < t0 + duration - 1e-9:
    p = tracker.update(turning, lat_active, t)
    if p is not None:
      prompts.append((round(t, 2), p))
    t += DT
  return prompts, t


def test_turn_then_straight_once():
  tr = TurnPromptTracker()
  p1, t = _run(tr, True, True, 0.0, 3.0)
  p2, _ = _run(tr, False, True, t, 2.0)
  assert [x[1] for x in p1] == [TurnPrompt.TURN]
  assert abs(p1[0][0] - TURN_ENTER_S) < 0.02
  assert [x[1] for x in p2] == [TurnPrompt.STRAIGHT]
  assert abs(p2[0][0] - (t + TURN_RELEASE_S)) < 0.02


def test_desire_flicker_says_nothing():
  tr = TurnPromptTracker()
  p1, t = _run(tr, True, True, 0.0, TURN_ENTER_S / 2)
  p2, _ = _run(tr, False, True, t, 2.0)
  assert p1 == [] and p2 == []


def test_short_gap_inside_one_turn_keeps_turning():
  tr = TurnPromptTracker()
  p1, t = _run(tr, True, True, 0.0, 1.0)
  p2, t = _run(tr, False, True, t, TURN_RELEASE_S / 2)
  p3, t = _run(tr, True, True, t, 1.0)
  p4, _ = _run(tr, False, True, t, 1.0)
  assert [x[1] for x in p1 + p2 + p3 + p4] == [TurnPrompt.TURN, TurnPrompt.STRAIGHT]


def test_no_prompt_without_lateral_and_losing_lateral_ends_silently():
  tr = TurnPromptTracker()
  p0, t = _run(tr, True, False, 0.0, 2.0)
  assert p0 == []
  p1, t = _run(tr, True, True, t, 1.0)
  p2, t = _run(tr, True, False, t, 0.5)
  p3, _ = _run(tr, False, True, t, 2.0)
  assert [x[1] for x in p1] == [TurnPrompt.TURN]
  assert p2 == [] and p3 == []
