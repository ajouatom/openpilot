from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_lib.lever import LEVER_LATCHED, LEVER_ONE_TOUCH, LEVER_RELEASED, LeverForcedTurn

LIT_S, DARK_S = 0.4, 0.4  # measured lamp phases (0x413 *_LAMP_ALT, 2026-10-11)


def _run(t, lever, lamp_side, seconds):
  """Feed `seconds` of the flashing lamp (one clock from the first call; lamp_side 0 = dark); returns the last result."""
  out = False
  for _ in range(round(seconds / DT_MDL)):
    k = t.__dict__.setdefault("_k", 0)
    lit = (k * DT_MDL) % (LIT_S + DARK_S) < LIT_S
    out = t.update(lever, lamp_side if lamp_side and lit else 0)
    t._k = k + 1
  return out


def test_one_touch_two_lightings_then_latch_forces_a_turn():
  t = LeverForcedTurn()
  _run(t, LEVER_ONE_TOUCH, 2, 0.15)          # tap: lit
  _run(t, LEVER_RELEASED, 2, 0.75)          # released, still flashing: dark, lit again
  assert _run(t, LEVER_LATCHED, 2, 0.5)     # latched afterwards -> forced turn
  assert not _run(t, LEVER_RELEASED, 0, 0.1)  # releasing the latch ends it


def test_plain_latch_through_the_detent_is_not_forced():
  t = LeverForcedTurn()
  _run(t, LEVER_ONE_TOUCH, 1, 0.1)          # a latch passes the one-touch detent for ~0.1 s
  assert not _run(t, LEVER_LATCHED, 1, 2.0)


def test_latch_after_only_one_lighting_is_not_forced():
  t = LeverForcedTurn()
  _run(t, LEVER_ONE_TOUCH, 1, 0.15)
  _run(t, LEVER_RELEASED, 1, 0.5)           # lit, then dark, but not lit again yet
  assert not _run(t, LEVER_LATCHED, 1, 1.0)


def test_gesture_expires_when_the_lamp_stops():
  t = LeverForcedTurn()
  _run(t, LEVER_ONE_TOUCH, 2, 0.15)
  _run(t, LEVER_RELEASED, 2, 0.75)
  _run(t, LEVER_RELEASED, 0, 1.2)           # flashing over
  assert not _run(t, LEVER_LATCHED, 2, 1.0)
