from types import SimpleNamespace as NS

from opendbc.car.hyundai.hyundaicanfd import _blink_hold_side


def _md(y_left, y_right):
  return NS(laneLines=[NS(y=[-5.0]), NS(y=[y_left]), NS(y=[y_right]), NS(y=[5.0])])


def test_blink_hold_follows_the_confirmed_change_without_lane_lines():
  state = {}
  assert _blink_hold_side(None, 3, state) == 3                      # no model message: hold for the desire
  assert _blink_hold_side(NS(laneLines=[]), 4, state) == 4          # missing / short lane-line lists
  assert _blink_hold_side(NS(laneLines=[NS(y=[]), NS(y=[]), NS(y=[])]), 4, state) == 4
  assert _blink_hold_side(None, 0, state) == 0


def test_blink_hold_releases_once_the_body_is_across():
  # Left change, 2026-09-19 #9: the left line closes in, the model relabels it over two frames,
  # then the crossed line (now on the right) moves away from the car centre.
  state = {}
  frames = [(-1.75, 1.75), (-1.2, 2.3), (-0.6, 2.8), (-0.28, 3.04), (-1.36, 2.21), (-3.46, 0.12),
            (-3.11, 0.32)]
  assert all(_blink_hold_side(_md(*f), 3, state) == 3 for f in frames)
  assert state["crossed"] and not state.get("released")         # centre across, 0.32 m < 0.1 lane width
  assert _blink_hold_side(_md(-2.62, 0.76), 3, state) == 0          # 0.76 m past the line: over 60 %
  assert _blink_hold_side(_md(-1.9, 1.6), 3, state) == 0            # stays released for this change
  assert _blink_hold_side(_md(-1.9, 1.6), 0, state) == 0            # desire ends: reset
  assert _blink_hold_side(_md(-1.8, 1.8), 4, state) == 4            # the next change holds again


def test_blink_hold_right_change_uses_the_right_line():
  state = {}
  for y_left, y_right in [(-1.9, 1.66), (-2.5, 0.98), (-3.06, 0.17), (-3.09, 0.27), (-1.64, 1.66), (-0.21, 3.08)]:
    assert _blink_hold_side(_md(y_left, y_right), 4, state) == 4
  assert _blink_hold_side(_md(-0.33, 3.0), 4, state) == 4           # 0.33 m < 0.1 x 3.33 m: under 60 %
  assert _blink_hold_side(_md(-0.4, 2.9), 4, state) == 0            # 0.4 m >= 0.33 m: 60 % over


def test_blink_hold_without_a_visible_crossing_lasts_to_the_end_of_the_desire():
  # 2026-09-19 #11: the model lines follow the car and never come within 1 m of the centre.
  state = {}
  for y_left, y_right in [(-2.9, 1.5), (-2.4, 1.1), (-2.2, 0.8), (-1.5, 1.3), (-1.2, 1.9), (-1.7, 1.8)]:
    assert _blink_hold_side(_md(y_left, y_right), 3, state) == 3
  assert _blink_hold_side(_md(-1.7, 1.8), 0, state) == 0
