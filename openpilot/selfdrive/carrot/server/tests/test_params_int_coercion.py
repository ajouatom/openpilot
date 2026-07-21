"""Integer coercion must round, not truncate.

int(float(v)) turns 14.999 into 14, which would surface as a setting that
reads one lower than the one that was saved -- the exact symptom this work
started from.
"""
import pytest

from openpilot.selfdrive.carrot.server.services.params import _to_int


@pytest.mark.parametrize("value, expected", [
  (15, 15),
  (15.0, 15),
  ("15", 15),
  ("15.0", 15),
])
def test_exact_values_pass_through(value, expected):
  assert _to_int(value) == expected


@pytest.mark.parametrize("value", [14.999, 14.9999999, "14.999"])
def test_a_value_just_below_the_target_rounds_up_to_it(value):
  assert _to_int(value) == 15, "truncation here is what makes a setting read one step low"


@pytest.mark.parametrize("value", [15.001, 15.4, "15.4"])
def test_a_value_just_above_the_target_stays_on_it(value):
  assert _to_int(value) == 15


def test_negative_values_round_toward_the_nearest_not_toward_zero():
  # Truncation would give 0 here, turning "-1" into "no effect".
  assert _to_int(-0.999) == -1
  assert _to_int(-14.999) == -15
  assert _to_int(-15.4) == -15


def test_rounding_matches_the_clamp_path_in_features_params():
  # features/params.py uses int(round(fv)); both must agree on the half case
  # so a value does not change depending on which path wrote it.
  for value in (0.5, 1.5, 2.5, -0.5, -1.5):
    assert _to_int(value) == int(round(float(value)))


def test_unusable_input_raises_rather_than_silently_writing_zero():
  for value in ("", "abc", None):
    with pytest.raises((TypeError, ValueError)):
      _to_int(value)
