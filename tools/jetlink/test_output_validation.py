"""One invalid output is still fatal; diagnostics cannot mutate or repair it."""
import numpy as np
import pytest

from openpilot.selfdrive.modeld.output_validation import validate_compiled_output


def test_finite_output_is_not_changed():
  value = np.array([0., -0., np.finfo(np.float32).max], np.float32)
  original = value.tobytes()
  validate_compiled_output(value, 3)
  assert value.tobytes() == original


def test_wrong_count_is_distinguished_from_nonfinite():
  with pytest.raises(ValueError, match='count=2, expected=3'):
    validate_compiled_output(np.array([0., np.nan], np.float32), 3)


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_one_invalid_element_is_still_rejected(bad):
  value = np.zeros(100, np.float32)
  value[71] = bad
  original = value.tobytes()
  with pytest.raises(ValueError, match=r'first_bad_indices=\[71\]'):
    validate_compiled_output(value, 100)
  assert value.tobytes() == original


def test_large_failure_diagnostics_are_bounded_and_distinguish_signs():
  value = np.full(100000, np.nan, np.float32)
  value[-2:] = [np.inf, -np.inf]
  with pytest.raises(ValueError) as error:
    validate_compiled_output(value, len(value))
  text = str(error.value)
  assert 'nan=99998, posinf=1, neginf=1' in text
  assert 'first_bad_indices=[0, 1, 2, 3, 4, 5, 6, 7]' in text
  assert len(text) < 200
