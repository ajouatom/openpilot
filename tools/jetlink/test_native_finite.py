import numpy as np
import pytest

from jetlink.finite import native_checker


@pytest.fixture(scope='module')
def check():
  value = native_checker()
  if value is None:
    pytest.skip('native Linux C compiler unavailable')
  return value


def test_every_binary16_pattern(check):
  values = np.arange(65536, dtype=np.uint16).view(np.float16)
  expected = np.isfinite(values)
  for i in range(values.size):
    assert check(values[i:i + 1]) == expected[i], i


@pytest.mark.parametrize('dtype', [np.float16, np.float32])
@pytest.mark.parametrize('size', [0, 1, 3, 7, 8, 9, 15, 16, 17, 33, 257])
def test_vector_tails_and_every_bad_position(check, dtype, size):
  values = np.ones(size, dtype=dtype)
  assert check(values)
  for i in range(size):
    for bad in (np.nan, np.inf, -np.inf):
      values[i] = bad
      assert not check(values)
    values[i] = 1


def test_float32_payloads_and_fallback_layouts(check):
  rng = np.random.default_rng(812)
  bits = rng.integers(0, 2**32, 10000, dtype=np.uint32)
  values = bits.view(np.float32)
  for i in range(len(values)):
    assert check(values[i:i + 1]) == bool(np.isfinite(values[i]))
  for dtype in ('>f4', '<f4', 'f8'):
    for values in ([1., -0., 3.], [1., np.nan, 3.]):
      a = np.asarray(values, dtype=dtype)
      assert check(a) == bool(np.isfinite(a).all())
      assert check(a[::-1]) == bool(np.isfinite(a[::-1]).all())


def test_unavailable_compiler_falls_back(monkeypatch):
  from jetlink import finite
  monkeypatch.setattr(finite.shutil, 'which', lambda _: None)
  assert finite.native_checker.__wrapped__() is None
