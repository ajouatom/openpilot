from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carcontroller import CarController


def views(mode, tenths=10, raw=0, frames=600):
  cc = SimpleNamespace(bvm_view=mode, bvm_raw=raw, bvm_alt_time=tenths, CAN=SimpleNamespace(ECAN=0), frame=0)
  out = []
  for cc.frame in range(frames):
    for addr, dat, bus in CarController.bvm_view_msg(cc):
      assert (addr, bus) == (0x400, 0) and dat[:4] == bytes(4) and dat[5:] == bytes(3)
      out.append((cc.frame, dat[4]))
  return out


def test_single_side_at_stock_5hz():
  assert views(1, frames=100) == [(f, 3) for f in range(0, 100, 20)]
  assert {v for _, v in views(2, frames=100)} == {6}


@pytest.mark.parametrize('tenths', [2, 5, 10, 30])
def test_alternate_uses_the_set_interval(tenths):
  period = tenths * 10
  assert all(v == (3 if (f // period) % 2 == 0 else 6) for f, v in views(4, tenths, frames=4 * period))


@pytest.mark.parametrize('tenths', [0, 1, 2])
def test_fastest_is_every_5hz_frame(tenths):
  # never faster than one stock-rate frame per side, whatever the setting
  assert [v for _, v in views(4, tenths, frames=200)] == [3, 6] * 5


def test_raw_wins():
  assert {v for _, v in views(1, raw=5, frames=100)} == {5}
