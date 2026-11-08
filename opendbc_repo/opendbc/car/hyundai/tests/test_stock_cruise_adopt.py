"""StockCruiseAdopt: a device restart while the camera's stock SCC cruises hands over instead of cutting it off."""
import pytest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.carstate import CarState, STOCK_ADOPT_FRAMES


def state(main=True, acc_mode=1):
  cs = CarState.__new__(CarState)
  cs.stock_adopt_pending, cs.stock_adopt_kph, cs.stock_adopt_frames = True, 0., 0
  cs.main_enabled, cs.manual_main_off_latched = False, True
  cs.MainMode_ACC, cs.ACCMode = main, acc_mode
  return cs


@pytest.mark.parametrize("acc_mode", [1, 2])
def test_cruising_stock_arms_main_and_reports_its_set_speed(acc_mode):
  cs = state(acc_mode=acc_mode)
  cs._update_stock_adopt(True, 87 * CV.KPH_TO_MS)
  assert cs.stock_adopt_pending and cs.main_enabled and not cs.manual_main_off_latched
  assert cs.stock_adopt_kph == pytest.approx(87)


def test_waits_for_the_camera_frame_before_deciding():
  cs = state(main=False, acc_mode=0)
  cs._update_stock_adopt(False, 0.)
  assert cs.stock_adopt_pending and not cs.main_enabled and cs.stock_adopt_kph == 0


@pytest.mark.parametrize("main, acc_mode", [(False, 0), (True, 0), (True, 4)])
def test_stock_not_cruising_is_a_normal_start(main, acc_mode):
  cs = state(main, acc_mode)
  cs._update_stock_adopt(True, 80 * CV.KPH_TO_MS)
  assert not cs.stock_adopt_pending and not cs.main_enabled and cs.stock_adopt_kph == 0
  cs.ACCMode = 1  # a later stock engage is not a restart hand-over
  cs._update_stock_adopt(True, 80 * CV.KPH_TO_MS)
  assert cs.stock_adopt_kph == 0


def test_window_closes_after_startup():
  cs = state()
  cs.stock_adopt_frames = STOCK_ADOPT_FRAMES
  cs._update_stock_adopt(True, 80 * CV.KPH_TO_MS)
  assert not cs.stock_adopt_pending and cs.stock_adopt_kph == 0
