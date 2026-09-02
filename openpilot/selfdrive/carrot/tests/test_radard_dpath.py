import pytest

from openpilot.selfdrive.carrot.radar import (
  effective_radar_track_mode,
)


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_hyundai_keeps_configured_radar_track_mode(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "hyundai", False, configured_mode,
  ) == configured_mode


@pytest.mark.parametrize(
  "brand", ("volkswagen", "honda", "toyota", "ford", "subaru"),
)
@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_ignore_option_and_use_front_radar(
  brand: str,
  configured_mode: int,
) -> None:
  assert effective_radar_track_mode(
    brand, False, configured_mode,
  ) == 1


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_without_radar_use_vision(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "mazda", True, configured_mode,
  ) == -2
