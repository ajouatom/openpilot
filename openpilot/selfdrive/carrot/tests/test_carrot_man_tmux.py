from types import SimpleNamespace

from openpilot.selfdrive.carrot.carrot_man import (
  CARROT_CAN_ERROR_TMUX_DELAY_SECONDS,
  CARROT_EXCEPTION_TMUX_REASONS,
  carrot_can_error,
  carrot_can_error_sources,
  carrot_can_error_send_ready,
)


def test_spi_error_requests_tmux_capture():
  assert "spi_error" in CARROT_EXCEPTION_TMUX_REASONS


def test_egpu_error_requests_tmux_capture():
  assert "egpu_error" in CARROT_EXCEPTION_TMUX_REASONS


def test_can_error_send_waits_for_five_seconds_after_detection():
  detected_at = 100.0

  assert not carrot_can_error_send_ready(None, detected_at + 10.0, True)
  assert not carrot_can_error_send_ready(detected_at, detected_at + CARROT_CAN_ERROR_TMUX_DELAY_SECONDS - 0.01, True)
  assert carrot_can_error_send_ready(detected_at, detected_at + CARROT_CAN_ERROR_TMUX_DELAY_SECONDS, True)


def test_can_error_send_is_canceled_offroad():
  assert not carrot_can_error_send_ready(100.0, 110.0, False)


def test_can_error_ignores_mock_car():
  invalid_car_state = SimpleNamespace(canTimeout=False, canValid=False)
  radar_state = SimpleNamespace(radarErrors=SimpleNamespace(canError=False))

  assert not carrot_can_error("MOCK", True, invalid_car_state, True, radar_state)
  assert not carrot_can_error(b"MOCK", True, invalid_car_state, True, radar_state)


def test_can_error_detects_car_and_radar_errors():
  valid_car_state = SimpleNamespace(canTimeout=False, canValid=True)
  invalid_car_state = SimpleNamespace(canTimeout=False, canValid=False)
  radar_ok = SimpleNamespace(radarErrors=SimpleNamespace(canError=False))
  radar_error = SimpleNamespace(radarErrors=SimpleNamespace(canError=True))

  assert carrot_can_error("HYUNDAI_SONATA", True, invalid_car_state, True, radar_ok)
  assert carrot_can_error("HYUNDAI_SONATA", True, valid_car_state, True, radar_error)
  assert not carrot_can_error("HYUNDAI_SONATA", False, invalid_car_state, False, radar_error)


def test_can_error_ignores_stale_previous_onroad_state():
  stale_invalid_car_state = SimpleNamespace(canTimeout=True, canValid=False)
  stale_radar_error = SimpleNamespace(radarErrors=SimpleNamespace(canError=True))

  assert carrot_can_error_sources(
    "HYUNDAI_SONATA", False, stale_invalid_car_state, False, stale_radar_error,
  ) == (False, False)
