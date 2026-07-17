from types import SimpleNamespace

from openpilot.selfdrive.carrot.carrot_man import (
  CARROT_TMUX_SEND_ONROAD_DELAY_SECONDS,
  carrot_can_error,
  carrot_tmux_send_ready,
)


def test_tmux_send_waits_for_five_seconds_onroad():
  start = 100.0

  assert not carrot_tmux_send_ready("tmux_send", None, start + 10.0)
  assert not carrot_tmux_send_ready("tmux_send", start, start + CARROT_TMUX_SEND_ONROAD_DELAY_SECONDS - 0.01)
  assert carrot_tmux_send_ready("tmux_send", start, start + CARROT_TMUX_SEND_ONROAD_DELAY_SECONDS)


def test_other_exception_reasons_are_not_delayed():
  assert carrot_tmux_send_ready("exception", None, 0.0)
  assert carrot_tmux_send_ready("log", None, 0.0)


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
