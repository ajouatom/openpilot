from openpilot.selfdrive.carrot.realtime.transports.raw_ws import RawWsHub


def test_empty_read_retries_before_the_next_display_interval():
  hub = RawWsHub(messaging=None)

  assert hub._next_read_delay(0.05, payload_received=False) == hub.EMPTY_READ_RETRY
  assert hub._next_read_delay(0.05, payload_received=True) == 0.05


def test_empty_read_retry_never_exceeds_a_fast_service_cadence():
  hub = RawWsHub(messaging=None)

  assert hub._next_read_delay(0.004, payload_received=False) == 0.004
  assert hub._next_read_delay(0.0, payload_received=False) == hub.MIN_POLL_SLEEP
