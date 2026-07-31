import pytest

from openpilot.selfdrive.ui.carrot_web import (
  CARROT_WEB_AUTO_CLOSE_SECONDS,
  CARROT_WEB_MIN_CLOSE_SECONDS,
  CarrotWebAddressWatcher,
  CarrotWebQrSession,
  build_carrot_web_url,
  fit_single_line_font_size,
)


class FakeParamsMemory:
  def __init__(self, value=None):
    self.value = value
    self.requested_keys = []

  def get(self, key):
    self.requested_keys.append(key)
    return self.value


class FakeQRCodeTexture:
  def __init__(self):
    self.values = []
    self.destroyed = False

  def set_data(self, value):
    self.values.append(value)

  def destroy(self):
    self.destroyed = True


@pytest.mark.parametrize(
  "address,expected",
  [
    (None, None),
    ("", None),
    ("0.0.0.0", None),
    ("127.0.0.1", None),
    ("not-an-ip", None),
    (" 192.168.43.10\n", "http://192.168.43.10:7000"),
    ("2001:db8::1", "http://[2001:db8::1]:7000"),
  ],
)
def test_build_carrot_web_url(address, expected):
  assert build_carrot_web_url(address) == expected


@pytest.mark.parametrize(
  "preferred_size,measured_width,max_width,expected",
  [
    (26, 250, 274, 26),
    (26, 300, 274, 23),
    (65, 750, 685, 59),
    (26, 300, 0, 1),
  ],
)
def test_fit_single_line_font_size(preferred_size, measured_width, max_width, expected):
  assert fit_single_line_font_size(preferred_size, measured_width, max_width) == expected


def test_address_watcher_polls_at_interval_and_reports_changes():
  params = FakeParamsMemory("192.168.0.2")
  watcher = CarrotWebAddressWatcher(params, refresh_interval=0.1)

  assert watcher.refresh(1.0)
  assert watcher.value.url == "http://192.168.0.2:7000"

  params.value = "192.168.0.3"
  assert not watcher.refresh(1.05)
  assert watcher.value.url == "http://192.168.0.2:7000"

  assert watcher.refresh(1.1)
  assert watcher.value.url == "http://192.168.0.3:7000"
  assert params.requested_keys == ["NetworkAddress", "NetworkAddress"]


def test_qr_session_tracks_address_changes_and_clears_stale_qr():
  params = FakeParamsMemory("192.168.43.1")
  qr = FakeQRCodeTexture()
  timestamps = iter(["16:43:27", "16:44:02"])
  session = CarrotWebQrSession(params, qr_texture=qr, timestamp_factory=lambda: next(timestamps))

  session.open(10.0)
  assert session.url == "http://192.168.43.1:7000"
  assert qr.values == ["http://192.168.43.1:7000"]
  assert session.updated_time == "16:43:27"

  params.value = "10.0.0.25"
  assert not session.update(10.05)
  assert qr.values == ["http://192.168.43.1:7000"]

  assert not session.update(10.1)
  assert session.url == "http://10.0.0.25:7000"
  assert qr.values[-1] == "http://10.0.0.25:7000"
  assert session.updated_time == "16:44:02"

  params.value = "0.0.0.0"
  assert not session.update(10.2)
  assert session.url is None
  assert qr.values[-1] is None
  assert session.updated_time is None


def test_qr_session_close_timing_and_cleanup():
  qr = FakeQRCodeTexture()
  session = CarrotWebQrSession(FakeParamsMemory("192.168.0.2"), qr_texture=qr)
  session.open(5.0)

  assert session.seconds_until_close(5.0) == 30
  assert session.seconds_until_close(5.1) == 30
  assert session.seconds_until_close(6.0) == 29
  assert session.seconds_until_close(35.0) == 0
  assert not session.can_close(5.0 + CARROT_WEB_MIN_CLOSE_SECONDS - 0.01)
  assert session.can_close(5.0 + CARROT_WEB_MIN_CLOSE_SECONDS + 0.001)
  assert not session.update(5.0 + CARROT_WEB_AUTO_CLOSE_SECONDS - 0.01)
  assert session.update(5.0 + CARROT_WEB_AUTO_CLOSE_SECONDS)

  session.destroy()
  assert qr.destroyed
