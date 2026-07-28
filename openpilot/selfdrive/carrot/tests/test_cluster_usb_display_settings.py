from pathlib import Path
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_usb_display import (
  TURZX_BRIGHTNESS_COMMAND_MAX,
  TuringUsbDisplay,
)


def successful_response(command_id: int) -> bytes:
  return bytes((command_id, 0xC8)) + bytes(510)


def recording_display(*, brightness: int = 80, orientation: int = 0):
  display = TuringUsbDisplay(brightness=brightness)
  display.orientation = orientation
  display.dev = object()
  calls = []

  def send(command_id, name, fields=None, **kwargs):
    calls.append((command_id, name, fields, kwargs))
    return successful_response(command_id)

  display._send_command = send
  return display, calls


def test_runtime_brightness_uses_captured_sync_then_command_14(monkeypatch):
  display, calls = recording_display(brightness=65)
  monkeypatch.setattr("cluster_usb_display.time.sleep", lambda _seconds: None)

  assert display.set_brightness(64)
  assert [call[0] for call in calls] == [10, 14]
  assert calls[1][2] == {8: int(64 / 100 * TURZX_BRIGHTNESS_COMMAND_MAX)}
  assert display.brightness == 64


def test_runtime_orientation_uses_captured_sync_then_command_13(monkeypatch):
  display, calls = recording_display(orientation=0)
  monkeypatch.setattr("cluster_usb_display.time.sleep", lambda _seconds: None)

  assert display.set_orientation(2)
  assert [call[0] for call in calls] == [10, 13]
  assert calls[1][2] == {8: 2}
  assert display.orientation == 2


def test_runtime_orientation_ignores_unsupported_values():
  display, calls = recording_display(orientation=2)

  assert not display.set_orientation(1)
  assert not display.set_orientation(3, force=True)
  assert calls == []
  assert display.orientation == 2


def test_h264_setup_carries_the_selected_orientation():
  display = TuringUsbDisplay()
  display.orientation = 2
  display.dev = object()
  calls = []
  display._send_optional_command = (
    lambda command_id, name, fields=None, **kwargs:
      calls.append((command_id, name, fields, kwargs))
  )
  display._h264_chunk_size = lambda _requested: 202752

  assert display.start_h264_stream() == 202752
  orientation = next(call for call in calls if call[0] == 13)
  assert orientation[2] == {8: 2}
