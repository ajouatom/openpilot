import sys
from pathlib import Path

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_usb_display import (
  TURZX_BRIGHTNESS_COMMAND_MAX,
  USBGPU_H264_CHUNK_GAP_S,
  USBGPU_H264_MAX_CHUNK_SIZE,
  TuringUsbDisplay,
  _transparent_h264_overlay_png,
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
  assert all(call[3]["expect_response"] is False for call in calls)
  assert calls[1][2] == {8: int(64 / 100 * TURZX_BRIGHTNESS_COMMAND_MAX)}
  assert display.brightness == 64


def test_runtime_orientation_uses_captured_sync_then_command_13(monkeypatch):
  display, calls = recording_display(orientation=0)
  monkeypatch.setattr("cluster_usb_display.time.sleep", lambda _seconds: None)

  assert display.set_orientation(2)
  assert [call[0] for call in calls] == [10, 13]
  assert all(call[3]["expect_response"] is False for call in calls)
  assert calls[1][2] == {8: 2}
  assert display.orientation == 2


def test_runtime_orientation_ignores_unsupported_values():
  display, calls = recording_display(orientation=2)

  assert not display.set_orientation(1)
  assert not display.set_orientation(3, force=True)
  assert calls == []
  assert display.orientation == 2


def test_preopen_orientation_is_carried_by_h264_setup_without_setting_transaction(monkeypatch):
  display = TuringUsbDisplay()
  assert not display.set_orientation(2)
  assert display.orientation == 2

  display.dev = object()
  calls = []
  display._send_optional_command = (
    lambda command_id, name, fields=None, **kwargs:
      calls.append((command_id, name, fields, kwargs))
  )
  display._send_frame_no_ack = (
    lambda command_id, frame, **kwargs:
      calls.append((command_id, "clear-overlay", {8: len(frame)}, kwargs))
  )
  display._h264_chunk_size = lambda _requested: 202752
  monkeypatch.setattr("cluster_usb_display._usbgpu_transfer_active", lambda: False)
  monkeypatch.setattr("cluster_usb_display.time.sleep", lambda _seconds: None)

  assert display.start_h264_stream() == 202752
  assert [call[0] for call in calls] == [10, 111, 112, 13, 14, 52, 102, 15]
  orientation = next(call for call in calls if call[0] == 13)
  assert orientation[2] == {8: 2}
  assert calls[4][2] == {8: int(80 / 100 * TURZX_BRIGHTNESS_COMMAND_MAX)}
  assert all(call[3].get("no_ack_gap_s") == 0.0 for call in calls[:6])
  assert calls[6][3] == {"drain_input": True}
  assert calls[7][3]["no_ack_gap_s"] == 0.0


def test_h264_egpu_coexistence_caps_chunks_and_yields_after_send(monkeypatch):
  display = TuringUsbDisplay(fast_write=True)
  display.dev = object()
  display._send_optional_command = lambda *_args, **_kwargs: None
  display._send_frame_no_ack = lambda *_args, **_kwargs: None
  display._h264_chunk_size = lambda _requested: 202752
  sent = []
  sleeps = []
  display._send_h264_chunk_no_ack = (
    lambda chunk, *, is_last, drain_input: sent.append((chunk, is_last, drain_input))
  )
  monkeypatch.setattr("cluster_usb_display._usbgpu_transfer_active", lambda: True)
  monkeypatch.setattr("cluster_usb_display.time.sleep", sleeps.append)

  assert display.start_h264_stream() == USBGPU_H264_MAX_CHUNK_SIZE
  assert display._h264_chunk_gap_s == USBGPU_H264_CHUNK_GAP_S

  sleeps.clear()
  display.send_h264_chunk(b"frame", wait_for_ack=False)

  assert sent == [(b"frame", False, False)]
  assert sleeps == [USBGPU_H264_CHUNK_GAP_S]


def test_h264_clear_overlay_matches_captured_shape_and_size():
  png = _transparent_h264_overlay_png()

  assert len(png) == 3585
  assert png[:8] == b"\x89PNG\r\n\x1a\n"
  assert int.from_bytes(png[16:20], "big") == 464
  assert int.from_bytes(png[20:24], "big") == 1920


def test_h264_stop_uses_bounded_nonblocking_status_drain():
  display, calls = recording_display()
  display._send_optional_command = (
    lambda command_id, name, fields=None, **kwargs:
      calls.append((command_id, name, fields, kwargs))
  )

  display.stop_h264_stream()

  assert [call[0] for call in calls] == [123, 122, 122]
  assert all(call[3]["no_ack_gap_s"] == 0.080 for call in calls)
  assert all(call[3]["no_ack_drain_attempts"] == 1 for call in calls)
