from openpilot.selfdrive.carrot import cluster_autorun


def test_wifi_recovery_skips_only_shared_usb_settling_delay(monkeypatch):
  clock = [0.0]
  class FakeParams:
    def get_bool(self, key):
      if key == cluster_autorun.USBGPU_LOADING_PARAM:
        return clock[0] < 1.0
      if key == cluster_autorun.USBGPU_ACTIVE_PARAM:
        return clock[0] >= 1.0
      return False

  monkeypatch.setattr(cluster_autorun, '_usbgpu_startup_expected', lambda: True)
  monkeypatch.setattr(cluster_autorun.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(cluster_autorun.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
  cluster_autorun._wait_for_usbgpu_startup(FakeParams(), shared_usb=False)
  assert 1.0 <= clock[0] < 1.2  # Model loading is still respected.
  clock[0] = 0.0
  cluster_autorun._wait_for_usbgpu_startup(FakeParams())
  assert 11.0 <= clock[0] < 11.2  # Wired USB protection is unchanged.


def test_device_cluster_uses_standalone_carrot_navi_ipc():
  args = cluster_autorun._cluster_args(
    hud_mode=1,
    configured_encoder_mode=cluster_autorun.ENCODER_AUTO,
    active_encoder_mode=cluster_autorun.ENCODER_JPEG,
  )

  assert args[:2] == ["--input", "live"]
  assert "--navi-overlay" not in args
  assert "--navi-publish-cereal" not in args
  assert args[args.index("--output") + 1] == "usb"


def test_device_cluster_caps_external_display_fps_while_egpu_is_active():
  args = cluster_autorun._cluster_args(
    hud_mode=1,
    configured_encoder_mode=cluster_autorun.ENCODER_AUTO,
    active_encoder_mode=cluster_autorun.ENCODER_HARDWARE,
    usbgpu_active=True,
  )

  assert args[args.index("--usb-display-fps") + 1] == str(cluster_autorun.USBGPU_DISPLAY_FPS)
  assert args[args.index("--fps") + 1] == str(cluster_autorun.USBGPU_DISPLAY_FPS)


def test_usb_hot_unplug_is_recognized_through_wrapped_pipeline_errors():
  try:
    try:
      raise RuntimeError("TURZX USB display disconnected")
    except RuntimeError as exc:
      raise RuntimeError("H264 USB pipeline failed") from exc
  except RuntimeError as exc:
    assert cluster_autorun._is_usb_disconnect_error(exc)

  assert not cluster_autorun._is_usb_disconnect_error(RuntimeError("encoder failed"))
