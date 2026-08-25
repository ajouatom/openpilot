from openpilot.selfdrive.carrot import cluster_autorun


def test_device_cluster_uses_standalone_carrot_navi_ipc():
  args = cluster_autorun._cluster_args(
    hud_mode=1,
    configured_encoder_mode=cluster_autorun.ENCODER_AUTO,
    active_encoder_mode=cluster_autorun.ENCODER_JPEG,
    core_mode=cluster_autorun.CORE_MODE_DEDICATED,
    priority=10,
  )

  assert args[:2] == ["--input", "live"]
  assert "--navi-overlay" not in args
  assert "--navi-publish-cereal" not in args
  assert args[args.index("--output") + 1] == "usb"


def test_usb_hot_unplug_is_recognized_through_wrapped_pipeline_errors():
  try:
    try:
      raise RuntimeError("TURZX USB display disconnected")
    except RuntimeError as exc:
      raise RuntimeError("H264 USB pipeline failed") from exc
  except RuntimeError as exc:
    assert cluster_autorun._is_usb_disconnect_error(exc)

  assert not cluster_autorun._is_usb_disconnect_error(RuntimeError("encoder failed"))
