def get_camera_packets(use_wide_camera: bool) -> list[str]:
  # Driver-camera/model health is handled by the always-running DM2 fallback.
  # Road-camera validity is independent and keeps the existing disable policy.
  packets = ["roadCameraState"]
  if use_wide_camera:
    packets.append("wideRoadCameraState")
  return packets
