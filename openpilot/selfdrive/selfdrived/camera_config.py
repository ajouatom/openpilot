def get_camera_packets(use_wide_camera: bool) -> list[str]:
  # DM2 handles driver-camera failures through interaction monitoring when DM is enabled.
  # Road-camera validity is independent and keeps the existing disable policy.
  packets = ["roadCameraState"]
  if use_wide_camera:
    packets.append("wideRoadCameraState")
  return packets
