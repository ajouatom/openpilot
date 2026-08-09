def get_camera_packets(use_wide_camera: bool, disable_dm: int, simulation: bool = False) -> list[str]:
  packets = ["roadCameraState"]
  if disable_dm == 0 or simulation:
    packets.append("driverCameraState")
  if use_wide_camera:
    packets.append("wideRoadCameraState")
  return packets
