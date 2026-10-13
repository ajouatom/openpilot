"""Front-radar orientation; kept independent of the lead-selection runtime."""


def front_radar_lateral(y_rel, yv_rel, source, flip):
  if flip and str(source) == "frontRadar":
    return -y_rel, -yv_rel
  return y_rel, yv_rel


def set_radar_track_flip(radar_data, flip):
  """Set the orientation on an owned publication copy, without double flipping."""
  flip = bool(flip)
  if bool(radar_data.radarTrackFlipped) != flip:
    # 표시 전용 점도 같은 게시 사본에서 한 번만 반전한다. 제어 목록에는 합치지 않는다.
    for group in (radar_data.points, getattr(radar_data, "displayPoints", ())):
      for point in group:
        point.yRel, point.yvRel = front_radar_lateral(point.yRel, point.yvRel, point.radarSource, True)
  radar_data.radarTrackFlipped = flip
