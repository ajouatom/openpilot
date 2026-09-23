"""Front-radar orientation; kept independent of the lead-selection runtime."""


def front_radar_lateral(y_rel, yv_rel, source, flip):
  if flip and str(source) == "frontRadar":
    return -y_rel, -yv_rel
  return y_rel, yv_rel


def set_radar_track_flip(radar_data, flip):
  """Set the orientation on an owned publication copy, without double flipping."""
  flip = bool(flip)
  if bool(radar_data.radarTrackFlipped) != flip:
    for point in radar_data.points:
      point.yRel, point.yvRel = front_radar_lateral(point.yRel, point.yvRel, point.radarSource, True)
  radar_data.radarTrackFlipped = flip
