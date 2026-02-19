import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  t = 1.0
  current_lane_y = np.interp(t, ModelConstants.T_IDXS, current_lane.y)
  lane_y = np.interp(t, ModelConstants.T_IDXS, lane.y)
  distance_to_lane = abs(current_lane_y - lane_y)

  road_edge_y = np.interp(t, ModelConstants.T_IDXS, road_edge.y)
  distance_to_road_edge = abs(current_lane_y - road_edge_y)
  distance_to_road_edge_far = abs(current_lane_y - np.interp(2.0, ModelConstants.T_IDXS, road_edge.y))

  lane_valid = lane_prob > 0.5
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, distance_to_road_edge_far, lane_valid
