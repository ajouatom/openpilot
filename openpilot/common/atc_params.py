def right_lane_change_distance(params):
  """Read metres, including an enabled value restored from the former toggle."""
  distance = params.get_int("AutoTurnControlRightLaneChange")
  if distance == 1:
    return 250
  return distance if 0 <= distance <= 500 else 0


def migrate_atc_params(params):
  """Convert the retired mode and the former add-on toggle before defaults."""
  key = "AutoTurnControlRightLaneChange"
  if params.get_int("AutoTurnControl") == 4:
    # Write the add-on first so interruption leaves the migration retryable.
    if params.get(key) is None:
      params.put_int(key, 250)
    params.put_int("AutoTurnControl", 1)
  if params.get_int(key) == 1:
    params.put_int(key, 250)
