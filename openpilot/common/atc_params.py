def migrate_atc_params(params):
  """Replace the retired TBT-only mode without enabling ATC speed control."""
  if params.get_int("AutoTurnControl") == 4:
    # Write the add-on first so interruption leaves the migration retryable.
    if params.get("AutoTurnControlRightLaneChange") is None:
      params.put_bool("AutoTurnControlRightLaneChange", True)
    params.put_int("AutoTurnControl", 1)
