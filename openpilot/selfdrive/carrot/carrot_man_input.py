import time


CARROT_MAN_TIMEOUT = 1.0


def get_carrot_man(sm):
  """Return a received navigation cap, never the zero-initialized SubMaster value.

  carrotMan is registered as an on-demand service, so alive/valid alone do not
  prove that it has published anything or that its last message is still fresh.
  Its publisher normally runs at 20 Hz. Navigation caps must be positive;
  traffic/lead stopping is handled independently by the longitudinal planner.
  """
  service = 'carrotMan'
  if not (sm.seen[service] and sm.alive[service] and sm.valid[service]):
    return None
  age = time.monotonic() - sm.recv_time[service]
  if not 0.0 <= age <= CARROT_MAN_TIMEOUT:
    return None
  carrot_man = sm[service]
  return carrot_man if 0 < carrot_man.desiredSpeed <= 250 else None
