"""Suspend signal-specific control only on a fresh, explicit vNavi road class, else the fresh stock HDA LinkClass."""
import time

NAVI_TIMEOUT = 2.0
ROAD_TIMEOUT = 10.0  # v2 JSON stream's advertised stale_timeout_ms


def _navi_motorway(sm, now):
  # True/False only when carrotNavi can say; None when it cannot (not connected, stale, unknown class).
  try:
    if not (sm.seen['carrotNavi'] and sm.valid['carrotNavi'] and sm.alive['carrotNavi']):
      return None
    if not 0 <= now - sm.recv_time['carrotNavi'] <= NAVI_TIMEOUT:
      return None
    navi = sm['carrotNavi']
    lane = navi.laneCurrent
    if navi.schemaVersion != 1 or not navi.connected or not navi.sessionId:
      return None
    if navi.navigationStatus.meta.present and navi.navigationStatus.offRoute:
      return None
    # roadCategory defaults to zero in old messages and when the source omits it.
    # Neither case is evidence of a motorway.
    if not (lane.meta.present and lane.roadCategoryValid and lane.meta.receivedMonoTimeNanos > 0
            and 0 <= now - lane.meta.receivedMonoTimeNanos / 1e9 <= ROAD_TIMEOUT):
      return None
    return lane.roadCategory in (0, 1)
  except (AttributeError, KeyError, TypeError):
    return None


def signal_detection_suspended(sm, now=None, car_state=None):
  now = time.monotonic() if now is None else now
  navi = _navi_motorway(sm, now)
  if navi is not None:
    return navi
  try:
    # card publishes carState itself, so it passes CS; plannerd subscribes to it.
    return bool((sm['carState'] if car_state is None else car_state).naviMotorway)
  except (AttributeError, KeyError, TypeError):
    return False
