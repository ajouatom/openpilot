"""Suspend signal-specific control only on a fresh, explicit vNavi road class."""
import time

NAVI_TIMEOUT = 2.0
ROAD_TIMEOUT = 10.0  # v2 JSON stream's advertised stale_timeout_ms


def signal_detection_suspended(sm, now=None):
  now = time.monotonic() if now is None else now
  try:
    if not (sm.seen['carrotNavi'] and sm.valid['carrotNavi'] and sm.alive['carrotNavi']):
      return False
    if not 0 <= now - sm.recv_time['carrotNavi'] <= NAVI_TIMEOUT:
      return False
    navi = sm['carrotNavi']
    lane = navi.laneCurrent
    if navi.schemaVersion != 1 or not navi.connected or not navi.sessionId:
      return False
    if navi.navigationStatus.meta.present and navi.navigationStatus.offRoute:
      return False
    # roadCategory defaults to zero in old messages and when the source omits it.
    # Neither case is evidence of a motorway.
    return bool(lane.meta.present and lane.roadCategoryValid and lane.roadCategory in (0, 1)
                and lane.meta.receivedMonoTimeNanos > 0
                and 0 <= now - lane.meta.receivedMonoTimeNanos / 1e9 <= ROAD_TIMEOUT)
  except (AttributeError, KeyError, TypeError):
    return False
