#!/usr/bin/env python3
import datetime
import subprocess
import time
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service
from openpilot.system.timezone_helper import (
  SOURCE_PRIORITY, current_source, apply_timezone,
  timezone_from_internet, timezone_from_gps,
)


def set_time(new_epoch):
  # new_epoch: GPS에서 받은 UNIX epoch(UTC 초). 화면 시계(localtime)는 시스템시계 + 타임존을
  # 읽으므로, 시스템시계를 GPS UTC로만 정확히 맞추면 로컬 시각으로 표시된다.
  diff = abs(time.time() - new_epoch)
  if diff < 10:
    cloudlog.debug(f"Time diff too small: {diff:.1f}s")
    return

  cloudlog.info(f"Setting system time from GPS (diff {diff:.1f}s)")
  try:
    # '@<epoch>'는 절대 UTC 순간을 지정 -> 타임존 이중변환 버그 없음.
    # sudo 필수: 일반 권한의 `date -s`는 조용히 실패한다.
    subprocess.run(["sudo", "date", "-s", f"@{int(new_epoch)}"], check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])

  last_tz_attempt = 0.0
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_ok = (sm.updated[gps_location_service]
              and (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) <= 2.0
              and gps.hasFix)

    # 로컬 타임존 해석(앱 미사용자용). 아직 신뢰 출처(app/wifi)로 설정되지 않았을 때만 시도.
    # 타임존이 전혀 없으면 빠르게(30s) 재시도, GPS 폴백이 이미 있으면 느리게(5min) -
    # 오프라인에서 인터넷 호출(최대 timeout)이 반복적으로 루프를 막지 않게 한다.
    cur_prio = SOURCE_PRIORITY.get(current_source(params), 0)
    if cur_prio < SOURCE_PRIORITY["wifi"]:
      retry_interval = 30.0 if cur_prio == 0 else 300.0
      if time.monotonic() - last_tz_attempt > retry_interval:
        last_tz_attempt = time.monotonic()
        tz = timezone_from_internet()        # [2순위] WiFi/인터넷
        if tz is not None:
          apply_timezone(tz, "wifi", params)
        elif gps_ok:                         # [3순위] GPS 경도 근사 (최후수단)
          apply_timezone(timezone_from_gps(gps.longitude), "gps", params)

    # 시스템 시계(절대 UTC)는 GPS fix가 있어야 설정 가능
    if not gps_ok:
      continue
    gps_epoch = gps.unixTimestampMillis / 1000.
    if datetime.datetime.fromtimestamp(gps_epoch) < min_date():
      continue

    set_time(gps_epoch)
    time.sleep(10)

if __name__ == "__main__":
  main()
