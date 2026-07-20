"""
로컬 타임존(화면 표시용) 해석 및 영구 저장 헬퍼.

GPS는 UTC(절대시각)만 주므로, 화면에 "그 나라의 로컬 시각"으로 표시하려면
타임존(IANA 이름)이 필요하다. 타임존은 아래 우선순위로 해석한다:

  1) app   - 캐롯 앱이 보낸 타임존 (carrot_serv) -- 가장 신뢰
  2) wifi  - 인터넷(IP 기반 지오로케이션)으로 받은 타임존
  3) gps   - GPS 경도 기반 근사 (오프라인 최후 수단, DST 미반영)

해석된 타임존은 /data/etc/localtime(시스템이 읽는 경로) 심볼릭링크로 적용하고,
이름/출처를 Params에 기록한다. 한 번 기록되면 오프라인 재부팅에도 유지된다.
낮은 우선순위 출처(gps)는 높은 출처(app/wifi)가 설정한 값을 덮어쓰지 않는다.
"""
import json
import os
import subprocess
import urllib.request

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

LOCALTIME_PATH = "/data/etc/localtime"
ZONEINFO_DIR = "/usr/share/zoneinfo"

# 숫자가 클수록 더 신뢰. ""(미설정)은 0.
SOURCE_PRIORITY = {"": 0, "gps": 1, "wifi": 2, "app": 3}


def _valid_zone(tz: str) -> bool:
  return bool(tz) and os.path.isfile(os.path.join(ZONEINFO_DIR, tz))


def current_source(params: Params | None = None) -> str:
  params = params or Params()
  return params.get("TimezoneSource") or ""


def apply_timezone(tz: str, source: str, params: Params | None = None) -> bool:
  """tz(IANA 이름)를 적용. source 우선순위가 기존보다 낮으면 무시(다운그레이드 방지)."""
  params = params or Params()
  if not _valid_zone(tz):
    cloudlog.error(f"timezone: invalid zone '{tz}' (source={source})")
    return False

  cur = current_source(params)
  if SOURCE_PRIORITY.get(source, 0) < SOURCE_PRIORITY.get(cur, 0):
    return False  # 더 신뢰도 높은 출처가 이미 설정함 -> 유지

  # 이미 같은 타임존이면 심링크 재작성 생략 (출처만 갱신)
  target = os.path.join(ZONEINFO_DIR, tz)
  already = os.path.islink(LOCALTIME_PATH) and os.path.realpath(LOCALTIME_PATH) == os.path.realpath(target)
  if not already:
    try:
      os.makedirs(os.path.dirname(LOCALTIME_PATH), exist_ok=True)
      subprocess.run(["sudo", "rm", "-f", LOCALTIME_PATH], check=True)
      subprocess.run(["sudo", "ln", "-s", target, LOCALTIME_PATH], check=True)
    except subprocess.CalledProcessError:
      cloudlog.exception("timezone: failed to set /data/etc/localtime")
      return False

  params.put("TimezoneName", tz)
  params.put("TimezoneSource", source)
  cloudlog.info(f"timezone: set to {tz} (source={source})")
  return True


def timezone_from_internet(timeout: float = 5.0) -> str | None:
  """[2순위] WiFi/인터넷 연결 시 IP 기반 지오로케이션으로 IANA 타임존을 받아온다.
     키 불필요한 무료 엔드포인트(ip-api.com). 실패 시 None."""
  try:
    req = urllib.request.Request(
      "http://ip-api.com/json/?fields=status,timezone",
      headers={"User-Agent": "openpilot-timed"},
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
      data = json.loads(resp.read().decode())
    if data.get("status") == "success":
      tz = data.get("timezone")
      return tz if _valid_zone(tz) else None
  except Exception:
    return None
  return None


def timezone_from_gps(longitude: float) -> str:
  """[3순위/최후수단] GPS 경도 기반 근사 타임존(Etc/GMT 고정 오프셋, DST 미반영).
     POSIX Etc/GMT 부호는 반대: Etc/GMT-9 == UTC+9."""
  offset = int(round(longitude / 15.0))
  offset = max(-12, min(14, offset))  # Etc/GMT+12 .. Etc/GMT-14 범위
  if offset == 0:
    return "Etc/GMT"
  return f"Etc/GMT{'-' if offset > 0 else '+'}{abs(offset)}"
