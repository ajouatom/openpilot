"""주행 리포트: 로그(rlog/qlog)만으로 주행 통계를 집계한다.

별도 AI 모델 없이, 로그에 기록된 carState / selfdriveState / onroadEvents 값과
아래 임계 상수만으로 리포트를 생성한다. 세그먼트 폴더(각 60초) 안의 rlog(100Hz)를
우선 사용하고, 없으면 qlog(10Hz, 영구 보존)로 대체한다.
"""

import bz2
import math
import os
import time
from typing import Any

import zstandard as zstd

from openpilot.cereal import log as capnp_log

from ...config import DASHCAM_ROOT
from .catalog import build_routes
from .paths import route_name, segment_index

# ── 급가감속 / 과가감속 판정 임계 (m/s²) ───────────────────────────
# 정상: 컴포트 캡 이내(자유순항 상한 ~1.4~1.5, COMFORT_BRAKE ~2.0) → 집계 안 함
# 과(불편): 정상범위를 벗어나 체감 불편하나 이상치는 아님
# 급(이상치): 액추에이터 한계 부근(Hyundai ACCEL_MAX 2.5 / -3.0 이상 제동)
OVER_ACCEL = 1.5    # 과가속 하한
HARD_ACCEL = 2.5    # 급가속 하한
OVER_DECEL = -2.0   # 과감속(이보다 작아야 함)
HARD_DECEL = -3.0   # 급감속(이보다 작아야 함)

# 하드코너링(횡가속). yawRate가 0인 각도조향 차량이 많아, 조향각+차량 지오메트리로
# 자전거 모델 추정: a_lat = vEgo² · tan(steerAngle/steerRatio) / wheelbase.
HARD_LAT_ACCEL = 3.0       # m/s²
CORNER_MIN_SPEED = 5.5     # m/s (~20km/h)
DEFAULT_STEER_RATIO = 14.0
DEFAULT_WHEELBASE = 2.8    # m

EVENT_MIN_GAP_S = 1.5    # 이 간격 미만의 연속 excursion은 하나로 병합(중복 방지)
DT_CAP_S = 0.5           # 세그먼트 내 샘플 간격 상한(로그 순단 시 시간 왜곡 방지)
MOVING_SPEED = 0.3       # m/s 이상이면 '주행 중'으로 간주
MAX_EVENTS_PER_CATEGORY = 60  # 응답 payload 상한

DRIVING_GEARS = {"drive", "low", "reverse", "sport", "eco", "manumatic"}

RLOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog")
QLOG_NAMES = ("qlog.zst", "qlog.bz2", "qlog")

# onroadEvents 경고 카테고리 → EventName 집합
WARN_CATEGORIES = {
  "fcw": {"fcw", "stockFcw"},
  "ldw": {"ldw"},
  "driverDistracted": {"preDriverDistracted", "promptDriverDistracted", "driverDistracted",
                       "promptDriverUnresponsive", "driverUnresponsive", "tooDistracted"},
}


def _decompress(path: str) -> bytes:
  with open(path, "rb") as f:
    dat = f.read()
  if path.endswith(".bz2") or dat[:4] == b"BZh9":
    return bz2.decompress(dat)
  if path.endswith(".zst") or dat[:4] == b"\x28\xB5\x2F\xFD":
    return zstd.ZstdDecompressor().stream_reader(dat).read()
  return dat


def _events(path: str):
  dat = _decompress(path)
  yield from capnp_log.Event.read_multiple_bytes(dat)


def _pick_log_file(segment_dir_path: str, prefer_rlog: bool) -> tuple[str, str] | None:
  order = (RLOG_NAMES + QLOG_NAMES) if prefer_rlog else (QLOG_NAMES + RLOG_NAMES)
  for name in order:
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, ("rlog" if name.startswith("rlog") else "qlog")
  return None


def _fmt_hms(seconds: float) -> str:
  s = max(0, round(seconds))
  return f"{s // 3600:02d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


def _fmt_ms(seconds: float) -> str:
  s = max(0, round(seconds))
  return f"{s // 60:02d}:{s % 60:02d}"


def _fmt_clock(epoch: float) -> str:
  if not epoch or epoch <= 0:
    return "-"
  return time.strftime("%H:%M:%S", time.localtime(epoch))


def _stamp(items: list[dict], key: str) -> list[dict]:
  out = []
  for it in items[:MAX_EVENTS_PER_CATEGORY]:
    row = {"clock": _fmt_clock(it["time"])}
    if key in it:
      row[key] = it[key]
    out.append(row)
  return out


class _Excursions:
  """정상범위를 벗어난 연속 구간을 모으고, 근접 구간을 병합한다.

  accel=True 이면 value >= over_thr 구간, False 이면 value <= over_thr 구간을 잡는다.
  각 병합 구간은 피크값으로 급/과 한 등급만 부여한다(중복 집계 없음).
  """

  def __init__(self, accel: bool, over_thr: float, hard_thr: float):
    self.accel = accel
    self.over_thr = over_thr
    self.hard_thr = hard_thr
    self.raw: list[dict[str, float]] = []
    self._cur: dict[str, float] | None = None

  def _in_band(self, v: float) -> bool:
    return v >= self.over_thr if self.accel else v <= self.over_thr

  def _peak(self, a: float, b: float) -> float:
    return max(a, b) if self.accel else min(a, b)

  def feed(self, t_wall: float, v: float):
    if self._in_band(v):
      if self._cur is None:
        self._cur = {"start": t_wall, "end": t_wall, "peak": v}
      else:
        self._cur["end"] = t_wall
        self._cur["peak"] = self._peak(self._cur["peak"], v)
    elif self._cur is not None:
      self.raw.append(self._cur)
      self._cur = None

  def events(self) -> tuple[list[dict], list[dict]]:
    """병합 후 (급 목록, 과 목록) 반환. 각 원소: {time(epoch), peak}"""
    if self._cur is not None:
      self.raw.append(self._cur)
      self._cur = None
    merged: list[dict[str, float]] = []
    for ex in self.raw:
      if merged and (ex["start"] - merged[-1]["end"]) < EVENT_MIN_GAP_S:
        merged[-1]["end"] = ex["end"]
        merged[-1]["peak"] = self._peak(merged[-1]["peak"], ex["peak"])
      else:
        merged.append(dict(ex))
    hard, over = [], []
    for m in merged:
      item = {"time": m["start"], "peak": round(m["peak"], 2)}
      is_hard = m["peak"] >= self.hard_thr if self.accel else m["peak"] <= self.hard_thr
      (hard if is_hard else over).append(item)
    return hard, over


def _disengage_cause(cs, last_cancel_wall: float, now_wall: float) -> str:
  try:
    if now_wall - last_cancel_wall <= 1.0:
      return "button"
    if cs.brakePressed:
      return "brake"
    if cs.steeringPressed:
      return "steer"
    if cs.gasPressed:
      return "gas"
  except Exception:
    pass
  return "other"


def build_route_report(route: str, prefer_rlog: bool = True) -> dict[str, Any]:
  # 세그먼트("route--3")가 넘어오면 route 이름으로 정규화.
  parts = route.split("--")
  if len(parts) > 2 and parts[-1].isdigit():
    route = route_name(route)
  routes = build_routes()
  entry = next((r for r in routes if r.get("route") == route), None)
  if not entry:
    return {"ok": False, "error": "route not found"}

  segments = sorted(entry.get("segmentFolders", []), key=lambda s: (segment_index(s), s))

  # 누적기 (라우트 전역)
  auto_enabled_t = auto_active_t = manual_t = 0.0
  manual_gas_t = manual_brake_t = stop_t = steer_ovr_t = 0.0
  total_dist = auto_dist = max_speed = max_lat = 0.0
  stop_count = disengage_count = steer_ovr_count = corner_count = 0
  disengage_causes: dict[str, int] = {}
  disengage_events: list[dict] = []
  warn_counts = {"fcw": 0, "ldw": 0, "driverDistracted": 0}
  warn_prev = {k: False for k in warn_counts}

  accel_exc = _Excursions(True, OVER_ACCEL, HARD_ACCEL)
  decel_exc = _Excursions(False, OVER_DECEL, HARD_DECEL)

  # 세그먼트 경계를 넘어 유지되는 상태
  prev_enabled = prev_standstill = prev_steer_ovr = prev_corner = False
  last_cancel_wall = -9e9
  first_wall = last_wall = 0.0
  source_kind = ""
  used_log = False
  steer_ratio = DEFAULT_STEER_RATIO
  wheelbase = DEFAULT_WHEELBASE

  for seg in segments:
    picked = _pick_log_file(os.path.join(DASHCAM_ROOT, seg), prefer_rlog)
    if not picked:
      continue
    path, kind = picked
    source_kind = source_kind or kind

    anchor_mono = anchor_wall = 0.0
    prev_cs_mono: float | None = None
    last_cs = None
    cur_enabled = prev_enabled
    cur_active = False

    try:
      for ev in _events(path):
        which = ev.which()
        mono = ev.logMonoTime

        if which == "initData":
          try:
            wt = ev.initData.wallTimeNanos
            if wt:
              anchor_mono, anchor_wall = mono, wt / 1e9
          except Exception:
            pass
          continue

        if anchor_wall == 0.0 and which == "clocks":
          try:
            wt = ev.clocks.wallTimeNanos
            if wt:
              anchor_mono, anchor_wall = mono, wt / 1e9
          except Exception:
            pass

        t_wall = (anchor_wall + (mono - anchor_mono) / 1e9) if anchor_wall else 0.0

        if which == "selfdriveState":
          ss = ev.selfdriveState
          cur_enabled, cur_active = ss.enabled, ss.active
          if prev_enabled and not cur_enabled and t_wall:
            disengage_count += 1
            reason = _disengage_cause(last_cs, last_cancel_wall, t_wall) if last_cs else "other"
            disengage_causes[reason] = disengage_causes.get(reason, 0) + 1
            if len(disengage_events) < MAX_EVENTS_PER_CATEGORY:
              disengage_events.append({"time": t_wall, "cause": reason})
          prev_enabled = cur_enabled
          continue

        if which == "onroadEvents":
          present = set()
          try:
            for e in ev.onroadEvents:
              present.add(str(e.name))
          except Exception:
            pass
          for cat, names in WARN_CATEGORIES.items():
            now_on = bool(present & names)
            if now_on and not warn_prev[cat]:
              warn_counts[cat] += 1
            warn_prev[cat] = now_on
          continue

        if which == "carParams":
          try:
            if ev.carParams.steerRatio > 0:
              steer_ratio = ev.carParams.steerRatio
            if ev.carParams.wheelbase > 0:
              wheelbase = ev.carParams.wheelbase
          except Exception:
            pass
          continue

        if which != "carState":
          continue

        cs = ev.carState
        last_cs = cs

        try:
          for be in cs.buttonEvents:
            if str(be.type) == "cancel" and be.pressed:
              last_cancel_wall = t_wall
        except Exception:
          pass

        if not t_wall:
          continue
        if first_wall == 0.0:
          first_wall = t_wall
        last_wall = t_wall

        dt = 0.0
        if prev_cs_mono is not None:
          dt = min(DT_CAP_S, max(0.0, (mono - prev_cs_mono) / 1e9))
        prev_cs_mono = mono

        v, a = cs.vEgo, cs.aEgo
        gear = str(cs.gearShifter)
        is_driving = gear in DRIVING_GEARS or v > MOVING_SPEED
        max_speed = max(max_speed, v)

        accel_exc.feed(t_wall, a)
        decel_exc.feed(t_wall, a)

        # 하드코너링(횡가속) rising-edge 카운트.
        # yawRate가 채워지면 그대로, 아니면 조향각 자전거 모델로 추정.
        cornering = False
        if v >= CORNER_MIN_SPEED:
          if abs(cs.yawRate) > 1e-3:
            lat = abs(v * cs.yawRate)
          else:
            wheel_rad = math.radians(cs.steeringAngleDeg / steer_ratio)
            lat = abs(v * v * math.tan(wheel_rad) / wheelbase)
          max_lat = max(max_lat, lat)
          cornering = lat >= HARD_LAT_ACCEL
        if cornering and not prev_corner:
          corner_count += 1
        prev_corner = cornering

        if cs.standstill and not prev_standstill:
          stop_count += 1
        prev_standstill = cs.standstill
        if cs.standstill:
          stop_t += dt

        used_log = True
        if cur_enabled:
          auto_enabled_t += dt
          auto_dist += v * dt
          if cur_active:
            auto_active_t += dt
          if cs.steeringPressed and not prev_steer_ovr:
            steer_ovr_count += 1
          if cs.steeringPressed:
            steer_ovr_t += dt
          prev_steer_ovr = cs.steeringPressed
        else:
          prev_steer_ovr = False
          if is_driving:
            manual_t += dt
            if cs.gasPressed:
              manual_gas_t += dt
            if cs.brakePressed:
              manual_brake_t += dt
        if is_driving:
          total_dist += v * dt
    except Exception:
      continue

  hard_accel, over_accel = accel_exc.events()
  hard_decel, over_decel = decel_exc.events()

  total_t = auto_enabled_t + manual_t
  avg_speed = (total_dist / total_t) if total_t > 0 else 0.0
  kmh = lambda ms: round(ms * 3.6, 1)  # noqa: E731

  return {
    "ok": True,
    "route": route,
    "title": entry.get("title") or route,
    "source": source_kind,
    "segments": len(segments),
    "hasData": used_log,
    "time": {
      "totalHms": _fmt_hms(total_t),
      "autoEnabledHms": _fmt_hms(auto_enabled_t),
      "autoActiveHms": _fmt_hms(auto_active_t),
      "manualHms": _fmt_hms(manual_t),
      "manualGasMs": _fmt_ms(manual_gas_t),
      "manualBrakeMs": _fmt_ms(manual_brake_t),
      "stopMs": _fmt_ms(stop_t),
      "steerOverrideMs": _fmt_ms(steer_ovr_t),
      "startClock": _fmt_clock(first_wall),
      "endClock": _fmt_clock(last_wall),
      "autoRatioPct": round(100.0 * auto_enabled_t / total_t, 1) if total_t > 0 else 0.0,
      "manualRatioPct": round(100.0 * manual_t / total_t, 1) if total_t > 0 else 0.0,
      # 구성비율 막대그래프용 원시 초 값
      "totalSec": round(total_t, 1),
      "autoSec": round(auto_enabled_t, 1),
      "manualSec": round(manual_t, 1),
      "manualGasSec": round(manual_gas_t, 1),
      "manualBrakeSec": round(manual_brake_t, 1),
    },
    "distance": {
      "totalKm": round(total_dist / 1000.0, 2),
      "autoKm": round(auto_dist / 1000.0, 2),
      "manualKm": round(max(0.0, total_dist - auto_dist) / 1000.0, 2),
      "avgSpeedKmh": kmh(avg_speed),
      "maxSpeedKmh": kmh(max_speed),
    },
    "events": {
      "hardAccel": {"count": len(hard_accel), "items": _stamp(hard_accel, "peak")},
      "overAccel": {"count": len(over_accel), "items": _stamp(over_accel, "peak")},
      "hardDecel": {"count": len(hard_decel), "items": _stamp(hard_decel, "peak")},
      "overDecel": {"count": len(over_decel), "items": _stamp(over_decel, "peak")},
    },
    "extras": {
      "disengageCount": disengage_count,
      "disengageCauses": disengage_causes,
      "disengageItems": _stamp(disengage_events, "cause"),
      "stopCount": stop_count,
      "steerOverrideCount": steer_ovr_count,
      "cornerCount": corner_count,
      "maxLatAccel": round(max_lat, 2),
      "warnCounts": warn_counts,
    },
  }
