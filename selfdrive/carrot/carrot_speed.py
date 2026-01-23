# -*- coding: utf-8 -*-
"""
CarrotSpeedTable v6 (Params backend, JSON+gzip, 1e-4° grid, 8 buckets)  [EVENT-BASED, NO TS]
- KEY: "CarrotSpeedTable"
- FORMAT(v6):
  {
    "format":"v6",
    "dir_buckets":8,
    "cells":{"gy,gx":[eid,...]},
    "events":{
       "eid":{
         "cy": float, "cx": float,   # center grid coords (float, averaging)
         "b": int,                  # heading bucket (대표)
         "v": float,                # speed (signed)
         "n": int                   # merge count
       }, ...
    }
  }
- v5 데이터는 로드 시 삭제(초기화)
- 시간정보(ts) 저장/사용 안 함
"""

import json, math, threading, time, gzip
from typing import Optional, Tuple, Dict, List
from openpilot.common.params import Params


def quantize_1e4(lat: float, lon: float) -> Tuple[int, int]:
  gy = int(math.floor(lat * 1e4 + 0.5))
  gx = int(math.floor(lon * 1e4 + 0.5))
  return gy, gx

def heading_to_bucket(heading_deg: float, buckets: int = 8) -> int:
  step = 360.0 / float(buckets)
  i = int((heading_deg % 360.0) // step)
  if i < 0: return 0
  if i >= buckets: return buckets - 1
  return i

def bucket_diff(a: int, b: int, buckets: int = 8) -> int:
  d = abs(int(a) - int(b)) % buckets
  return min(d, buckets - d)

def project_point(lat: float, lon: float, heading_deg: float, distance_m: float) -> Tuple[float, float]:
  if distance_m <= 0.0:
    return lat, lon
  R = 6_371_000.0
  h = math.radians(heading_deg)
  dlat = (distance_m * math.cos(h)) / R
  dlon = (distance_m * math.sin(h)) / (R * math.cos(math.radians(lat)))
  return lat + math.degrees(dlat), lon + math.degrees(dlon)

def _is_gzip(data: bytes) -> bool:
  return len(data) >= 2 and data[0] == 0x1F and data[1] == 0x8B


class CarrotSpeed:
  KEY = "CarrotSpeedTable"

  def __init__(self,
               neighbor_ring: int = 1,
               neighbor_old_threshold_s: int = 120,  # v6: 미사용(호환)
               use_gzip: bool = True,
               gzip_level: int = 5):
    self.buckets = 8

    # 기존 인터페이스 호환 파라미터
    self.neighbor_ring = max(0, int(neighbor_ring))
    self.neighbor_old_threshold_s = int(neighbor_old_threshold_s)  # v6: 미사용
    self.use_gzip = bool(use_gzip)
    self.gzip_level = int(gzip_level)

    # v6 이벤트/머지 파라미터
    self.merge_ring = 2
    self.bucket_tol = 1

    self._lock = threading.RLock()
    self._params = Params()
    self._dirty = False
    self._last_save = 0

    # v6 data
    self._events: Dict[str, Dict] = {}
    self._cells: Dict[Tuple[int, int], List[str]] = {}

    self._load_from_params_if_exists()

    # invalidate용(시간창 없이 "마지막으로 반환한 이벤트"만 기억)
    self._last_hit_eid: Optional[str] = None

  # ---------------- internals ----------------

  def _now(self) -> int:
    return int(time.time())

  def _make_eid(self, gy: int, gx: int, b: int, nonce: int) -> str:
    # ts를 쓰지 않으므로 nonce로 충돌만 피함
    return f"{gy},{gx},{b},{nonce}"

  def _index_add(self, eid: str, gy: int, gx: int) -> None:
    lst = self._cells.get((gy, gx))
    if lst is None:
      self._cells[(gy, gx)] = [eid]
    else:
      if eid not in lst:
        lst.append(eid)

  def _index_remove(self, eid: str, gy: int, gx: int) -> None:
    lst = self._cells.get((gy, gx))
    if not lst:
      return
    try:
      lst.remove(eid)
    except ValueError:
      pass
    if not lst:
      self._cells.pop((gy, gx), None)

  def _near_ids_in_ring(self, gy0: int, gx0: int, ring: int) -> List[str]:
    out: List[str] = []
    r = max(0, int(ring))
    for dy in range(-r, r + 1):
      for dx in range(-r, r + 1):
        lst = self._cells.get((gy0 + dy, gx0 + dx))
        if lst:
          out.extend(lst)
    return out

  def _grid_dist(self, gy: int, gx: int, cy: float, cx: float) -> float:
    return abs(float(gy) - float(cy)) + abs(float(gx) - float(cx))

  # ---------------- public ----------------
  def export_cells_around_with_here(self, lat: float, lon: float,
                                    heading_deg: float,
                                    ring: int = 1, max_points: int = 64,
                                    lateral_m: float = 6.0) -> Tuple[str, float]:
    gy0, gx0 = quantize_1e4(lat, lon)
    b0 = heading_to_bucket(heading_deg, self.buckets)
    pts = []

    def _pick_speed_at(lat0: float, lon0: float) -> float:
      """해당 좌표의 셀(quantize)에서만(speed control용) speed 1개 선택. 없으면 0."""
      gy, gx = quantize_1e4(lat0, lon0)
      best_v = 0.0
      best_d = 1e18
      best_bd = 999

      cand_ids = self._near_ids_in_ring(gy, gx, 0)  # ✅ 현재 셀만
      for eid in cand_ids:
        ev = self._events.get(eid)
        if not ev:
          continue
        bd = bucket_diff(b0, int(ev["b"]), self.buckets)
        if bd > self.bucket_tol:
          continue

        dd = self._grid_dist(gy, gx, float(ev["cy"]), float(ev["cx"]))
        if (dd < best_d) or (dd == best_d and bd < best_bd):
          best_d = dd
          best_bd = bd
          best_v = float(ev["v"])

      return best_v

    with self._lock:
      # 1) 표시용 pts (주변 ring)
      cand_ids = self._near_ids_in_ring(gy0, gx0, ring)
      seen = set()
      for eid in cand_ids:
        if eid in seen:
          continue
        seen.add(eid)

        ev = self._events.get(eid)
        if not ev:
          continue
        if bucket_diff(b0, int(ev["b"]), self.buckets) > self.bucket_tol:
          continue

        # 표시용(기존 그대로)
        cell_lat = (float(ev["cy"]) + 0.5) * 1e-4
        cell_lon = (float(ev["cx"]) + 0.5) * 1e-4
        pts.append([cell_lat, cell_lon, float(ev["v"])])
        if len(pts) >= max_points:
          break

      # 2) speed: 현재 위치 셀
      speed = _pick_speed_at(lat, lon)

      # 3) speed가 0이면 좌/우 셀(heading 기준 lateral)에서만 가져오기
      if speed == 0.0 and lateral_m > 0.0:
        # 좌 먼저, 그 다음 우
        yL, xL = project_point(lat, lon, heading_deg - 90.0, lateral_m)
        sL = _pick_speed_at(yL, xL)
        if sL != 0.0:
          speed = sL
        else:
          yR, xR = project_point(lat, lon, heading_deg + 90.0, lateral_m)
          sR = _pick_speed_at(yR, xR)
          if sR != 0.0:
            speed = sR

    return {"pts": pts}, float(speed)

  def export_cells_around1(self, lat: float, lon: float,
                          heading_deg: float,
                          ring: int = 1, max_points: int = 64) -> str:
    gy0, gx0 = quantize_1e4(lat, lon)
    b0 = heading_to_bucket(heading_deg, self.buckets)
    pts = []

    with self._lock:
      cand_ids = self._near_ids_in_ring(gy0, gx0, ring)
      seen = set()
      for eid in cand_ids:
        if eid in seen:
          continue
        seen.add(eid)
        ev = self._events.get(eid)
        if not ev:
          continue
        if bucket_diff(b0, int(ev["b"]), self.buckets) > self.bucket_tol:
          continue
        cell_lat = (float(ev["cy"]) + 0.5) * 1e-4
        cell_lon = (float(ev["cx"]) + 0.5) * 1e-4
        pts.append([cell_lat, cell_lon, float(ev["v"])])
        if len(pts) >= max_points:
          break

    return json.dumps({"pts": pts}, separators=(",", ":"))

  def add_sample(self, lat: float, lon: float, heading_deg: float, speed_signed: float):
    v_in = round(float(speed_signed), 1)
    if v_in == 0.0:
      return

    gy, gx = quantize_1e4(lat, lon)
    b = heading_to_bucket(heading_deg, self.buckets)

    with self._lock:
      cand_ids = self._near_ids_in_ring(gy, gx, self.merge_ring)

      best_eid = None
      best_score = 1e18

      for eid in cand_ids:
        ev = self._events.get(eid)
        if not ev:
          continue
        if bucket_diff(b, int(ev["b"]), self.buckets) > self.bucket_tol:
          continue

        d = self._grid_dist(gy, gx, float(ev["cy"]), float(ev["cx"]))
        # 거리 우선. (speed 차이는 굳이 안 넣어도 됨)
        score = d
        if score < best_score:
          best_score = score
          best_eid = eid

      if best_eid is None:
        eid = self._make_eid(gy, gx, b, self._now())
        self._events[eid] = {
          "cy": float(gy),
          "cx": float(gx),
          "b": int(b),
          "v": float(v_in),
          "n": 1,
        }
        self._index_add(eid, gy, gx)
        self._dirty = True
        return

      # merge (중심만 수렴)
      ev = self._events[best_eid]
      n = int(ev.get("n", 1))

      cy_old, cx_old = float(ev["cy"]), float(ev["cx"])
      cy_new = (cy_old * n + float(gy)) / (n + 1)
      cx_new = (cx_old * n + float(gx)) / (n + 1)

      old_cell = (int(round(cy_old)), int(round(cx_old)))
      new_cell = (int(round(cy_new)), int(round(cx_new)))
      if old_cell != new_cell:
        self._index_remove(best_eid, old_cell[0], old_cell[1])
        self._index_add(best_eid, new_cell[0], new_cell[1])

      ev["cy"], ev["cx"] = cy_new, cx_new
      ev["n"] = n + 1

      # ✅ 핵심: 값은 항상 최신으로 overwrite
      ev["v"] = float(v_in)

      # (선택) 대표 방향도 최신으로 갱신하고 싶으면 아래 1줄 켜도 됨
      # ev["b"] = int(b)

      self._dirty = True

  def invalidate_last_hit(self, window_s: float = 2.0, action: str = "clear") -> bool:
    """
    ts 저장을 안 하므로 window_s는 무시.
    - clear: 이벤트 삭제
    - age_bump: 이벤트 완화(0에 가깝게)
    """
    eid = self._last_hit_eid
    if not eid:
      return False

    with self._lock:
      ev = self._events.get(eid)
      if not ev:
        return False

      if action == "clear":
        cell = (int(round(float(ev["cy"]))), int(round(float(ev["cx"]))))
        self._index_remove(eid, cell[0], cell[1])
        self._events.pop(eid, None)
      else:
        v = float(ev["v"])
        if v < 0.0:
          ev["v"] = min(0.0, v + 1.0)
        else:
          ev["v"] = v + 1.0

      self._dirty = True
      return True

  def maybe_save(self, interval_s: int = 60) -> None:
    now = self._now()
    if (not self._dirty) or (now - self._last_save < int(interval_s)):
      return
    self.save()

  def save(self) -> None:
    payload = self._encode_payload()
    self._params.put_nonblocking(self.KEY, payload)
    self._last_save = self._now()
    self._dirty = False

  def close(self) -> None:
    try:
      if self._dirty:
        self.save()
    except Exception:
      pass

  # ---------------- serialization ----------------

  def _encode_payload(self) -> bytes:
    with self._lock:
      obj = {
        "format": "v6",
        "dir_buckets": self.buckets,
        "cells": {f"{gy},{gx}": lst for (gy, gx), lst in self._cells.items()},
        "events": self._events,
      }
      raw = json.dumps(obj, separators=(",", ":")).encode("utf-8")
      return gzip.compress(raw, compresslevel=self.gzip_level) if self.use_gzip else raw

  def _load_from_params_if_exists(self) -> None:
    raw = self._params.get(self.KEY)
    if not raw:
      return
    try:
      b = raw
      if _is_gzip(b):
        b = gzip.decompress(b)
      data = json.loads(b.decode("utf-8"))

      if data.get("format") != "v6":
        # v5 포함 모두 삭제
        try:
          self._params.remove(self.KEY)
        except Exception:
          pass
        with self._lock:
          self._events = {}
          self._cells = {}
          self._dirty = False
        return

      buckets = int(data.get("dir_buckets", 8))
      if buckets != 8:
        try:
          self._params.remove(self.KEY)
        except Exception:
          pass
        with self._lock:
          self._events = {}
          self._cells = {}
          self._dirty = False
        return

      events_in = data.get("events", {})
      events: Dict[str, Dict] = {}
      if isinstance(events_in, dict):
        for eid, ev in events_in.items():
          if not isinstance(ev, dict):
            continue
          if "cy" not in ev or "cx" not in ev or "b" not in ev or "v" not in ev:
            continue
          events[str(eid)] = {
            "cy": float(ev["cy"]),
            "cx": float(ev["cx"]),
            "b": int(ev["b"]),
            "v": float(ev["v"]),
            "n": int(ev.get("n", 1)),
          }

      # 인덱스 rebuild(정합성 우선)
      rebuilt_cells: Dict[Tuple[int, int], List[str]] = {}
      for eid, ev in events.items():
        gy = int(round(float(ev["cy"])))
        gx = int(round(float(ev["cx"])))
        lst = rebuilt_cells.get((gy, gx))
        if lst is None:
          rebuilt_cells[(gy, gx)] = [eid]
        else:
          lst.append(eid)

      with self._lock:
        self._events = events
        self._cells = rebuilt_cells
        self._dirty = False

    except Exception:
      try:
        self._params.remove(self.KEY)
      except Exception:
        pass
      with self._lock:
        self._events = {}
        self._cells = {}
        self._dirty = False
