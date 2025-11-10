# -*- coding: utf-8 -*-
"""
CarrotSpeedTable v2.1 (Params backend, JSON+gzip, 1e-4° grid, 8 buckets)
- 저장 키: "CarrotSpeedTable"
- 포맷(JSON): {"format":"v2","dir_buckets":8,"cells":{"gy,gx":[[v,ts],...]} }
- gzip 저장/로드 지원 (기본 on). 기존 비압축 v2도 로드 가능.
- 격자: 위/경도 각 1e-4° 스냅(한국 위도에서 약 9~11m)
- 저장: 단일 speed(부호 포함)만 해당 셀 1곳에 기록
    * 입력 > 0: 기존 None/음수/더 작은 양수면 갱신(더 큰 +)
    * 입력 < 0: 기존 None/양수/덜 음수면 갱신(더 작은 -)
- 조회: 전방 lookahead 셀 → 없으면 이웃 탐색(ring=1)
    * 본셀: 시간 필터 없음
    * 이웃: 오래된 데이터만 사용(age ≥ 120s)
- 정리(청소) 없음: 오래된 데이터도 유지
"""

import json, math, threading, time, gzip
from typing import Optional, Tuple, Dict, List
from openpilot.common.params import Params


# ---------- 지오/도우미 ----------

def quantize_1e4(lat: float, lon: float) -> Tuple[int, int]:
    gy = int(math.floor(lat * 1e4))
    gx = int(math.floor(lon * 1e4))
    return gy, gx

def heading_to_bucket(heading_deg: float) -> int:
    # 8 버킷 고정
    step = 45.0  # 360/8
    i = int((heading_deg % 360.0) // step)
    if i < 0: return 0
    if i > 7: return 7
    return i

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


# ---------- 메인 클래스 ----------

class CarrotSpeed:
    KEY = "CarrotSpeedTable"

    def __init__(self,
                 neighbor_ring: int = 1,
                 neighbor_old_threshold_s: int = 120,
                 use_gzip: bool = True,
                 gzip_level: int = 5):
        # 고정 사양
        self.buckets = 8

        # 파라미터
        self.neighbor_ring = max(0, int(neighbor_ring))
        self.neighbor_old_threshold_s = int(neighbor_old_threshold_s)
        self.use_gzip = bool(use_gzip)
        self.gzip_level = int(gzip_level)

        # 내부 상태
        self._lock = threading.RLock()
        # _cells[(gy,gx)] = [[value or None, ts(int seconds) or None] * 8]
        self._cells: Dict[Tuple[int, int], List[List[Optional[float]]]] = {}
        self._dirty = False
        self._last_save = 0
        self._params = Params()

        self._load_from_params_if_exists()

        self._last_hit = None        # (gy, gx, b, ts_when_read)
        self._last_hit_read_ms = 0   # 밀리초

    # ----- 내부 유틸 -----

    def _ensure_cell(self, gy: int, gx: int) -> List[List[Optional[float]]]:
        arr = self._cells.get((gy, gx))
        if arr is None:
            arr = [[None, None] for _ in range(self.buckets)]  # [v, ts]
            self._cells[(gy, gx)] = arr
        return arr

    def _now(self) -> int:
        # int 초
        return int(time.time())

    def _age(self, ts: Optional[float]) -> Optional[int]:
        if ts is None:
            return None
        return self._now() - int(ts)

    def _neighbor_indices(self, gy: int, gx: int) -> List[Tuple[int, int]]:
        r = self.neighbor_ring
        if r <= 0:
            return []
        out = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dy == 0 and dx == 0:
                    continue
                out.append((gy + dy, gx + dx))
        return out

    def _neighbors_8(self, gy, gx):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                yield gy + dy, gx + dx

    def _try_cell_buckets_old(self, arr, b):
        for off in (0, -1, +1):
            bi = (b + off) % self.buckets
            v, ts = arr[bi]
            if v is None or ts is None:
                continue
            if self._now() - int(ts) < self.neighbor_old_threshold_s:
                continue
            return float(v), bi
        return None, None
    # ----- 공용 API -----

    def add_sample(self, lat: float, lon: float, heading_deg: float, speed_signed: float):
        """
        단일 speed(부호 포함) 저장. 한 셀만 기록.
        >0: 더 큰 양수로 갱신(기존 음수/None도 교체)
        <0: 더 작은 음수로 갱신(기존 양수/None도 교체)
        ==0: 무시
        """
        v_in = round(float(speed_signed), 1)
        if v_in == 0.0:
            return

        gy, gx = quantize_1e4(lat, lon)
        b = heading_to_bucket(heading_deg)
        now = self._now()

        with self._lock:
            arr = self._ensure_cell(gy, gx)
            v_old, ts_old = arr[b]
            
            if v_old is None:
                arr[b] = [v_in, now]
                self._dirty = True
            else:                
                new_val = round((v_old + v_in) / 2.0, 1)
                if v_in > 0.0:
                    if v_old < 0.0:
                        arr[b] = [v_in, ts_old]
                    else:
                        arr[b] = [new_val, ts_old]
                else:  # v_in < 0
                  arr[b] = [v_in, ts_old]  
                  #if v_old > 0.0: # 기존에 양수이면 돌발상황 감속이므로 저장안함.
                  #  pass  
                  #else:
                  #  arr[b] = [new_val, now]

            self._dirty = True
        
    def query_target(self, lat: float, lon: float, heading_deg: float, v_ego: float,
                     lookahead_s: float = 2.0, neighbor_fallback: bool = True) -> float:
        dist = max(0.0, float(v_ego) * float(lookahead_s))
        return self.query_target_dist(lat, lon, heading_deg, dist, neighbor_fallback)
    
    def query_target_dist(self, lat: float, lon: float, heading_deg: float, dist: float,
                          neighbor_fallback: bool = True) -> float:
        """
        간단 전략:
          - 거리 후보: [dist, dist+3, max(0, dist-3), dist+6, max(0, dist-6)]
          - 각 지점에서:   해당 셀의 b → b±1 (old-only)
          - 그래도 없으면: 그 셀의 8-이웃에서 b → b±1 (old-only)
          - 전부 실패하면 0.0
        """
        b = heading_to_bucket(heading_deg)

        cand_ds = [dist]
        for off in (3.0, -3.0, 6.0, -6.0):
            d2 = dist + off
            if d2 >= 0.0:
                cand_ds.append(d2)

        with self._lock:
            for d in cand_ds:
                y, x = project_point(lat, lon, heading_deg, d)
                gy, gx = quantize_1e4(y, x)

                # 1) 해당 셀: b → b±1
                arr = self._cells.get((gy, gx))
                if arr:
                    v, b_sel = self._try_cell_buckets_old(arr, b)
                    if v is not None:
                        now_sec = int(time.time())
                        self._last_hit = (gy, gx, b_sel, now_sec)
                        self._last_hit_read_ms = int(time.time() * 1000)  
                        return v

                if not neighbor_fallback:
                    continue

                # 2) 같은 지점의 8-이웃: b → b±1
                for ny, nx in self._neighbors_8(gy, gx):
                    arr2 = self._cells.get((ny, nx))
                    if not arr2:
                        continue
                    v2, b_sel2 = self._try_cell_buckets_old(arr2, b)
                    if v2 is not None:
                        now_sec = int(time.time())
                        self._last_hit = (ny, nx, b_sel2, now_sec)
                        self._last_hit_read_ms = int(time.time() * 1000)  
                        return v2

        return 0.0

    def invalidate_last_hit(self, window_s: float = 2.0, action: str = "clear") -> bool:
        """
        최근 query에서 읽은 셀/버킷을 window_s 이내면 무효화.
        action:
          - "clear": 아예 None 처리
          - "age_bump": ts를 지금 시각으로 덮어써서 120s 동안 old-only에 걸리게 함
        반환: 무효화 여부
        """
        if self._last_hit is None:
            return False
        gy, gx, b, read_ts = self._last_hit
        now = int(time.time())
        if (now - int(read_ts)) > window_s:
            return False

        with self._lock:
            arr = self._cells.get((gy, gx))
            if not arr:
                return False
            v, ts = arr[b]
            if action == "clear":
                arr[b] = [None, None]
            else:  # "age_bump"
                # 값은 유지하되 방금 값으로 만들어서 old-only 필터에 120s 동안 걸리게
                if v is not None:
                    arr[b] = [v, now]
                else:
                    # 값 자체가 없으면 할 게 없음
                    return False
            self._dirty = True
        return True

    def maybe_save(self, interval_s: int = 60) -> None:
        now = self._now()
        if (not self._dirty) or (now - self._last_save < interval_s):
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

    # ----- 직렬화 -----

    def _encode_payload(self) -> bytes:
        with self._lock:
            cells = {}
            for (gy, gx), arr in self._cells.items():
                key = f"{gy},{gx}"
                # arr: [[v, ts], ...]  (ts는 int 또는 None)
                cells[key] = [[None if v is None else float(v),
                               None if ts is None else int(ts)] for (v, ts) in arr]
            obj = {"format": "v4", "dir_buckets": self.buckets, "cells": cells}
            raw = json.dumps(obj, separators=(",", ":")).encode("utf-8")
            if self.use_gzip:
                return gzip.compress(raw, compresslevel=self.gzip_level)
            return raw

    def _load_from_params_if_exists(self) -> None:
        raw = self._params.get(self.KEY)
        if not raw:
            return
        try:
            data_bytes = raw
            if _is_gzip(data_bytes):
                data_bytes = gzip.decompress(data_bytes)
            data = json.loads(data_bytes.decode("utf-8"))

            # v3 아니면 삭제/초기화
            if data.get("format") != "v4":
                self._params.remove(self.KEY)
                with self._lock:
                    self._cells = {}
                    self._dirty = False
                return

            buckets = int(data.get("dir_buckets", 8))
            if buckets != 8:
                # 버킷 불일치도 삭제/초기화
                self._params.remove(self.KEY)
                with self._lock:
                    self._cells = {}
                    self._dirty = False
                return

            restored: Dict[Tuple[int, int], List[List[Optional[float]]]] = {}
            for key, arr in data.get("cells", {}).items():
                gy, gx = map(int, key.split(","))
                fixed: List[List[Optional[float]]] = []
                if isinstance(arr, list) and len(arr) == 8:
                    for pair in arr:
                        if isinstance(pair, list) and len(pair) == 2:
                            v, ts = pair
                            v2 = None if v is None else float(v)
                            # ts는 int로 강제
                            ts2 = None if ts is None else int(ts)
                            fixed.append([v2, ts2])
                        else:
                            fixed.append([None, None])
                else:
                    fixed = [[None, None] for _ in range(8)]
                restored[(gy, gx)] = fixed

            with self._lock:
                self._cells = restored
                self._dirty = False

        except Exception:
            # 파싱 실패 시 안전 초기화
            self._params.delete(self.KEY)
            with self._lock:
                self._cells = {}
                self._dirty = False
