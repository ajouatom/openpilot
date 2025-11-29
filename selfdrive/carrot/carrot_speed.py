# -*- coding: utf-8 -*-
"""
CarrotSpeedTable v2.1 (Params backend, JSON+gzip, 1e-4° grid, 8 buckets)
- 저장 키: "CarrotSpeedTable"
- 포맷(JSON): {"format":"v5","dir_buckets":8,"cells":{"gy,gx":[[v,ts],...]} }
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
    gy = int(math.floor(lat * 1e4 + 0.5))
    gx = int(math.floor(lon * 1e4 + 0.5))
    return gy, gx

def heading_to_bucket(heading_deg: float) -> int:
    # 8 버킷 고정
    step = 45.0  # 360/8
    i = int((heading_deg % 360.0) // step)
    if i < 0: return 0
    if i > 7: return 7
    return i

DIR_8 = {
    0: ( 1,  0),  # 북
    1: ( 1,  1),  # 북동
    2: ( 0,  1),  # 동
    3: (-1,  1),  # 남동
    4: (-1,  0),  # 남
    5: (-1, -1),  # 남서
    6: ( 0, -1),  # 서
    7: ( 1, -1),  # 북서
}

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

    def _try_cell_bucket_old(self, arr, b):
        v, ts = arr[b]
        if v is None or ts is None:
            return None, None
        if self._now() - int(ts) < self.neighbor_old_threshold_s:
            return None, None
        return float(v), b
    # ----- 공용 API -----
    def export_cells_around(self, lat: float, lon: float,
                            heading_deg: float,
                            ring: int = 1, max_points: int = 64) -> str:
        """
        현재 lat, lon 기준 주변 그리드(ring 범위)에서
        값이 있는 셀들을 (lat, lon, speed) 리스트로 JSON으로 반환.
        Params("CarrotSpeedViz")에 그대로 넣을 용도.
        """
        gy0, gx0 = quantize_1e4(lat, lon)
        b0 = heading_to_bucket(heading_deg)
        pts = []

        with self._lock:
            for dy in range(-ring, ring + 1):
                for dx in range(-ring, ring + 1):
                    gy = gy0 + dy
                    gx = gx0 + dx
                    arr = self._cells.get((gy, gx))
                    if not arr:
                        continue

                    # 먼저 exact bucket(b0)
                    v, ts = arr[b0]
                    if v is not None:
                        cell_lat = (gy + 0.5) * 1e-4
                        cell_lon = (gx + 0.5) * 1e-4
                        pts.append([cell_lat, cell_lon, float(v)])
                        if len(pts) >= max_points:
                            return json.dumps({"pts": pts}, separators=(",",":"))

                    # 없다면 좌/우
                    for b in ((b0 - 1) % self.buckets, (b0 + 1) % self.buckets):
                        v, ts = arr[b]
                        if v is None:
                            continue
                        cell_lat = (gy + 0.5) * 1e-4
                        cell_lon = (gx + 0.5) * 1e-4
                        pts.append([cell_lat, cell_lon, float(v)])
                        if len(pts) >= max_points:
                            return json.dumps({"pts": pts}, separators=(",",":"))

        return json.dumps({"pts": pts}, separators=(",",":"))

    def add_sample(self, lat: float, lon: float, heading_deg: float, speed_signed: float):
        """
        단일 speed(부호 포함) 저장.
        - 기준 셀(현재 위치) + heading 기준 좌/우 1셀, 2셀까지 동일 speed 기록
        - 각 셀 안에서는 heading 버킷 b와 b±1 세 개 버킷 모두 같은 값으로 갱신.
        - >0: 기존 음수/None도 교체, 기존 양수면 평균으로 완만하게 갱신.
        - <0: 항상 새 음수로 덮어쓰기(돌발 감속 우선).
        ==0: 무시
        """
        v_in = round(float(speed_signed), 1)
        if v_in == 0.0:
            return

        # 현재 위치를 그리드로
        gy0, gx0 = quantize_1e4(lat, lon)
        b = heading_to_bucket(heading_deg)
        now = self._now()

        # bucket에 해당하는 전진 방향 그리드 벡터
        dy_f, dx_f = DIR_8[b]

        # heading 기준 좌/우 1셀, 2셀 (project_point 사용 X)
        # 좌 = 전진벡터를 90° 회전 (dy,dx) -> (dx,-dy)
        # 우 = 전진벡터를 -90° 회전 (dy,dx) -> (-dx,dy)
        dy_l1, dx_l1 = dx_f, -dy_f
        dy_r1, dx_r1 = -dx_f, dy_f

        dy_l2, dx_l2 = 2 * dy_l1, 2 * dx_l1
        dy_r2, dx_r2 = 2 * dy_r1, 2 * dx_r1

        # 기록할 셀들: 중앙 + 좌/우 1칸 + 좌/우 2칸
        target_cells = {
            (gy0, gx0),
            (gy0 + dy_l1, gx0 + dx_l1),
            (gy0 + dy_r1, gx0 + dx_r1),
            (gy0 + dy_l2, gx0 + dx_l2),
            (gy0 + dy_r2, gx0 + dx_r2),
        }

        with self._lock:
            for gy, gx in target_cells:
                arr = self._ensure_cell(gy, gx)

                # b, b-1, b+1 세 버킷 모두 같은 정책으로 업데이트
                for off in (0, -1, +1):
                    bi = (b + off) % self.buckets
                    v_old, ts_old = arr[bi]

                    if v_old is None:
                        # 처음 쓰는 버킷
                        arr[bi] = [v_in, now]
                    else:
                        if v_in > 0.0:
                            # 가속 정보: 기존 양수면 평균, 음수면 교체
                            if v_old < 0.0:
                                # 음수 -> 양수로 바뀌면 새 양수로 교체 (ts는 기존 유지)
                                arr[bi] = [v_in, ts_old]
                            else:
                                new_val = round((v_old + v_in) / 2.0, 1)
                                arr[bi] = [new_val, ts_old]
                        else:
                            # 감속 정보: 항상 새 음수로 덮어쓰기, ts는 기존 유지
                            arr[bi] = [v_in, ts_old]

            self._dirty = True

       
    def query_target(self, lat: float, lon: float, heading_deg: float, v_ego: float,
                     lookahead_s: float = 2.0) -> float:
        dist = max(0.0, float(v_ego) * float(lookahead_s))
        return self.query_target_dist(lat, lon, heading_deg, dist)
    
    def query_target_dist(self, lat: float, lon: float, heading_deg: float, dist: float) -> float:
        b = heading_to_bucket(heading_deg)

        cand_ds = [dist]
        for off in (3.0, -3.0):
            d2 = dist + off
            if d2 >= 0.0:
                cand_ds.append(d2)

        with self._lock:
            for d in cand_ds:
                y, x = project_point(lat, lon, heading_deg, d)
                gy, gx = quantize_1e4(y, x)

                arr = self._cells.get((gy, gx))
                if not arr:
                    continue

                v, b_sel = self._try_cell_bucket_old(arr, b)
                if v is not None:
                    now_sec = int(time.time())
                    self._last_hit = (gy, gx, b_sel, now_sec)
                    self._last_hit_read_ms = int(time.time() * 1000)
                    return v

        return 0.0
    
    def invalidate_last_hit(self, window_s: float = 2.0, action: str = "clear") -> bool:
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

            # b, b-1, b+1 모두 invalidate
            for off in (0, -1, +1):
                bi = (b + off) % self.buckets
                v, ts = arr[bi]

                if action == "clear":
                    if v is not None and v < 0.0:
                      arr[bi] = [None, None]
                else:  # "age_bump"
                    if v is not None:
                        arr[bi] = [v, now]
                    else:
                        # 값이 없으면 넘어가기만 (그 버킷만 skip)
                        pass

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
            obj = {"format": "v5", "dir_buckets": self.buckets, "cells": cells}
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
            if data.get("format") != "v5":
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
