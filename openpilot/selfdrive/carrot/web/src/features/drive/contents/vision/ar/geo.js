/* 위경도 → route-local FLU 좌표. ARmap 식 "월드에 박힌" 마커의 근거.
 *
 * 왜 필요한가:
 *   지금까지 마커는 modelV2 경로 위에 얹혔다. 그 경로는 카메라가 실제로 보는
 *   도로라 정확하지만 190m 안팎에서 끊긴다. 그래서 320m 앞 단속 카메라는
 *   세울 자리가 아예 없어 사라졌다. 위경도로 놓으면 경로 길이와 무관해진다.
 *
 * 왜 TMap 의 차량 위치를 쓰는가(중요):
 *   기준점으로 GPS(gpsLocationExternal)를 쓰지 않고 carrotNavi.vehicle 을 쓴다.
 *   안내 지점과 차량 위치가 **같은 출처**여야 그 출처의 계통 오차가 뺄셈에서
 *   상쇄된다. 서로 다른 출처를 섞으면 두 오차가 더해져서 마커가 도로 옆으로
 *   밀린다. 우리가 필요한 것은 절대 위치가 아니라 상대 위치다.
 *
 * 고도에 대하여:
 *   TMap 안내 지점에는 고도가 없다. 그래서 z 는 노면(0)으로 둔다. 표지를
 *   띄우는 높이는 descriptor.mountHeightM 이 따로 담당한다. 고도를 아는 척하지 않는다.
 */

import { AR_COORDINATE_FRAME } from "./coordinate_frames.js";
import { UNKNOWN_ROAD_HEIGHT } from "./road_observation.js";

/** 위도 1도의 거리(m). 지구를 구로 본 근사. 수백 m 범위에서는 오차가 무시된다. */
const M_PER_DEG_LAT = 111320;

export const AR_GEO_LIMITS = Object.freeze({
  /** 이보다 먼 지점은 다루지 않는다. AR 로 의미가 없고 평면 근사도 나빠진다. */
  maxRangeM: 2000,
  /** 직선거리가 경로거리를 이만큼 넘으면 좌표를 의심한다(직선 ≤ 경로 이므로). */
  routeSlackRatio: 1.2,
  routeSlackM: 30,
  /** 안내점에서 이 거리 안의 route segment만 방향 근거로 쓴다. */
  maxTangentSnapM: 140,
  /** 차량을 route polyline에 붙일 때 허용하는 최대 시작 오차. 완화: 보이기 우선. */
  maxRouteStartSnapM: 160,
  /** 회전 지점에서 이 거리만큼 이후 경로를 보고 실제 진출 방향을 정한다. */
  maneuverLookAheadM: 12,
});

function finite(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

/**
 * 두 위경도 사이의 동/북 방향 거리(m).
 * 기준 위도에서의 접평면 근사. 경도 1도 거리는 위도에 따라 줄어든다.
 */
export function enuOffset(fromLat, fromLon, toLat, toLon) {
  const lat0 = finite(fromLat);
  const lon0 = finite(fromLon);
  const lat1 = finite(toLat);
  const lon1 = finite(toLon);
  if (lat0 === null || lon0 === null || lat1 === null || lon1 === null) return null;
  const north = (lat1 - lat0) * M_PER_DEG_LAT;
  const east = (lon1 - lon0) * M_PER_DEG_LAT * Math.cos((lat0 * Math.PI) / 180);
  return { east, north };
}

/**
 * 동/북 → route-local FLU(x 전방, y 좌측, z 상방).
 *
 * heading 은 진북 기준 시계방향 도(degree). 즉 자차 전방 단위벡터는
 * ENU 에서 (sin h, cos h) 이고, 좌측은 그것을 반시계로 90도 돌린 (-cos h, sin h).
 */
export function toVehicleFrame(east, north, headingDeg) {
  const e = finite(east);
  const n = finite(north);
  const h = finite(headingDeg);
  if (e === null || n === null || h === null) return null;
  const r = (h * Math.PI) / 180;
  const s = Math.sin(r);
  const c = Math.cos(r);
  return {
    x: e * s + n * c,        // 전방
    y: n * s - e * c,        // 좌측
    ...UNKNOWN_ROAD_HEIGHT,  // 고도 정보 없음 — unknown flat 노면으로 둔다
    coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
  };
}

/**
 * 안내 지점의 자차 기준 좌표.
 *
 * @param vehicle  carrotNavi.vehicle { present, latitude, longitude, headingDeg }
 * @param point    { pointValid, latitude, longitude }
 * @param opts.routeDistanceM 안내가 알려 준 경로거리(m). 있으면 타당성 검사에 쓴다.
 * @returns {x,y,z, rangeM} 또는 null
 */
export function geoAnchor(vehicle, point, opts = {}) {
  if (!vehicle || vehicle.present === false) return null;
  if (!point || point.pointValid === false) return null;

  const offset = enuOffset(vehicle.latitude, vehicle.longitude, point.latitude, point.longitude);
  if (!offset) return null;
  const heading = finite(vehicle.headingDeg);
  if (heading === null) return null;

  const local = toVehicleFrame(offset.east, offset.north, heading);
  if (!local) return null;

  const rangeM = Math.hypot(local.x, local.y);
  if (!(rangeM > 0) || rangeM > AR_GEO_LIMITS.maxRangeM) return null;

  /* 타당성: 직선거리는 경로거리를 넘을 수 없다(굽은 길이면 오히려 짧다).
   * 넘으면 좌표나 heading 이 잘못된 것이므로 쓰지 않는다. 조용히 어긋난
   * 마커를 그리느니 경로 기반 배치로 떨어지는 편이 낫다. */
  const routeDistanceM = finite(opts.routeDistanceM);
  if (routeDistanceM !== null && routeDistanceM > 0) {
    const ceiling = routeDistanceM * AR_GEO_LIMITS.routeSlackRatio + AR_GEO_LIMITS.routeSlackM;
    if (rangeM > ceiling) return null;
  }

  // 뒤쪽 지점은 그리지 않는다(이미 지났다).
  if (local.x <= 0) return null;

  return Object.freeze({ ...local, rangeM });
}

/**
 * 안내점에 가장 가까운 route.polyline 구간의 자차 상대 방향(rad).
 *
 * turnType의 좌/우 코드는 마커 모양을 고를 뿐 실제 도로 각도가 아니다. 게이트
 * 방향은 TMap 경로의 접선으로 정한다. 좌표가 없거나 안내점과 동떨어진 구간밖에
 * 없으면 null을 반환해 modelV2 접선 또는 정면(0rad)으로 안전하게 떨어진다.
 */
export function routeTangentHeading(vehicle, point, polyline, opts = {}) {
  if (!vehicle || vehicle.present === false || !point || point.pointValid === false) return null;
  if (!Array.isArray(polyline) || polyline.length < 2) return null;

  const maxSnapM = finite(opts.maxTangentSnapM) ?? AR_GEO_LIMITS.maxTangentSnapM;
  const lookAheadM = Math.max(0, finite(opts.lookAheadM) ?? 0);
  const localPoints = [];
  for (const routePoint of polyline) {
    const offset = enuOffset(point.latitude, point.longitude, routePoint?.latitude, routePoint?.longitude);
    const local = offset && toVehicleFrame(offset.east, offset.north, vehicle.headingDeg);
    if (local) localPoints.push(local);
  }
  if (localPoints.length < 2) return null;

  let best = null;
  for (let i = 1; i < localPoints.length; i += 1) {
    const a = localPoints[i - 1];
    const b = localPoints[i];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len2 = dx ** 2 + dy ** 2;
    if (!(len2 > 0.01)) continue;
    const t = Math.max(0, Math.min(1,
      -(a.x * dx + a.y * dy) / len2));
    const x = a.x + dx * t;
    const y = a.y + dy * t;
    const distanceM = Math.hypot(x, y);
    if (best && distanceM >= best.distanceM) continue;
    best = { distanceM, headingRad: Math.atan2(dy, dx), index: i, x, y };
  }

  if (!best || best.distanceM > maxSnapM) return null;
  if (!(lookAheadM > 0)) return best.headingRad;

  let remaining = lookAheadM;
  let cursor = { x: best.x, y: best.y };
  for (let index = best.index; index < localPoints.length; index += 1) {
    const end = localPoints[index];
    const dx = end.x - cursor.x;
    const dy = end.y - cursor.y;
    const length = Math.hypot(dx, dy);
    if (length > 0.01) {
      if (remaining <= length) {
        const ratio = remaining / length;
        const targetX = cursor.x + dx * ratio;
        const targetY = cursor.y + dy * ratio;
        return Math.atan2(targetY - best.y, targetX - best.x);
      }
      remaining -= length;
    }
    cursor = end;
  }
  return best.headingRad;
}

/**
 * TMap route polyline을 차량 위치부터 routeDistanceM만큼 걸어 원거리 앵커를 만든다.
 *
 * modelV2 path는 카메라 정합에 가장 좋지만 약 200m 안팎에서 끝난다. 정확한
 * 이벤트 좌표도 없는 SDI/다음 안내는 그보다 멀리서 놓을 수 없었으므로, 원거리
 * 구간에만 같은 TMap 좌표계의 route polyline을 사용한다. 가까워져 model path가
 * 닿으면 anchor_store가 비전 경로 쪽으로 연속 보정한다.
 */
/** route_matcher가 승인한 시작 세그먼트(정밀 힌트). 좌표계는 points와 동일. */
function routeMatchStart(suppliedMatch, pointCount) {
  if (!(suppliedMatch
    && suppliedMatch.routePointCount === pointCount
    && Number.isInteger(suppliedMatch.index)
    && suppliedMatch.index > 0
    && suppliedMatch.index < pointCount)) return null;
  return {
    index: suppliedMatch.index,
    t: suppliedMatch.t,
    x: suppliedMatch.x,
    y: suppliedMatch.y,
    snapDistanceM: suppliedMatch.snapDistanceM,
    alongTrackM: suppliedMatch.alongTrackM,
  };
}

/** 자체 최근접 세그먼트 검색. match가 없거나 그 start가 실패했을 때의 안전망. */
function nearestRouteSegmentStart(points) {
  let start = null;
  for (let index = 1; index < points.length; index += 1) {
    const a = points[index - 1];
    const b = points[index];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const length2 = dx * dx + dy * dy;
    if (!(length2 > 0.01)) continue;
    // 명백히 뒤로 가는 세그먼트만 건너뛴다. route_matcher가 heading/progress를
    // 승인하지 못한 프레임에서도 최소한 "앞쪽 최근접"으로는 표지를 세운다.
    if (dx < -Math.sqrt(length2) * 0.25) continue;
    const t = Math.max(0, Math.min(1, -(a.x * dx + a.y * dy) / length2));
    const x = a.x + dx * t;
    const y = a.y + dy * t;
    const snapDistanceM = Math.hypot(x, y);
    if (!start || snapDistanceM < start.snapDistanceM) {
      start = { index, t, x, y, snapDistanceM };
    }
  }
  return start;
}

/** start에서 routeDistanceM만큼 걸어 자차 앞의 앵커를 만든다. 실패 시 null. */
function walkRouteToAnchor(points, start, distanceM) {
  let remaining = distanceM;
  let segmentStart = { x: start.x, y: start.y };
  for (let index = start.index; index < points.length; index += 1) {
    const segmentEnd = points[index];
    const dx = segmentEnd.x - segmentStart.x;
    const dy = segmentEnd.y - segmentStart.y;
    const length = Math.hypot(dx, dy);
    if (length > 0.01) {
      if (remaining <= length) {
        const t = remaining / length;
        const x = segmentStart.x + dx * t;
        const y = segmentStart.y + dy * t;
        const rangeM = Math.hypot(x, y);
        if (!(x > 0) || rangeM > AR_GEO_LIMITS.maxRangeM) return null;
        return Object.freeze({
          x,
          y,
          ...UNKNOWN_ROAD_HEIGHT,
          rangeM,
          headingRad: Math.atan2(dy, dx),
          routeStartIndex: start.index,
          routeAlongTrackM: finite(start.alongTrackM),
          routeDerived: true,
          coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
        });
      }
      remaining -= length;
    }
    segmentStart = segmentEnd;
  }
  return null;
}

export function routeDistanceAnchor(vehicle, polyline, routeDistanceM, opts = {}) {
  if (!vehicle || vehicle.present === false) return null;
  if (!Array.isArray(polyline) || polyline.length < 2) return null;
  const distanceM = finite(routeDistanceM);
  if (distanceM === null || !(distanceM > 0) || distanceM > AR_GEO_LIMITS.maxRangeM) return null;

  const points = [];
  for (const point of polyline) {
    const offset = enuOffset(vehicle.latitude, vehicle.longitude, point?.latitude, point?.longitude);
    const local = offset && toVehicleFrame(offset.east, offset.north, vehicle.headingDeg);
    if (local) points.push(local);
  }
  if (points.length < 2) return null;

  const maxSnapM = finite(opts.maxRouteStartSnapM) ?? AR_GEO_LIMITS.maxRouteStartSnapM;

  // 일단 보이는 게 우선: matcher가 준 정밀 가지를 먼저 시도하고, 그 start로
  // 앵커를 못 만들면(짧은 가지를 걸어 벗어나거나 자차 뒤로 계산되면) 자체
  // 최근접 검색으로 재시도한다. route와 전방거리가 유효하면 표지를 지우지 않는다.
  const candidateStarts = [
    routeMatchStart(opts.routeMatch, points.length),
    nearestRouteSegmentStart(points),
  ];
  for (const start of candidateStarts) {
    if (!start || start.snapDistanceM > maxSnapM) continue;
    const anchor = walkRouteToAnchor(points, start, distanceM);
    if (anchor) return anchor;
  }
  return null;
}

/** 자차 뒤/옆이 아니라 "앞"인지. 렌더 전 빠른 판정용. */
export function isAhead(anchor) {
  return Boolean(anchor) && Number(anchor.x) > 0;
}
