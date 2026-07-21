/* 한 프레임에 무엇을 어디에 그릴지 정하는 순수 계산.
 *
 * 여기 있는 것은 전부 부수효과가 없는 계산이다. 그래서 워커에서 돌릴 수 있고,
 * 실제로 워커에서 돈다(OffscreenCanvas 를 못 쓰는 환경에서만 메인이 같은
 * 함수를 부른다). 워커와 메인이 **같은 코드**를 쓰므로 두 경로가 갈라지지 않는다.
 *
 * 입력은 provider 스냅샷에서 뽑은 순수 데이터뿐이다. DOM 도 캔버스도 모른다.
 */

import { describeMarkers } from "./tmap_catalog.js";
import { signboardFromMarker } from "./signboard.js";
import { phaseForDistance } from "./tokens.js";
import { pointOnPath } from "./projection.js";
import {
  AR_GEO_LIMITS,
  geoAnchor,
  routeDistanceAnchor,
  routeTangentHeading,
} from "./geo.js";
import { markerIdentity, markerLifecycleSlot } from "./marker_identity.js";
import { roadFrameFields, roadFrameFromHeading } from "./road_frame.js";

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/** navi 스냅샷 → 그릴 표지 목록(앵커는 아직 없음). */
export function collectSigns(input = {}) {
  const navi = input.navi;
  const laneWidthM = finite(input.laneWidthM, 3.5);
  const egoSpeedMps = finite(input.egoSpeedMps, 0);

  /* 실제 Navi 마커가 우선이다.
   * calibrationProbe 는 "Navi 가 없을 때 대신 세우는 검증용 표지"이지
   * "항상 검증용만 그린다"가 아니다. */
  const out = [];
  if (navi && input.naviUsable !== false) {
    for (const marker of describeMarkers(navi, { laneWidthM })) {
      const distanceM = finite(marker.distanceM, 0);
      const descriptor = signboardFromMarker({
        ...marker,
        phase: phaseForDistance(distanceM, egoSpeedMps),
      });
      if (descriptor) {
        const identityInput = { ...marker, descriptor };
        const markerId = markerIdentity(identityInput);
        out.push({
          descriptor,
          distanceM,
          point: marker.point || null,
          source: marker.source || "unknown",
          markerId,
          // Compatibility alias for existing diagnostics and fixtures.
          eventKey: markerId,
          lifecycleSlot: markerLifecycleSlot(identityInput),
        });
      }
    }
  }
  if (out.length) return out;

  /* 활성 안내가 off-route/미연결 상태이면 즉시 숨긴다. 이때 probe로 떨어지면
   * 실제 안내처럼 오해할 수 있다. 반대로 경로 안내 자체가 비활성인 상태는
   * 사용자가 명시적으로 AR을 켠 현장 점검에서만 probe를 허용한다. */
  const navigationStatus = navi?.navigationStatus;
  if (
    navi
    && input.naviUsable === false
    && (navigationStatus?.guidanceActive === true || navigationStatus?.offRoute === true)
  ) return [];

  if (!input.calibrationProbe) return [];
  const distanceM = finite(input.probeDistanceM, 40);
  const descriptor = signboardFromMarker({
    kind: "turn_gate", distanceM,
    turn: { direction: "straight", sign: 0 },
    label: "AR 표시 확인", phase: phaseForDistance(distanceM, egoSpeedMps),
  });
  if (!descriptor) return [];
  const identityInput = { descriptor, source: "calibrationProbe" };
  const markerId = markerIdentity(identityInput);
  return [{
    descriptor,
    distanceM,
    source: "calibrationProbe",
    markerId,
    eventKey: markerId,
    lifecycleSlot: markerLifecycleSlot(identityInput),
  }];
}

/**
 * 표지에 앵커(자차 기준 좌표)를 붙인다.
 *
 * 배치는 두 갈래다.
 *   1순위 위경도(geo). 경로 길이와 무관하므로 190m 를 넘는 안내도 세울 수 있다.
 *         320m 앞 단속 카메라가 사라지던 원인이 여기 있었다.
 *   2순위 modelV2 경로. 카메라가 실제로 보는 도로라 근거리에서 더 정확하다.
 *
 * geo 는 좌표가 경로거리와 모순되면 스스로 null 을 준다(geo.js 의 검사).
 * 그때는 조용히 경로 배치로 떨어진다.
 */
export function anchorSigns(signs, {
  vehicle,
  routePolyline,
  modelPosition,
  geoAllowed = false,
} = {}) {
  const out = [];
  for (const item of Array.isArray(signs) ? signs : []) {
    const kind = item.descriptor?.kind;
    const maneuver = kind === "turn_gate" || kind === "commit_arrow" || kind === "destination_pin";
    const routeSemantic = maneuver
      || item.source === "guidanceCurrent"
      || item.source === "guidanceNext"
      || item.source === "crossroad";
    let viaGeo = geoAllowed && item.point
      ? geoAnchor(vehicle, item.point, { routeDistanceM: item.distanceM })
      : null;
    if (viaGeo) {
      const headingRad = routeTangentHeading(vehicle, item.point, routePolyline, {
        lookAheadM: maneuver ? AR_GEO_LIMITS.maneuverLookAheadM : 0,
      });
      const roadFrame = roadFrameFromHeading(headingRad ?? 0);
      viaGeo = Object.freeze({ ...viaGeo, ...roadFrameFields(roadFrame) });
    }
    let viaRoute = null;
    // 회전/분기 의미가 있는 안내는 modelV2가 가까이 있더라도 Navi route를 먼저
    // 쓴다. model path는 카메라가 보는 현재 차로라서 좌·우회전 진출 가지를 반드시
    // 포함하지 않는다. anchor_store가 이후 model 높이/노면을 연속 보정한다.
    if (!viaGeo && geoAllowed && routeSemantic) {
      viaRoute = routeDistanceAnchor(vehicle, routePolyline, item.distanceM);
      if (viaRoute) {
        let headingRad = viaRoute.headingRad ?? 0;
        if (maneuver) {
          const ahead = routeDistanceAnchor(
            vehicle,
            routePolyline,
            item.distanceM + AR_GEO_LIMITS.maneuverLookAheadM,
          );
          if (ahead && Math.hypot(ahead.x - viaRoute.x, ahead.y - viaRoute.y) > 0.5) {
            headingRad = Math.atan2(ahead.y - viaRoute.y, ahead.x - viaRoute.x);
          }
        }
        const roadFrame = roadFrameFromHeading(headingRad);
        viaRoute = Object.freeze({ ...viaRoute, headingRad, ...roadFrameFields(roadFrame) });
      }
    }
    const viaPath = viaGeo || viaRoute ? null : pointOnPath(modelPosition, item.distanceM);
    if (!viaGeo && !viaRoute && !viaPath && geoAllowed) {
      viaRoute = routeDistanceAnchor(vehicle, routePolyline, item.distanceM);
      if (viaRoute) {
        const roadFrame = roadFrameFromHeading(viaRoute.headingRad ?? 0);
        viaRoute = Object.freeze({ ...viaRoute, ...roadFrameFields(roadFrame) });
      }
    }
    const anchor = viaGeo || viaPath || viaRoute;
    if (anchor) out.push({
      ...item,
      anchor,
      placedBy: viaGeo || viaRoute ? "geo" : "path",
    });
  }
  return out;
}

/** collectSigns + anchorSigns 한 번에. 워커/메인이 공통으로 부른다. */
export function composeFrame(input = {}) {
  const signs = collectSigns(input);
  const anchored = input.canDrawPrecise
    ? anchorSigns(signs, {
      vehicle: input.navi?.vehicle,
      routePolyline: input.navi?.route?.polyline,
      modelPosition: input.modelPosition,
      geoAllowed: input.geoAllowed === true,
    })
    : null;
  return { signs, fresh: anchored && anchored.length ? anchored : null };
}
